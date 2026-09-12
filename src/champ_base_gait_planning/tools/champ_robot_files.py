#!/usr/bin/env python3
"""Robot-agnostic helpers shared by the launch files and the offline tools.

A robot is described only by files named after it inside this package:

    urdf/<robot>.urdf | urdf/<robot>.xacro   kinematics, inertials, joint limits
    config/<robot>_gait.yaml                 CHAMP gait parameters
    config/<robot>_joints.yaml               CHAMP joints_map
    config/<robot>_links.yaml                CHAMP links_map (+ base, imu)
    config/<robot>_sim.yaml                  optional MuJoCo tuning (sim.*)
    config/<robot>_lowcmd.yaml               optional unitree_dds_bridge gains
    mujoco/<robot>.xml                       optional MuJoCo model
    rviz/<robot>_gait.rviz                   optional RViz layout

Nothing in here knows a particular robot; every value is read from those files.

    python3 champ_robot_files.py list [package_dir]
    python3 champ_robot_files.py expand <urdf_or_xacro> <output_urdf>
"""

from __future__ import annotations

import math
import os
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence

import numpy as np

LEGS = ('left_front', 'right_front', 'left_hind', 'right_hind')
LEG_SHORT = {'left_front': 'LF', 'right_front': 'RF', 'left_hind': 'LH', 'right_hind': 'RH'}
JOINTS_PER_LEG = 3
CHAIN_LENGTH = 4  # hip, upper leg, lower leg, foot
PACKAGE_NAME = 'champ_base_gait_planning'
SOURCE_PACKAGE_DIR = Path(__file__).resolve().parents[1]


# ---------------------------------------------------------------------------
# File resolution
# ---------------------------------------------------------------------------

@dataclass
class RobotFiles:
    robot: str
    package_dir: Path
    urdf: Path
    gait_yaml: Path
    joints_yaml: Path
    links_yaml: Path
    sim_yaml: Optional[Path]
    lowcmd_yaml: Optional[Path]
    mujoco_xml: Optional[Path]
    rviz_config: Optional[Path]

    @property
    def is_xacro(self) -> bool:
        return self.urdf.suffix == '.xacro'

    @property
    def urdf_command(self) -> str:
        """Shell command that prints the URDF (launch `Command` substitution)."""
        return 'xacro' if self.is_xacro else 'cat'

    def champ_yamls(self) -> List[Path]:
        return [self.gait_yaml, self.joints_yaml, self.links_yaml]

    def sim_params(self) -> Dict[str, Any]:
        params = load_ros_params(self.sim_yaml) if self.sim_yaml else {}
        sim = params.get('sim', {})
        return sim if isinstance(sim, dict) else {}


def available_robots(package_dir: Path | str = SOURCE_PACKAGE_DIR) -> List[str]:
    urdf_dir = Path(package_dir) / 'urdf'
    names = set()
    for path in urdf_dir.glob('*'):
        if path.suffix in ('.urdf', '.xacro'):
            names.add(path.stem)
    return sorted(names)


def resolve_robot_files(package_dir: Path | str, robot: str) -> RobotFiles:
    package_dir = Path(package_dir)
    robot = robot.strip()
    robots = available_robots(package_dir)
    if not robot or robot not in robots:
        raise FileNotFoundError(
            f"unknown robot '{robot}'. Available robots (urdf/<robot>.urdf|.xacro): "
            f"{', '.join(robots) or 'none'}"
        )
    urdf = package_dir / 'urdf' / f'{robot}.urdf'
    if not urdf.exists():
        urdf = package_dir / 'urdf' / f'{robot}.xacro'

    def required(rel: str) -> Path:
        path = package_dir / rel
        if not path.exists():
            raise FileNotFoundError(f"robot '{robot}' is missing {rel}")
        return path

    def optional(rel: str) -> Optional[Path]:
        path = package_dir / rel
        return path if path.exists() else None

    return RobotFiles(
        robot=robot,
        package_dir=package_dir,
        urdf=urdf,
        gait_yaml=required(f'config/{robot}_gait.yaml'),
        joints_yaml=required(f'config/{robot}_joints.yaml'),
        links_yaml=required(f'config/{robot}_links.yaml'),
        sim_yaml=optional(f'config/{robot}_sim.yaml'),
        lowcmd_yaml=optional(f'config/{robot}_lowcmd.yaml'),
        mujoco_xml=optional(f'mujoco/{robot}.xml'),
        rviz_config=optional(f'rviz/{robot}_gait.rviz'),
    )


# ---------------------------------------------------------------------------
# ROS 2 parameter yaml
# ---------------------------------------------------------------------------

def _merge(dst: Dict[str, Any], src: Dict[str, Any]) -> None:
    for key, value in src.items():
        if isinstance(value, dict) and isinstance(dst.get(key), dict):
            _merge(dst[key], value)
        else:
            dst[key] = value


def load_ros_params(*yaml_paths: Optional[Path | str]) -> Dict[str, Any]:
    """Merge the ros__parameters blocks of ROS 2 parameter files (any node key)."""
    import yaml

    merged: Dict[str, Any] = {}
    for path in yaml_paths:
        if path is None:
            continue
        with open(path, 'r', encoding='utf-8') as handle:
            document = yaml.safe_load(handle) or {}
        found = False
        for node_key, node_value in document.items():
            if isinstance(node_value, dict) and 'ros__parameters' in node_value:
                _merge(merged, node_value['ros__parameters'] or {})
                found = True
        if not found:
            raise ValueError(f'{path}: no ros__parameters block found')
    return merged


def champ_joint_names(params: Dict[str, Any]) -> List[str]:
    """Actuated joints in CHAMP order LF, RF, LH, RH x (hip, upper, lower)."""
    joints_map = params.get('joints_map')
    if not isinstance(joints_map, dict):
        raise ValueError('joints_map missing from the joints yaml')
    names: List[str] = []
    for leg in LEGS:
        chain = joints_map.get(leg)
        if not chain or len(chain) < JOINTS_PER_LEG:
            raise ValueError(f'joints_map.{leg} needs hip, upper and lower joint names')
        names.extend(str(n) for n in chain[:JOINTS_PER_LEG])
    return names


def leg_chains(params: Dict[str, Any], key: str) -> Dict[str, List[str]]:
    """links_map / joints_map legs as {leg: [hip, upper, lower, foot]}."""
    mapping = params.get(key)
    if not isinstance(mapping, dict):
        raise ValueError(f'{key} missing from yaml')
    chains: Dict[str, List[str]] = {}
    for leg in LEGS:
        chain = mapping.get(leg)
        if not chain or len(chain) < CHAIN_LENGTH:
            raise ValueError(f'{key}.{leg} needs {CHAIN_LENGTH} names (hip, upper, lower, foot)')
        chains[leg] = [str(n) for n in chain[:CHAIN_LENGTH]]
    return chains


def foot_geom_names(params: Dict[str, Any], sim: Dict[str, Any]) -> List[str]:
    override = sim.get('foot_geom_names')
    if override:
        return [str(n) for n in override]
    links = leg_chains(params, 'links_map')
    return [f'{links[leg][CHAIN_LENGTH - 1]}_collision' for leg in LEGS]


def foot_site_names(params: Dict[str, Any], sim: Dict[str, Any]) -> List[str]:
    override = sim.get('foot_site_names')
    if override:
        return [str(n) for n in override]
    links = leg_chains(params, 'links_map')
    return [f'{links[leg][CHAIN_LENGTH - 1]}_site' for leg in LEGS]


# ---------------------------------------------------------------------------
# URDF (with a minimal xacro fallback)
# ---------------------------------------------------------------------------

def _package_share_dir(package: str) -> str:
    # generate_mjcf / champ_mesh_assets run against the source tree; prefer that
    # so $(find) and package:// do not pick a stale colcon install of this package.
    if package == PACKAGE_NAME and (SOURCE_PACKAGE_DIR / 'urdf').is_dir():
        return str(SOURCE_PACKAGE_DIR)
    try:
        from ament_index_python.packages import get_package_share_directory

        return get_package_share_directory(package)
    except Exception:
        return package


def expand_urdf_text(path: Path | str) -> str:
    """Return plain URDF XML for a .urdf or .xacro file.

    Files that use xacro macros/properties need the `xacro` executable. Files
    that only use `${expr}` with pi and `$(find pkg)` are expanded here so the
    offline checks work without a sourced ROS environment.
    """
    path = Path(path)
    text = path.read_text(encoding='utf-8')
    if path.suffix != '.xacro':
        return text
    if re.search(r'<xacro:', text):
        exe = shutil.which('xacro')
        if exe is None:
            raise RuntimeError(f'{path} uses xacro macros; install/source xacro to expand it')
        return subprocess.check_output([exe, str(path)], text=True)

    namespace = {'pi': math.pi, 'math': math}

    def eval_expr(match: re.Match) -> str:
        return repr(float(eval(match.group(1), {'__builtins__': {}}, namespace)))

    def find_pkg(match: re.Match) -> str:
        return _package_share_dir(match.group(1))

    text = re.sub(r'\$\(find\s+([A-Za-z0-9_]+)\)', find_pkg, text)
    text = re.sub(r'\$\{([^}]+)\}', eval_expr, text)
    text = re.sub(r'\s+xmlns:xacro="[^"]*"', '', text)
    return text


def _vec(text: Optional[str], default: str = '0 0 0') -> np.ndarray:
    return np.array([float(v) for v in (text or default).split()], dtype=float)


@dataclass
class UrdfJoint:
    name: str
    type: str
    parent: str
    child: str
    xyz: np.ndarray
    rpy: np.ndarray
    axis: np.ndarray
    limit: Dict[str, str]
    xyz_text: str
    rpy_text: str
    axis_text: str


@dataclass
class UrdfCollision:
    geometry: str  # box | cylinder | sphere | mesh
    attrib: Dict[str, str]
    xyz: np.ndarray
    rpy: np.ndarray
    xyz_text: str
    rpy_text: str


@dataclass
class UrdfVisual:
    geometry: str  # box | cylinder | sphere | mesh
    attrib: Dict[str, str]  # mesh: filename [, scale]
    xyz_text: str
    rpy_text: str
    materials: Dict[str, str]  # material name -> "r g b a" (every <material> of the visual)

    @property
    def rgba(self) -> Optional[str]:
        """Colour of the first material with a <color>, if any."""
        return next(iter(self.materials.values()), None)


@dataclass
class UrdfLink:
    name: str
    mass: Optional[float]
    mass_text: str
    inertial_xyz_text: str
    inertial_rpy_text: str
    inertia: Dict[str, str]
    collisions: List[UrdfCollision]
    visuals: List[UrdfVisual] = field(default_factory=list)


@dataclass
class Urdf:
    name: str
    root: str
    links: Dict[str, UrdfLink]
    joints: Dict[str, UrdfJoint]
    parent_joint: Dict[str, UrdfJoint]  # child link -> joint
    children: Dict[str, List[UrdfJoint]]  # parent link -> joints

    def chain_xyz(self, ref_link: str, end_link: str) -> np.ndarray:
        """champ::URDF::getPose: sum of joint origin xyz from end_link up to ref_link."""
        total = np.zeros(3)
        current = end_link
        while current != ref_link:
            joint = self.parent_joint.get(current)
            if joint is None:
                raise ValueError(f'link {end_link!r} is not below {ref_link!r} in the URDF')
            total = total + joint.xyz
            current = joint.parent
        return total

    def revolute_joint_names(self) -> List[str]:
        return [j.name for j in self.joints.values() if j.type in ('revolute', 'continuous')]


def parse_urdf_text(text: str) -> Urdf:
    import xml.etree.ElementTree as ET

    root = ET.fromstring(text)
    # Top-level <material name><color rgba/> definitions referenced by name from visuals.
    global_materials: Dict[str, str] = {}
    for material in root.findall('material'):
        colour = material.find('color')
        if colour is not None and 'rgba' in colour.attrib and 'name' in material.attrib:
            global_materials[material.attrib['name']] = colour.attrib['rgba']
    links: Dict[str, UrdfLink] = {}
    for link in root.findall('link'):
        inertial = link.find('inertial')
        mass = None
        mass_text = '0'
        inertial_xyz = '0 0 0'
        inertial_rpy = '0 0 0'
        inertia: Dict[str, str] = {}
        if inertial is not None:
            mass_el = inertial.find('mass')
            if mass_el is not None:
                mass_text = mass_el.attrib['value']
                mass = float(mass_text)
            origin = inertial.find('origin')
            if origin is not None:
                inertial_xyz = origin.attrib.get('xyz', '0 0 0')
                inertial_rpy = origin.attrib.get('rpy', '0 0 0')
            inertia_el = inertial.find('inertia')
            if inertia_el is not None:
                inertia = dict(inertia_el.attrib)
        collisions: List[UrdfCollision] = []
        for collision in link.findall('collision'):
            geometry = collision.find('geometry')
            if geometry is None or len(geometry) == 0:
                continue
            shape = geometry[0]
            origin = collision.find('origin')
            xyz_text = origin.attrib.get('xyz', '0 0 0') if origin is not None else '0 0 0'
            rpy_text = origin.attrib.get('rpy', '0 0 0') if origin is not None else '0 0 0'
            collisions.append(UrdfCollision(
                geometry=shape.tag,
                attrib=dict(shape.attrib),
                xyz=_vec(xyz_text),
                rpy=_vec(rpy_text),
                xyz_text=xyz_text,
                rpy_text=rpy_text,
            ))
        visuals: List[UrdfVisual] = []
        for visual in link.findall('visual'):
            geometry = visual.find('geometry')
            if geometry is None or len(geometry) == 0:
                continue
            shape = geometry[0]
            origin = visual.find('origin')
            materials: Dict[str, str] = {}
            # The Unitree URDFs list one <material> per COLLADA effect (non-standard
            # but harmless); keep them all so mesh parts can be coloured by name.
            for material in visual.findall('material'):
                colour = material.find('color')
                if colour is not None and 'rgba' in colour.attrib:
                    materials[material.attrib.get('name', '')] = colour.attrib['rgba']
                elif material.attrib.get('name') in global_materials:
                    materials[material.attrib['name']] = global_materials[material.attrib['name']]
            visuals.append(UrdfVisual(
                geometry=shape.tag,
                attrib=dict(shape.attrib),
                xyz_text=origin.attrib.get('xyz', '0 0 0') if origin is not None else '0 0 0',
                rpy_text=origin.attrib.get('rpy', '0 0 0') if origin is not None else '0 0 0',
                materials=materials,
            ))
        links[link.attrib['name']] = UrdfLink(
            name=link.attrib['name'],
            mass=mass,
            mass_text=mass_text,
            inertial_xyz_text=inertial_xyz,
            inertial_rpy_text=inertial_rpy,
            inertia=inertia,
            collisions=collisions,
            visuals=visuals,
        )

    joints: Dict[str, UrdfJoint] = {}
    parent_joint: Dict[str, UrdfJoint] = {}
    children: Dict[str, List[UrdfJoint]] = {}
    for joint in root.findall('joint'):
        origin = joint.find('origin')
        axis = joint.find('axis')
        limit = joint.find('limit')
        xyz_text = origin.attrib.get('xyz', '0 0 0') if origin is not None else '0 0 0'
        rpy_text = origin.attrib.get('rpy', '0 0 0') if origin is not None else '0 0 0'
        axis_text = axis.attrib.get('xyz', '1 0 0') if axis is not None else '1 0 0'
        item = UrdfJoint(
            name=joint.attrib['name'],
            type=joint.attrib['type'],
            parent=joint.find('parent').attrib['link'],
            child=joint.find('child').attrib['link'],
            xyz=_vec(xyz_text),
            rpy=_vec(rpy_text),
            axis=_vec(axis_text, '1 0 0'),
            limit=dict(limit.attrib) if limit is not None else {},
            xyz_text=xyz_text,
            rpy_text=rpy_text,
            axis_text=axis_text,
        )
        joints[item.name] = item
        parent_joint[item.child] = item
        children.setdefault(item.parent, []).append(item)

    roots = [name for name in links if name not in parent_joint]
    if len(roots) != 1:
        raise ValueError(f'URDF must have exactly one root link, found {roots}')
    return Urdf(
        name=root.attrib.get('name', ''),
        root=roots[0],
        links=links,
        joints=joints,
        parent_joint=parent_joint,
        children=children,
    )


def load_urdf(path: Path | str) -> Urdf:
    return parse_urdf_text(expand_urdf_text(path))


def champ_leg_translations(urdf: Urdf, links: Dict[str, List[str]]) -> Dict[str, np.ndarray]:
    """champ::URDF::fillLeg: translation of hip, upper, lower, foot joints per leg.

    Joint i is measured from links[i-1] (the URDF root for the hip) to links[i],
    summing origin xyz only (rpy is ignored by CHAMP, hence the rpy=0 contract).
    """
    out: Dict[str, np.ndarray] = {}
    for leg in LEGS:
        chain = links[leg]
        xyz = np.zeros((CHAIN_LENGTH, 3))
        for i in range(CHAIN_LENGTH - 1, -1, -1):
            ref = chain[i - 1] if i > 0 else urdf.root
            xyz[i] = urdf.chain_xyz(ref, chain[i])
        out[leg] = xyz
    return out


def inertia_matrix(inertia: Dict[str, str]) -> np.ndarray:
    ixx = float(inertia['ixx'])
    iyy = float(inertia['iyy'])
    izz = float(inertia['izz'])
    ixy = float(inertia.get('ixy', '0'))
    ixz = float(inertia.get('ixz', '0'))
    iyz = float(inertia.get('iyz', '0'))
    return np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])


def rpy_to_matrix(rpy: Sequence[float]) -> np.ndarray:
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


# ---------------------------------------------------------------------------
# CHAMP gait dump (compiles tools/dump_champ_gait.cpp on demand)
# ---------------------------------------------------------------------------

DUMP_SOURCE = SOURCE_PACKAGE_DIR / 'tools' / 'dump_champ_gait.cpp'
DUMP_HEADER = SOURCE_PACKAGE_DIR / 'tools' / 'champ_robot_config.h'
CHAMP_INCLUDE = SOURCE_PACKAGE_DIR.parent / 'champ' / 'include' / 'champ'


def _pkg_config(*args: str) -> List[str]:
    return subprocess.check_output(['pkg-config', *args], text=True).split()


def dump_binary(build_dir: Path | str = '/tmp') -> Path:
    """Build (if stale) and return the dump_champ_gait executable."""
    binary = Path(build_dir) / 'dump_champ_gait'
    sources = [DUMP_SOURCE, DUMP_HEADER]
    stale = not binary.exists() or any(
        s.exists() and s.stat().st_mtime > binary.stat().st_mtime for s in sources)
    if stale:
        cmd = ['g++', '-std=c++17', '-O2', f'-I{CHAMP_INCLUDE}', f'-I{DUMP_HEADER.parent}',
               *_pkg_config('--cflags', 'tinyxml2', 'yaml-cpp'),
               '-o', str(binary), str(DUMP_SOURCE),
               *_pkg_config('--libs', 'tinyxml2', 'yaml-cpp')]
        subprocess.check_call(cmd)
    return binary


def expanded_urdf_path(files: RobotFiles, build_dir: Path | str = '/tmp') -> Path:
    """Plain .urdf for the C++ tools (expands .xacro into build_dir)."""
    if not files.is_xacro:
        return files.urdf
    out = Path(build_dir) / f'{files.robot}_expanded.urdf'
    if not out.exists() or files.urdf.stat().st_mtime > out.stat().st_mtime:
        out.write_text(expand_urdf_text(files.urdf), encoding='utf-8')
    return out


def run_dump(files: RobotFiles, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0,
             ticks: int = 0, swing: Optional[float] = None,
             stance: Optional[float] = None) -> str:
    args = [str(dump_binary()), str(expanded_urdf_path(files)), str(files.gait_yaml),
            str(files.joints_yaml), str(files.links_yaml),
            repr(vx), repr(vy), repr(wz), str(ticks)]
    if swing is not None:
        args.append(repr(swing))
        args.append(repr(stance) if stance is not None else '-1')
    return subprocess.check_output(args, text=True)


def dump_stand(files: RobotFiles) -> tuple[np.ndarray, np.ndarray]:
    """CHAMP standing joints (12, CHAMP order) and base-frame foot positions (4x3)."""
    joints = None
    feet = np.zeros((4, 3))
    for line in run_dump(files, ticks=0).splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == 'stand_joints':
            joints = np.array([float(v) for v in parts[1:]])
        elif parts[0].startswith('stand_foot_'):
            feet[int(parts[0].rsplit('_', 1)[1])] = [float(v) for v in parts[1:4]]
    if joints is None or len(joints) != 12:
        raise RuntimeError('dump_champ_gait did not print stand_joints')
    return joints, feet


def dump_trajectory(files: RobotFiles, vx: float, vy: float, wz: float, ticks: int,
                    swing: Optional[float] = None, stance: Optional[float] = None) -> np.ndarray:
    rows = [[float(v) for v in line.split()]
            for line in run_dump(files, vx, vy, wz, ticks, swing, stance).splitlines() if line]
    return np.asarray(rows, dtype=np.float64)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _main(argv: Iterable[str]) -> int:
    argv = list(argv)
    if len(argv) >= 1 and argv[0] == 'list':
        pkg = Path(argv[1]) if len(argv) > 1 else SOURCE_PACKAGE_DIR
        for robot in available_robots(pkg):
            print(robot)
        return 0
    if len(argv) == 3 and argv[0] == 'expand':
        out = Path(argv[2])
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(expand_urdf_text(argv[1]), encoding='utf-8')
        return 0
    print(__doc__)
    return 2


if __name__ == '__main__':
    raise SystemExit(_main(sys.argv[1:]))
