#!/usr/bin/env python3
"""Write mujoco/<robot>.xml from the robot URDF and its CHAMP/sim yaml files.

    python3 tools/generate_mjcf.py <robot> [--output path]
    python3 tools/generate_mjcf.py <robot> --patch-visuals   # keep a hand-written MJCF

Everything comes from the files of the selected robot:
  * kinematic tree, joint axes/limits/efforts and inertials: urdf/<robot>.urdf
  * collision geometry: the URDF <collision> primitives (box, cylinder, sphere);
    mesh collisions are skipped, so a robot whose URDF only has mesh collision
    needs a hand-written MJCF (see mujoco/xgo.xml). `--patch-visuals` keeps that
    file's kinematics/collision and only inserts the URDF visual meshes.
  * visual geometry: the URDF <visual> meshes (dae/stl), converted by
    tools/champ_mesh_assets.py into mujoco/assets/<robot>/*.obj (one per
    material, coloured from the URDF/COLLADA materials) and emitted as
    non-colliding group-1 mesh geoms; collision primitives go to group 3 so
    the viewer shows the real robot (press 3 to overlay the primitives)
  * actuated joints: config/<robot>_joints.yaml (joints_map, CHAMP order)
  * base/foot links: config/<robot>_links.yaml
  * spawn height: gait.nominal_height from config/<robot>_gait.yaml
  * simulator tuning: sim.* from config/<robot>_sim.yaml

Numeric URDF attribute strings (xyz, sizes, limits, inertia) are copied
verbatim so that tools/validate_mujoco_against_urdf.py compares equal at float
precision; rpy origins become quaternions (see origin_attrs).
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional

sys.path.insert(0, str(Path(__file__).resolve().parent))

from champ_mesh_assets import MeshAssets, convert_robot_meshes  # noqa: E402
from champ_robot_files import (  # noqa: E402
    CHAIN_LENGTH, LEGS, SOURCE_PACKAGE_DIR, RobotFiles, Urdf, UrdfCollision, UrdfLink,
    UrdfVisual, champ_joint_names, leg_chains, load_ros_params, load_urdf,
    resolve_robot_files,
)

MASSLESS = 1e-6
PRIMITIVES = ('box', 'cylinder', 'sphere')
VISUAL_GROUP = 1  # shown by default in the MuJoCo viewers
COLLISION_GROUP = 3  # hidden by default; toggled with key 3

SIM_DEFAULTS = {
    'actuator_kp_hip': 80.0,
    'actuator_kp_leg': 160.0,
    'actuator_kv': 6.0,
    'joint_damping': 0.5,
    'joint_armature': 0.01,
    'joint_frictionloss': 0.05,
    'geom_friction': [1.0, 0.005, 0.0001],
    'spawn_clearance': 0.02,
    'timestep': 0.001,
}

COLORS = {
    'base': '0.2 0.3 0.6 1',
    'leg': '0.35 0.35 0.35 1',
    'foot': '0.9 0.4 0.2 1',
    'other': '0.5 0.5 0.5 1',
}


def num(value: float) -> str:
    """Shortest decimal that round-trips to the same double."""
    return repr(float(value))


def fullinertia(link: UrdfLink) -> str:
    i = link.inertia
    return ' '.join(i.get(k, '0') for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'))


def rpy_to_quat(rpy_text: str) -> str:
    r, p, y = (float(v) for v in rpy_text.split())
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return f'{num(qw)} {num(qx)} {num(qy)} {num(qz)}'


def is_zero(text: str) -> bool:
    return all(abs(float(v)) < 1e-12 for v in text.split())


def primitive_shape(geometry: str, attrib: Dict[str, str]) -> str:
    """MuJoCo type/size attributes for a URDF box, cylinder or sphere."""
    if geometry == 'box':
        sx, sy, sz = (float(v) for v in attrib['size'].split())
        return f'type="box" size="{num(sx / 2)} {num(sy / 2)} {num(sz / 2)}"'
    if geometry == 'cylinder':
        return f'type="cylinder" size="{attrib["radius"]} {num(float(attrib["length"]) / 2)}"'
    if geometry == 'sphere':
        return f'type="sphere" size="{attrib["radius"]}"'
    raise ValueError(f'not a primitive: {geometry}')


def origin_attrs(xyz_text: str, rpy_text: str) -> List[str]:
    """pos/quat attributes for a URDF <origin>.

    URDF rpy is R = Rz(yaw) Ry(pitch) Rx(roll). MuJoCo's default euler
    sequence "xyz" composes the other way round (Rx Ry Rz), which only agrees
    when at most one angle is non-zero, so the rotation is emitted as a
    quaternion instead of copying the rpy text into euler="...".
    """
    parts = []
    if not is_zero(xyz_text):
        parts.append(f'pos="{xyz_text}"')
    if not is_zero(rpy_text):
        parts.append(f'quat="{rpy_to_quat(rpy_text)}"')
    return parts


def geom_xml(name: str, collision: UrdfCollision, rgba: str, indent: str) -> str:
    """MuJoCo collision geom for a URDF collision primitive; a comment for meshes."""
    if collision.geometry not in PRIMITIVES:
        return f'{indent}<!-- {name}: URDF {collision.geometry} collision skipped -->'
    parts = [f'<geom name="{name}" class="collision" {primitive_shape(collision.geometry, collision.attrib)}']
    parts.extend(origin_attrs(collision.xyz_text, collision.rpy_text))
    parts.append(f'rgba="{rgba}"/>')
    return indent + ' '.join(parts)


def rgba_text(values) -> str:
    return ' '.join(f'{float(v):g}' for v in values)


class Generator:
    def __init__(self, files: RobotFiles, visual_meshes: bool = True) -> None:
        self.files = files
        self.urdf: Urdf = load_urdf(files.urdf)
        self.assets: Optional[MeshAssets] = None
        if visual_meshes:
            self.assets = convert_robot_meshes(files.package_dir, files.robot, self.urdf)
        self.mesh_assets: Dict[str, str] = {}  # MuJoCo mesh name -> <mesh .../> line
        params = load_ros_params(files.gait_yaml, files.joints_yaml, files.links_yaml)
        self.sim = dict(SIM_DEFAULTS)
        self.sim.update(files.sim_params())
        self.links = leg_chains(params, 'links_map')
        self.joint_names = champ_joint_names(params)
        self.hip_joints = {self.joint_names[i] for i in range(0, 12, 3)}
        self.base_link = str(params['links_map'].get('base', self.urdf.root))
        self.nominal_height = float(params['gait']['nominal_height'])
        self.foot_links = [self.links[leg][CHAIN_LENGTH - 1] for leg in LEGS]
        self.leg_links = {name for leg in LEGS for name in self.links[leg][:CHAIN_LENGTH - 1]}
        self.skipped: List[str] = []
        self.lines: List[str] = []

    # -- geometry helpers ---------------------------------------------------------

    def foot_bottom_offset(self) -> float:
        """Distance from the foot link origin down to the lowest point of its sphere."""
        offsets = []
        for foot in self.foot_links:
            link = self.urdf.links[foot]
            spheres = [c for c in link.collisions if c.geometry == 'sphere']
            if not spheres:
                raise RuntimeError(
                    f'foot link {foot!r} has no sphere collision in the URDF; cannot place '
                    'the floor contact. Hand-write the MJCF for this robot.'
                )
            sphere = spheres[0]
            offsets.append(float(sphere.attrib['radius']) - sphere.xyz[2])
        return max(offsets)

    def spawn_height(self) -> float:
        return self.nominal_height + self.foot_bottom_offset() + float(self.sim['spawn_clearance'])

    def color(self, link: str) -> str:
        if link == self.base_link:
            return COLORS['base']
        if link in self.foot_links:
            return COLORS['foot']
        if link in self.leg_links:
            return COLORS['leg']
        return COLORS['other']

    # -- emitters -----------------------------------------------------------------

    def emit(self, text: str) -> None:
        self.lines.append(text)

    def emit_inertial(self, link: UrdfLink, indent: str) -> None:
        if link.mass is None or link.mass < MASSLESS or not link.inertia:
            self.emit(f'{indent}<!-- {link.name}: massless in URDF -->')
            return
        parts = [f'<inertial pos="{link.inertial_xyz_text}"']
        if not is_zero(link.inertial_rpy_text):
            parts.append(f'quat="{rpy_to_quat(link.inertial_rpy_text)}"')
        parts.append(f'mass="{link.mass_text}"')
        parts.append(f'fullinertia="{fullinertia(link)}"/>')
        self.emit(indent + ' '.join(parts))

    def emit_geoms(self, link: UrdfLink, indent: str) -> None:
        rgba = self.color(link.name)
        for index, collision in enumerate(link.collisions):
            name = f'{link.name}_collision' if index == 0 else f'{link.name}_collision_{index}'
            if collision.geometry not in PRIMITIVES:
                self.skipped.append(f'{link.name} ({collision.geometry})')
            self.emit(geom_xml(name, collision, rgba, indent))
        for index, visual in enumerate(link.visuals):
            base = f'{link.name}_visual' if index == 0 else f'{link.name}_visual_{index}'
            self.emit_visual(link, base, visual, indent)

    def mesh_asset(self, part_name: str, file: str, scale_text: str) -> str:
        """Register a <mesh> asset (one per file and scale) and return its name."""
        name = part_name
        attrs = [f'name="{name}"', f'file="{file}"']
        if not all(abs(float(v) - 1.0) < 1e-12 for v in scale_text.split()):
            suffix = '_'.join(f'{float(v):g}' for v in scale_text.split()).replace('.', 'p').replace('-', 'm')
            name = f'{part_name}_s{suffix}'
            attrs = [f'name="{name}"', f'file="{file}"', f'scale="{scale_text}"']
        self.mesh_assets.setdefault(name, f'    <mesh {" ".join(attrs)}/>')
        return name

    def emit_visual(self, link: UrdfLink, base: str, visual: UrdfVisual, indent: str) -> None:
        origin = origin_attrs(visual.xyz_text, visual.rpy_text)
        if visual.geometry in PRIMITIVES:
            parts = [f'<geom name="{base}" class="visual" {primitive_shape(visual.geometry, visual.attrib)}']
            parts.extend(origin)
            parts.append(f'rgba="{visual.rgba or self.color(link.name)}"/>')
            self.emit(indent + ' '.join(parts))
            return
        if visual.geometry != 'mesh':
            self.emit(f'{indent}<!-- {base}: URDF {visual.geometry} visual skipped -->')
            return
        filename = visual.attrib.get('filename', '')
        entries = self.assets.entries.get(filename) if self.assets else None
        if not entries:
            self.emit(f'{indent}<!-- {base}: visual mesh {filename} not converted -->')
            return
        scale_text = visual.attrib.get('scale', '1 1 1')
        for k, entry in enumerate(entries):
            mesh = self.mesh_asset(entry.name, entry.file, scale_text)
            # Colour: the URDF <material> named after the COLLADA effect wins, then the mesh file's own colour.
            rgba = visual.materials.get(entry.material) or rgba_text(entry.rgba)
            name = base if len(entries) == 1 else f'{base}_{k}'
            parts = [f'<geom name="{name}" class="visual" mesh="{mesh}"']
            parts.extend(origin)
            parts.append(f'rgba="{rgba}"/>')
            self.emit(indent + ' '.join(parts))

    def visual_geom_elements(self, link: UrdfLink):
        """Parseable <geom class="visual"> elements for one URDF link (registers assets)."""
        import xml.etree.ElementTree as ET

        saved, self.lines = self.lines, []
        for index, visual in enumerate(link.visuals):
            base = f'{link.name}_visual' if index == 0 else f'{link.name}_visual_{index}'
            self.emit_visual(link, base, visual, '')
        lines, self.lines = self.lines, saved
        elements = []
        for line in lines:
            text = line.strip()
            if text.startswith('<geom'):
                elements.append(ET.fromstring(text))
        return elements

    def patch_existing(self, xml_path: Path) -> str:
        """Keep a hand-written MJCF; add/replace URDF visual mesh geoms and assets."""
        import xml.etree.ElementTree as ET

        if not xml_path.exists():
            raise FileNotFoundError(
                f'{xml_path} does not exist; --patch-visuals needs a hand-written MJCF')
        parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
        root = ET.parse(xml_path, parser=parser).getroot()

        compiler = root.find('compiler')
        if compiler is None:
            raise RuntimeError(f'{xml_path}: no <compiler>')
        compiler.set('meshdir', f'assets/{self.files.robot}')

        default = root.find('default')
        if default is None:
            raise RuntimeError(f'{xml_path}: no <default>')
        self._ensure_default_class(default, 'collision', {'group': str(COLLISION_GROUP)})
        self._ensure_default_class(default, 'visual', {
            'type': 'mesh', 'group': str(VISUAL_GROUP),
            'contype': '0', 'conaffinity': '0', 'mass': '0',
        })

        bodies = {body.attrib['name']: body for body in root.iter('body') if 'name' in body.attrib}
        for body in bodies.values():
            for geom in list(body.findall('geom')):
                name = geom.attrib.get('name', '')
                if geom.attrib.get('class') == 'visual' or '_visual' in name:
                    body.remove(geom)
                else:
                    geom.set('class', 'collision')

        attached = 0
        missing: List[str] = []
        for link in self.urdf.links.values():
            if not link.visuals:
                continue
            body = bodies.get(link.name)
            if body is None:
                missing.append(link.name)
                continue
            for element in self.visual_geom_elements(link):
                self._insert_before_child_bodies(body, element)
                attached += 1
        if missing:
            print('skipped visuals on URDF links with no MJCF body: ' + ', '.join(missing))

        for child in list(root):
            if isinstance(child.tag, str):
                continue
            text = child.text or ''
            if 'Kinematics' in text and 'Visual meshes' not in text:
                child.text = (text.rstrip() +
                              '\n    Visual meshes: tools/generate_mjcf.py patch-visuals '
                              f'(assets/{self.files.robot}; collision stays in group {COLLISION_GROUP}).\n  ')
            break

        old_asset = root.find('asset')
        if old_asset is not None:
            root.remove(old_asset)
        if self.mesh_assets:
            asset = ET.Element('asset')
            for line in self.mesh_assets.values():
                asset.append(ET.fromstring(line.strip()))
            self._insert_after_tag(root, asset, 'default')

        ET.indent(root, space='  ')
        xml = ET.tostring(root, encoding='unicode')
        if not xml.endswith('\n'):
            xml += '\n'
        print(f'patched {attached} visual geoms into {xml_path.name} '
              f'({len(self.mesh_assets)} mesh assets)')
        self.lines = xml.splitlines()
        return xml

    @staticmethod
    def _ensure_default_class(default, class_name: str, geom_attrib: Dict[str, str]) -> None:
        import xml.etree.ElementTree as ET

        for child in default.findall('default'):
            if child.attrib.get('class') == class_name:
                geom = child.find('geom')
                if geom is None:
                    geom = ET.SubElement(child, 'geom')
                geom.attrib.update(geom_attrib)
                return
        child = ET.SubElement(default, 'default', {'class': class_name})
        ET.SubElement(child, 'geom', geom_attrib)

    @staticmethod
    def _insert_after_tag(parent, element, tag: str) -> None:
        for index, child in enumerate(list(parent)):
            if isinstance(child.tag, str) and child.tag == tag:
                parent.insert(index + 1, element)
                return
        parent.append(element)

    @staticmethod
    def _insert_before_child_bodies(body, element) -> None:
        """Put visual geoms after joints/inertial/collision, before sites and child bodies."""
        insert_at = 0
        for index, child in enumerate(list(body)):
            tag = child.tag if isinstance(child.tag, str) else ''
            if tag in ('joint', 'freejoint', 'inertial', 'geom'):
                insert_at = index + 1
            elif tag in ('site', 'body'):
                body.insert(index, element)
                return
        body.insert(insert_at, element)

    def emit_body(self, link_name: str, depth: int) -> None:
        indent = '  ' * depth
        link = self.urdf.links[link_name]
        joint = self.urdf.parent_joint.get(link_name)
        if joint is None:
            self.emit(f'{indent}<body name="{link_name}" pos="0 0 {num(self.spawn_height())}">')
            self.emit(f'{indent}  <freejoint name="root"/>')
        else:
            attrs = [f'name="{link_name}"'] + origin_attrs(joint.xyz_text, joint.rpy_text)
            self.emit(f'{indent}<body {" ".join(attrs)}>')
            if joint.type in ('revolute', 'continuous'):
                jattrs = [f'name="{joint.name}"', 'type="hinge"', f'axis="{joint.axis_text}"']
                if 'lower' in joint.limit and 'upper' in joint.limit:
                    jattrs.append(f'range="{joint.limit["lower"]} {joint.limit["upper"]}"')
                self.emit(f'{indent}  <joint {" ".join(jattrs)}/>')
            elif joint.type != 'fixed':
                raise RuntimeError(f'joint {joint.name!r} type {joint.type!r} is not supported')
        self.emit_inertial(link, indent + '  ')
        self.emit_geoms(link, indent + '  ')
        if link_name in self.foot_links:
            self.emit(f'{indent}  <site name="{link_name}_site" pos="0 0 0" size="0.006"/>')
        for child_joint in self.urdf.children.get(link_name, []):
            self.emit_body(child_joint.child, depth + 1)
        self.emit(f'{indent}</body>')

    def emit_actuators(self) -> None:
        kv = num(float(self.sim['actuator_kv']))
        self.emit('  <actuator>')
        for name in self.joint_names:
            joint = self.urdf.joints[name]
            kp = self.sim['actuator_kp_hip'] if name in self.hip_joints else self.sim['actuator_kp_leg']
            attrs = [f'name="{name}"', f'joint="{name}"', f'kp="{num(float(kp))}"', f'kv="{kv}"']
            if 'lower' in joint.limit and 'upper' in joint.limit:
                attrs.append(f'ctrlrange="{joint.limit["lower"]} {joint.limit["upper"]}"')
            if 'effort' in joint.limit:
                attrs.append(f'forcerange="-{joint.limit["effort"]} {joint.limit["effort"]}"')
            self.emit(f'    <position {" ".join(attrs)}/>')
        self.emit('  </actuator>')

    def generate(self) -> str:
        if self.base_link != self.urdf.root:
            raise RuntimeError(
                f'links_map.base {self.base_link!r} must be the URDF root {self.urdf.root!r}')
        for name in self.joint_names:
            if self.urdf.joints[name].type not in ('revolute', 'continuous'):
                raise RuntimeError(f'joints_map joint {name!r} is not revolute in the URDF')
        friction = ' '.join(num(float(v)) for v in self.sim['geom_friction'])

        # Bodies first: they register the mesh assets that the <asset> block lists.
        body_lines: List[str] = []
        self.lines = body_lines
        self.emit(f'    <!-- Spawn above standing contact: nominal {num(self.nominal_height)} + '
                  f'foot bottom {num(self.foot_bottom_offset())} + clearance '
                  f'{num(float(self.sim["spawn_clearance"]))}. -->')
        self.emit_body(self.urdf.root, 2)

        self.lines = []
        self.emit(f'<mujoco model="{self.urdf.name or self.files.robot}">')
        self.emit('  <!--')
        self.emit(f'    Generated by tools/generate_mjcf.py {self.files.robot}; do not edit.')
        self.emit(f'    Source: {self.files.urdf.name} + config/{self.files.robot}_*.yaml.')
        self.emit('    Collision = URDF primitives, robot self-collision off (contype 1 / conaffinity 0),')
        self.emit('    floor conaffinity 1. Requires MuJoCo >= 3.1 (position kv).')
        if self.mesh_assets:
            self.emit(f'    Visual = URDF meshes converted to assets/{self.files.robot}/*.obj '
                      f'(group {VISUAL_GROUP}, no contacts);')
            self.emit(f'    collision primitives are in group {COLLISION_GROUP} (hidden by default, '
                      f'press {COLLISION_GROUP} in the viewer).')
        self.emit('  -->')
        compiler = '  <compiler angle="radian" autolimits="true" inertiafromgeom="false"'
        if self.mesh_assets:
            compiler += f' meshdir="assets/{self.files.robot}"'
        self.emit(compiler + '/>')
        self.emit(f'  <option timestep="{num(float(self.sim["timestep"]))}" gravity="0 0 -9.81" '
                  'integrator="implicitfast"/>')
        self.emit('')
        self.emit('  <default>')
        self.emit(f'    <joint damping="{num(float(self.sim["joint_damping"]))}" '
                  f'armature="{num(float(self.sim["joint_armature"]))}" '
                  f'frictionloss="{num(float(self.sim["joint_frictionloss"]))}"/>')
        self.emit(f'    <geom friction="{friction}" condim="3" contype="1" conaffinity="0"/>')
        self.emit('    <position ctrllimited="true" forcelimited="true"/>')
        self.emit('    <default class="collision">')
        self.emit(f'      <geom group="{COLLISION_GROUP}"/>')
        self.emit('    </default>')
        self.emit('    <default class="visual">')
        self.emit(f'      <geom type="mesh" group="{VISUAL_GROUP}" contype="0" conaffinity="0" mass="0"/>')
        self.emit('    </default>')
        self.emit('  </default>')
        self.emit('')
        if self.mesh_assets:
            self.emit('  <asset>')
            for line in self.mesh_assets.values():
                self.emit(line)
            self.emit('  </asset>')
            self.emit('')
        self.emit('  <worldbody>')
        self.emit('    <geom name="floor" type="plane" size="10 10 0.1" contype="1" conaffinity="1" '
                  'rgba="0.8 0.8 0.8 1"/>')
        self.emit('    <light pos="0 0 2" dir="0 0 -1"/>')
        self.emit('')
        self.lines.extend(body_lines)
        self.emit('  </worldbody>')
        self.emit('')
        self.emit_actuators()
        self.emit('</mujoco>')
        return '\n'.join(self.lines) + '\n'


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('robot')
    parser.add_argument('--package-dir', type=Path, default=SOURCE_PACKAGE_DIR)
    parser.add_argument('--output', type=Path, default=None,
                        help='default mujoco/<robot>.xml inside the package')
    parser.add_argument('--no-visual-meshes', action='store_true',
                        help='emit collision primitives only (no mesh assets)')
    parser.add_argument('--patch-visuals', action='store_true',
                        help='keep an existing hand-written MJCF; only insert URDF visual meshes')
    args = parser.parse_args()

    files = resolve_robot_files(args.package_dir, args.robot)
    generator = Generator(files, visual_meshes=not args.no_visual_meshes)
    output = args.output or (args.package_dir / 'mujoco' / f'{args.robot}.xml')
    if args.patch_visuals:
        xml = generator.patch_existing(output)
    else:
        try:
            xml = generator.generate()
        except RuntimeError as exc:
            if 'Hand-write the MJCF' in str(exc) and output.exists() and not args.no_visual_meshes:
                print(f'{exc}')
                print(f'patching visual meshes into existing {output}')
                xml = generator.patch_existing(output)
            else:
                raise
    output.write_text(xml, encoding='utf-8')
    print(f'wrote {output} ({len(generator.lines)} lines)')
    if generator.mesh_assets:
        print(f'visual meshes: {len(generator.mesh_assets)} assets in '
              f'{generator.assets.directory.relative_to(args.package_dir)}')
    if generator.skipped:
        print('skipped mesh collisions on: ' + ', '.join(generator.skipped))

    try:
        import mujoco
    except ImportError:
        print('mujoco not installed; skipped load check')
        return 0
    model = mujoco.MjModel.from_xml_path(str(output))
    print(f'MuJoCo load OK: nbody={model.nbody} nq={model.nq} nu={model.nu} '
          f'total mass={float(model.body_mass.sum()):.4f} kg')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
