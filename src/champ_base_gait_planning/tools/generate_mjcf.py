#!/usr/bin/env python3
"""Write mujoco/<robot>.xml from the robot URDF and its CHAMP/sim yaml files.

    python3 tools/generate_mjcf.py <robot> [--output path]

Everything comes from the files of the selected robot:
  * kinematic tree, joint axes/limits/efforts and inertials: urdf/<robot>.urdf
  * collision geometry: the URDF <collision> primitives (box, cylinder, sphere);
    mesh collisions are skipped, so a robot whose URDF only has mesh collision
    needs a hand-written MJCF (see mujoco/xgo.xml)
  * actuated joints: config/<robot>_joints.yaml (joints_map, CHAMP order)
  * base/foot links: config/<robot>_links.yaml
  * spawn height: gait.nominal_height from config/<robot>_gait.yaml
  * simulator tuning: sim.* from config/<robot>_sim.yaml

Numeric URDF attribute strings are copied verbatim so that
tools/validate_mujoco_against_urdf.py compares equal at float precision.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Dict, List

sys.path.insert(0, str(Path(__file__).resolve().parent))

from champ_robot_files import (  # noqa: E402
    CHAIN_LENGTH, LEGS, SOURCE_PACKAGE_DIR, RobotFiles, Urdf, UrdfCollision, UrdfLink,
    champ_joint_names, leg_chains, load_ros_params, load_urdf, resolve_robot_files,
)

MASSLESS = 1e-6

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


def geom_xml(name: str, collision: UrdfCollision, rgba: str, indent: str) -> str:
    """MuJoCo geom for a URDF collision primitive; None for meshes."""
    attrib = collision.attrib
    if collision.geometry == 'box':
        sx, sy, sz = (float(v) for v in attrib['size'].split())
        shape = f'type="box" size="{num(sx / 2)} {num(sy / 2)} {num(sz / 2)}"'
    elif collision.geometry == 'cylinder':
        shape = (f'type="cylinder" size="{attrib["radius"]} '
                 f'{num(float(attrib["length"]) / 2)}"')
    elif collision.geometry == 'sphere':
        shape = f'type="sphere" size="{attrib["radius"]}"'
    else:
        return f'{indent}<!-- {name}: URDF {collision.geometry} collision skipped -->'
    parts = [f'<geom name="{name}" {shape}']
    if not is_zero(collision.xyz_text):
        parts.append(f'pos="{collision.xyz_text}"')
    if not is_zero(collision.rpy_text):
        # URDF rpy is fixed-axis roll, pitch, yaw = MuJoCo eulerseq "xyz" (extrinsic).
        parts.append(f'euler="{collision.rpy_text}"')
    parts.append(f'rgba="{rgba}"/>')
    return indent + ' '.join(parts)


class Generator:
    def __init__(self, files: RobotFiles) -> None:
        self.files = files
        self.urdf: Urdf = load_urdf(files.urdf)
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
            if collision.geometry not in ('box', 'cylinder', 'sphere'):
                self.skipped.append(f'{link.name} ({collision.geometry})')
            self.emit(geom_xml(name, collision, rgba, indent))

    def emit_body(self, link_name: str, depth: int) -> None:
        indent = '  ' * depth
        link = self.urdf.links[link_name]
        joint = self.urdf.parent_joint.get(link_name)
        if joint is None:
            self.emit(f'{indent}<body name="{link_name}" pos="0 0 {num(self.spawn_height())}">')
            self.emit(f'{indent}  <freejoint name="root"/>')
        else:
            attrs = [f'name="{link_name}"']
            if not is_zero(joint.xyz_text):
                attrs.append(f'pos="{joint.xyz_text}"')
            if not is_zero(joint.rpy_text):
                attrs.append(f'euler="{joint.rpy_text}"')
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
        self.emit(f'<mujoco model="{self.urdf.name or self.files.robot}">')
        self.emit('  <!--')
        self.emit(f'    Generated by tools/generate_mjcf.py {self.files.robot}; do not edit.')
        self.emit(f'    Source: {self.files.urdf.name} + config/{self.files.robot}_*.yaml.')
        self.emit('    Collision = URDF primitives, robot self-collision off (contype 1 / conaffinity 0),')
        self.emit('    floor conaffinity 1. Requires MuJoCo >= 3.1 (position kv).')
        self.emit('  -->')
        self.emit('  <compiler angle="radian" autolimits="true" inertiafromgeom="false"/>')
        self.emit(f'  <option timestep="{num(float(self.sim["timestep"]))}" gravity="0 0 -9.81" '
                  'integrator="implicitfast"/>')
        self.emit('')
        self.emit('  <default>')
        self.emit(f'    <joint damping="{num(float(self.sim["joint_damping"]))}" '
                  f'armature="{num(float(self.sim["joint_armature"]))}" '
                  f'frictionloss="{num(float(self.sim["joint_frictionloss"]))}"/>')
        self.emit(f'    <geom friction="{friction}" condim="3" contype="1" conaffinity="0"/>')
        self.emit('    <position ctrllimited="true" forcelimited="true"/>')
        self.emit('  </default>')
        self.emit('')
        self.emit('  <worldbody>')
        self.emit('    <geom name="floor" type="plane" size="10 10 0.1" contype="1" conaffinity="1" '
                  'rgba="0.8 0.8 0.8 1"/>')
        self.emit('    <light pos="0 0 2" dir="0 0 -1"/>')
        self.emit('')
        self.emit(f'    <!-- Spawn above standing contact: nominal {num(self.nominal_height)} + '
                  f'foot bottom {num(self.foot_bottom_offset())} + clearance '
                  f'{num(float(self.sim["spawn_clearance"]))}. -->')
        self.emit_body(self.urdf.root, 2)
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
    args = parser.parse_args()

    files = resolve_robot_files(args.package_dir, args.robot)
    generator = Generator(files)
    xml = generator.generate()
    output = args.output or (args.package_dir / 'mujoco' / f'{args.robot}.xml')
    output.write_text(xml, encoding='utf-8')
    print(f'wrote {output} ({len(generator.lines)} lines)')
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
