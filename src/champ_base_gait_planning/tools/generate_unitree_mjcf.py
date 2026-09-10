#!/usr/bin/env python3
"""Write mujoco/<robot>.xml from official Unitree URDF translations and inertias.

    python3 tools/generate_unitree_mjcf.py            # go2 and b2
    python3 tools/generate_unitree_mjcf.py b2

Inertial strings are copied verbatim from the URDF so that
tools/validate_mujoco_against_urdf.py compares equal at float precision.
"""

import sys
from pathlib import Path

MUJOCO_DIR = Path(__file__).resolve().parents[1] / 'mujoco'

GO2_LEGS = {
    'FL': {
        'hip': (0.1934, 0.0465, 0.0),
        'thigh': (0.0, 0.0955, 0.0),
        'calf': (0.0, 0.0, -0.213),
        'foot': (0.0, 0.0, -0.213),
        'hip_i': ('-0.0054 0.00194 -0.000105', 0.678,
                  '0.00048 0.000884 0.000596 -3.01e-06 1.11e-06 -1.42e-06'),
        'thigh_i': ('-0.00374 -0.0223 -0.0327', 1.152,
                    '0.00584 0.0058 0.00103 8.72e-05 -0.000289 0.000808'),
        'calf_i': ('0.00548 -0.000975 -0.115', 0.154,
                   '0.00108 0.0011 3.29e-05 3.4e-07 1.72e-05 8.28e-06'),
        'hip_range': '-1.0472 1.0472',
        'thigh_range': '-1.5708 3.4907',
        'calf_range': '-2.7227 -0.83776',
        'hip_force': 23.7,
        'thigh_force': 23.7,
        'calf_force': 45.43,
    },
    'FR': {
        'hip': (0.1934, -0.0465, 0.0),
        'thigh': (0.0, -0.0955, 0.0),
        'calf': (0.0, 0.0, -0.213),
        'foot': (0.0, 0.0, -0.213),
        'hip_i': ('-0.0054 -0.00194 -0.000105', 0.678,
                  '0.00048 0.000884 0.000596 3.01e-06 1.11e-06 1.42e-06'),
        'thigh_i': ('-0.00374 0.0223 -0.0327', 1.152,
                    '0.00584 0.0058 0.00103 -8.72e-05 -0.000289 -0.000808'),
        'calf_i': ('0.00548 0.000975 -0.115', 0.154,
                   '0.00108 0.0011 3.29e-05 -3.4e-07 1.72e-05 -8.28e-06'),
        'hip_range': '-1.0472 1.0472',
        'thigh_range': '-1.5708 3.4907',
        'calf_range': '-2.7227 -0.83776',
        'hip_force': 23.7,
        'thigh_force': 23.7,
        'calf_force': 45.43,
    },
    'RL': {
        'hip': (-0.1934, 0.0465, 0.0),
        'thigh': (0.0, 0.0955, 0.0),
        'calf': (0.0, 0.0, -0.213),
        'foot': (0.0, 0.0, -0.213),
        'hip_i': ('0.0054 0.00194 -0.000105', 0.678,
                  '0.00048 0.000884 0.000596 3.01e-06 -1.11e-06 -1.42e-06'),
        'thigh_i': ('-0.00374 -0.0223 -0.0327', 1.152,
                    '0.00584 0.0058 0.00103 8.72e-05 -0.000289 0.000808'),
        'calf_i': ('0.00548 -0.000975 -0.115', 0.154,
                   '0.00108 0.0011 3.29e-05 3.4e-07 1.72e-05 8.28e-06'),
        'hip_range': '-1.0472 1.0472',
        'thigh_range': '-0.5236 4.5379',
        'calf_range': '-2.7227 -0.83776',
        'hip_force': 23.7,
        'thigh_force': 23.7,
        'calf_force': 45.43,
    },
    'RR': {
        'hip': (-0.1934, -0.0465, 0.0),
        'thigh': (0.0, -0.0955, 0.0),
        'calf': (0.0, 0.0, -0.213),
        'foot': (0.0, 0.0, -0.213),
        'hip_i': ('0.0054 -0.00194 -0.000105', 0.678,
                  '0.00048 0.000884 0.000596 -3.01e-06 -1.11e-06 1.42e-06'),
        'thigh_i': ('-0.00374 0.0223 -0.0327', 1.152,
                    '0.00584 0.0058 0.00103 -8.72e-05 -0.000289 -0.000808'),
        'calf_i': ('0.00548 0.000975 -0.115', 0.154,
                   '0.00108 0.0011 3.29e-05 -3.4e-07 1.72e-05 -8.28e-06'),
        'hip_range': '-1.0472 1.0472',
        'thigh_range': '-0.5236 4.5379',
        'calf_range': '-2.7227 -0.83776',
        'hip_force': 23.7,
        'thigh_force': 23.7,
        'calf_force': 45.43,
    },
}

# urdf/b2.urdf (unitree_ros b2_description). Joint limits are identical on all
# legs; the calf joint origin keeps the official +-8.7e-5 m y offset.
B2_CALF_I = ('0.012422 0 -0.12499', 0.404, '0.01143 0.011534 0.000331 0 0.000643 0')
B2_FOOT_I = ('-0.006511 0 -0.010144', 0.126, '3.8E-05 4.1E-05 4.2E-05 0 1.1E-05 0')
B2_ROTOR_I = ('0.0 0.0 0.0', 0.2734, None, '0.000144463 0.000144463 0.000263053')
B2_LEGS = {
    'FL': {
        'hip': (0.3285, 0.072, 0.0),
        'thigh': (0.0, 0.11973, 0.0),
        'calf': (0.0, -8.6984e-05, -0.35),
        'foot': (0.0, 0.0, -0.35),
        'hip_i': ('-0.003841 -0.009068 0', 2.673,
                  '0.0033188 0.0048743 0.0037087 7.16E-05 -3.77E-07 4E-09'),
        'thigh_i': ('-0.006279 -0.032049 -0.057835', 4.536,
                    '0.062299 0.061399 0.0081997 0.00087781 -0.0036475 0.0083537'),
        'calf_i': B2_CALF_I,
        'foot_i': B2_FOOT_I,
        'hip_rotor': (0.20205, 0.072, 0.0),
        'thigh_rotor': (0.0, -0.00798, 0.0),
        'calf_rotor': (0.0, -0.05788, 0.0),
        'hip_range': '-0.87 0.87',
        'thigh_range': '-0.94 4.69',
        'calf_range': '-2.82 -0.43',
        'hip_force': 200.0,
        'thigh_force': 200.0,
        'calf_force': 320.0,
    },
    'FR': {
        'hip': (0.3285, -0.072, 0.0),
        'thigh': (0.0, -0.11973, 0.0),
        'calf': (0.0, 8.6986e-05, -0.35),
        'foot': (0.0, 0.0, -0.35),
        'hip_i': ('-0.009305 0.010228 0.000264', 2.673,
                  '0.0026431 0.0046728 0.0034208 -0.00019234 -6.76E-06 7.16E-06'),
        'thigh_i': ('-0.006279 0.032049 -0.057835', 4.536,
                    '0.06229908 0.06139927 0.008199745 -0.000877808 -0.003647464 -0.008353671'),
        'calf_i': B2_CALF_I,
        'foot_i': B2_FOOT_I,
        'hip_rotor': (0.20205, -0.072, 0.0),
        'thigh_rotor': (0.0, 0.00798, 0.0),
        'calf_rotor': (0.0, 0.05788, 0.0),
        'hip_range': '-0.87 0.87',
        'thigh_range': '-0.94 4.69',
        'calf_range': '-2.82 -0.43',
        'hip_force': 200.0,
        'thigh_force': 200.0,
        'calf_force': 320.0,
    },
    'RL': {
        'hip': (-0.3285, 0.072, 0.0),
        'thigh': (0.0, 0.11973, 0.0),
        'calf': (0.0, -8.6984e-05, -0.35),
        'foot': (0.0, 0.0, -0.35),
        'hip_i': ('0.009305 -0.010228 0.000264', 2.673,
                  '0.0026431 0.0046728 0.0034208 -0.00019234 6.76E-06 -7.16E-06'),
        'thigh_i': ('-0.006279 -0.032049 -0.057835', 4.536,
                    '0.06229908 0.06139927 0.008199745 0.000877808 -0.003647464 0.008353671'),
        'calf_i': B2_CALF_I,
        'foot_i': B2_FOOT_I,
        'hip_rotor': (-0.20205, 0.072, 0.0),
        'thigh_rotor': (0.0, -0.00798, 0.0),
        'calf_rotor': (0.0, -0.05788, 0.0),
        'hip_range': '-0.87 0.87',
        'thigh_range': '-0.94 4.69',
        'calf_range': '-2.82 -0.43',
        'hip_force': 200.0,
        'thigh_force': 200.0,
        'calf_force': 320.0,
    },
    'RR': {
        'hip': (-0.3285, -0.072, 0.0),
        'thigh': (0.0, -0.11973, 0.0),
        'calf': (0.0, 8.6986e-05, -0.35),
        'foot': (0.0, -8.6984e-05, -0.35),
        'hip_i': ('0.003841 0.009068 0', 2.673,
                  '0.0033188 0.0048743 0.0037087 7.16E-05 3.77E-07 -4E-09'),
        'thigh_i': ('-0.006279 0.032049 -0.057835', 4.536,
                    '0.062299 0.061399 0.0081997 -0.00087781 -0.0036475 -0.0083537'),
        'calf_i': B2_CALF_I,
        'foot_i': B2_FOOT_I,
        'hip_rotor': (-0.20205, -0.072, 0.0),
        'thigh_rotor': (0.0, 0.00798, 0.0),
        'calf_rotor': (0.0, 0.05788, 0.0),
        'hip_range': '-0.87 0.87',
        'thigh_range': '-0.94 4.69',
        'calf_range': '-2.82 -0.43',
        'hip_force': 200.0,
        'thigh_force': 200.0,
        'calf_force': 320.0,
    },
}

ROBOTS = {
    'go2': {
        'model': 'go2',
        'legs': GO2_LEGS,
        'comment': (
            'Kinematics and inertial values copied from official Unitree go2_description.urdf.\n'
            '    Collision is capsules/spheres, not DAE meshes. Requires MuJoCo >= 3.1 (position kv).'
        ),
        'base_body': 'base',
        'base_i': ('0.021112 0 -0.005366', 6.921,
                   '0.02448 0.098077 0.107 0.00012166 0.0014849 -3.12e-05'),
        'base_box': '0.1881 0.04675 0.057',
        'imu_body': 'imu',
        'imu_pos': '-0.02557 0 0.04232',
        'spawn_comment': 'Spawn above standing contact: nominal 0.30 + foot radius 0.022.',
        'spawn_z': 0.33,
        'foot_radius': 0.022,
        'capsule_radii': (0.03, 0.02, 0.015),
        'kp_hip': '80',
        'kp_leg': '160',
        'kv': '6.0',
        'fixed_bodies': '',
        'leg_extra': False,
    },
    'b2': {
        'model': 'b2',
        'legs': B2_LEGS,
        'comment': (
            'Kinematics and inertial values copied from official Unitree b2_description.urdf\n'
            '    (unitree_ros/robots/b2_description). Rotor, foot, head and tail links are kept as\n'
            '    welded bodies so the simulated mass matches the URDF (~74.6 kg).\n'
            '    Collision is capsules/spheres, not DAE meshes. Requires MuJoCo >= 3.1 (position kv).'
        ),
        'base_body': 'base_link',
        'base_i': ('0.000458 0.005261 0.000665', 35.86,
                   '0.27466 1.0618 1.1825 -0.000622 0.00315 -0.00139'),
        'base_box': '0.25 0.14 0.075',
        'imu_body': 'imu_link',
        'imu_pos': '0 -0.02341 0.04927',
        'spawn_comment': 'Spawn above standing contact: nominal 0.50 + foot radius 0.032.',
        'spawn_z': 0.55,
        'foot_radius': 0.032,
        'capsule_radii': (0.05, 0.03, 0.02),
        # unitree_sdk2 b2_stand_example drives the real B2 with Kp 1000 / Kd 10.
        'kp_hip': '500',
        'kp_leg': '1000',
        'kv': '20.0',
        'fixed_bodies': '''
      <body name="head_Link" pos="0 0 0">
        <inertial pos="0.33989 -0.000168 0.12029" mass="3.6885"
                  fullinertia="0.026455 0.029221 0.010918 3.15E-05 0.00166 -3.27E-05"/>
        <geom name="head_collision" type="box" pos="0.41 0 0.005" size="0.02 0.06 0.07"
              rgba="0.2 0.3 0.6 1"/>
      </body>
      <body name="tail_link" pos="0 0 0">
        <inertial pos="-0.37448 0.000488 0.006495" mass="0.795"
                  fullinertia="0.0028118 0.0047506 0.0029154 1.95E-05 -7.87E-06 -1.57E-05"/>
        <geom name="tail_collision" type="box" pos="-0.405 0 0.005" size="0.0125 0.06 0.07"
              rgba="0.2 0.3 0.6 1"/>
      </body>''',
        'leg_extra': True,
    },
}


def xyz(t):
    return f'{t[0]} {t[1]} {t[2]}'


def rotor_xml(name, pos, indent):
    com, mass, _, diag = B2_ROTOR_I
    pad = ' ' * indent
    return (
        f'\n{pad}<body name="{name}" pos="{xyz(pos)}">'
        f'\n{pad}  <inertial pos="{com}" mass="{mass}" diaginertia="{diag}"/>'
        f'\n{pad}</body>'
    )


def leg_xml(name, spec, robot):
    tx, ty, tz = spec['thigh']
    cx, cy, cz = spec['calf']
    fx, fy, fz = spec['foot']
    hip_com, hip_m, hip_i = spec['hip_i']
    th_com, th_m, th_i = spec['thigh_i']
    ca_com, ca_m, ca_i = spec['calf_i']
    r_hip, r_thigh, r_calf = robot['capsule_radii']
    foot_r = robot['foot_radius']
    hip_extra = thigh_extra = calf_extra = ''
    if robot['leg_extra']:
        hip_extra = rotor_xml(f'{name}_thigh_rotor', spec['thigh_rotor'], 8)
        thigh_extra = rotor_xml(f'{name}_calf_rotor', spec['calf_rotor'], 10)
        fo_com, fo_m, fo_i = spec['foot_i']
        calf_extra = (
            f'\n            <body name="{name}_foot" pos="{xyz(spec["foot"])}">'
            f'\n              <inertial pos="{fo_com}" mass="{fo_m}" fullinertia="{fo_i}"/>'
            f'\n            </body>'
        )
    return f'''
      <body name="{name}_hip" pos="{xyz(spec['hip'])}">
        <joint name="{name}_hip_joint" type="hinge" axis="1 0 0"
               limited="true" range="{spec['hip_range']}"/>
        <inertial pos="{hip_com}" mass="{hip_m}" fullinertia="{hip_i}"/>
        <geom name="{name}_hip_collision" type="capsule"
              fromto="0 0 0 {tx} {ty} {tz}" size="{r_hip}"
              rgba="0.5 0.5 0.5 1"/>{hip_extra}

        <body name="{name}_thigh" pos="{xyz(spec['thigh'])}">
          <joint name="{name}_thigh_joint" type="hinge" axis="0 1 0"
                 limited="true" range="{spec['thigh_range']}"/>
          <inertial pos="{th_com}" mass="{th_m}" fullinertia="{th_i}"/>
          <geom name="{name}_thigh_collision" type="capsule"
                fromto="0 0 0 {cx} {cy} {cz}" size="{r_thigh}"
                rgba="0.3 0.3 0.3 1"/>{thigh_extra}

          <body name="{name}_calf" pos="{xyz(spec['calf'])}">
            <joint name="{name}_calf_joint" type="hinge" axis="0 1 0"
                   limited="true" range="{spec['calf_range']}"/>
            <inertial pos="{ca_com}" mass="{ca_m}" fullinertia="{ca_i}"/>
            <geom name="{name}_calf_collision" type="capsule"
                  fromto="0 0 0 {fx} {fy} {fz}" size="{r_calf}"
                  rgba="0.3 0.3 0.3 1"/>
            <geom name="{name}_foot" type="sphere" pos="{xyz(spec['foot'])}"
                  size="{foot_r}" rgba="0.9 0.4 0.2 1"/>
            <site name="{name}_foot_site" pos="{xyz(spec['foot'])}" size="0.006"/>{calf_extra}
          </body>
        </body>
      </body>'''


def hip_rotors_xml(robot):
    if not robot['leg_extra']:
        return ''
    return ''.join(
        rotor_xml(f'{name}_hip_rotor', spec['hip_rotor'], 6)
        for name, spec in robot['legs'].items()
    )


def actuators(robot):
    lines = []
    for name, spec in robot['legs'].items():
        for joint, force, rng in (
            ('hip', spec['hip_force'], spec['hip_range']),
            ('thigh', spec['thigh_force'], spec['thigh_range']),
            ('calf', spec['calf_force'], spec['calf_range']),
        ):
            jn = f'{name}_{joint}_joint'
            kp = robot['kp_hip'] if joint == 'hip' else robot['kp_leg']
            lines.append(
                f'    <position name="{jn}" joint="{jn}" kp="{kp}" kv="{robot["kv"]}" '
                f'ctrlrange="{rng}" forcerange="-{force} {force}"/>'
            )
    return '\n'.join(lines)


def build(robot) -> str:
    legs = '\n'.join(leg_xml(name, spec, robot) for name, spec in robot['legs'].items())
    base_com, base_m, base_i = robot['base_i']
    return f'''<mujoco model="{robot['model']}">
  <!--
    {robot['comment']}
  -->
  <compiler angle="radian" autolimits="true" inertiafromgeom="false"/>
  <option timestep="0.001" gravity="0 0 -9.81" integrator="implicitfast"/>

  <default>
    <joint damping="0.5" armature="0.01" frictionloss="0.05"/>
    <geom friction="1.2 0.005 0.0001" condim="3"/>
    <position kp="{robot['kp_leg']}" kv="{robot['kv']}" ctrllimited="true" forcelimited="true"/>
  </default>

  <worldbody>
    <geom name="floor" type="plane" size="10 10 0.1" rgba="0.8 0.8 0.8 1"/>
    <light pos="0 0 2" dir="0 0 -1"/>

    <!-- {robot['spawn_comment']} -->
    <body name="{robot['base_body']}" pos="0 0 {robot['spawn_z']}">
      <freejoint name="root"/>
      <inertial
        pos="{base_com}"
        mass="{base_m}"
        fullinertia="{base_i}"/>
      <geom name="base_collision" type="box" size="{robot['base_box']}"
            rgba="0.2 0.3 0.6 1"/>
      <body name="{robot['imu_body']}" pos="{robot['imu_pos']}">
        <inertial pos="0 0 0" mass="0.001" diaginertia="1e-6 1e-6 1e-6"/>
      </body>{robot['fixed_bodies']}{hip_rotors_xml(robot)}
{legs}
    </body>
  </worldbody>

  <actuator>
{actuators(robot)}
  </actuator>
</mujoco>
'''


def main(argv) -> None:
    names = argv[1:] or sorted(ROBOTS)
    for name in names:
        robot = ROBOTS[name]
        out = MUJOCO_DIR / f'{name}.xml'
        out.write_text(build(robot))
        print(f'wrote {out}')


if __name__ == '__main__':
    main(sys.argv)
