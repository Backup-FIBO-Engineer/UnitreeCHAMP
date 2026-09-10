#!/usr/bin/env python3
"""Write mujoco/go2.xml from official Go2 URDF translations and inertias."""

from pathlib import Path

OUT = Path(__file__).resolve().parents[1] / 'mujoco' / 'go2.xml'

LEGS = {
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


def xyz(t):
    return f'{t[0]} {t[1]} {t[2]}'


def leg_xml(name, spec):
    hx, hy, hz = spec['hip']
    tx, ty, tz = spec['thigh']
    cx, cy, cz = spec['calf']
    fx, fy, fz = spec['foot']
    hip_com, hip_m, hip_i = spec['hip_i']
    th_com, th_m, th_i = spec['thigh_i']
    ca_com, ca_m, ca_i = spec['calf_i']
    return f'''
      <body name="{name}_hip" pos="{xyz(spec['hip'])}">
        <joint name="{name}_hip_joint" type="hinge" axis="1 0 0"
               limited="true" range="{spec['hip_range']}"/>
        <inertial pos="{hip_com}" mass="{hip_m}" fullinertia="{hip_i}"/>
        <geom name="{name}_hip_collision" type="capsule"
              fromto="0 0 0 {tx} {ty} {tz}" size="0.03"
              rgba="0.5 0.5 0.5 1"/>

        <body name="{name}_thigh" pos="{xyz(spec['thigh'])}">
          <joint name="{name}_thigh_joint" type="hinge" axis="0 1 0"
                 limited="true" range="{spec['thigh_range']}"/>
          <inertial pos="{th_com}" mass="{th_m}" fullinertia="{th_i}"/>
          <geom name="{name}_thigh_collision" type="capsule"
                fromto="0 0 0 {cx} {cy} {cz}" size="0.02"
                rgba="0.3 0.3 0.3 1"/>

          <body name="{name}_calf" pos="{xyz(spec['calf'])}">
            <joint name="{name}_calf_joint" type="hinge" axis="0 1 0"
                   limited="true" range="{spec['calf_range']}"/>
            <inertial pos="{ca_com}" mass="{ca_m}" fullinertia="{ca_i}"/>
            <geom name="{name}_calf_collision" type="capsule"
                  fromto="0 0 0 {fx} {fy} {fz}" size="0.015"
                  rgba="0.3 0.3 0.3 1"/>
            <geom name="{name}_foot" type="sphere" pos="{xyz(spec['foot'])}"
                  size="0.022" rgba="0.9 0.4 0.2 1"/>
            <site name="{name}_foot_site" pos="{xyz(spec['foot'])}" size="0.006"/>
          </body>
        </body>
      </body>'''


def actuators():
    lines = []
    for name, spec in LEGS.items():
        for joint, force, rng in (
            ('hip', spec['hip_force'], spec['hip_range']),
            ('thigh', spec['thigh_force'], spec['thigh_range']),
            ('calf', spec['calf_force'], spec['calf_range']),
        ):
            jn = f'{name}_{joint}_joint'
            kp = '80' if joint == 'hip' else '160'
            lines.append(
                f'    <position name="{jn}" joint="{jn}" kp="{kp}" kv="6.0" '
                f'ctrlrange="{rng}" forcerange="-{force} {force}"/>'
            )
    return '\n'.join(lines)


def main() -> None:
    legs = '\n'.join(leg_xml(name, spec) for name, spec in LEGS.items())
    xml = f'''<mujoco model="go2">
  <!--
    Kinematics and inertial values copied from official Unitree go2_description.urdf.
    Collision is capsules/spheres, not DAE meshes. Requires MuJoCo >= 3.1 (position kv).
  -->
  <compiler angle="radian" autolimits="true" inertiafromgeom="false"/>
  <option timestep="0.001" gravity="0 0 -9.81" integrator="implicitfast"/>

  <default>
    <joint damping="0.5" armature="0.01" frictionloss="0.05"/>
    <geom friction="1.2 0.005 0.0001" condim="3"/>
    <position kp="160" kv="6.0" ctrllimited="true" forcelimited="true"/>
  </default>

  <worldbody>
    <geom name="floor" type="plane" size="10 10 0.1" rgba="0.8 0.8 0.8 1"/>
    <light pos="0 0 2" dir="0 0 -1"/>

    <!-- Spawn above standing contact: nominal 0.30 + foot radius 0.022. -->
    <body name="base" pos="0 0 0.33">
      <freejoint name="root"/>
      <inertial
        pos="0.021112 0 -0.005366"
        mass="6.921"
        fullinertia="0.02448 0.098077 0.107 0.00012166 0.0014849 -3.12e-05"/>
      <geom name="base_collision" type="box" size="0.1881 0.04675 0.057"
            rgba="0.2 0.3 0.6 1"/>
      <body name="imu" pos="-0.02557 0 0.04232">
        <inertial pos="0 0 0" mass="0.001" diaginertia="1e-6 1e-6 1e-6"/>
      </body>
{legs}
    </body>
  </worldbody>

  <actuator>
{actuators()}
  </actuator>
</mujoco>
'''
    OUT.write_text(xml)
    print(f'wrote {OUT}')


if __name__ == '__main__':
    main()
