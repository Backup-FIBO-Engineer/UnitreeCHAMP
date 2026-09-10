#!/usr/bin/env python3
"""Comprehensive static checker for a robot URDF/Xacro versus MuJoCo MJCF (XGO, Go2, B2)."""

from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict

import numpy as np

REVOLUTE_JOINTS = [
    'lf_hip_joint', 'lf_upper_leg_joint', 'lf_lower_leg_joint',
    'rf_hip_joint', 'rf_upper_leg_joint', 'rf_lower_leg_joint',
    'lh_hip_joint', 'lh_upper_leg_joint', 'lh_lower_leg_joint',
    'rh_hip_joint', 'rh_upper_leg_joint', 'rh_lower_leg_joint',
]
FIXED_PAYLOAD_LINKS = ('camera_link', 'laser_link', 'imu_link')
LEGS = ('lf', 'rf', 'lh', 'rh')

UNITREE_JOINTS = [
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
]

# Unitree robots: MJCF collision is a box for the trunk (half sizes from the URDF
# collision box) and the IMU is a massless marker body, so its inertial is not compared.
UNITREE = {
    'go2': {
        'base_link': 'base',
        'imu_link': 'imu',
        'base_half_size': np.array([0.1881, 0.04675, 0.057]),
    },
    'b2': {
        'base_link': 'base_link',
        'imu_link': 'imu_link',
        'base_half_size': np.array([0.25, 0.14, 0.075]),
    },
}


def vec(text: str) -> np.ndarray:
    return np.array([float(value) for value in text.split()], dtype=float)


def close(a: np.ndarray, b: np.ndarray, tolerance: float = 1e-9) -> bool:
    return bool(np.allclose(a, b, atol=tolerance, rtol=0.0))


def parent_map(root: ET.Element) -> Dict[ET.Element, ET.Element]:
    return {child: parent for parent in root.iter() for child in parent}


def parse_urdf(path: Path):
    root = ET.parse(path).getroot()
    joints = {}
    links = {}
    for joint in root.findall('joint'):
        origin = joint.find('origin')
        axis = joint.find('axis')
        joints[joint.attrib['name']] = {
            'type': joint.attrib['type'],
            'origin': vec(origin.attrib.get('xyz', '0 0 0')),
            'rpy': vec(origin.attrib.get('rpy', '0 0 0')),
            'axis': vec(axis.attrib.get('xyz', '0 0 0')) if axis is not None else np.zeros(3),
            'parent': joint.find('parent').attrib['link'],
            'child': joint.find('child').attrib['link'],
            'limit': joint.find('limit').attrib if joint.find('limit') is not None else {},
        }
    for link in root.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        inertia = inertial.find('inertia').attrib
        links[link.attrib['name']] = {
            'mass': float(inertial.find('mass').attrib['value']),
            'com': vec(inertial.find('origin').attrib.get('xyz', '0 0 0')),
            'rpy': vec(inertial.find('origin').attrib.get('rpy', '0 0 0')),
            'fullinertia': np.array([
                float(inertia['ixx']), float(inertia['iyy']), float(inertia['izz']),
                float(inertia['ixy']), float(inertia['ixz']), float(inertia['iyz']),
            ]),
        }
    return joints, links


def parse_inertial(inertial: ET.Element) -> Dict[str, np.ndarray]:
    if 'fullinertia' in inertial.attrib:
        full = vec(inertial.attrib['fullinertia'])
    else:
        diag = vec(inertial.attrib['diaginertia'])
        full = np.array([diag[0], diag[1], diag[2], 0.0, 0.0, 0.0])
    return {
        'mass': float(inertial.attrib['mass']),
        'com': vec(inertial.attrib.get('pos', '0 0 0')),
        'fullinertia': full,
    }


def parse_mjcf(path: Path, revolute_joints):
    root = ET.parse(path).getroot()
    parents = parent_map(root)
    bodies = {body.attrib['name']: body for body in root.findall('.//body') if 'name' in body.attrib}
    joints = {}
    inertials = {}
    for name, body in bodies.items():
        inertial = body.find('inertial')
        if inertial is not None:
            inertials[name] = parse_inertial(inertial)
    for joint in root.findall('.//joint'):
        name = joint.attrib.get('name')
        if name not in revolute_joints:
            continue
        body = parents[joint]
        parent_element = parents[body]
        joints[name] = {
            'origin': vec(body.attrib.get('pos', '0 0 0')) + vec(joint.attrib.get('pos', '0 0 0')),
            'axis': vec(joint.attrib.get('axis', '0 0 1')),
            'range': vec(joint.attrib['range']),
            'body': body.attrib['name'],
            'parent': parent_element.attrib.get('name') if parent_element.tag == 'body' else None,
        }
    actuators = root.findall('./actuator/position')
    return root, bodies, joints, inertials, actuators, parents


def inertia_matrix(full: np.ndarray) -> np.ndarray:
    ixx, iyy, izz, ixy, ixz, iyz = full
    return np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])


def expected_limit(value: str) -> float:
    if value == '${pi}':
        return math.pi
    if value == '${-pi}':
        return -math.pi
    return float(value)


def result(label: str, ok: bool, detail: str = '') -> int:
    print(f"[{'PASS' if ok else 'FAIL'}] {label}" + (f': {detail}' if detail else ''))
    return 0 if ok else 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('urdf_xacro', type=Path)
    parser.add_argument('mujoco_xml', type=Path)
    parser.add_argument('--robot', choices=('xgo',) + tuple(sorted(UNITREE)), default='xgo')
    args = parser.parse_args()

    unitree = UNITREE.get(args.robot)
    if unitree is not None:
        revolute_joints = UNITREE_JOINTS
        payload_links = (unitree['imu_link'],)
        base_link = unitree['base_link']
        legs = ('FL', 'FR', 'RL', 'RR')
    else:
        revolute_joints = REVOLUTE_JOINTS
        payload_links = FIXED_PAYLOAD_LINKS
        base_link = 'base_link'
        legs = LEGS

    urdf_joints, urdf_links = parse_urdf(args.urdf_xacro)
    root, bodies, mj_joints, mj_inertials, actuators, parents = parse_mjcf(
        args.mujoco_xml, revolute_joints
    )
    failures = 0

    failures += result('12 revolute joints present', set(mj_joints) == set(revolute_joints))

    actuator_names = [a.attrib.get('name') for a in actuators]
    actuator_joints = [a.attrib.get('joint') for a in actuators]
    failures += result('12 position actuators present', len(actuators) == 12)
    failures += result('actuator names are unique', len(set(actuator_names)) == len(actuator_names))
    failures += result(
        'actuator joint mapping',
        actuator_names == revolute_joints and actuator_joints == revolute_joints,
    )

    for name in revolute_joints:
        uj = urdf_joints[name]
        mj = mj_joints.get(name)
        if mj is None:
            failures += result(f'{name} exists', False)
            continue
        failures += result(f'{name} parent', mj['parent'] == uj['parent'], f"urdf={uj['parent']} mjcf={mj['parent']}")
        failures += result(f'{name} child body', mj['body'] == uj['child'], f"urdf={uj['child']} mjcf={mj['body']}")
        failures += result(f'{name} origin', close(uj['origin'], mj['origin']), f"urdf={uj['origin']} mjcf={mj['origin']}")
        failures += result(f'{name} axis', close(uj['axis'], mj['axis']), f"urdf={uj['axis']} mjcf={mj['axis']}")
        expected_range = np.array([
            expected_limit(uj['limit']['lower']), expected_limit(uj['limit']['upper'])
        ])
        failures += result(f'{name} range', close(expected_range, mj['range'], 1e-15), f"urdf={expected_range} mjcf={mj['range']}")

        link = uj['child']
        ui = urdf_links[link]
        mi = mj_inertials.get(link)
        if mi is None:
            failures += result(f'{link} explicit inertial', False)
            continue
        failures += result(f'{link} mass', math.isclose(ui['mass'], mi['mass'], abs_tol=1e-12), f"urdf={ui['mass']:.15g} mjcf={mi['mass']:.15g}")
        failures += result(f'{link} COM', close(ui['com'], mi['com'], 1e-12))
        failures += result(f'{link} inertia', close(ui['fullinertia'], mi['fullinertia'], 1e-15))

    for link in (base_link,) + payload_links:
        ui = urdf_links.get(link)
        mi = mj_inertials.get(link)
        if unitree is not None and link == unitree['imu_link']:
            failures += result(f'{link} body exists', bodies.get(link) is not None)
            continue
        failures += result(f'{link} explicit inertial', mi is not None)
        if ui is None or mi is None:
            continue
        failures += result(f'{link} mass', math.isclose(ui['mass'], mi['mass'], abs_tol=1e-12))
        failures += result(f'{link} COM', close(ui['com'], mi['com'], 1e-12))
        failures += result(f'{link} inertia', close(ui['fullinertia'], mi['fullinertia'], 1e-15))

    for link in payload_links:
        joint = next(j for j in urdf_joints.values() if j['child'] == link)
        body = bodies.get(link)
        failures += result(f'{link} body exists', body is not None)
        if body is not None:
            parent_element = parents[body]
            parent_name = parent_element.attrib.get('name') if parent_element.tag == 'body' else None
            failures += result(f'{link} fixed parent', parent_name == joint['parent'])
            failures += result(f'{link} fixed origin', close(joint['origin'], vec(body.attrib.get('pos', '0 0 0')), 1e-12))

    for leg in legs:
        foot_joint = urdf_joints[f'{leg}_foot_joint']
        for element_name, element in (
            ('geom', root.find(f".//geom[@name='{leg}_foot']")),
            ('site', root.find(f".//site[@name='{leg}_foot_site']")),
        ):
            failures += result(f'{leg} foot {element_name} exists', element is not None)
            if element is not None:
                failures += result(f'{leg} foot {element_name} origin', close(foot_joint['origin'], vec(element.attrib['pos']), 1e-12))

    default_actuator = root.find('./default/position')
    if unitree is not None:
        for name in revolute_joints:
            effort = float(urdf_joints[name]['limit']['effort'])
            actuator = next(a for a in actuators if a.attrib.get('joint') == name)
            failures += result(
                f'{name} actuator force limit',
                close(vec(actuator.attrib['forcerange']), np.array([-effort, effort]), 1e-9),
            )
    elif default_actuator is None:
        failures += result('default position actuator exists', False)
    else:
        failures += result('actuator force limit equals URDF effort', vec(default_actuator.attrib['forcerange']).tolist() == [-25.0, 25.0])
        failures += result('actuator control range equals URDF joint range', close(vec(default_actuator.attrib['ctrlrange']), np.array([-math.pi, math.pi]), 1e-15))

    for link, item in mj_inertials.items():
        eigenvalues = np.linalg.eigvalsh(inertia_matrix(item['fullinertia']))
        positive = bool(np.all(eigenvalues > 0.0))
        triangle = bool(eigenvalues[0] + eigenvalues[1] >= eigenvalues[2] - 1e-15)
        failures += result(f'{link} inertia positive definite', positive, str(eigenvalues))
        failures += result(f'{link} principal inertia triangle', triangle, str(eigenvalues))

    chain_links = [base_link] + [urdf_joints[name]['child'] for name in revolute_joints]
    if unitree is not None:
        urdf_total = sum(urdf_links[link]['mass'] for link in chain_links)
        mjcf_total = sum(mj_inertials[link]['mass'] for link in chain_links)
        failures += result(
            'chain explicit mass',
            math.isclose(urdf_total, mjcf_total, abs_tol=1e-12),
            f'urdf={urdf_total:.12f} kg mjcf={mjcf_total:.12f} kg',
        )
    else:
        urdf_total = sum(link['mass'] for link in urdf_links.values())
        mjcf_total = sum(link['mass'] for link in mj_inertials.values())
        failures += result('total explicit mass', math.isclose(urdf_total, mjcf_total, abs_tol=1e-12), f'urdf={urdf_total:.12f} kg mjcf={mjcf_total:.12f} kg')

    if unitree is not None:
        expected_base_size = unitree['base_half_size']
        expected_base_pos = np.array([0.0, 0.0, 0.0])
        size_label = 'base collision half-size from URDF box'
        pos_label = 'base collision origin'
    else:
        expected_base_size = np.array([0.10447969287633896, 0.030220000073313713, 0.040951005241367966])
        expected_base_pos = np.array([-0.0016277730464935303, 0.0, 0.03910232422640547])
        size_label = 'base collision STL-bound size'
        pos_label = 'base collision STL-bound center'
    base_geom = root.find(".//geom[@name='base_collision']")
    failures += result('base collision geom exists', base_geom is not None)
    if base_geom is not None:
        failures += result(size_label, close(vec(base_geom.attrib['size']), expected_base_size, 1e-12))
        failures += result(pos_label, close(vec(base_geom.attrib.get('pos', '0 0 0')), expected_base_pos, 1e-12))

    print(f'\nResult: {failures} failed checks')
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())
