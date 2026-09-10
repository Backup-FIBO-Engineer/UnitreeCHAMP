#!/usr/bin/env python3
"""Check that XGO URDF satisfies CHAMP's loader/IK assumptions.

champ::URDF::getPose sums parent_to_joint xyz and ignores rpy, and the IK
expects hip axis +X and thigh/calf axis +Y. This is the contract between
urdf/xgo_rviz.xacro, config/xgo_links.yaml, and config/xgo_joints.yaml.
"""

from __future__ import annotations

import argparse
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

LEGS = ('left_front', 'right_front', 'left_hind', 'right_hind')
PRESETS = {
    'xgo': {
        'prefix': {
            'left_front': 'lf',
            'right_front': 'rf',
            'left_hind': 'lh',
            'right_hind': 'rh',
        },
        'link_suffixes': ('_hip_link', '_upper_leg_link', '_lower_leg_link', '_foot_link'),
        'joint_suffixes': ('_hip_joint', '_upper_leg_joint', '_lower_leg_joint', '_foot_joint'),
        'hip_parent': 'base_link',
        'reach': 0.146,
        'l0_abs_range': (0.04, 0.06),
    },
    'go2': {
        'prefix': {
            'left_front': 'FL',
            'right_front': 'FR',
            'left_hind': 'RL',
            'right_hind': 'RR',
        },
        'link_suffixes': ('_hip', '_thigh', '_calf', '_foot'),
        'joint_suffixes': ('_hip_joint', '_thigh_joint', '_calf_joint', '_foot_joint'),
        'hip_parent': 'base',
        'reach': 0.426,
        'l0_abs_range': (0.09, 0.11),
        'xyz': {
            'FL': {
                'hip': [0.1934, 0.0465, 0.0],
                'upper': [0.0, 0.0955, 0.0],
                'lower': [0.0, 0.0, -0.213],
                'foot': [0.0, 0.0, -0.213],
            },
            'FR': {
                'hip': [0.1934, -0.0465, 0.0],
                'upper': [0.0, -0.0955, 0.0],
                'lower': [0.0, 0.0, -0.213],
                'foot': [0.0, 0.0, -0.213],
            },
            'RL': {
                'hip': [-0.1934, 0.0465, 0.0],
                'upper': [0.0, 0.0955, 0.0],
                'lower': [0.0, 0.0, -0.213],
                'foot': [0.0, 0.0, -0.213],
            },
            'RR': {
                'hip': [-0.1934, -0.0465, 0.0],
                'upper': [0.0, -0.0955, 0.0],
                'lower': [0.0, 0.0, -0.213],
                'foot': [0.0, 0.0, -0.213],
            },
        },
    },
}


def vec(text: str) -> np.ndarray:
    return np.array([float(v) for v in text.split()], dtype=float)


def parse_string_list_block(text: str, key: str) -> Dict[str, List[str]]:
    """Parse a ros2 yaml map of string lists without requiring PyYAML."""
    out: Dict[str, List[str]] = {}
    current = None
    in_section = False
    for raw in text.splitlines():
        line = raw.rstrip()
        stripped = line.strip()
        if stripped.startswith(f'{key}:'):
            in_section = True
            continue
        if not in_section:
            continue
        if re.match(r'^[a-zA-Z_].*:', stripped) and not stripped.startswith('-') and not line.startswith(' '):
            break
        m = re.match(r'^\s+([a-zA-Z0-9_]+):\s*$', line)
        if m:
            current = m.group(1)
            out[current] = []
            continue
        m = re.match(r'^\s+-\s+(\S+)\s*$', line)
        if m and current is not None:
            out[current].append(m.group(1))
    return out


def result(label: str, ok: bool, detail: str = '') -> int:
    print(f"[{'PASS' if ok else 'FAIL'}] {label}" + (f': {detail}' if detail else ''))
    return 0 if ok else 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('urdf_xacro', type=Path)
    parser.add_argument('links_yaml', type=Path)
    parser.add_argument('joints_yaml', type=Path)
    parser.add_argument('--preset', choices=sorted(PRESETS), default='xgo')
    args = parser.parse_args()
    preset = PRESETS[args.preset]
    prefixes = preset['prefix']

    root = ET.parse(args.urdf_xacro).getroot()
    joints = {}
    for joint in root.findall('joint'):
        origin = joint.find('origin')
        axis = joint.find('axis')
        joints[joint.attrib['name']] = {
            'type': joint.attrib['type'],
            'origin': vec(origin.attrib.get('xyz', '0 0 0')) if origin is not None else np.zeros(3),
            'rpy': vec(origin.attrib.get('rpy', '0 0 0')) if origin is not None else np.zeros(3),
            'axis': vec(axis.attrib.get('xyz', '0 0 0')) if axis is not None else np.zeros(3),
            'parent': joint.find('parent').attrib['link'],
            'child': joint.find('child').attrib['link'],
        }

    links_map = parse_string_list_block(args.links_yaml.read_text(), 'links_map')
    joints_map = parse_string_list_block(args.joints_yaml.read_text(), 'joints_map')
    failures = 0

    failures += result('links_map has four legs', set(links_map) >= set(LEGS))
    failures += result('joints_map has four legs', set(joints_map) >= set(LEGS))

    for leg in LEGS:
        prefix = prefixes[leg]
        expected_links = [f'{prefix}{suffix}' for suffix in preset['link_suffixes']]
        expected_joints = [f'{prefix}{suffix}' for suffix in preset['joint_suffixes']]
        failures += result(
            f'{leg} links_map',
            links_map.get(leg) == expected_links,
            f'{links_map.get(leg)}',
        )
        failures += result(
            f'{leg} joints_map',
            joints_map.get(leg) == expected_joints,
            f'{joints_map.get(leg)}',
        )

        chain: List[Tuple[str, str]] = [
            (expected_joints[0], 'hip X'),
            (expected_joints[1], 'upper Y'),
            (expected_joints[2], 'lower Y'),
            (expected_joints[3], 'foot fixed'),
        ]
        for name, role in chain:
            j = joints.get(name)
            if j is None:
                failures += result(f'{name} exists', False)
                continue
            failures += result(f'{name} rpy=0 ({role})', np.allclose(j['rpy'], 0.0), str(j['rpy']))

        hip = joints[expected_joints[0]]
        upper = joints[expected_joints[1]]
        lower = joints[expected_joints[2]]
        foot = joints[expected_joints[3]]
        failures += result(f'{prefix} hip axis +X', np.allclose(hip['axis'], [1.0, 0.0, 0.0]), str(hip['axis']))
        failures += result(f'{prefix} upper axis +Y', np.allclose(upper['axis'], [0.0, 1.0, 0.0]), str(upper['axis']))
        failures += result(f'{prefix} lower axis +Y', np.allclose(lower['axis'], [0.0, 1.0, 0.0]), str(lower['axis']))
        failures += result(
            f'{prefix} hip parent {preset["hip_parent"]}',
            hip['parent'] == preset['hip_parent'],
        )
        failures += result(f'{prefix} foot is fixed', foot['type'] == 'fixed')

        l1 = float(np.linalg.norm(lower['origin'][[0, 2]]))
        l2 = float(np.linalg.norm(foot['origin'][[0, 2]]))
        reach = preset['reach']
        failures += result(
            f'{prefix} planar reach ~{reach} m',
            abs(l1 + l2 - reach) < 0.002,
            f'l1={l1:.4f} l2={l2:.4f} sum={l1 + l2:.4f}',
        )

        # CHAMP fillLeg: translation of each joint is the origin xyz (rpy=0).
        print(
            f'  CHAMP {prefix} xyz hip={hip["origin"].tolist()} '
            f'upper={upper["origin"].tolist()} lower={lower["origin"].tolist()} '
            f'foot={foot["origin"].tolist()}'
        )
        expected_xyz = preset.get('xyz', {}).get(prefix)
        if expected_xyz is not None:
            for role, joint in (
                ('hip', hip),
                ('upper', upper),
                ('lower', lower),
                ('foot', foot),
            ):
                failures += result(
                    f'{prefix} {role} xyz matches CHAMP verifier',
                    np.allclose(joint['origin'], expected_xyz[role], atol=1e-9, rtol=0.0),
                    f'urdf={joint["origin"].tolist()} lock={expected_xyz[role]}',
                )

    print(f'\nResult: {failures} failed checks')
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())
