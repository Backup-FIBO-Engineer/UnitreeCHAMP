#!/usr/bin/env python3
"""Check that a robot URDF + CHAMP yaml satisfy the CHAMP loader/IK assumptions.

    verify_urdf_champ_contract.py --robot <robot>

champ::URDF::getPose sums parent_to_joint xyz and ignores rpy, the IK expects
the hip axis +X and the upper/lower leg axes +Y, and the leg chain must hang
off the URDF root (links_map.base). Everything is read from urdf/<robot>.*,
config/<robot>_links.yaml, config/<robot>_joints.yaml and config/<robot>_gait.yaml.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from champ_robot_files import (  # noqa: E402
    CHAIN_LENGTH, LEGS, LEG_SHORT, SOURCE_PACKAGE_DIR, champ_leg_translations, leg_chains,
    load_ros_params, load_urdf, resolve_robot_files,
)

ROLES = ('hip', 'upper leg', 'lower leg', 'foot')


def result(label: str, ok: bool, detail: str = '') -> int:
    print(f"[{'PASS' if ok else 'FAIL'}] {label}" + (f': {detail}' if detail else ''))
    return 0 if ok else 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', required=True)
    parser.add_argument('--package-dir', type=Path, default=SOURCE_PACKAGE_DIR)
    args = parser.parse_args()

    files = resolve_robot_files(args.package_dir, args.robot)
    params = load_ros_params(*files.champ_yamls())
    urdf = load_urdf(files.urdf)
    links = leg_chains(params, 'links_map')
    joints = leg_chains(params, 'joints_map')
    base_link = str(params['links_map'].get('base', ''))
    imu_link = str(params['links_map'].get('imu', ''))
    gait = params['gait']
    nominal_height = float(gait['nominal_height'])
    failures = 0

    print(f"{args.robot}: URDF '{urdf.name}' root={urdf.root} base={base_link} imu={imu_link}")
    failures += result('links_map.base is the URDF root link', base_link == urdf.root, f'{base_link!r} vs {urdf.root!r}')
    failures += result('links_map.imu is a URDF link', imu_link in urdf.links, imu_link)
    for key in ('knee_orientation', 'max_linear_velocity_x', 'max_linear_velocity_y',
                'max_angular_velocity_z', 'swing_height', 'stance_duration', 'swing_duration', 'nominal_height'):
        failures += result(f'gait.{key} present', key in gait)
    failures += result('gait.knee_orientation is ">>" (Unitree-style knees)', gait.get('knee_orientation') == '>>', str(gait.get('knee_orientation')))

    for leg in LEGS:
        for name in links[leg]:
            failures += result(f'{leg} link {name} exists', name in urdf.links)
        for i, name in enumerate(joints[leg]):
            j = urdf.joints.get(name)
            if j is None:
                failures += result(f'{leg} joint {name} exists', False)
                continue
            failures += result(f'{name} drives links_map[{i}] {links[leg][i]}', j.child == links[leg][i], f'child={j.child}')
            failures += result(f'{name} rpy=0 ({ROLES[i]})', np.allclose(j.rpy, 0.0), str(j.rpy))
            if i == 0:
                failures += result(f'{name} axis +X (hip roll)', j.type == 'revolute' and np.allclose(j.axis, [1, 0, 0]), f'{j.type} {j.axis}')
                failures += result(f'{name} parent is links_map.base', j.parent == base_link, j.parent)
            elif i < CHAIN_LENGTH - 1:
                failures += result(f'{name} axis +Y ({ROLES[i]} pitch)', j.type == 'revolute' and np.allclose(j.axis, [0, 1, 0]), f'{j.type} {j.axis}')
                failures += result(f'{name} parent is links_map[{i - 1}]', j.parent == links[leg][i - 1], j.parent)
            else:
                failures += result(f'{name} foot joint is fixed', j.type == 'fixed', j.type)
                failures += result(f'{name} parent is the lower leg', j.parent == links[leg][i - 1], j.parent)
            if i < CHAIN_LENGTH - 1:
                lim = j.limit
                has = all(k in lim for k in ('lower', 'upper', 'effort', 'velocity'))
                failures += result(f'{name} has lower/upper/effort/velocity limits', has, str(lim))
                if 'lower' in lim and 'upper' in lim:
                    failures += result(f'{name} lower < upper', float(lim['lower']) < float(lim['upper']), f"{lim['lower']} {lim['upper']}")

    xyz = champ_leg_translations(urdf, links)
    reaches = {}
    for leg in LEGS:
        hip, upper, lower, foot = xyz[leg]
        l1 = float(np.linalg.norm(lower[[0, 2]]))
        l2 = float(np.linalg.norm(foot[[0, 2]]))
        reaches[leg] = l1 + l2
        l0 = float(upper[1] + lower[1] + foot[1])
        short = LEG_SHORT[leg]
        print(f'  CHAMP {short} xyz hip={hip.tolist()} upper={upper.tolist()} lower={lower.tolist()} foot={foot.tolist()}')
        failures += result(f'{short} planar reach l1+l2 > 0', l1 > 0 and l2 > 0, f'l1={l1:.4f} l2={l2:.4f} reach={l1 + l2:.4f}')
        failures += result(f'{short} l0 (hip Y offset) points to the same side as the hip', (hip[1] > 0) == (l0 > 0) and abs(l0) > 1e-4, f'hip.y={hip[1]:.4f} l0={l0:.5f}')
        failures += result(f'{short} hip is {"front" if "front" in leg else "hind"} (x sign)', (hip[0] > 0) == ('front' in leg), f'{hip[0]:.4f}')
        failures += result(f'{short} hip is {"left" if "left" in leg else "right"} (y sign)', (hip[1] > 0) == ('left' in leg), f'{hip[1]:.4f}')
    spread = max(reaches.values()) - min(reaches.values())
    failures += result('all legs share the same planar reach', spread < 1e-3, f'{reaches}')
    reach = min(reaches.values())
    failures += result(f'gait.nominal_height {nominal_height} < leg reach {reach:.4f}', nominal_height < reach, f'margin={(reach - nominal_height) * 1000:.1f} mm')
    failures += result('gait.nominal_height leaves >= 15% of reach for stepping', nominal_height <= 0.85 * reach, f'{nominal_height / reach * 100:.0f}% of reach')

    print(f'\nResult: {failures} failed checks')
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())
