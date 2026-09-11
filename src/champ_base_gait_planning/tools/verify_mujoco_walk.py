#!/usr/bin/env python3
"""Replay a CHAMP gait dump in MuJoCo and report the achieved body velocity.

    verify_mujoco_walk.py --robot <robot> [vx] [--ticks N] [--swing S] [--stance T]
                          [--kp KP] [--kv KV] [--friction MU]

The trajectory comes from tools/dump_champ_gait.cpp (URDF + yaml of the robot),
the joint names from joints_map and the model from mujoco/<robot>.xml. Without a
vx the command is 70% of gait.max_linear_velocity_x; the check passes when the
mean forward velocity after start-up exceeds 55% of the command.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from champ_robot_files import (  # noqa: E402
    SOURCE_PACKAGE_DIR, RobotFiles, champ_joint_names, dump_trajectory, load_ros_params,
    resolve_robot_files,
)


def run(
    files: RobotFiles,
    xml_path: Path,
    vx_cmd: float,
    ticks: int,
    swing: Optional[float],
    stance: Optional[float],
    kp: Optional[float],
    kv: Optional[float],
    friction: Optional[float],
) -> float:
    params = load_ros_params(*files.champ_yamls())
    joints = champ_joint_names(params)
    traj = dump_trajectory(files, vx_cmd, 0.0, 0.0, ticks, swing, stance)
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    jids = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in joints])
    qadr = model.jnt_qposadr[jids]
    aids = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in joints])
    if np.any(jids < 0) or np.any(aids < 0):
        raise RuntimeError(f'{xml_path} lacks a joint/actuator from joints_map: {joints}')
    if kp is not None:
        model.actuator_gainprm[aids, 0] = kp
        model.actuator_biasprm[aids, 1] = -kp
    if kv is not None:
        model.actuator_biasprm[aids, 2] = -kv
    if friction is not None:
        model.geom_friction[:, 0] = friction
    root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'root')
    root_q = int(model.jnt_qposadr[root])
    root_v = int(model.jnt_dofadr[root])

    data.qpos[qadr] = traj[0]
    data.qvel[:] = 0
    data.ctrl[aids] = traj[0]
    mujoco.mj_forward(model, data)

    xs = []
    vxs = []
    tilts = []
    sat = 0
    dt = float(model.opt.timestep)
    steps_per = max(1, int(round(0.005 / dt)))
    forcerange = np.abs(model.actuator_forcerange[aids, 1])
    limited = model.actuator_forcelimited[aids].astype(bool)
    for q in traj:
        data.ctrl[aids] = q
        for _ in range(steps_per):
            mujoco.mj_step(model, data)
            if np.any(limited & (np.abs(data.actuator_force[aids]) >= 0.98 * forcerange)):
                sat += 1
        xs.append(float(data.qpos[root_q]))
        vxs.append(float(data.qvel[root_v]))
        qw, qx, qy, qz = data.qpos[root_q + 3:root_q + 7]
        z_axis = np.array([
            2 * (qx * qz + qy * qw),
            2 * (qy * qz - qx * qw),
            1 - 2 * (qx * qx + qy * qy),
        ])
        tilts.append(float(np.degrees(np.arccos(np.clip(z_axis[2], -1, 1)))))

    vxs = np.array(vxs)
    xs = np.array(xs)
    skip = min(200, len(vxs) // 4)
    mean_vx = float(np.mean(vxs[skip:]))
    dist = float(xs[-1] - xs[skip])
    duration = (len(vxs) - skip) * 0.005
    print(
        f'{files.robot} vx={vx_cmd:.3f} swing={swing} stance={stance} kp={kp} kv={kv} mu={friction} '
        f'mean_vx={mean_vx:.3f} ({100 * mean_vx / max(vx_cmd, 1e-6):.0f}%) '
        f'dx={dist:.3f}/{duration:.2f}s tilt={np.max(tilts):.1f}deg sat={sat}'
    )
    return mean_vx


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', required=True)
    parser.add_argument('--package-dir', type=Path, default=SOURCE_PACKAGE_DIR)
    parser.add_argument('--xml', type=Path, default=None)
    parser.add_argument('vx', nargs='?', type=float, default=None, help='commanded vx in m/s')
    parser.add_argument('--ticks', type=int, default=800, help='5 ms ticks to replay')
    parser.add_argument('--swing', type=float, default=None, help='override gait.swing_height')
    parser.add_argument('--stance', type=float, default=None, help='override gait.stance_duration')
    parser.add_argument('--kp', type=float, default=None)
    parser.add_argument('--kv', type=float, default=None)
    parser.add_argument('--friction', type=float, default=None)
    parser.add_argument('--min-ratio', type=float, default=0.55)
    args = parser.parse_args()

    files = resolve_robot_files(args.package_dir, args.robot)
    xml_path = args.xml or files.mujoco_xml
    if xml_path is None:
        print(f'FAIL: robot {args.robot!r} has no mujoco/{args.robot}.xml')
        return 1
    params = load_ros_params(files.gait_yaml)
    vx_cmd = args.vx if args.vx is not None else 0.7 * float(params['gait']['max_linear_velocity_x'])
    mean_vx = run(files, xml_path, vx_cmd, args.ticks, args.swing, args.stance,
                  args.kp, args.kv, args.friction)
    ok = mean_vx > args.min_ratio * vx_cmd
    print('PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
