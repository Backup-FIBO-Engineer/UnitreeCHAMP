#!/usr/bin/env python3
"""Replay a CHAMP Go2/B2 gait dump in MuJoCo and report achieved body velocity.

    verify_mujoco_walk.py [--robot go2|b2] [vx | sweep]
"""
from __future__ import annotations

import argparse
import subprocess
from pathlib import Path
from typing import Optional

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DUMP_SRC = ROOT / 'tools' / 'dump_unitree_gait.cpp'
DUMP_BIN = Path('/tmp/dump_unitree_gait')
CHAMP_INC = ROOT.parent / 'champ' / 'include' / 'champ'
JOINTS = [
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
]

# swing/stance defaults mirror config/<robot>_gait.yaml.
ROBOTS = {
    'go2': {'xml': ROOT / 'mujoco' / 'go2.xml', 'swing': 0.08, 'stance': 0.25, 'default_vx': 0.35},
    'b2': {'xml': ROOT / 'mujoco' / 'b2.xml', 'swing': 0.10, 'stance': 0.30, 'default_vx': 0.35},
}


def load_traj(robot: str, vx: float, ticks: int, swing: float, stance: float) -> np.ndarray:
    if not DUMP_BIN.exists() or DUMP_SRC.stat().st_mtime > DUMP_BIN.stat().st_mtime:
        subprocess.check_call(
            ['g++', '-std=c++17', '-O2', f'-I{CHAMP_INC}', '-o', str(DUMP_BIN), str(DUMP_SRC)]
        )
    out = subprocess.check_output(
        [str(DUMP_BIN), robot, str(vx), '0', '0', str(ticks), str(swing), str(stance)], text=True
    )
    rows = [list(map(float, line.split())) for line in out.strip().splitlines() if line]
    return np.asarray(rows, dtype=np.float64)


def run(
    robot: str,
    vx_cmd: float,
    ticks: int,
    swing: float,
    stance: float,
    kp: Optional[float],
    friction: Optional[float],
    kv: Optional[float] = None,
) -> float:
    traj = load_traj(robot, vx_cmd, ticks, swing, stance)
    model = mujoco.MjModel.from_xml_path(str(ROBOTS[robot]['xml']))
    data = mujoco.MjData(model)
    jids = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINTS])
    qadr = model.jnt_qposadr[jids]
    aids = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in JOINTS])
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
    for q in traj:
        data.ctrl[aids] = q
        for _ in range(steps_per):
            mujoco.mj_step(model, data)
            if np.any(np.abs(data.actuator_force[aids]) >= 0.98 * forcerange):
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
        f'{robot} vx={vx_cmd:.2f} swing={swing:.2f} stance={stance:.2f} kp={kp} kv={kv} mu={friction} '
        f'mean_vx={mean_vx:.3f} ({100 * mean_vx / max(vx_cmd, 1e-6):.0f}%) '
        f'dx={dist:.3f}/{duration:.2f}s tilt={np.max(tilts):.1f}deg sat={sat}'
    )
    return mean_vx


SWEEPS = {
    'go2': [
        (0.25, 0.05, 0.25, None, None, None),
        (0.35, 0.05, 0.25, None, None, None),
        (0.25, 0.08, 0.25, None, None, None),
        (0.25, 0.05, 0.25, 200.0, 1.5, 8.0),
        (0.25, 0.08, 0.25, 200.0, 1.5, 8.0),
        (0.35, 0.08, 0.25, 160.0, 1.2, 6.0),
        (0.50, 0.08, 0.25, 160.0, 1.2, 6.0),
        (0.25, 0.08, 0.20, 160.0, 1.2, 6.0),
        (0.25, 0.06, 0.30, 160.0, 1.2, 6.0),
        (0.50, 0.08, 0.25, 120.0, 1.5, 5.0),
    ],
    'b2': [
        (0.25, 0.10, 0.30, None, None, None),
        (0.35, 0.10, 0.30, None, None, None),
        (0.60, 0.10, 0.30, None, None, None),
        (0.35, 0.08, 0.30, None, None, None),
        (0.35, 0.10, 0.25, None, None, None),
        (0.35, 0.10, 0.30, 1500.0, 1.2, 30.0),
        (0.35, 0.10, 0.30, 700.0, 1.2, 15.0),
        (0.60, 0.10, 0.30, 1500.0, 1.2, 30.0),
    ],
}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', choices=sorted(ROBOTS), default='go2')
    parser.add_argument('vx', nargs='?', default=None, help='commanded vx in m/s, or "sweep"')
    args = parser.parse_args()
    cfg = ROBOTS[args.robot]
    ticks = 800
    if args.vx == 'sweep':
        for vx, swing, stance, kp, mu, kv in SWEEPS[args.robot]:
            run(args.robot, vx, ticks, swing, stance, kp, mu, kv)
        return 0
    vx_cmd = float(args.vx) if args.vx is not None else cfg['default_vx']
    mean_vx = run(args.robot, vx_cmd, ticks, cfg['swing'], cfg['stance'], None, None)
    ok = mean_vx > 0.55 * vx_cmd
    print('PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
