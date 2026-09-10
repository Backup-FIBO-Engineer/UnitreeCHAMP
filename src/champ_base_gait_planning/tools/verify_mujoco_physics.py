#!/usr/bin/env python3
"""Physics checks for mujoco/xgo.xml against CHAMP standing pose and URDF FK.

Requires the `mujoco` Python package. Does not need ROS.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import List

import numpy as np

XGO = {
    'joint_names': [
        'lf_hip_joint', 'lf_upper_leg_joint', 'lf_lower_leg_joint',
        'rf_hip_joint', 'rf_upper_leg_joint', 'rf_lower_leg_joint',
        'lh_hip_joint', 'lh_upper_leg_joint', 'lh_lower_leg_joint',
        'rh_hip_joint', 'rh_upper_leg_joint', 'rh_lower_leg_joint',
    ],
    'foot_sites': ['lf_foot_site', 'rf_foot_site', 'lh_foot_site', 'rh_foot_site'],
    'foot_geoms': ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot'],
    'base_body': 'base_link',
    'imu_body': 'imu_link',
    'nominal_height': 0.10,
    'imu_offset': np.array([0.085, 4.4164e-05, 0.070]),
    'stand_joints': np.array([
        0.00627760682, 0.823441148, -1.68932748,
        -4.37113883e-08, 0.827949941, -1.6963079,
        0.00627760682, 0.823441148, -1.68932748,
        -4.37113883e-08, 0.827949941, -1.6963079,
    ], dtype=np.float64),
    'stand_feet': np.array([
        [0.0749950036, 0.0717200041, -0.0999999866],
        [0.0749949887, -0.0718300045, -0.099999994],
        [-0.0750049949, 0.0717200562, -0.0999999866],
        [-0.0750050098, -0.0718300045, -0.099999994],
    ], dtype=np.float64),
}

GO2 = {
    'joint_names': [
        'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
        'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
        'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
        'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
    ],
    'foot_sites': ['FL_foot_site', 'FR_foot_site', 'RL_foot_site', 'RR_foot_site'],
    'foot_geoms': ['FL_foot', 'FR_foot', 'RL_foot', 'RR_foot'],
    'base_body': 'base',
    'imu_body': 'imu',
    'nominal_height': 0.30,
    'imu_offset': np.array([-0.02557, 0.0, 0.04232]),
    'stand_joints': np.array([
        -1.39090677e-08, 0.789464891, -1.57892966,
        -7.35137107e-08, 0.789464891, -1.57892966,
        -1.39090677e-08, 0.789464891, -1.57892966,
        -7.35137107e-08, 0.789464891, -1.57892966,
    ], dtype=np.float64),
    'stand_feet': np.array([
        [0.1933999658, 0.1419999897, -0.3000000119],
        [0.1933999658, -0.1420000196, -0.3000000119],
        [-0.1934000254, 0.1419999897, -0.3000000119],
        [-0.1934000254, -0.1420000196, -0.3000000119],
    ], dtype=np.float64),
}

ROBOTS = {'xgo': XGO, 'go2': GO2}


def result(label: str, ok: bool, detail: str = '') -> int:
    print(f"[{'PASS' if ok else 'FAIL'}] {label}" + (f': {detail}' if detail else ''))
    return 0 if ok else 1


def world_to_body(quat_wxyz: np.ndarray, vector_world: np.ndarray) -> np.ndarray:
    qw, qx, qy, qz = quat_wxyz
    rotation_world_from_body = np.array([
        [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw),
         2.0 * (qx * qz + qy * qw)],
        [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz),
         2.0 * (qy * qz - qx * qw)],
        [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw),
         1.0 - 2.0 * (qx * qx + qy * qy)],
    ])
    return rotation_world_from_body.T @ vector_world


def main() -> int:
    try:
        import mujoco
    except ImportError:
        print('SKIP: mujoco Python package is not installed')
        return 0

    parser = argparse.ArgumentParser()
    parser.add_argument('xml_path', type=Path)
    parser.add_argument('--robot', choices=sorted(ROBOTS), default=None)
    args = parser.parse_args()
    robot_name = args.robot
    if robot_name is None:
        robot_name = 'go2' if 'go2' in args.xml_path.stem.lower() else 'xgo'
    cfg = ROBOTS[robot_name]
    joint_names = cfg['joint_names']
    foot_sites = cfg['foot_sites']
    foot_geoms = cfg['foot_geoms']

    model = mujoco.MjModel.from_xml_path(str(args.xml_path))
    data = mujoco.MjData(model)
    failures = 0

    failures += result('MuJoCo >= 3.1', tuple(int(x) for x in mujoco.__version__.split('.')[:2]) >= (3, 1), mujoco.__version__)
    failures += result('nq includes freejoint + 12 hinges', model.nq == 7 + 12, f'nq={model.nq}')
    failures += result('12 position actuators', model.nu == 12, f'nu={model.nu}')

    qpos_adr = np.array([model.joint(name).qposadr[0] for name in joint_names])
    act_id = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in joint_names])
    failures += result('actuator ids match joint order', np.all(act_id == np.arange(12)), str(act_id.tolist()))

    # Identity quat: world_to_body is a no-op (same helper as mujoco_sim.py).
    ident = np.array([1.0, 0.0, 0.0, 0.0])
    vec = np.array([0.1, -0.2, 0.3])
    failures += result('world_to_body identity', np.allclose(world_to_body(ident, vec), vec))

    # CHAMP standing FK vs MuJoCo foot sites, base at origin so world==base.
    mujoco.mj_resetData(model, data)
    data.qpos[0:3] = [0.0, 0.0, 0.0]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    data.qpos[qpos_adr] = cfg['stand_joints']
    mujoco.mj_forward(model, data)

    site_err = 0.0
    for index, site_name in enumerate(foot_sites):
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        pos = data.site_xpos[site_id]
        err = float(np.linalg.norm(pos - cfg['stand_feet'][index]))
        site_err = max(site_err, err)
        failures += result(
            f'{site_name} vs CHAMP foot_from_base',
            err < 1e-4,
            f'mj={pos} champ={cfg["stand_feet"][index]} err={err:.6f} m',
        )
    failures += result('all foot sites match CHAMP standing FK', site_err < 1e-4, f'max err={site_err:.6f} m')

    foot_radius = float(model.geom(foot_geoms[0]).size[0])
    spawn_z = float(model.body(cfg['base_body']).pos[2])
    expected_settle_z = cfg['nominal_height'] + foot_radius
    failures += result(
        'spawn is above standing contact height',
        spawn_z > expected_settle_z,
        f'spawn={spawn_z:.3f} m  stand+radius={expected_settle_z:.3f} m  radius={foot_radius:.3f} m',
    )

    # Hold CHAMP stance with position actuators and drop onto the floor.
    mujoco.mj_resetData(model, data)
    data.qpos[qpos_adr] = cfg['stand_joints']
    data.ctrl[act_id] = cfg['stand_joints']
    mujoco.mj_forward(model, data)

    floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
    foot_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name) for name in foot_geoms]

    def contacts() -> List[bool]:
        flags = [False, False, False, False]
        for i in range(int(data.ncon)):
            geom1 = int(data.contact[i].geom1)
            geom2 = int(data.contact[i].geom2)
            for index, foot_id in enumerate(foot_ids):
                if (geom1 == floor_id and geom2 == foot_id) or (geom2 == floor_id and geom1 == foot_id):
                    flags[index] = True
        return flags

    hold_steps = 4000 if robot_name == 'go2' else 2500
    for _ in range(hold_steps):
        data.ctrl[act_id] = cfg['stand_joints']
        mujoco.mj_step(model, data)

    base_z = float(data.qpos[2])
    quat = data.qpos[3:7].copy()
    tilt = 2.0 * math.acos(max(-1.0, min(1.0, float(quat[0]))))
    stood = contacts()
    settle_tol = 0.03 if robot_name == 'go2' else 0.02
    failures += result(
        f'held stance settles near {cfg["nominal_height"]:.2f} m + foot radius',
        abs(base_z - expected_settle_z) < settle_tol,
        f'base_z={base_z:.4f} m expected {expected_settle_z:.4f} m tilt={math.degrees(tilt):.2f} deg',
    )
    failures += result(
        'held stance stays upright',
        tilt < math.radians(15.0),
        f'tilt={math.degrees(tilt):.2f} deg',
    )
    failures += result('all four feet contact the floor while standing', all(stood), str(stood))

    gravity = model.opt.gravity.astype(np.float64)
    lin_acc_world = data.qacc[0:3].copy()
    specific_force = world_to_body(quat, lin_acc_world - gravity)
    # At rest, accelerometer specific force is +g in body Z (~9.81).
    failures += result(
        'IMU-style specific force at rest is +Z gravity',
        abs(float(specific_force[2]) - 9.81) < 1.5 and abs(float(specific_force[0])) < 1.5 and abs(float(specific_force[1])) < 1.5,
        f'sf={specific_force} (expect ~[0,0,9.81])',
    )

    imu_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, cfg['imu_body'])
    imu_offset = model.body_pos[imu_body].copy()
    failures += result(
        f'{cfg["imu_body"]} offset matches URDF xyz',
        np.allclose(imu_offset, cfg['imu_offset'], atol=1e-9),
        str(imu_offset),
    )

    print(f'\nResult: {failures} failed checks')
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())
