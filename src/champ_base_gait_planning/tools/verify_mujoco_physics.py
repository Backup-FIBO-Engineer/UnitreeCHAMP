#!/usr/bin/env python3
"""Physics checks of mujoco/<robot>.xml against the CHAMP standing pose and URDF FK.

    verify_mujoco_physics.py --robot <robot> [--xml path]

The standing joints/feet come from tools/dump_champ_gait.cpp (compiled on demand)
fed with the robot URDF + yaml, the IMU offset from the URDF chain, the joint
names from joints_map and the foot geoms/sites from links_map (or sim.* overrides).
Requires the `mujoco` Python package. Does not need ROS.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import List

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from champ_robot_files import (  # noqa: E402
    SOURCE_PACKAGE_DIR, champ_joint_names, dump_stand, foot_geom_names, foot_site_names,
    load_ros_params, load_urdf, resolve_robot_files,
)


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
    parser.add_argument('--robot', required=True)
    parser.add_argument('--package-dir', type=Path, default=SOURCE_PACKAGE_DIR)
    parser.add_argument('--xml', type=Path, default=None, help='default mujoco/<robot>.xml')
    args = parser.parse_args()

    files = resolve_robot_files(args.package_dir, args.robot)
    xml_path = args.xml or files.mujoco_xml
    if xml_path is None:
        print(f'FAIL: robot {args.robot!r} has no mujoco/{args.robot}.xml')
        return 1
    params = load_ros_params(*files.champ_yamls())
    sim = files.sim_params()
    urdf = load_urdf(files.urdf)
    joint_names = champ_joint_names(params)
    foot_geoms = foot_geom_names(params, sim)
    foot_sites = foot_site_names(params, sim)
    base_body = str(params['links_map']['base'])
    imu_body = str(params['links_map']['imu'])
    nominal_height = float(params['gait']['nominal_height'])
    stand_joints, stand_feet = dump_stand(files)
    print(f'{args.robot}: CHAMP stand joints {np.round(stand_joints, 4).tolist()}')

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    failures = 0

    failures += result('MuJoCo >= 3.1', tuple(int(x) for x in mujoco.__version__.split('.')[:2]) >= (3, 1), mujoco.__version__)
    failures += result('nq includes freejoint + 12 hinges', model.nq == 7 + 12, f'nq={model.nq}')
    failures += result('12 position actuators', model.nu == 12, f'nu={model.nu}')

    qpos_adr = np.array([model.joint(name).qposadr[0] for name in joint_names])
    act_id = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in joint_names])
    failures += result('every joints_map joint has an actuator of the same name', np.all(act_id >= 0), str(act_id.tolist()))
    root_qpos = int(model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'root')])
    failures += result('free joint "root" on the base', root_qpos == 0 and model.body(base_body).id == model.jnt_bodyid[0])

    ident = np.array([1.0, 0.0, 0.0, 0.0])
    vec = np.array([0.1, -0.2, 0.3])
    failures += result('world_to_body identity', np.allclose(world_to_body(ident, vec), vec))

    # CHAMP standing FK vs MuJoCo foot sites, base at origin so world==base.
    mujoco.mj_resetData(model, data)
    data.qpos[root_qpos:root_qpos + 3] = [0.0, 0.0, 0.0]
    data.qpos[root_qpos + 3:root_qpos + 7] = [1.0, 0.0, 0.0, 0.0]
    data.qpos[qpos_adr] = stand_joints
    mujoco.mj_forward(model, data)

    site_err = 0.0
    for index, site_name in enumerate(foot_sites):
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if site_id < 0:
            failures += result(f'{site_name} exists', False)
            continue
        pos = data.site_xpos[site_id]
        err = float(np.linalg.norm(pos - stand_feet[index]))
        site_err = max(site_err, err)
        failures += result(
            f'{site_name} vs CHAMP foot_from_base',
            err < 1e-4,
            f'mj={np.round(pos, 6)} champ={np.round(stand_feet[index], 6)} err={err:.6f} m',
        )
    failures += result('all foot sites match CHAMP standing FK', site_err < 1e-4, f'max err={site_err:.6f} m')

    foot_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name) for name in foot_geoms]
    failures += result('foot geoms exist', all(i >= 0 for i in foot_ids), str(dict(zip(foot_geoms, foot_ids))))
    if any(i < 0 for i in foot_ids):
        print(f'\nResult: {failures} failed checks')
        return 1
    foot_radius = float(model.geom_size[foot_ids[0]][0])
    # Lowest point of the first foot geom below the CHAMP foot point, at stand.
    foot_bottom = float(stand_feet[0][2] - (data.geom_xpos[foot_ids[0]][2] - foot_radius))
    spawn_z = float(model.body(base_body).pos[2])
    expected_settle_z = nominal_height + foot_bottom
    failures += result(
        'spawn is above standing contact height',
        spawn_z > expected_settle_z,
        f'spawn={spawn_z:.3f} m  stand+foot={expected_settle_z:.3f} m  radius={foot_radius:.3f} m',
    )

    # Hold CHAMP stance with position actuators and drop onto the floor.
    mujoco.mj_resetData(model, data)
    data.qpos[qpos_adr] = stand_joints
    data.ctrl[act_id] = stand_joints
    mujoco.mj_forward(model, data)

    floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'floor')

    def contacts() -> List[bool]:
        flags = [False, False, False, False]
        for i in range(int(data.ncon)):
            geom1 = int(data.contact[i].geom1)
            geom2 = int(data.contact[i].geom2)
            for index, foot_id in enumerate(foot_ids):
                if (geom1 == floor_id and geom2 == foot_id) or (geom2 == floor_id and geom1 == foot_id):
                    flags[index] = True
        return flags

    hold_steps = int(round(4.0 / float(model.opt.timestep)))
    for _ in range(hold_steps):
        data.ctrl[act_id] = stand_joints
        mujoco.mj_step(model, data)

    base_z = float(data.qpos[root_qpos + 2])
    quat = data.qpos[root_qpos + 3:root_qpos + 7].copy()
    tilt = 2.0 * math.acos(max(-1.0, min(1.0, float(quat[0]))))
    stood = contacts()
    settle_tol = max(0.02, 0.1 * nominal_height)
    failures += result(
        f'held stance settles near {nominal_height:.3f} m + foot',
        abs(base_z - expected_settle_z) < settle_tol,
        f'base_z={base_z:.4f} m expected {expected_settle_z:.4f} m (tol {settle_tol:.3f}) tilt={math.degrees(tilt):.2f} deg',
    )
    failures += result('held stance stays upright', tilt < math.radians(15.0), f'tilt={math.degrees(tilt):.2f} deg')
    failures += result('all four feet contact the floor while standing', all(stood), str(stood))

    gravity = model.opt.gravity.astype(np.float64)
    lin_acc_world = data.qacc[0:3].copy()
    specific_force = world_to_body(quat, lin_acc_world - gravity)
    failures += result(
        'IMU-style specific force at rest is +Z gravity',
        abs(float(specific_force[2]) - 9.81) < 1.5 and abs(float(specific_force[0])) < 1.5 and abs(float(specific_force[1])) < 1.5,
        f'sf={np.round(specific_force, 3)} (expect ~[0,0,9.81])',
    )

    imu_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, imu_body)
    failures += result(f'IMU body {imu_body!r} (links_map.imu) exists', imu_id >= 0)
    if imu_id >= 0:
        # mujoco_sim.py uses body_pos of the IMU body as lever arm, so it must be a direct
        # child of the base and sit at the URDF chain offset.
        imu_offset = model.body_pos[imu_id].copy()
        urdf_offset = urdf.chain_xyz(urdf.root, imu_body)
        failures += result(f'{imu_body} is a direct child of {base_body}', int(model.body_parentid[imu_id]) == model.body(base_body).id)
        failures += result(f'{imu_body} offset matches URDF xyz', np.allclose(imu_offset, urdf_offset, atol=1e-9), f'mj={imu_offset} urdf={urdf_offset}')

    print(f'\nResult: {failures} failed checks')
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())
