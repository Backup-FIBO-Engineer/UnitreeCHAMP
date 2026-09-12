#!/usr/bin/env python3
"""Static checker: robot URDF versus its MuJoCo MJCF, for any robot in this package.

    validate_mujoco_against_urdf.py --robot <robot> [--xml path]

The MJCF is compiled with MuJoCo and compared body by body against the URDF:
  * every URDF link on the CHAMP leg chains (links_map) and the base/IMU links
    exist as bodies, their kinematic offsets at q=0 match the URDF joint
    origins (sum of xyz, the same quantity CHAMP uses), joint axes and ranges
    match, actuators are position servos with ctrlrange = URDF limits and
    forcerange = URDF effort;
  * explicit inertials (mass, COM, principal inertia) match the URDF for every
    link that has one in both files;
  * URDF collision primitives (box/cylinder/sphere) that exist as geoms named
    <link>_collision[_n] match in type, size and pose; mesh collisions are
    reported and skipped;
  * URDF visual meshes emitted as <link>_visual[_n][_part] geoms are meshes on
    the right body at the URDF visual origin, never collide (contype =
    conaffinity = 0) and sit in group 1 while collision primitives are in group 3.
Requires the `mujoco` Python package. Does not need ROS.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Dict

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from champ_robot_files import (  # noqa: E402
    CHAIN_LENGTH, LEGS, SOURCE_PACKAGE_DIR, Urdf, champ_joint_names, foot_geom_names,
    foot_site_names, inertia_matrix, leg_chains, load_ros_params, load_urdf,
    resolve_robot_files, rpy_to_matrix,
)


def result(label: str, ok: bool, detail: str = '') -> int:
    print(f"[{'PASS' if ok else 'FAIL'}] {label}" + (f': {detail}' if detail else ''))
    return 0 if ok else 1


def close(a, b, tol: float = 1e-9) -> bool:
    return bool(np.allclose(np.asarray(a, dtype=float), np.asarray(b, dtype=float), atol=tol, rtol=0.0))


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def urdf_world_pose(urdf: Urdf, link: str):
    """Position/rotation of a link frame in the root frame at q=0 (fixed + hinge joints)."""
    pos = np.zeros(3)
    rot = np.eye(3)
    chain = []
    current = link
    while current != urdf.root:
        joint = urdf.parent_joint[current]
        chain.append(joint)
        current = joint.parent
    for joint in reversed(chain):
        pos = pos + rot @ joint.xyz
        rot = rot @ rpy_to_matrix(joint.rpy)
    return pos, rot


def main() -> int:
    try:
        import mujoco
    except ImportError:
        print('SKIP: mujoco Python package is not installed')
        return 0

    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', required=True)
    parser.add_argument('--package-dir', type=Path, default=SOURCE_PACKAGE_DIR)
    parser.add_argument('--xml', type=Path, default=None)
    args = parser.parse_args()

    files = resolve_robot_files(args.package_dir, args.robot)
    xml_path = args.xml or files.mujoco_xml
    if xml_path is None:
        print(f'FAIL: robot {args.robot!r} has no mujoco/{args.robot}.xml')
        return 1
    params = load_ros_params(*files.champ_yamls())
    sim = files.sim_params()
    urdf = load_urdf(files.urdf)
    links = leg_chains(params, 'links_map')
    joints_map = leg_chains(params, 'joints_map')
    joint_names = champ_joint_names(params)
    base_link = str(params['links_map']['base'])
    imu_link = str(params['links_map']['imu'])

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    root_jnt = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'root')
    root_q = int(model.jnt_qposadr[root_jnt])
    data.qpos[:] = 0.0
    data.qpos[root_q + 3] = 1.0
    mujoco.mj_forward(model, data)
    failures = 0

    def body_id(name: str) -> int:
        return int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name))

    failures += result('links_map.base is the URDF root', base_link == urdf.root, f'{base_link} vs {urdf.root}')
    failures += result('base body carries the free joint', body_id(base_link) == int(model.jnt_bodyid[root_jnt]))

    # -- leg chains: bodies, joints, axes, ranges ---------------------------------------
    # Foot links are fixed to the lower leg; a hand-written MJCF may fold them into the
    # lower-leg body (only the foot site/geom is required, checked below).
    required_links = [base_link, imu_link] + [name for leg in LEGS for name in links[leg][:CHAIN_LENGTH - 1]]
    for link in required_links:
        failures += result(f'body {link} exists', body_id(link) >= 0)
    if any(body_id(l) < 0 for l in required_links):
        print(f'\nResult: {failures} failed checks')
        return 1
    compared_links = list(required_links)
    for leg in LEGS:
        foot = links[leg][CHAIN_LENGTH - 1]
        if body_id(foot) >= 0:
            compared_links.append(foot)
        else:
            print(f'[INFO] foot link {foot} has no MJCF body (folded into {links[leg][CHAIN_LENGTH - 2]})')

    for link in compared_links:
        if link == base_link:
            continue
        urdf_pos, urdf_rot = urdf_world_pose(urdf, link)
        bid = body_id(link)
        mj_pos = data.xpos[bid] - data.xpos[body_id(base_link)]
        mj_rot = data.xmat[bid].reshape(3, 3)
        failures += result(f'{link} frame origin at q=0', close(urdf_pos, mj_pos, 1e-9), f'urdf={urdf_pos} mjcf={mj_pos}')
        failures += result(f'{link} frame rotation at q=0', close(urdf_rot, mj_rot, 1e-9))
        # Direct kinematic parent (skipping nothing): MuJoCo bodies map 1:1 to URDF links.
        urdf_parent = urdf.parent_joint[link].parent
        mj_parent = model.body(int(model.body_parentid[bid])).name
        failures += result(f'{link} parent body', mj_parent == urdf_parent, f'urdf={urdf_parent} mjcf={mj_parent}')

    for leg in LEGS:
        for i in range(CHAIN_LENGTH):
            jname = joints_map[leg][i]
            uj = urdf.joints[jname]
            failures += result(f'{jname} drives {links[leg][i]} in the URDF', uj.child == links[leg][i])
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if i < CHAIN_LENGTH - 1:
                failures += result(f'{jname} hinge exists', jid >= 0 and model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_HINGE)
                if jid < 0:
                    continue
                failures += result(f'{jname} on body {uj.child}', model.body(int(model.jnt_bodyid[jid])).name == uj.child)
                failures += result(f'{jname} axis', close(model.jnt_axis[jid], uj.axis, 1e-12), f"urdf={uj.axis} mjcf={model.jnt_axis[jid]}")
                failures += result(f'{jname} anchored at body origin', close(model.jnt_pos[jid], [0, 0, 0], 1e-12))
                if 'lower' in uj.limit and 'upper' in uj.limit:
                    expected = np.array([float(uj.limit['lower']), float(uj.limit['upper'])])
                    failures += result(f'{jname} range', bool(model.jnt_limited[jid]) and close(expected, model.jnt_range[jid], 1e-12), f'urdf={expected} mjcf={model.jnt_range[jid]}')
            else:
                failures += result(f'{jname} is fixed (no MuJoCo joint)', uj.type == 'fixed' and jid < 0)

    # -- actuators -----------------------------------------------------------------------
    failures += result('12 actuators', model.nu == 12, f'nu={model.nu}')
    for jname in joint_names:
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, jname)
        if aid < 0:
            failures += result(f'actuator {jname} exists', False)
            continue
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        failures += result(f'actuator {jname} drives joint {jname}', model.actuator_trntype[aid] == mujoco.mjtTrn.mjTRN_JOINT and int(model.actuator_trnid[aid, 0]) == jid)
        kp = float(model.actuator_gainprm[aid, 0])
        failures += result(f'actuator {jname} is a position servo', kp > 0 and math.isclose(model.actuator_biasprm[aid, 1], -kp, rel_tol=1e-9) and model.actuator_biasprm[aid, 2] <= 0.0, f'kp={kp} bias={model.actuator_biasprm[aid, :3]}')
        uj = urdf.joints[jname]
        if 'lower' in uj.limit and 'upper' in uj.limit:
            expected = np.array([float(uj.limit['lower']), float(uj.limit['upper'])])
            failures += result(f'actuator {jname} ctrlrange = URDF limits', bool(model.actuator_ctrllimited[aid]) and close(expected, model.actuator_ctrlrange[aid], 1e-12), f'{model.actuator_ctrlrange[aid]}')
        if 'effort' in uj.limit:
            effort = float(uj.limit['effort'])
            failures += result(f'actuator {jname} forcerange = URDF effort', bool(model.actuator_forcelimited[aid]) and close([-effort, effort], model.actuator_forcerange[aid], 1e-9), f'{model.actuator_forcerange[aid]}')

    # -- inertials -----------------------------------------------------------------------
    compared = 0
    urdf_total = 0.0
    mj_total = 0.0
    for link in urdf.links.values():
        bid = body_id(link.name)
        if bid < 0 or link.mass is None or not link.inertia:
            continue
        if link.mass < 1e-6:
            continue
        compared += 1
        urdf_total += link.mass
        mj_total += float(model.body_mass[bid])
        failures += result(f'{link.name} mass', math.isclose(link.mass, float(model.body_mass[bid]), abs_tol=1e-12), f'urdf={link.mass:.15g} mjcf={float(model.body_mass[bid]):.15g}')
        com = np.array([float(v) for v in link.inertial_xyz_text.split()])
        failures += result(f'{link.name} COM', close(com, model.body_ipos[bid], 1e-12), f'urdf={com} mjcf={model.body_ipos[bid]}')
        tensor = inertia_matrix(link.inertia)
        eig = np.sort(np.linalg.eigvalsh(tensor))
        failures += result(f'{link.name} principal inertia', close(eig, np.sort(model.body_inertia[bid]), 1e-12), f'urdf={eig} mjcf={np.sort(model.body_inertia[bid])}')
        # Rebuild the full tensor in the body frame from MuJoCo's principal frame.
        r = quat_to_matrix(model.body_iquat[bid])
        rebuilt = r @ np.diag(model.body_inertia[bid]) @ r.T
        rot_urdf = rpy_to_matrix([float(v) for v in link.inertial_rpy_text.split()])
        # MuJoCo stores the principal frame as a quaternion from its own eigen
        # decomposition; rebuilding the tensor is accurate to ~1e-6 relative.
        tensor_tol = 1e-6 * float(np.abs(tensor).max())
        failures += result(f'{link.name} inertia tensor', close(rot_urdf @ tensor @ rot_urdf.T, rebuilt, tensor_tol))
        failures += result(f'{link.name} inertia positive definite', bool(np.all(eig > 0)) and eig[0] + eig[1] >= eig[2] - 1e-12, str(eig))
    failures += result('inertials compared for every massive URDF link present in MJCF', compared > 0, f'{compared} links')
    failures += result('explicit mass total', math.isclose(urdf_total, mj_total, abs_tol=1e-9), f'urdf={urdf_total:.9f} kg mjcf={mj_total:.9f} kg')

    # -- collision primitives --------------------------------------------------------------
    size_map = {
        'box': lambda a: np.array([float(v) / 2 for v in a['size'].split()]),
        'cylinder': lambda a: np.array([float(a['radius']), float(a['length']) / 2, 0.0]),
        'sphere': lambda a: np.array([float(a['radius']), 0.0, 0.0]),
    }
    type_map = {'box': mujoco.mjtGeom.mjGEOM_BOX, 'cylinder': mujoco.mjtGeom.mjGEOM_CYLINDER, 'sphere': mujoco.mjtGeom.mjGEOM_SPHERE}
    primitives = 0
    meshes = 0
    for link in urdf.links.values():
        if body_id(link.name) < 0:
            continue
        for index, col in enumerate(link.collisions):
            if col.geometry not in size_map:
                meshes += 1
                continue
            name = f'{link.name}_collision' if index == 0 else f'{link.name}_collision_{index}'
            gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
            if gid < 0:
                # Hand-written models may name geoms differently; only report.
                print(f'[INFO] {name}: URDF {col.geometry} collision has no geom of that name')
                continue
            primitives += 1
            failures += result(f'{name} type', model.geom_type[gid] == type_map[col.geometry])
            expected_size = size_map[col.geometry](col.attrib)
            n = {'box': 3, 'cylinder': 2, 'sphere': 1}[col.geometry]
            failures += result(f'{name} size', close(expected_size[:n], model.geom_size[gid][:n], 1e-12), f'urdf={expected_size[:n]} mjcf={model.geom_size[gid][:n]}')
            failures += result(f'{name} pos', close(col.xyz, model.geom_pos[gid], 1e-12), f'urdf={col.xyz} mjcf={model.geom_pos[gid]}')
            if col.geometry != 'sphere':
                failures += result(f'{name} orientation', close(rpy_to_matrix(col.rpy), quat_to_matrix(model.geom_quat[gid]), 1e-9))
            failures += result(f'{name} collides with the floor', int(model.geom_contype[gid]) & int(model.geom_conaffinity[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")]) != 0)
    print(f'[INFO] {primitives} URDF collision primitives compared, {meshes} mesh collisions skipped')

    # -- visual meshes ----------------------------------------------------------------------
    # generate_mjcf.py names them <link>_visual[_n][_part]; they must never take part in
    # contacts or carry mass (inertials are explicit), only be drawn.
    visual_ids = [g for g in range(model.ngeom) if '_visual' in model.geom(g).name]
    collision_ids = [g for g in range(model.ngeom) if '_collision' in model.geom(g).name]
    if visual_ids:
        failures += result('visual geoms have contype = conaffinity = 0',
                           all(int(model.geom_contype[g]) == 0 and int(model.geom_conaffinity[g]) == 0 for g in visual_ids),
                           f'{len(visual_ids)} geoms')
        failures += result('visual geoms are drawn in group 1, collision primitives hidden in group 3',
                           all(int(model.geom_group[g]) == 1 for g in visual_ids)
                           and all(int(model.geom_group[g]) == 3 for g in collision_ids))
        for link in urdf.links.values():
            if body_id(link.name) < 0:
                continue
            for index, vis in enumerate(link.visuals):
                if vis.geometry != 'mesh':
                    continue
                base_name = f'{link.name}_visual' if index == 0 else f'{link.name}_visual_{index}'
                gids = [g for g in visual_ids if model.geom(g).name == base_name
                        or model.geom(g).name.startswith(base_name + '_')]
                failures += result(f'{base_name} mesh geom(s) exist', len(gids) > 0, vis.attrib.get('filename', ''))
                for g in gids:
                    failures += result(f'{model.geom(g).name} is a mesh on body {link.name}',
                                       model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH
                                       and model.body(int(model.geom_bodyid[g])).name == link.name)
                    # MuJoCo stores mesh geoms in the mesh's centroid/principal frame:
                    # geom_quat = q_user * mesh_quat, geom_pos = pos_user + R_user @ mesh_pos.
                    mid = int(model.geom_dataid[g])
                    r_user = quat_to_matrix(model.geom_quat[g]) @ quat_to_matrix(model.mesh_quat[mid]).T
                    pos_user = model.geom_pos[g] - r_user @ model.mesh_pos[mid]
                    xyz = np.array([float(v) for v in vis.xyz_text.split()])
                    failures += result(f'{model.geom(g).name} pos', close(xyz, pos_user, 1e-9),
                                       f'urdf={xyz} mjcf={pos_user}')
                    failures += result(f'{model.geom(g).name} orientation',
                                       close(rpy_to_matrix([float(v) for v in vis.rpy_text.split()]), r_user, 1e-9))
        print(f'[INFO] {len(visual_ids)} visual mesh geoms checked')
    else:
        print('[INFO] no visual mesh geoms in the MJCF (primitives only)')

    # -- feet ------------------------------------------------------------------------------
    for leg, geom, site in zip(LEGS, foot_geom_names(params, sim), foot_site_names(params, sim)):
        foot_link = links[leg][CHAIN_LENGTH - 1]
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom)
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site)
        failures += result(f'{leg} foot geom {geom} exists', gid >= 0)
        failures += result(f'{leg} foot site {site} exists', sid >= 0)
        if sid >= 0:
            urdf_pos, _ = urdf_world_pose(urdf, foot_link)
            mj_site = data.site_xpos[sid] - data.xpos[body_id(base_link)]
            failures += result(f'{site} at the URDF {foot_link} origin (CHAMP foot point)', close(urdf_pos, mj_site, 1e-9), f'urdf={urdf_pos} mjcf={mj_site}')
        if gid >= 0:
            failures += result(f'{geom} belongs to the {foot_link} chain', model.body(int(model.geom_bodyid[gid])).name in links[leg])

    print(f'\nResult: {failures} failed checks')
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())
