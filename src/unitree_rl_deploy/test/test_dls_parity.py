"""Golden checks against iit-DLSLab Flat / Rough-Blind observation packing."""
from pathlib import Path

import numpy as np
import pytest

from unitree_rl_deploy.config import DeployConfig
from unitree_rl_deploy.controller import PolicyController, SensorSample
from unitree_rl_deploy.gait_clock import GaitClock
from unitree_rl_deploy.observation import (
    DLS_BLIND_TERMS,
    ObservationConfig,
    ObservationHistory,
    RobotObservation,
    pack_observation,
)

PKG = Path(__file__).resolve().parents[1]

DLS_JOINTS = [
    'FL_hip_joint', 'FR_hip_joint', 'RL_hip_joint', 'RR_hip_joint',
    'FL_thigh_joint', 'FR_thigh_joint', 'RL_thigh_joint', 'RR_thigh_joint',
    'FL_calf_joint', 'FR_calf_joint', 'RL_calf_joint', 'RR_calf_joint',
]

DLS_DEFAULTS = np.array(
    [0.0, 0.0, 0.0, 0.0, 0.9, 0.9, 0.9, 0.9, -1.8, -1.8, -1.8, -1.8],
    dtype=np.float32,
)


def _dls_obs_cfg(**overrides):
    data = dict(
        terms=list(DLS_BLIND_TERMS),
        num_dofs=12,
        clip=100.0,
        history=5,
        clock_size=4,
        lin_vel_scale=1.0,
        ang_vel_scale=1.0,
        dof_pos_scale=1.0,
        dof_vel_scale=1.0,
        cmd_scale=[1.0, 1.0, 1.0],
    )
    data.update(overrides)
    return ObservationConfig(**data)


def _dls_concatenate(lin_vel, ang_vel, gravity, command, dof_pos, dof_vel, last_action, clock):
    """Same cat order as LocomotionEnv._get_observations (use_imu=False)."""
    return np.concatenate([
        np.asarray(lin_vel, dtype=np.float32).reshape(3),
        np.asarray(ang_vel, dtype=np.float32).reshape(3),
        np.asarray(gravity, dtype=np.float32).reshape(3),
        np.asarray(command, dtype=np.float32).reshape(3),
        np.asarray(dof_pos, dtype=np.float32).reshape(12),
        np.asarray(dof_vel, dtype=np.float32).reshape(12),
        np.asarray(last_action, dtype=np.float32).reshape(12),
        np.asarray(clock, dtype=np.float32).reshape(4),
    ])


def test_dls_single_frame_matches_train_concatenate():
    cfg = _dls_obs_cfg(history=1)
    lin_vel = np.array([0.31, -0.04, 0.02], np.float32)
    ang_vel = np.array([0.1, -0.2, 0.3], np.float32)
    gravity = np.array([0.05, -0.02, -0.99], np.float32)
    command = np.array([0.4, -0.1, 0.2], np.float32)
    dof_pos = np.linspace(-0.2, 0.2, 12, dtype=np.float32)
    dof_vel = np.linspace(-1.0, 1.0, 12, dtype=np.float32)
    last_action = np.linspace(-0.5, 0.5, 12, dtype=np.float32)
    clock = np.array([0.1, 0.6, 0.6, 0.1], np.float32)
    packed = pack_observation(cfg, RobotObservation(
        ang_vel=ang_vel, gravity=gravity, command=command, dof_pos=dof_pos,
        dof_vel=dof_vel, last_action=last_action, lin_vel=lin_vel, clock=clock,
    ))
    expected = _dls_concatenate(
        lin_vel, ang_vel, gravity, command, dof_pos, dof_vel, last_action, clock)
    assert packed.size == 52
    np.testing.assert_allclose(packed, expected, atol=1e-6)
    np.testing.assert_allclose(packed[:3], lin_vel)
    np.testing.assert_allclose(packed[9:12], command)
    np.testing.assert_allclose(packed[48:52], clock)


def test_dls_wrapper_hardcoded_still_clock_slots():
    cfg = _dls_obs_cfg(history=1)
    sample = RobotObservation(
        ang_vel=np.zeros(3), gravity=np.array([0, 0, -1], np.float32),
        command=np.zeros(3), dof_pos=np.zeros(12), dof_vel=np.zeros(12),
        last_action=np.zeros(12), lin_vel=np.zeros(3),
        clock=np.full(4, -1.0, np.float32),
    )
    packed = pack_observation(cfg, sample)
    np.testing.assert_allclose(packed[48:52], -1.0)


def test_dls_history_oldest_first_newest_last():
    cfg = _dls_obs_cfg()
    hist = ObservationHistory(cfg)
    frames = []
    for i in range(5):
        clock = np.full(4, float(i), np.float32)
        frame = pack_observation(cfg, RobotObservation(
            ang_vel=np.zeros(3), gravity=np.array([0, 0, -1], np.float32),
            command=np.array([float(i), 0.0, 0.0], np.float32),
            dof_pos=np.zeros(12), dof_vel=np.zeros(12), last_action=np.zeros(12),
            lin_vel=np.zeros(3), clock=clock,
        ))
        stacked = hist.push(frame)
        frames.append(frame)
    stacked = stacked.reshape(5, 52)
    for i in range(5):
        np.testing.assert_allclose(stacked[i], frames[i])
        assert stacked[i, 9] == pytest.approx(float(i))


def test_per_leg_joint_state_remaps_into_hip_group_order():
    cfg = DeployConfig.from_yaml(PKG / 'config' / 'go2_rl.yaml')
    name_to_index = {name: i for i, name in enumerate(cfg.joint_names)}
    per_leg = [
        'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
        'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
        'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
        'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
    ]
    measured = np.arange(12, dtype=np.float32)
    q = np.full(12, np.nan, dtype=np.float32)
    for i, name in enumerate(per_leg):
        q[name_to_index[name]] = measured[i]
    np.testing.assert_allclose(q, [
        0, 3, 6, 9,
        1, 4, 7, 10,
        2, 5, 8, 11,
    ])


def test_action_index_maps_to_dls_legs():
    """Policy action i → FL=[0,4,8], FR=[1,5,9], RL=[2,6,10], RR=[3,7,11]."""
    cfg = DeployConfig.from_yaml(PKG / 'config' / 'go2_rl.yaml')
    assert cfg.joint_names == DLS_JOINTS
    fl = [cfg.joint_names.index(n) for n in ('FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint')]
    fr = [cfg.joint_names.index(n) for n in ('FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint')]
    rl = [cfg.joint_names.index(n) for n in ('RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint')]
    rr = [cfg.joint_names.index(n) for n in ('RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint')]
    assert fl == [0, 4, 8]
    assert fr == [1, 5, 9]
    assert rl == [2, 6, 10]
    assert rr == [3, 7, 11]


def test_controller_260d_and_unfiltered_last_action_with_target_filter():
    cfg = DeployConfig.from_yaml(PKG / 'config' / 'go2_rl.yaml')
    assert cfg.num_obs == 260
    seen = {}

    def policy(obs):
        seen['obs'] = np.asarray(obs, dtype=np.float32).copy()
        return np.full(12, 2.0, dtype=np.float32)

    loop = PolicyController(cfg, policy)
    sample = SensorSample(
        q=np.asarray(cfg.default_angles, dtype=np.float32),
        dq=np.zeros(12, dtype=np.float32),
        quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
        ang_vel=np.zeros(3, dtype=np.float32),
        cmd_vx=0.5,
        lin_vel=np.array([0.2, 0.0, 0.0], dtype=np.float32),
        orientation_valid=True,
    )
    targets = loop.targets(sample)
    assert seen['obs'].size == 260
    # Newest frame is the last 52. last_action was zeros on the first tick.
    np.testing.assert_allclose(seen['obs'][-12 - 4:-4], 0.0)
    # Unfiltered clipped action stored for the next obs.
    np.testing.assert_allclose(loop.last_action, 2.0)
    # Filter only on the joint command: 0.8*2 + 0.2*0 = 1.6, scale 0.5 → +0.8
    np.testing.assert_allclose(targets, cfg.default_angles + 0.8, atol=1e-5)

    loop.targets(sample)
    np.testing.assert_allclose(seen['obs'][-12 - 4:-4], 2.0, atol=1e-5)


def test_controller_still_clock_is_minus_one():
    cfg = DeployConfig.from_yaml(PKG / 'config' / 'b2_rl.yaml')
    seen = {}

    def policy(obs):
        seen['obs'] = np.asarray(obs, dtype=np.float32).copy()
        return np.zeros(12, dtype=np.float32)

    loop = PolicyController(cfg, policy)
    sample = SensorSample(
        q=np.asarray(cfg.default_angles, dtype=np.float32),
        dq=np.zeros(12, dtype=np.float32),
        quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
        ang_vel=np.zeros(3, dtype=np.float32),
        lin_vel=np.zeros(3, dtype=np.float32),
        orientation_valid=True,
    )
    loop.targets(sample)
    np.testing.assert_allclose(seen['obs'][-4:], -1.0)


def test_controller_applies_com_lever_to_lin_vel():
    cfg = DeployConfig.from_yaml(PKG / 'config' / 'go2_rl.yaml')
    seen = {}

    def policy(obs):
        seen['obs'] = np.asarray(obs, dtype=np.float32).copy()
        return np.zeros(12, dtype=np.float32)

    loop = PolicyController(cfg, policy)
    omega = np.array([0.0, 0.0, 1.0], dtype=np.float32)
    v_link = np.array([0.4, 0.0, 0.0], dtype=np.float32)
    sample = SensorSample(
        q=np.asarray(cfg.default_angles, dtype=np.float32),
        dq=np.zeros(12, dtype=np.float32),
        quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
        ang_vel=omega,
        cmd_vx=0.4,
        lin_vel=v_link,
        orientation_valid=True,
    )
    loop.targets(sample)
    expected = v_link + np.cross(omega, cfg.lin_vel_com_offset)
    np.testing.assert_allclose(seen['obs'][-52:-49], expected, atol=1e-5)


def test_gait_clock_in_controller_matches_standalone():
    cfg = DeployConfig.from_yaml(PKG / 'config' / 'go2_rl.yaml')
    clock = GaitClock(
        offsets=cfg.gait_phase_offsets,
        freq_min=cfg.gait_step_freq_min,
        freq_max=cfg.gait_step_freq_max,
        speed_min=cfg.gait_speed_min,
        speed_max=cfg.gait_speed_max,
        still_threshold=cfg.gait_still_command_threshold,
        dt=1.0 / cfg.control_rate,
    )
    seen = {}

    def policy(obs):
        seen['obs'] = np.asarray(obs, dtype=np.float32).copy()
        return np.zeros(12, dtype=np.float32)

    loop = PolicyController(cfg, policy)
    sample = SensorSample(
        q=np.asarray(cfg.default_angles, dtype=np.float32),
        dq=np.zeros(12, dtype=np.float32),
        quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
        ang_vel=np.zeros(3, dtype=np.float32),
        cmd_vx=0.6,
        lin_vel=np.zeros(3, dtype=np.float32),
        orientation_valid=True,
    )
    loop.targets(sample)
    expected = clock.step((0.6, 0.0, 0.0))
    np.testing.assert_allclose(seen['obs'][-4:], expected, atol=1e-6)


def test_clip_actions_then_filter():
    cfg = DeployConfig.from_yaml(PKG / 'config' / 'go2_rl.yaml')
    loop = PolicyController(cfg, lambda obs: np.full(12, 10.0, dtype=np.float32))
    sample = SensorSample(
        q=np.asarray(cfg.default_angles, dtype=np.float32),
        dq=np.zeros(12, dtype=np.float32),
        quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
        ang_vel=np.zeros(3, dtype=np.float32),
        cmd_vx=0.3,
        lin_vel=np.zeros(3, dtype=np.float32),
        orientation_valid=True,
    )
    targets = loop.targets(sample)
    np.testing.assert_allclose(loop.last_action, 3.0)
    np.testing.assert_allclose(targets, cfg.default_angles + 0.8 * 3.0 * 0.5, atol=1e-5)
