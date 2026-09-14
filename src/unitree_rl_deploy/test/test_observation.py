import math

import numpy as np
import pytest

from unitree_rl_deploy.observation import (
    ObservationConfig,
    ObservationHistory,
    RobotObservation,
    pack_observation,
    projected_gravity_xyzw,
    quat_rotate_inverse_xyzw,
)


def test_identity_quaternion_projects_gravity_down():
    gravity = projected_gravity_xyzw((0.0, 0.0, 0.0, 1.0))
    np.testing.assert_allclose(gravity, [0.0, 0.0, -1.0], atol=1e-6)


def test_roll_90_deg_projects_gravity_along_minus_y():
    # +90 deg about X: body Y points world-up, gravity in body is -Y.
    s = math.sin(math.pi / 4.0)
    c = math.cos(math.pi / 4.0)
    gravity = projected_gravity_xyzw((s, 0.0, 0.0, c))
    np.testing.assert_allclose(gravity, [0.0, -1.0, 0.0], atol=1e-5)


def test_rotate_inverse_matches_identity():
    vector = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    np.testing.assert_allclose(
        quat_rotate_inverse_xyzw((0.0, 0.0, 0.0, 1.0), vector), vector, atol=1e-6)


def _cfg(**overrides):
    data = dict(
        terms=['ang_vel', 'gravity', 'command', 'dof_pos', 'dof_vel', 'action'],
        num_dofs=12,
        clip=100.0,
        history=1,
    )
    data.update(overrides)
    return ObservationConfig(**data)


def test_default_layout_is_45():
    assert _cfg().single_size == 45
    assert _cfg().size == 45


def test_clock_and_lin_vel_change_size():
    cfg = _cfg(terms=['ang_vel', 'gravity', 'command', 'lin_vel', 'dof_pos', 'dof_vel', 'action', 'clock'])
    assert cfg.single_size == 50


def test_history_stacks_oldest_first():
    cfg = _cfg(history=3, terms=['command'], num_dofs=12)
    hist = ObservationHistory(cfg)
    sample = RobotObservation(
        ang_vel=np.zeros(3), gravity=np.array([0, 0, -1], np.float32),
        command=np.array([1.0, 0.0, 0.0], np.float32),
        dof_pos=np.zeros(12), dof_vel=np.zeros(12), last_action=np.zeros(12),
    )
    first = hist.push(pack_observation(cfg, sample))
    assert first.size == 9
    sample.command = np.array([2.0, 0.0, 0.0], np.float32)
    second = hist.push(pack_observation(cfg, sample))
    sample.command = np.array([3.0, 0.0, 0.0], np.float32)
    third = hist.push(pack_observation(cfg, sample))
    # cmd_scale default [2, 2, 0.25] so vx obs is 2 * cmd
    assert third.reshape(3, 3)[0, 0] == pytest.approx(2.0)
    assert third.reshape(3, 3)[1, 0] == pytest.approx(4.0)
    assert third.reshape(3, 3)[2, 0] == pytest.approx(6.0)


def test_clip_obs():
    cfg = _cfg(clip=5.0, terms=['command'], num_dofs=12, cmd_scale=[1.0, 1.0, 1.0])
    hist = ObservationHistory(cfg)
    sample = RobotObservation(
        ang_vel=np.zeros(3), gravity=np.zeros(3),
        command=np.array([100.0, 0.0, 0.0], np.float32),
        dof_pos=np.zeros(12), dof_vel=np.zeros(12), last_action=np.zeros(12),
    )
    out = hist.push(pack_observation(cfg, sample))
    assert out[0] == pytest.approx(5.0)


def test_unknown_term_rejected():
    with pytest.raises(ValueError, match='unknown observation'):
        _cfg(terms=['ang_vel', 'foo'])
