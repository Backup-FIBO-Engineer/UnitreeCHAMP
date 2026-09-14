import numpy as np
import pytest

from unitree_rl_deploy.gait_clock import GaitClock


def test_still_command_is_all_minus_one_but_phase_advances():
    clock = GaitClock(dt=0.02)
    first = clock.step((0.0, 0.0, 0.0))
    np.testing.assert_allclose(first, -1.0)
    phase_after_still = clock.phase.copy()
    assert np.any(np.abs(phase_after_still - clock.offsets) > 1e-9)
    moving = clock.step((0.5, 0.0, 0.0))
    np.testing.assert_allclose(moving, (phase_after_still + clock.step_freq * 0.02) % 1.0)


def test_phase_offsets_and_wrap():
    clock = GaitClock(offsets=(0.0, 0.5, 0.5, 0.0), freq_min=1.4, freq_max=1.4, dt=0.02)
    out = clock.step((0.5, 0.0, 0.0))
    expected = (np.array([0.0, 0.5, 0.5, 0.0]) + 1.4 * 0.02) % 1.0
    np.testing.assert_allclose(out, expected, atol=1e-6)


def test_freq_ramps_with_xy_speed():
    clock = GaitClock(
        freq_min=1.4, freq_max=1.8, speed_min=0.4, speed_max=0.8, dt=0.02)
    clock.step((0.0, 0.0, 0.2))
    assert clock.step_freq == pytest.approx(1.4)
    clock.step((0.4, 0.0, 0.0))
    assert clock.step_freq == pytest.approx(1.4)
    clock.step((0.6, 0.0, 0.0))
    assert clock.step_freq == pytest.approx(1.6)
    clock.step((0.8, 0.0, 0.0))
    assert clock.step_freq == pytest.approx(1.8)
    clock.step((1.2, 0.0, 0.0))
    assert clock.step_freq == pytest.approx(1.8)


def test_reset_restores_offsets():
    clock = GaitClock()
    clock.step((0.5, 0.0, 0.0))
    clock.reset()
    np.testing.assert_allclose(clock.phase, [0.0, 0.5, 0.5, 0.0])
