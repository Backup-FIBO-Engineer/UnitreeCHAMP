from xbox_one_s_teleop.mapping import (
    apply_deadzone,
    axis_at,
    integrate_body_pose,
    locomotion_twist,
    quaternion_from_rpy,
    rising_edge,
)


def test_nose_up_0_10_matches_readme_quaternion():
    x, y, z, w = quaternion_from_rpy(0.0, -0.10, 0.0)
    assert abs(x) < 1e-9
    assert abs(z) < 1e-9
    assert abs(y - (-0.04997916667)) < 1e-5
    assert abs(w - 0.99875026039) < 1e-5


def test_identity_is_w1():
    x, y, z, w = quaternion_from_rpy(0.0, 0.0, 0.0)
    assert (x, y, z, w) == (0.0, 0.0, 0.0, 1.0)


def test_deadzone_and_scale():
    assert apply_deadzone(0.05, 0.15) == 0.0
    assert apply_deadzone(1.0, 0.15) == 1.0
    assert apply_deadzone(-1.0, 0.15) == -1.0


def test_linux_stick_up_is_negative_becomes_plus_vx():
    # axes[1] = -1.0 (stick up), invert_vx True → +vx
    vx, vy, wz = locomotion_twist(
        [-0.2, -1.0, 0.0, 0.5, 0.0],
        vx_axis=1,
        vy_axis=0,
        yaw_axis=3,
        invert_vx=True,
        invert_vy=False,
        invert_yaw=False,
        deadzone=0.15,
        max_linear_x=0.50,
        max_linear_y=0.15,
        max_angular_z=0.6,
    )
    assert vx == 0.50
    assert abs(vy + 0.15 * ((0.2 - 0.15) / 0.85)) < 1e-9
    assert abs(wz - 0.6 * ((0.5 - 0.15) / 0.85)) < 1e-9


def test_body_pose_stick_up_is_nose_up_negative_pitch():
    roll, pitch, moved = integrate_body_pose(
        0.0, 0.0, [0.0, 0.0, 0.0, 0.0, -1.0],
        roll_axis=3,
        pitch_axis=4,
        invert_roll=False,
            invert_pitch=False,
        deadzone=0.15,
        roll_rate=0.4,
        pitch_rate=0.4,
        max_roll=0.25,
        max_pitch=0.20,
        dt=0.25,
    )
    assert moved
    assert abs(roll) < 1e-12
    assert abs(pitch - (-0.10)) < 1e-9


def test_body_pose_clamped_to_yaml_limit():
    roll, pitch, _ = integrate_body_pose(
        0.0, -0.19, [0.0, 0.0, 0.0, 0.0, -1.0],
        roll_axis=3,
        pitch_axis=4,
        invert_roll=False,
            invert_pitch=False,
        deadzone=0.0,
        roll_rate=1.0,
        pitch_rate=1.0,
        max_roll=0.25,
        max_pitch=0.20,
        dt=1.0,
    )
    assert pitch == -0.20


def test_rising_edge_mode_toggle():
    assert rising_edge([0, 0, 0, 0, 1], [0, 0, 0, 0, 0], 4)
    assert not rising_edge([0, 0, 0, 0, 1], [0, 0, 0, 0, 1], 4)
    assert not rising_edge([0, 0, 0, 0, 0], [0, 0, 0, 0, 1], 4)


def test_axis_missing_is_zero():
    assert axis_at([], 4, False, 0.1) == 0.0
