from xbox_one_s_teleop.mapping import (
    apply_deadzone,
    axis_at,
    integrate_body_pose,
    locomotion_twist,
    quaternion_from_rpy,
    rising_edge,
)

# Defaults match config/xbox_one_s_1708_bt.yaml (SDL2 joy_node).
_LOCO = dict(
    vx_axis=1,
    vy_axis=0,
    yaw_axis=2,
    invert_vx=True,
    invert_vy=True,
    invert_yaw=True,
    deadzone=0.15,
    max_linear_x=0.50,
    max_linear_y=0.15,
    max_angular_z=0.6,
)
_POSE = dict(
    roll_axis=2,
    pitch_axis=3,
    invert_roll=False,
    invert_pitch=False,
    deadzone=0.15,
    roll_rate=0.4,
    pitch_rate=0.4,
    max_roll=0.25,
    max_pitch=0.20,
)

# xpadneo + joy_node: Linux axis order, stick up / stick left are positive.
_LINUX = dict(
    vx_axis=1,
    vy_axis=0,
    yaw_axis=3,
    invert_vx=False,
    invert_vy=False,
    invert_yaw=False,
    deadzone=0.0,
    max_linear_x=0.50,
    max_linear_y=0.15,
    max_angular_z=0.6,
)
_LINUX_POSE = dict(
    roll_axis=3,
    pitch_axis=4,
    invert_roll=True,
    invert_pitch=True,
    deadzone=0.0,
    roll_rate=0.4,
    pitch_rate=0.4,
    max_roll=0.25,
    max_pitch=0.20,
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
    vx, vy, wz = locomotion_twist([1.0, -1.0, 0.0, 0.0, 0.0], **_LOCO)
    assert vx == 0.50
    assert vy == -0.15  # stick right (+1), invert → robot right (−y)
    assert wz == 0.0


def test_stick_right_turns_right():
    _, _, wz = locomotion_twist(
        [0.0, 0.0, 1.0], **{**_LOCO, 'deadzone': 0.0}
    )
    assert wz == -0.6


def test_vx_vy_yaw_together():
    vx, vy, wz = locomotion_twist(
        [1.0, -1.0, 1.0], **{**_LOCO, 'deadzone': 0.0}
    )
    assert vx == 0.50
    assert vy == -0.15
    assert wz == -0.6


def test_xpadneo_stick_up_is_forward_stick_left_turns_left():
    # axes: 0 LX, 1 LY, 2 LT, 3 RX, 4 RY
    vx, vy, wz = locomotion_twist(
        [0.0, 1.0, 0.0, 0.0, 0.0], **_LINUX
    )
    assert vx == 0.50
    vx, vy, wz = locomotion_twist(
        [0.0, 0.0, 0.0, 1.0, 0.0], **_LINUX
    )
    assert wz == 0.6  # stick left (positive RX) → +yaw = turn left


def test_invert_vx_overlay_flips_linux_stick_up_to_backward():
    vx, _, _ = locomotion_twist(
        [0.0, 1.0, 0.0, 0.0, 0.0], **{**_LINUX, 'invert_vx': True}
    )
    assert vx == -0.50


def test_xpadneo_stick_right_strafes_right():
    _, vy, _ = locomotion_twist([ -1.0, 0.0, 0.0, 0.0, 0.0], **_LINUX)
    assert vy == -0.15  # stick right (negative LX) → −vy


def test_xpadneo_body_pose_stick_up_is_nose_up():
    roll, pitch, moved = integrate_body_pose(
        0.0, 0.0, [0.0, 0.0, 0.0, 0.0, 1.0], dt=0.25, **_LINUX_POSE
    )
    assert moved
    assert abs(roll) < 1e-12
    assert abs(pitch - (-0.10)) < 1e-9


def test_xpadneo_body_pose_stick_right_is_positive_roll():
    roll, pitch, moved = integrate_body_pose(
        0.0, 0.0, [0.0, 0.0, 0.0, -1.0, 0.0], dt=0.25, **_LINUX_POSE
    )
    assert moved
    assert abs(roll - 0.10) < 1e-9
    assert abs(pitch) < 1e-12


def test_first_packet_is_not_a_rising_edge():
    assert not rising_edge([0, 0, 0, 0, 1], [], 4)
    assert not rising_edge([0, 0, 0, 0, 1], None, 4)


def test_body_pose_stick_up_is_nose_up_negative_pitch():
    roll, pitch, moved = integrate_body_pose(
        0.0, 0.0, [0.0, 0.0, 0.0, -1.0], dt=0.25, **_POSE
    )
    assert moved
    assert abs(roll) < 1e-12
    assert abs(pitch - (-0.10)) < 1e-9
    x, y, z, w = quaternion_from_rpy(roll, pitch, 0.0)
    assert abs(y - (-0.04997916667)) < 1e-5
    assert abs(w - 0.99875026039) < 1e-5


def test_body_pose_stick_right_is_positive_roll():
    roll, pitch, moved = integrate_body_pose(
        0.0, 0.0, [0.0, 0.0, 1.0, 0.0],
        dt=0.25,
        **{**_POSE, 'deadzone': 0.0},
    )
    assert moved
    assert abs(roll - 0.10) < 1e-9
    assert abs(pitch) < 1e-12


def test_body_pose_roll_and_pitch_together():
    roll, pitch, moved = integrate_body_pose(
        0.0, 0.0, [0.0, 0.0, 1.0, -1.0],
        dt=0.25,
        **{**_POSE, 'deadzone': 0.0},
    )
    assert moved
    assert abs(roll - 0.10) < 1e-9
    assert abs(pitch - (-0.10)) < 1e-9


def test_body_pose_release_holds_angle():
    roll, pitch, moved = integrate_body_pose(
        0.12, -0.08, [0.0, 0.0, 0.0, 0.0], dt=0.05, **_POSE
    )
    assert not moved
    assert roll == 0.12
    assert pitch == -0.08


def test_body_pose_clamped_to_yaml_limit():
    roll, pitch, _ = integrate_body_pose(
        0.0, -0.19, [0.0, 0.0, 0.0, -1.0],
        dt=1.0,
        **{**_POSE, 'deadzone': 0.0, 'pitch_rate': 1.0, 'roll_rate': 1.0},
    )
    assert pitch == -0.20
    assert roll == 0.0


def test_rising_edge_mode_toggle():
    assert rising_edge([0, 0, 0, 0, 1], [0, 0, 0, 0, 0], 4)
    assert not rising_edge([0, 0, 0, 0, 1], [0, 0, 0, 0, 1], 4)
    assert not rising_edge([0, 0, 0, 0, 0], [0, 0, 0, 0, 1], 4)


def test_axis_missing_is_zero():
    assert axis_at([], 4, False, 0.1) == 0.0
