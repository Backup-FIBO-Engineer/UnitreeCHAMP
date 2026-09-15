from xbox_one_s_teleop.mapping import (
    apply_deadzone,
    axis_at,
    locomotion_twist,
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
    _, vy, _ = locomotion_twist([-1.0, 0.0, 0.0, 0.0, 0.0], **_LINUX)
    assert vy == -0.15  # stick right (negative LX) → −vy


def test_first_packet_is_not_a_rising_edge():
    assert not rising_edge([0, 0, 0, 0, 1], [], 4)
    assert not rising_edge([0, 0, 0, 0, 1], None, 4)


def test_rising_edge_stop_button():
    assert rising_edge([0, 1], [0, 0], 1)
    assert not rising_edge([0, 1], [0, 1], 1)
    assert not rising_edge([0, 0], [0, 1], 1)


def test_axis_missing_is_zero():
    assert axis_at([], 4, False, 0.1) == 0.0
