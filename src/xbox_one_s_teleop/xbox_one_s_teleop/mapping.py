"""Joystick → command math (no ROS, no robot constants).

Linux / joy_node convention for a classic Xbox pad: stick-up is a negative
axis value. xpadneo + SDL (Xbox 360 spoof) is the opposite: stick-up and
stick-left are positive; see xbox_one_s_1708_bt_linuxjs.yaml.
"""
from __future__ import annotations

import math
from typing import Tuple


def apply_deadzone(value: float, deadzone: float) -> float:
    if not math.isfinite(value):
        return 0.0
    dz = max(0.0, min(0.95, deadzone))
    mag = abs(value)
    if mag <= dz:
        return 0.0
    scaled = (mag - dz) / (1.0 - dz)
    return math.copysign(min(1.0, scaled), value)


def axis_at(axes, index: int, invert: bool, deadzone: float) -> float:
    if index < 0 or index >= len(axes):
        return 0.0
    value = apply_deadzone(float(axes[index]), deadzone)
    return -value if invert else value


def button_at(buttons, index: int) -> bool:
    if index < 0 or index >= len(buttons):
        return False
    return bool(buttons[index])


def rising_edge(buttons, previous, index: int) -> bool:
    # No previous sample (first /joy packet, or empty latch): not a press.
    if previous is None or len(previous) == 0:
        return False
    return button_at(buttons, index) and not button_at(previous, index)


def locomotion_twist(
    axes,
    *,
    vx_axis: int,
    vy_axis: int,
    yaw_axis: int,
    invert_vx: bool,
    invert_vy: bool,
    invert_yaw: bool,
    deadzone: float,
    max_linear_x: float,
    max_linear_y: float,
    max_angular_z: float,
) -> Tuple[float, float, float]:
    vx = axis_at(axes, vx_axis, invert_vx, deadzone) * max_linear_x
    vy = axis_at(axes, vy_axis, invert_vy, deadzone) * max_linear_y
    wz = axis_at(axes, yaw_axis, invert_yaw, deadzone) * max_angular_z
    return vx, vy, wz
