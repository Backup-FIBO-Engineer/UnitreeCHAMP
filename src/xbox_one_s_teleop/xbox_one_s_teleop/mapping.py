"""Joystick → CHAMP command math (no ROS, no robot constants).

Linux / joy_node convention for an Xbox pad (xpad / xpadneo): stick-up is a
negative axis value. Body pose uses the same RPY → quaternion as CHAMP
(tf2 setRPY): +roll = right side down, +pitch = nose down.
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


def clamp(value: float, limit: float) -> float:
    if limit <= 0.0:
        return value
    return max(-limit, min(limit, value))


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


def quaternion_from_rpy(roll: float, pitch: float, yaw: float = 0.0) -> Tuple[float, float, float, float]:
    """(x, y, z, w) of Rz(yaw) Ry(pitch) Rx(roll). Same as tf2 / CHAMP getRPY."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


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


def integrate_body_pose(
    roll: float,
    pitch: float,
    axes,
    *,
    roll_axis: int,
    pitch_axis: int,
    invert_roll: bool,
    invert_pitch: bool,
    deadzone: float,
    roll_rate: float,
    pitch_rate: float,
    max_roll: float,
    max_pitch: float,
    dt: float,
) -> Tuple[float, float, bool]:
    """Stick deflection is a rate (full stick = *_rate rad/s). Returns (roll, pitch, moved)."""
    if not (dt > 0.0 and math.isfinite(dt)):
        return roll, pitch, False
    droll = axis_at(axes, roll_axis, invert_roll, deadzone) * roll_rate * dt
    dpitch = axis_at(axes, pitch_axis, invert_pitch, deadzone) * pitch_rate * dt
    moved = droll != 0.0 or dpitch != 0.0
    return clamp(roll + droll, max_roll), clamp(pitch + dpitch, max_pitch), moved
