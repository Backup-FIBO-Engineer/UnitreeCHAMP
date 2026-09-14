"""ROS-free observation packing for a Unitree locomotion actor.

Shipped yaml is the 48-D unitree_rl_gym / Isaac Lab layout:

    lin_vel(3) | ang_vel(3) | gravity(3) | command(3) | dof_pos(12) | dof_vel(12) | action(12)

A 45-D actor (no lin_vel) is the same list without the first term. Terms, scales
and history length come from yaml so a checkpoint trained with clock or a
stacked history still loads without code changes.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Mapping, Optional, Sequence, Union

import numpy as np

TERM_ANG_VEL = 'ang_vel'
TERM_GRAVITY = 'gravity'
TERM_COMMAND = 'command'
TERM_LIN_VEL = 'lin_vel'
TERM_DOF_POS = 'dof_pos'
TERM_DOF_VEL = 'dof_vel'
TERM_ACTION = 'action'
TERM_CLOCK = 'clock'

KNOWN_TERMS = (
    TERM_ANG_VEL,
    TERM_GRAVITY,
    TERM_COMMAND,
    TERM_LIN_VEL,
    TERM_DOF_POS,
    TERM_DOF_VEL,
    TERM_ACTION,
    TERM_CLOCK,
)

LIN_VEL_FRAME_BODY = 'body'
LIN_VEL_FRAME_WORLD = 'world'
LIN_VEL_FRAMES = (LIN_VEL_FRAME_BODY, LIN_VEL_FRAME_WORLD)


def _as_float_array(value: Sequence[float], size: int, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float32).reshape(-1)
    if array.size != size:
        raise ValueError(f'{name} needs {size} values, got {array.size}')
    if not np.all(np.isfinite(array)):
        raise ValueError(f'{name} contains NaN or Inf')
    return array


def quat_rotate_inverse_xyzw(quat_xyzw: Sequence[float], vector: Sequence[float]) -> np.ndarray:
    """Rotate a world vector into the body frame (Isaac `quat_rotate_inverse`)."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    q_vec = np.array([x, y, z], dtype=np.float64)
    v = np.array([float(c) for c in vector], dtype=np.float64)
    a = v * (2.0 * w * w - 1.0)
    b = np.cross(q_vec, v) * (2.0 * w)
    c = q_vec * np.dot(q_vec, v) * 2.0
    return (a - b + c).astype(np.float32)


def projected_gravity_xyzw(quat_xyzw: Sequence[float]) -> np.ndarray:
    """Gravity direction in the body frame. Identity quaternion → (0, 0, −1)."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    norm = (w * w + x * x + y * y + z * z) ** 0.5
    if not np.isfinite(norm) or norm <= 1e-12:
        return np.array([0.0, 0.0, -1.0], dtype=np.float32)
    scaled = (x / norm, y / norm, z / norm, w / norm)
    return quat_rotate_inverse_xyzw(scaled, (0.0, 0.0, -1.0))


def gravity_from_specific_force(accel_body: Sequence[float]) -> np.ndarray:
    """When orientation is missing: ROS linear_acceleration is specific force."""
    accel = np.asarray(accel_body, dtype=np.float32).reshape(3)
    norm = float(np.linalg.norm(accel))
    if not np.isfinite(norm) or norm <= 1e-6:
        return np.array([0.0, 0.0, -1.0], dtype=np.float32)
    return (-accel / norm).astype(np.float32)


def as_vector3(value: Union[Sequence[float], object]) -> np.ndarray:
    """geometry_msgs Vector3 or a length-3 sequence (DLS Screw.linear)."""
    if hasattr(value, 'x'):
        return _as_float_array(
            (float(value.x), float(value.y), float(value.z)), 3, 'vector3')
    return _as_float_array(value, 3, 'vector3')


def as_quat_xyzw(value: Union[Sequence[float], object]) -> np.ndarray:
    """geometry_msgs Quaternion or a length-4 xyzw sequence (DLS Pose.orientation)."""
    if hasattr(value, 'x') and hasattr(value, 'w'):
        return _as_float_array(
            (float(value.x), float(value.y), float(value.z), float(value.w)),
            4, 'quat_xyzw')
    return _as_float_array(value, 4, 'quat_xyzw')


def quat_xyzw_is_valid(quat_xyzw: Sequence[float]) -> bool:
    q = np.asarray(quat_xyzw, dtype=np.float64).reshape(-1)
    if q.size != 4 or not np.all(np.isfinite(q)):
        return False
    return float(np.linalg.norm(q)) > 1e-6


def linear_velocity_in_body_frame(
    linear_xyz: Sequence[float],
    source_frame: str,
    quat_xyzw: Sequence[float],
) -> np.ndarray:
    """Training `base_lin_vel` is body-frame. Rotate world twist with `quat_rotate_inverse`."""
    velocity = as_vector3(linear_xyz)
    frame = str(source_frame).strip().lower()
    if frame == LIN_VEL_FRAME_BODY:
        return velocity
    if frame == LIN_VEL_FRAME_WORLD:
        if not quat_xyzw_is_valid(quat_xyzw):
            raise ValueError('world-frame lin_vel needs a finite orientation quaternion')
        return quat_rotate_inverse_xyzw(quat_xyzw, velocity)
    raise ValueError(
        f'lin_vel_frame must be one of {LIN_VEL_FRAMES}, got {source_frame!r}')


@dataclass
class ObservationConfig:
    terms: List[str]
    num_dofs: int
    clip: float = 100.0
    history: int = 1
    ang_vel_scale: float = 0.25
    dof_pos_scale: float = 1.0
    dof_vel_scale: float = 0.05
    lin_vel_scale: float = 2.0
    cmd_scale: np.ndarray = field(default_factory=lambda: np.array([2.0, 2.0, 0.25], dtype=np.float32))
    clock_period: float = 0.8

    def __post_init__(self) -> None:
        if self.num_dofs <= 0:
            raise ValueError('num_dofs must be > 0')
        if self.history < 1:
            raise ValueError('observation.history must be >= 1')
        if not self.terms:
            raise ValueError('observation.terms must not be empty')
        unknown = [term for term in self.terms if term not in KNOWN_TERMS]
        if unknown:
            raise ValueError(f'unknown observation terms {unknown}; known: {KNOWN_TERMS}')
        self.cmd_scale = _as_float_array(self.cmd_scale, 3, 'cmd_scale')

    @property
    def single_size(self) -> int:
        size = 0
        for term in self.terms:
            if term in (TERM_ANG_VEL, TERM_GRAVITY, TERM_COMMAND, TERM_LIN_VEL):
                size += 3
            elif term in (TERM_DOF_POS, TERM_DOF_VEL, TERM_ACTION):
                size += self.num_dofs
            elif term == TERM_CLOCK:
                size += 2
        return size

    @property
    def size(self) -> int:
        return self.single_size * self.history

    @property
    def needs_lin_vel(self) -> bool:
        return TERM_LIN_VEL in self.terms

    @classmethod
    def from_mapping(cls, data: Mapping, num_dofs: int) -> 'ObservationConfig':
        terms = [str(t) for t in data.get('terms', (
            TERM_ANG_VEL, TERM_GRAVITY, TERM_COMMAND, TERM_DOF_POS, TERM_DOF_VEL, TERM_ACTION,
        ))]
        return cls(
            terms=terms,
            num_dofs=num_dofs,
            clip=float(data.get('clip', 100.0)),
            history=int(data.get('history', 1)),
            ang_vel_scale=float(data.get('ang_vel_scale', 0.25)),
            dof_pos_scale=float(data.get('dof_pos_scale', 1.0)),
            dof_vel_scale=float(data.get('dof_vel_scale', 0.05)),
            lin_vel_scale=float(data.get('lin_vel_scale', 2.0)),
            cmd_scale=data.get('cmd_scale', [2.0, 2.0, 0.25]),
            clock_period=float(data.get('clock_period', 0.8)),
        )


@dataclass
class RobotObservation:
    ang_vel: np.ndarray
    gravity: np.ndarray
    command: np.ndarray
    dof_pos: np.ndarray
    dof_vel: np.ndarray
    last_action: np.ndarray
    lin_vel: Optional[np.ndarray] = None
    time_sec: float = 0.0


def pack_observation(cfg: ObservationConfig, sample: RobotObservation) -> np.ndarray:
    """One frame, unclipped. Length = cfg.single_size."""
    pieces: List[np.ndarray] = []
    n = cfg.num_dofs
    for term in cfg.terms:
        if term == TERM_ANG_VEL:
            pieces.append(_as_float_array(sample.ang_vel, 3, 'ang_vel') * cfg.ang_vel_scale)
        elif term == TERM_GRAVITY:
            pieces.append(_as_float_array(sample.gravity, 3, 'gravity'))
        elif term == TERM_COMMAND:
            pieces.append(_as_float_array(sample.command, 3, 'command') * cfg.cmd_scale)
        elif term == TERM_LIN_VEL:
            if sample.lin_vel is None:
                raise ValueError(
                    'observation.terms includes lin_vel but the sample has no body-frame lin_vel')
            pieces.append(_as_float_array(sample.lin_vel, 3, 'lin_vel') * cfg.lin_vel_scale)
        elif term == TERM_DOF_POS:
            pieces.append(_as_float_array(sample.dof_pos, n, 'dof_pos') * cfg.dof_pos_scale)
        elif term == TERM_DOF_VEL:
            pieces.append(_as_float_array(sample.dof_vel, n, 'dof_vel') * cfg.dof_vel_scale)
        elif term == TERM_ACTION:
            pieces.append(_as_float_array(sample.last_action, n, 'last_action'))
        elif term == TERM_CLOCK:
            period = cfg.clock_period if cfg.clock_period > 0.0 else 0.8
            phase = (float(sample.time_sec) / period) % 1.0
            pieces.append(np.array(
                [np.sin(2.0 * np.pi * phase), np.cos(2.0 * np.pi * phase)],
                dtype=np.float32,
            ))
    packed = np.concatenate(pieces).astype(np.float32, copy=False)
    return packed


class ObservationHistory:
    """Stacks the last `history` frames, oldest first."""

    def __init__(self, cfg: ObservationConfig) -> None:
        self.cfg = cfg
        self._frames = np.zeros((cfg.history, cfg.single_size), dtype=np.float32)
        self._filled = 0

    def reset(self) -> None:
        self._frames.fill(0.0)
        self._filled = 0

    def push(self, frame: np.ndarray) -> np.ndarray:
        frame = np.asarray(frame, dtype=np.float32).reshape(-1)
        if frame.size != self.cfg.single_size:
            raise ValueError(
                f'observation frame has {frame.size} values, expected {self.cfg.single_size}')
        clip = abs(float(self.cfg.clip))
        if clip > 0.0:
            frame = np.clip(frame, -clip, clip)
        if self.cfg.history == 1:
            self._frames[0] = frame
            self._filled = 1
            return frame.copy()
        if self._filled == 0:
            self._frames[:] = frame
            self._filled = self.cfg.history
        else:
            self._frames[:-1] = self._frames[1:]
            self._frames[-1] = frame
            self._filled = min(self._filled + 1, self.cfg.history)
        return self._frames.reshape(-1).copy()

    @property
    def stacked(self) -> np.ndarray:
        return self._frames.reshape(-1).copy()
