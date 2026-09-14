"""Per-robot RL deploy config. Values live in config/<robot>_rl.yaml only."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence

import numpy as np
import yaml

from unitree_rl_deploy.observation import LIN_VEL_FRAMES, ObservationConfig


def parse_ros_bool(value, default: bool = False) -> bool:
    """Launch substitutions arrive as strings; `bool('false')` is True in Python."""
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, np.integer)):
        return bool(value)
    text = str(value).strip().lower()
    if text in ('true', '1', 'yes', 'on'):
        return True
    if text in ('false', '0', 'no', 'off', ''):
        return False
    raise ValueError(f'invalid bool {value!r}')


def load_ros_params(path: Path | str) -> dict:
    document = yaml.safe_load(Path(path).read_text(encoding='utf-8')) or {}
    for node_value in document.values():
        if isinstance(node_value, dict) and 'ros__parameters' in node_value:
            return dict(node_value['ros__parameters'] or {})
    raise ValueError(f'{path}: no ros__parameters block found')


def _floats(value: Sequence[Any], size: int, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float32).reshape(-1)
    if array.size != size:
        raise ValueError(f'{name} needs {size} values, got {array.size}')
    if not np.all(np.isfinite(array)):
        raise ValueError(f'{name} contains NaN or Inf')
    return array


def _strings(value: Sequence[Any], size: int, name: str) -> list:
    names = [str(item) for item in value]
    if len(names) != size:
        raise ValueError(f'{name} needs {size} names, got {len(names)}')
    if len(set(names)) != len(names):
        raise ValueError(f'{name} lists a joint twice')
    return names


@dataclass
class DeployConfig:
    joint_names: list
    default_angles: np.ndarray
    observation: ObservationConfig
    policy_path: str = ''
    control_rate: float = 50.0
    action_scale: float = 0.5
    hip_scale_reduction: float = 1.0
    hip_indices: np.ndarray = field(default_factory=lambda: np.array([0, 1, 2, 3], dtype=np.int32))
    clip_actions: float = 3.0
    use_filter_actions: bool = False
    filter_alpha: float = 0.8
    max_cmd: np.ndarray = field(default_factory=lambda: np.array([0.8, 0.25, 0.5], dtype=np.float32))
    lin_vel_com_offset: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    gait_phase_offsets: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.5, 0.5, 0.0], dtype=np.float32))
    gait_step_freq_min: float = 1.4
    gait_step_freq_max: float = 1.8
    gait_speed_min: float = 0.4
    gait_speed_max: float = 0.8
    gait_still_command_threshold: float = 0.01
    imu_timeout_sec: float = 0.2
    joint_timeout_sec: float = 0.2
    cmd_timeout_sec: float = 0.5
    lin_vel_timeout_sec: float = 0.2
    command_topic: str = 'joint_commands'
    joint_state_topic: str = 'joint_states'
    imu_topic: str = 'imu/data'
    cmd_vel_topic: str = 'cmd_vel'
    odom_topic: str = 'odom/ground_truth'
    base_state_topic: str = ''
    lin_vel_frame: str = 'body'
    kp: Optional[float] = None
    kd: Optional[float] = None

    def __post_init__(self) -> None:
        n = len(self.joint_names)
        if n == 0:
            raise ValueError('joint_names must not be empty')
        self.default_angles = _floats(self.default_angles, n, 'default_angles')
        if self.control_rate <= 0.0:
            raise ValueError('control_rate must be > 0')
        self.hip_indices = np.asarray(self.hip_indices, dtype=np.int32).reshape(-1)
        if np.any(self.hip_indices < 0) or np.any(self.hip_indices >= n):
            raise ValueError('hip_indices must be in [0, num_actions)')
        self.max_cmd = _floats(self.max_cmd, 3, 'max_cmd')
        self.lin_vel_com_offset = _floats(self.lin_vel_com_offset, 3, 'lin_vel_com_offset')
        self.gait_phase_offsets = _floats(self.gait_phase_offsets, 4, 'gait_phase_offsets')
        if not (0.0 <= float(self.filter_alpha) <= 1.0):
            raise ValueError('filter_alpha must be in [0, 1]')
        if self.gait_speed_max < self.gait_speed_min:
            raise ValueError('gait_speed_max must be >= gait_speed_min')
        if self.observation.num_dofs != n:
            raise ValueError('observation.num_dofs must equal len(joint_names)')
        self.odom_topic = str(self.odom_topic or '').strip()
        self.base_state_topic = str(self.base_state_topic or '').strip()
        self.lin_vel_frame = str(self.lin_vel_frame or 'body').strip().lower()
        if self.lin_vel_frame not in LIN_VEL_FRAMES:
            raise ValueError(
                f'lin_vel_frame must be one of {LIN_VEL_FRAMES}, got {self.lin_vel_frame!r}')
        if self.lin_vel_timeout_sec < 0.0:
            raise ValueError('lin_vel_timeout_sec must be >= 0')
        if self.observation.needs_lin_vel and not self.odom_topic and not self.base_state_topic:
            raise ValueError(
                'observation.terms includes lin_vel but odom_topic and '
                'base_state_topic are empty')

    @property
    def num_actions(self) -> int:
        return len(self.joint_names)

    @property
    def num_obs(self) -> int:
        return self.observation.size

    def action_scales(self) -> np.ndarray:
        scales = np.full(self.num_actions, float(self.action_scale), dtype=np.float32)
        if self.hip_scale_reduction != 1.0:
            scales[self.hip_indices] = float(self.action_scale) * float(self.hip_scale_reduction)
        return scales

    @classmethod
    def from_mapping(cls, data: Mapping) -> 'DeployConfig':
        joint_names = [str(n) for n in data.get('joint_names', [])]
        observation = ObservationConfig.from_mapping(
            data.get('observation', {}), num_dofs=len(joint_names))
        kp = data.get('kp', None)
        kd = data.get('kd', None)
        return cls(
            joint_names=_strings(joint_names, len(joint_names), 'joint_names'),
            default_angles=data.get('default_angles', []),
            observation=observation,
            policy_path=str(data.get('policy_path', '') or ''),
            control_rate=float(data.get('control_rate', 50.0)),
            action_scale=float(data.get('action_scale', 0.5)),
            hip_scale_reduction=float(data.get('hip_scale_reduction', 1.0)),
            hip_indices=data.get('hip_indices', [0, 1, 2, 3]),
            clip_actions=float(data.get('clip_actions', 3.0)),
            use_filter_actions=parse_ros_bool(data.get('use_filter_actions', False)),
            filter_alpha=float(data.get('filter_alpha', 0.8)),
            max_cmd=data.get('max_cmd', [0.8, 0.25, 0.5]),
            lin_vel_com_offset=data.get('lin_vel_com_offset', [0.0, 0.0, 0.0]),
            gait_phase_offsets=data.get('gait_phase_offsets', [0.0, 0.5, 0.5, 0.0]),
            gait_step_freq_min=float(data.get('gait_step_freq_min', 1.4)),
            gait_step_freq_max=float(data.get('gait_step_freq_max', 1.8)),
            gait_speed_min=float(data.get('gait_speed_min', 0.4)),
            gait_speed_max=float(data.get('gait_speed_max', 0.8)),
            gait_still_command_threshold=float(data.get('gait_still_command_threshold', 0.01)),
            imu_timeout_sec=float(data.get('imu_timeout_sec', 0.2)),
            joint_timeout_sec=float(data.get('joint_timeout_sec', 0.2)),
            cmd_timeout_sec=float(data.get('cmd_timeout_sec', 0.5)),
            lin_vel_timeout_sec=float(data.get('lin_vel_timeout_sec', 0.2)),
            command_topic=str(data.get('command_topic', 'joint_commands')),
            joint_state_topic=str(data.get('joint_state_topic', 'joint_states')),
            imu_topic=str(data.get('imu_topic', 'imu/data')),
            cmd_vel_topic=str(data.get('cmd_vel_topic', 'cmd_vel')),
            odom_topic=str(data.get('odom_topic', 'odom/ground_truth') or ''),
            base_state_topic=str(data.get('base_state_topic', '') or ''),
            lin_vel_frame=str(data.get('lin_vel_frame', 'body') or 'body'),
            kp=None if kp is None else float(kp),
            kd=None if kd is None else float(kd),
        )

    @classmethod
    def from_yaml(cls, path: Path | str) -> 'DeployConfig':
        return cls.from_mapping(load_ros_params(path))
