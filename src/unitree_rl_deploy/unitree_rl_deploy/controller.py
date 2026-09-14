"""One policy tick: obs → action → joint targets. No ROS."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from unitree_rl_deploy.config import DeployConfig
from unitree_rl_deploy.observation import (
    ObservationHistory,
    RobotObservation,
    gravity_from_specific_force,
    pack_observation,
    projected_gravity_xyzw,
)
from unitree_rl_deploy.policy import PolicyFn


@dataclass
class SensorSample:
    q: np.ndarray
    dq: np.ndarray
    quat_xyzw: np.ndarray
    ang_vel: np.ndarray
    cmd_vx: float = 0.0
    cmd_vy: float = 0.0
    cmd_wz: float = 0.0
    lin_vel: Optional[np.ndarray] = None
    accel: Optional[np.ndarray] = None
    orientation_valid: bool = True
    time_sec: float = 0.0


class PolicyController:
    def __init__(self, cfg: DeployConfig, policy: PolicyFn) -> None:
        self.cfg = cfg
        self.policy = policy
        self.history = ObservationHistory(cfg.observation)
        self.last_action = np.zeros(cfg.num_actions, dtype=np.float32)
        self._scales = cfg.action_scales()

    def reset(self) -> None:
        self.history.reset()
        self.last_action[:] = 0.0

    def clamp_command(self, vx: float, vy: float, wz: float) -> np.ndarray:
        cmd = np.array([vx, vy, wz], dtype=np.float32)
        limit = self.cfg.max_cmd
        return np.clip(cmd, -limit, limit)

    def targets(self, sample: SensorSample) -> np.ndarray:
        n = self.cfg.num_actions
        q = np.asarray(sample.q, dtype=np.float32).reshape(-1)
        dq = np.asarray(sample.dq, dtype=np.float32).reshape(-1)
        if q.size != n or dq.size != n:
            raise ValueError(f'joint vectors must have {n} values')
        if sample.orientation_valid:
            gravity = projected_gravity_xyzw(sample.quat_xyzw)
        elif sample.accel is not None:
            gravity = gravity_from_specific_force(sample.accel)
        else:
            gravity = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        command = self.clamp_command(sample.cmd_vx, sample.cmd_vy, sample.cmd_wz)
        frame = pack_observation(self.cfg.observation, RobotObservation(
            ang_vel=np.asarray(sample.ang_vel, dtype=np.float32).reshape(3),
            gravity=gravity,
            command=command,
            dof_pos=q - self.cfg.default_angles,
            dof_vel=dq,
            last_action=self.last_action,
            lin_vel=None if sample.lin_vel is None else np.asarray(sample.lin_vel, dtype=np.float32),
            time_sec=sample.time_sec,
        ))
        obs = self.history.push(frame)
        action = np.asarray(self.policy(obs), dtype=np.float32).reshape(-1)
        if action.size != n:
            raise ValueError(f'policy returned {action.size} actions, expected {n}')
        clip = abs(float(self.cfg.clip_actions))
        if clip > 0.0:
            action = np.clip(action, -clip, clip)
        self.last_action = action
        return self.cfg.default_angles + action * self._scales
