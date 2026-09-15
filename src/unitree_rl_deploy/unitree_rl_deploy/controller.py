"""One policy tick: obs → action → joint targets. No ROS."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from unitree_rl_deploy.config import DeployConfig
from unitree_rl_deploy.gait_clock import GaitClock
from unitree_rl_deploy.observation import (
    ObservationHistory,
    RobotObservation,
    TERM_CLOCK,
    com_linear_velocity_body,
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
        self.gait_clock: Optional[GaitClock] = None
        if cfg.observation.needs_clock and cfg.observation.clock_size == 4:
            self.gait_clock = GaitClock(
                offsets=cfg.gait_phase_offsets,
                freq_min=cfg.gait_step_freq_min,
                freq_max=cfg.gait_step_freq_max,
                speed_min=cfg.gait_speed_min,
                speed_max=cfg.gait_speed_max,
                still_threshold=cfg.gait_still_command_threshold,
                dt=1.0 / cfg.control_rate,
            )

    def reset(self) -> None:
        self.history.reset()
        self.last_action[:] = 0.0
        if self.gait_clock is not None:
            self.gait_clock.reset()

    def clamp_command(self, vx: float, vy: float, wz: float) -> np.ndarray:
        cmd = np.array([vx, vy, wz], dtype=np.float32)
        limit = self.cfg.max_cmd
        return np.clip(cmd, -limit, limit)

    def _body_lin_vel(self, sample: SensorSample) -> Optional[np.ndarray]:
        if sample.lin_vel is None:
            return None
        velocity = np.asarray(sample.lin_vel, dtype=np.float32).reshape(-1)
        offset = self.cfg.lin_vel_com_offset
        if float(np.linalg.norm(offset)) <= 0.0:
            return velocity
        return com_linear_velocity_body(velocity, sample.ang_vel, offset)

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
        clock = None
        if self.gait_clock is not None:
            clock = self.gait_clock.step(command)
        elif TERM_CLOCK in self.cfg.observation.terms and self.cfg.observation.clock_size == 4:
            raise ValueError('gait clock is required for clock_size 4')
        frame = pack_observation(self.cfg.observation, RobotObservation(
            ang_vel=np.asarray(sample.ang_vel, dtype=np.float32).reshape(3),
            gravity=gravity,
            command=command,
            dof_pos=q - self.cfg.default_angles,
            dof_vel=dq,
            last_action=self.last_action,
            lin_vel=self._body_lin_vel(sample),
            clock=clock,
            time_sec=sample.time_sec,
        ))
        obs = self.history.push(frame)
        action = np.asarray(self.policy(obs), dtype=np.float32).reshape(-1)
        if action.size != n:
            raise ValueError(f'policy returned {action.size} actions, expected {n}')
        clip = abs(float(self.cfg.clip_actions))
        if clip > 0.0:
            action = np.clip(action, -clip, clip)
        # DLS: last_action in the next obs is the unfiltered clipped policy output.
        # The first-order filter is applied only to the joint targets.
        if self.cfg.use_filter_actions:
            alpha = float(self.cfg.filter_alpha)
            applied = alpha * action + (1.0 - alpha) * self.last_action
        else:
            applied = action
        self.last_action = action
        return self.cfg.default_angles + applied * self._scales
