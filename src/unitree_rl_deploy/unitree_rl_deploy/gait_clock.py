"""DLS gait clock used as the last 4 values of a Flat/Blind observation.

Matches iit-DLSLab ``LocomotionEnv._get_observations`` / ``LocomotionPolicyWrapper``:

* four phases (FL, FR, RL, RR) in [0, 1)
* offsets default ``[0.0, 0.5, 0.5, 0.0]``
* ``phase += step_freq * dt`` then wrap, even while standing
* ``step_freq`` ramps ``freq_min → freq_max`` with ``||cmd_xy||`` between
  ``speed_min`` and ``speed_max``
* if ``||cmd|| < still_threshold`` the observation is all ``-1`` (phase still advances)

The values packed into the observation are the raw phases, not sin/cos.
"""
from __future__ import annotations

from typing import Sequence

import numpy as np


class GaitClock:
    def __init__(
        self,
        offsets: Sequence[float] = (0.0, 0.5, 0.5, 0.0),
        freq_min: float = 1.4,
        freq_max: float = 1.8,
        speed_min: float = 0.4,
        speed_max: float = 0.8,
        still_threshold: float = 0.01,
        dt: float = 0.02,
    ) -> None:
        self.offsets = np.asarray(offsets, dtype=np.float64).reshape(-1)
        if self.offsets.size != 4:
            raise ValueError(f'gait phase offsets need 4 values, got {self.offsets.size}')
        if not np.all(np.isfinite(self.offsets)):
            raise ValueError('gait phase offsets contain NaN or Inf')
        if dt <= 0.0:
            raise ValueError('gait dt must be > 0')
        if speed_max < speed_min:
            raise ValueError('gait speed_max must be >= speed_min')
        self.freq_min = float(freq_min)
        self.freq_max = float(freq_max)
        self.speed_min = float(speed_min)
        self.speed_max = float(speed_max)
        self.still_threshold = float(still_threshold)
        self.dt = float(dt)
        self.phase = self.offsets.copy() % 1.0
        self.step_freq = self.freq_min

    def reset(self) -> None:
        self.phase = self.offsets.copy() % 1.0
        self.step_freq = self.freq_min

    def step(self, command: Sequence[float]) -> np.ndarray:
        cmd = np.asarray(command, dtype=np.float64).reshape(-1)
        if cmd.size != 3:
            raise ValueError(f'gait command needs 3 values, got {cmd.size}')
        xy_norm = float(np.linalg.norm(cmd[:2]))
        span = self.speed_max - self.speed_min
        if span <= 0.0:
            ramp = 1.0 if xy_norm >= self.speed_max else 0.0
        else:
            ramp = float(np.clip((xy_norm - self.speed_min) / span, 0.0, 1.0))
        self.step_freq = self.freq_min + ramp * (self.freq_max - self.freq_min)
        self.phase = (self.phase + self.step_freq * self.dt) % 1.0
        if float(np.linalg.norm(cmd)) < self.still_threshold:
            return np.full(4, -1.0, dtype=np.float32)
        return self.phase.astype(np.float32)
