"""Closed reference paths with exact velocity and acceleration, for trajectory tracking.

The path is p(phi) with a phase phi(t). The phase rate ramps smoothly from 0 to its top
value (smoothstep, so the acceleration is continuous), holds for the requested laps and
ramps down. Each ramp covers half a lap, so the motion starts and ends at rest at the
first point of the path.

    lemniscate (figure eight, Gerono):  x = A sin(phi),  y = A/2 sin(2 phi)
    circle through the start point:     x = A sin(phi),  y = A (1 - cos(phi))

v = p'(phi) phi_dot and a = p''(phi) phi_dot^2 + p'(phi) phi_ddot.
"""
from __future__ import annotations

import math

import numpy as np

SHAPES = ("lemniscate", "circle")


class Trajectory:
    def __init__(self, shape: str, size: float, max_speed: float, laps: float, start, yaw: float = 0.0):
        if shape not in SHAPES:
            raise ValueError(f"unknown shape {shape!r}, expected one of {SHAPES}")
        if size <= 0.0 or max_speed <= 0.0:
            raise ValueError("size and max_speed must be positive")
        self.shape, self.A, self.yaw = shape, float(size), float(yaw)
        self.start = np.asarray(start, dtype=float)
        # largest |dp/dphi| along the path, reached where the top speed is reached
        speed_per_rad = self.A * (math.sqrt(2.0) if shape == "lemniscate" else 1.0)
        self.w = max_speed / speed_per_rad          # top phase rate [rad/s]
        self.laps = max(int(round(laps)), 0)
        self.t_ramp = 2.0 * math.pi / self.w        # each ramp covers half a lap
        self.t1 = self.t_ramp
        self.t2 = self.t1 + self.laps * 2.0 * math.pi / self.w
        self.duration = self.t2 + self.t_ramp
        self.total_phase = 2.0 * math.pi * (self.laps + 1)

    def top_speed_window(self) -> tuple[float, float]:
        """Times [t1, t2] spent at top speed, where tracking errors are compared."""
        return self.t1, self.t2

    def phase(self, t: float) -> tuple[float, float, float]:
        w, T = self.w, self.t_ramp
        if t <= 0.0:
            return 0.0, 0.0, 0.0
        if t < self.t1:
            s = t / T
            return w * T * (s ** 3 - s ** 4 / 2.0), w * (3 * s ** 2 - 2 * s ** 3), w * (6 * s - 6 * s ** 2) / T
        if t < self.t2:
            return w * T / 2.0 + w * (t - self.t1), w, 0.0
        if t < self.duration:
            s = (self.duration - t) / T
            return (self.total_phase - w * T * (s ** 3 - s ** 4 / 2.0), w * (3 * s ** 2 - 2 * s ** 3),
                    -w * (6 * s - 6 * s ** 2) / T)
        return self.total_phase, 0.0, 0.0

    def _path(self, phi: float):
        A, s, c = self.A, math.sin(phi), math.cos(phi)
        if self.shape == "lemniscate":
            s2, c2 = math.sin(2 * phi), math.cos(2 * phi)
            return np.array([A * s, A / 2 * s2, 0.0]), np.array([A * c, A * c2, 0.0]), np.array([-A * s, -2 * A * s2, 0.0])
        return np.array([A * s, A * (1 - c), 0.0]), np.array([A * c, A * s, 0.0]), np.array([-A * s, A * c, 0.0])

    def sample(self, t: float):
        """Position, velocity and acceleration at time t since the start [s]."""
        phi, phi_d, phi_dd = self.phase(t)
        p, dp, ddp = self._path(phi)
        return self.start + p, dp * phi_d, ddp * phi_d ** 2 + dp * phi_dd
