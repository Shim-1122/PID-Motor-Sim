"""Closed-loop simulation: PID controller + DC motor + optional encoder and load disturbance."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .motor import DCMotor
from .pid import PID


@dataclass
class Response:
    t: np.ndarray          # time [s]
    theta: np.ndarray      # true shaft angle [rad]
    voltage: np.ndarray    # controller output [V]
    current: np.ndarray    # armature current [A]
    setpoint: float


def simulate(
    motor: DCMotor,
    pid: PID,
    setpoint: float = np.pi / 2,
    t_end: float = 1.0,
    load_torque: float = 0.0,
    t_load: float = 0.5,
    encoder_cpr: int | None = None,
) -> Response:
    """Step the closed loop once per controller sample.

    load_torque : constant torque applied from t_load onward [N m]
    encoder_cpr : encoder counts per revolution; None means perfect angle measurement
    """
    Ad, Bd = motor.discretize(pid.ts)
    n = int(round(t_end / pid.ts)) + 1
    t = np.arange(n) * pid.ts
    theta, voltage, current = np.zeros(n), np.zeros(n), np.zeros(n)

    counts_per_rad = encoder_cpr / (2 * np.pi) if encoder_cpr else None
    x = np.zeros(3)                                   # start at rest: [theta, omega, i] = 0
    pid.reset()

    for k in range(n):
        measured = x[0] if counts_per_rad is None else np.round(x[0] * counts_per_rad) / counts_per_rad
        u = pid.update(setpoint, measured)
        tau = load_torque if t[k] >= t_load else 0.0

        theta[k], current[k], voltage[k] = x[0], x[2], u
        x = Ad @ x + Bd @ np.array([u, tau])          # exact ZOH update to the next sample

    return Response(t, theta, voltage, current, setpoint)
