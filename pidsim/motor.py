"""Armature-controlled DC motor, modeled in state space.

State  x = [theta, omega, i]      (angle [rad], speed [rad/s], current [A])
Input  u = [V, tau_load]          (terminal voltage [V], external load torque [N m])

    theta' = omega
    J omega' = K i - b omega - tau_load        (mechanical: Newton's 2nd law)
    L i'     = V - R i - K omega               (electrical: Kirchhoff's voltage law)
"""
from dataclasses import dataclass

import numpy as np
from scipy.signal import cont2discrete


@dataclass(frozen=True)
class DCMotor:
    R: float = 2.0     # armature resistance [ohm]
    L: float = 1e-3    # armature inductance [H]
    K: float = 0.05    # torque constant [N m/A], equal to back-EMF constant [V s/rad] in SI units
    J: float = 5e-5    # rotor + load inertia [kg m^2]
    b: float = 1e-5    # viscous friction [N m s/rad]

    def state_space(self) -> tuple[np.ndarray, np.ndarray]:
        """Continuous-time x' = A x + B u."""
        R, L, K, J, b = self.R, self.L, self.K, self.J, self.b
        A = np.array([
            [0, 1, 0],
            [0, -b / J, K / J],
            [0, -K / L, -R / L],
        ])
        B = np.array([
            [0, 0],
            [0, -1 / J],
            [1 / L, 0],
        ])
        return A, B

    def discretize(self, ts: float) -> tuple[np.ndarray, np.ndarray]:
        """Exact zero-order-hold discretization: x[k+1] = Ad x[k] + Bd u[k].

        A digital controller holds its output constant between samples (the ZOH
        assumption), so for a linear plant this is exact at every sample.
        """
        A, B = self.state_space()
        C, D = np.eye(3), np.zeros((3, 2))
        Ad, Bd, *_ = cont2discrete((A, B, C, D), ts, method="zoh")
        return Ad, Bd

    def steady_state_speed(self, voltage: float) -> float:
        """Open-loop speed for a constant voltage (set omega' = i' = 0 and solve)."""
        return self.K * voltage / (self.R * self.b + self.K**2)

    def stall_torque(self, voltage: float) -> float:
        """Torque at zero speed: all voltage drops across R."""
        return self.K * voltage / self.R
