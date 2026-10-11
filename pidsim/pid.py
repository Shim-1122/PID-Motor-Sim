"""Discrete PID controller (microcontroller-style implementation).

    u = Kp * e  +  Ki * integral(e)  -  Kd * d(y)/dt          with e = setpoint - y

Additions to the textbook form:
  1. Derivative on measurement (dy/dt, not de/dt): no output spike on setpoint
     steps. Damping is otherwise unchanged, since de/dt = -dy/dt.
  2. Filtered derivative: first-order low-pass limits encoder noise amplification.
  3. Output saturation at the supply voltage.
  4. Anti-windup (clamping): integration pauses while the output is saturated
     and the error would increase saturation.
"""
from dataclasses import dataclass, field


@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    ts: float = 1e-3            # sample period [s] (1 kHz loop)
    u_max: float = 12.0         # output limit, +/- supply voltage [V]
    tau_d: float = 2e-3         # derivative low-pass time constant [s]
    anti_windup: bool = True

    # internal state, reset before every run
    integral: float = field(default=0.0, init=False)
    d_filtered: float = field(default=0.0, init=False)
    y_prev: float | None = field(default=None, init=False)

    def reset(self) -> None:
        self.integral, self.d_filtered, self.y_prev = 0.0, 0.0, None

    def update(self, setpoint: float, y: float) -> float:
        """One control step: take the latest measurement, return the command."""
        e = setpoint - y

        # (1)+(2) filtered derivative of the measurement
        dy_dt = 0.0 if self.y_prev is None else (y - self.y_prev) / self.ts
        alpha = self.ts / (self.tau_d + self.ts)          # low-pass blend factor, 0..1
        self.d_filtered += alpha * (dy_dt - self.d_filtered)
        self.y_prev = y

        # (3) saturate
        u_unclamped = self.kp * e + self.ki * self.integral - self.kd * self.d_filtered
        u = min(max(u_unclamped, -self.u_max), self.u_max)

        # (4) anti-windup: skip integration if it would only deepen saturation
        saturated = u != u_unclamped
        pushing_further = (e > 0) == (u_unclamped > 0)
        if not (self.anti_windup and saturated and pushing_further):
            self.integral += e * self.ts
        return u
