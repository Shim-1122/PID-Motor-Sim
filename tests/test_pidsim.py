"""Physics and control checks against closed-form results."""
import numpy as np
import pytest

from pidsim import PID, DCMotor, simulate, step_metrics

MOTOR = DCMotor()


# ---------------------------------------------------------------- plant model
def test_open_loop_speed_matches_steady_state_formula():
    Ad, Bd = MOTOR.discretize(1e-3)
    x = np.zeros(3)
    for _ in range(2000):                                  # 2 s, many mechanical time constants
        x = Ad @ x + Bd @ np.array([12.0, 0.0])
    assert x[1] == pytest.approx(MOTOR.steady_state_speed(12.0), rel=1e-6)


def test_stall_is_an_equilibrium():
    """Shaft held at 0 speed with i = V/R: motor torque exactly balances the stall load, nothing moves."""
    Ad, Bd = MOTOR.discretize(1e-3)
    x0 = np.array([0.0, 0.0, 12.0 / MOTOR.R])
    x = x0.copy()
    for _ in range(1000):
        x = Ad @ x + Bd @ np.array([12.0, MOTOR.stall_torque(12.0)])
    np.testing.assert_allclose(x, x0, atol=1e-9)


# -------------------------------------------------------------------- metrics
def test_metrics_on_analytic_second_order_response():
    zeta, wn = 0.4, 10.0
    t = np.linspace(0, 5, 50_001)
    wd = wn * np.sqrt(1 - zeta**2)
    y = 1 - np.exp(-zeta * wn * t) * (np.cos(wd * t) + zeta / np.sqrt(1 - zeta**2) * np.sin(wd * t))
    expected_overshoot = 100 * np.exp(-np.pi * zeta / np.sqrt(1 - zeta**2))
    assert step_metrics(t, y, 1.0)["overshoot_pct"] == pytest.approx(expected_overshoot, rel=1e-3)


def test_metrics_on_first_order_response():
    tau = 0.2
    t = np.linspace(0, 3, 300_001)
    m = step_metrics(t, 1 - np.exp(-t / tau), 1.0)
    assert m["rise_time"] == pytest.approx(tau * np.log(9), rel=1e-3)        # 10% -> 90%
    assert m["settling_time"] == pytest.approx(tau * np.log(50), rel=1e-3)   # within 2%
    assert m["overshoot_pct"] == 0.0


def test_metrics_scale_with_setpoint():
    t = np.linspace(0, 3, 3001)
    y = 1 - np.exp(-t / 0.2)
    assert step_metrics(t, 5 * y, 5.0)["rise_time"] == pytest.approx(step_metrics(t, y, 1.0)["rise_time"])


# ----------------------------------------------------------------- controller
def test_p_control_error_under_load_matches_prediction():
    """At rest under load: i = tau/K, V = R i, and V = Kp e  =>  e = R tau / (K Kp)."""
    kp, load = 25.0, 0.05
    r = simulate(MOTOR, PID(kp, 0, 0.3), load_torque=load, t_load=0.0, t_end=2.0)
    expected = MOTOR.R * load / (MOTOR.K * kp)
    assert r.setpoint - r.theta[-1] == pytest.approx(expected, rel=1e-3)


def test_integral_action_removes_error_under_load():
    r = simulate(MOTOR, PID(25, 100, 0.3), load_torque=0.05, t_load=0.0, t_end=3.0)
    assert abs(r.setpoint - r.theta[-1]) < 1e-3


def test_output_never_exceeds_supply_voltage():
    r = simulate(MOTOR, PID(200, 500, 1.0, u_max=12.0), setpoint=10.0)
    assert np.abs(r.voltage).max() <= 12.0


def test_no_derivative_kick_on_setpoint_step():
    """Derivative acts on the measurement, so the first output is pure proportional."""
    pid = PID(kp=2.0, ki=0.0, kd=100.0, u_max=1e9)
    assert pid.update(setpoint=1.0, y=0.0) == pytest.approx(2.0)


def test_anti_windup_reduces_overshoot_when_saturated():
    overshoot = {}
    for aw in (False, True):
        r = simulate(MOTOR, PID(25, 100, 0.3, anti_windup=aw), setpoint=6 * np.pi)
        overshoot[aw] = step_metrics(r.t, r.theta, r.setpoint)["overshoot_pct"]
    assert overshoot[True] < 0.25 * overshoot[False]


def test_encoder_quantization_limits_accuracy():
    cpr = 256
    r = simulate(MOTOR, PID(25, 100, 0.3), setpoint=np.pi / 2, t_end=2.0, encoder_cpr=cpr)
    assert abs(r.setpoint - r.theta[-1]) <= 2 * np.pi / cpr    # within one encoder count
