"""Generate the README figures and results table:  python make_figures.py"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from pidsim import PID, DCMotor, simulate, step_metrics

FIG_DIR = Path("figures")
MOTOR = DCMotor()
SETPOINT = np.pi / 2       # 90 degree move
LOAD = 0.05                # N m, about 17% of stall torque at 12 V
T_LOAD, T_END = 0.6, 1.5   # load applied at 0.6 s
ENCODER_CPR = 2048


def plot_gain_comparison():
    """Fixed Kp; add D, then I."""
    controllers = {
        "P   (Kp=25)": PID(kp=25, ki=0, kd=0),
        "PD  (Kp=25, Kd=0.3)": PID(kp=25, ki=0, kd=0.3),
        "PID (Kp=25, Ki=100, Kd=0.3)": PID(kp=25, ki=100, kd=0.3),
    }
    fig, (ax_pos, ax_v) = plt.subplots(2, 1, figsize=(10, 7), sharex=True, height_ratios=[2, 1])
    rows = []
    for name, pid in controllers.items():
        r = simulate(MOTOR, pid, SETPOINT, T_END, LOAD, T_LOAD, ENCODER_CPR)
        ax_pos.plot(r.t, np.degrees(r.theta), label=name, lw=1.6)
        ax_v.plot(r.t, r.voltage, lw=1.0)

        before_load = r.t < T_LOAD
        m = step_metrics(r.t[before_load], r.theta[before_load], SETPOINT)
        final_error_deg = np.degrees(SETPOINT - r.theta[-50:].mean())
        rows.append((name.split()[0], m, final_error_deg))

    ax_pos.axhline(90, color="k", ls="--", lw=0.8)
    ax_pos.axvline(T_LOAD, color="0.5", ls=":")
    ax_pos.text(T_LOAD + 0.01, 20, f"load torque {LOAD} N m applied", color="0.3")
    ax_pos.set_ylabel("shaft angle (deg)")
    ax_pos.set_title("90° Step Response with Load Disturbance")
    ax_pos.legend(loc="lower right")
    ax_pos.grid(alpha=0.3)
    ax_v.axhline(12, color="C3", ls="--", lw=0.8)
    ax_v.axhline(-12, color="C3", ls="--", lw=0.8)
    ax_v.set_ylabel("motor voltage (V)")
    ax_v.set_xlabel("time (s)")
    ax_v.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(FIG_DIR / "gain_comparison.png", dpi=150)
    plt.close(fig)
    return rows


def plot_anti_windup():
    """Long saturated move (3 revolutions), with and without anti-windup."""
    big_move = 6 * np.pi   # three full revolutions
    fig, (ax_pos, ax_v) = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True, height_ratios=[2, 1])
    for anti_windup, style in [(False, "C3"), (True, "C0")]:
        pid = PID(kp=25, ki=100, kd=0.3, anti_windup=anti_windup)
        r = simulate(MOTOR, pid, big_move, t_end=1.0)
        m = step_metrics(r.t, r.theta, big_move)
        label = f"anti-windup {'on ' if anti_windup else 'off'}: overshoot {m['overshoot_pct']:.1f}%"
        ax_pos.plot(r.t, r.theta / (2 * np.pi), color=style, label=label, lw=1.6)
        ax_v.plot(r.t, r.voltage, color=style, lw=1.2)
    ax_pos.axhline(3, color="k", ls="--", lw=0.8)
    ax_pos.set_ylabel("shaft angle (revolutions)")
    ax_pos.set_title("Integrator Windup During Actuator Saturation")
    ax_pos.legend(loc="lower right")
    ax_pos.grid(alpha=0.3)
    ax_v.set_ylabel("motor voltage (V)")
    ax_v.set_xlabel("time (s)")
    ax_v.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(FIG_DIR / "anti_windup.png", dpi=150)
    plt.close(fig)


def write_table(rows, path="results.md"):
    lines = ["| Controller | Rise Time (ms) | Overshoot (%) | Settling Time, 2% (ms) | Error Under Load (deg) |",
             "|---|---|---|---|---|"]
    for name, m, final_err in rows:
        settle = "never" if np.isnan(m["settling_time"]) else f"{m['settling_time'] * 1000:.0f}"
        lines.append(f"| {name} | {m['rise_time'] * 1000:.0f} | {m['overshoot_pct']:.1f} | {settle} | {final_err:.2f} |")
    Path(path).write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    FIG_DIR.mkdir(exist_ok=True)
    write_table(plot_gain_comparison())
    plot_anti_windup()
    print(f"Figures saved to {FIG_DIR}/")
