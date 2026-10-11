"""Interactive PID tuner.  Run with:  streamlit run app.py"""
import matplotlib.pyplot as plt
import numpy as np
import streamlit as st

from pidsim import PID, DCMotor, simulate, step_metrics

st.set_page_config(page_title="PID Motor Tuner", layout="wide")
st.title("PID Position Control of a DC Motor")
st.caption("Closed-loop step response. Load torque is applied at t = 0.6 s.")

# ------------------------------------------------------------------ controls
with st.sidebar:
    st.header("Controller Gains")
    kp = st.slider("Kp (V/rad)", 0.0, 60.0, 25.0, 0.5)
    ki = st.slider("Ki (V/(rad s))", 0.0, 400.0, 100.0, 5.0)
    kd = st.slider("Kd (V s/rad)", 0.0, 1.0, 0.3, 0.01)
    anti_windup = st.checkbox("Anti-Windup", value=True)

    st.header("Scenario")
    setpoint_deg = st.slider("Target angle (deg)", 10, 1080, 90, 10)
    supply_v = st.slider("Supply voltage (V)", 3.0, 24.0, 12.0, 0.5)
    load = st.slider("Load torque (N m)", 0.0, 0.2, 0.05, 0.01)
    encoder = st.selectbox("Encoder resolution (counts/rev)", [None, 256, 1024, 2048, 8192], index=3,
                           format_func=lambda c: "perfect sensor" if c is None else str(c))

# ---------------------------------------------------------------- simulate
T_END, T_LOAD = 1.5, 0.6
motor = DCMotor()
pid = PID(kp, ki, kd, u_max=supply_v, anti_windup=anti_windup)
setpoint = np.radians(setpoint_deg)
r = simulate(motor, pid, setpoint, T_END, load, T_LOAD, encoder)

before_load = r.t < T_LOAD
m = step_metrics(r.t[before_load], r.theta[before_load], setpoint)
final_error_deg = np.degrees(setpoint - r.theta[-50:].mean())

# ------------------------------------------------------------------ results
fmt_ms = lambda s: "never" if np.isnan(s) else f"{s * 1000:.0f} ms"
cols = st.columns(4)
cols[0].metric("Rise Time (10 to 90%)", fmt_ms(m["rise_time"]))
cols[1].metric("Overshoot", f"{m['overshoot_pct']:.1f} %")
cols[2].metric("Settling Time (2%)", fmt_ms(m["settling_time"]))
cols[3].metric("Error Under Load", f"{final_error_deg:.2f} deg")

fig, (ax_pos, ax_v) = plt.subplots(2, 1, figsize=(10, 6), sharex=True, height_ratios=[2, 1])
ax_pos.plot(r.t, np.degrees(r.theta), lw=1.8, label="shaft angle")
ax_pos.axhline(setpoint_deg, color="k", ls="--", lw=0.8, label="target")
ax_pos.axvline(T_LOAD, color="0.5", ls=":", label="load applied")
ax_pos.set_ylabel("angle (deg)")
ax_pos.legend(loc="lower right")
ax_pos.grid(alpha=0.3)
ax_v.plot(r.t, r.voltage, color="C1", lw=1.0)
ax_v.axhline(supply_v, color="C3", ls="--", lw=0.8)
ax_v.axhline(-supply_v, color="C3", ls="--", lw=0.8)
ax_v.set_ylabel("voltage (V)")
ax_v.set_xlabel("time (s)")
ax_v.grid(alpha=0.3)
fig.tight_layout()
st.pyplot(fig)

with st.expander("Motor Model and Parameters"):
    st.latex(r"J\dot\omega = K i - b\omega - \tau_{load} \qquad L\dot i = V - R i - K\omega")
    st.write({"R (ohm)": motor.R, "L (H)": motor.L, "K (N m/A)": motor.K,
              "J (kg m^2)": motor.J, "b (N m s/rad)": motor.b,
              "stall torque at supply (N m)": round(motor.stall_torque(supply_v), 3)})
