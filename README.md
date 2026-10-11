# PID Motor Simulator

![Tests](https://github.com/Shim-1122/PID-Motor-Sim/actions/workflows/tests.yml/badge.svg)

Discrete PID position control of a DC motor with a 1 kHz loop, voltage saturation, anti-windup, filtered derivative, and encoder quantization. Includes a Streamlit app for interactive gain tuning.

![Gain Comparison](figures/gain_comparison.png)

## Results

90° step; 0.05 N·m load (17% of stall torque at 12 V) applied at 0.6 s; 2048-count encoder. Step metrics are computed before the load step. Error under load is the mean over the final 50 ms.

| Controller | Rise Time (ms) | Overshoot (%) | Settling Time, 2% (ms) | Error Under Load (deg) |
|---|---|---|---|---|
| P (Kp = 25) | 17 | 60.4 | 533 | 4.58 |
| PD (+ Kd = 0.3) | 24 | 0.0 | 52 | 4.63 |
| PID (+ Ki = 100) | 23 | 2.5 | 147 | 0.10 |

- **P:** fast but underdamped. The steady error under load equals R·τ/(K·Kp), verified in the tests.
- **PD:** the derivative term removes the oscillation. Load error is unchanged because dy/dt = 0 at rest.
- **PID:** the integral term drives the load error to near zero.
- The voltage ripple in the lower plot comes from one-count encoder steps passing through the derivative term; the derivative filter limits it.

### Integrator Windup

![Anti-Windup](figures/anti_windup.png)

A 3-revolution move saturates the output at 12 V for about 0.1 s. Without anti-windup, the integrator accumulates error during saturation, causing 29.2% overshoot and a slow return. Clamping reduces overshoot to 0.9%.

## Model

Armature-controlled DC motor, state x = [θ, ω, i], input u = [V, τ_load]:

```
J·dω/dt = K·i − b·ω − τ_load
L·di/dt = V − R·i − K·ω
```

| R | L | K | J | b |
|---|---|---|---|---|
| 2 Ω | 1 mH | 0.05 N·m/A | 5×10⁻⁵ kg·m² | 1×10⁻⁵ N·m·s/rad |

The plant is discretized with a zero-order hold (`scipy.signal.cont2discrete`). The controller output is constant between samples, so for this linear plant the discrete model is exact at every sample, and each control step is a single 3x3 matrix update.

## Controller

`pidsim/pid.py` implements `u = Kp·e + Ki·∫e − Kd·dy/dt` with:

1. **Derivative on measurement** instead of error, so setpoint steps do not cause an output spike
2. **Low-pass filtered derivative** (2 ms time constant) to limit encoder noise amplification
3. **Output saturation** at the supply voltage
4. **Anti-windup by clamping:** integration pauses while the output is saturated and the error would increase saturation

## Usage

```bash
git clone https://github.com/Shim-1122/PID-Motor-Sim.git
cd PID-Motor-Sim
python -m venv .venv && source .venv/bin/activate      # Windows: .venv\Scripts\activate
pip install -r requirements.txt
streamlit run app.py      # interactive tuner
python make_figures.py    # regenerate figures/ and results.md
pytest
```

The app deploys to Streamlit Community Cloud with `app.py` as the main file.

## Project Structure

```
pidsim/
  motor.py        state-space DC motor, ZOH discretization
  pid.py          discrete PID with saturation, anti-windup, filtered derivative
  sim.py          closed-loop simulation with encoder and load disturbance
  metrics.py      rise time, overshoot, settling time, steady-state error
app.py            Streamlit tuner
make_figures.py   figures and results table
tests/
```

## Tests

- **Plant:** open-loop speed matches K·V/(R·b + K²); stall is an exact equilibrium
- **Metrics:** overshoot matches exp(−πζ/√(1−ζ²)) for an analytic second-order response; rise and settling times match τ·ln 9 and τ·ln 50 for a first-order response
- **Controller:** P-only load error matches R·τ/(K·Kp); integral action removes it; output stays within the supply; no derivative kick; anti-windup reduces overshoot; final error is within one encoder count
- **App:** runs headlessly through Streamlit's testing API and responds to slider input

## License

MIT
