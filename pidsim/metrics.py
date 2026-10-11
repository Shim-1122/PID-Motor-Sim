"""Standard step-response metrics, computed from sampled data."""
import numpy as np


def step_metrics(t: np.ndarray, y: np.ndarray, setpoint: float, band: float = 0.02) -> dict:
    """
    rise_time      10% -> 90% of setpoint [s]
    overshoot_pct  peak above setpoint, as % of setpoint
    settling_time  time after which y stays within +/- band of setpoint [s]
    ss_error       setpoint minus the mean of the last 5% of samples

    Rise and settling time are NaN if the response never gets there.
    """
    t = np.asarray(t)
    y_n = np.asarray(y) / setpoint                    # normalize: target is 1 for any setpoint

    reached_90 = y_n.max() >= 0.9
    rise_time = t[np.argmax(y_n >= 0.9)] - t[np.argmax(y_n >= 0.1)] if reached_90 else np.nan

    outside = np.flatnonzero(np.abs(y_n - 1) > band)
    if outside.size == 0:
        settling_time = 0.0
    elif outside[-1] == len(t) - 1:                   # still outside at the end
        settling_time = np.nan
    else:
        settling_time = t[outside[-1] + 1]

    tail = y_n[-max(1, len(y_n) // 20):]
    return {
        "rise_time": rise_time,
        "overshoot_pct": max(0.0, y_n.max() - 1) * 100,
        "settling_time": settling_time,
        "ss_error": (1 - tail.mean()) * setpoint,
    }
