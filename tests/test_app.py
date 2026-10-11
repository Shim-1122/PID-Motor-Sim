"""Smoke test: the Streamlit app runs headlessly and reacts to slider changes."""
from pathlib import Path

from streamlit.testing.v1 import AppTest

APP = Path(__file__).resolve().parent.parent / "app.py"


def test_app_runs_and_updates():
    app = AppTest.from_file(str(APP), default_timeout=30).run()
    assert not app.exception
    overshoot_before = app.metric[1].value

    app.slider[2].set_value(0.0).run()        # remove derivative damping (Kd = 0)
    assert not app.exception
    assert app.metric[1].value != overshoot_before
