"""Springback: the angle you command is not the angle the wire keeps.

Stainless wire bent around a post springs partly back when the pin lets go.
For the small bends a retainer is made of (mostly 3-20 degrees) springback is
a large share of each bend, so every machine/wire combination needs its own
table, measured with `rbend calibrate`.

A table is a list of [commanded_deg, measured_deg] pairs. Commanded means
degrees the arm turned *past first touching the wire*; measured means the
angle the wire kept.
"""

import json
from pathlib import Path

import numpy as np

CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"


class Springback:
    def __init__(self, table, wire="", calibrated=False, source=""):
        t = np.array(sorted(table), float)
        if t.ndim != 2 or t.shape[1] != 2 or len(t) < 2:
            raise ValueError("springback table needs at least two [commanded, measured] pairs")
        if np.any(np.diff(t[:, 1]) <= 0):
            raise ValueError("measured angles must rise with commanded angles "
                             "(below the elastic limit, list only one 0-degree row)")
        self.cmd, self.meas = t[:, 0], t[:, 1]
        self.wire = wire
        self.calibrated = calibrated
        self.source = source

    @classmethod
    def load(cls, path):
        data = json.loads(Path(path).read_text())
        return cls(data["table"], data.get("wire", ""), bool(data.get("calibrated")), str(path))

    @classmethod
    def default_for(cls, wire_key):
        return cls.load(CONFIG_DIR / f"springback_ss_{wire_key}.json")

    def _extrap(self, x, xs, ys):
        if x <= xs[-1]:
            return float(np.interp(x, xs, ys))
        slope = (ys[-1] - ys[-2]) / (xs[-1] - xs[-2])
        return float(ys[-1] + slope * (x - xs[-1]))

    def kept(self, commanded_deg):
        """Angle the wire keeps for a commanded angle."""
        return self._extrap(commanded_deg, self.cmd, self.meas)

    def command_for(self, target_deg):
        """Commanded angle needed for the wire to keep `target_deg`."""
        if target_deg <= 0:
            return 0.0
        return self._extrap(target_deg, self.meas, self.cmd)
