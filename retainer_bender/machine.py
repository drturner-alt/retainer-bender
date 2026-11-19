"""Turn a bend list into machine commands, and check it against the machine.

Machine frame, seen from above: the wire leaves the die at the origin
travelling +x. The bend arm turns about the die's vertical axis and carries
two pins, one each side of the wire, at +/- pin_angle from +x. Turning the
arm counter-clockwise brings the lower pin up against the wire and bends the
free end counter-clockwise (+); clockwise does the opposite (-).

The command language the firmware understands (one per line, `ok` back):
    HOME           home the feed stage and the arm
    FEED <mm>      push the wire out by mm (relative)
    ARM <deg>      turn the arm to an absolute angle, 0 = neutral
Lines starting with ';' are comments and are not sent.
"""

import json
import math
from dataclasses import dataclass, fields
from pathlib import Path

import numpy as np

from .springback import CONFIG_DIR


@dataclass
class Machine:
    feed_travel_mm: float = 120.0
    lead_in_mm: float = 0.0
    arm_radius_mm: float = 6.0
    pin_angle_deg: float = 60.0
    pin_diameter_mm: float = 3.0
    contact_trim_deg: float = 0.0
    arm_max_deg: float = 100.0
    body_start_mm: float = 0.5
    body_halfwidth_mm: float = 3.0
    clearance_mm: float = 0.15
    min_segment_mm: float = 1.0
    max_segment_mm: float = 6.0
    max_bend_deg: float = 35.0
    path_tolerance_mm: float = 0.04

    @classmethod
    def load(cls, path=None):
        path = Path(path) if path else CONFIG_DIR / "machine.json"
        data = json.loads(path.read_text())
        names = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in data.items() if k in names})

    def contact_deg(self, wire_d):
        """Arm angle at which a pin first touches the wire."""
        reach = wire_d / 2 + self.pin_diameter_mm / 2
        if reach >= self.arm_radius_mm * math.sin(math.radians(self.pin_angle_deg)):
            raise ValueError("pins sit on the wire at neutral; increase arm radius or pin angle")
        return (self.pin_angle_deg - math.degrees(math.asin(reach / self.arm_radius_mm))
                + self.contact_trim_deg)

    def pins_at(self, arm_deg=0.0):
        R = self.arm_radius_mm
        out = []
        for a in (self.pin_angle_deg, -self.pin_angle_deg):
            t = math.radians(a + arm_deg)
            out.append((R * math.cos(t), R * math.sin(t)))
        return np.array(out)


def machine_bends(turns_rad):
    """Planned turns (walking the wire tip-first) -> machine bend angles.

    The wire leaves the die tip first, so walking out from the die toward the
    tip retraces the plan backwards, which flips the sign of every turn.
    Whether the result is the planned wire or its mirror image depends only
    on which face is up, and a flat wire turned over is its mirror image.
    """
    return -np.degrees(np.asarray(turns_rad))


def make_program(lengths, turns_rad, wire, machine, springback):
    """Build the command list. Returns (lines, steps) where steps are dicts."""
    bends = machine_bends(turns_rad)
    contact = machine.contact_deg(wire["d_mm"])
    steps = []
    lines = [
        "; retainer-bender program",
        f"; wire {wire['label']}  ({wire['d_mm']:.3f} mm)",
        f"; springback table: {springback.source or 'built-in'}"
        + ("" if springback.calibrated else "  ** UNCALIBRATED PLACEHOLDER **"),
        f"; total wire {float(np.sum(lengths)) + machine.lead_in_mm:.2f} mm, {len(bends)} bends",
        "; load: tip flush with the die exit, clamp closed, arm at neutral",
    ]
    if machine.lead_in_mm:
        lines.append(f"FEED {machine.lead_in_mm:.3f}")
    for k, L in enumerate(lengths):
        lines.append(f"FEED {L:.3f}")
        steps.append({"op": "feed", "mm": float(L)})
        if k < len(bends):
            b = float(bends[k])
            cmd = springback.command_for(abs(b))
            arm = math.copysign(contact + cmd, b) if cmd > 0 else 0.0
            steps.append({"op": "bend", "target_deg": b, "command_deg": cmd, "arm_deg": arm})
            if arm == 0.0:
                lines.append(f"; bend {k + 1}: {b:+.2f} deg is too small to make, skipped")
                continue
            lines.append(f"ARM {arm:.2f}    ; bend {k + 1}: keep {b:+.2f} deg")
            lines.append("ARM 0")
    lines.append("; done: cut the wire at the die exit")
    return lines, steps


def free_wire(fed_lengths, kept_bends_deg, partial):
    """Shape of the wire beyond the die, walking from the die out to the tip.

    fed_lengths: whole segments already out (tip first); kept_bends_deg: the
    angle kept at the bend after each of those segments; partial: how much of
    the current segment is out.
    """
    pts = [np.zeros(2)]
    h = 0.0
    seq = [partial] + list(fed_lengths[::-1])
    bends = list(kept_bends_deg[::-1])
    for k, L in enumerate(seq):
        if k > 0:
            h += math.radians(bends[k - 1])
        pts.append(pts[-1] + L * np.array([math.cos(h), math.sin(h)]))
    return np.array(pts)


def _dist_points_to_polyline(points, poly):
    from .geometry import point_to_polyline
    if len(poly) < 2:
        return np.full(len(points), np.inf)
    return point_to_polyline(points, poly)


def check_program(lengths, turns_rad, wire, machine, springback, step_mm=0.25):
    """Walk the program and report anything the machine cannot do.

    Checks the feed travel, the arm's reach, the size of each bend, and that
    the wire already out never runs into a pin (at neutral, while feeding) or
    back into the machine body behind the die.
    """
    problems = []
    bends = machine_bends(turns_rad)
    r = wire["d_mm"] / 2
    total = float(np.sum(lengths)) + machine.lead_in_mm
    if total > machine.feed_travel_mm:
        problems.append(f"needs {total:.1f} mm of feed, stage travel is {machine.feed_travel_mm:.1f} mm")
    contact = machine.contact_deg(wire["d_mm"])
    for k, b in enumerate(bends):
        if abs(b) > machine.max_bend_deg:
            problems.append(f"bend {k + 1} is {b:+.1f} deg, over the {machine.max_bend_deg:.0f} deg limit")
        arm = contact + springback.command_for(abs(b))
        if arm > machine.arm_max_deg:
            problems.append(f"bend {k + 1} needs the arm at {arm:.0f} deg, past its {machine.arm_max_deg:.0f} deg limit")

    pins = machine.pins_at(0.0)
    pin_clear = machine.pin_diameter_mm / 2 + r + machine.clearance_mm
    kept = [springback.kept(springback.command_for(abs(b))) * np.sign(b) for b in bends]
    hit_pin = hit_body = None
    for k, L in enumerate(lengths):
        for s in np.arange(step_mm, L + 1e-9, step_mm):
            shape = free_wire(lengths[:k], kept[:k], s)
            # the straight piece in the die's own line cannot hit a pin unless
            # the pins are on the wire, which contact_deg already refuses
            if hit_pin is None and np.any(_dist_points_to_polyline(pins, shape) < pin_clear):
                hit_pin = (k + 1, s)
            dense = shape
            inside = ((dense[:, 0] < -machine.body_start_mm)
                      & (np.abs(dense[:, 1]) < machine.body_halfwidth_mm + r))
            if hit_body is None and inside.any():
                hit_body = (k + 1, s)
    if hit_pin:
        problems.append(f"wire already bent hits a bend pin while feeding segment {hit_pin[0]} "
                        f"(at {hit_pin[1]:.2f} mm); try a larger pin angle or arm radius")
    if hit_body:
        problems.append(f"free end swings back into the machine body while feeding segment "
                        f"{hit_body[0]}; narrow the guide behind the die")
    return problems
