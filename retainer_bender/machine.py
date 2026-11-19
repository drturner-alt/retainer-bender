"""Turn a bend list into machine commands, and simulate it on the machine.

Machine frame, seen from above: the wire leaves the die at the origin
travelling +x. The bend arm turns about the die's vertical axis and carries
one bend pin at `arm_radius_mm`. The pin drops below the deck while the wire
feeds and while the arm repositions, so the wire never meets it except to
bend. For each bend the arm:

    1. turns (pin down) to an approach angle on the side opposite the bend,
    2. raises the pin,
    3. turns until the pin touches the wire, then on by the commanded angle,
    4. lowers the pin.

Where the pin first touches depends on the shape already out of the die (a
curled hook ends up near the pin's path), so it is found by simulation
rather than assumed.

Firmware commands (one per line, `ok` back):
    HOME           home the feed stage and the arm, pin down
    FEED <mm>      push the wire out by mm (refused unless the pin is down)
    ARM <deg>      turn the arm to an absolute angle, 0 = pin straight ahead
    PIN UP|DOWN    raise or lower the bend pin
Lines starting with ';' are comments and are not sent.
"""

import json
import math
from dataclasses import dataclass, fields
from pathlib import Path

import numpy as np

from .geometry import point_to_polyline
from .springback import CONFIG_DIR


@dataclass
class Machine:
    feed_travel_mm: float = 120.0
    lead_in_mm: float = 6.0
    arm_radius_mm: float = 5.0
    pin_diameter_mm: float = 3.0
    approach_margin_deg: float = 6.0
    contact_trim_deg: float = 0.0
    arm_max_deg: float = 150.0
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

    def pin_at(self, arm_deg):
        t = math.radians(arm_deg)
        return np.array([self.arm_radius_mm * math.cos(t), self.arm_radius_mm * math.sin(t)])


def machine_bends(turns_rad):
    """Planned turns (walking the wire tip-first) -> machine bend angles.

    The wire leaves the die tip first, so walking out from the die toward the
    tip retraces the plan backwards, which flips the sign of every turn.
    Whether the result is the planned wire or its mirror image depends only
    on which face is up, and a flat wire turned over is its mirror image.
    """
    return -np.degrees(np.asarray(turns_rad))


def free_wire(fed_lengths, kept_bends_deg, partial):
    """Shape of the wire beyond the die, walking from the die out to the tip.

    fed_lengths: whole segments already out (tip first); kept_bends_deg: the
    angle kept at the bend after each of those; partial: how much of the
    current segment is out.
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


def _rotate(pts, deg):
    t = math.radians(deg)
    R = np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])
    return pts @ R.T


def _in_body(pts, machine, r):
    return np.any((pts[:, 0] < -machine.body_start_mm)
                  & (np.abs(pts[:, 1]) < machine.body_halfwidth_mm + r))


def wire_crossing(shape, radius):
    """Angle (deg) where the wire first crosses the pin's circle, walking out
    from the die; None if no wire reaches that far."""
    d = np.linalg.norm(shape, axis=1)
    for i in range(len(shape) - 1):
        if d[i] <= radius < d[i + 1] or d[i + 1] <= radius < d[i]:
            a, b = shape[i], shape[i + 1]
            ab = b - a
            # solve |a + t ab| = radius for t in [0, 1]
            A, B, C = ab @ ab, 2 * a @ ab, a @ a - radius ** 2
            t = (-B + math.sqrt(max(B * B - 4 * A * C, 0))) / (2 * A)
            if not 0 <= t <= 1:
                t = (-B - math.sqrt(max(B * B - 4 * A * C, 0))) / (2 * A)
            p = a + t * ab
            return math.degrees(math.atan2(p[1], p[0]))
    return None


def find_contact(shape, direction, machine, r, step=0.05):
    """Where to raise the pin and where it first touches the wire.

    `direction` +1 bends counter-clockwise, so the pin comes from the
    clockwise side of where the wire crosses its circle. Returns
    (approach_deg, contact_deg), or a string saying why it can't.
    """
    touch = machine.pin_diameter_mm / 2 + r
    clear = touch + machine.clearance_mm
    cross = wire_crossing(shape, machine.arm_radius_mm)
    if cross is None:
        return "not enough wire out of the die for the pin to reach; raise lead_in_mm"
    dist = lambda ang: point_to_polyline(machine.pin_at(ang)[None], shape)[0]
    start = None
    for a in np.arange(0, 120, 0.5):
        ang = cross - direction * a
        if dist(ang) > clear:
            start = ang
            break
    if start is None:
        return "nowhere to raise the pin clear of the wire"
    a = start
    for _ in range(int(180 / step)):
        a += direction * step
        if dist(a) <= touch:
            contact = a + direction * machine.contact_trim_deg
            approach = contact - direction * machine.approach_margin_deg
            if dist(approach) <= clear:
                approach = start
            return approach, contact
    return "the pin never reaches the wire"


def simulate(lengths, commands_deg, kept_deg, wire, machine, notes=None):
    """Walk a program on the machine.

    commands_deg: signed commanded bend after each segment but the last
    (degrees past first contact); kept_deg: what the wire keeps.
    Returns (lines, steps, problems).
    """
    r = wire["d_mm"] / 2
    notes = notes or [""] * len(commands_deg)
    lines, steps, problems = [], [], []
    total = float(np.sum(lengths))
    if total > machine.feed_travel_mm:
        problems.append(f"needs {total:.1f} mm of feed, stage travel is {machine.feed_travel_mm:.1f} mm")
    hit_body = None
    for k, L in enumerate(lengths):
        for s in np.linspace(0, L, max(2, int(L / 0.25) + 1))[1:]:
            if hit_body is None and _in_body(free_wire(lengths[:k], kept_deg[:k], s), machine, r):
                hit_body = k + 1
        lines.append(f"FEED {L:.3f}")
        steps.append({"op": "feed", "mm": float(L)})
        if k >= len(commands_deg):
            break
        cmd = float(commands_deg[k])
        if abs(cmd) < 1e-6:
            lines.append(f"; bend {k + 1}: too small to make, skipped{notes[k]}")
            steps.append({"op": "bend", "command_deg": 0.0, "kept_deg": 0.0})
            continue
        direction = 1 if cmd > 0 else -1
        shape = free_wire(lengths[:k], kept_deg[:k], L)
        found = find_contact(shape, direction, machine, r)
        if isinstance(found, str):
            problems.append(f"bend {k + 1}: {found}")
            continue
        approach, contact = found
        target = contact + cmd
        if max(abs(approach), abs(target)) > machine.arm_max_deg:
            problems.append(f"bend {k + 1} needs the arm at {max(abs(approach), abs(target)):.0f} deg, "
                            f"past its {machine.arm_max_deg:.0f} deg limit")
        if abs(kept_deg[k]) > machine.max_bend_deg:
            problems.append(f"bend {k + 1} is {kept_deg[k]:+.1f} deg, over the {machine.max_bend_deg:.0f} deg limit")
        if _in_body(_rotate(shape, cmd), machine, r):
            problems.append(f"bend {k + 1}: the free end swings into the guide behind the die")
        lines += [f"ARM {approach:.2f}", "PIN UP",
                  f"ARM {target:.2f}    ; bend {k + 1}: {cmd:+.2f} past contact at {contact:+.2f}{notes[k]}",
                  "PIN DOWN"]
        steps.append({"op": "bend", "command_deg": cmd, "kept_deg": float(kept_deg[k]),
                      "approach_deg": approach, "contact_deg": contact, "arm_deg": target})
    if hit_body:
        problems.append(f"free end swings back into the guide while feeding segment {hit_body}")
    lines.append("ARM 0")
    lines.append("; done: cut the wire at the die exit")
    return lines, steps, problems


def make_program(lengths, turns_rad, wire, machine, springback):
    """Program for a planned wire. Returns (lines, steps, problems)."""
    bends = machine_bends(turns_rad)
    lengths = np.array(lengths, float).copy()
    lengths[0] += machine.lead_in_mm      # spare wire at the tip for the pin to push on
    cmds = np.array([math.copysign(springback.command_for(abs(b)), b) for b in bends])
    kept = np.array([math.copysign(springback.kept(abs(c)), c) if c else 0.0 for c in cmds])
    notes = [f"; keep {b:+.2f}" for b in bends]
    lines, steps, problems = simulate(lengths, cmds, kept, wire, machine, notes)
    j = 0
    for st in steps:
        if st["op"] == "bend":
            st["target_deg"] = float(bends[j])
            j += 1
    header = [
        "; retainer-bender program",
        f"; wire {wire['label']}  ({wire['d_mm']:.3f} mm)",
        f"; springback table: {springback.source or 'built-in'}"
        + ("" if springback.calibrated else "  ** UNCALIBRATED PLACEHOLDER **"),
        f"; total wire {float(np.sum(lengths)):.2f} mm, {len(bends)} bends",
        "; load: tip flush with the die exit, clamp closed, pin down",
        f"; afterwards trim {machine.lead_in_mm:.1f} mm of straight lead-in off the first end out",
    ]
    return header + lines, steps, problems
