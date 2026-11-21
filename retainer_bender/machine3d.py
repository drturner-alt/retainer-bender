"""3D wires: feed, roll and bend.

A fourth motor turns the clamp that holds the back of the wire, rolling the
whole wire about its own axis. Bends still happen in the machine's
horizontal plane; rolling the wire first puts each bend in the plane it
belongs in.

Machine frame as in machine.py: the wire leaves the die at the origin along
+x, z is up. The free wire is tracked as a 3D polyline from the die out to
the tip. Operations:
    FEED L     everything already out moves +x by L, and a straight piece of
               length L appears between the die and it
    ROT a      the wire turns about the x axis to absolute angle a
    bend b     everything already out turns about the vertical axis through
               the die exit by b

How the roll for each bend is found: carry a reference direction along the
planned wire from its tail end by parallel transport (the wire is never
twisted, so this is the material's own orientation). At each joint, the
bend axis (the cross product of the two segment directions) must point
straight up or straight down when the pin pushes. That fixes the roll up to
half a turn; of the two choices, take the one that turns the wire least
and keeps what is already bent out of the deck.
"""

import math

import numpy as np

from . import geometry as g
from .machine import machine_bends  # noqa: F401  (re-exported for callers)


def material_frames(V):
    """Unit directions t[j] of each segment (tip first) and a material
    reference u[j] perpendicular to each, parallel-transported from the tail."""
    t = np.diff(V, axis=0)
    t /= np.linalg.norm(t, axis=1, keepdims=True)
    n = len(t)
    u = np.zeros_like(t)
    ref = np.array([0.0, 0.0, 1.0])            # the plan's normal
    if abs(np.dot(ref, t[-1])) > 0.9:
        ref = np.array([0.0, 1.0, 0.0])
    u[-1] = ref - np.dot(ref, t[-1]) * t[-1]
    u[-1] /= np.linalg.norm(u[-1])
    for j in range(n - 2, -1, -1):
        u[j] = g.rotation_between(t[j + 1], t[j]) @ u[j + 1]
        u[j] -= np.dot(u[j], t[j]) * t[j]
        u[j] /= np.linalg.norm(u[j])
    return t, u


def machine_map(t, u, roll_deg):
    """Rotation taking plan directions to machine directions for a segment at
    the die with direction t (walking tip to tail) and material reference u,
    with the wire rolled to roll_deg."""
    w = np.cross(t, u)
    r = math.radians(roll_deg)
    img_t = np.array([-1.0, 0, 0])
    img_u = np.array([0, math.cos(r), math.sin(r)])
    img_w = np.array([0, math.sin(r), -math.cos(r)])
    return np.outer(img_t, t) + np.outer(img_u, u) + np.outer(img_w, w)


def bend_options(t, u, j):
    """The two (roll, signed bend) choices for the joint between segment j-1
    and j (j counts from the tip). Returns [] for a straight joint."""
    c = np.cross(t[j], t[j - 1])
    s = np.linalg.norm(c)
    if s < 1e-9:
        return []
    b = c / s
    w = np.cross(t[j], u[j])
    alpha = math.degrees(math.atan2(np.dot(b, w), np.dot(b, u[j])))
    out = []
    for roll in (alpha + 90.0, alpha - 90.0):
        M = machine_map(t[j], u[j], roll)
        d = M @ (-t[j - 1])                         # where the free end must point after the bend
        out.append((roll, math.degrees(math.atan2(d[1], d[0]))))
    return out


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def _densify(P, step=0.3):
    if len(P) < 2:
        return P
    return g.resample(P, step)


def keepout_hits(P, machine, r, arm_deg=0.0):
    """Which machine parts the free wire (machine coords, die outward, wire
    centre at z = 0 on the die tongue) runs into. arm_deg: where the arm, and
    so the lowered pin, sits."""
    if len(P) < 2:
        return []
    Q = _densify(P)
    hits = []
    x, y, z = Q[:, 0], Q[:, 1], Q[:, 2]
    eps = 0.05
    # die tongue: top face level with the wire's underside
    hw = np.where(x > -11.0, machine.die_halfwidth_mm, machine.tongue_tail_halfwidth_mm)
    on_tongue = (x > -40.0) & (((x <= 0) & (np.abs(y) < hw + r))
                               | ((x > 0) & (np.hypot(x, y) < machine.die_halfwidth_mm + r)))
    if (on_tongue & (z < -eps)).any():
        hits.append("the die tongue")
    cx0, cx1, chw, ctop = machine.clamp_box_mm
    if ((x > cx0) & (x < cx1) & (np.abs(y) < chw + r) & (z < ctop + r)).any():
        hits.append("the tube clamp")
    deck = ((x > machine.deck_x0_mm) & (x < machine.deck_x1_mm)
            & (np.abs(y) < machine.deck_halfwidth_mm) & (z < -machine.deck_drop_mm))
    if deck.any():
        hits.append("the deck")
    px, py = machine.pin_at(arm_deg)
    pin = (np.hypot(x - px, y - py) < machine.pin_diameter_mm / 2 + r) & (z < -machine.pin_down_drop_mm)
    if pin.any():
        hits.append("the lowered bend pin")
    if (z < -machine.plate_depth_mm).any():
        hits.append("the top plate")
    return hits


def _segments2d(P, zmax):
    """xy segments of the wire low enough for the raised pin to meet."""
    segs = []
    for a, b in zip(P[:-1], P[1:]):
        if max(a[2], b[2]) < zmax:
            segs.append((a[:2], b[:2]))
    return segs


def _dist(p, segs):
    if not segs:
        return np.inf
    A = np.array([s[0] for s in segs])
    B = np.array([s[1] for s in segs])
    AB = B - A
    t = np.clip(((p - A) * AB).sum(1) / np.maximum((AB ** 2).sum(1), 1e-18), 0, 1)
    return float(np.min(np.linalg.norm(A + t[:, None] * AB - p, axis=1)))


def find_contact3d(P, direction, machine, r, step=0.05):
    """As machine.find_contact, but the free wire may leave the bending plane:
    parts of it higher than the raised pin's top are passed under."""
    from .machine import wire_crossing
    zmax = machine.pin_up_top_mm - 0.05            # wire centre height the pin can still meet
    segs = _segments2d(P, zmax)
    if not segs:
        return "no wire within reach of the pin"
    low = [P[0]]
    for a, b in zip(P[:-1], P[1:]):               # the low part, as one polyline from the die
        if max(a[2], b[2]) >= zmax:
            break
        low.append(b)
    low2 = np.array(low)[:, :2]
    cross = wire_crossing(low2, machine.arm_radius_mm) if len(low2) > 1 else None
    if cross is None:
        return "not enough wire out of the die for the pin to reach; raise lead_in_mm"
    touch = machine.pin_diameter_mm / 2 + r
    clear = touch + machine.clearance_mm
    start = None
    for a in np.arange(0, 120, 0.5):
        ang = cross - direction * a
        if _dist(machine.pin_at(ang), segs) > clear:
            start = ang
            break
    if start is None:
        return "nowhere to raise the pin clear of the wire"
    a = start
    for _ in range(int(180 / step)):
        a += direction * step
        if _dist(machine.pin_at(a), segs) <= touch:
            # how high off the bending plane the pin meets the wire: pushing
            # there would roll the wire in the clamp instead of bending it
            p = machine.pin_at(a)
            best = None
            for (a2, b2), (a3, b3) in zip(segs, [(P[i], P[i + 1]) for i in range(len(P) - 1)
                                                if max(P[i][2], P[i + 1][2]) < zmax]):
                ab = b2 - a2
                tt = np.clip(np.dot(p - a2, ab) / max(np.dot(ab, ab), 1e-18), 0, 1)
                dd = np.linalg.norm(a2 + tt * ab - p)
                if best is None or dd < best[0]:
                    best = (dd, a3[2] + tt * (b3[2] - a3[2]))
            if best is not None and abs(best[1]) > machine.max_push_height_mm:
                return (f"the pin would meet the wire {best[1]:.1f} mm off the bending plane "
                        "and twist it")
            contact = a + direction * machine.contact_trim_deg
            approach = contact - direction * machine.approach_margin_deg
            if _dist(machine.pin_at(approach), segs) <= clear:
                approach = start
            return approach, contact
    return "the pin never reaches the wire"


def make_program_3d(V, wire, machine, springback, roll0=None):
    """Program for a 3D polyline V (plan frame, tip first).

    Tries both starting rolls (the wire built one way up or the other) and
    returns the first that runs clean: (lines, steps, problems, final shape
    in machine coordinates).
    """
    best = None
    for start in ((roll0,) if roll0 is not None else (90.0, -90.0)):
        out = _program_3d(V, wire, machine, springback, start)
        if not out[2]:
            return out
        if best is None or len(out[2]) < len(best[2]):
            best = out
    return best


def _program_3d(V, wire, machine, springback, roll_start):
    r = wire["d_mm"] / 2
    t, u = material_frames(V)
    L = np.linalg.norm(np.diff(V, axis=0), axis=1)
    L = L.copy()
    L[0] += machine.lead_in_mm
    n = len(L)
    lines, steps, problems = [], [], []
    if float(L.sum()) > machine.feed_travel_mm:
        problems.append(f"needs {L.sum():.1f} mm of feed, stage travel is {machine.feed_travel_mm:.1f} mm")

    P = np.zeros((1, 3))          # free wire, die outward (just the die point)
    roll = roll_start
    arm_now = 0.0
    lines.append(f"ROT {roll:.2f}")
    steps.append({"op": "rot", "deg": roll})
    reported = set()

    def note(msg):
        if msg not in reported:
            reported.add(msg)
            problems.append(msg)

    for k in range(n):
        # feed segment k, checking as it goes
        for s in np.linspace(0, L[k], max(2, int(L[k] / 0.5) + 1))[1:]:
            Q = np.vstack([np.zeros(3), P + [s, 0, 0]])
            for h in keepout_hits(Q, machine, r, arm_now):
                note(f"while feeding segment {k + 1} the wire runs into {h}")
        P = np.vstack([np.zeros(3), P + [L[k], 0, 0]])
        lines.append(f"FEED {L[k]:.3f}")
        steps.append({"op": "feed", "mm": float(L[k])})
        if k == n - 1:
            break
        j = k + 1                                   # joint between segment k and k+1
        opts = bend_options(t, u, j)
        if not opts:
            steps.append({"op": "bend", "target_deg": 0.0, "command_deg": 0.0, "kept_deg": 0.0})
            continue

        def score(opt):
            new_roll = roll + _wrap(opt[0] - roll)
            turn = abs(new_roll - roll)
            worst = 0.0
            for a in np.linspace(roll, new_roll, max(2, int(turn / 10) + 2)):
                Q = P @ g.rot_x(a - roll).T
                worst += len(keepout_hits(Q, machine, r, arm_now))
            Q = (P @ g.rot_x(new_roll - roll).T) @ g.rot_z(opt[1]).T
            worst += len(keepout_hits(Q, machine, r, arm_now))
            return (worst, round(turn, 3))

        opt = min(opts, key=score)
        new_roll = roll + _wrap(opt[0] - roll)
        if abs(new_roll - roll) > 1e-6:
            for a in np.linspace(roll, new_roll, max(2, int(abs(new_roll - roll) / 10) + 2)):
                for h in keepout_hits(P @ g.rot_x(a - roll).T, machine, r, arm_now):
                    note(f"rolling the wire before bend {j} swings it into {h}")
            P = P @ g.rot_x(new_roll - roll).T
            roll = new_roll
            lines.append(f"ROT {roll:.2f}")
            steps.append({"op": "rot", "deg": roll})
        target = opt[1]
        cmd = math.copysign(springback.command_for(abs(target)), target)
        kept = math.copysign(springback.kept(abs(cmd)), cmd) if cmd else 0.0
        if abs(cmd) < 1e-6:
            steps.append({"op": "bend", "target_deg": target, "command_deg": 0.0, "kept_deg": 0.0})
            lines.append(f"; bend {j}: too small to make, skipped")
            continue
        found = find_contact3d(P, 1 if cmd > 0 else -1, machine, r)
        if isinstance(found, str):
            note(f"bend {j}: {found}")
            approach = contact = 0.0
        else:
            approach, contact = found
        arm = contact + cmd
        from .machine import pin_hits_die
        sweep = np.linspace(approach, arm, max(2, int(abs(arm - approach)) + 1))
        if any(pin_hits_die(a, machine) for a in sweep):
            note(f"bend {j}: the raised pin would hit the die nose")
        if max(abs(approach), abs(arm)) > machine.arm_max_deg:
            note(f"bend {j} needs the arm at {max(abs(approach), abs(arm)):.0f} deg, past its "
                 f"{machine.arm_max_deg:.0f} deg limit")
        if abs(target) > machine.max_bend_deg:
            note(f"bend {j} is {target:+.1f} deg, over the {machine.max_bend_deg:.0f} deg limit")
        for h in keepout_hits(P @ g.rot_z(cmd).T, machine, r, arm):
            note(f"bend {j} swings the wire into {h}")
        P = P @ g.rot_z(kept).T
        arm_now = arm
        lines += [f"ARM {approach:.2f}", "PIN UP",
                  f"ARM {arm:.2f}    ; bend {j}: {cmd:+.2f} past contact at {contact:+.2f}; keep {target:+.2f}",
                  "PIN DOWN"]
        steps.append({"op": "bend", "target_deg": target, "command_deg": cmd, "kept_deg": kept,
                      "approach_deg": approach, "contact_deg": contact, "arm_deg": arm,
                      "roll_deg": roll})
    lines += ["ARM 0", "; done: cut the wire at the die exit"]
    return lines, steps, problems, P


def built_error(V, P, lead_in):
    """How far the simulated wire (machine coords, die outward, so tail first)
    is from the plan, after the best rigid fit; the lead-in is left out."""
    target = V[::-1]
    shape = P.copy()
    # the lead-in is the last stretch of the first segment out, at the tip end
    d = shape[-1] - shape[-2]
    shape[-1] = shape[-1] - d / np.linalg.norm(d) * lead_in
    moved = g.kabsch(shape, target)
    return float(np.max(np.linalg.norm(moved - target, axis=1)))
