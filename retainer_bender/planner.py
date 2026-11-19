"""From points picked on a scan to a flat wire and a bend list.

The picks run from one end of the wire to the other: the bond area on one
canine, the contact point on each incisor at about two-thirds crown height,
the bond area on the other canine. Each pick is a point on the tooth surface
and, from the picker, the surface normal there.

1. Move each pick out along its normal by the wire radius, so the wire's
   centre line passes where the wire would touch the tooth.
2. Fit the plane the wire will lie in, and flatten the centres onto it. How
   far each centre had to move is reported: a flat wire cannot reach a
   contact that sits well above or below the others.
3. Run a smooth curve (or straight lines, for `style="polyline"`) through
   the flattened centres.
4. Replace the curve with straight segments the machine can make, and work
   out the length of each and the angle at each joint.
"""

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import geometry as g


@dataclass
class Pick:
    p: np.ndarray
    n: np.ndarray | None = None
    label: str = ""


def load_picks(path):
    data = json.loads(Path(path).read_text())
    out = []
    for k, item in enumerate(data["points"]):
        n = item.get("n")
        out.append(Pick(np.array(item["p"], float),
                        None if n is None else np.array(n, float) / np.linalg.norm(n),
                        item.get("label") or f"P{k + 1}"))
    if len(out) < 3:
        raise SystemExit("need at least three picked points (both canines and the incisors between)")
    return out


@dataclass
class Plan:
    wire: dict
    style: str
    origin: np.ndarray
    e_u: np.ndarray
    e_v: np.ndarray
    normal: np.ndarray
    labels: list
    contacts_2d: np.ndarray          # picks flattened onto the plane
    centers_2d: np.ndarray           # wire centre at each pick
    out_of_plane: np.ndarray         # how far each centre was flattened (mm)
    curve: np.ndarray                # dense target centre line
    vertices: np.ndarray             # bend points
    lengths: np.ndarray
    turns: np.ndarray                # radians, + = left walking from the first pick
    path_error: float
    warnings: list = field(default_factory=list)

    def to_3d(self, uv):
        uv = np.asarray(uv, float)
        return self.origin + uv[..., :1] * self.e_u + uv[..., 1:2] * self.e_v

    @property
    def total_length(self):
        return float(self.lengths.sum())


def plan_wire(picks, wire, machine, style="smooth", contact_offset=0.0, mesh=None, resolve=True):
    r = wire["d_mm"] / 2
    warnings = []

    normals = []
    for pk in picks:
        n = pk.n
        if n is None and mesh is not None:
            from .mesh import nearest_normal
            n = nearest_normal(pk.p, mesh)
        normals.append(n)
    have_normals = all(n is not None for n in normals)

    P = np.array([pk.p for pk in picks])
    for a, b, la, lb in zip(P[:-1], P[1:], [pk.label for pk in picks][:-1],
                            [pk.label for pk in picks][1:]):
        if np.linalg.norm(b - a) < 0.3:
            raise SystemExit(f"picks {la} and {lb} are under 0.3 mm apart; remove one")

    C3 = P + (np.array(normals) * (r + contact_offset) if have_normals else 0.0)

    origin, normal = g.fit_plane(C3)
    chord = C3[-1] - C3[0]
    e_u = chord - np.dot(chord, normal) * normal
    e_u /= np.linalg.norm(e_u)
    e_v = np.cross(normal, e_u)
    rel = C3 - origin
    uv = np.column_stack([rel @ e_u, rel @ e_v])
    # Point +v toward the front teeth: the incisors bulge away from the line
    # joining the two canine ends.
    mid = uv[1:-1, 1].mean() - (uv[0, 1] + uv[-1, 1]) / 2
    if mid < 0:
        e_v, uv[:, 1] = -e_v, -uv[:, 1]
    normal = np.cross(e_u, e_v)
    off = rel @ normal
    contacts = np.column_stack([(P - origin) @ e_u, (P - origin) @ e_v])

    if not have_normals:
        # No normals: offset in the plane, toward the tongue side.
        warnings.append("picks have no surface normals; the wire was offset in its own plane")
        dense = g.catmull_rom(uv) if style == "smooth" else uv
        nrm = g.left_normals(dense)
        centre = nrm[:, 1].mean()
        sign = -1 if centre > 0 else 1   # move toward -v (lingual)
        idx = [int(np.argmin(np.linalg.norm(dense - q, axis=1))) for q in uv]
        uv = uv + sign * nrm[idx] * (r + contact_offset)

    for lab, d in zip([pk.label for pk in picks], off):
        if abs(d) > 0.35:
            warnings.append(f"{lab}: the flat wire passes {d:+.2f} mm off this contact "
                            "(out of its plane); the contact will be loose or tight")

    if style == "smooth":
        dense = g.catmull_rom(uv, samples_per_seg=400)
    elif style == "polyline":
        dense = uv
    else:
        raise SystemExit(f"unknown style {style!r}")
    curve = g.resample(dense, 0.02)
    if mesh is not None and resolve:
        curve, pushed = push_clear(curve, origin, e_u, e_v, r, mesh)
        if pushed > 0.005:
            warnings.append(f"wire moved up to {pushed:.2f} mm off the teeth where the "
                            "curve between picks would have cut into them")

    idx, err = g.fit_polyline(curve, machine.path_tolerance_mm,
                              machine.min_segment_mm, machine.max_segment_mm)
    if style == "polyline":
        # keep every pick as a bend point
        pick_idx = [int(np.argmin(np.linalg.norm(curve - q, axis=1))) for q in uv]
        idx = np.unique(np.concatenate([idx, pick_idx]))
    vertices = curve[idx]
    lengths = g.segment_lengths(vertices)
    turns = g.turning_angles(vertices)

    signs = np.sign(np.round(np.degrees(turns), 1))
    flips = int(np.sum(signs[1:] * signs[:-1] < 0))
    if flips:
        warnings.append(f"bend direction changes {flips} time(s); expected where the wire "
                        "wraps a canine, and where a tooth sits lingual of its neighbours")

    return Plan(wire=wire, style=style, origin=origin, e_u=e_u, e_v=e_v, normal=normal,
                labels=[pk.label for pk in picks], contacts_2d=contacts, centers_2d=uv,
                out_of_plane=off, curve=curve, vertices=vertices, lengths=lengths,
                turns=turns, path_error=err, warnings=warnings)


def push_clear(curve, origin, e_u, e_v, r, mesh, step=0.2, iters=8, sigma=0.6):
    """Move the wire out of the teeth wherever it cuts into them.

    The curve through the picks is free to bow between them, and across a
    convex surface (a canine) it cuts in. Each pass measures the clearance
    along the wire, pushes offending stretches out along the tooth's surface
    normal (flattened into the wire's plane) by the shortfall, smooths the
    push so it adds no kinks, and repeats. Places with a gap are left alone.
    Returns the new curve and the largest total push.
    """
    from .mesh import signed_distances
    curve = curve.copy()
    s_dense = g.arc_lengths(curve)
    total = np.zeros(len(curve))
    for _ in range(iters):
        samples = g.resample(curve, step)
        s = g.arc_lengths(samples) * (s_dense[-1] / max(g.arc_lengths(samples)[-1], 1e-9))
        pts3 = origin + samples[:, :1] * e_u + samples[:, 1:2] * e_v
        d, n = signed_distances(pts3, mesh, return_normals=True)
        short = np.where(np.isfinite(d), r + 0.01 - d, 0.0)
        short[short < 0] = 0
        if short.max() < 0.005:
            break
        push = np.column_stack([n @ e_u, n @ e_v])
        L = np.linalg.norm(push, axis=1, keepdims=True)
        push = np.where(L > 1e-6, push / np.where(L > 1e-6, L, 1), 0) * short[:, None]
        # spread onto the dense curve and smooth along the wire
        disp = np.column_stack([np.interp(s_dense, s, push[:, k]) for k in range(2)])
        h = s_dense[1] - s_dense[0] if len(s_dense) > 1 else 1
        half = int(3 * sigma / h)
        x = np.arange(-half, half + 1) * h
        ker = np.exp(-0.5 * (x / sigma) ** 2)
        norm = np.convolve(np.ones(len(curve)), ker, "same")
        disp = np.column_stack([np.convolve(disp[:, k], ker, "same") / norm for k in range(2)])
        # smoothing spreads the push thinner; scale back up so the worst spot clears
        mag = np.linalg.norm(disp, axis=1)
        need = np.interp(s_dense, s, short)
        hot = need > 0.005
        if hot.any():
            disp *= max(1.0, float(np.max(need[hot] / np.maximum(mag[hot], 1e-9))))
        curve += disp
        total += np.linalg.norm(disp, axis=1)
    return g.resample(curve, 0.02), float(total.max())


def predicted_shape(plan, springback):
    """The wire the machine should make once springback is allowed for, aligned
    onto the target, and how far it strays from the target centre line."""
    bends = np.degrees(plan.turns)
    kept = np.array([np.sign(b) * springback.kept(springback.command_for(abs(b))) for b in bends])
    built = g.build_polyline(plan.lengths, np.radians(kept))
    built = g.align_2d(built, plan.vertices)
    dev = g.point_to_polyline(plan.curve, built)
    return built, float(dev.max())


def uncorrected_shape(plan, springback):
    """What you would get bending to the target angles with no correction."""
    bends = np.degrees(plan.turns)
    kept = np.array([np.sign(b) * springback.kept(abs(b)) for b in bends])
    built = g.build_polyline(plan.lengths, np.radians(kept))
    built = g.align_2d(built, plan.vertices)
    return built, float(g.point_to_polyline(plan.curve, built).max())


def clearance(plan, mesh, step=0.2):
    """Gap between wire surface and teeth along the wire (negative = into a tooth).

    Returns (arc positions, clearance per sample, clearance at each pick).
    """
    from .mesh import signed_distances
    samples = g.resample(plan.curve, step)
    s = g.arc_lengths(samples)
    d = signed_distances(plan.to_3d(samples), mesh) - plan.wire["d_mm"] / 2
    at_picks = []
    for q in plan.centers_2d:
        i = int(np.argmin(np.linalg.norm(samples - q, axis=1)))
        at_picks.append(float(d[i]))
    return s, d, np.array(at_picks)
