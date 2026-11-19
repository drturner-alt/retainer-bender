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
    centers_2d: np.ndarray           # wire centre at each contact
    out_of_plane: np.ndarray         # how far each centre was flattened (mm)
    curve: np.ndarray                # dense target centre line
    vertices: np.ndarray             # bend points
    lengths: np.ndarray
    turns: np.ndarray                # radians, + = left walking from the first pick
    path_error: float
    warnings: list = field(default_factory=list)
    contact_labels: list = field(default_factory=list)

    def to_3d(self, uv):
        uv = np.asarray(uv, float)
        return self.origin + uv[..., :1] * self.e_u + uv[..., 1:2] * self.e_v

    @property
    def total_length(self):
        return float(self.lengths.sum())


@dataclass
class Hooks:
    """Retention curls at the two canine ends, in the wire's plane."""
    angle_deg: float = 90.0     # how far the end curls round; 0 = straight ends
    radius_mm: float = 2.0      # radius of the curl, to the wire centre
    toward: str = "tongue"      # "tongue" curls in toward the midline; "tooth" onto the canine


def add_hooks(curve, hooks, tooth_side_start, tooth_side_end, step=0.02):
    """Extend both ends of a 2D curve with circular curls.

    tooth_side_*: 2D direction from the wire toward the canine at each end.
    """
    if not hooks or hooks.angle_deg <= 0:
        return curve
    phi = np.radians(hooks.angle_deg)
    R = hooks.radius_mm
    n = max(2, int(np.ceil(R * phi / step)))

    def curl(end, outward, side):
        outward = outward / np.linalg.norm(outward)
        left = np.array([-outward[1], outward[0]])
        sgn = 1.0 if np.dot(left, side) >= 0 else -1.0
        if hooks.toward == "tongue":
            sgn = -sgn
        centre = end + sgn * R * left
        a0 = np.arctan2(*(end - centre)[::-1])
        a = a0 + sgn * np.linspace(0, phi, n + 1)[1:]
        return centre + R * np.column_stack([np.cos(a), np.sin(a)])

    k = max(2, int(0.3 / step))
    head = curl(curve[0], curve[0] - curve[k], tooth_side_start)[::-1]
    tail = curl(curve[-1], curve[-1] - curve[-1 - k], tooth_side_end)
    return np.vstack([head, curve, tail])


def _min_filter(x, k):
    pad = np.pad(x, k, mode="edge")
    return np.min(np.lib.stride_tricks.sliding_window_view(pad, 2 * k + 1), axis=1)


def _max_filter(x, k):
    pad = np.pad(x, k, mode="edge")
    return np.max(np.lib.stride_tricks.sliding_window_view(pad, 2 * k + 1), axis=1)


def lingual_outline(mesh, origin, e_u, e_v, normal, a, m, b, offset, r_wire, bridge_mm=4.0,
                    step=0.1, min_spacing=2.5, prominence=0.1):
    """Auto-design: find where the wire touches each tooth.

    The scan is cut at the wire's plane (and a wire radius above and below
    it). Rays fan out from a point behind the arch (the midline click
    mirrored across the canine-to-canine line), and along each the wire's
    centre can come out until it is `offset` from a tooth. Along that outline
    each tooth is a dip toward the tongue and each embrasure a bump away from
    it. The wire touches the bottom of each dip: those become its contact
    points, along with the two canine clicks at the ends. a, m, b are the
    three clicks in plane coordinates.

    Returns (contact points in order, the outline, warnings).
    """
    from .mesh import slice_plane
    warnings = []
    # The wire is a cylinder, so a tooth can reach it from anywhere within a
    # wire radius above or below its centre plane. Cut the scan at several
    # heights across that band; at height z a tooth must stay
    # sqrt(r^2 - z^2) away in the plane.
    pts, rad = [], []
    for z in np.linspace(-r_wire, r_wire, 9):
        segs = slice_plane(mesh, origin + z * normal, normal)
        if not len(segs):
            continue
        rel = segs - origin
        seg2 = np.stack([rel @ e_u, rel @ e_v], axis=-1)
        reach = np.sqrt(max(r_wire ** 2 - z ** 2, 0.0)) + (offset - r_wire)
        for p0, p1 in seg2:
            n = max(1, int(np.ceil(np.linalg.norm(p1 - p0) / 0.1)))
            t = np.linspace(0, 1, n + 1)[:, None]
            pts.append(p0 + t * (p1 - p0))
            rad.append(np.full(n + 1, reach))
    if not pts:
        raise SystemExit("the wire's plane does not cut the scan; check the three clicks")
    P = np.vstack(pts)
    rad = np.concatenate(rad)
    lo = np.minimum(a, b) - [6, 6]
    hi = np.maximum(a, b) + [6, 0]
    hi[1] = max(hi[1], m[1] + 6)
    keep = np.all((P >= lo) & (P <= hi), axis=1)
    P, rad = P[keep], rad[keep]

    chord_mid = (a + b) / 2
    depth = max(m[1] - chord_mid[1], 3.0)
    C = chord_mid - np.array([0.0, depth])
    angA = np.arctan2(*(a - C)[::-1])
    angB = np.arctan2(*(b - C)[::-1])
    # a is on the -u side, b on +u: the fan sweeps from angA down to angB through +v
    span = angA - angB
    radius = np.linalg.norm(m - C)
    n_rays = max(50, int(span * radius / step))
    angs = np.linspace(angA, angB, n_rays)
    D = np.column_stack([np.cos(angs), np.sin(angs)])
    rel = P - C
    along = D @ rel.T                                   # (rays, pts)
    perp = D[:, 0:1] * rel[:, 1] - D[:, 1:2] * rel[:, 0]
    hit = (np.abs(perp) < rad) & (along > 0)
    t = np.where(hit, along - np.sqrt(np.maximum(rad ** 2 - perp ** 2, 0)), np.inf)
    w = t.min(axis=1)
    if not np.isfinite(w).any():
        raise SystemExit("found no teeth along the wire's plane; check the three clicks")
    finite = np.isfinite(w)
    idx = np.flatnonzero(finite)
    h = radius * span / (n_rays - 1)
    # runs of rays that hit nothing: a tooth missing, or rays past the ends
    runs, i = [], 0
    while i < n_rays:
        if not finite[i]:
            j = i
            while j < n_rays and not finite[j]:
                j += 1
            runs.append((i, j))
            i = j
        else:
            i += 1
    if any((j - i) * h > bridge_mm for i, j in runs):
        warnings.append("a gap wider than the bridge length (missing tooth or diastema?) "
                        "was spanned with nothing to touch")
    w = np.interp(np.arange(n_rays), idx, w[idx])
    outline = C + w[:, None] * D

    # Light smoothing so scan noise does not make false dips.
    sig = max(1, int(0.3 / h))
    ker = np.exp(-0.5 * (np.arange(-3 * sig, 3 * sig + 1) / sig) ** 2)
    ws = np.convolve(np.pad(w, 3 * sig, mode="edge"), ker / ker.sum(), "valid")
    win = max(1, int(min_spacing / 2 / h))
    cands = [i for i in range(win, n_rays - win)
             if ws[i] <= ws[max(0, i - win):i + win + 1].min() + 1e-12]
    # Keep a dip only if a real bump (an embrasure) separates it from the
    # previous one; otherwise keep whichever sits further toward the tongue.
    dips = []
    for i in cands:
        if dips and ws[dips[-1]:i + 1].max() - max(ws[dips[-1]], ws[i]) < prominence:
            if ws[i] < ws[dips[-1]]:
                dips[-1] = i
            continue
        dips.append(i)
    end_gap = max(1, int(1.5 / h))
    dips = [i for i in dips if end_gap <= i <= n_rays - 1 - end_gap]
    contacts = np.vstack([outline[0], outline[dips], outline[-1]])
    return contacts, outline, warnings


def plan_wire(picks, wire, machine, style="smooth", contact_offset=0.0, mesh=None, resolve=True,
              hooks=None):
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

    if style == "auto":
        if len(picks) != 3 or mesh is None:
            raise SystemExit("auto design needs exactly three clicks (43, midline, 33) and the scan")
        found, outline, w2 = lingual_outline(mesh, origin, e_u, e_v, normal, contacts[0],
                                             contacts[1], contacts[2], r + contact_offset, r)
        warnings += w2
        dense = g.catmull_rom(found, samples_per_seg=400)
        auto_contacts = found
    elif style == "smooth":
        dense = g.catmull_rom(uv, samples_per_seg=400)
    elif style == "polyline":
        dense = uv
    else:
        raise SystemExit(f"unknown style {style!r}")
    curve = g.resample(dense, 0.02)
    if hooks and hooks.angle_deg > 0:
        if have_normals:
            sides = [-np.array([normals[i] @ e_u, normals[i] @ e_v]) for i in (0, -1)]
        else:
            sides = [np.array([0.0, 1.0])] * 2
        curve = g.resample(add_hooks(curve, hooks, *sides), 0.02)
        if hooks.radius_mm < machine.min_segment_mm / np.radians(machine.max_bend_deg):
            warnings.append(f"a {hooks.radius_mm} mm hook radius needs bends over "
                            f"{machine.max_bend_deg:.0f} deg at {machine.min_segment_mm} mm spacing")
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

    contact_labels = [pk.label for pk in picks]
    if style == "auto":
        uv = auto_contacts
        contact_labels = ([picks[0].label] + [f"tooth {i + 1}" for i in range(len(uv) - 2)]
                          + [picks[-1].label])
    return Plan(contact_labels=contact_labels, wire=wire, style=style, origin=origin, e_u=e_u, e_v=e_v, normal=normal,
                labels=[pk.label for pk in picks], contacts_2d=contacts, centers_2d=uv,
                out_of_plane=off, curve=curve, vertices=vertices, lengths=lengths,
                turns=turns, path_error=err, warnings=warnings)


def push_clear(curve, origin, e_u, e_v, r, mesh, step=0.2, iters=20, sigma=0.4):
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
        # Smoothing spreads the push thinner, so where the smoothed push falls
        # short of what a spot needs, top it up locally (a narrow window) and
        # let the next pass catch anything left.
        mag = np.linalg.norm(disp, axis=1)
        need = np.interp(s_dense, s, short)
        boost = np.where(need > mag, need / np.maximum(mag, 1e-9), 1.0)
        boost = np.minimum(boost, 3.0)
        disp *= np.convolve(np.pad(boost, half, mode="edge"), ker / ker.sum(), "valid")[:, None]
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
