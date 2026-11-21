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
    curve_w: np.ndarray | None = None    # height of the dense curve off the plane (3D wires)
    vertices3: np.ndarray | None = None  # bend points (u, v, w) for 3D wires
    canine_step: float = 0.0
    prefinish: np.ndarray | None = None   # flat wire the machine bends before finishing (2D vertices)
    prefinish_dense: np.ndarray | None = None
    finish: list | None = None            # where to joggle and elbow, measured from each end
    finish_step: float = 0.0

    @property
    def is3d(self):
        return self.vertices3 is not None

    def to_3d(self, uv, w=None):
        uv = np.asarray(uv, float)
        out = self.origin + uv[..., :1] * self.e_u + uv[..., 1:2] * self.e_v
        if w is not None:
            out = out + np.asarray(w, float)[..., None] * self.normal
        return out

    def curve3(self):
        """Dense centre line as (u, v, w)."""
        w = self.curve_w if self.curve_w is not None else np.zeros(len(self.curve))
        return np.column_stack([self.curve, w])

    def world(self, step=None):
        """Dense centre line in scan coordinates, optionally resampled."""
        c = self.curve3()
        if step:
            c = g.resample(c, step)
        return self.to_3d(c[:, :2], c[:, 2]), c

    @property
    def total_length(self):
        return float(self.lengths.sum())


@dataclass
class Hooks:
    """Retention curls at the two canine ends, in the wire's plane."""
    angle_deg: float = 90.0     # how far the end curls round; 0 = straight ends
    radius_mm: float = 2.0      # radius of the curl, to the wire centre
    toward: str = "tooth"       # "tooth" wraps onto the canine; "tongue" curls in toward the midline


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


def roll_disc(outline, C, D, w, R):
    """Roll a disc of radius R along the inside of the outline.

    The wire goes where the disc's edge reaches: it follows each lingual
    surface and spans any embrasure the disc is too big to drop into, with an
    arc no tighter than R. outline[i] = C + w[i] * D[i] (rays from C).
    Returns (wire curve along the same rays, mask of rays where the wire
    rests on the outline).
    """
    t = np.gradient(outline, axis=0)
    t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-12)
    n = np.column_stack([-t[:, 1], t[:, 0]])
    flip = np.sum(n * (C - outline), axis=1) < 0
    n[flip] *= -1                                   # point into the free side
    centres = outline + R * n
    dist = np.linalg.norm(centres[:, None, :] - outline[None, :, :], axis=2).min(axis=1)
    centres = centres[dist >= R - 0.01]             # discs that fit without cutting a tooth
    if not len(centres):
        return outline.copy(), np.ones(len(w), bool)
    rel = centres - C                               # (k, 2)
    along = D @ rel.T                               # (rays, k)
    perp2 = np.sum(rel ** 2, axis=1)[None, :] - along ** 2
    far = np.where(perp2 <= R * R, along + np.sqrt(np.maximum(R * R - perp2, 0)), -np.inf)
    reach = far.max(axis=1)
    reach = np.where(np.isfinite(reach), np.minimum(reach, w), w)
    return C + reach[:, None] * D, reach >= w - 0.03


def lingual_outline(mesh, origin, e_u, e_v, normal, a, m, b, offset, r_wire, bridge_mm=4.0,
                    step=0.1, roll_radius=3.0):
    """Auto-design: find where the wire touches each tooth.

    The scan is cut at the wire's plane (and a wire radius above and below
    it). Rays fan out from a point behind the arch (the midline click
    mirrored across the canine-to-canine line), and along each the wire's
    centre can come out until it is `offset` from a tooth. A disc of
    `roll_radius` is rolled along the inside of that outline (roll_disc):
    the wire follows the lingual surfaces and bridges embrasures. a, m, b
    are the three clicks in plane coordinates.

    Returns (centre of each contact patch, with the two ends; the wire
    centre line; warnings).
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

    curve, touching = roll_disc(outline, C, D, w, roll_radius)
    # contact patches: runs where the wire rests on the outline
    contacts = [curve[0]]
    i = 0
    while i < n_rays:
        if touching[i]:
            j = i
            while j < n_rays and touching[j]:
                j += 1
            mid = (i + j - 1) // 2
            if (j - i) * h >= 0.3 and 1.0 / h <= mid <= n_rays - 1 - 1.0 / h:
                contacts.append(curve[mid])
            i = j
        else:
            i += 1
    contacts.append(curve[-1])
    return np.array(contacts), curve, warnings


def shift_gingivally(picks, normals, mesh, mm):
    """Slide every click `mm` toward the gingiva along its tooth's surface.

    "Toward the gingiva" is measured along the normal of the plane through
    the clicks. Lower lingual surfaces face occlusally, so the side of that
    plane the click normals lean toward is occlusal and the other is
    gingival. Each click is moved down by `mm`, then put back on the tooth by
    a ray cast in from the lingual. Negative `mm` moves toward the incisal.
    """
    from .mesh import raycast
    P = np.array([pk.p for pk in picks])
    N = np.array(normals)
    _, up = g.fit_plane(P)
    if np.dot(N.mean(axis=0), up) < 0:
        up = -up
    out_picks, out_normals = [], []
    for pk, n in zip(picks, N):
        q = pk.p - mm * up
        inward = -(n - np.dot(n, up) * up)       # into the tooth, level with the plane
        inward /= np.linalg.norm(inward)
        hit, hn = raycast(q - 3.0 * inward, inward, mesh)
        if hit is None or np.linalg.norm(hit - q) > 3.0:
            raise SystemExit(f"{pk.label}: no tooth surface {mm} mm gingival of this click")
        out_picks.append(Pick(hit, hn, pk.label))
        out_normals.append(hn)
    return out_picks, out_normals


def step_profile(s, start, end, step, inset, ramp):
    """Height (as a positive drop) along the wire for a step at each canine.

    s: arc length of each dense point; start/end: arc positions of the two
    canine clicks; the drop is `step` from each end up to `inset` toward the
    midline, easing to zero over `ramp` mm (a cosine, so the two bends that
    make each step share the turn equally).
    """
    def ease(x):            # 1 before the step, 0 after it
        return np.where(x <= 0, 1.0, np.where(x >= 1, 0.0, 0.5 * (1 + np.cos(np.pi * x))))
    a = ease((s - (start + inset - ramp / 2)) / ramp)
    b = ease(((end - inset + ramp / 2) - s) / ramp)
    return step * np.maximum(a, b)


def _at(path, sarr, x):
    """Point on a polyline at arc length x."""
    x = min(max(x, 0.0), sarr[-1])
    return np.array([np.interp(x, sarr, path[:, k]) for k in range(path.shape[1])])


def _piece(path, sarr, x0, x1):
    """The polyline between arc lengths x0 and x1, ends included."""
    inner = path[(sarr > x0) & (sarr < x1)]
    return np.vstack([_at(path, sarr, x0), inner, _at(path, sarr, x1)])


def _line(p, q, step=0.02):
    n = max(2, int(np.ceil(np.linalg.norm(q - p) / step)) + 1)
    t = np.linspace(0, 1, n)[:, None]
    return p + t * (q - p)


def _side(hi, low, c, inset, step_w, margin, tan_max):
    """One end of the wire, walking in from the canine end.

    hi: the wire at incisor height (2D), low: the same wire settled onto the
    teeth a step lower (2D), both oriented from this end inward; c: arc
    position of the canine click on `hi`. Returns (3D path from the end to
    the junction with the incisor run, arc position on `hi` where the run
    resumes, the two ramp points).
    """
    sh = g.arc_lengths(hi)
    sl = g.arc_lengths(low)
    along = 2.5
    for _ in range(8):
        a = c + inset - along / 2              # ramp starts (low side)
        b = a + along                          # ramp ends (incisor side)
        P = _at(low, sl, a - margin)           # end of the settled low run
        Q = _at(hi, sh, b + margin)            # start of the incisor run
        d = (Q - P) / np.linalg.norm(Q - P)    # one in-plane direction for both lead-ins
        A = P + d * margin
        B = Q - d * margin
        lateral = (B - A) - d * np.dot(B - A, d)
        need = np.hypot(np.linalg.norm(lateral), step_w) / tan_max
        have = float(np.dot(B - A, d))
        if have >= need - 1e-3:
            break
        along += need - have + 0.05
    wl = step_w
    low3 = np.column_stack([low, np.full(len(low), wl)])
    path = np.vstack([
        _piece(low3, sl, 0.0, a - margin),
        _line(np.append(P, wl), np.append(A, wl))[1:],
        _line(np.append(A, wl), np.append(B, 0.0))[1:],
        _line(np.append(B, 0.0), np.append(Q, 0.0))[1:],
    ])
    return path, b + margin, (np.append(P, wl), np.append(A, wl), np.append(B, 0.0), np.append(Q, 0.0))


def _embrasure(hi, c, origin, e_u, e_v, mesh, window=(1.5, 6.5)):
    """Arc position (on `hi`, oriented from this end) of the lateral-canine
    embrasure: where, a little in from the canine click, the wire stands
    furthest off the teeth."""
    from .mesh import signed_distances
    sh = g.arc_lengths(hi)
    xs = np.arange(c + window[0], min(c + window[1], sh[-1] - 1.0), 0.1)
    if len(xs) < 3 or mesh is None:
        return c + 3.0, None
    pts = np.array([_at(hi, sh, x) for x in xs])
    d = signed_distances(origin + pts[:, :1] * e_u + pts[:, 1:2] * e_v, mesh)
    d = np.where(np.isfinite(d), d, -np.inf)
    i = int(np.argmax(d))
    return float(xs[i]), float(d[i])


def elbow_ends(curve, ends2d, picks_world, gum_world, step, leg_range, gum_gap, up_sign,
               machine, origin, e_u, e_v, normal, r, mesh, margin=0.5):
    """Canine ends as a 0.75 mm gingival step at the lateral-canine embrasure,
    then a 90 degree elbow toward the gingiva at the canine click (mid-crown)
    with a short leg running down the canine's lingual surface.

    Returns the finished 3D wire, the flat wire the machine bends (step and
    elbow unfolded, same length), the points where vertices must fall on
    each, where to joggle and elbow measured from each end, and warnings.
    """
    warnings = []
    wl = -up_sign * step
    low3 = np.column_stack([curve, np.full(len(curve), wl)])
    if mesh is not None:
        low3, _ = push_clear(low3, origin, e_u, e_v, r, mesh, normal=normal)
    low = low3[:, :2]
    tan_max = np.tan(np.radians(35.0))
    s = g.arc_lengths(curve)
    i0 = int(np.argmin(np.linalg.norm(curve - ends2d[0], axis=1)))
    i1 = int(np.argmin(np.linalg.norm(curve - ends2d[1], axis=1)))
    up = up_sign * normal

    sides = []
    for k, (hi_k, low_k, c) in enumerate(((curve, low, s[i0]),
                                          (curve[::-1], low[::-1], s[-1] - s[i1]))):
        emb, gap = _embrasure(hi_k, c, origin, e_u, e_v, mesh)
        path, x_resume, (P, A, B, Q) = _side(hi_k, low_k, c, emb - c, wl, margin, tan_max)
        # elbow at the start of the lowered run (the canine click), leg toward the gingiva
        K = path[0]
        Kw = origin + K[0] * e_u + K[1] * e_v + K[2] * normal
        n_s = picks_world[k][1]
        down = -up - np.dot(-up, n_s) * n_s
        down /= np.linalg.norm(down)
        leg = leg_range[1]
        if gum_world[k] is not None:
            room = float(np.dot(gum_world[k] - Kw, down)) - r
            leg = float(np.clip(room - gum_gap, *leg_range))
            if room - leg < gum_gap - 0.05:
                warnings.append(f"{'first' if k == 0 else 'second'} canine: only {room:.1f} mm from the "
                                f"elbow to the gingiva; a {leg:.1f} mm leg leaves {room - leg:.1f} mm")
        # tilt the leg off the tooth until it clears along its whole length
        # (a convex canine bulges toward the cingulum under a straight leg)
        from .mesh import signed_distances
        tilt = 0.0
        for tilt in np.arange(0.0, 41.0, 1.0):
            a = np.radians(tilt)
            dir_w = np.cos(a) * down + np.sin(a) * n_s
            dir_w /= np.linalg.norm(dir_w)
            ts = np.linspace(0.2, 1.0, 9) * leg
            dd = signed_distances(Kw + np.outer(ts, dir_w), mesh) - r if mesh is not None else np.zeros(1)
            if not np.isfinite(dd).any() or np.nanmin(np.where(np.isfinite(dd), dd, np.nan)) >= 0.02:
                break
        else:
            warnings.append(f"{'first' if k == 0 else 'second'} canine: the leg still touches the tooth "
                            "even tilted 40 degrees off it")
        g_uvw = np.array([dir_w @ e_u, dir_w @ e_v, dir_w @ normal])
        run_dir = path[min(len(path) - 1, int(1.0 / 0.02))] - path[0]
        run_dir /= np.linalg.norm(run_dir)
        elbow_deg = float(np.degrees(np.arccos(np.clip(np.dot(-run_dir, g_uvw), -1, 1))))
        leg_pts = K + np.outer(np.linspace(1, 0, max(2, int(leg / 0.02) + 1)), g_uvw * leg)
        final = np.vstack([leg_pts, path[1:]])
        # the flat version: leg straight on, joggle undone (lengths kept)
        d2 = (Q[:2] - P[:2]) / np.linalg.norm(Q[:2] - P[:2])
        A_pre = B[:2] - d2 * np.linalg.norm(B - A)
        T = A_pre - A[:2]
        sl = g.arc_lengths(path[:, :2])
        iA = int(np.argmin(np.linalg.norm(path - A, axis=1)))
        before = path[:iA + 1, :2] + T
        t_out = before[0] - before[min(10, len(before) - 1)]
        t_out /= np.linalg.norm(t_out)
        leg_flat = before[0] + np.outer(np.linspace(leg, 0, max(2, int(leg / 0.02) + 1)), t_out)
        flat = np.vstack([leg_flat, before[1:], _line(A_pre, B[:2])[1:], path[np.argmin(np.linalg.norm(path - B, axis=1)) + 1:, :2]])
        fl = g.arc_lengths(flat)
        marks = {"elbow_mm": leg,
                 "joggle_from_mm": float(fl[len(leg_flat) - 1 + iA]),
                 "joggle_to_mm": float(fl[len(leg_flat) - 1 + iA] + np.linalg.norm(B - A)),
                 "joggle_offset_mm": float(np.linalg.norm((B - A) - np.append(d2, 0) * np.dot(B - A, np.append(d2, 0)))),
                 "embrasure_gap_mm": gap, "leg_mm": leg, "elbow_deg": 180.0 - elbow_deg,
                 "leg_tilt_deg": float(tilt)}
        sides.append((final, flat, x_resume, marks, [K, A, B], [before[0], A_pre, B[:2]]))

    x_head = sides[0][2]
    x_tail = s[-1] - sides[1][2]
    if x_head >= x_tail:
        raise SystemExit("the two canine ends overlap")
    mid = _piece(np.column_stack([curve, np.zeros(len(curve))]), s, x_head, x_tail)
    final = g.resample(np.vstack([sides[0][0], mid[1:], sides[1][0][::-1][1:]]), 0.02)
    flat = g.resample(np.vstack([sides[0][1], mid[1:, :2], sides[1][1][::-1][1:]]), 0.02)
    cuts3 = [int(np.argmin(np.linalg.norm(final - p, axis=1))) for p in sides[0][4] + sides[1][4]]
    cuts2 = [int(np.argmin(np.linalg.norm(flat - p, axis=1))) for p in sides[0][5] + sides[1][5]]
    return final, flat, cuts3, cuts2, [sides[0][3], sides[1][3]], warnings


def make_steps(curve, ends2d, step, inset, up_sign, machine, origin, e_u, e_v, normal, r, mesh,
               margin=1.0):
    """Put a gingival step at each canine, the way it would be bent by hand.

    The canine part of the wire is lowered by `step` and settled onto the
    tooth at that height (on a sloping lingual surface that also moves it
    lingually). It joins the run across the incisors by one straight ramp,
    entered and left along the same in-plane direction, so the two bends
    that make the step lie in one plane: the machine rolls the wire once,
    bends, feeds the ramp and bends back. The ramp is long enough that each
    step bend is at most 70% of the machine's limit.

    Returns (2D curve, height of each point along the plane normal, indices
    where vertices must fall, warnings).
    """
    warnings = []
    wl = -up_sign * step
    low3 = np.column_stack([curve, np.full(len(curve), wl)])
    if mesh is not None:
        low3, pushed = push_clear(low3, origin, e_u, e_v, r, mesh, normal=normal)
    low = low3[:, :2]
    tan_max = np.tan(np.radians(0.7 * machine.max_bend_deg))
    s = g.arc_lengths(curve)
    i0 = int(np.argmin(np.linalg.norm(curve - ends2d[0], axis=1)))
    i1 = int(np.argmin(np.linalg.norm(curve - ends2d[1], axis=1)))
    head, x_head, kp0 = _side(curve, low, s[i0], inset, wl, margin, tan_max)
    tail_r, x_tail, kp1 = _side(curve[::-1], low[::-1], s[-1] - s[i1], inset, wl, margin, tan_max)
    x_tail = s[-1] - x_tail
    if x_head >= x_tail:
        raise SystemExit("the two canine steps overlap; reduce --canine-step or --step-inset")
    mid = _piece(np.column_stack([curve, np.zeros(len(curve))]), s, x_head, x_tail)
    path = np.vstack([head, mid[1:], tail_r[::-1][1:]])
    path = g.resample(path, 0.02)
    cuts = [int(np.argmin(np.linalg.norm(path - p, axis=1))) for p in (*kp0, *kp1)]
    return path[:, :2], path[:, 2], cuts, warnings


def plan_wire(picks, wire, machine, style="smooth", contact_offset=0.0, mesh=None, resolve=True,
              hooks=None, gingival=0.0, canine_step=0.0, step_inset=2.5, ends="curl",
              elbow_step=0.75, leg_range=(1.0, 1.5), gum_gap=2.0):
    r = wire["d_mm"] / 2
    warnings = []
    gum = [None, None]
    if len(picks) == 5 and style == "auto":
        # optional clicks 4 and 5: the gingival margin under each canine
        gum = [picks[3].p, picks[4].p]
        picks = picks[:3]
    if ends == "elbow":
        hooks = None
        canine_step = 0.0

    normals = []
    for pk in picks:
        n = pk.n
        if n is None and mesh is not None:
            from .mesh import nearest_normal
            n = nearest_normal(pk.p, mesh)
        normals.append(n)
    have_normals = all(n is not None for n in normals)

    if gingival and have_normals:
        if mesh is None:
            raise SystemExit("--gingival needs the scan")
        picks, normals = shift_gingivally(picks, normals, mesh, gingival)
    P = np.array([pk.p for pk in picks])
    for a, b, la, lb in zip(P[:-1], P[1:], [pk.label for pk in picks][:-1],
                            [pk.label for pk in picks][1:]):
        if np.linalg.norm(b - a) < 0.3:
            raise SystemExit(f"picks {la} and {lb} are under 0.3 mm apart; remove one")

    C3 = P + (np.array(normals) * (r + contact_offset) if have_normals else 0.0)

    up_sign = 1.0
    lift = canine_step if canine_step else (elbow_step if ends == "elbow" else 0.0)
    if lift:
        if not have_normals:
            raise SystemExit("a canine step needs click normals (use the picker)")
        # The canine clicks are where the wire lands, a step below the run
        # across the incisors: lift them by the step so the plane is the
        # incisor run's. Lower lingual surfaces face occlusally, so the side
        # the click normals lean toward is up.
        _, n0 = g.fit_plane(C3)
        up0 = n0 if np.dot(np.mean(normals, axis=0), n0) > 0 else -n0
        C3 = C3.copy()
        C3[0] += lift * up0
        C3[-1] += lift * up0

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
    if have_normals:
        up_sign = 1.0 if np.dot(np.mean(normals, axis=0), normal) > 0 else -1.0
    off = rel @ normal
    # (with a canine step the canine clicks were lifted onto the plane above,
    # and the wire is lowered back onto them: "off" is how far it misses them)
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
            raise SystemExit("auto design needs three clicks (43, midline, 33), or five with the two "
                             "gum clicks, and the scan")
        found, outline, w2 = lingual_outline(mesh, origin, e_u, e_v, normal, contacts[0],
                                             contacts[1], contacts[2], r + contact_offset, r)
        warnings += w2
        dense = outline
        auto_contacts = found
    elif style == "smooth":
        dense = g.catmull_rom(uv, samples_per_seg=400)
    elif style == "polyline":
        dense = uv
    else:
        raise SystemExit(f"unknown style {style!r}")
    curve = g.resample(dense, 0.02)
    ends2d = (curve[0].copy(), curve[-1].copy())
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

    curve_w = None
    prefinish = prefinish_dense = finish = None
    if ends == "elbow":
        if not have_normals or mesh is None:
            raise SystemExit("elbow ends need the scan and click normals (use the picker)")
        pw = [(picks[0].p, normals[0]), (picks[-1].p, normals[-1])]
        final, flat, cuts3, cuts2, finish, w2 = elbow_ends(
            curve, ends2d, pw, gum, elbow_step, leg_range, gum_gap, up_sign, machine,
            origin, e_u, e_v, normal, r, mesh)
        warnings += w2
        pidx, _ = g.fit_polyline_pieces(flat, cuts2, machine.path_tolerance_mm,
                                        machine.min_segment_mm, machine.max_segment_mm)
        prefinish = flat[pidx]
        prefinish_dense = flat
        curve_w = final[:, 2].copy()
        idx, err = g.fit_polyline_pieces(final, cuts3, machine.path_tolerance_mm,
                                         machine.min_segment_mm, machine.max_segment_mm)
        curve = final
    elif canine_step:
        curve, curve_w, cuts, w2 = make_steps(curve, ends2d, canine_step, step_inset, up_sign,
                                              machine, origin, e_u, e_v, normal, r,
                                              mesh if resolve else None)
        warnings += w2
        curve3 = np.column_stack([curve, curve_w])
        idx, err = g.fit_polyline_pieces(curve3, cuts, machine.path_tolerance_mm,
                                         machine.min_segment_mm, machine.max_segment_mm)
        curve = curve3
    else:
        idx, err = g.fit_polyline(curve, machine.path_tolerance_mm,
                                  machine.min_segment_mm, machine.max_segment_mm)
    if style == "polyline":
        # keep every pick as a bend point
        pick_idx = [int(np.argmin(np.linalg.norm(curve - q, axis=1))) for q in uv]
        idx = np.unique(np.concatenate([idx, pick_idx]))
    vertices3 = curve[idx] if curve_w is not None else None
    vertices = curve[idx][:, :2]
    lengths = g.segment_lengths(curve[idx])
    turns = g.turning_angles(vertices)
    if curve_w is not None:
        curve = curve[:, :2]

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
                turns=turns, path_error=err, warnings=warnings,
                curve_w=curve_w, vertices3=vertices3, canine_step=canine_step,
                prefinish=prefinish, prefinish_dense=prefinish_dense, finish=finish,
                finish_step=elbow_step if finish else 0.0)


def push_clear(curve, origin, e_u, e_v, r, mesh, step=0.2, iters=20, sigma=0.4, normal=None,
               lock=None):
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
        if curve.shape[1] == 3:
            pts3 = pts3 + samples[:, 2:3] * normal
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
        if curve.shape[1] == 3:
            disp = np.column_stack([disp, np.zeros(len(disp))])      # height stays as planned
        if lock is not None:
            disp[lock] = 0.0                                          # straight step zones stay put
        curve += disp
        total += np.linalg.norm(disp, axis=1)
    if lock is not None:
        return curve, float(total.max())          # keep point-for-point with the lock mask
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
    pts, c = plan.world(step)
    samples = c[:, :2]
    s = g.arc_lengths(c)
    d = signed_distances(pts, mesh) - plan.wire["d_mm"] / 2
    at_picks = []
    for q in plan.centers_2d:
        i = int(np.argmin(np.linalg.norm(samples - q, axis=1)))
        at_picks.append(float(d[i]))
    return s, d, np.array(at_picks)
