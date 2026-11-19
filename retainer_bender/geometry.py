"""Plane fitting, curves and polylines.

Everything here is plain numpy so the planner runs anywhere. Units are mm and
radians unless a name says otherwise.
"""

import numpy as np


def fit_plane(points):
    """Least-squares plane through 3D points. Returns (centroid, unit normal)."""
    pts = np.asarray(points, float)
    c = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - c)
    return c, vt[2] / np.linalg.norm(vt[2])


def catmull_rom(points, samples_per_seg=200, alpha=0.5):
    """Centripetal Catmull-Rom curve through every point, in order.

    Centripetal (alpha=0.5) is used because it never forms loops or cusps
    between closely spaced points, which uniform Catmull-Rom can.
    """
    P = np.asarray(points, float)
    if len(P) < 2:
        raise ValueError("need at least two points")
    if len(P) == 2:
        t = np.linspace(0, 1, samples_per_seg)[:, None]
        return P[0] + t * (P[1] - P[0])
    Q = np.vstack([2 * P[0] - P[1], P, 2 * P[-1] - P[-2]])
    out = []
    for i in range(len(Q) - 3):
        p0, p1, p2, p3 = Q[i:i + 4]
        t0 = 0.0
        t1 = t0 + np.linalg.norm(p1 - p0) ** alpha
        t2 = t1 + np.linalg.norm(p2 - p1) ** alpha
        t3 = t2 + np.linalg.norm(p3 - p2) ** alpha
        t = np.linspace(t1, t2, samples_per_seg, endpoint=False)[:, None]
        a1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1
        a2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2
        a3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3
        b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2
        b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3
        out.append((t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2)
    out.append(P[-1][None])
    return np.vstack(out)


def arc_lengths(poly):
    seg = np.linalg.norm(np.diff(poly, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])


def resample(poly, step):
    """Resample a polyline at (nearly) equal arc-length spacing."""
    poly = np.asarray(poly, float)
    s = arc_lengths(poly)
    n = max(2, int(np.ceil(s[-1] / step)) + 1)
    t = np.linspace(0, s[-1], n)
    return np.column_stack([np.interp(t, s, poly[:, k]) for k in range(poly.shape[1])])


def chord_deviation(curve, i, j):
    """Largest distance of curve[i..j] from the straight chord curve[i]->curve[j]."""
    a, b = curve[i], curve[j]
    d = b - a
    L = np.linalg.norm(d)
    seg = curve[i:j + 1] - a
    if L < 1e-12:
        return float(np.max(np.linalg.norm(seg, axis=1)))
    return float(np.max(np.abs(seg[:, 0] * d[1] - seg[:, 1] * d[0]) / L))


def fit_polyline(curve, tol, min_seg, max_seg):
    """Replace a dense 2D curve with as few straight segments as possible.

    Each chord stays within `tol` of the curve, is at least `min_seg` long
    (the machine cannot place two bends closer than that) and at most
    `max_seg`. Returns the indices of the chosen vertices into `curve`, and
    the worst deviation actually reached (above `tol` only where `min_seg`
    forced it).
    """
    s = arc_lengths(curve)
    n = len(curve)
    idx = [0]
    worst = 0.0
    i = 0
    while True:
        if s[-1] - s[i] <= max_seg:
            d_end = chord_deviation(curve, i, n - 1)
            if d_end <= tol:
                idx.append(n - 1)
                worst = max(worst, d_end)
                break
        best = None
        j = i + 1
        while j < n and s[j] - s[i] <= max_seg:
            if chord_deviation(curve, i, j) <= tol:
                best = j
            elif s[j] - s[i] >= min_seg:
                break
            j += 1
        if best is None or s[best] - s[i] < min_seg:
            best = int(min(np.searchsorted(s, s[i] + min_seg), n - 1))
        if s[-1] - s[best] < min_seg:
            # The piece left over is too short to bend; run straight to the end.
            best = n - 1
        worst = max(worst, chord_deviation(curve, i, best))
        idx.append(best)
        if best == n - 1:
            break
        i = best
    return np.array(idx), worst


def wrap_angle(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def turning_angles(vertices):
    """Signed turn at each interior vertex of a 2D polyline (+ = left/CCW)."""
    d = np.diff(vertices, axis=0)
    heading = np.arctan2(d[:, 1], d[:, 0])
    return wrap_angle(np.diff(heading))


def segment_lengths(vertices):
    return np.linalg.norm(np.diff(vertices, axis=0), axis=1)


def build_polyline(lengths, turns, start=(0.0, 0.0), heading=0.0):
    """Inverse of (segment_lengths, turning_angles)."""
    pts = [np.asarray(start, float)]
    h = heading
    for k, L in enumerate(lengths):
        if k > 0:
            h += turns[k - 1]
        pts.append(pts[-1] + L * np.array([np.cos(h), np.sin(h)]))
    return np.array(pts)


def point_to_polyline(points, poly):
    """Distance from each point to the nearest point on a polyline."""
    P = np.asarray(points, float)[:, None, :]
    A = poly[:-1][None]
    B = poly[1:][None]
    AB = B - A
    denom = np.maximum((AB ** 2).sum(-1), 1e-18)
    t = np.clip(((P - A) * AB).sum(-1) / denom, 0, 1)
    proj = A + t[..., None] * AB
    return np.min(np.linalg.norm(P - proj, axis=-1), axis=1)


def align_2d(src, dst, allow_reflection=True):
    """Best rigid (optionally mirrored) fit of src onto dst, matched point for point.

    Returns the transformed src. A planar wire turned over is its own mirror
    image, so a mirrored match is the same physical wire.
    """
    src = np.asarray(src, float)
    dst = np.asarray(dst, float)
    best = None
    for mirror in ([False, True] if allow_reflection else [False]):
        s = src * np.array([1, -1]) if mirror else src
        cs, cd = s.mean(0), dst.mean(0)
        H = (s - cs).T @ (dst - cd)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[1] *= -1
            R = Vt.T @ U.T
        out = (s - cs) @ R.T + cd
        err = np.sqrt(np.mean(np.sum((out - dst) ** 2, axis=1)))
        if best is None or err < best[0]:
            best = (err, out)
    return best[1]


def left_normals(poly):
    """Unit left-hand normal at each vertex of a 2D polyline (tangent-averaged)."""
    d = np.diff(poly, axis=0)
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    t = np.vstack([d[0], d[:-1] + d[1:], d[-1]])
    t /= np.linalg.norm(t, axis=1, keepdims=True)
    return np.column_stack([-t[:, 1], t[:, 0]])
