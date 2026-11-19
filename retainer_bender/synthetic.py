"""A made-up lower anterior segment for trying the planner without a scan.

Six ellipsoid 'teeth' (43..33) on a parabolic arch, with picks where a
3-3 retainer would sit: two on each canine, one on each incisor at about
two-thirds crown height. One incisor can be set lingual of its neighbours to
exercise a reverse bend.
"""

import numpy as np

# mesiodistal width, labiolingual depth, crown height (mm)
TEETH = [("43", 7.0, 7.5, 11.0), ("42", 5.9, 6.0, 9.5), ("41", 5.4, 5.8, 9.0),
         ("31", 5.4, 5.8, 9.0), ("32", 5.9, 6.0, 9.5), ("33", 7.0, 7.5, 11.0)]


def _arch(k=0.055, n=4000, half=20):
    x = np.linspace(-half, half, n)
    pts = np.column_stack([x, -k * x ** 2])
    s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
    return pts, s


def _ellipsoid(center, e1, e2, e3, a, b, c, nu=28, nv=16):
    u = np.linspace(0, 2 * np.pi, nu, endpoint=False)
    v = np.linspace(0, np.pi, nv + 1)
    grid = lambda i, j: (center + a * np.cos(u[i % nu]) * np.sin(v[j]) * e1
                         + b * np.sin(u[i % nu]) * np.sin(v[j]) * e2 + c * np.cos(v[j]) * e3)
    tris = []
    for j in range(nv):
        for i in range(nu):
            p00, p10 = grid(i, j), grid(i + 1, j)
            p01, p11 = grid(i, j + 1), grid(i + 1, j + 1)
            if j > 0:
                tris.append([p00, p01, p10])
            if j < nv - 1:
                tris.append([p10, p01, p11])
    return np.array(tris)


def make_case(lingual_shift=None, height_frac=2 / 3):
    """Returns (triangles, picks) with picks as dicts like the picker writes."""
    lingual_shift = lingual_shift or {}
    pts, s = _arch()
    total = sum(t[1] for t in TEETH)
    start = s[-1] / 2 - total / 2
    tris, picks = [], []
    pos = start
    for name, md, ll, h in TEETH:
        mid = pos + md / 2
        pos += md
        i = int(np.searchsorted(s, mid))
        tangent = pts[min(i + 1, len(pts) - 1)] - pts[max(i - 1, 0)]
        tangent /= np.linalg.norm(tangent)
        labial = np.array([-tangent[1], tangent[0]])      # left of travel = +y side = labial
        e1 = np.array([*tangent, 0.0])
        e2 = np.array([*labial, 0.0])
        e3 = np.array([0.0, 0.0, 1.0])
        a, b, c = md / 2, ll / 2, h / 2
        center = np.array([*pts[i], c]) - e2 * lingual_shift.get(name, 0.0)
        tris.append(_ellipsoid(center, e1, e2, e3, a, b, c))
        z = h * height_frac - c          # local height of the contact above the centre
        locs = [0.0] if name not in ("43", "33") else ([-0.45, 0.15] if name == "43" else [-0.15, 0.45])
        for xl in locs:
            x = xl * a
            y = -b * np.sqrt(max(1 - (x / a) ** 2 - (z / c) ** 2, 0.0))
            p = center + x * e1 + y * e2 + z * e3
            n = (x / a ** 2) * e1 + (y / b ** 2) * e2 + (z / c ** 2) * e3
            n /= np.linalg.norm(n)
            picks.append({"p": p.round(4).tolist(), "n": n.round(5).tolist(),
                          "label": name if len(locs) == 1 else f"{name}{'d' if (xl < 0) == (name == '43') else 'm'}"})
    return np.vstack(tris), picks
