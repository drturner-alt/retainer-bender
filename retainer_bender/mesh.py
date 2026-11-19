"""STL reading/writing and the few mesh queries the planner needs.

A mesh here is just an (m, 3, 3) array of triangles in mm. No mesh library
is needed: the planner only asks how far the wire is from the teeth and what
the teeth look like where the wire's plane cuts them.
"""

import struct

import numpy as np


def load_stl(path):
    with open(path, "rb") as f:
        data = f.read()
    if len(data) >= 84:
        (count,) = struct.unpack_from("<I", data, 80)
        if 84 + 50 * count == len(data):
            rec = np.frombuffer(data, dtype=np.dtype([
                ("n", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")]),
                count=count, offset=84)
            return rec["v"].astype(float)
    text = data.decode("ascii", errors="replace")
    verts = [list(map(float, line.split()[1:4]))
             for line in text.splitlines() if line.strip().startswith("vertex")]
    if not verts or len(verts) % 3:
        raise ValueError(f"{path}: not a readable STL file")
    return np.array(verts, float).reshape(-1, 3, 3)


def face_normals(tris):
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    L = np.linalg.norm(n, axis=1, keepdims=True)
    return n / np.where(L == 0, 1, L)


def save_stl(path, tris):
    tris = np.asarray(tris, np.float32)
    n = face_normals(tris.astype(float)).astype(np.float32)
    rec = np.zeros(len(tris), dtype=np.dtype([
        ("n", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")]))
    rec["n"] = n
    rec["v"] = tris
    with open(path, "wb") as f:
        f.write(b"retainer-bender".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        f.write(rec.tobytes())


def signed_volume(tris):
    return float(np.einsum("ij,ij->i", tris[:, 0],
                           np.cross(tris[:, 1], tris[:, 2])).sum() / 6.0)


def closest_points(p, tris):
    """Closest point on each triangle to point p (Ericson's region test, vectorised)."""
    A, B, C = tris[:, 0], tris[:, 1], tris[:, 2]
    ab, ac, ap = B - A, C - A, p - A
    d1 = (ab * ap).sum(1)
    d2 = (ac * ap).sum(1)
    bp = p - B
    d3 = (ab * bp).sum(1)
    d4 = (ac * bp).sum(1)
    cp = p - C
    d5 = (ab * cp).sum(1)
    d6 = (ac * cp).sum(1)
    vc = d1 * d4 - d3 * d2
    vb = d5 * d2 - d1 * d6
    va = d3 * d6 - d5 * d4

    res = np.empty_like(A)
    done = np.zeros(len(A), bool)
    with np.errstate(divide="ignore", invalid="ignore"):
        def take(mask, value):
            nonlocal done
            m = mask & ~done
            res[m] = value[m]
            done |= m

        take((d1 <= 0) & (d2 <= 0), A)
        take((d3 >= 0) & (d4 <= d3), B)
        take((d6 >= 0) & (d5 <= d6), C)
        take((vc <= 0) & (d1 >= 0) & (d3 <= 0), A + (d1 / (d1 - d3))[:, None] * ab)
        take((vb <= 0) & (d2 >= 0) & (d6 <= 0), A + (d2 / (d2 - d6))[:, None] * ac)
        take((va <= 0) & (d4 - d3 >= 0) & (d5 - d6 >= 0),
             B + ((d4 - d3) / ((d4 - d3) + (d5 - d6)))[:, None] * (C - B))
        denom = va + vb + vc
        denom = np.where(denom == 0, 1, denom)
        take(np.ones(len(A), bool),
             A + ab * (vb / denom)[:, None] + ac * (vc / denom)[:, None])
    return res


def signed_distances(points, tris, search_radius=3.0, return_normals=False):
    """Signed distance from each point to the mesh surface (negative = inside).

    Only triangles within `search_radius` of a point are considered; a point
    farther than that from everything reports +inf. With `return_normals`,
    also returns the outward normal of the nearest triangle (zeros if none).
    """
    points = np.asarray(points, float)
    lo = points.min(0) - search_radius - 1
    hi = points.max(0) + search_radius + 1
    keep = np.all((tris.max(1) >= lo) & (tris.min(1) <= hi), axis=1)
    tris = tris[keep]
    normals = face_normals(tris)
    centroids = tris.mean(1)
    span = np.max(np.linalg.norm(tris - centroids[:, None], axis=2), axis=1)
    out = np.full(len(points), np.inf)
    nout = np.zeros((len(points), 3))
    for k, p in enumerate(points):
        near = np.linalg.norm(centroids - p, axis=1) <= search_radius + span
        if not near.any():
            continue
        t = tris[near]
        q = closest_points(p, t)
        d = np.linalg.norm(q - p, axis=1)
        j = int(np.argmin(d))
        sign = 1.0 if np.dot(p - q[j], normals[near][j]) >= 0 else -1.0
        out[k] = sign * d[j]
        nout[k] = normals[near][j]
    return (out, nout) if return_normals else out


def nearest_normal(point, tris, search_radius=2.0):
    """Outward normal of the triangle nearest a picked point."""
    centroids = tris.mean(1)
    near = np.linalg.norm(centroids - point, axis=1) <= search_radius
    if not near.any():
        raise ValueError(f"no mesh surface within {search_radius} mm of {point}")
    t = tris[near]
    q = closest_points(np.asarray(point, float), t)
    return face_normals(t)[int(np.argmin(np.linalg.norm(q - point, axis=1)))]


def slice_plane(tris, origin, normal):
    """Segments where a plane cuts the mesh, as an (k, 2, 3) array."""
    d = (tris - origin) @ normal
    crossing = (d.min(1) < 0) & (d.max(1) > 0)
    tris, d = tris[crossing], d[crossing]
    segs = []
    for e0, e1 in ((0, 1), (1, 2), (2, 0)):
        da, db = d[:, e0], d[:, e1]
        m = (da < 0) != (db < 0)
        t = np.zeros_like(da)
        t[m] = da[m] / (da[m] - db[m])
        pts = tris[:, e0] + t[:, None] * (tris[:, e1] - tris[:, e0])
        segs.append(np.where(m[:, None], pts, np.nan))
    segs = np.stack(segs, axis=1)          # (k, 3 edges, 3)
    out = []
    for row in segs:
        ok = row[~np.isnan(row[:, 0])]
        if len(ok) == 2:
            out.append(ok)
    return np.array(out).reshape(-1, 2, 3)
