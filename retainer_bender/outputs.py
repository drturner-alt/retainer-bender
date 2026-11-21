"""Files a plan produces: a preview, a 1:1 print template and a check jig."""

import numpy as np

from . import geometry as g
from .mesh import save_stl, signed_volume, slice_plane


def _path(pts, fx):
    return " ".join(("M" if i == 0 else "L") + f"{fx(p)[0]:.3f},{fx(p)[1]:.3f}"
                    for i, p in enumerate(pts))


def preview_svg(plan, path, mesh=None, built=None, clearance_info=None, scale=25.0):
    """Occlusal view, front teeth at the top, `scale` px per mm."""
    pts = [plan.curve, plan.contacts_2d]
    segs = None
    if mesh is not None:
        segs3 = slice_plane(mesh, plan.origin, plan.normal)
        if len(segs3):
            rel = segs3 - plan.origin
            segs = np.stack([rel @ plan.e_u, rel @ plan.e_v], axis=-1)
            lo, hi = plan.curve.min(0) - 6, plan.curve.max(0) + 6
            keep = np.all((segs.min(1) > lo) & (segs.max(1) < hi), axis=1)
            segs = segs[keep]
            if len(segs):
                pts.append(segs.reshape(-1, 2))
    allp = np.vstack(pts)
    lo, hi = allp.min(0) - 3, allp.max(0) + 3
    W, H = (hi - lo) * scale

    def fx(p):
        return ((p[0] - lo[0]) * scale, (hi[1] - p[1]) * scale)

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W:.0f}" height="{H + 40:.0f}" '
           f'viewBox="0 0 {W:.1f} {H + 40:.1f}" font-family="sans-serif">',
           f'<rect width="100%" height="100%" fill="#fff"/>']
    if segs is not None and len(segs):
        d = " ".join(f"M{fx(a)[0]:.2f},{fx(a)[1]:.2f}L{fx(b)[0]:.2f},{fx(b)[1]:.2f}" for a, b in segs)
        out.append(f'<path d="{d}" stroke="#9aa3ad" stroke-width="1" fill="none"/>')
    w = plan.wire["d_mm"] * scale
    out.append(f'<path d="{_path(plan.curve, fx)}" stroke="#2f6fd6" stroke-opacity="0.35" '
               f'stroke-width="{w:.1f}" fill="none" stroke-linecap="round"/>')
    out.append(f'<path d="{_path(plan.vertices, fx)}" stroke="#d9822b" stroke-width="1.5" fill="none"/>')
    if built is not None:
        out.append(f'<path d="{_path(built, fx)}" stroke="#2a9d5c" stroke-width="1" '
                   f'stroke-dasharray="4 3" fill="none"/>')
    for v in plan.vertices[1:-1]:
        x, y = fx(v)
        out.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="2.2" fill="#d9822b"/>')
    for q, lab in zip(plan.contacts_2d, plan.labels):
        x, y = fx(q)
        out.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="4" fill="none" stroke="#c0392b" stroke-width="1.5"/>')
        out.append(f'<text x="{x + 6:.1f}" y="{y - 6:.1f}" font-size="12" fill="#c0392b">{lab}</text>')
    if clearance_info is not None:
        s, d, _ = clearance_info
        samples = g.resample(plan.curve, 0.2)
        for p, c in zip(samples, d):
            if c < -0.05:
                x, y = fx(p)
                out.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="3" fill="#c0392b"/>')
    # 5 mm scale bar
    x0, y0 = 10, H + 25
    out.append(f'<line x1="{x0}" y1="{y0}" x2="{x0 + 5 * scale}" y2="{y0}" stroke="#333" stroke-width="2"/>')
    out.append(f'<text x="{x0}" y="{y0 - 6}" font-size="11" fill="#333">5 mm</text>')
    out.append(f'<text x="{x0 + 5 * scale + 12}" y="{y0 + 4}" font-size="11" fill="#555">'
               'grey: teeth cut at wire plane · blue: wire · orange: bends'
               + (' · green dashed: predicted after springback' if built is not None else '')
               + (' · red dots: wire into tooth' if clearance_info is not None else '')
               + '</text>')
    out.append("</svg>")
    path.write_text("\n".join(out))


def template_svg(plan, path):
    """True-size drawing to print at 100% and lay the bent wire on. For wires
    finished after bending (elbow ends) it shows the flat wire as it comes
    off the machine, with the joggle and elbow marks."""
    flat = getattr(plan, "prefinish_dense", None)
    line = flat if flat is not None else plan.curve
    lo = line.min(0) - 8
    hi = line.max(0) + 8
    W, H = hi - lo
    H += 14

    def fx(p):
        return (p[0] - lo[0], hi[1] - p[1])

    d = plan.wire["d_mm"]
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W:.2f}mm" height="{H:.2f}mm" '
           f'viewBox="0 0 {W:.3f} {H:.3f}" font-family="sans-serif">',
           '<rect width="100%" height="100%" fill="#fff"/>',
           f'<path d="{_path(line, fx)}" stroke="#000" stroke-width="{d + 0.3:.3f}" '
           f'fill="none" stroke-linecap="butt"/>',
           f'<path d="{_path(line, fx)}" stroke="#fff" stroke-width="{d - 0.2:.3f}" '
           f'fill="none" stroke-linecap="butt"/>']
    if flat is not None and plan.finish:
        sarc = g.arc_lengths(flat)
        n = g.left_normals(flat)
        def mark(x, label, color):
            i = int(np.clip(np.searchsorted(sarc, x), 0, len(flat) - 1))
            a = flat[i] + n[i] * 1.6
            b = flat[i] - n[i] * 1.6
            out.append(f'<path d="M{fx(a)[0]:.2f},{fx(a)[1]:.2f}L{fx(b)[0]:.2f},{fx(b)[1]:.2f}" '
                       f'stroke="{color}" stroke-width="0.25"/>')
            t = flat[i] + n[i] * 2.6
            out.append(f'<text x="{fx(t)[0]:.2f}" y="{fx(t)[1]:.2f}" font-size="1.4" '
                       f'text-anchor="middle" fill="{color}">{label}</text>')
        for k, f in enumerate(plan.finish):
            conv = (lambda x: x) if k == 0 else (lambda x: sarc[-1] - x)
            mark(conv(f["elbow_mm"]), "elbow", "#c0392b")
            mark(conv(f["joggle_from_mm"]), "J", "#2f6fd6")
            mark(conv(f["joggle_to_mm"]), "J", "#2f6fd6")
    for q, lab in zip(plan.contacts_2d, plan.labels):
        x, y = fx(q)
        out.append(f'<text x="{x:.2f}" y="{y + 3:.2f}" font-size="1.8" text-anchor="middle">{lab}</text>')
    y0 = (hi[1] - lo[1]) + 6
    out.append(f'<line x1="4" y1="{y0}" x2="14" y2="{y0}" stroke="#000" stroke-width="0.3"/>')
    for x in (4, 14):
        out.append(f'<line x1="{x}" y1="{y0 - 1}" x2="{x}" y2="{y0 + 1}" stroke="#000" stroke-width="0.3"/>')
    out.append(f'<text x="16" y="{y0 + 0.7}" font-size="2">10 mm: check this measures 10 mm '
               f'before trusting the template · {plan.wire["label"]} · '
               f'{plan.total_length:.1f} mm</text>')
    out.append("</svg>")
    path.write_text("\n".join(out))


def _sweep(path2d, a, b, z0, z1):
    """Closed solid: the band between offsets a and b of a 2D path, z0..z1."""
    n = g.left_normals(path2d)
    ring = []
    for off, z in ((a, z0), (b, z0), (b, z1), (a, z1)):
        xy = path2d + off * n
        ring.append(np.column_stack([xy, np.full(len(xy), z)]))
    R = np.stack(ring, axis=1)            # (k, 4, 3)
    tris = []
    for i in range(len(R) - 1):
        for c in range(4):
            p, q = R[i, c], R[i, (c + 1) % 4]
            p2, q2 = R[i + 1, c], R[i + 1, (c + 1) % 4]
            tris += [[p, q, q2], [p, q2, p2]]
    for end, flip in ((R[0], True), (R[-1], False)):
        t = [[end[0], end[1], end[2]], [end[0], end[2], end[3]]]
        tris += [tt[::-1] for tt in t] if flip else t
    tris = np.array(tris)
    return tris if signed_volume(tris) > 0 else tris[:, ::-1]


def _box(lo, hi):
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    v = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]])
    f = [[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
         [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]]
    tris = v[np.array(f)]
    return tris if signed_volume(tris) > 0 else tris[:, ::-1]


def jig_stl(plan, path, plate_mm=2.0, wall_mm=1.2, slack_mm=0.12):
    """A flat plate with a groove the finished wire should drop into.

    Separate overlapping shells (plate and two walls); every slicer merges
    them. Print flat, 0.1-0.15 mm layers.
    """
    d = plan.wire["d_mm"]
    flat = getattr(plan, "prefinish_dense", None)
    path2d = g.resample(flat if flat is not None else plan.curve, 0.25)
    half = d / 2 + slack_mm
    # a stepped wire is laid steps-up: it rests on its middle run and the
    # walls are tall enough to hold the raised ends too
    top = plate_mm + 0.8 * d + getattr(plan, "canine_step", 0.0)
    lo = path2d.min(0) - 4
    hi = path2d.max(0) + 4
    parts = [_box((lo[0], lo[1], 0), (hi[0], hi[1], plate_mm)),
             _sweep(path2d, half, half + wall_mm, plate_mm - 0.2, top),
             _sweep(path2d, -half - wall_mm, -half, plate_mm - 0.2, top)]
    save_stl(path, np.vstack(parts))


def wire_stl(plan, path, sides=16, step=0.2):
    """The planned wire as a solid, in the scan's own coordinates.

    Load it with the scan to look at the fit in 3D, or merge it into the
    model before printing an overlay clear retainer so the retainer clears
    the wire.
    """
    c3, _ = plan.world(step)
    r = plan.wire["d_mm"] / 2
    t = np.gradient(c3, axis=0)
    t /= np.linalg.norm(t, axis=1, keepdims=True)
    b = np.cross(t, plan.normal)
    b /= np.linalg.norm(b, axis=1, keepdims=True)
    n = np.cross(b, t)
    ang = np.linspace(0, 2 * np.pi, sides, endpoint=False)
    rings = (c3[:, None, :] + r * (np.cos(ang)[None, :, None] * n[:, None, :]
                                   + np.sin(ang)[None, :, None] * b[:, None, :]))
    tris = []
    for i in range(len(rings) - 1):
        for j in range(sides):
            a0, a1 = rings[i, j], rings[i, (j + 1) % sides]
            b0, b1 = rings[i + 1, j], rings[i + 1, (j + 1) % sides]
            tris += [[a0, b1, b0], [a0, a1, b1]]      # outward: (n, b, t) is right-handed
    for ring, centre, flip in ((rings[0], c3[0], True), (rings[-1], c3[-1], False)):
        for j in range(sides):
            tri = [centre, ring[j], ring[(j + 1) % sides]]
            tris.append(tri[::-1] if flip else tri)
    tris = np.array(tris)
    if signed_volume(tris) < 0:
        tris = tris[:, ::-1]
    save_stl(path, tris)
