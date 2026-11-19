import numpy as np

from retainer_bender import geometry as g


def arc(radius=10.0, degrees=150, n=400):
    t = np.radians(np.linspace(0, degrees, n))
    return np.column_stack([radius * np.cos(t), radius * np.sin(t)])


def test_catmull_rom_passes_through_points():
    pts = np.array([[0, 0], [3, 1], [6, 1.5], [9, 1], [12, 0]], float)
    c = g.catmull_rom(pts, 50)
    for p in pts:
        assert np.min(np.linalg.norm(c - p, axis=1)) < 1e-9


def test_fit_polyline_respects_limits():
    curve = g.resample(arc(), 0.02)
    idx, err = g.fit_polyline(curve, tol=0.04, min_seg=1.0, max_seg=6.0)
    v = curve[idx]
    L = g.segment_lengths(v)
    assert err <= 0.04 + 1e-9
    assert L.min() >= 1.0 - 0.05 and L.max() <= 6.0 + 0.05
    assert np.max(g.point_to_polyline(curve, v)) <= 0.04 + 1e-6


def test_build_polyline_inverts_lengths_and_turns():
    curve = g.resample(arc(), 0.5)
    L, T = g.segment_lengths(curve), g.turning_angles(curve)
    rebuilt = g.align_2d(g.build_polyline(L, T), curve)
    assert np.max(np.linalg.norm(rebuilt - curve, axis=1)) < 1e-6


def test_align_accepts_mirror_image():
    curve = g.resample(arc(degrees=90), 0.5)
    mirrored = curve * [1, -1] + [5, 2]
    assert np.max(np.linalg.norm(g.align_2d(mirrored, curve) - curve, axis=1)) < 1e-6
