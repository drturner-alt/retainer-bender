import json

import numpy as np
import pytest

from retainer_bender.cli import calibration_program, fit_springback, run_plan
from retainer_bender.machine import Machine, free_wire, machine_bends, make_program
from retainer_bender.mesh import load_stl, save_stl, signed_volume
from retainer_bender.planner import Hooks, Pick
from retainer_bender.springback import Springback
from retainer_bender.synthetic import make_case
from retainer_bender.wires import get_wire


def consistently_wound(tris):
    """Closed and every face facing the same way: each directed edge once,
    and its reverse once."""
    e = {}
    for t in np.round(tris, 4):
        for a, b in ((0, 1), (1, 2), (2, 0)):
            k = (tuple(t[a]), tuple(t[b]))
            e[k] = e.get(k, 0) + 1
    return all(v == 1 for v in e.values()) and all((k[1], k[0]) in e for k in e)


@pytest.fixture(params=["026", "028", "032"])
def wire(request):
    return get_wire(request.param)


def test_wire_names():
    assert get_wire(".032")["key"] == get_wire("0.032")["key"] == get_wire("32")["key"] == "032"


def test_springback_round_trip(wire):
    sb = Springback.default_for(wire["key"])
    for target in (1, 3, 7, 12, 20, 33, 70):
        assert abs(sb.kept(sb.command_for(target)) - target) < 1e-6


def test_demo_plan_is_clear_of_teeth_and_buildable(tmp_path, wire):
    tris, picks = make_case({"41": 0.3})
    pk = [Pick(np.array(p["p"]), np.array(p["n"]), p["label"]) for p in picks]
    plan, problems, report = run_plan(pk, wire, Machine.load(), Springback.default_for(wire["key"]),
                                      tmp_path, mesh=tris)
    assert problems == []
    assert 20 < plan.total_length < 45
    data = json.loads((tmp_path / "plan.json").read_text())
    assert len(data["turns_deg"]) == len(data["segments_mm"]) - 1
    # wire never presses into a tooth, and touches every incisor
    from retainer_bender.planner import clearance
    _, d, at = clearance(plan, tris)
    assert np.nanmin(d[np.isfinite(d)]) > -0.02
    incisors = [i for i, lab in enumerate(plan.labels) if lab in ("42", "41", "31", "32")]
    assert np.all(np.abs(at[incisors]) < 0.08)
    for name in ("program.txt", "preview.svg", "template_1to1.svg", "check_jig.stl", "report.txt"):
        assert (tmp_path / name).stat().st_size > 0


def test_program_reproduces_the_plan(tmp_path, wire):
    """Feeding and bending as programmed, with the springback the table
    predicts, rebuilds the planned wire (seen from the die)."""
    tris, picks = make_case()
    pk = [Pick(np.array(p["p"]), np.array(p["n"]), p["label"]) for p in picks]
    sb = Springback.default_for(wire["key"])
    plan, _, _ = run_plan(pk, wire, Machine.load(), sb, tmp_path, mesh=tris)
    bends = machine_bends(plan.turns)
    kept = [np.sign(b) * sb.kept(sb.command_for(abs(b))) for b in bends]
    n = len(plan.lengths)
    shape = free_wire(plan.lengths[:n - 1], kept, plan.lengths[-1])
    from retainer_bender.geometry import align_2d
    target = plan.vertices[::-1]
    assert np.max(np.linalg.norm(align_2d(shape, target) - target, axis=1)) < 0.01


def test_jig_is_closed_and_outward(tmp_path, wire):
    tris, picks = make_case()
    pk = [Pick(np.array(p["p"]), np.array(p["n"]), p["label"]) for p in picks]
    run_plan(pk, wire, Machine.load(), Springback.default_for(wire["key"]), tmp_path, mesh=tris)
    jig = load_stl(tmp_path / "check_jig.stl")
    assert signed_volume(jig) > 0
    # every edge of every shell is shared by exactly two triangles
    edges = {}
    for t in np.round(jig, 4):
        for a, b in ((0, 1), (1, 2), (2, 0)):
            k = tuple(sorted((tuple(t[a]), tuple(t[b]))))
            edges[k] = edges.get(k, 0) + 1
    assert set(edges.values()) == {2}
    assert consistently_wound(jig)


def test_stl_round_trip(tmp_path):
    tris, _ = make_case()
    save_stl(tmp_path / "x.stl", tris)
    assert np.allclose(load_stl(tmp_path / "x.stl"), tris, atol=1e-4)


def test_calibration_and_fit(tmp_path, wire):
    m = Machine.load()
    calibration_program(wire, m, Springback.default_for(wire["key"]), tmp_path)
    rows = (tmp_path / "calibration_sheet.csv").read_text().strip().splitlines()
    filled = [rows[0]] + [r + str(max(0.0, float(r.split(",")[2]) * 0.93 - 3.2)) for r in rows[1:]]
    (tmp_path / "sheet.csv").write_text("\n".join(filled) + "\n")
    out = tmp_path / "sb.json"
    fit_springback(tmp_path / "sheet.csv", wire, out)
    sb = Springback.load(out)
    assert sb.calibrated
    assert abs(sb.kept(sb.command_for(10)) - 10) < 1e-6


def test_hooks_curl_both_ends_and_still_build(tmp_path, wire):
    tris, picks = make_case()
    pk = [Pick(np.array(p["p"]), np.array(p["n"]), p["label"]) for p in picks]
    sb = Springback.default_for(wire["key"])
    plain, _, _ = run_plan(pk, wire, Machine.load(), sb, tmp_path / "a", mesh=tris, hooks=Hooks(0))
    hooked, problems, _ = run_plan(pk, wire, Machine.load(), sb, tmp_path / "b", mesh=tris,
                                   hooks=Hooks(90, 2.0))
    assert problems == []
    assert hooked.total_length > plain.total_length + 2 * 2.5
    # two curls add turning (less than 2 x 90 where they are wrapped onto the
    # canine), and wrapped curls stay on the tooth where the pad covers them
    extra = np.abs(np.degrees(hooked.turns)).sum() - np.abs(np.degrees(plain.turns)).sum()
    assert 90 < extra < 260
    from retainer_bender.planner import clearance
    s, d, _ = clearance(hooked, tris)
    ends = ((s < 3) | (s > s[-1] - 3)) & np.isfinite(d)
    assert d[ends].max() < 0.6


def test_program_raises_and_lowers_pin_around_every_bend(wire):
    m = Machine.load()
    sb = Springback.default_for(wire["key"])
    lines, steps, problems = make_program(np.array([3.0, 2.0, 2.0]), np.radians([10, -12]),
                                          wire, m, sb)
    assert problems == []
    cmds = [l.split(";")[0].strip() for l in lines if not l.startswith(";")]
    assert cmds[0] == f"FEED {3.0 + m.lead_in_mm:.3f}"
    # every FEED happens with the pin down
    up = False
    for c in cmds:
        if c == "PIN UP":
            up = True
        elif c == "PIN DOWN":
            up = False
        elif c.startswith("FEED"):
            assert not up
    assert cmds.count("PIN UP") == 2


def _auto_picks():
    tris, picks = make_case({"41": 0.3})
    three = [p for p in picks if p["label"] in ("43m", "41", "33m")]
    return tris, [Pick(np.array(p["p"]), np.array(p["n"]), p["label"]) for p in three]


def test_auto_design_touches_every_incisor(tmp_path, wire):
    tris, pk = _auto_picks()
    plan, problems, _ = run_plan(pk, wire, Machine.load(), Springback.default_for(wire["key"]),
                                 tmp_path, mesh=tris, style="auto")
    assert problems == []
    from retainer_bender.planner import clearance
    _, d, at = clearance(plan, tris)
    assert np.min(d[np.isfinite(d)]) > -0.02            # never into a tooth
    assert len(at) == 6                                 # two ends + four incisors found
    assert np.all(np.abs(at) < 0.08)                    # and touching each


def test_wire_stl_is_closed_and_sits_on_the_scan(tmp_path):
    wire = get_wire("032")
    tris, pk = _auto_picks()
    plan, _, _ = run_plan(pk, wire, Machine.load(), Springback.default_for("032"), tmp_path,
                          mesh=tris, style="auto")
    w = load_stl(tmp_path / "wire.stl")
    assert signed_volume(w) > 0
    assert consistently_wound(w)
    from retainer_bender.geometry import arc_lengths
    sides = 16
    area = 0.5 * sides * (wire["d_mm"] / 2) ** 2 * np.sin(2 * np.pi / sides)
    expected = area * arc_lengths(plan.curve)[-1]
    assert abs(signed_volume(w) - expected) / expected < 0.02
    from retainer_bender.mesh import signed_distances
    d = signed_distances(w.reshape(-1, 3), tris)
    assert np.min(d[np.isfinite(d)]) > -0.03


def test_occlusion_check_flags_close_upper_teeth(tmp_path):
    wire = get_wire("032")
    tris, pk = _auto_picks()
    sb = Springback.default_for("032")
    far = tris + [0, 0, 30]
    _, _, text = run_plan(pk, wire, Machine.load(), sb, tmp_path / "far", mesh=tris,
                          style="auto", opposing=far)
    assert "upper teeth come within" not in text
    # a fake "upper" sitting right on the wire's lingual side
    near = tris + [0, -1.2, 1.5]
    _, _, text = run_plan(pk, wire, Machine.load(), sb, tmp_path / "near", mesh=tris,
                          style="auto", opposing=near)
    assert "upper teeth come within" in text


def test_gingival_moves_the_wire_down_and_keeps_contact(tmp_path):
    wire = get_wire("032")
    tris, pk = _auto_picks()
    sb = Springback.default_for("032")
    base, _, _ = run_plan(pk, wire, Machine.load(), sb, tmp_path / "a", mesh=tris, style="auto")
    low, problems, _ = run_plan(pk, wire, Machine.load(), sb, tmp_path / "b", mesh=tris,
                                style="auto", gingival=1.0)
    assert problems == []
    # synthetic teeth stand along +z, so gingival is -z
    drop = float(np.mean(low.to_3d(low.curve)[:, 2]) - np.mean(base.to_3d(base.curve)[:, 2]))
    assert -1.2 < drop < -0.8
    from retainer_bender.planner import clearance
    _, d, at = clearance(low, tris)
    assert np.min(d[np.isfinite(d)]) > -0.02
    assert np.all(np.abs(at) < 0.08)


def test_elbow_ends_fit_and_unfold_to_a_flat_wire_of_the_same_length(tmp_path):
    from retainer_bender.geometry import arc_lengths
    from retainer_bender.planner import clearance
    wire = get_wire("032")
    tris, pk = _auto_picks()
    plan, problems, text = run_plan(pk, wire, Machine.load(), Springback.default_for("032"),
                                    tmp_path, mesh=tris, style="auto", ends="elbow")
    assert problems == []                      # the flat wire is machine-buildable
    assert plan.finish and len(plan.finish) == 2
    for f in plan.finish:
        assert 1.0 <= f["leg_mm"] <= 2.0
        assert f["joggle_from_mm"] > f["elbow_mm"]
        assert 45 <= f["elbow_deg"] <= 120
    # unfolding the step and elbows keeps the wire's length
    final_len = arc_lengths(plan.curve3())[-1]
    flat_len = arc_lengths(plan.prefinish_dense)[-1]
    assert abs(final_len - flat_len) < 0.05
    # the finished wire does not press into the teeth and drops at the canines
    _, d, _ = clearance(plan, tris)
    assert np.min(d[np.isfinite(d)]) > -0.05
    assert abs(plan.curve_w[0]) > 0.5 and abs(plan.curve_w[-1]) > 0.5
    assert "Finishing" in text


def test_elbow_without_step_stays_on_the_teeth(tmp_path):
    from retainer_bender.planner import clearance
    wire = get_wire("032")
    tris, pk = _auto_picks()
    plan, problems, text = run_plan(pk, wire, Machine.load(), Springback.default_for("032"),
                                    tmp_path, mesh=tris, style="auto", ends="elbow", elbow_step=0.0)
    assert problems == []
    assert all(f["joggle_from_mm"] is None for f in plan.finish)
    _, d, at = clearance(plan, tris)
    assert np.min(d[np.isfinite(d)]) > -0.05
    assert np.all(np.abs(at) < 0.08)
    assert "no step" in text
