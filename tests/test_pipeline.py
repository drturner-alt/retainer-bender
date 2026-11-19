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


@pytest.fixture(params=["028", "032"])
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
    T = np.degrees(hooked.turns)
    s = np.cumsum(hooked.lengths)[:-1]
    head = T[s < 5.0].sum()
    tail = T[s > hooked.total_length - 5.0].sum()
    assert abs(head) > 70 and abs(tail) > 70


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
