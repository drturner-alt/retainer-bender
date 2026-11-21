"""rbend: plan, calibrate and send lower 3-3 lingual retainer wires.

    rbend plan picks.json --scan lower.stl --wire 032 -o out/
    rbend demo -o demo/
    rbend calibrate --wire 032 -o calib/
    rbend send out/program.txt --port /dev/ttyACM0
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from .machine import Machine, machine_bends, make_program, simulate
from .mesh import load_stl, save_stl
from .outputs import jig_stl, preview_svg, template_svg, wire_stl
from .planner import (Hooks, Pick, clearance, load_picks, plan_wire, predicted_shape,
                      uncorrected_shape)
from .springback import Springback
from .wires import get_wire


def _report(plan, steps, problems, sb, built_err, raw_err, clr, out, machine, occl=None):
    w = plan.wire
    L = [f"Lower 3-3 lingual retainer, {w['label']} ({w['d_mm']:.3f} mm), style {plan.style}"
         + ((f", ends: gingival step at the lateral-canine embrasure + {plan.finish[0]['elbow_deg']:.0f} deg elbow toward the gingiva"
             if plan.finish_step else
             f", ends: {plan.finish[0]['elbow_deg']:.0f} deg elbow toward the gingiva at each canine (no step)")
            if plan.finish else
            f", 3D with {plan.canine_step:.1f} mm gingival steps onto the canines" if plan.is3d else ""),
         f"Wire length {plan.total_length:.2f} mm, {len(plan.turns)} bends",
         f"Straight segments follow the curve to within {plan.path_error:.3f} mm",
         f"Predicted shape after springback correction: within {built_err:.3f} mm of target",
         f"Without correction it would be off by up to {raw_err:.2f} mm",
         "" if sb.calibrated else
         "!! Springback table is a PLACEHOLDER. Calibrate before bending for a patient.",
         "",
         "Clicks (out of plane = how far the flat wire passes above/below the click):"]
    for i, lab in enumerate(plan.labels):
        L.append(f"  {lab:<8} out of plane {plan.out_of_plane[i]:+.2f} mm")
    if clr is not None:
        L.append("Wire contacts (gap between wire and tooth; 0 = touching):")
        for lab, gap in zip(plan.contact_labels, clr[2]):
            L.append(f"  {lab:<8} {gap:+.2f} mm")
    if clr is not None:
        s, d, _ = clr
        worst = float(np.min(d[np.isfinite(d)])) if np.isfinite(d).any() else float("nan")
        L.append(f"Closest approach along the wire: {worst:+.2f} mm "
                 "(negative = wire would press into a tooth)")
        ends = (s < 3.0) | (s > s[-1] - 3.0)
        fin = np.isfinite(d) & ends
        if fin.any():
            stand = float(d[fin].max())
            L.append(f"End 3 mm at each canine: up to {stand:.2f} mm off the tooth "
                     "(the composite pad must cover this)")
            if stand > 0.6:
                plan.warnings.append(f"an end of the wire stands {stand:.2f} mm off the canine; "
                                     "try --hook-toward tooth or a smaller --hook-angle")
    if occl is not None:
        L.append(f"Occlusion: nearest upper tooth is {occl:.2f} mm from the wire")
    L += ["", "Bend list, from the first end out (also works for bending by hand):",
          f"  (the machine feeds {machine.lead_in_mm:.1f} mm extra before bend 1; trim it off after)",
          "   #   feed mm   keep deg   command deg   pin touches   arm to"
          + ("   roll deg" if plan.is3d else "")]
    k = 0
    feeds = [st["mm"] for st in steps if st["op"] == "feed"]
    feeds[0] -= machine.lead_in_mm
    for st in steps:
        if st["op"] != "bend":
            continue
        row = f"  {k + 1:>2}   {feeds[k]:7.2f}   {st['target_deg']:+8.2f}   {st['command_deg']:+11.2f}"
        if "arm_deg" in st:
            row += f"   {st['contact_deg']:+10.2f}   {st['arm_deg']:+7.2f}"
            if "roll_deg" in st:
                row += f"   {st['roll_deg']:+8.1f}"
        L.append(row)
        k += 1
    L.append(f"  last  {feeds[-1]:7.2f}   (then cut)")
    if plan.finish:
        names = [plan.labels[0], plan.labels[-1]]
        L += ["", "Finishing (after the machine; measure along the wire from each end):"]
        for nm, f in zip(names, plan.finish):
            row = (f"  {nm} end: bend {f['elbow_deg']:.0f} deg toward the gingiva at {f['elbow_mm']:.2f} mm "
                   f"(leg {f['leg_mm']:.2f} mm)")
            if f["joggle_from_mm"] is not None:
                row += (f"; joggle {plan.finish_step:.2f} mm gingival between "
                        f"{f['joggle_from_mm']:.2f} and {f['joggle_to_mm']:.2f} mm "
                        f"(total offset {f['joggle_offset_mm']:.2f} mm, down and lingual)")
            L.append(row)
    L += ["", "Warnings:"] + [f"  - {x}" for x in plan.warnings] if plan.warnings else []
    L += ["", "Machine checks:"] + ([f"  - {x}" for x in problems] if problems else ["  all passed"])
    text = "\n".join(L) + "\n"
    (out / "report.txt").write_text(text)
    return text


class _NoCorrection:
    """Bend to the target angle as if nothing sprang back (to show what
    springback would cost)."""
    def __init__(self, sb):
        self.sb = sb

    def command_for(self, target):
        return max(target, 0.0)

    def kept(self, commanded):
        return self.sb.kept(commanded)


def run_plan(picks, wire, machine, sb, out, mesh=None, style="smooth", contact_offset=0.0,
             hooks=Hooks(), opposing=None, occlusion_min=0.5, gingival=0.0, canine_step=0.0,
             ends="curl", elbow_step=0.75, elbow_deg=60.0, elbow_distal=1.0, leg=2.0):
    out.mkdir(parents=True, exist_ok=True)
    plan = plan_wire(picks, wire, machine, style=style, contact_offset=contact_offset, mesh=mesh,
                     hooks=hooks, gingival=gingival, canine_step=canine_step, ends=ends,
                     elbow_step=elbow_step, elbow_deg=elbow_deg, elbow_distal=elbow_distal,
                     leg_range=(min(1.0, leg), leg))
    if plan.prefinish is not None:
        # the machine bends the flat wire; the step and elbows are formed after
        from .geometry import align_2d, build_polyline, point_to_polyline, segment_lengths, turning_angles
        L2 = segment_lengths(plan.prefinish)
        T2 = turning_angles(plan.prefinish)
        lines, steps, problems = make_program(L2, T2, wire, machine, sb)
        kept = np.array([np.sign(b) * sb.kept(sb.command_for(abs(b))) for b in np.degrees(T2)])
        built = align_2d(build_polyline(L2, np.radians(kept)), plan.prefinish)
        built_err = float(point_to_polyline(plan.prefinish_dense, built).max())
        raw = np.array([np.sign(b) * sb.kept(abs(b)) for b in np.degrees(T2)])
        raw_err = float(point_to_polyline(plan.prefinish_dense,
                                          align_2d(build_polyline(L2, np.radians(raw)), plan.prefinish)).max())
        built = None
    elif plan.is3d:
        from .machine3d import built_error, make_program_3d
        lines, steps, problems, P = make_program_3d(plan.vertices3, wire, machine, sb)
        built = None
        built_err = max(plan.path_error, built_error(plan.vertices3, P, machine.lead_in_mm))
        _, _, _, Praw = make_program_3d(plan.vertices3, wire, machine, _NoCorrection(sb))
        raw_err = built_error(plan.vertices3, Praw, machine.lead_in_mm)
        lines = ["; retainer-bender program (3D: ROT turns the wire about its own axis)",
                 f"; wire {wire['label']}  ({wire['d_mm']:.3f} mm), canine steps {plan.canine_step:.2f} mm",
                 f"; springback table: {sb.source or 'built-in'}"
                 + ("" if sb.calibrated else "  ** UNCALIBRATED PLACEHOLDER **"),
                 "; load: tip flush with the die exit, clamp closed, pin down",
                 f"; afterwards trim {machine.lead_in_mm:.1f} mm of straight lead-in off the first end out",
                 ] + lines
    else:
        built, built_err = predicted_shape(plan, sb)
        _, raw_err = uncorrected_shape(plan, sb)
        lines, steps, problems = make_program(plan.lengths, plan.turns, wire, machine, sb)
    clr = clearance(plan, mesh) if mesh is not None else None

    (out / "program.txt").write_text("\n".join(lines) + "\n")
    preview_svg(plan, out / "preview.svg", mesh=mesh, built=built, clearance_info=clr)
    template_svg(plan, out / "template_1to1.svg")
    jig_stl(plan, out / "check_jig.stl")
    wire_stl(plan, out / "wire.stl")
    occl = None
    if opposing is not None:
        from .mesh import signed_distances
        pts, _ = plan.world(0.2)
        d = signed_distances(pts, opposing, search_radius=4.0) - wire["d_mm"] / 2
        occl = float(np.min(d)) if np.isfinite(d).any() else None
    (out / "plan.json").write_text(json.dumps({
        "wire": wire, "style": plan.style, "length_mm": plan.total_length,
        "labels": plan.labels,
        "plane": {"origin": plan.origin.tolist(), "u": plan.e_u.tolist(),
                  "v": plan.e_v.tolist(), "normal": plan.normal.tolist()},
        "centers_2d": plan.centers_2d.tolist(),
        "vertices_2d": plan.vertices.tolist(),
        "centerline_3d": (plan.to_3d(plan.vertices3[:, :2], plan.vertices3[:, 2]) if plan.is3d
                          else plan.to_3d(plan.vertices)).tolist(),
        "canine_step_mm": plan.canine_step,
        "segments_mm": plan.lengths.tolist(),
        "turns_deg": np.degrees(plan.turns).tolist(),
        "machine_bends_deg": None if plan.is3d else machine_bends(plan.turns).tolist(),
        "steps": steps, "problems": problems, "warnings": plan.warnings,
        "springback": {"source": sb.source, "calibrated": sb.calibrated},
    }, indent=1))
    if occl is not None and occl < occlusion_min:
        plan.warnings.append(f"upper teeth come within {occl:.2f} mm of the wire in occlusion "
                             f"(want at least {occlusion_min:.1f}); move the wire gingivally")
    text = _report(plan, steps, problems, sb, built_err, raw_err, clr, out, machine, occl)
    return plan, problems, text


def calibration_program(wire, machine, sb, out, angles=(4, 6, 8, 10, 12, 15, 20, 25, 30), seg=12.0):
    """One test wire: straight pieces with alternating bends between them.

    Alternating the direction keeps the wire from curling into the machine
    and checks both directions. Measure each angle and enter the results in
    calibration_sheet.csv, then run `rbend fit-springback`.
    """
    out.mkdir(parents=True, exist_ok=True)
    cmds = [a if k % 2 == 0 else -a for k, a in enumerate(angles)]
    kept = [np.sign(c) * sb.kept(abs(c)) for c in cmds]
    lengths = [seg] * (len(angles) + 1)
    lines, _, problems = simulate(lengths, cmds, kept, wire, machine,
                                  [f"; test bend {k + 1}" for k in range(len(angles))])
    if problems:
        raise SystemExit("calibration program won't run on this machine:\n  " + "\n  ".join(problems))
    lines = ["; retainer-bender springback calibration",
             f"; wire {wire['label']}; each ARM line turns past first contact by the commanded angle",
             "; load: tip flush with the die exit, pin down"] + lines
    rows = ["bend,direction,commanded_deg,measured_deg"]
    rows += [f"{k + 1},{'+' if c > 0 else '-'},{abs(c)}," for k, c in enumerate(cmds)]
    (out / "calibration_program.txt").write_text("\n".join(lines) + "\n")
    (out / "calibration_sheet.csv").write_text("\n".join(rows) + "\n")
    return lines


def fit_springback(csv_path, wire, out_path):
    import csv
    rows = list(csv.DictReader(open(csv_path)))
    pairs = sorted((float(r["commanded_deg"]), float(r["measured_deg"]))
                   for r in rows if r.get("measured_deg", "").strip())
    if len(pairs) < 3:
        raise SystemExit("enter at least three measured angles first")
    # Average repeats, then force the table to rise (measuring noise can make a
    # larger command read smaller), and add the origin and the elastic limit.
    by_cmd = {}
    for c, m in pairs:
        by_cmd.setdefault(c, []).append(m)
    table = [[c, float(np.mean(ms))] for c, ms in sorted(by_cmd.items())]
    for i in range(1, len(table)):
        table[i][1] = max(table[i][1], table[i - 1][1] + 0.05)
    c = np.array([t[0] for t in table])
    m = np.array([t[1] for t in table])
    slope, icpt = np.polyfit(m[:3], c[:3], 1)        # commanded at which nothing is kept
    elastic = max(0.0, float(icpt))
    if elastic < c[0] and m[0] > 0:
        table = [[0.0, 0.0], [elastic, 0.01]] + table
    else:
        table = [[0.0, 0.0]] + table
    Path(out_path).write_text(json.dumps({"wire": wire["label"], "calibrated": True,
                                          "from": str(csv_path), "table": table}, indent=1))
    return table


def main(argv=None):
    ap = argparse.ArgumentParser(prog="rbend", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p):
        p.add_argument("--wire", default="032", help="026, 028 or 032 (default 032)")
        p.add_argument("--machine", help="machine.json (default config/machine.json)")
        p.add_argument("--springback", help="springback table json (default: placeholder for the wire)")
        p.add_argument("-o", "--out", type=Path, required=True)

    def hook_args(p):
        p.add_argument("--hook-angle", type=float, default=90.0,
                       help="degrees each canine end curls round (0 = straight ends; default 90)")
        p.add_argument("--hook-radius", type=float, default=2.0,
                       help="radius of the end curls in mm (default 2.0)")
        p.add_argument("--hook-toward", choices=["tooth", "tongue"], default="tooth",
                       help="tooth (default): the curls wrap onto the canine, under the pad")

    p = sub.add_parser("plan", help="plan a wire from picked points")
    p.add_argument("picks", type=Path)
    p.add_argument("--scan", type=Path, help="lower scan STL, for clearance checks and the preview")
    p.add_argument("--opposing", type=Path,
                   help="upper scan STL in occlusion (as iTero exports it), for the occlusion check")
    p.add_argument("--style", choices=["auto", "smooth", "polyline"], default=None,
                   help="auto: design from 3 clicks (43, midline, 33) and the scan; "
                        "smooth/polyline: through every click. Default: auto for 3 clicks")
    p.add_argument("--contact-offset", type=float, default=0.0,
                   help="mm to stand the wire off the teeth (negative presses it in)")
    p.add_argument("--gingival", type=float, default=0.0,
                   help="mm to move the clicks toward the gingiva (negative: toward the incisal)")
    p.add_argument("--ends", choices=["elbow", "curl"], default="elbow",
                   help="elbow (default): 0.75 mm gingival step at the lateral-canine embrasure, then a "
                        "90 deg elbow toward the gingiva at the canine click; curl: flat curls")
    p.add_argument("--elbow-deg", type=float, default=60.0,
                   help="with elbow ends: how far the wire turns toward the gingiva (default 60)")
    p.add_argument("--leg", type=float, default=2.0,
                   help="with elbow ends: leg length in mm (default 2; shortened, not below 1, "
                        "to stay 2 mm off the gingiva when gum clicks are given)")
    p.add_argument("--elbow-distal", type=float, default=1.0,
                   help="with elbow ends: mm past the canine click, toward the distal, to put the elbow")
    p.add_argument("--elbow-step", type=float, default=0.0,
                   help="with elbow ends: gingival step at the lateral-canine embrasure in mm (default 0 = none)")
    p.add_argument("--canine-step", type=float, default=0.0,
                   help="3D: drop the wire this many mm toward the gingiva onto each canine; the canine "
                        "clicks are then where it lands (needs the roll motor)")
    hook_args(p)
    common(p)

    p = sub.add_parser("demo", help="plan a wire for a built-in synthetic arch")
    p.add_argument("--lingual", type=float, default=0.3,
                   help="mm to set tooth 41 lingual of its neighbours (default 0.3)")
    p.add_argument("--auto", action="store_true", help="use the 3-click auto design")
    hook_args(p)
    common(p)

    p = sub.add_parser("calibrate", help="write a springback test program")
    common(p)

    p = sub.add_parser("fit-springback", help="turn a filled-in calibration sheet into a table")
    p.add_argument("sheet", type=Path)
    p.add_argument("--wire", default="032")
    p.add_argument("-o", "--out", type=Path, required=True)

    p = sub.add_parser("send", help="send a program to the machine")
    p.add_argument("program", type=Path)
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--no-home", action="store_true")
    p.add_argument("--dry-run", action="store_true")

    a = ap.parse_args(argv)

    if a.cmd == "send":
        from .sender import send
        return send(a.program, a.port, a.baud, home=not a.no_home, dry_run=a.dry_run)
    if a.cmd == "fit-springback":
        table = fit_springback(a.sheet, get_wire(a.wire), a.out)
        print(f"wrote {a.out} with {len(table)} rows")
        return 0

    wire = get_wire(a.wire)
    machine = Machine.load(a.machine)
    sb = Springback.load(a.springback) if a.springback else Springback.default_for(wire["key"])

    if a.cmd == "calibrate":
        calibration_program(wire, machine, sb, a.out)
        print(f"wrote {a.out / 'calibration_program.txt'} and calibration_sheet.csv")
        return 0

    hooks = Hooks(a.hook_angle, a.hook_radius, a.hook_toward) if a.cmd in ("plan", "demo") else None
    if a.cmd == "demo":
        from .synthetic import make_case
        tris, picks = make_case({"41": a.lingual} if a.lingual else None)
        a.out.mkdir(parents=True, exist_ok=True)
        save_stl(a.out / "synthetic_lower.stl", tris)
        (a.out / "picks.json").write_text(json.dumps({"units": "mm", "points": picks}, indent=1))
        if a.auto:
            picks = [x for x in picks if x["label"] in ("43m", "41", "33m")]
            (a.out / "picks.json").write_text(json.dumps({"units": "mm", "points": picks}, indent=1))
        pk = [Pick(np.array(x["p"]), np.array(x["n"]), x["label"]) for x in picks]
        _, problems, text = run_plan(pk, wire, machine, sb, a.out, mesh=tris, hooks=hooks,
                                     style="auto" if a.auto else "smooth")
    else:
        mesh = load_stl(a.scan) if a.scan else None
        picks = load_picks(a.picks)
        style = a.style or ("auto" if len(picks) == 3 and mesh is not None else "smooth")
        _, problems, text = run_plan(picks, wire, machine, sb, a.out, mesh=mesh,
                                     style=style, contact_offset=a.contact_offset, hooks=hooks,
                                     opposing=load_stl(a.opposing) if a.opposing else None,
                                     gingival=a.gingival, canine_step=a.canine_step,
                                     ends=a.ends if style == "auto" else "curl",
                                     elbow_step=a.elbow_step, elbow_deg=a.elbow_deg,
                                     elbow_distal=a.elbow_distal, leg=a.leg)
    print(text)
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
