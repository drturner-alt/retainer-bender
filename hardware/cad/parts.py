"""Printed parts for the bend head, as code (version 0).

    python hardware/cad/parts.py            # writes hardware/cad/stl/*.stl

Coordinates: millimetres. Origin on the bend arm's axis, at the top face of
the aluminium top plate. The wire leaves the die travelling +x. z is up.

Numbers marked VERIFY come from listings, not from parts in hand. Measure
them with calipers when the parts arrive, change them here, and re-run.

Stack-up along z (pin down / pin up):
    gearbox under the top plate, keyed 14 mm shaft up through it
    arm disc        ARM_Z0 .. ARM_Z1   clamped to the shaft top, carries the pin
    lift ring       below the arm disc, slides on two guide rods, raised by a servo
    housing slab    SLAB_Z0 .. SLAB_Z1 (the deck), arc slot for the pin
    die tongue      on the slab; the wire runs on its top face at TONGUE_TOP
"""

import math
import sys
from pathlib import Path

import numpy as np
from manifold3d import CrossSection, Manifold

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from retainer_bender.mesh import save_stl  # noqa: E402

SEG = 96                       # circle facets

# ---- bought parts -------------------------------------------------------
SHAFT_D = 14.0                 # gearbox output shaft, keyed (VERIFY)
KEY_W = 5.0                    # key width (VERIFY: 5 x 5 key for a 14 mm shaft)
KEY_HUB_DEPTH = 2.3            # keyway depth in the hub beyond the shaft surface
SHAFT_TOP = 28.0               # shaft tip above the top plate (VERIFY: 34 mm shaft, 6 mm plate)
PIN_D = 3.0                    # bend pin and die pins: hardened dowel pins
PIN_L = 24.0                   # bend pin length (3 x 24 dowel)
DIE_PIN_L = 8.0                # die pins (3 x 8 dowel)
GUIDE_ROD_D = 3.0              # lift ring guide rods (3 x 20 dowel)
M3_CLEAR, M3_NUT_AF, M3_NUT_T = 3.4, 5.6, 2.6
M4_CLEAR, M4_NUT_AF, M4_NUT_T = 4.5, 7.2, 3.4
TUBE_OD = {"032": 0.042 * 25.4, "028": 0.050 * 25.4, "026": 0.050 * 25.4}   # McMaster 5560K49 / 8988K43
WIRE_D = {"032": 0.032 * 25.4, "028": 0.028 * 25.4, "026": 0.026 * 25.4}

# ---- machine geometry (matches config/machine.json) --------------------
ARM_R = 10.0                   # bend pin radius from the arm axis
DIE_HALF = 3.5                 # die nose half-width
ARM_SWEEP = 124.0              # arc slot reaches +/- this many degrees (arm max 110 + pin width)

# ---- stack-up -----------------------------------------------------------
ARM_Z0, ARM_Z1 = 17.0, 27.0    # arm disc
SLAB_Z0, SLAB_Z1 = 29.0, 33.0  # housing top slab (the deck)
TONGUE_T = 5.0
TONGUE_TOP = SLAB_Z1 + TONGUE_T            # the wire rests here
PIN_TRAVEL = 2.0
PIN_DOWN_TOP = TONGUE_TOP - 0.4            # just under the wire
RING_TOP_DOWN = PIN_DOWN_TOP - PIN_L       # lift ring top face, pin down
RING_T = 4.0

# ---- housing ------------------------------------------------------------
HOUSING_X = (-60.0, 40.0)
HOUSING_Y = (-35.0, 35.0)
WALL = 4.0
TONGUE_SCREWS = [(-18.0, 6.0), (-18.0, -6.0), (-32.0, 6.0), (-32.0, -6.0)]
GUIDE_RODS = [(0.0, 20.0), (0.0, -20.0)]
HOUSING_SCREWS = [(-65.0, 24.0), (-65.0, -24.0), (45.0, 24.0), (45.0, -24.0)]   # outside ears

# ---- aluminium top plate (drilled to a printed template) ------------------
PLATE = (-210.0, 90.0, -75.0, 75.0)   # x0, x1, y0, y1: 300 x 150 mm; the feed stage sits behind (-x)
GEARBOX_PILOT_D = 40.0          # clearance for the gearbox's output boss (VERIFY)
GEARBOX_BOLTS = 47.14           # square bolt pattern on the gearbox face (VERIFY)
GEARBOX_BOLT_D = 5.5            # M5 clearance (VERIFY thread size)
SERVO_BRACKET_SCREWS = None     # filled in below from the servo geometry

# ---- servo (MG90S; VERIFY against the one you buy) ----------------------
SERVO_BODY = (22.8, 12.2, 22.7)   # length (x), width (z), height (y, below the horn)
SERVO_AXIS = (-32.0, 6.8)         # horn axis (x, z); the axis runs along y
SERVO_FACE_Y = -6.5               # servo top face (horn side); the horn sits just past it
SERVO_HORN = (14.0, 2.0, 5.0)     # horn arm length from the axis, thickness, width (VERIFY)
SERVO_HORN_GAP = 4.5              # top face to the horn's far face
SERVO_SHAFT_FROM_END = 5.9        # shaft centre from the near end of the body
SERVO_TAB_SPAN = 27.8             # between the two tab screw holes


def cyl(r, z0, z1, x=0.0, y=0.0, seg=SEG):
    return Manifold.cylinder(z1 - z0, r, r, seg).translate((x, y, z0))


def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube((x1 - x0, y1 - y0, z1 - z0)).translate((x0, y0, z0))


def hex_prism(af, z0, z1, x, y):
    r = af / math.sqrt(3)
    pts = [(x + r * math.cos(math.radians(60 * i)), y + r * math.sin(math.radians(60 * i)))
           for i in range(6)]
    return Manifold.extrude(CrossSection([pts]), z1 - z0).translate((0, 0, z0))


def fan(a0, a1, r, z0, z1, n=64):
    """Pie slice from angle a0 to a1 (degrees), radius r."""
    pts = [(0.0, 0.0)] + [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a)))
                          for a in np.linspace(a0, a1, n)]
    return Manifold.extrude(CrossSection([pts]), z1 - z0).translate((0, 0, z0))


def die_y(wire):
    return WIRE_D[wire] / 2 + PIN_D / 2


# ---- parts ---------------------------------------------------------------

def arm_disc():
    """Clamps the shaft top on its key; carries the bend pin at ARM_R."""
    od = 2 * (ARM_R + 4.0)
    part = cyl(od / 2, ARM_Z0, ARM_Z1)
    part -= cyl(SHAFT_D / 2 + 0.05, ARM_Z0 - 1, ARM_Z1 + 1)
    # keyway toward -x, away from the pin
    part -= box(-(SHAFT_D / 2 + KEY_HUB_DEPTH), 0, -KEY_W / 2 - 0.05, KEY_W / 2 + 0.05,
                ARM_Z0 - 1, ARM_Z1 + 1)
    # bend pin bore at +x: a close sliding fit (ream to 3.0 H7 after printing)
    part -= cyl(PIN_D / 2 + 0.05, ARM_Z0 - 1, ARM_Z1 + 1, x=ARM_R)
    # two M4 set screws onto the shaft at +/-y, nuts captured in pockets
    zc = (ARM_Z0 + ARM_Z1) / 2
    # M4 thread clearance straight through along y
    part -= Manifold.cylinder(od + 2, M4_CLEAR / 2 - 0.4, M4_CLEAR / 2 - 0.4, 32,
                              center=True).rotate((90, 0, 0)).translate((0, 0, zc))
    for s in (1, -1):
        # nut slot, dropped in from the top face
        w = M4_NUT_AF + 0.3
        t = M4_NUT_T + 0.3
        yc = s * (SHAFT_D / 2 + 2.4)
        part -= box(-w / 2, w / 2, yc - t / 2, yc + t / 2, zc - w / 2, ARM_Z1 + 1)
    return part


def lift_ring():
    """Pushes the pin up; slides on two guide rods; the servo horn lifts the rear tab."""
    z0 = RING_TOP_DOWN - RING_T
    z1 = RING_TOP_DOWN
    part = cyl(ARM_R + 3.0, z0, z1) - cyl(SHAFT_D / 2 + 1.5, z0 - 1, z1 + 1)
    for gx, gy in GUIDE_RODS:
        part += cyl(4.0, z0, z1, gx, gy) + box(-4, 4, min(0, gy), max(0, gy), z0, z1)
    part += box(-24.0, -ARM_R, -4.0, 4.0, z0, z1)        # rear tab for the servo horn
    part += cyl(4.0, z0, z1, -24.0, 0.0)
    for gx, gy in GUIDE_RODS:
        part -= cyl(GUIDE_ROD_D / 2 + 0.15, z0 - 1, z1 + 1, gx, gy)
    part -= cyl(SHAFT_D / 2 + 1.5, z0 - 1, z1 + 1)
    return part


def housing():
    """The deck: a shell over the arm with the pin's arc slot in its top slab."""
    x0, x1 = HOUSING_X
    y0, y1 = HOUSING_Y
    outer = box(x0, x1, y0, y1, 0, SLAB_Z1)
    inner = box(x0 + WALL, x1 - WALL, y0 + WALL, y1 - WALL, -1, SLAB_Z0)
    part = outer - inner
    # base flanges with M4 screws into the top plate
    for hx, hy in HOUSING_SCREWS:
        ex0, ex1 = (hx - 5.5, x0 + 1) if hx < x0 else (x1 - 1, hx + 5.5)
        part += box(ex0, ex1, hy - 5.5, hy + 5.5, 0, 6.0) + cyl(5.5, 0, 6.0, hx, hy)
        part -= cyl(M4_CLEAR / 2, -1, 7, hx, hy)
    # arc slot for the bend pin, clear of the arm disc below
    slot = (cyl(ARM_R + 2.0, SLAB_Z0 - 1, SLAB_Z1 + 1) - cyl(ARM_R - 2.0, SLAB_Z0 - 2, SLAB_Z1 + 2))
    part -= slot ^ fan(-ARM_SWEEP, ARM_SWEEP, ARM_R + 4, SLAB_Z0 - 1, SLAB_Z1 + 1)
    # tongue: shallow locating pocket, screws with nuts captured underneath
    part -= tongue_outline(0.15, SLAB_Z1 - 0.6, SLAB_Z1 + 1)
    for tx, ty in TONGUE_SCREWS:
        part -= cyl(M3_CLEAR / 2, SLAB_Z0 - 1, SLAB_Z1 + 1, tx, ty)
        part -= hex_prism(M3_NUT_AF + 0.3, SLAB_Z0 - 1, SLAB_Z0 + M3_NUT_T, tx, ty)
    # die pin clearance: the die pins stop at the tongue's underside, but give
    # them room in case a pin is pushed through
    part -= box(-1.8, 1.8, -2.5, 2.5, SLAB_Z1 - 1.5, SLAB_Z1 + 1)
    # guide rod bosses hanging from the slab
    for gx, gy in GUIDE_RODS:
        part += cyl(3.5, RING_TOP_DOWN + PIN_TRAVEL + 1.0, SLAB_Z0, gx, gy)
        part -= cyl(GUIDE_ROD_D / 2 - 0.05, RING_TOP_DOWN + PIN_TRAVEL + 0.5, SLAB_Z0 - 0.5, gx, gy)
    # windows: rear for the servo cable, side for looking in
    part -= box(x0 - 1, x0 + WALL + 1, -8, 8, 3, 15)
    part -= box(-20, 10, y0 - 1, y0 + WALL + 1, 4, 20)
    return part


def tongue_outline(grow, z0, z1):
    """Narrow nose (half-width DIE_HALF) ending in a half-round on the axis;
    wider behind x = -11 where the pin never reaches."""
    nose = (cyl(DIE_HALF + grow, z0, z1)
            + box(-11.0, 0.0, -DIE_HALF - grow, DIE_HALF + grow, z0, z1))
    tail = box(-40.0 - grow, -11.0, -9.0 - grow, 9.0 + grow, z0, z1)
    return nose + tail


def die_tongue(wire):
    """Holds the two die pins and the guide tube; one per wire size.

    Printed in Rigid 10K for first tests. For daily use have it made in
    steel from this same shape.
    """
    z0 = SLAB_Z1 - 0.5      # sits in the 0.6 mm pocket
    part = tongue_outline(0.0, z0, TONGUE_TOP)
    y = die_y(wire)
    for s in (1, -1):                      # die pins, press fit
        part -= cyl(PIN_D / 2 - 0.04, z0 - 1, TONGUE_TOP + 1, 0.0, s * y)
    od = TUBE_OD[wire]
    zc = TONGUE_TOP + WIRE_D[wire] / 2     # tube centre = wire centre
    tube = Manifold.cylinder(60, od / 2 + 0.03, od / 2 + 0.03, 32).rotate((0, 90, 0))
    part -= tube.translate((-60 - (PIN_D / 2 + 0.2), 0, zc))
    for tx, ty in TONGUE_SCREWS:
        part -= cyl(M3_CLEAR / 2, z0 - 1, TONGUE_TOP + 1, tx, ty)
    return part


def tube_clamp(wire):
    """Holds the guide tube down in the tongue's groove."""
    od = TUBE_OD[wire]
    zc = TONGUE_TOP + WIRE_D[wire] / 2
    z1 = zc + od / 2 + 3.0
    part = box(-36.0, -14.0, -9.0, 9.0, TONGUE_TOP, z1)
    tube = Manifold.cylinder(60, od / 2 - 0.02, od / 2 - 0.02, 32).rotate((0, 90, 0))
    part -= tube.translate((-60, 0, zc))
    for tx, ty in TONGUE_SCREWS:
        part -= cyl(M3_CLEAR / 2, TONGUE_TOP - 1, z1 + 1, tx, ty)
    return part


def servo_bracket():
    """Holds the lift servo on its side, horn axis along y, under the ring's rear tab."""
    ax, az = SERVO_AXIS
    L, W, H = SERVO_BODY
    bx0 = ax - (L - SERVO_SHAFT_FROM_END)       # body runs toward -x from the shaft
    bx1 = ax + SERVO_SHAFT_FROM_END
    y_face = SERVO_FACE_Y
    part = box(bx0 - 6, bx1 + 6, y_face - H - 1.5, y_face, 0, az + W / 2 + 3)
    part -= box(bx0 - 0.3, bx1 + 0.3, y_face - H - 0.5, y_face + 1, az - W / 2 - 0.3, az + W / 2 + 0.3)
    # tab screw holes (M2 self-tapping)
    for dx in (-SERVO_TAB_SPAN / 2, SERVO_TAB_SPAN / 2):
        cx = (bx0 + bx1) / 2 + dx
        part -= Manifold.cylinder(10, 0.8, 0.8, 16).rotate((90, 0, 0)).translate((cx, y_face + 1, az))
    # M4 screws into the top plate (kept clear of the gearbox bolts)
    for sx, sy in servo_bracket_screws():
        part -= cyl(M4_CLEAR / 2, -1, az + W / 2 + 4, sx, sy)
    return part


PARTS = {
    "arm_disc": (arm_disc, "Rigid 10K"),
    "lift_ring": (lift_ring, "Tough 2000"),
    "housing": (housing, "Rigid 10K"),
    "servo_bracket": (servo_bracket, "Tough 2000"),
}
for _w in ("032", "028"):
    PARTS[f"die_tongue_{_w}"] = ((lambda w=_w: die_tongue(w)), "Rigid 10K (steel for daily use)")
    PARTS[f"tube_clamp_{_w}"] = ((lambda w=_w: tube_clamp(w)), "Tough 2000")


def bought_parts(wire="032", pin_up=False, horn_deg=0.0, arm_deg=0.0):
    """Stand-ins for the parts you buy, in place, for the clash check."""
    ax, az = SERVO_AXIS
    L, W, H = SERVO_BODY
    lift = PIN_TRAVEL if pin_up else 0.0
    pin_top = PIN_DOWN_TOP + lift
    hl, ht, hw = SERVO_HORN
    horn = box(0, hl, -ht / 2, ht / 2, -hw / 2, hw / 2)            # along +x from the axis
    horn = horn.rotate((0, -horn_deg, 0)).translate((ax, SERVO_FACE_Y + SERVO_HORN_GAP - ht / 2, az))
    y = die_y(wire)
    wire_z = TONGUE_TOP + WIRE_D[wire] / 2
    return {
        "gearbox shaft": cyl(SHAFT_D / 2, -6.0, SHAFT_TOP),
        "bend pin": cyl(PIN_D / 2, pin_top - PIN_L, pin_top,
                        x=ARM_R * math.cos(math.radians(arm_deg)), y=ARM_R * math.sin(math.radians(arm_deg))),
        "die pin +": cyl(PIN_D / 2, TONGUE_TOP + 3.0 - DIE_PIN_L, TONGUE_TOP + 3.0, 0, y),
        "die pin -": cyl(PIN_D / 2, TONGUE_TOP + 3.0 - DIE_PIN_L, TONGUE_TOP + 3.0, 0, -y),
        "guide rod +": cyl(GUIDE_ROD_D / 2, SLAB_Z0 - 0.5 - 20, SLAB_Z0 - 0.5, *GUIDE_RODS[0]),
        "guide rod -": cyl(GUIDE_ROD_D / 2, SLAB_Z0 - 0.5 - 20, SLAB_Z0 - 0.5, *GUIDE_RODS[1]),
        "servo body": box(ax - (L - SERVO_SHAFT_FROM_END), ax + SERVO_SHAFT_FROM_END,
                          SERVO_FACE_Y - H, SERVO_FACE_Y, az - W / 2, az + W / 2),
        "servo horn": horn,
        "guide tube": Manifold.cylinder(40, TUBE_OD[wire] / 2, TUBE_OD[wire] / 2, 32)
                              .rotate((0, 90, 0)).translate((-40 - (PIN_D / 2 + 0.2), 0, wire_z)),
        "wire": Manifold.cylinder(80, WIRE_D[wire] / 2, WIRE_D[wire] / 2, 32)
                        .rotate((0, 90, 0)).translate((-60, 0, wire_z)),
    }


# Pairs that are meant to touch or nest (fits, not clashes).
EXPECTED = {
    frozenset(p) for p in [
        ("arm_disc", "gearbox shaft"), ("arm_disc", "bend pin"),
        ("die_tongue", "die pin +"), ("die_tongue", "die pin -"),
        ("die_tongue", "guide tube"), ("tube_clamp", "guide tube"),
        ("guide tube", "wire"), ("housing", "guide rod +"), ("housing", "guide rod -"),
        ("lift_ring", "guide rod +"), ("lift_ring", "guide rod -"),
        ("servo_bracket", "servo body"), ("servo body", "servo horn"),
        ("die_tongue", "tube_clamp"),
    ]
}


def clash_check(wire="032"):
    """Every pair of parts, pin down and pin up (ring and horn raised)."""
    problems = []
    for pin_up in (False, True):
        lift = PIN_TRAVEL if pin_up else 0.0
        # horn raised just far enough to lift the ring (tip rises by the gap plus the lift)
        gap = (RING_TOP_DOWN - RING_T) - (SERVO_AXIS[1] + SERVO_HORN[2] / 2)
        horn_deg = math.degrees(math.asin(min(1.0, (lift + gap) / SERVO_HORN[0]))) if pin_up else 0.0
        # the arm turns the pin round with it: the pin is raised beside the
        # wire (here 30 degrees), never in line with it
        arm_deg = 30.0
        disc = arm_disc().rotate((0, 0, arm_deg))
        parts = {"arm_disc": disc, "lift_ring": lift_ring().translate((0, 0, lift)),
                 "housing": housing(), "servo_bracket": servo_bracket(),
                 "die_tongue": die_tongue(wire), "tube_clamp": tube_clamp(wire)}
        parts.update(bought_parts(wire, pin_up, horn_deg, arm_deg))
        names = list(parts)
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                if frozenset((a, b)) in EXPECTED:
                    continue
                v = (parts[a] ^ parts[b]).volume()
                if {a, b} == {"lift_ring", "servo horn"} and pin_up and v < 0.5:
                    continue          # the horn pushing the ring: contact, not a clash
                if v > 0.01:
                    problems.append(f"{'pin up' if pin_up else 'pin down'}: {a} and {b} overlap by {v:.2f} mm^3")
    # Swing the raised pin through the arm's full range.
    house, tongue = housing(), die_tongue(wire)
    for a in np.arange(-110, 111, 5):
        pin = cyl(PIN_D / 2, PIN_DOWN_TOP + PIN_TRAVEL - PIN_L, PIN_DOWN_TOP + PIN_TRAVEL,
                  x=ARM_R * math.cos(math.radians(a)), y=ARM_R * math.sin(math.radians(a)))
        for name, part in (("housing", house), ("die_tongue", tongue)):
            if (pin ^ part).volume() > 0.01:
                problems.append(f"raised pin at {a:+.0f} deg hits the {name}")
    # The raised pin must reach above the wire, the lowered pin stay below it.
    wire_bottom = TONGUE_TOP
    if PIN_DOWN_TOP >= wire_bottom:
        problems.append("lowered pin does not clear the wire")
    if PIN_DOWN_TOP + PIN_TRAVEL < TONGUE_TOP + max(WIRE_D.values()) + 0.3:
        problems.append("raised pin does not reach past the wire")
    return problems


def servo_bracket_screws():
    ax, az = SERVO_AXIS
    L, W, H = SERVO_BODY
    bx0 = ax - (L - SERVO_SHAFT_FROM_END)
    bx1 = ax + SERVO_SHAFT_FROM_END
    return [(bx0 - 3, SERVO_FACE_Y - 5.0), (bx1 + 3, SERVO_FACE_Y - 5.0)]


def drill_template(path):
    """1:1 SVG for the top plate: print at 100%, tape down, centre-punch."""
    x0, x1, y0, y1 = PLATE
    W, H = x1 - x0, y1 - y0
    holes = [((0.0, 0.0), GEARBOX_PILOT_D, "gearbox boss (VERIFY)")]
    g = GEARBOX_BOLTS / 2
    holes += [((sx * g, sy * g), GEARBOX_BOLT_D, "gearbox M5 (VERIFY)")
              for sx in (1, -1) for sy in (1, -1)]
    holes += [(xy, 3.3, "housing M4 tap") for xy in HOUSING_SCREWS]
    holes += [(xy, 3.3, "servo bracket M4 tap") for xy in servo_bracket_screws()]
    for i, ((xa, ya), da, la) in enumerate(holes):
        for (xb, yb), db, lb in holes[i + 1:]:
            gap = math.hypot(xa - xb, ya - yb) - (da + db) / 2
            if gap < 3.0 and not (la.startswith("gearbox boss") or lb.startswith("gearbox boss")):
                raise SystemExit(f"drill template: {la} and {lb} are only {gap:.1f} mm apart")
    fx = lambda x: x - x0
    fy = lambda y: y1 - y                      # +y up on paper
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}mm" height="{H + 20}mm" '
           f'viewBox="0 0 {W} {H + 20}" font-family="sans-serif">',
           '<rect width="100%" height="100%" fill="#fff"/>',
           f'<rect x="0" y="0" width="{W}" height="{H}" fill="none" stroke="#000" stroke-width="0.3"/>']
    for (x, y), d, label in holes:
        cx, cy = fx(x), fy(y)
        out.append(f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="{d / 2:.2f}" fill="none" stroke="#000" stroke-width="0.25"/>')
        out.append(f'<path d="M{cx - 1.5:.2f},{cy:.2f}h3M{cx:.2f},{cy - 1.5:.2f}v3" stroke="#000" stroke-width="0.2"/>')
        out.append(f'<text x="{cx + d / 2 + 1:.2f}" y="{cy - 1:.2f}" font-size="2.2">{label}</text>')
    # wire line and feed direction
    out.append(f'<path d="M{fx(x0 + 5):.1f},{fy(0):.1f}H{fx(x1 - 5):.1f}" stroke="#c0392b" stroke-width="0.3" stroke-dasharray="2 1"/>')
    out.append(f'<text x="{fx(x0 + 5):.1f}" y="{fy(0) - 1.5:.1f}" font-size="2.5" fill="#c0392b">wire line, feed '
               'direction left to right: the feed stage mounts here, behind the head (holes once the stage is measured)</text>')
    out.append(f'<text x="{fx(8):.1f}" y="{fy(-12):.1f}" font-size="2.5" fill="#c0392b">bend head</text>')
    out.append(f'<path d="M5,{H + 8}h50" stroke="#000" stroke-width="0.4"/>'
               f'<text x="58" y="{H + 9}" font-size="3">50 mm: check this before drilling. '
               'Top plate 300 x 150 x 6 mm aluminium, seen from above.</text>')
    out.append("</svg>")
    Path(path).write_text("\n".join(out))


def to_tris(m):
    mesh = m.to_mesh()
    v = np.asarray(mesh.vert_properties)[:, :3]
    t = np.asarray(mesh.tri_verts)
    return v[t]


def main(out=None):
    out = Path(out) if out else Path(__file__).resolve().parent / "stl"
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    for name, (fn, resin) in PARTS.items():
        m = fn()
        if m.is_empty():
            raise SystemExit(f"{name}: empty solid")
        lo, hi = m.bounding_box()[:3], m.bounding_box()[3:]
        size = np.array(hi) - np.array(lo)
        save_stl(out / f"{name}.stl", to_tris(m))
        fits = sorted(size)[-1] <= 210 and sorted(size)[-2] <= 200 and sorted(size)[0] <= 125
        rows.append((name, resin, size, m.volume() / 1000, fits))
    for name, resin, size, vol, fits in rows:
        print(f"{name:16s} {size[0]:6.1f} x {size[1]:6.1f} x {size[2]:5.1f} mm  "
              f"{vol:5.1f} mL  {resin}{'' if fits else '  ** too big for Form 4 **'}")
    drill_template(out.parent / "top_plate_drill_template.svg")
    for w in ("032", "028"):
        problems = clash_check(w)
        print(f"clash check ({w}): " + ("none" if not problems else ""))
        for p in problems:
            print("  " + p)
    return rows


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else None)
