# Hardware: 2D bender for .028" / .032" stainless retainer wire

Version 1 makes flat lower 3-3 retainers, with a retention curl at each canine
end, from **straight lengths** of wire loaded by hand and cut by hand. Two
motors do the work: one pushes the wire out, and one turns the bend arm. A
hobby servo raises and lowers the bend pin. Everything that carries bending
load is steel; everything else can be printed.

```
   feed stage (lead screw)            guide tube       die     bend arm turns about the die exit;
   ┌──────────────────────────┐   ════════════════════╗ ◉      its one pin drops below the deck
   │ pin vise ▶ clamps wire   │ ─────── wire ─────────╫────────────▶ free end
   └──────────────────────────┘   ════════════════════╝ ◉   ●  ← bend pin, raised only to bend
          NEMA 17                                     die pins
```

Why one pin that drops, and not two fixed pins either side of the wire: the end
curls need several 20–30° bends 1 mm apart. That folds the wire right back
beside the die, where a fixed second pin would sit. Spread the fixed pins wide
enough to miss the curl and the far pin swings round into the guide. With a
dropping pin, nothing is near the wire while it feeds. The software finds where
the raised pin will first touch the (possibly curled) wire on each bend, and
checks that it can be raised clear.

## Why these sizes

Bending torque grows with the cube of wire diameter. Fully bending spring-hard
.032" SS takes roughly 0.18 N·m, and .028" about 0.12 N·m. For .018" it's about
0.03 N·m, which is why the Bender II stops there. With friction, work hardening
and a safety margin, size the arm drive for about **1 N·m** and the die for
**about 150 N** of side load at the exit.

## 1. Feed: a push stage, not rollers

A 3-3 needs about 30–45 mm of wire, so one stroke of a short linear stage
feeds the whole retainer. A clamp on the carriage holds the back of the wire
and pushes it out through the guide. Nothing can slip, so feed length is only
as wrong as the lead screw (a few hundredths of a mm).

| Part | Spec | Approx. |
|---|---|---|
| Linear stage | 100–150 mm travel, T8×2 lead screw or SFU1204 ball screw, NEMA 17 | $40–90 |
| NEMA 17 stepper | 0.4–0.5 N·m (usually on the stage) | — |
| Pin vise | Jeweller's double-ended pin vise, 0–1.0 mm collet | $8 |
| Rear limit switch | Micro switch with lever | $2 |

Mount the pin vise on a printed bracket on the carriage, in line with the
guide tube. Load a wire: open the vise, push the wire through until the tip is
flush with the die exit, then tighten the vise.

## 2. Guide and die

The wire runs through a stainless **hypodermic tube** from the carriage to the
die. The tube stops the wire from buckling while it's pushed. Choose an inner
diameter 0.03–0.08 mm bigger than the wire. Keep one tube per wire size.

| Wire | Diameter | Tube ID to look for |
|---|---|---|
| .028" | 0.711 mm | 0.74–0.79 mm |
| .032" | 0.813 mm | 0.84–0.89 mm |

The **die** is where the wire leaves the guide. All the bending reaction is
taken here. Make it from two hardened 3 mm dowel pins (h6 ground), standing
side by side with a gap equal to the wire diameter plus about 0.03 mm. The wire
passes between them, and each bend wraps the wire around one of them. Press
both pins into a small steel or aluminium plate. Drill the holes on a drill
press with a printed drilling jig, or have a machine shop do this one part.
Tie that plate rigidly to the same base plate as the arm's motor.

## 3. Bend arm and lifting pin

| Part | Spec | Approx. |
|---|---|---|
| NEMA 23 stepper + 10:1 planetary gearbox | ≥ 1 N·m continuous at the output, backlash up to 30 arcmin is fine (each bend approaches in its own direction, which takes up the slack) | $90–150 |
| Arm | Steel or aluminium disc on the gearbox shaft, with a bronze or steel bushing for the pin | $10–20 |
| Bend pin | Hardened 3 mm dowel pin, about 20 mm long, sliding in the bushing | $3 |
| Lift ring | A printed (PTFE-faced) or aluminium ring around the shaft, under the arm, that does not turn | — |
| Lift servo | Metal-gear micro servo (MG90S or similar), fixed to the frame, lifting the ring through a short lever | $8 |
| Home switch | Micro switch or optical endstop, triggered by a flag on the arm | $2 |

How it fits together:
- The gearbox shaft is vertical and sits **directly under the die exit**.
- The pin slides vertically in a bushing in the arm, **10 mm** from the shaft
  centre. That clears the 14 mm gearbox shaft, so the pin can pass down beside
  it to the lift ring, and it clears the die nose for arm angles up to about
  110°.
- Its lower end rests on the lift ring. The ring doesn't rotate, so the pin
  slides around on it as the arm turns, and no wires have to turn with the arm.
- Servo up: the ring lifts and the pin rises about 1.5 mm above the deck,
  through a slot that runs around the die. Servo down: the pin drops below the
  deck and the arm can swing anywhere under the wire.
- The bending load goes sideways into the bushing and the arm. Neither the
  ring nor the servo carries it.

**Bend sequence**, done automatically:
1. Swing the arm (pin down) to the near side of the wire.
2. Pin up.
3. Turn until the pin touches the wire, then on by the commanded angle.
4. Pin down.

Because the pin is 10 mm out, at least 10 mm of wire has to be out of the die
before the first bend. The program feeds an 11 mm straight lead-in (set by
`lead_in_mm`); snip it off the first end after bending.

These numbers are in `config/machine.json` (`arm_radius_mm`,
`pin_diameter_mm`, `lead_in_mm`). The software uses them to simulate every
program, so a program that can't be made is refused before it reaches the
machine.

**Keep the guide slim behind the die.** The free end curls round as the wire
is bent. Keep the die nose and the tube clamp within ±3.5 mm of the wire line
(`die_halfwidth_mm`, `body_halfwidth_mm`). The software checks every bend for
the pin against the die nose and the free end against the clamp.

## 4. Frame and printed parts

- Base: 10 mm aluminium plate, or 18 mm plywood with a steel plate under
  the die/arm area. The die and the arm motor must share one stiff plate.
- The printed parts are designed for a resin printer (Formlabs Form 4/4B,
  build volume about 200 × 125 × 210 mm). Nothing is longer than that, and the
  enclosure is split into panels that fit.
- Resin: **Rigid 10K** for anything stiff near the bend head (deck, die block
  holder, tube clamps, servo bracket); **Tough 2000** (or Tough 1500) for
  parts that flex or take screws repeatedly (carriage mount, switch mounts,
  enclosure). Dental model resin is fine for the check jigs, not for the
  machine.
- Resin parts don't take heat-set inserts. Screws go into captured hex-nut
  pockets or through-holes with nuts behind; small parts can use
  self-tapping plastic screws into undersized holes.
- Wash and fully post-cure (Form Cure) before assembly. Uncured resin parts
  creep under a clamping screw.
- **Don't print** the die, the bend pin, the arm, or anything between the die and the
  arm motor that carries load.

## 5. Electronics

| Part | Spec | Approx. |
|---|---|---|
| Controller | Teensy 4.1 (or Arduino Mega 2560) | $30 |
| Feed driver | TMC2209 or DRV8825 stepstick, 16 microsteps | $8 |
| Arm driver | DM542 / DM556 external driver, 16 microsteps | $25 |
| Power | 24 V, 5 A supply | $20 |
| E-stop | Red mushroom button (normally closed) that cuts the motor supply *and* signals the controller | $10 |
| 5 V supply for the servo | 5 V 2 A buck converter from the 24 V supply | $5 |

Controller pin numbers are set at the top of `firmware/retainer_bender/retainer_bender.ino`.
So are steps/mm, steps/degree, direction inversion, and the offset from the
arm's home switch to neutral.

Rough total: **$320–580** plus the die plate and arm.

## 6. Bring-up

1. Flash the firmware, open a serial monitor at 115200 baud, and send `HOME`
   and `STATUS`.
2. **Check the feed.** Send `FEED 50` and measure how far the carriage moved
   with calipers. Correct `FEED_STEPS_PER_MM`.
3. **Set zero.** With the pin down, `HOME` should leave the pin pointing
   straight down the wire line (0°). Adjust `ARM_HOME_TO_NEUTRAL_DEG` until it
   does.
4. **Set the lift.** Adjust `LIFT_DOWN_DEG` and `LIFT_UP_DEG` until `PIN DOWN`
   puts the pin top 0.5 mm below the deck and `PIN UP` raises it 1.5 mm above.
5. **Check the bend direction.** Feed 10 mm of test wire, then send `ARM -40`,
   `PIN UP`, `ARM 10`. The pin should come up from below the wire (seen from
   above, with the wire travelling left to right) and bend the end
   counter-clockwise. If it bends the other way, flip `ARM_INVERT`.
6. **Calibrate springback** (below) for each wire size.

## 7. Springback calibration

```
rbend calibrate --wire 032 -o calib032
rbend send calib032/calibration_program.txt --port /dev/ttyACM0
```

The calibration program bends one test wire: 12 mm straight pieces joined by
bends of 4–30° commanded, alternating direction. Lay the wire flat and measure
each angle, either with a protractor or by scanning it on a flatbed and
measuring in any image tool. Enter the angles in
`calib032/calibration_sheet.csv`, then run:

```
rbend fit-springback calib032/calibration_sheet.csv --wire 032 -o config/springback_ss_032.json
```

Repeat for every new wire lot, and whenever you change the pins or die.

## Safety

The arm makes about 1 N·m and the pin pinches. Keep fingers off the deck
whenever the machine is powered. Send `PIN DOWN` then `OFF` before reaching in to load wire,
and wire the E-stop so it cuts motor power, not just the signal.
