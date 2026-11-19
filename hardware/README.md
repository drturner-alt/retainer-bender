# Hardware: 2D bender for .028" / .032" stainless retainer wire

Version 1 makes flat lower 3-3 retainers from **straight lengths** of wire,
loaded by hand and cut by hand. Two motors: one pushes the wire out, one turns
the bend arm. Everything that carries bending load is steel; everything else
can be printed.

```
   feed stage (lead screw)            guide tube        die    bend arm (under the deck,
   ┌──────────────────────────┐   ════════════════════╗ ◉ ◉   turns about the die exit)
   │ pin vise ▶ clamps wire   │ ─────── wire ─────────╫──●──────────▶ free end
   └──────────────────────────┘   ════════════════════╝ ◉ ◉        ● = pins on the arm
          NEMA 17                                       die pins
```

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

## 3. Bend arm

| Part | Spec | Approx. |
|---|---|---|
| NEMA 23 stepper + 10:1 planetary gearbox | ≥ 1 N·m continuous at the output, low backlash (≤ 15 arcmin) | $90–150 |
| Arm hub | Steel/aluminium flange coupling or shaft collar to fit the gearbox shaft | $10 |
| Arm pins | Two hardened 3 mm dowel pins | $5 |
| Home switch | Micro switch or optical endstop, triggered by a flag on the hub | $2 |

- The gearbox shaft is vertical and sits **directly under the die exit**.
- The two pins stand up from the hub at radius **6 mm** from the shaft, at
  **±60°** from the direction the wire travels, one on each side of the wire.
- The pins stick up through a curved slot in the deck, just high enough to
  catch the wire.
- Turning the arm brings one pin against the wire and bends the free end
  around the die pin on that side.

These numbers are in `config/machine.json` (`arm_radius_mm`,
`pin_angle_deg`, `pin_diameter_mm`). From them, the software works out where
the pins touch the wire, and it checks every program so the bent wire never
runs into a pin or back into the guide.

**Keep the guide slim behind the die.** As the wire curls into a U, the
non-pushing pin swings round to about 100°, which is behind the die exit. The
tube and die plate must fit within ±3 mm of the wire line for the first 6 mm
behind the exit (`body_halfwidth_mm`).

## 4. Frame and printed parts

- Base: 10 mm aluminium plate, or 18 mm plywood with a steel plate under
  the die/arm area. The die and the arm motor must share one stiff plate.
- Print brackets, the pin-vise mount, the tube clamps, the deck and
  covers in PETG or PA-CF, with 5+ walls and 40%+ infill.
- **Don't print** the die, the arm pins, or anything between the die and the
  arm motor that carries load.

## 5. Electronics

| Part | Spec | Approx. |
|---|---|---|
| Controller | Teensy 4.1 (or Arduino Mega 2560) | $30 |
| Feed driver | TMC2209 or DRV8825 stepstick, 16 microsteps | $8 |
| Arm driver | DM542 / DM556 external driver, 16 microsteps | $25 |
| Power | 24 V, 5 A supply | $20 |
| E-stop | Red mushroom button (normally closed) that cuts the motor supply *and* signals the controller | $10 |

The pins are set at the top of `firmware/retainer_bender/retainer_bender.ino`.
So are steps/mm, steps/degree, direction inversion, and the offset from the
arm's home switch to neutral.

Rough total: **$300–550** plus the die plate.

## 6. Bring-up

1. Flash the firmware, open a serial monitor at 115200 baud, and send `HOME`
   and `STATUS`.
2. **Check the feed.** Send `FEED 50` and measure how far the carriage moved
   with calipers. Correct `FEED_STEPS_PER_MM`.
3. **Set neutral.** Turn the arm to neutral (pins symmetric about the wire),
   then set `ARM_HOME_TO_NEUTRAL_DEG` so that `HOME` ends there.
4. **Check the bend direction.** With a test wire, `ARM 50` should bend the free
   end counter-clockwise seen from above. If it bends the other way, flip
   `ARM_INVERT`.
5. **Calibrate springback** (below) for each wire size.

## 7. Springback calibration

```
rbend calibrate --wire 032 -o calib032
rbend send calib032/calibration_program.txt --port /dev/ttyACM0
```

The calibration program bends one test wire: 8 mm straight pieces joined by
bends of 4–30° commanded, alternating direction. Lay the wire flat and measure
each angle, either with a protractor or by scanning it on a flatbed and
measuring in any image tool. Enter the angles in
`calib032/calibration_sheet.csv`, then run:

```
rbend fit-springback calib032/calibration_sheet.csv --wire 032 -o config/springback_ss_032.json
```

Repeat for every new wire lot, and whenever you change the pins or die.

## Safety

The arm makes about 1 N·m and the pins pinch. Keep fingers off the deck
whenever the machine is powered. Use `OFF` before reaching in to load wire,
and wire the E-stop so it cuts motor power, not just the signal.
