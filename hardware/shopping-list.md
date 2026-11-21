# Shopping list

Everything needed for the 2D machine. It bends the flat wire; you fold the
two canine elbows by hand at the marked points. The roll motor for true 3D
bending is not needed.

Links were checked in November 2026; listings change, so confirm specs and
stock before ordering. Prices are rough US prices. Buy the **exact** stage, gearbox and hub you'll use, then
send their dimensions (or listing pages) so the printed parts can be designed
to fit them.

## Motion

| Qty | Item | Search for / spec | ~$ |
|---|---|---|---|
| 1 | Linear stage with NEMA 17 | [SainSmart 100 mm stage, SFU1605 ball screw, NEMA 17](https://www.sainsmart.com/products/cnc-linear-stage-actuator-with-nema17-stepper-motor) (check the listing says 100 mm travel). Any 100–150 mm ball-screw or lead-screw stage with a NEMA 17 works. **Set `FEED_STEPS_PER_MM` in the firmware to the screw's lead**: 640 for a 5 mm lead (1605), 800 for 4 mm (1204), 1600 for 2 mm (T8×2). | 60–110 |
| 1 | NEMA 23 stepper + 10:1 planetary gearbox | [StepperOnline 23HS30-2904S-MG10](https://www.omc-stepperonline.com/nema-23-stepper-motor-l-76mm-gear-ratio-10-1-mg-series-planetary-gearbox-23hs30-2904s-mg10), motor and 10:1 gearbox together. 30 arc-min backlash is fine: every bend approaches the wire in the direction it bends, so the slack is taken up before the pin touches. | 46 |
| 1 | Lift servo | MG90S metal-gear micro servo, e.g. [Adafruit 1143 via Little Bird](https://littlebirdelectronics.com.au/products/micro-servo-mg90s-high-torque-metal-gear) or [Pro Supplies](https://protosupplies.com/product/servo-motor-micro-mg90s/) | 8–20 |
| 2 | Limit switches | Micro switch with roller lever (stage rear and arm home): [McMaster snap-acting switches](https://www.mcmaster.com/snap-acting-switches) | 5 |

## Wire path (hardened steel where it touches the wire)

| Qty | Item | Search for / spec | ~$ |
|---|---|---|---|
| 1 | Hardened dowel pin, 3 × 26 mm | Bend pin. Hardened (HRC 52–62), precision ground, e.g. from [Metric & Multistandard](https://www.metricmcc.com/) or McMaster's dowel pins filtered to 3 mm | 3–10 |
| 2 | Hardened dowel pins, 3 × 8 mm | Die pins (pressed into the die tongue) | 3–10 |
| 2 | Hardened dowel pins, 3 × 24 mm | Lift-ring guide rods | 3–10 |
| 1 | Guide tube for .032" wire | [McMaster 304 stainless tubing](https://www.mcmaster.com/hypodermic-tubing) part **5560K49**: 0.042" OD, 0.035" ID (0.89 mm) | 10–20 |
| 1 | Guide tube for .028" (and .026") wire | [McMaster 304 stainless tubing](https://www.mcmaster.com/needle-tubing) part **8988K43**: 0.050" OD, 0.030" ID (0.76 mm) | 10–20 |
| 1 | Jeweller's pin vise, double-ended | Clamps the back of the wire on the stage: [Esslinger 0–1.0 mm / 1.8–2.5 mm](https://www.esslinger.com/double-ended-pin-vise-and-collet-tool-0-to-1-0mm-and-1-8-to-2-5mm/) | 8–15 |
| later | Steel die tongue | The printed one is fine for first tests; for daily use have a machine shop make `die_tongue_032.stl` in steel | 50–150 |

## Electronics

| Qty | Item | Spec | ~$ |
|---|---|---|---|
| 1 | Controller | [Teensy 4.1 from PJRC](https://pjrc.com/store/teensy41.html) (the version without Ethernet is enough), also at [Adafruit](https://www.adafruit.com/product/4622) | 34 |
| 1 | Feed motor driver | [BIGTREETECH TMC2209 V1.3](https://us.biqu.equipment/products/bigtreetech-tmc2209-stepper-motor-driver-for-3d-printer-board-vs-tmc2208) | 5–10 |
| 1 | Arm motor driver | [StepperOnline DM542T](https://www.omc-stepperonline.com/digital-stepper-driver-1-0-4-2a-20-50vdc-for-nema-17-23-24-stepper-motor-dm542t) | 30 |
| 1 | 24 V power supply | [Mean Well LRS-150-24](https://www.trcelectronics.com/products/mean-well-lrs-150-24) (24 V 6.5 A), also at [Digi-Key](https://azcus.digikey.com/en/products/detail/mean-well-usa-inc/LRS-150-24/7705015) | 20 |
| 1 | 24 V → 5 V regulator for the servo | [Pololu D24V22F5](https://www.pololu.com/product-info-merged/2858) (5 V 2.5 A) | 11 |
| 1 | Emergency stop, normally closed, mushroom head | Wired to cut motor power *and* signal the controller: [McMaster emergency stop switches](https://www.mcmaster.com/emergency-stop-switches) | 15–30 |
| — | Wire, terminals, USB cable, small enclosure | | 20 |

## Frame

| Qty | Item | Spec | ~$ |
|---|---|---|---|
| 1 | Top plate | 6 mm aluminium, 300 × 150 mm (6061 plate from any metal supplier); drilled to `cad/top_plate_drill_template.svg` | 20–40 |
| 4 | Legs | M5 aluminium hex standoffs about 150 mm long (or threaded rod with nuts), so the motor hangs clear below the plate | 10–20 |
| — | Screws | M3 × 16 + nuts (4, die tongue and clamp), M4 × 10 (8, housing and servo bracket), M4 set screws + nuts (2, arm disc), M5 × 12 (4, gearbox), M2 self-tapping (2, servo). An M3/M4/M5 assortment covers it. Resin parts use captured nuts, not heat-set inserts. | 20 |
| 1 | Formlabs Rigid 10K resin | Stiff parts near the bend head | ~ 1 cartridge |
| 1 | Formlabs Tough 2000 (or Tough 1500) resin | Clamps, mounts, enclosure | ~ 1 cartridge |

## Wire for bending

Straight lengths (not spools) of round stainless, in .028" and .032". Also
.026" if you want to compare. Get them from your usual orthodontic supplier.
Each 14" length gives three 110–120 mm blanks.

## Tools you'll want

Digital calipers, a 3.0 mm hand reamer (for the bend pin's hole in the arm
disc), M4/M5 taps, a drill press, a protractor or flatbed scanner for
measuring bent test wires, a soldering iron, and hex keys. For finishing the
wires, the pliers you already use to bend wire.

**Total: about $400–650**, plus the tools if you don't have them.

## Build order

1. **Order** everything above, plus the wire.
2. **Print** the bend-head parts in `cad/stl/` while it ships: housing and
   arm disc in Rigid 10K; lift ring, servo bracket and tube clamp in
   Tough 2000; one die tongue per wire size. Wash and fully post-cure.
3. **Measure** when the parts arrive and send the numbers back:
   - the gearbox's output shaft height above its face, and its bolt pattern
   - the servo body
   - the linear stage's carriage hole pattern and footprint

   The CAD gets updated and re-checked, then the stage mount, pin-vise
   carriage and enclosure are drawn.
4. **Drill** the top plate from the template; mount the gearbox, legs and
   head.
5. **Wire up and flash** the firmware; run the bring-up checks in
   `README.md` (feed distance, arm zero, pin lift, bend direction).
6. **Calibrate** springback for each wire size.
7. **First wires** on printed models: check each in the jig and on the
   model before any patient use.
