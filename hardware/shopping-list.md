# Shopping list (version 1)

Links were checked in November 2026; listings change, so confirm specs and
stock before ordering. Prices are rough US prices. Buy the **exact** stage, gearbox and hub you'll use, then
send their dimensions (or listing pages) so the printed parts can be designed
to fit them.

## Motion

| Qty | Item | Search for / spec | ~$ |
|---|---|---|---|
| 1 | Linear stage with NEMA 17 | [SainSmart 100 mm stage, SFU1605 ball screw, NEMA 17](https://www.sainsmart.com/products/cnc-linear-stage-actuator-with-nema17-stepper-motor) (check the listing says 100 mm travel). Any 100–150 mm ball-screw or lead-screw stage with a NEMA 17 works. **Set `FEED_STEPS_PER_MM` in the firmware to the screw's lead**: 640 for a 5 mm lead (1605), 800 for 4 mm (1204), 1600 for 2 mm (T8×2). | 60–110 |
| 1 | NEMA 23 stepper + 10:1 planetary gearbox | [StepperOnline 23HS30-2904S-MG10](https://www.omc-stepperonline.com/nema-23-stepper-motor-l-76mm-gear-ratio-10-1-mg-series-planetary-gearbox-23hs30-2904s-mg10), motor and 10:1 gearbox together. 30 arc-min backlash is fine: every bend approaches the wire in the direction it bends, so the slack is taken up before the pin touches. | 46 |
| 1 | Hub for the gearbox shaft | Rigid flange coupling or clamping shaft collar to fit the gearbox's output shaft. Buy after the gearbox arrives and you've measured the shaft: [McMaster shaft collars](https://www.mcmaster.com/shaft-collars) | 10–20 |
| 1 | Lift servo | MG90S metal-gear micro servo, e.g. [Adafruit 1143 via Little Bird](https://littlebirdelectronics.com.au/products/micro-servo-mg90s-high-torque-metal-gear) or [Pro Supplies](https://protosupplies.com/product/servo-motor-micro-mg90s/) | 8–20 |
| 2 | Limit switches | Micro switch with roller lever (stage rear and arm home): [McMaster snap-acting switches](https://www.mcmaster.com/snap-acting-switches) | 5 |

## Wire path (hardened steel where it touches the wire)

| Qty | Item | Search for / spec | ~$ |
|---|---|---|---|
| 1 pack | Hardened dowel pins, 3 mm × 20 mm, HRC 52–62, precision ground | [Zoro M3×20 hardened dowel pins (100)](https://zoro.com/zoro-select-dowel-pin-std-m3x20mm-l-pk100-m390800300020/i/G3032985) or [Metric & Multistandard 3×20 hardened](https://www.metricmcc.com/6325-hardened-steel-hard-dowel-pin-3x20-p-18467.html). Two for the die, one bend pin, spares. | 10–20 |
| 1 | Bronze flange bushing, 3 mm bore | Bend pin slides in it. McMaster lists a 932 bronze flanged sleeve bearing for a 3 mm shaft in a 6 mm housing: filter [McMaster flanged sleeve bearings](https://www.mcmaster.com/bearings/length~3-mm/) by 3 mm shaft. | 5 |
| 1 | Guide tube for .032" wire | [McMaster 304 stainless tubing](https://www.mcmaster.com/hypodermic-tubing) part **5560K49**: 0.042" OD, 0.035" ID (0.89 mm) | 10–20 |
| 1 | Guide tube for .028" (and .026") wire | [McMaster 304 stainless tubing](https://www.mcmaster.com/needle-tubing) part **8988K43**: 0.050" OD, 0.030" ID (0.76 mm) | 10–20 |
| 1 | Jeweller's pin vise, double-ended | Clamps the back of the wire on the stage: [Esslinger 0–1.0 mm / 1.8–2.5 mm](https://www.esslinger.com/double-ended-pin-vise-and-collet-tool-0-to-1-0mm-and-1-8-to-2-5mm/) | 8–15 |
| 1 | Small steel or aluminium plate for the die, ~25 × 25 × 6 mm | Two 3 mm holes, drilled to the wire's diameter apart | 5 (or a machine shop) |

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
| 1 | Base plate | 10 mm aluminium ~300 × 150 mm, or 18 mm plywood with a steel plate under the die and arm | 20–40 |
| — | M3/M4/M5 screws, nuts, heat-set inserts | Assortment kits | 20 |
| — | Filament | PETG or PA-CF | 25 |

## Wire for bending

Straight lengths (not spools) of round stainless, in .028" and .032". Also
.026" if you want to compare. Get them from your usual orthodontic supplier.
Each 14" length gives three 110–120 mm blanks.

## Tools you'll want

Digital calipers, a protractor (or a flatbed scanner for measuring bent test
wires), a drill press (for the die plate), a soldering iron, and hex keys.

**Total: about $400–650**, plus the tools if you don't have them.
