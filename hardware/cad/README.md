# Printed parts (version 0)

`parts.py` builds every printed part from named dimensions, checks that none
of them collide with each other or with the bought parts, and writes:

- `stl/*.stl`: ready for PreForm. Every part is modelled in its assembled
  position, so lay each one flat in PreForm before printing.
- `top_plate_drill_template.svg`: print it at 100%, check the 50 mm scale
  bar, tape it to the aluminium top plate and centre-punch the holes.

```bash
pip install manifold3d
python hardware/cad/parts.py
```

## Before printing: measure and update

Numbers marked `VERIFY` in `parts.py` came from listings, not from parts in
hand. Measure them with calipers when the parts arrive, change them, and
re-run the script. It re-checks for collisions every time.

| What | Default | Where it matters |
|---|---|---|
| Gearbox shaft diameter and key | 14 mm, 5 mm key | arm disc bore |
| Shaft tip height above the top plate | 28 mm | arm disc must clamp the top 10 mm |
| Gearbox face: boss diameter and bolt pattern | 40 mm, 47.14 mm square, M5 | drill template |
| MG90S body, shaft position, tab spacing, horn length | see `SERVO_*` | servo bracket, lift |

## Parts

| File | Resin | Notes |
|---|---|---|
| `housing.stl` | Rigid 10K | The deck. Arc slot for the bend pin; nut pockets underneath for the tongue screws; holes for the guide rods |
| `arm_disc.stl` | Rigid 10K | Keyed onto the shaft, two M4 set screws with captured nuts. Ream the pin bore to 3.0 mm |
| `lift_ring.stl` | Tough 2000 | Slides on two 3 × 20 dowels; the servo horn lifts its rear tab |
| `servo_bracket.stl` | Tough 2000 | MG90S on its side; two M4 screws into the top plate |
| `die_tongue_032.stl` / `_028.stl` | Rigid 10K | One per wire size. Holds the two 3 × 8 die pins and the guide tube. Fine for first tests; have it made in steel for daily use |
| `tube_clamp_032.stl` / `_028.stl` | Tough 2000 | Clamps the guide tube; shares the tongue's four M3 screws |

Wash, then fully post-cure in Form Cure before assembly.

## Extra hardware for these parts

- Hardened dowel pins: one 3 × 24 (bend pin), two 3 × 8 (die pins), two 3 × 20 (guide rods)
- M3 × 16 screws and nuts (4), M4 × 10 screws (6), M4 set screws and nuts (2)
- 3.0 mm hand reamer, for the arm disc's pin bore

## Not drawn yet

- Feed stage mount and pin-vise carriage: wait for the stage, then measure its
  hole pattern.
- Enclosure panels, and the arm home-switch mount.
