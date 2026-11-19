# retainer-bender

Plans and bends flat (2D) lower 3-3 lingual retainer wires in **.028"** or
**.032"** stainless steel from an iTero scan. The wire is bonded to the canines
and arcs across to touch each incisor about two-thirds of the way up its
lingual surface.

```
iTero lower STL ──► picker (browser) ──► picks.json ──► rbend plan ──► program.txt ──► rbend send ──► bender
                     click the contacts                  │
                                                         ├── report.txt         bend list, clearances, checks
                                                         ├── preview.svg        occlusal view over the teeth
                                                         ├── template_1to1.svg  print at 100% and lay the wire on it
                                                         └── check_jig.stl      print it; the finished wire should drop into the groove
```

Hardware build and bill of materials: [hardware/README.md](hardware/README.md).
Firmware: [firmware/retainer_bender](firmware/retainer_bender/retainer_bender.ino).

## Install

```bash
python -m venv .venv && source .venv/bin/activate
pip install -e ".[machine,test]"
```

The planner needs only numpy. `pyserial` is needed only to talk to the machine.

## Try it without a scan

```bash
rbend demo -o demo
```

This builds a made-up lower anterior segment, with tooth 41 set 0.3 mm lingual,
and plans a .032" wire for it. Open `demo/preview.svg` and read
`demo/report.txt`.

## Make a wire

1. **Export** the lower scan from iTero as an STL.
2. **Pick the contacts.** Open `picker/picker.html` in Chrome, load the STL,
   and click, in order:
   - 43: the distal end of the bond area, then the mesial end
   - 42, 41, 31, 32: where the wire should touch, about two-thirds up the lingual
   - 33: the mesial end of the bond area, then the distal end

   Click **Download picks.json**.
3. **Plan:**
   ```bash
   rbend plan picks.json --scan lower.stl --wire 032 -o case123
   ```
   Read `case123/report.txt` and look at `case123/preview.svg`. The report
   gives the gap to each tooth, the closest approach along the wire, and
   whether the machine can make every bend.
4. **Bend:**
   ```bash
   rbend send case123/program.txt --port /dev/ttyACM0
   ```
   Load the wire tip flush with the die before sending. Cut at the die
   when it finishes.
5. **Check the result.** Lay the wire on the printed `template_1to1.svg`,
   drop it into the printed `check_jig.stl`, and try it on the printed model.

## How the planner works

- Each pick is moved out along the tooth's surface normal by the wire radius,
  which gives the wire's centre line at that contact.
- A plane is fitted through those centres, and the centres are flattened onto
  it. The report says how far each contact had to move: a flat wire can't reach
  one that sits well above or below the rest.
- A smooth curve is run through the centres. Wherever it would cut into a tooth
  (across a convex canine, for example), it is pushed out just clear.
- The curve is replaced by straight pieces at least 1 mm long that stay within
  0.04 mm of it. That gives a feed length for each piece and a bend angle at
  each joint, usually about 12–16 bends of 3–15°.
- Each bend angle is converted to an arm angle. The conversion adds the angle
  at which a pin first touches the wire, and an over-bend from that wire's
  springback table.
- The program is then simulated on the machine: feed travel, arm reach, and
  whether the bent wire would hit a pin or the guide.

`--style polyline` makes straight runs between picks instead of a smooth curve.
`--contact-offset 0.05` stands the wire 0.05 mm off the teeth, and a negative
value presses it in.

## Before treating patients

- The springback tables in `config/` are **placeholders**. Run the calibration
  in hardware/README.md for each wire size and lot, and redo it whenever pins
  or the die change. The report says loudly when a placeholder is in use.
- Check every wire on a printed model before bonding.
- Patient scans and picks are git-ignored on purpose. Keep them out of this
  repository.

## Commands

| Command | What it does |
|---|---|
| `rbend plan PICKS --scan STL --wire 028/032 -o DIR` | plan a wire |
| `rbend demo -o DIR` | plan for the synthetic arch |
| `rbend calibrate --wire 032 -o DIR` | write the springback test program and a measuring sheet |
| `rbend fit-springback SHEET.csv --wire 032 -o config/springback_ss_032.json` | turn measurements into a table |
| `rbend send PROGRAM --port PORT [--dry-run]` | stream a program to the machine |

## Tests

```bash
pytest
```
