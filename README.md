# retainer-bender

Plans and bends flat (2D) lower 3-3 lingual retainer wires in **.028"** or
**.032"** stainless steel from an iTero scan. The wire is bonded to the canines,
with a retention curl at each end, and arcs across to touch each incisor about
two-thirds of the way up its lingual surface.

```
iTero lower STL ──► picker (browser) ──► picks.json ──► rbend plan ──► program.txt ──► rbend send ──► bender
                     3 clicks: 43, midline, 33           │
                                                         ├── report.txt         bend list, contacts, occlusion, checks
                                                         ├── preview.svg        occlusal view over the teeth
                                                         ├── wire.stl           the wire in scan coordinates (view it, or
                                                         │                      add it to the model under an overlay retainer)
                                                         ├── template_1to1.svg  print at 100% and lay the wire on it
                                                         └── check_jig.stl      print it; the finished wire should drop into the groove
```

Hardware build and bill of materials: [hardware/README.md](hardware/README.md); what to order: [hardware/shopping-list.md](hardware/shopping-list.md).
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
and plans a .032" wire for it through all eight contacts. Add `--auto` to use
the 3-click design instead. Open `demo/preview.svg` and read
`demo/report.txt`.

## Make a wire

1. **Export** the lower scan from iTero as an STL.
2. **Click 3 points.** Open `picker/picker.html` in Chrome, load the STL,
   and with **Auto** selected click:
   - 43: the lingual, where the wire should end (centre of the bond pad)
   - the midline: lingual of 41/31, at the height the wire should run (about
     two-thirds up)
   - 33: the lingual, where the wire should end

   Click **Download picks.json**. If you'd rather place every contact
   yourself, choose **Every contact** instead and click 43 distal and
   mesial, 42, 41, 31, 32, then 33 mesial and distal.
3. **Plan:**
   ```bash
   rbend plan picks.json --scan lower.stl --opposing upper.stl --wire 032 -o case123
   ```
   Read `case123/report.txt` and look at `case123/preview.svg`. The report
   gives the gap at every contact, the closest approach along the wire, how
   near the upper teeth come in occlusion (pass the upper scan as iTero
   exports it, already in occlusion), and whether the machine can make every
   bend. Open `wire.stl` together with the scan in any STL viewer to see the
   wire in 3D.
4. **Bend:**
   ```bash
   rbend send case123/program.txt --port /dev/ttyACM0
   ```
   Load the wire tip flush with the die before sending. When it finishes, cut
   at the die and snip the 6 mm lead-in off the end that came out first.
5. **Check the result.** Lay the wire on the printed `template_1to1.svg`,
   drop it into the printed `check_jig.stl`, and try it on the printed model.

## How the planner works

- Each click is moved out along the tooth's surface normal by the wire radius,
  which gives the wire's centre line there. A plane is fitted through those
  centres: with 3 clicks it passes exactly through them.
- **Auto (3 clicks):** the scan is cut at the wire's plane, and also a wire
  radius above and below it, because the lingual surface bulges toward the
  cingulum just under the wire's centre. Rays fan out from behind the arch to
  find how far out the wire can sit at each point. Along that outline each
  tooth shows up as a dip and each embrasure as a bump, so the wire's contact
  with each incisor is the bottom of its dip. A smooth curve runs through
  those contacts and the two canine clicks.
- **Every contact:** the clicks are flattened onto the plane, and the report
  says how far each had to move. A flat wire can't reach a contact well above
  or below the rest. A smooth curve runs through them.
- Either way, wherever the curve would cut into a tooth (across a convex
  canine, for example), it is pushed out just clear.
- A curl is added at each end, 90° at 2 mm radius by default. It turns back in
  toward the midline, like the usual hand-bent 3-3. Set it with `--hook-angle`
  and `--hook-radius`, turn it onto the canine with `--hook-toward tooth`, or
  leave it off with `--hook-angle 0`.
- The curve is replaced by straight pieces at least 1 mm long that stay within
  0.04 mm of it. That gives a feed length for each piece and a bend angle at
  each joint, usually about 20 bends: 3–15° along the arc and 20–30° in the curls.
- The program is simulated on the machine bend by bend. For each bend it
  finds where to raise the pin clear of the wire already out (curls
  included) and where the pin first touches it. It then turns the arm past
  that point by the target angle plus the over-bend from the wire's
  springback table. It also checks feed travel, arm reach, and that the free
  end never swings into the guide.

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
| `rbend plan PICKS --scan STL [--opposing STL] --wire 028/032 -o DIR` | plan a wire (3 clicks: auto design) |
| `rbend demo [--auto] -o DIR` | plan for the synthetic arch |
| `rbend calibrate --wire 032 -o DIR` | write the springback test program and a measuring sheet |
| `rbend fit-springback SHEET.csv --wire 032 -o config/springback_ss_032.json` | turn measurements into a table |
| `rbend send PROGRAM --port PORT [--dry-run]` | stream a program to the machine |

## Tests

```bash
pytest
```
