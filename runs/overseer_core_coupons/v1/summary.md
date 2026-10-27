# Overseer core mounting coupons v1 (27 Oct 2026)

Spark 27 Oct: "make a fast to print mounting check coupons first, in the orientation we are going to put them in. make it fast to print". These are drawn with the least material: only the mounting features of the two core pieces of round 5 (Spark 27 Oct: the FC stack needs no mount, it bolts to the carbon top plate) (`nox/overseer/core_v1.py`), with every position read from `runs/overseer_core/v1/result.json`, on 6 x 4 bars. Script: `nox/overseer/core_coupons_v1.py`.

overseer core coupons v1: ready to print
System: Overseer core v1 round 4 (`runs/overseer_core/v1/summary.md`); the coupons carry the pieces' interfaces only, no load case
Weight: hub deck 17.9 g, sled 29.2 g (M, solid)
Strength: n/a (fit check, not flown)
Solid: solid_check PASS on both
Print: each one printed flat in the pose it is fitted in (M):
- hub deck: bottom-plate face down, bed contact 3389 mm², no overhang
- sled: its top (the face against the plate) down, so it bolts on flipped over as printed; bed contact 4877 mm², 185 mm² of bridge over the flow pocket
Files and sizes (M):
- `print/overseer_core_coupon_hub_deck_v1.stl`: 154 x 85 x 8
- `print/overseer_core_coupon_sled_v1.stl`: 177 x 125 x 8.8
Previews: `preview_overseer_core_coupon_hub_deck_v1.png`, `preview_overseer_core_coupon_sled_v1.png`

## What each one checks
- **Hub deck**, on the bottom plate before the arms go on.
  - 6 M3 go down through the feet at (±24, ±36.5) and (-65, ±22.5). The holes are Ø3.4; the plate holes are Ø3.1 for M3 (CAD).
  - The Pi 5 goes on its 4 posts with M2.5 self-tapping screws (Ø2.2 x 4).
  - Things to see: the Pi sits between the arm roots, its plugs go in aft, and with the top plate on the cooler has room. The UBEC and buck lie on the front rails.
- **Sled**, under the bottom plate, flipped over as printed.
  - The 6 bolts line up through the plate (Ø4.0: inserts, or nuts for the check).
  - The 8 arm bottom-screw heads pass the Ø9 rings.
  - The MTF-02P fits its pocket: 0.2 a side and 0.3 deep extra, with the lens window and the cable slot on the short edge. This checks plan position and fit only; the 55 drop to the lens is not in the coupon.
  - The 6 receiver bolt pads wait for the receiver.

Write what you find (holes off, grommet fit, Pi clearance) as P in `dimensions.md` M and in System Overseer core.
