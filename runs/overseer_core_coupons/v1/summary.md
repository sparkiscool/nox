# Overseer core mounting coupons v1 (27 Oct 2026)

Spark 27 Oct: "make a fast to print mounting check coupons first, in the orientation we are going to put them in. make it fast to print". Drawn, least material: only the mounting features of the two core pieces (`nox/overseer/core_v1.py`, numbers read from `runs/overseer_core/v1/result.json`) on 6 x 4 bars. Script `nox/overseer/core_coupons_v1.py`.

overseer core coupons v1: ready to print
System: Overseer core v1 round 3 (`runs/overseer_core/v1/summary.md`); coupons carry the pieces' interfaces only, no load case
Weight: hub deck 33.8 g, sled 27.9 g (M, solid)
Strength: n/a (fit check, not flown)
Solid: solid_check PASS on both
Print: hub deck plate face down (as it sits on the plate), bed contact 4098 mm² (M), 736 mm² of short bridges under the Pi rails; sled top face down (bolts on flipped, as printed), bed contact 4593 mm² (M); files `print/overseer_core_coupon_hub_deck_v1.stl` (174 x 85 x 22), `print/overseer_core_coupon_sled_v1.stl` (178 x 125 x 8.5)
Previews: `preview_overseer_core_coupon_hub_deck_v1.png`, `preview_overseer_core_coupon_sled_v1.png`

## What each one checks
- **Hub deck coupon** on the bottom plate's top (before the arms): 6 feet on the plate holes (±24, ±36.5) and (-72.5, ±31.75) with M3 (nuts under the plate for the check); the stack on its 4 posts (Ø4.0 x 5: M3 x 4 short inserts, or M3 nuts); the ESC's 44 x 43.5 and the FC's USB-C clear; the Pi 5 on its 4 posts (M2.5 self-tapping, Ø2.2) at its real height: cooler under the top plate edge, ports aft, 4 Pi screws straight down with the top plate on; nothing touches the arm roots. The two aft legs stand on the bed here (on the sled's tail in the drone); the leg under the rails' aft end is for printing only.
- **Sled coupon** under the bottom plate, flipped over as printed: the 8 bolts line up through the plate (Ø4.0 holes: inserts or nuts), the 8 arm bottom screw heads pass the Ø9 rings, the MTF-02P fits its pocket (0.2 clearance, lens window, cable slot on the short edge). The 6 receiver bolt pads wait for the receiver.

Write what you measure (hole positions off, grommet fit, Pi clearance) as P in `dimensions.md` M and System Overseer core.
