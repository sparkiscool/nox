# battery latch v3 (24 Oct 2026): cartridge v6 (centre rib) + the Scout TPU edge skids, on receiver v2

Script `nox/box/battery_latch.py`; figures `battery_latch_figure.py` (`figs/`); FE reused from `runs/battery_latch/v2/` (receiver body unchanged, copied into `fe/`). Doc `nox/box/battery-latch.md` ("Cartridge v6 and the Scout skids"). Numbers `nox/dimensions.md` "System box swap: latch". Layout artifact: https://claude.ai/artifact/MX2dpRcjHegntjHvqp6U8x. Tags: M mesh, C CAD, E estimate. Supersedes `runs/battery_latch/v2/` (the cartridge v5 there).

**What changed:**
- **Cartridge v6 centre rib** (heads v7 ask): the tabs stop on it at ±1.0 / ±7.0, stroke 3.0 (C). Both bolts retract 3.0 at any head offset (need 2.6). Release 2.6 / 4.1 N per finger (E).
- **Scout TPU edge skids** (Scout v3 ask, cage role): cartridge strips (2), receiver nose plate, receiver side skids (strip + nose bar, 2).

**Scout set: HOLD** (the Scout's layout with these skids lands first; its pass reads every latch probe PASS, Scout v3, E). **Overseer set: DESIGN** (mount bolts open). Nothing in `print/`.

```
battery latch v3 (Scout set): HOLD
System (old refit snapshot 3595381, its cage predates the skids): cage first 4.5 mm (E), fit 59.6 mm³ (E) (snapshot cage rods / receiver side skid) FAIL, mass 803 g (E), CG 1.6 mm off thrust axis (E), 2.3 mm (E) without swap parts, probes −43.5 mm (E) FAIL (side windows), access 7/7 clear; collar planes 7.04 (C); assembled worst index: Scout frame model ≤ 0.98 (F), latch parts ≤ 0.51 (M mesh, E loads, receiver v2 FE)
Weight: Scout set 156.0 g (M) (receiver v2 run 147.7, v1 91.0)
Strength: receiver v2 FE unchanged (swap worst 0.51 / 0.55, M mesh, E loads); sliders at a 74 N stall 0.22, grip pull 0.40 (E)
Solid: solid_check PASS on all 12 Scout files
```
Overseer: set 331.9 g (M); cartridge v6 rib only (no skids); system as run v2 (`System: Overseer, cage first −59.0 mm (E) (Unitree L2) FAIL, fit 27792.2 mm³ (E) (belly sled / receiver) FAIL, CG 10.7 / 26.8 mm (E) FAIL, probes −0.0 mm (E) (L2) FAIL`); assembled worst index open, latch parts ≤ 0.55 (M mesh, E loads).

| Pack | File | Material x count | g (M) | Extents in print pose, mm (M) | Bed mm² (M) | Bed face |
|---|---|---|---|---|---|---|
| Scout | box_battery_cartridge_scout_v6.stl | PETG x 1 | 32.52 | 40.0 x 45.6 x 96.0 | 1248 | closed end (z 0) down |
| Scout | box_battery_cart_strip_scout_v1.stl | TPU x 2 | 1.12 | 4.2 x 64.0 x 5.0 | 269 | foot down (one part, both sides) |
| Scout | box_battery_rskid_plate_scout_v1.stl | TPU x 1 | 2.79 | 65.5 x 16.0 x 2.2 | 1048 | flat, wall face down |
| Scout | box_battery_rskid_side_px_scout_v1.stl | TPU x 1 | 1.29 | 8.7 x 88.8 x 29.0 | 174 | strip foot down, the nose bar on tree supports |
| Scout | box_battery_rskid_side_nx_scout_v1.stl | TPU x 1 | 1.29 | 8.7 x 88.8 x 29.0 | 174 | strip foot down, the nose bar on tree supports (mirror of px) |
| Scout | box_battery_slider_scout_v5.stl | PETG x 2 | 3.18 | 18.6 x 22.0 x 11.5 | 328 | bar top (y 11.5) down: largest face, bar layers along its length |
| Scout | box_battery_receiver_scout_v2.stl | PETG x 1 | 93.10 | 63.6 x 105.6 x 53.9 | 1318 | mount plane (deck top) down: layers across the install axis |
| Scout | box_battery_xt60_carrier_scout_v1.stl | PETG x 1 | 4.62 | 61.2 x 20.0 x 7.0 | 796 | ear tops down (pocket open to the bed) |
| Scout | box_battery_receiver_lid_scout_v1.stl | PETG x 1 | 5.34 | 61.2 x 20.0 x 5.5 | 1001 | top face down (the XT60 carrier's cap, = v1 lid) |
| Scout | box_battery_receiver_strip_scout_v1.stl | TPU x 2 | 0.98 | 4.8 x 70.6 x 2.4 | 339 | flat |
| Scout | box_battery_skid_closed_scout_v1.stl | TPU x 1 | 1.38 | 38.0 x 16.3 x 2.1 | 537 | outer (chamfer) face down |
| Scout | box_battery_skid_lead_scout_v1.stl | TPU x 1 | 3.12 | 38.0 x 34.2 x 2.1 | 1298 | outer (chamfer) face down |
| Overseer | box_battery_cartridge_overseer_v6.stl | PETG x 1 | 90.35 | 55.0 x 58.6 x 174.0 | 3125 | closed end (z 0) down |
| Overseer | box_battery_slider_overseer_v5.stl | PETG x 2 | 4.04 | 26.1 x 22.0 x 11.5 | 463 | bar top (y 11.5) down: largest face, bar layers along its length |
| Overseer | box_battery_receiver_overseer_v2.stl | PETG x 1 | 216.24 | 66.8 x 183.6 x 68.6 | 2626 | mount plane (deck top) down: layers across the install axis |
| Overseer | box_battery_xt60_carrier_overseer_v1.stl | PETG x 1 | 4.95 | 65.6 x 20.0 x 7.0 | 884 | ear tops down (pocket open to the bed) |
| Overseer | box_battery_receiver_lid_overseer_v1.stl | PETG x 1 | 8.14 | 65.6 x 20.0 x 7.2 | 1089 | top face down (the XT60 carrier's cap, = v1 lid) |
| Overseer | box_battery_receiver_strip_overseer_v1.stl | TPU x 2 | 2.07 | 4.8 x 148.6 x 2.4 | 713 | flat |

solid_check per file (`log.txt`):
- box_battery_cartridge_scout_v6.stl PASS
- box_battery_cart_strip_scout_v1.stl PASS
- box_battery_rskid_plate_scout_v1.stl PASS
- box_battery_rskid_side_px_scout_v1.stl PASS
- box_battery_rskid_side_nx_scout_v1.stl PASS
- box_battery_slider_scout_v5.stl PASS
- box_battery_receiver_scout_v2.stl PASS
- box_battery_xt60_carrier_scout_v1.stl PASS
- box_battery_receiver_lid_scout_v1.stl PASS
- box_battery_receiver_strip_scout_v1.stl PASS
- box_battery_skid_closed_scout_v1.stl PASS
- box_battery_skid_lead_scout_v1.stl PASS
- box_battery_cartridge_overseer_v6.stl PASS
- box_battery_slider_overseer_v5.stl PASS
- box_battery_receiver_overseer_v2.stl PASS
- box_battery_xt60_carrier_overseer_v1.stl PASS
- box_battery_receiver_lid_overseer_v1.stl PASS
- box_battery_receiver_strip_overseer_v1.stl PASS

Checks (C unless tagged):
- Insertion 50 mm every 1.0 with the strips and skids: 0 mm³, no stuck step.
- Grip: retract 3.0 at head offset −1 / 0 / +1. Asymmetric squeeze ±1.0: 0 mm³, pull-out clear, tips 0.7 inside the face.
- Hall 157 / 1.7 / 2.9 mT (latched / half in / squeezed, E model).
- Skids ≥ 3.8 inside the funnel planes (need 1.0).

Review: owed on the PR.
