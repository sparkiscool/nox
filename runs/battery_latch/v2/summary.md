# battery latch v2 (24 Oct 2026): receiver latch block v2, XT60 carrier v1, with cartridge v5, sliders v5, Scout end skids

Script `nox/box/battery_latch.py`; FE `battery_latch_topo.py --swap-v1 / --swap-v2` (outputs in `fe/`); figures `battery_latch_figure.py` (`figs/`). Doc `nox/box/battery-latch.md`. Numbers `nox/dimensions.md` "System box swap: latch". Tags: M mesh, C CAD, E estimate, F frame model. Supersedes `runs/battery_latch/v1/` (receiver v1).

**Why v2:** Spark (24 Oct): the bay latch "seems to bend since its just thin plastic", and the arm now plugs and unplugs the XT60 on every swap.
- Swap load set (arm, E): 74 N stall insert, 57 N pull, 43.5 N across, 7.6 N / 1.05 N·m centring. No guide couple: the arm's pitch fits the XT60 float, so there are no guide posts.
- On it v1 reaches index 2.08 (Scout) / 1.53 (Overseer) in 43.5 N across, and its feet move 5.4 / 4.4 mm.

**Scout set: HOLD, do not print** (assembled probe FAIL at the Scout's side windows; the Scout chat). **Overseer set: DESIGN, do not print** (mount bolts and system open). Files only in `stl/`, previews beside them.

```
battery latch v2 (Scout set): HOLD
System: Scout v3 (latch refit snapshot 3595381, current latch parts, skids as cage): cage first 4.5 mm (E), fit 0.0 mm³ (E), bbox 6.9 L (E), extents 233 x 233 x 128 mm (E), mass 794 g (E), CG 1.4 mm off thrust axis (E), 2.0 mm without swap parts, probes −47.0 mm (E) at the receiver's tail end FAIL (Scout side windows; swap-opening exemption 40°), access 7/7 clear; collar planes 7.04 (C, inset 11.0)
Weight: Scout set 147.7 g (M), v1 91.0 g (M): the grams buy 43.5 N across 0.35 mm (v1 5.4) and swap worst 0.51 (v1 2.08)
Strength: receiver v2 FE (h 1.0, deck-down print, M mesh, E loads): swap worst 0.51 (4.6 MPa in-layer, 2.8 across layers), crash 0.07 at 15.3 g, handling 0.12; 74 N insert deflection 0.001 mm; first mode ~560 Hz (E); sliders at the 74 N stall 0.22, grip pull 0.40 (E)
Solid: solid_check PASS on all 8 Scout files
Print: receiver mount plane down (bed 1318 mm², M), carrier ear tops down (796), cap top down (1001), cartridge closed end down (1248), slider bar top down (328), strips flat (339, TPU), skids chamfer face down (537 / 1298, TPU); tree supports
```
Overseer set (DESIGN):
- `System: Overseer, cage first −59.0 mm (E) at (+0.91, −0.01, −0.41) (Unitree L2) FAIL, fit 27792.2 mm³ (E) (belly sled + pods / receiver) FAIL, bbox 85.7 L (E), extents 634 x 570 x 237 mm (E), mass 2636 g (E), CG 10.7 mm off thrust axis (E), 26.8 mm without swap parts FAIL (tol 3), probes −0.0 mm (E) edge at (+0.71, −0.06, +0.70) (Unitree L2, unshielded) FAIL` (pose B; the sled was cut for receiver v1).
- Its mount bolts at x ±27.8 come up through the cartridge's rail zone (v1 too). Open item for the Overseer and box sessions.
- Latch set 66.8 x 68.8 x 183.6 mm (0.84 L, M).
- Receiver FE (h 1.5, M mesh, E loads): swap worst 0.55 (2.7 MPa in-layer, 3.0 across), crash 0.39 (35 g), 43.5 N across 0.78 mm; first mode ~460 Hz (E).
- Pitch at the XT60's first contact > 4° (C). Set 331.3 g (M); v1 was 185.9.

| Piece | Scout g (M) | Overseer g (M) | Extents in print pose, mm (M) | Bed mm² (M) |
|---|---|---|---|---|
| cartridge v5 | 31.85 | 89.70 | 40.0 x 45.6 x 96.0 / 55.0 x 58.6 x 174.0 | 1248 / 3125 |
| slider v5 (x 2) | 3.18 | 4.04 | 18.6 x 22.0 x 11.5 / 26.1 x 22.0 x 11.5 | 328 / 463 |
| receiver v2 | 93.10 | 216.24 | 63.6 x 105.6 x 53.9 / 66.8 x 183.6 x 68.6 | 1318 / 2626 |
| XT60 carrier v1 | 4.62 | 4.95 | 61.2 x 20.0 x 7.0 / 65.6 x 20.0 x 7.0 | 796 / 884 |
| receiver lid v1 (carrier cap) | 5.34 | 8.14 | 61.2 x 20.0 x 5.5 / 65.6 x 20.0 x 7.2 | 1001 / 1089 |
| receiver strip v1 (TPU, x 2) | 0.98 | 2.07 | 4.8 x 70.6 x 2.4 / 4.8 x 148.6 x 2.4 | 339 / 713 |
| skid closed v1 (TPU, Scout) | 1.38 | – | 38.0 x 16.3 x 2.1 | 537 |
| skid lead v1 (TPU, Scout) | 3.12 | – | 38.0 x 34.2 x 2.1 | 1298 |

solid_check per file (`log.txt`):
- box_battery_cartridge_scout_v5.stl PASS
- box_battery_slider_scout_v5.stl PASS
- box_battery_receiver_scout_v2.stl PASS
- box_battery_xt60_carrier_scout_v1.stl PASS
- box_battery_receiver_lid_scout_v1.stl PASS
- box_battery_receiver_strip_scout_v1.stl PASS
- box_battery_skid_closed_scout_v1.stl PASS
- box_battery_skid_lead_scout_v1.stl PASS
- box_battery_cartridge_overseer_v5.stl PASS
- box_battery_slider_overseer_v5.stl PASS
- box_battery_receiver_overseer_v2.stl PASS
- box_battery_xt60_carrier_overseer_v1.stl PASS
- box_battery_receiver_lid_overseer_v1.stl PASS
- box_battery_receiver_strip_overseer_v1.stl PASS

Swap load set, worst index per case (Scout / Overseer, flight SF 4: 10.8 MPa in-layer, 5.4 across layers at 1.0; deflection = max |u| at the loaded faces, mm):
| Case | Scout | Overseer | Deflection Scout / Overseer |
|---|---|---|---|
| insert 74 N on the XT60 seat | 0.01 | 0.00 | 0.001 / 0.001 |
| insert 74 N on the stops | 0.03 | 0.01 | 0.003 / 0.004 |
| pull 57 N on the XT60 lips | 0.07 | 0.06 | 0.014 / 0.016 |
| pull 57 N on the bolts | 0.21 | 0.12 | 0.097 / 0.021 |
| insert 74 N + across 43.5 N | 0.41 | 0.45 | 0.300 / 0.609 |
| across 43.5 N +x | 0.51 | 0.55 | 0.350 / 0.780 |
| centring 7.6 N + 1.05 N m | 0.24 | 0.25 | 0.142 / 0.214 |

Other checks (C unless tagged):
- Insertion 50 mm every 1.0: 0 mm³, no stuck step.
- 0.3 off in x: receiver 0 mm³. The rigid-modelled XT60 male meets the plug 10.6 mm³ in the last 4; its float carrier takes ±1.0.
- Gripped pull-out 0 mm³; squeeze 0..4 0 mm³; asymmetric squeeze ±1.0 0 mm³, pull-out clear.
- Hall 157 / 1.7 / 3.1 mT (latched / half in / squeezed, E model).

Reviews: receiver v2 concept review FAIL, 6 findings, all taken or answered (doc, "Receiver v2"). The PR re-review follows this run.
