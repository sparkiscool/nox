# battery latch v2 (24 Oct 2026): receiver latch block v2, XT60 carrier v1, with cartridge v5, sliders v5, Scout end skids

Script `nox/box/battery_latch.py`; FE `battery_latch_topo.py --swap-v1 / --swap-v2` (outputs in `fe/`); figures `battery_latch_figure.py` (`figs/`). Doc `nox/box/battery-latch.md`. Numbers `nox/dimensions.md` "System box swap: latch". Tags: M mesh, C CAD, E estimate, F frame model. Supersedes `runs/battery_latch/v1/` (receiver v1).

**Why v2:** Spark (24 Oct): the bay latch "seems to bend since its just thin plastic", and the arm now plugs and unplugs the XT60 on every swap. The swap load set (arm, E) is 74 N stall insert, 57 N pull, pitch guide 3.5 N·m (6.1 misuse), 43.5 N across, 7.6 N / 1.05 N·m centring. On it v1 reaches guide index 3.59 (Scout) / 1.72 (Overseer), and 43.5 N across moves its feet 5.3 / 4.4 mm.

**Scout set: HOLD, do not print** (assembled probe FAIL at the Scout's side windows; Scout chat). **Overseer set: DESIGN, do not print** (mount bolts and system open). Files only in `stl/`, previews beside them.

```
battery latch v2 (Scout set): HOLD
System: Scout v3 (latch refit snapshot 3595381, current latch parts, skids as cage): cage first 4.5 mm (E), fit 0.0 mm³ (E), bbox 6.9 L (E), extents 233 x 233 x 128 mm (E), mass 788 g (E), CG 1.5 mm off thrust axis (E), 2.1 mm without swap parts, probes −46.1 mm (E) at the receiver's tail end FAIL (Scout side windows), access 7/7 clear; collar planes 7.08 (C, inset 11.0)
Weight: Scout set 141.6 g (M), v1 91.0 g (M): the grams buy 43.5 N across 0.40 mm (v1 5.3) and swap worst 0.61 (v1 3.59)
Strength: receiver v2 FE (h 1.0, deck-down print, M mesh, E loads): swap worst 0.61 (≤ 6.6 MPa in-layer, ≤ 3.3 across layers), crash 0.07 at 15.3 g, handling 0.10; 74 N insert deflection 0.001 mm; first mode ~559 Hz (E); sliders at the 74 N stall 0.22, grip pull 0.40 (E)
Solid: solid_check PASS on all 8 Scout files
Print: receiver mount plane down (bed 1217 mm², M), carrier ear tops down (796), cap top down (1001), cartridge closed end down (1248), slider bar top down (328), strips flat (339, TPU), skids chamfer face down (537 / 1298, TPU); tree supports
```
Overseer set (DESIGN): `System: Overseer, cage first −59.0 mm (E) at (+0.91, −0.01, −0.41) (Unitree L2) FAIL, fit 27630.3 mm³ (E) (belly sled + pods / receiver) FAIL, bbox 85.7 L (E), extents 634 x 570 x 237 mm (E), mass 2638 g (E), CG 10.6 mm off thrust axis (E), 26.7 mm without swap parts FAIL (tol 3), probes −0.0 mm (E) edge at (+0.71, −0.06, +0.70) (Unitree L2, unshielded) FAIL` (pose B; the sled was cut for receiver v1). Its mount bolts at x ±26.8 come up through the cartridge's rail zone (v1 too): an open item for the Overseer and box sessions. The latch set is 66.8 x 68.8 x 181.3 mm (0.83 L, M). Receiver FE (h 1.5, M mesh, E loads): swap worst 0.46, crash 0.39 (35 g), across deflection 0.77 mm, first mode ~461 Hz (E); pitch at the XT60's first contact ±0.36° (C). Set 333.2 g (M); v1 was 185.9.

| Piece | Scout g (M) | Overseer g (M) | Extents in print pose, mm (M) | Bed mm² (M) |
|---|---|---|---|---|
| cartridge v5 | 31.85 | 89.7 | 40.0 x 45.6 x 96.0 / 55.0 x 58.6 x 174.0 | 1248 / 3125 |
| slider v5 (x 2) | 3.18 | 4.04 | 18.6 x 22.0 x 11.5 / 26.1 x 22.0 x 11.5 | 328 / 463 |
| receiver v2 | 86.91 | 218.16 | 63.6 x 103.6 x 53.9 / 66.8 x 181.3 x 68.6 | 1217 / 2494 |
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

Swap load set, worst index per case (Scout / Overseer, flight SF 4, the 6.1 N·m case misuse SF 2.5; deflection max |u| at the loaded faces, mm):
| Case | Scout | Overseer | Deflection Scout / Overseer |
|---|---|---|---|
| insert 74 N on the XT60 seat | 0.01 | 0.00 | 0.001 / 0.001 |
| insert 74 N on the stops | 0.03 | 0.01 | 0.003 / 0.004 |
| pull 57 N on the XT60 lips | 0.07 | 0.06 | 0.015 / 0.016 |
| pull 57 N on the bolts (fault) | 0.21 | 0.12 | 0.095 / 0.021 |
| guide 3.5 N·m (worse sense) | 0.56 | 0.10 | 0.27 / 0.017 |
| guide 6.1 N·m (misuse) | 0.61 | 0.11 | 0.47 / 0.029 |
| insert 74 N + guide 3.5 N·m | 0.56 | 0.10 | 0.26 / 0.014 |
| across 43.5 N | 0.61 | 0.46 | 0.40 / 0.77 |
| centring 7.6 N + 1.05 N·m | 0.27 | 0.20 | 0.15 / 0.20 |

The Scout has no guide posts. Its guide case is the seated couple, one end's stops against the bolts over a 23 mm lever. The Overseer's guide case is the post couple over a 47 mm lever.

Other checks (C unless tagged):
- Insertion 50 mm every 1.0: 0 mm³, no stuck step.
- 0.3 off in x: receiver 0 mm³. The rigid-modelled XT60 male meets the plug 10.6 mm³ in the last 4; its float carrier takes ±1.0.
- Gripped pull-out 0 mm³; squeeze 0..4 0 mm³; asymmetric squeeze ±1.0 0 mm³, pull-out clear.
- Hall 157 / 1.7 / 3.1 mT (latched / half in / squeezed, E model).

Reviews: receiver v2 concept review FAIL, 6 findings, all taken or answered (doc, "Receiver v2"). The PR re-review is owed. Script wall time: ~25 min plus the FE runs (`fe/`).
