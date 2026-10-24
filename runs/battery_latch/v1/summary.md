# battery latch v1 (24 Oct 2026): cartridge v5 (Scout, Overseer), receiver latch block v1, sliders

Script `nox/box/battery_latch.py` (+ `battery_latch_topo.py --fe-v1 / --mode` for the receiver FE and first mode, `battery_latch_figure.py`). Doc `nox/box/battery-latch.md`. Numbers `nox/dimensions.md` "System box swap: latch". Drawn parts (the latch module and its locating faces are must-haves; the receiver topology attempt failed and v1 stays, doc). Tags: M mesh, C CAD, E estimate, F frame model.

**Scout set: READY** (files in `print/`). **Overseer set: DESIGN, do not print** (files only in `stl/`) until funnel v8 and the arm's Y_C / TAB_OFF decision land (rule 9).

```
battery latch v1 (Scout set): ready to print
System: Scout v3 (latch refit, 3595381, current latch parts): cage first 4.5 mm (E), fit 0.0 mm³ (E), bbox 6.9 L (E), extents 233 x 233 x 128 mm (E), mass 734 g (E), CG 1.3 mm off thrust axis (E), 2.0 mm without swap parts, probes −77.8 mm (E) at the cartridge FAIL (Scout side windows; its DEFAULT exempts the swap openings to 35°), access 7/7 clear; assembled worst index ≤ 0.98 (F, Scout frame model), latch parts ≤ 0.67 (handling on a receiver foot), crash 0.17 at the suspension's ~16.7 g (E)
Weight: Scout set 87.5 g (M), was 122.3 g (M, PR 44)
Strength: receiver FE crash worst 0.16 at 15.3 g (E loads, M mesh), handling 50 N on a foot 0.67; first mode of the seated pack ~373 Hz (E); latch hand calc worst 0.35 (grip pull with the XT60, E)
Solid: solid_check PASS on all 5 Scout print files
Print: cartridge closed end down (bed 1267 mm², M), receiver closed-end face down (837 mm², M), lid top down (1001 mm², M), slider bar top down (328 mm², M), strips flat (339 mm², M, TPU); tree supports; files print/box_battery_*_scout_*
```
Overseer set (DESIGN): `System: Overseer (main, pose B), cage first −59.0 mm (E) (Unitree L2) FAIL, fit 11845.6 mm³ (E) (belly sled / receiver) FAIL, bbox 85.7 L (E), extents 634 x 570 x 237 mm (E), mass 2491 g (E), CG 13.0 mm off thrust axis (E), 31.5 mm without swap parts FAIL`; latch set 66.8 x 68.8 x 179.6 mm (0.83 L, M); receiver FE crash worst 0.56 (35 g, E loads), handling 0.69, first mode ~355 Hz (E); set 185.9 g (M).

| Piece | Scout g (M) | Overseer g (M) | Extents in print pose, mm (M) | Bed mm² (M) |
|---|---|---|---|---|
| cartridge v5 | 33.06 | 89.7 | 40.0 x 45.6 x 96.0 / 55.0 x 58.6 x 174.0 | 1267 / 3125 |
| slider v5 (x 2) | 3.18 | 4.04 | 18.6 x 22.0 x 11.5 / 26.1 x 22.0 x 11.5 | 328 / 463 |
| receiver v1 | 40.8 | 76.12 | 61.2 x 53.9 x 101.6 / 66.8 x 68.6 x 179.6 | 837 / 1229 |
| receiver lid v1 | 5.34 | 7.85 | 61.2 x 20.0 x 5.5 / 63.6 x 20.0 x 7.2 | 1001 / 1049 |
| receiver strip v1 (TPU, x 2) | 0.98 | 2.07 | 4.8 x 70.6 x 2.4 / 4.8 x 148.6 x 2.4 | 339 / 713 |

Checks (C unless tagged): insertion 50 mm every 1.0 (and 0.3 off) 0 mm³, no stuck step; gripped pull-out 0 mm³; squeeze 0..4 0 mm³; asymmetric squeeze ±1.0 0 mm³, short bolt tip 0.7 inside the face; hall 157 / 1.7 / 3.1 mT (latched / half in / squeezed, nominal, E model), ≥ 109 vs ≤ 16 at head offset ±0.5; side-hit slider hold x1.27 / x2.07 (E); collar planes 7.00 (Scout); receiver bolts up from below (from above the frame plate and props block the key). Concept review FAIL x6 fixed, re-review PASS; PR review 1 FAIL fixed (this version). Wall time of the script ~5 min (4 cores, M).

Receiver topology attempt (Scout, branch `run/battery_latch_receiver-scout-v1`): 34.8 g (M), solid_check FAIL, worst 25.3 (M mesh, E loads), not converged: does not beat v1; the drawn receivers stay (doc, "Receiver topology attempt"). The receivers' top band + 3 legs a side are drawn structure owed a topology rerun (Scout and Overseer) as a follow-up.

## Previews (rendered from the exact print files)
| Print file | Preview |
|---|---|
| `print/box_battery_cartridge_scout_v5.stl` | `runs/battery_latch/v1/preview_box_battery_cartridge_scout_v5.png` |
| `runs/battery_latch/v1/stl/box_battery_cartridge_overseer_v5.stl` (DESIGN, do not print) | `runs/battery_latch/v1/preview_box_battery_cartridge_overseer_v5.png` |
| `print/box_battery_slider_scout_v5.stl` | `runs/battery_latch/v1/preview_box_battery_slider_scout_v5.png` |
| `runs/battery_latch/v1/stl/box_battery_slider_overseer_v5.stl` (DESIGN, do not print) | `runs/battery_latch/v1/preview_box_battery_slider_overseer_v5.png` |
| `print/box_battery_receiver_scout_v1.stl` | `runs/battery_latch/v1/preview_box_battery_receiver_scout_v1.png` |
| `runs/battery_latch/v1/stl/box_battery_receiver_overseer_v1.stl` (DESIGN, do not print) | `runs/battery_latch/v1/preview_box_battery_receiver_overseer_v1.png` |
| `print/box_battery_receiver_lid_scout_v1.stl` | `runs/battery_latch/v1/preview_box_battery_receiver_lid_scout_v1.png` |
| `runs/battery_latch/v1/stl/box_battery_receiver_lid_overseer_v1.stl` (DESIGN, do not print) | `runs/battery_latch/v1/preview_box_battery_receiver_lid_overseer_v1.png` |
| `print/box_battery_receiver_strip_scout_v1.stl` | `runs/battery_latch/v1/preview_box_battery_receiver_strip_scout_v1.png` |
| `runs/battery_latch/v1/stl/box_battery_receiver_strip_overseer_v1.stl` (DESIGN, do not print) | `runs/battery_latch/v1/preview_box_battery_receiver_strip_overseer_v1.png` |
