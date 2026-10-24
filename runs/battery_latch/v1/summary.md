**Superseded by `runs/battery_latch/v2/` (receiver v2, 24 Oct): v1 failed the swap load set (guide 3.59 / 1.72, 43.5 N across moved its feet 5.3 / 4.4 mm). Meshes recoverable from git.**

# battery latch v1 (24 Oct 2026): cartridge v5 (Scout, Overseer), receiver latch block v1, sliders, Scout end skids

Script `nox/box/battery_latch.py` (+ `battery_latch_topo.py --fe-v1 / --mode` for the receiver FE and first mode, `battery_latch_figure.py`). Doc `nox/box/battery-latch.md`. Numbers `nox/dimensions.md` "System box swap: latch". Drawn parts (the latch module and its locating faces are must-haves; the receiver topology attempt failed and v1 stays, doc). Tags: M mesh, C CAD, E estimate, F frame model.

**Scout set: READY** (files in `print/`). **Overseer set: DESIGN, do not print** (files only in `stl/`) until funnel v8 and the arm's Y_C decision land (rule 9).

```
battery latch v1 (Scout set): ready to print
System: Scout v3 (latch refit, 3595381, current latch parts, skids as cage): cage first 4.5 mm (E), fit 0.0 mm³ (E), bbox 6.9 L (E), extents 233 x 233 x 128 mm (E), mass 738 g (E), CG 1.4 mm off thrust axis (E), 2.0 mm without swap parts, probes −43.9 mm (E) at the receiver's tail end FAIL (Scout side windows), access 7/7 clear; collar planes 7.00 at clip inset 10.5 (C); XT60 pitch taken by the XT60E1-M float ±2.37° (C, no guide posts, DEFAULT); assembled worst index ≤ 0.98 (F, Scout frame model), latch parts ≤ 0.63 (handling on a receiver foot, E loads), crash 0.16 at the suspension's ~16.7 g (E)
Weight: Scout set 91.0 g (M), was 122.3 g (M, PR 44)
Strength: receiver FE crash worst 0.16 at 15.3 g (E loads, M mesh), handling 50 N on a foot 0.67; first mode of the seated pack ~373 Hz (E); latch hand calc worst 0.35 (grip pull with the XT60, E)
Solid: solid_check PASS on all 7 Scout print files
Print: cartridge closed end down (bed 1267 mm², M), receiver closed-end face down (837 mm², M), lid top down (1001 mm², M), slider bar top down (328 mm², M), strips flat (339 mm², M, TPU), skids chamfer face down (537 / 1271 mm², M, TPU); tree supports; files print/box_battery_*_scout_*
```
Overseer set (DESIGN): `System: Overseer (main, pose B), cage first −59.0 mm (E) (Unitree L2) FAIL, fit 12101.0 mm³ (E) (belly sled + pods / receiver) FAIL, bbox 85.7 L (E), extents 634 x 570 x 237 mm (E), mass 2491 g (E), CG 12.9 mm off thrust axis (E), 31.3 mm without swap parts FAIL`; latch set 66.8 x 68.8 x 179.6 mm (0.82 L, M); receiver FE crash worst 0.58 (35 g, E loads), handling 0.71, first mode ~355 Hz (E); XT60 float ±2.37° (C); set 185.9 g (M).

| Piece | Scout g (M) | Overseer g (M) | Extents in print pose, mm (M) | Bed mm² (M) |
|---|---|---|---|---|
| cartridge v5 | 32.09 | 89.7 | 40.0 x 45.6 x 96.0 / 55.0 x 58.6 x 174.0 | 1267 / 3125 |
| slider v5 (x 2) | 3.18 | 4.04 | 18.6 x 22.0 x 11.5 / 26.1 x 22.0 x 11.5 | 328 / 463 |
| receiver v1 | 40.80 | 76.12 | 61.2 x 54.1 x 101.6 / 66.8 x 68.8 x 179.6 | 837 / 1229 |
| receiver lid v1 | 5.34 | 7.85 | 61.2 x 20.0 x 5.5 / 63.6 x 20.0 x 7.2 | 1001 / 1049 |
| receiver strip v1 (TPU, x 2) | 0.98 | 2.07 | 4.8 x 70.6 x 2.4 / 4.8 x 148.6 x 2.4 | 339 / 713 |
| skid closed v1 (TPU, Scout) | 1.38 | – | 38.0 x 16.3 x 2.1 | 537 |
| skid lead v1 (TPU, Scout) | 3.06 | – | 38.0 x 34.2 x 2.1 | 1271 |

Checks (C unless tagged): insertion 50 mm every 1.0: 0 mm³, no stuck step; 0.3 off in x: receiver 0 mm³, the rigid-modelled XT60 male meets the plug 10.6 mm³ in the last 4 (its float carrier takes ±1.0); gripped pull-out 0 mm³; squeeze 0..4 0 mm³; asymmetric squeeze ±1.0 0 mm³, short bolt tip 0.7 inside the face; XT60 pitch: the float carrier gives ±2.37° (0.5 play over the 12.1 flange; PR 44's ±1.06° is roll), more than the arm's free mate 1.33 / 1.14°, so no guide posts (DEFAULT); option B (`GUIDE`, corner posts, end gap 0.15, truss windows) gives ±0.47 / ±0.38° at first contact and carries the arm's 3.5 / 6.1 N·m guide moment on the Overseer (0.49 / 0.54, +12 g), not on the Scout (0.85 / 0.97, +19.5 g, collar-clipped posts); hall 157 / 1.7 / 3.1 mT (latched / half in / squeezed, nominal, E model), ≥ 109 vs ≤ 16 at head offset ±0.5; side-hit slider hold x1.27 / x2.08 (E); collar planes 7.00 (Scout); receiver bolts up from below. Concept review FAIL x6 fixed, re-review PASS; PR review 1 FAIL fixed. Script wall time ~5 min (4 cores, M).

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
| `print/box_battery_skid_closed_scout_v1.stl` | `runs/battery_latch/v1/preview_box_battery_skid_closed_scout_v1.png` |
| `print/box_battery_skid_lead_scout_v1.stl` | `runs/battery_latch/v1/preview_box_battery_skid_lead_scout_v1.png` |
