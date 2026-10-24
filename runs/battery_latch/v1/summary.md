# battery latch v1 (24 Oct 2026): cartridge v5 (Scout, Overseer), receiver latch block v1, sliders

Script `nox/box/battery_latch.py` (+ `battery_latch_topo.py --fe-v1` for the receiver FE, `battery_latch_figure.py`). Doc `nox/box/battery-latch.md`. Numbers `nox/dimensions.md` "System box swap: latch". Drawn parts (the latch module and its locating faces are must-haves); receiver topology candidates run separately (`runs/battery_latch_receiver_topo/`).

```
battery latch v1: ready to print
System: Scout v3 (pass 1 snapshot, XT60 end at the nose), cage first −4.2 mm at the tail-left bottom (Scout cage: missing keel node; +5.0 with it), fit 2194 mm³ vs the PR 44 pogo carrier (Scout belly frame), collar planes 7.00, mass 689 g (E), CG 2.0 off axis (3.0 pack out: Scout dry-mass rebalance)
Weight: Scout set 87.6 g (M), was 122.3 (PR 44); Overseer set 186.4 g (M), new
Strength: receiver FE crash worst 0.16 (Scout, 15.3 g suspension cap) / 0.48 (Overseer, 35 g), handling 50 N on a foot 0.67 / 0.69; latch hand calc worst 0.35 / 0.39 (grip pull with the XT60)
Solid: solid_check PASS on all 10 files
Print (each saved on its largest flat face): cartridge closed end down (bed 1267 / 3125 mm²), receiver closed-end face down (837 / 1251 mm²), lid top down, slider bar top down (328 / 463 mm², largest face; bar layers along its length), strips flat (TPU); tree supports; files print/box_battery_*
```

| Piece | Scout g (M) | Overseer g (M) | Extents (print pose) | Bed mm² |
|---|---|---|---|---|
| cartridge v5 | 33.06 | 89.70 | 40 x 45.6 x 96 / 55 x 58.6 x 174 | 1267 / 3125 |
| slider v5 (x 2) | 3.18 | 4.04 | 18.6 / 26.1 x 22 x 11.5 | 328 / 463 |
| receiver v1 | 40.87 | 76.58 | 61.2 x 53.9 x 101.6 / 66.8 x 68.6 x 179.6 | 837 / 1229 |
| receiver lid v1 | 5.34 | 7.85 | 61.2 x 20 x 5.5 / 63.6 x 20 x 7.2 | 1001 / 1049 |
| receiver strip v1 (TPU, x 2) | 0.98 | 2.07 | 4.8 x 70.6 x 2.4 / 148.6 | 339 / 713 |

Checks: insertion 50 mm every 1.0 (and 0.3 off) 0 mm³, no stuck step; gripped pull-out 0 mm³; squeeze 0..4 0 mm³; asymmetric squeeze ±1.0 0 mm³, short bolt tip 0.7 inside the face; hall 96.7 / 7.2 / 1.0 mT (latched / half in / squeezed); side-hit slider hold x1.27 / x2.07. Concept review: FAIL with 6 findings, all fixed (doc). Wall time 8.5 min (4 cores).

## Previews (rendered from the exact print files)
| Print file | Preview |
|---|---|
| `print/box_battery_cartridge_overseer_v5.stl` | `runs/battery_latch/v1/preview_box_battery_cartridge_overseer_v5.png` |
| `print/box_battery_cartridge_scout_v5.stl` | `runs/battery_latch/v1/preview_box_battery_cartridge_scout_v5.png` |
| `print/box_battery_receiver_lid_overseer_v1.stl` | `runs/battery_latch/v1/preview_box_battery_receiver_lid_overseer_v1.png` |
| `print/box_battery_receiver_lid_scout_v1.stl` | `runs/battery_latch/v1/preview_box_battery_receiver_lid_scout_v1.png` |
| `print/box_battery_receiver_overseer_v1.stl` | `runs/battery_latch/v1/preview_box_battery_receiver_overseer_v1.png` |
| `print/box_battery_receiver_scout_v1.stl` | `runs/battery_latch/v1/preview_box_battery_receiver_scout_v1.png` |
| `print/box_battery_receiver_strip_overseer_v1.stl` | `runs/battery_latch/v1/preview_box_battery_receiver_strip_overseer_v1.png` |
| `print/box_battery_receiver_strip_scout_v1.stl` | `runs/battery_latch/v1/preview_box_battery_receiver_strip_scout_v1.png` |
| `print/box_battery_slider_overseer_v5.stl` | `runs/battery_latch/v1/preview_box_battery_slider_overseer_v5.png` |
| `print/box_battery_slider_scout_v5.stl` | `runs/battery_latch/v1/preview_box_battery_slider_scout_v5.png` |
