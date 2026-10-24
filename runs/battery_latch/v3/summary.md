# battery latch v3 (24 Oct 2026): cartridge v6 (centre rib) + the Scout TPU edge skids, on receiver v2

Script `nox/box/battery_latch.py`; figures `battery_latch_figure.py` (`figs/`); FE reused from `runs/battery_latch/v2/` (receiver body unchanged, copied into `fe/`). Doc `nox/box/battery-latch.md` ("Cartridge v6 and the Scout skids"). Numbers `nox/dimensions.md` "System box swap: latch". Layout artifact: https://claude.ai/artifact/MX2dpRcjHegntjHvqp6U8x (sections with the rib and the skids). Tags: M mesh, C CAD, E estimate, F frame model. Supersedes `runs/battery_latch/v2/` (the cartridge v5 there).

**What changed:**
- **Cartridge v6 + slider v6, centre rib** (heads v7 ask, DEFAULT): rib x ±2.0, y 0..11.5, notched for the spring, held by two 6.0 cross walls frame to frame (C). Tabs at rest ±5.0 / ±10.0, squeezed ±2.0 / ±7.0 on the rib, stroke 3.0 (C). Both bolts retract 3.0 at any head offset ±1.0 (need 2.6, C). Release 2.6 / 4.1 N per finger (E). Rib index 0.25 at the 43.5 N across (E).
- **Scout TPU edge skids** (Scout v3 ask, cage role): cartridge strips (2), receiver nose plate, receiver side skids (Ø3.5 round bar + strip, 2). Receiver skids clipped at funnel inset 5.25: margins 1.16 / 1.86 / 3.15 (need 1.0, C); bolt key lines 13.5 / 5.58 / 10.1 (C). Receiver skids on the drone receiver only (DEFAULT).
- Insertion with every strip and skid: 0 mm³, no stuck step (C).

**Scout set: HOLD** until the Scout's own pass loads these meshes and reads fit 0 and probes PASS (sent to the Scout session 24 Oct). **Overseer set: DESIGN** (mount bolts open). Nothing in `print/`.

```
battery latch v3 (Scout set): HOLD
System: Scout v3, cage first 4.5 mm (E) at (+0.00, +0.66, -0.75) (flow MTF-02P), fit 44.5 mm³ (E) (cage (rods + nodes)/receiver skid side px (latch)) FAIL, bbox 6.9 L (E), extents 233 x 233 x 128 mm (E), mass 806 g (E), CG 1.6 mm off thrust axis (E) [+1.6, +0.0, -38.2], 2.2 mm without swap parts, probes -44.4 mm (E) edge at (-0.78, -0.39, +0.49) (receiver v1 (latch)) FAIL, access 7/7 clear
  (old refit snapshot 3595381: its cage predates the skids and the Scout's new layout; collar planes 7.04 (C))
Assembled worst index: Scout frame model ≤ 0.98 (F); latch parts ≤ 0.512 (M mesh, E loads, receiver v2 FE), sliders grip pull 0.365, rib 0.252 (E)
Weight: Scout set 159.6 g (M) (drone receiver with skids; bay set 155.7 g; v1 91.0)
Size: Scout set bbox 63.6 x 54.1 x 105.6 mm, largest 105.6 (C, fits the 180 bed)
First mode (seated pack): 552 Hz (≥ 100, E)
Carbon: no carbon rods, every piece printed
Solid: solid_check PASS on all 12 Scout files
```
Overseer: set 338.5 g (M); cartridge v6 rib only (no skids); bbox 66.8 x 68.8 x 183.6 mm, largest 183.6 (C); first mode 458 Hz (E); latch parts ≤ 0.548 (M mesh, E loads), grip pull 0.398, rib carry 3 g 0.129 (E); system as `dimensions.md` Overseer fit (pose B: fit, CG, L2 FAIL, Overseer session). solid_check PASS on all 6 Overseer files.

| Pack | File | Material x count | g (M) | Extents in print pose, mm (M) | Bed mm² (M) | Bed face |
|---|---|---|---|---|---|---|
| Scout | box_battery_cartridge_scout_v6.stl | PETG x 1 | 38.19 | 40.0 x 45.6 x 96.0 | 1248 | closed end (z 0) down |
| Scout | box_battery_cart_strip_scout_v1.stl | TPU x 2 | 1.12 | 4.2 x 64.0 x 5.0 | 269 | foot down (one part, both sides) |
| Scout | box_battery_rskid_plate_scout_v1.stl | TPU x 1 | 2.24 | 54.0 x 16.0 x 2.2 | 864 | flat, wall face down |
| Scout | box_battery_rskid_side_px_scout_v1.stl | TPU x 1 | 0.84 | 8.7 x 88.8 x 29.7 | 154 | strip foot down, the nose bar on tree supports |
| Scout | box_battery_rskid_side_nx_scout_v1.stl | TPU x 1 | 0.84 | 8.7 x 88.8 x 29.7 | 154 | strip foot down, the nose bar on tree supports (mirror of px) |
| Scout | box_battery_slider_scout_v6.stl | PETG x 2 | 2.88 | 17.6 x 22.0 x 11.5 | 309 | bar top (y 11.5) down: largest face, bar layers along its length |
| Scout | box_battery_receiver_scout_v2.stl | PETG x 1 | 93.10 | 63.6 x 105.6 x 53.9 | 1318 | mount plane (deck top) down: layers across the install axis |
| Scout | box_battery_xt60_carrier_scout_v1.stl | PETG x 1 | 4.62 | 61.2 x 20.0 x 7.0 | 796 | ear tops down (pocket open to the bed) |
| Scout | box_battery_receiver_lid_scout_v1.stl | PETG x 1 | 5.34 | 61.2 x 20.0 x 5.5 | 1001 | top face down (the XT60 carrier's cap, = v1 lid) |
| Scout | box_battery_receiver_strip_scout_v1.stl | TPU x 2 | 0.98 | 4.8 x 70.6 x 2.4 | 339 | flat |
| Scout | box_battery_skid_closed_scout_v1.stl | TPU x 1 | 1.38 | 38.0 x 16.3 x 2.1 | 537 | outer (chamfer) face down |
| Scout | box_battery_skid_lead_scout_v1.stl | TPU x 1 | 3.12 | 38.0 x 34.2 x 2.1 | 1298 | outer (chamfer) face down |
| Overseer | box_battery_cartridge_overseer_v6.stl | PETG x 1 | 97.56 | 55.0 x 58.6 x 174.0 | 3125 | closed end (z 0) down |
| Overseer | box_battery_slider_overseer_v6.stl | PETG x 2 | 3.73 | 25.1 x 22.0 x 11.5 | 444 | bar top (y 11.5) down: largest face, bar layers along its length |
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
- box_battery_slider_scout_v6.stl PASS
- box_battery_receiver_scout_v2.stl PASS
- box_battery_xt60_carrier_scout_v1.stl PASS
- box_battery_receiver_lid_scout_v1.stl PASS
- box_battery_receiver_strip_scout_v1.stl PASS
- box_battery_skid_closed_scout_v1.stl PASS
- box_battery_skid_lead_scout_v1.stl PASS
- box_battery_cartridge_overseer_v6.stl PASS
- box_battery_slider_overseer_v6.stl PASS
- box_battery_receiver_overseer_v2.stl PASS
- box_battery_xt60_carrier_overseer_v1.stl PASS
- box_battery_receiver_lid_overseer_v1.stl PASS
- box_battery_receiver_strip_overseer_v1.stl PASS
