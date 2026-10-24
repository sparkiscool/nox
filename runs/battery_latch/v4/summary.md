# battery latch v4 (24 Oct 2026): Scout receiver v3 (head seats, bolt pattern, narrowed under the keel) + TPU end pieces, cartridge v6

Script `nox/box/battery_latch.py`; figures `battery_latch_figure.py` (`figs/`); receiver v3 FE `battery_latch_topo.py scout --swap-v2` (`fe/fe_scout_receiver_v2.json`, the Scout's v3 body); Overseer FE carried over from v2 (`fe/`). Doc `nox/box/battery-latch.md` ("Receiver v3 (Scout) and the end pieces"). Numbers `nox/dimensions.md` "System box swap: latch". Layout artifact: https://claude.ai/artifact/MX2dpRcjHegntjHvqp6U8x. Tags: M mesh, C CAD, E estimate, F frame model. Supersedes `runs/battery_latch/v3/`.

**What changed (Scout v3 system pass with the keel back, and the PR 109 review, 24 Oct):**
- **Receiver v3 (Scout):** head flats at y 21.5 (C), bolts at x ±26.55 with the tail pair at z −0.1 (C): every seat bears on its whole annulus with a full 3.0 ring (`head_seats`, C). M3 x 40 (DEFAULT, E). Clip inset 11.5 (collar 7.45, C). Drone |y| ≤ 25.5 below z −38 (C; the clip already held the columns inside 25.9, so this shaves the pocket blocks' outer 0.4). Keel nodes: receiver 1.2, end pieces 5.57, cartridge strips 2.48 clear (need 1.0, C).
- **Receiver TPU end pieces** (drone receiver only, DEFAULT): nose = plate + 2 Ø3.5 bars (C), bond 369 mm² (M); tail = wall and chamfer plate + 2 bars (C), bond 1197 mm² (M). Bars at drone y ±34, above z −40 (C). Receiver bottom strips dropped.
- Funnel margins at ±5° / ±2°: nose 1.21, tail 2.24, cartridge strips 3.15 (need 1.0, C). Insertion 0 mm³, no stuck step (C).
- 0.81 J through 2 mm of TPU: ~4 kN on a flat floor (E). The skids are a scrape layer; the hit goes through the Scout's suspension (the reason the keel is back).

**Scout set: HOLD** until the Scout's own pass (its cage, keel back, the new bolt pattern) reads fit 0 and probes PASS on these meshes. **Overseer set: DESIGN** (mount bolts open). Nothing in `print/`.

```
battery latch v4 (Scout set): HOLD
Latch manifest on its collar-band stand-in: System: battery_latch_scout, cage first 5.8 mm (E), fit 0.0 mm³ (E), bbox 3.75 L (E), extents 208 x 208 x 87 mm (E), mass 515 g (E), CG 0.9 mm off thrust axis (E), 1.8 mm without swap parts, probes 2.9 mm (E) (cartridge)
Old refit snapshot (not the judge; its cage predates the skids, the keel and the bolt pattern): System: Scout v3, cage first 4.5 mm (E) at (+0.00, +0.66, -0.75) (flow MTF-02P), fit 2.6 mm³ (E) (mount + nose legs (Scout)/receiver skid nose (latch)) FAIL, bbox 6.9 L (E), extents 233 x 233 x 128 mm (E), mass 812 g (E), CG 1.3 mm off thrust axis (E) [+1.3, +0.0, -38.3], 1.8 mm without swap parts, probes -27.9 mm (E) edge at (+0.96, -0.06, +0.26) (receiver v1 (latch)) FAIL, access 7/7 clear
Assembled worst index: Scout frame model ≤ 0.98 (F); latch parts ≤ 0.415 (M mesh, E loads, receiver v3 FE), sliders grip pull 0.365, rib 0.252 (E)
Weight: Scout set 165.3 g (M) (drone receiver with the end pieces; bay set 157.1 g; run v3 159.6, v1 91.0): the v4 grams buy the whole head seats, the tail end piece and the wider bolt pattern
Size: Scout set bbox 65.5 x 54.1 x 105.6 mm, largest 105.6 (M, fits the 180 bed)
First mode (seated pack): 546 Hz (≥ 100, E)
Carbon: no carbon rods, every piece printed
Solid: solid_check PASS on all 11 Scout files
```
Overseer: set 338.5 g (M); receiver v2, cartridge v6 rib (no skids); bbox 66.8 x 68.8 x 183.6 mm, largest 183.6 (M); first mode 458 Hz (E); latch parts ≤ 0.548 (M mesh, E loads), grip pull 0.398, rib carry 3 g 0.129 (E); system as `dimensions.md` Overseer fit (pose B: fit, CG, L2 FAIL, Overseer session). solid_check PASS on all 6 Overseer files.

| Pack | File | Material x count | g (M) | Extents in print pose, mm (M) | Bed mm² (M) | Bed face |
|---|---|---|---|---|---|---|
| Scout | box_battery_cartridge_scout_v6.stl | PETG x 1 | 37.92 | 40.0 x 45.6 x 96.0 | 1229 | closed end (z 0) down |
| Scout | box_battery_cart_strip_scout_v1.stl | TPU x 2 | 1.12 | 4.2 x 64.0 x 5.0 | 269 | foot down (one part, both sides) |
| Scout | box_battery_rskid_nose_scout_v2.stl | TPU x 1 | 3.20 | 71.5 x 35.8 x 25.5 | 856 | plate's outer face down, the bars on tree supports |
| Scout | box_battery_rskid_tail_scout_v1.stl | TPU x 1 | 5.00 | 71.5 x 35.8 x 17.6 | 1277 | plate's outer face down, the chamfer and bars on tree supports |
| Scout | box_battery_slider_scout_v6.stl | PETG x 2 | 2.88 | 17.6 x 22.0 x 11.5 | 309 | bar top (y 11.5) down: largest face, bar layers along its length |
| Scout | box_battery_receiver_scout_v3.stl | PETG x 1 | 94.39 | 65.5 x 105.6 x 53.9 | 1337 | mount plane (deck top) down: layers across the install axis |
| Scout | box_battery_xt60_carrier_scout_v1.stl | PETG x 1 | 4.76 | 63.1 x 20.0 x 7.0 | 834 | ear tops down (pocket open to the bed) |
| Scout | box_battery_receiver_lid_scout_v1.stl | PETG x 1 | 5.54 | 63.1 x 20.0 x 5.5 | 1039 | top face down (the XT60 carrier's cap, = v1 lid) |
| Scout | box_battery_receiver_strip_scout_v1.stl | TPU x 2 | 0.98 | 4.8 x 70.6 x 2.4 | 339 | flat |
| Scout | box_battery_skid_closed_scout_v1.stl | TPU x 1 | 1.38 | 38.0 x 16.3 x 2.1 | 537 | outer (chamfer) face down |
| Scout | box_battery_skid_lead_scout_v1.stl | TPU x 1 | 3.19 | 38.0 x 34.9 x 2.1 | 1325 | outer (chamfer) face down |
| Overseer | box_battery_cartridge_overseer_v6.stl | PETG x 1 | 97.56 | 55.0 x 58.6 x 174.0 | 3125 | closed end (z 0) down |
| Overseer | box_battery_slider_overseer_v6.stl | PETG x 2 | 3.73 | 25.1 x 22.0 x 11.5 | 444 | bar top (y 11.5) down: largest face, bar layers along its length |
| Overseer | box_battery_receiver_overseer_v2.stl | PETG x 1 | 216.24 | 66.8 x 183.6 x 68.6 | 2626 | mount plane (deck top) down: layers across the install axis |
| Overseer | box_battery_xt60_carrier_overseer_v1.stl | PETG x 1 | 4.95 | 65.6 x 20.0 x 7.0 | 884 | ear tops down (pocket open to the bed) |
| Overseer | box_battery_receiver_lid_overseer_v1.stl | PETG x 1 | 8.14 | 65.6 x 20.0 x 7.2 | 1089 | top face down (the XT60 carrier's cap, = v1 lid) |
| Overseer | box_battery_receiver_strip_overseer_v1.stl | TPU x 2 | 2.07 | 4.8 x 148.6 x 2.4 | 713 | flat |

solid_check per file (`log.txt`):
- box_battery_cartridge_scout_v6.stl PASS
- box_battery_cart_strip_scout_v1.stl PASS
- box_battery_rskid_nose_scout_v2.stl PASS
- box_battery_rskid_tail_scout_v1.stl PASS
- box_battery_slider_scout_v6.stl PASS
- box_battery_receiver_scout_v3.stl PASS
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
