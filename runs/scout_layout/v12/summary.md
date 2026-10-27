# Scout v3 layout v3 / cage v2, race-car shape: the tail bay with its real mount (27 Oct 2026, concept)

**What changed from `v11/` (PR 133):** the Scout electronics mount session drew the tail bay mount for the v11 poses (PR 130, round 3; input copied as `electronics_envelope.json`, `SOURCE.txt`).
- The real bay is heavier than the 25 g placeholder:
  - mount 38.6 g (belly-mount tail block 17.3, carrier 21.3);
  - cables 6.4 g;
  - the XR2 placed.
- At v11's catch it read CG 5.6 / 6.3.

The rerun:
- **Bay (Spark: out of the prop wash):** the Radxa and UBEC keep their v11 poses relative to the receiver (they ride with the catch on the belly mount).
  - **UBEC:** at the electronics session's worst-size envelope (61 x 15.4 x 8, not measured), 7 further out in y (y 11.2..26.6) for 8 of air over the Radxa's SoC.
  - **XR2:** stays at its v11 place in the frame (x −110..−94, y −27..−15, z 23..45). Riding forward with the bay, its antenna entered the rear prop disk (67 mm³, system_check).
    - **Requirement for the mount:** the XR2 shelf moves 8.5 aft along the spine (the spine still spans x −110..−94 after the shift); the XR2 is 3.8 out of the wash there.
  - Wash, plan distance outside every disk (need ≥ 3): Radxa +9.5, UBEC +17.9, plug keep-clear +9.7, XR2 +3.8.
  - **The mount's own envelopes reach into the rear disks:** spine section (y −13.05..−4.6) −3.4, tail block −4.3. They were −3.3 at v11 already; these are bounding boxes.
    - **Requirement for the electronics mount:** its real spine and block stay ≥ 3 outside the rear disks (|y| < 6.6 between x −98.6 and −9.6 under the disks), or the part of them in the wash is a thin member only (rule 8).
- **Balance (Spark: the CG in the middle of the props):** the catch is free (scanned −16..+4). Picked: **catch 5.0 forward of the thrust centre** (was 3.5 aft).
  - CG 0.9 / 0.8 off the thrust axis, x and y (system_check); layout model 0.8 / 0.9.
  - This is the electronics session's "bay riding" case (their estimate catch −7.25, 1.25 / 1.17).
  - The "bay fixed in the frame" case was weighed (`alt_bay_fixed/`: `run.log`, `checks.txt`, `preview.png`; `SCOUT_BAY_FIXED=1`).
    - Picked: catch 3.0 forward, CG 1.3 / 2.9 (layout, x), 58 rods.
    - Further forward, the shell grows to 61 to 64 rods and the tail-node gate fails.
    - Out: pack-out at 2.9 sits on the 3.0 gate.
- **Keep-clears:**
  - The electronics session's item boxes and plug box are in the shell target.
  - Their tool lines (Ø10, 80 out) clear every rod: SD card +6.4, USB-C +7.4.
  - The mount's own boxes are bounding boxes and are not in the target: their lower aft corners reach where the real spine does not, which brought a low tail node back. The electronics session keeps the mount inside the shell and over the funnel line on its real geometry.
  - Their yoke and plate keys are not modelled here; the electronics session rechecks them against this shell.

Everything else as v11 (`v11/summary.md`): the bottom on the Overseer v5 loop's side nodes, the seat −49.62, the funnel band, the saddle ties, the camera on snap clips, nodes PETG with 3.2 sockets.

Script `nox/scout/v3/layout_v3.py` (now reading `SCOUT_ELEC`; v12 is the default output), view `layout_v3_view.py`.

Visuals:
- The bay-fixed alternative: `alt_bay_fixed/preview.png`.
- Whole drone: https://claude.ai/artifact/WHdjYkzKPifXuHT1dWeqoR (source `motion.html`). Still `preview.png`. Since 27 Oct both show the saved camera mount v1 (`runs/scout_camera_mount/v1/`, PR 146) in place of the layout's cradle, arms and clips; the assembly too (version 22, with the electronics round 5 masses: cage first 6.2, CG 0.6 / 1.5, 746 g E).
- Camera clips: https://claude.ai/artifact/3nsfgsnE6tmctThfu9HZqs (version 4; source `camera_clips.html`, still `camera_clips.png`).

System: Scout v3, cage first 6.2 mm (E) at +Z (camera) (field cones open: camera 55 deg, out 0 mm), shell 47 rods closed, h/w 0.31, fit 0.0 mm³ (E) (cartridge v6 (latch)/cartridge skid lead end (latch, TPU)), bbox 13.8 L (E), extents 390 x 278 x 127 mm (E), mass 746 of 500 g (149 %) (E) over budget (total), CG 0.6 mm off thrust axis (E) [+0.4, +0.4, -38.6], 1.5 mm without swap parts, probes 7.2 mm (E) edge at (-0.53, +0.03, -0.85) (UBEC), access 8/8 clear PASS. (On the assembly with the saved camera mount v1 and the electronics session's round 5 masses (concept), `system_check_mount.json`; the drawn 3.0 rods: the sized 6/4 tubes add +74 g E, not in this line.)

## Cage
- **Shell, 47 rods** (61..286, 6.13 m) on 18 nodes, offset 24.
  - Node extents 354 x 266 x 111; largest cell Ø133; rods 13.9 off the swept disks.
  - Offset scan: 16 → 56 rods, 20 → 53, 24 → 47, 28 → 47, 32 → 53. 24 is the most compact within the cap.
- **Saddle ties, 8 rods** (4 a side; v11 had 6: the 4th station at x 15 now finds a clear node), from the saddle face (y ±44.3, z −7):
  - x −45 to the waist node (6.6, ±125.0, −11.9), 96;
  - x −15 to (44.8, ±132.8, 44.2), 118;
  - x 15 to the waist node, 81;
  - x 38 to (122.6, ±62.2, 11.5), 88.
- **55 rods** in all (cap 62), every one 3.0 as drawn; 0 links in the funnel band.
- No node past x −130 about the catch is under 91 above the plane.
- **Camera clips:**
  - Top pair on the top front cross rod at (110.5, ±18.0, 61.1), arms 16.
  - Bottom pair on the front lower rods at (122.6, ±18.0, 11.5), arms 9.
- Swap column: receiver + 26.5 at the nose end, + 17.6 at the tail, + 30 at the sides; clear.
- Props shadow 3.6 % above, 6.7 % below (E).

## Strength (frame model v3 on v12 with the camera mount v1, `runs/scout_frame/v3/`)
- **Every rod 3.0 fails:** 25 of 55 buckle, 10 crash stresses and 19 handling stresses over 600 MPa, first mode 47 Hz (the camera) (F on E).
- **Sized** (`v3/alt_sized`): 39 members to 6/4 tube (+74 g E) pass every case at 810 g E (278 N crash), first mode 111 Hz (F; camera mount over 100). v2 read 138 Hz with the tubes weighed as 3.0 rods and the old 27 g cradle.
- **DEFAULT for the pieces:** the sized cage (below, Open). Spark can revert to 3.0 everywhere, which fails these cases.

## Carbon (C lengths, node centre to node centre)
- **As drawn, every rod 3.0 solid:** 55 (shell 47, saddle ties 8), 61..286, shortest 61 (over the 33 break-even, every one earns its carbon).
- **Sized (DEFAULT for the pieces): 39 x 6/4 tube**, 5543 mm in all: rod 7-15 61, 8-14 61, 4-11 75, saddle ties 3.1 81 (x2), 0-14 88, 3-15 88, saddle ties 4.1 88 (x2), saddle ties 1.1 96 (x2), 7-16 100, 8-16 100, 0-8 114, 3-7 114, saddle ties 2.1 118 (x2), 0-17 121, 3-17 121, 14-15 124, 4-10 129, 4-9 129, 1-12 144, 2-13 144, 10-12 147, 9-13 147, 0-1 155, 2-3 155, 10-11 175, 9-11 175, 7-8 193, 7-9 212, 8-10 212, 1-11 225, 2-11 225, 0-3 232, 1-2 239, 4-7 286, 4-8 286.
- **The other 16 stay 3.0 solid:** 67, 67, 68, 68, 71, 71, 77, 77, 80, 80, 93, 93, 106, 106, 116, 116 (1356 mm).
- **Printed struts:** none in the cage (the tie saddle and the camera mount are printed parts, not struts); no member under 33.

## Mass (E)
746 g (system_check, on the drawn 3.0 rods; the sized tubes add +74):
- cage 132 (shell rods 67, nodes 39, saddle ties 8, tie saddle 17);
- camera 10 + camera mount v1 21.8 + 2 collars 0.5 (M) = 32.3;
- the electronics session's tail bay mount 49.2 and cables 6.3 (round 5 concept);
- bought, latch set and wires 526.

## Open
- **Rods (DEFAULT for the pieces, 27 Oct):** the node and saddle sockets are sized for the frame model's 6/4 tube on its 39 members (6.2 bores), and 3.0 on the rest (3.2).
  - The general rod rule allows thicker where the frame model needs it; the 3.0-everywhere layout fails every crash case.
  - The layout drawing keeps 3.0 per the Scout clause and its check 13, until Spark rules.
  - The alternative, 3.0 everywhere, needs a different load path; none is drawn.
- **Electronics session:** the XR2 shelf 8.5 aft; recheck the yoke and plate keys against this shell; real UBEC size.
- **Design center:** rerun the funnel check (the catch moved 8.5 forward; the bottom loop moves with it).
