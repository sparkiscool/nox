# Scan stick handoff (23 Oct 2026, from the scan stick chat)

Branch `part/scan-stick-cam` (pushed, no PR yet). Main has the reviewed set from PR 22: base v17, head v13, cam v4, thread test v1, all READY.

## Open task: camera bracket v6 (Spark 23 Oct)
- Camera on **10 mm standoffs** so the plug and parts on the back of the board fit. Standoffs as thin as the gate allows, strong enough to press inserts and mount the camera.
- The camera takes **M2.5** (Spark: "I think"), not M2. Nothing takes M3 except the lidar. Check the bracket-to-head bolts too: they are M3 into the head's camera block, so either keep them (head v13 is on main) or move them to M2.5 with a head v14.
- The **UBEC is taped** to the shelf side. Q4 is closed.

## State of the bracket runs
- `cam_topo_v5.py`: posts Ø9 x 10 (C), Ø3.2 (E) through holes for M2.5 inserts, Ø12 x 4 foot pads. Gate PASS, 13.0 g (M), worst 0.31 (F). Not saved: the envelope reached the post tops, so the web wrapped the posts (r 8, M) and filled the space under the board. `runs/scan_stick_cam/v5/summary.md`.
- `cam_topo_v6.py`: envelope in the plate layer only, board footprint cut as a keep-out round the posts. Stopped at iteration 44: the design region is only 6.5 cm3 (M) against 5 cm3 (E) generated, so it stayed 75 % grey. Rerun at `0.75 2.5 --post-d 9` or lower. `runs/scan_stick_cam/v6/summary.md`.
- Gate facts found here: a blind insert hole leaves a solid post foot thicker than the plate, which fails the drawn-core erosion test; a through hole passes. Ø8 and Ø9 posts both pass the gate with a through hole (tested on drawn bodies plus a full plate). Ø9 leaves 2.9 mm wall round Ø3.2.

## Already changed on the branch (not on main)
- `nox/dimensions.md`: camera M2.5 on 28 x 28, M2.5 inserts (E: OD 3.5 x 5.7, hole Ø3.2); camera bracket row 44 x 60 x 14 with Ø9 x 10 posts; UBEC taped.
- `nox/parts-list.md`: M2.5 inserts, M2.5 x 6 camera screws, UBEC taped.
- `scan_stick.md` and `assembly_view.py` already point at `print/scan_stick_cam_v6.stl`, which does not exist yet. Do not merge before v6 is saved.

## To finish
1. Rerun `cam_topo_v6.py` with less generated volume, check the posts stay Ø9 on the mesh, `python3 nox/solid_check.py print/scan_stick_cam_v6.stl` PASS, `print_orient.py --all`, `render_preview.py`.
2. Summary in the fixed form, result.json keys, the cam row in `nox/current-parts.md` (v6 READY), one log line, `tools/runs_index.py`.
3. `xvfb-run -a python3 nox/overseer/scan_stick/assembly_view.py`: every fit line 0 mm3; republish the motion artifact.
4. PR with `tools/pr.py`, `review` agent, merge on PASS. Delete this file in that PR.

## Pole check (Spark asked)
The pole in Spark's screenshot is the one the base is built for: EXTEND-A-REACH 1.5-3 ft, universal twist-on metal tip, B08LL28XZW (bought 22 Oct). That tip is the standard 3/4"-5 ACME (spec, section S). The base has the matching female thread, 27 deep, 0.40 radial clearance (C). Not measured on the real pole: print `print/scan_stick_thread_test_v1.stl` first and screw it on. A light drag by hand is right; if tight, raise `acme_clear` by 0.1 and rerun the base.
