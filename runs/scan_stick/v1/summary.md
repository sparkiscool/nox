scan_stick v1: ready to print
Weight: head 86.8 g, thread test 8.3 g, cam 10.0 g, cradle 194.7 g (M), was none (first version)
Strength: no FE on the drawn v1 (hand-held tool, loads E: 230 g lidar at 25 g knocks); the v2 topology run of the head (`head_topo_run.py`) rechecks every case
Solid: solid_check PASS on all four files (1 body, 1 eroded body, 0 necks, 0 thin walls): head `print/scan_stick_head_v1.stl`, thread test `print/scan_stick_thread_test_v1.stl`, cam `print/scan_stick_cam_v1.stl`, cradle `print/scan_stick_cradle_v1.stl`
Print: head lidar-seat face down, bed contact 6440 mm² (M), boss up, tree supports 0 g (E: only the thread crests overhang); thread test flat, 543 mm² (M), 0 g supports (E); cam flat, 2418 mm² (M), 0 g (E); cradle bottom face (ledge + hub + plate feet), 4257 mm² (M), tree supports ~8 g (E) under the Pi standoffs and clamp-bolt holes; the two larger cradle faces carry the standoffs and the ledge so they cannot be the bed face; files above

Design: `nox/overseer/scan_stick/scan_stick.scad` (OpenSCAD, parametric: rod_d, acme_clear, lidar_facing). Doc `scan_stick.md`.
DEFAULT: lidar up-facing (Overseer L2 top-mounted, section O); pole lower tube OD 25.4 (E) until measured; 12 V (spec) 2 A (E) buck for the L2; 25 g misuse loads (E). Review fixes: head rear notch removed (was a Mid-360 leftover), +1.5 g (M).
Open: pole lower tube OD (P), camera lens / board thickness (Q3), UBEC and buck bodies (Q4).
Previews: preview.png (head), preview_cam.png, preview_cradle.png, preview_thread_test.png, assembly_up.png, assembly_down.png (OpenSCAD).
