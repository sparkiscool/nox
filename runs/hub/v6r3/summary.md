# hub v6 run 3 (topo, arm v7 round 3d frame, detent v6): solid_check PASS, swap 0.52 and sustained 1.08 still MISS

solid_check reads PASS on the exact file runs/hub/v6r3/box_hub_v6.stl. Targets swap and sustained are not met; misuse is met.

- Command: cd nox/box && NOX_ARM=v7 python3 hub_v6_topo.py 2.05 52 ../../runs/hub/v6r3 (NOX_GIT unset), branch run/hub-v6r2 (PR 111). First attempt at h 2.05 completed, no OOM, no rerun at 2.15. Grid 96x40x96, 368,640 cells, design 90,703, drawn 12,313. Generated 52.0 cm3, ENV_OVERLAP 1.0, COL_FLARE on. No DEFAULT taken.
- Cases: 69 rows (29 swap/flight, 37 misuse/handling, 3 sustained) incl. detent springs; 24 tie cases.
- Mass M 199.3 g PETG (run 2: 184.4; budget 190 soft; the 15 g buys the plate overlap and the flared tower). Grey 0.9 %.
- Peak memory: engine layout peak 9.52 GB (run 2: 9.69); RSS of the main python process polled every 20 s peaked 11.1 GB (build phase, workers not summed). Wall 2895 s (layout 2270 s, build 609 s; 4 cores; run 2: 3671 s). Launcher PID 24693.
- Strength F (class max of in-layer / across index):
  - swap (flight, 29 cases): 0.523 worst, "pull at bay -60 50 N [Overseer] [swap]" at (37.4, 47.6, 22.1) (target <= 0.5: MISS by 0.02, was 0.596); next 0.501 "pull at bay 30 50 N [swap]" same place.
  - misuse / handling (37 cases): 0.466 worst, "hand on rail A tip 50 N [misuse]" at (49.7, 51.7, -18.9) (target <= 1.0: MET, was 0.509).
  - sustained (3 cases): 1.075, "detent springs (pivot 22 N, anchor 8.4 N)" at (21.0, 62.0, -25.1) (target <= 1.0: MISS by 0.08, was 1.329); next 0.228.
- System: box swap arm v7, assembled worst index 1.08 in detent springs (sustained) (was 1.33 run 2), fit function check ok (skin missing 0.0 mm3, in cuts 0, outside envelope 0), mass 199.3 g (M), CG n/a (the arm worker does the assembled recheck)

## solid_check
Exact output of python3 nox/solid_check.py runs/hub/v6r3/box_hub_v6.stl:
```
runs/hub/v6r3/box_hub_v6.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [107652.2], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 1, 'ok': True}
```
Note: the in-run check inside the script (result.json "solid") listed drawn_not_in_main [0.0, 0.997, erode 2.0] and ok false; the standalone gate on the file is the rule and reads PASS (plain bodies 1; run 2 had 16). The two checks differ on drawn part 0.0 (0.997 of it in the main body after a 2.0 erosion); flagged for the arm worker, mesh not patched.

## print_orient
```
runs/hub/v6r3/box_hub_v6.stl extents [170.6  66.9 169.3] g PETG 199.34 watertight False
   saved: bed     16 mm2  overhang  12964 mm2  height 169.3  first layer 1 islands [14.6] (total 14.6)  thin-island run 0 mm
```
Bed contact only 16 mm2 (render_preview: 3 mm2, overhang > 45 deg 9078 mm2, fits A1 mini 180: yes); the mesh is not on a flat bed face as it stands. Watertight NO. Preview preview.png; box_hub_v6_view.stl is the 120k-face viewing mesh; full mesh box_hub_v6.stl (24.5 MB) committed.

## What it says
The overlap and flare worked: one plain body, solid PASS, sustained 1.33 to 1.08, misuse 0.47. Swap 0.52 now sits at the bay pull case at (37.4, 47.6, 22.1), not under the wheel lanes; sustained is still the detent tower, moved up the column to (21.0, 62.0, -25.1), above the flare. Next: a drawn boss or wider column at Y 55..65 at the detent tower, and a pull-path rib at (37, 48, 22). Checkpoints (npz) are not committed.
