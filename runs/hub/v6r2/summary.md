# hub v6 run 2 (topo, arm v7 round 3d frame, detent v6): RESULT FAILED

solid_check FAIL on the exact file; mesh not patched. Not for print.

- Command `cd nox/box && NOX_ARM=v7 NOX_GIT=1 python3 hub_v6_topo.py 2.05 40 ../../runs/hub/v6r2`, branch `run/hub-v6r2`. First attempt at h 2.05 completed (no OOM, no rerun at 2.15). Grid 96x40x96, 368,640 cells, design 90,703, drawn 12,290. No DEFAULT taken.
- Cases: 69 rows (29 swap/flight, 37 misuse/handling, 3 sustained) incl. detent springs; unbraked power cuts out, swap cases enveloped at 15 %; 24 tie cases.
- Mass M 184.4 g PETG (budget 190 soft). Grey 0.9 %. Peak memory 9.69 GB (engine) / 9.55 GB (sum of process RSS polled every 20 s). Wall 3671 s (layout 3078 s, build 593 s; 4 cores). PID 16262 family.
- Strength F (class max of in-layer / across index):
  - swap (flight, 29 cases): 0.596 worst, "B insert at bay 30 60 N [swap]" at (-67.2, 49.7, -29.2) (target <= 0.5: MISS, was 0.561).
  - misuse / handling (37 cases): 0.509 worst, "dropped pack [Overseer] [misuse]" at (-7.7, 47.6, 61.0) (target <= 1.0: MET, was 3.40).
  - sustained (3 cases): 1.329, "detent springs (pivot 22 N, anchor 8.4 N)" at (25.1, 55.8, -25.1) (target <= 1.0: MISS, was 1.083).
- System: box swap arm v7, assembled worst index 1.33 in detent springs (sustained) (was 3.40 run 1), fit function check ok (skin missing 0.0 mm3, in cuts 0, outside envelope 0), mass 184.4 g (M), CG n/a (the arm worker does the assembled recheck)

## solid_check
```
runs/hub/v6r2/box_hub_v6.stl FAIL {'bodies_plain': 16, 'bodies_eroded': 1, 'eroded_volumes_mm3': [96603.5], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 1, 'ok': False}
```
The in-run check also listed drawn_not_in_main [0.0, 0.997, erode 2.0]. Failing items: 16 plain bodies (1 after erosion) and 1 acute edge. The thin wall of run 1 is gone.

## print_orient
```
runs/hub/v6r2/box_hub_v6.stl extents [170.1  66.9 169.5] g PETG 184.37 watertight False
   saved: bed      0 mm2  overhang  12807 mm2  height 169.5  first layer 2 islands [0.0, 0.1] (total 0.1)  thin-island run 0 mm
```
Bed contact 0 mm2: the mesh does not sit on a face. Preview `preview.png`; committed mesh `box_hub_v6_view.stl` (120k faces, viewing only). Full mesh (22 MB) kept on disk at `runs/hub/v6r2/box_hub_v6.stl`, not committed.

## What it says
Dropping the unbraked power cuts fixes misuse (3.40 to 0.51). Swap 0.596 and sustained 1.33 (detent anchor at the tower) still miss; solid_check fails on 16 loose plain bodies and one acute edge. Next: a drawn gusset or boss at the detent anchor (25.1, 55.8, -25.1), and the insert-at-bay load path; do not reopen the park-stop case.
