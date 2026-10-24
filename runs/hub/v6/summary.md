# hub v6 (topo, arm v7 round 3d frame, detent v6 tower Q 21.92 -28.56): RESULT FAILED

solid_check FAIL on the exact file; mesh not patched. Not for print.

- Script `nox/box/hub_v6_topo.py 2.25 40 runs/hub/v6`, 82 system cases (+ detent springs) from `runs/arm/v7/system.json`. Branch `run/hub-v6`, merged with part/swap-arm-v7 (detent v6).
- DEFAULT: h 2.25 mm (brief said 1.5). h 1.5 and h 1.75 were OOM-killed in the fine stage (cgroup cap 14 GB; logs `log_h1.5_oom.txt`, `log_h1.75_oom.txt`); h 2.0 stopped at layout setup: "zero compliance ... detent springs: the load sits on fixed nodes" (`log_h2.0_fail.txt`). `log_h1.75_stale_ck.txt`: a stale h 1.5 checkpoint was resumed (my error, cleared). Grid 88x32x88, 248k cells.
- Mass M 188.6 g PETG (budget 190 soft). Grey 0.8 %. Peak memory 9.9 GB (sum of process RSS, polled every 30 s). Wall 3007 s (4 cores).
- Strength F (class max of in-layer / across index, 82 cases):
  - swap (flight, 33 cases): 0.561 worst, "insert at bay -210 60 N [swap]" (target <= 0.5: MISS).
  - misuse / handling (47 cases): 3.40 worst, "power cut carrying: head A falls to the park stop [Overseer] [misuse]" at (9.6, 30.3, 69.2) (target <= 1.0: MISS).
  - sustained (3 cases): 1.083, detent springs (pivot 22 N, anchor 8.4 N).
- System: assembled worst index 3.40 (misuse, power-cut park-stop), mass 188.6 g, fit: function check ok (drawn skins complete, nothing in cuts, nothing outside the envelope). The arm worker does the assembled recheck.

## solid_check
```
runs/hub/v6/box_hub_v6.stl FAIL {'bodies_plain': 13, 'bodies_eroded': 1, 'eroded_volumes_mm3': [99906.9], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [{'kind': 'wall', 'thick_mm': 1.0, 'core_mm3': 2.02, 'at': [15.5, 67.4, -24.5], 'span': [2.1, 1.5, 0.2]}], 'acute_edges': 0, 'ok': False}
```
(the in-run check also listed drawn_not_in_main [0.0, 0.995, erode 2.0]).

## print_orient
```
runs/hub/v6/box_hub_v6.stl extents [170.1  66.9 169.4] g PETG 188.6 watertight False
   saved: bed      0 mm2  overhang  12635 mm2  height 169.4  first layer 2 islands [0.1, 0.1] (total 0.1)  thin-island run 1 mm
```
Bed face +y (sleeve end), bed contact 0 mm2: the oriented mesh does not sit on a face. Preview `preview.png`.

## What it says
The h 2.25 grid is coarse against the 6.25 mm gate member, and the park-stop power-cut case (misuse 3.4) needs a load path the topology did not find at this resolution; a finer grid does not fit the 16 GB box with 83 cases. Next: fewer or merged cases (envelope the swap cases), or a drawn gusset at the park-stop load.
