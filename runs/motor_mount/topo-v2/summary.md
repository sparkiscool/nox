motor_mount topo-v2: NOT READY (solid_check FAIL, by 1-voxel specks)

Geometry v2: pads Ø13 x 5, seat 6 mm (nox/topo/demos/motor_mount.py @ 90b5afe). Engine nox/topo, unchanged.

| Item | Value |
|---|---|
| Mass | 16.46 g (M); drawn 7.72 g, generated 8.74 g. v1 was 13.06 g |
| Worst strength index | 0.075 (M) in crash push 40 N +X on the motor, across-layer, at (-10.0, -3.6, 26.6) under the seat. v1 0.11 |
| Solid check | FAIL: 1 body, 1 eroded body (7647.6 mm3), 0 necks, 0 thin walls; drawn_not_in_main: pad 0.0 share 0.998, pad 0.2 share 0.998 (both erode 2.0; limit 0.999) |
| Function check | ok (drawn missing 0, in cuts 0, outside envelope 0) |
| Iterations | 154: 55 coarse (h 1.6, 62 s) + 99 fine (h 0.8), stopped at max_stage, beta 32, grey 3.5 % |
| Wall time | 1928 s total (layout 1901 s, build 27 s) |
| Peak memory | 6.05 GB (M) of 16 GB |
| Solver | Pardiso direct, 324,843 active dofs on the fine grid |
| Min member | 5.0 mm (raised from 3.2 by the gate-member rule; plates 4.54 mm, filter R 5.15 mm) |
| Body | 1 watertight body, no dropped specks; mesh 258,404 faces |

Preview (`preview.png`): a clean 4-leg frame from the pads to the seat, smooth, no slivers. The seat and pads keep
their drawn shape.

## Why it fails

v2 fixed the v1 problem: the seat now passes (0 off-core voxels), and every pad core sits in the main body except
for a handful of single voxels. Diagnostic (`diag_drawn_cores.py`, read only, reruns the gate's per-body test):

| Pad | Core voxels at 2.0 mm | Off the main body | Share |
|---|---|---|---|
| 0 (+12, +12) | 2369 | 4 | 0.998 FAIL |
| 1 (+12, -12) | 2377 | 1 | 0.9996 |
| 2 (-12, +12) | 2386 | 4 | 0.998 FAIL |
| 3 (-12, -12) | 2393 | 2 | 0.9992 |
| seat | 49526 | 0 | 1.000 |

Every off voxel is a lone 0.25 mm voxel (0.016 mm3) with depth 2.02 mm against a 2.00 mm cut, at z 3.8 to 4.6,
on the ring round the key access cut where a leg meets the pad top.

Cause: **setup geometry, marginal.** A 5 mm pad has its deepest point at 2.51 mm (half its height), so the gate erodes
it at clip(0.8 x 2.51) = 2.0 mm and leaves a core only about 1 mm thick. Where the generated leg's fillet meets the pad
top, the distance field rises and falls by a voxel, and single voxels just over 2.0 mm separate from the core by face
connectivity. 4 of about 2370 is enough to drop the share under 0.999. The design itself is sound; the margin is not.

A gate side note (not a fix here): the per-body share test in `nox/solid_check.py` counts sub-mm3 specks that the
body count ignores (min_cc_mm3 = 1.0). That is a voxel artifact, not a disconnected structure.

## Best fix

Setup: pads Ø13 x 6 (PAD_T 6.0 in `nox/topo/demos/motor_mount.py`). Depth then ~3.0 mm, gate erosion stays 2.0 mm,
core margin ~2 mm thick instead of ~1 mm, so speck voxels cannot split off. Cost about +0.5 g. Rerun as topo-v3.
Alternative (Spark's call, gate change): apply min_cc_mm3 to the per-body share test in `nox/solid_check.py`.
