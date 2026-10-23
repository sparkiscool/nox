bracket_jA topo-v1: solid_check FAIL, not READY (nothing goes to print/)

Engine nox/topo, branch run/topo-bracket_jA, cloud box 4 cores / 16 GB. Setup: nox/box/stand/stand_v7_topo.py via topo/demos/stand.py, unchanged.

| Item | Value |
|---|---|
| Mass | 24.48 g (M), drawn 21.4 g, so generated about 3.1 g |
| Worst strength index | 0.317 in 'ring up + fore 30 N' (also 'ring up + aft 30 N'), across-layer, at [181.5, 139.5, -60.0] |
| Solid check | FAIL: 1 body, 1 eroded body (0.9 mm), 0 necks, 0 thin walls; drawn_not_in_main at erode 2.0: 0.0 share 0.0, 0.2 share 0.0, 0.3 share 0.0, 0.4 (wall plate) share 0.369 |
| Function check | ok (0 mm3 drawn missing, 0 in cuts, 0 outside envelope) |
| Iterations | 111 (38 coarse h 2.0 + 73 fine h 1.0), stopped at max_stage, beta 32, grey 3.1 % |
| Wall time | 748 s (layout 722 s, build 25 s) |
| Peak memory | 4.18 GB (M), 26 % of 16 GB |
| Solver | Pardiso direct, 282,417 active dofs (fine) |
| Min member | 5.0 mm (raised from 3.2 by gate_member), plates 4.54 mm, filter R 5.15 mm |
| Baseline (result.json) | noxopt v8, shipped 20 Oct 17:55: 38.4 g, worst 0.06, solid_ok then true, gen 12 cm3 |

Preview: preview.png. Four clamp pads, each on a short stub to the 4 mm wall plate. Smooth, no specks.
The pads are not tied to each other except through the plate.

## Why it fails (diagnosis, measured on bracket_jA.stl at 0.25 mm pitch)

Fully solid (envelope minus cuts) the setup PASSES solid_check, so the geometry is reachable. The fail comes
from the gate's 2.0 mm erosion against the 4.0 mm drawn wall plate (body 0.4, own half thickness 2.00 mm):

- At erosion 1.9 mm and below the part has one main body (others specks under 0.3 mm3), every pad core in it.
  At 2.0 mm the plate's core vanishes and the part splits into 4 lumps of 348 / 348 / 506 / 508 mm3, one per pad.
  So every load path between pads runs through the 4 mm plate; the optimiser added no generated member over 4 mm
  that bridges the pads directly.
- The gate picks the depth from the finished part: pads read 3.72 mm half thickness in the part (2.25 of their
  own), the plate 4.00 (2.00 of its own), so all clip to erode 2.0. With the drawn bodies' own depth
  (pads 1.8, plate 1.6) every core sits in one main body. This is the same gate issue as motor_mount topo-v1.
- Engine side: the run log shows "anchored cores outside the main body: 3" from beta 4 to the end, so the
  eroded design (eta 0.75, 1.37 mm) never joined the pads to each other either; gate_member raises generated
  members to 5.0 mm but cannot thicken the 4 mm drawn plate the gate erodes through.

Best fix (Spark's call, outside runs/): measure the per-body erosion depth on the drawn body's own voxels in
nox/solid_check.py; or, in the setup, make the wall plate 5 mm so it keeps a core at 2.0 mm.
