# nox/topo demo summary (23 Oct 2026, branch topo/new-engine)

Cloud box: 4 cores, 16 GB. Self-test 12/12 green (127 s after the last engine edit). Both parts build as one watertight body
and pass the function check. Both **FAIL solid_check**, so neither is READY and nothing went to `print/`.

| Part | Mass g (M) | Worst index (case) | Solid check | Function | Iterations (coarse + fine) | Wall time | Peak GB | Solver | Baseline (result.json) |
|---|---|---|---|---|---|---|---|---|---|
| holder | 10.26 (drawn 2.95) | 0.50 (misuse 8) | FAIL: 1 body, 7 eroded bodies, 0 necks, 0 thin walls; 3 drawn bodies off the main body at 2.0 mm | ok | 178 (63 + 115), stopped on grey 2.0 % | 2530 s | 3.2 | Pardiso (direct) | noxopt v0 demo (h 4/3, vf 0.12): 10.4 g, worst 0.32 |
| motor_mount | 13.06 (drawn 4.47) | 0.11 (crash push 40 N +Y) | FAIL: 1 body, 1 eroded body, 0 necks, 0 thin walls; per-body test: pads 0.71, seat 0.50 in the main body | ok | 152 (55 + 97), stopped at max_stage, grey 3.6 % | 2577 s | 6.1 | Pardiso (direct) | none |

Previews: `runs/holder/topo-v1/preview.png`, `runs/motor_mount/topo-v1/preview.png`. Both are smooth, connected frames
with no specks or slivers.

## Why each part fails the gate

- **holder: a setup defect.** Even filled completely solid (envelope minus cuts), the holder fails solid_check: the pad sits in
  its slot on a 4 mm layer between two cuts, so its core leaves the main body at 2.0 mm erosion. The bolt bosses are 6 mm
  squares round a 3.2 mm hole, so their walls are 1.4 mm, under the 1.8 mm floor. Their outer corners are the 6 eroded
  specks of 3.4 mm3. The holder also ran before the gate-member fix (3.2 mm members); the design space only routes 4.44 mm.
  Fix: in `demos/holder.py`, make the bosses 7.5 mm square and open the cut under the pad. Not changed here, because the
  holder is the old engine's benchmark geometry.
- **motor_mount: it fails because of how the gate measures depth.** The solid_check docstring says each drawn body is eroded to
  0.4 x its own thickness. The code takes the depth from the finished part, where generated legs fuse onto the pads and
  the seat. That lifts the erosion from 1.41 to 1.56 mm (pads) and from 1.60 to 2.0 mm (seat, a 4 mm drawn disc that
  then has no core of its own). With the depth of the drawn body itself, every body sits fully in the main body (share
  1.000, checked with a diagnostic copy). Spark's call, best answer: measure the depth on the drawn body's own voxels in
  `nox/solid_check.py`. That file is outside nox/topo, so it is unchanged and the verdict stays FAIL.

## Engine fixes (all in nox/topo)

1. **Multigrid CG** (`fem.py`):
   - the dead-node mask had the wrong shape (every mg solve crashed);
   - the batched dot product was an invalid einsum;
   - Jacobi 0.6 diverges on 3D hex elasticity (lam_max of D^-1 K is 4.4 to 5.4), so the V-cycle was not a valid
     preconditioner and CG stalled at 300 iterations. Damping is now min(0.6, 4/(3 lam_max)) per level;
   - supports are eliminated exactly on the free dofs instead of by 1e4 springs. Error against Pardiso fell from 2e-2 to 7e-14.
2. **Calibration test** (`test_topo.py`): "plate = 1.0 R" holds for the 1D hat filter, not the 3D cone, whose plate is
   0.881 R. The code was right. The test now checks against a brute-force voxelised cone, within 0.03 R (stricter).
3. **Gradient test**: an FD step of 1e-5 sat on the roundoff floor of J (about 1e3). The step is now 1e-4; the analytic
   gradient was right and the 1e-3 threshold is unchanged.
4. **Neck test**: frozen and anchored modes cannot differ on a generated neck, because x_tilde is the same in both and
   the filter already erodes the neck. The test now compares the two modes on a 2 mm drawn stem (anchored 3.3e3, frozen
   2.7) and keeps the generated-neck check.
5. **V\* reset** (`layout.py`): the fine stage started with V\* = volfrac, below the value the coarse stage had adapted to,
   so the dilated limit read +69 % violated and MMA stripped the design (J 10 to 124). The same happened at every beta step. V\*
   is now set to its fixed point at every beta step and at the warm start.
6. **Minimum member from the solid gate** (`layout.gate_member_mm`, `run.py`): a 3.2 mm frame split into 72 pieces at
   the gate's 2.0 mm erosion round drawn bodies. optimise() now raises the member to 2 x gate depth + 1 mm (5.0 mm),
   capped at what the design space can reach, with a warning when it is capped.

Commits: `git log --grep "^topo:" topo/new-engine`.
