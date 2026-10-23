# nox/topo development round 2 (23 Oct 2026, branch topo/new-engine)

Cloud box: 4 cores, 16 GB. Self-test 15 / 15 green (`cd nox && python3 -m topo.test_topo`, 148 s, after the merge of
Spark's organic finish and change 9). Gate `nox/solid_check.py` and every setup under `nox/box` unchanged.
All round 2 results below use `finish="drawn"` (bracket_jA topo-v4 with `--set finish=drawn` after the organic default
landed from Spark's session; the others ran before it). The organic runs are that session's.

## Result

| Part | Run | Mass g (M) | Worst index, same grid: new | old v8 | Gate | Wall time | Peak GB |
|---|---|---|---|---|---|---|---|
| bracket_jA | topo-v4 | 36.02 (v8 38.37) | **0.124** ('ring -Z 30 N') | 0.133 ('ring up + fore 30 N') | **PASS** | 615 s | 2.48 |
| bar_half | topo-v3 (= v2 layout, cavity kept) | 71.23 (v8 77.58) | **0.804** ('box on one tip 50 N up') | 0.929 (same case, same spot) | **PASS** | 1070 s + 60 s rebuild | 4.60 |
| motor_mount | topo-v3 (6 mm pads) | 17.21 | 0.117 ('crash push 40 N +Y') | none | **PASS** | 1761 s | 6.00 |

Both targets met: lighter than v8 and a lower worst index on the same Part grid and load cases (`topo.compare`,
`runs/<part>/<run>/compare.json`). Intermediate runs: bracket_jA topo-v2 (35.87 g, 0.119, gate FAIL, the reason for
gate ties), bracket_jA topo-v3 (36.04 g, 0.125, PASS, gate ties before change 9) and bar_half topo-v2 (76.06 g, 0.804, PASS, sealed cavity filled).

Not run: holder (fails the gate even filled solid: setup), bracket_e (its coarse stage now builds; a 1 M cell fine grid,
hours on this box).

## Changes, in commit order

1. **Drawn-masked eroded filter** (default `erosion="masked"`). Eroded field = conv(x on design) / max(1 - conv(drawn),
   0.05); drawn cells 1, loads in place; exact transpose with the 1/den weight (FD to 1e-6). Anchored erosion is an
   option, off. Why: anchored erosion isolated thin drawn plates next to cuts (jA topo-v1: 3 bodies off from beta 8, tie
   ratio ~1e5). Test: 2 mm neck along a cut vs full-width root, eroded compliance ratio 3.2e3 (> 50).
2. **Part.elems fallback.** A region with no cell centre uses the cells it intersects (3 x 3 x 3 samples), then the part
   cells touching those. bracket_e's 1.5 mm support band at h 2.0 now gets 1378 cells (it crashed).
3. **Geometric constraints off by default, now satisfiable.** bar_half topo-v1 +294: all 977 counted cells were at
   x = 1, 1 to 2 cells off a cut, where the constant-normalised filter cannot reach eta_e. jA +6: 97 % next to drawn
   bodies. Both held constant for 25+ iterations. Reachability masks fix that (same density: 0.002 eps solid), which
   also shows the robust formulation already holds the solid length scale. On with `--set geometric=true`.
4. **jA build volume loss: the voxeliser.** Rays started below the grid, inside the stand's ring cut (r 180) and BIG
   half spaces, so whole columns read inverted: 43,352 design cells of jA's 64,808 were inside the ring. The optimiser
   put 10.9 of its 12 cm3 there, and the exact build cut it off. Rays now start below the mesh. jA design space is
   21.5 cm3; bar_half and motor_mount masks unchanged; bracket_e 662 -> 515 cm3.
5. **compare.py.** Old print file to W by the inverse print-pose rotation (bar_half 45.12 deg) and a translation fitted
   on the drawn bodies; both v8 parts fit with 0.0 mm3 drawn missing, in cuts, outside the envelope.
6. **motor_mount pads 6 mm tall.** v2 failed by 4 voxels of 2370; v3 passes.
7. **Gate ties** (`gate_ties=True`). jA topo-v2 failed only because its two arms joined through the 4 mm drawn flange,
   which the gate's 2.0 mm erosion removes where nothing covers it (two 2.0 mm components, 3903 / 3890 mm3). A held
   drawn body with half thickness under 2.0 mm now counts in the eroded design by its cover (density of the design
   next to it), and a second tie set holds the largest unheld body. Test: legs-only gate tie ratio 5.7e5 vs 15 with a
   bridge; cover gradient exact. jA topo-v3 grew one thick beam between the arms and passes.
8. **Build keeps sealed cavities; `run --rebuild-from`.** bar_half's nominal field is a hollow box beam (cavity 3.8 cm3,
   72 x 6 x 13 mm). The speck filter read the inverted shell as a negative speck and filled it: +4.8 g at the same
   worst index. topo-v3 is the v2 layout rebuilt with the cavity.
9. **Gate ties on their own realization.** After change 7 the holder end-to-end smoke test built 2 bodies: crediting a
   thin held bolt boss by its cover in every case let the optimiser drop it (183 mm3 loose). Now only the gate ties see
   the cover (one extra solve group); real cases and ordinary ties see the body whole. Smoke test one body; jA rerun
   as topo-v4 (36.02 g, 0.124, PASS). bar_half and motor_mount have no gate ties and are unchanged.

Engine defaults otherwise unchanged (p-mean 6, tie weight 0.5, continuation, volume gain 0.3, gate member 5.0 mm):
no measured reason to change them once 1 to 8 were in.

## Notes

- Runs share the branch with Spark's session. A merge while a run is writing replaced motor_mount topo-v3's log.txt
  under the running process (lines after fine iteration 61 lost; result.json complete). Merge between runs, not during.
- The worst spot of bar_half (new and old) is the beam root at the lap end, so the index there is set mostly by how
  fully the root fills the envelope.
