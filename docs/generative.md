# Generative parts, heavy runs, save and print

Every generated part runs the whole `nox/noxopt_demo.py` pipeline, in order, nothing skipped. A part script is a copy of the demo with the part's own loads, drawn features, cuts and envelope. Layout and smooth build: `noxopt.py` (`noxopt_local.py` with `backend="gmg"` above ~60k design cells or ~450k dof). FE kernel and strength checker: `printopt.py`. Tags: T measured in a cloud run, E estimate.

## The rule
1. **Drawn = mounting + must-haves only:** fastener seats with their boss walls, mating and contact faces, pin bores, rod sockets, connector seats, flexures, hooks, windows, TPU feet, each at its smallest volume. Everything else is generated. Every drawn body lies inside the envelope and the design space can reach it over its whole length. Access cuts for bolt heads and tools are never wider than the pad they serve.
2. **Layout:** stiffness run to a mass target, every load case plus a tie load on every drawn feature (noxopt adds those). Members come out at the filter radius. Symmetric mounting gives a symmetric part (auto-detected; centre the grid on the mirror plane). Pick the bed face first: the drawn mounting pads, usually.
3. **Stop** at ≤ 2 % grey and settled. Hard cap 6 h, then build from the last checkpoint. Too big means coarser or in pieces.
4. **No thin rods:** `prune_thin` after the layout; `thin_check` reads 0 after the build. A thin member goes back to `prune_thin`, never thickened by hand.
5. **Build:** `smooth_surface` polish (fillets where members meet, holes stay holes, nothing under 2.0, off every keep-out), then exact drawn bodies, envelope trim, exact cuts. `function_check` reads 0 / 0 / 0. Report `wrinkle` and look at close-ups; lumpy means a finer layout, not more flow.
6. **No patching:** no grey fill, dilation, member growing or "keep the largest body". A detached feature means rerun. `solid_check` PASS or the part does not exist.
7. **Strength:** as-built recheck of every case with `printopt.py`. A must-hold case over 1.0 gets `smooth_surface(..., reinforce=points)`, at most 3 passes. A topology part replaces a drawn one only if it beats the drawn part's worst index on the same FE grid and passes the solid check; the FE grid is at most a third of the smallest neck it judges.

## How-to
Copy `noxopt_demo.py`; swap in the part's `printopt.Problem`, drawn bodies, cuts, envelope and mirror plane; keep every step. Worked parts: `nox/box/arm_topo.py` + `arm_run_*.py`, `nox/box/stand/stand_v7_topo.py` + `stand_v7_topo_build.py`.
```python
import noxopt as NO
L = NO.Layout(pb, volfrac)                                   # pb.R filter radius in cells; pb.sym, pb.overhang as before
dens = L.run(checkpoint="ck.npz", resume="ck.npz")
L, dens, removed = NO.prune_thin(L, dens, floor_mm=2.0, margin_mm=0.75)
NO.connectivity(dens, pb)                                    # one body, every drawn body in it
part = NO.smooth_surface(rho, solid, void, shape, h, origin, keep_mesh=drawn, clip_mesh=allowed, floor_mm=2.0)
NO.function_check(part, drawn, cuts, allowed)               # 0 / 0 / 0
NO.thin_check(part, drawn, floor_mm=2.0)                     # 0 members
NO.wrinkle(part, drawn=drawn, allowed=allowed)              # p95 / p99, reported
```
Then the as-built recheck (pattern: end of `noxopt_demo.py`, `nox/box/arm_recheck.py`), `solid_check.py`, `print_orient.py`, `render_preview.py`.

| Default | Value |
|---|---|
| Grid | h 1.0 or finer; small parts 0.5; members at least 2 cells wide |
| Filter radius `pb.R` | 2.0 cells, and at least wall class / h |
| Robust thresholds | 0.7 / 0.5 / 0.3 |
| Case weighting | p-mean, P 4; tie cases x 0.25 |
| Mass target | generated volume from the part's budget (`GEN_CM3`); lower it while the as-built worst index stays ≤ 1.0 |
| Overhang filter | off (tree supports are allowed everywhere); on only for a part that prints flat with no supports |
| Smooth build | fine grid h / 4, sigma ~0.6 x h, volume preserved; polish on, `floor_mm` 2.0 (2.4 TPU), `clip_margin` 0.5 |
| Envelope | faces that are not real restrictions go out until the layout stops touching them; skin 2.0 from each face void; report any sliced flat area |

Known limits (T): the layout is a stiffness design, strength is fixed locally; a one-voxel peak with p99.9 well under 1.0 is a flag, not a failure; waves along a long member or a crease between two close members mean a finer layout (gripper head v3 at h 1.32 came out smoother and 9 % lighter than h 2.0). Engine edit: `python3 nox/test_noxopt.py` once, commit, one `log.md` line.

## Heavy runs (cloud box)
- Branch `run/<part>-<version>`, one run per box. Start in the background (`nohup python3 <script> > runs/<part>/<version>/log.txt 2>&1 &`): a run lives only while a command runs. Checkpoint every ~10 iterations and commit + push each checkpoint (`ckpt <n>`); a reclaimed box keeps the branch, not the files.
- Measure peak memory at the start, tag it, keep it under 75 % of the box. Report only the final numbers. Hard cap 6 h.
- A box that does not fit in RAM: run it coarse first (h 3.0), then h 2.0 with cells outside that region + 10 mm set void. Give the mass target as volume, not a share.
- Done: `runs/<part>/<version>/summary.md` in the form below, `result.json` (mass g M, worst index + case, solid_check pass, bed face, bed contact mm², peak memory GB, wall time), `preview.png`; `python3 tools/runs_index.py`; commit, push, PR.

## Save and print (done = saved)
A part is finished only when one turn does all of: `solid_check.py` PASS on the exact file; `print_orient.py` pass with the STL saved on its largest flat face (no first-layer island under 20 mm² lasting over 10 mm, no free-standing post over 10 mm); file at `print/<part>_<version>.<ext>` (zip over 50 MB; the File cell in `current-parts.md` is the only pointer to it); State READY; one `log.md` line; `summary.md` + `preview.png` (`render_preview.py <mesh> --mat PETG|TPU --prev <last g>`); committed, pushed, PR open. Spark sees the preview and this before "ready to print":
```
<part> <version>: ready to print
Weight: <g> (M), was <g>
Strength: worst index <x> in <case> (was <y>); weakest spot <where>   [box parts: "fits" + support g]
Solid: solid_check PASS
Print: <bed face>, bed contact <mm²> (M), tree supports <g>; file print/<name>
```
Slicing: tree(auto) supports, 30°, global; brim 5 mm under 400 mm² of bed contact; box parts 4 walls, 15 % gyroid. Report bed face, bed contact, support g, time; over ~25 % of the mass in supports goes back to the designer. A print that fails for orientation is re-sliced; one that snaps at a member is a design failure (`log.md` failure log). The moment a print is sent: one row at the top of `nox/print-log.md`, State PRINTING. A printed version is frozen; any change is a new version. Bambu Studio on the laptop: one window, never launch it while running, type paths, Ctrl+R slice, Ctrl+G export; set orientation in the `.3mf` transform if a remote session must re-orient.
