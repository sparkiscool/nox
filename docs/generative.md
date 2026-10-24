# Generative parts, heavy runs, save and print

Every generated part uses the topology engine `nox/topo` (default since 24 Oct, Spark). The old `noxopt.py`, `noxopt_local.py`, `printopt.py` and `noxopt_demo.py` stay in the repo for history and old part scripts only; no new part uses them. Method, settings and test results: `nox/topo/README.md`. Tags: T measured in a cloud run, E estimate.

## The rule
1. **Drawn = mounting + must-haves only:** fastener seats with their boss walls, mating and contact faces, pin bores, rod sockets, connector seats, flexures, hooks, windows, TPU feet, each at its smallest volume. Everything else is generated. Every drawn body lies inside the envelope and the design space can reach it. Access cuts for bolt heads and tools are never wider than the pad they serve. A drawn body must keep a 2 mm core at the solid gate's 2.0 mm erosion (bosses about 12 mm round on an M3, plates at least 6 mm under 6.25 mm members); `topo` checks reachability before the first solve and raises if a body cannot be joined.
2. **Organic finish (default, Spark 23 Oct):** the whole part is one smooth field of round members blended with fillets; only holes, bores and flat contact faces stay geometric. Drawn bodies stay only as 2.5 mm functional skins next to cuts and the envelope's outside; the mounting features the main support holds stay whole. Do not switch it off (`finish="drawn"`) without Spark.
3. **Layout:** robust (eroded / nominal / dilated) stiffness design to a mass target over every load case plus tie loads on every drawn body (`topo` adds those). Minimum member in mm; the engine raises it to what the solid gate needs (6.25 mm). Pick the bed face first.
4. **Stop** at ≤ 2 % grey or the end of the beta 32 stage. Hard cap 6 h, then build from the last checkpoint.
5. **No patching:** no grey fill, dilation, member growing or "keep the largest body". A detached feature means a setup or engine fix and a rerun. `solid_check` PASS or the part does not exist.
6. **Strength:** `topo` writes every case's as-built index. A topology part replaces a drawn or older one only if it beats the old part's worst index on the same FE grid (`python3 -m topo.compare`) and passes the solid check.
7. **Engine first:** a fix that would help every part goes in `nox/topo`, never in one part's script or mesh. One session edits the engine at a time.

## How-to
Worked parts: `nox/topo/demos/holder.py` (strut frame, the look test part), `nox/topo/demos/stand.py` (box stand parts), `nox/overseer/scan_stick/*_topo_*.py`.
```python
from topo import Part, Settings, optimise
def setup(h):
    p = Part("my_part", h, drawn, cuts, env, build="+z", material="PETG")
    p.support("default", region); p.load("case", region, (fx, fy, fz), cls="misuse")
    return p
optimise(setup, h=1.0, gen_cm3=6.0, out_dir="runs/my_part/v1", name="my_part", settings=Settings(), git=True)
```
The run writes `runs/<part>/<version>/` with the STL, `<part>_view.stl`, `preview.png`, `log.txt`, `result.json` (mass, worst index per case, solid check, function check), checkpoints. Then `print_orient.py` and the save steps below. Any `Settings` field can be set per run: `python3 -m topo.run <demo> --set key=value`.

| Default | Value |
|---|---|
| Grid | h 1.0 (a coarse 2h stage runs first); small parts 0.5 to 0.8 |
| Minimum member | 3.2 mm asked, raised to 6.25 mm for the solid gate |
| Finish | `organic`: 2.5 mm skins, 2.5 mm fillets, 1.0 mm rounded rims |
| Case weighting | p-mean, P 6, cases over the full solid's compliance; ties weight 0.5 |
| Mass target | `gen_cm3`, generated volume; lower it while the as-built worst index stays at target |
| Solver | Pardiso on active dofs; multigrid CG beyond the RAM budget, chosen automatically |

Known limits (T): the layout is a stiffness design; strength is reported per case, not optimised. A keep-out with a sharp edge that members must bend round sets the worst index (holder: rounding the battery channel edge r 3 mm took 0.569 to 0.493 at 11.7 g). Engine edit: `cd nox && python3 -m topo.test_topo` once, commit, one `log.md` line.

## Heavy runs (cloud box)
- Branch `run/<part>-<version>`, one `run-part` worker per run, each in its own worktree. Independent runs start together (Spark 24 Oct): as many as fit the box's memory budget below, the rest in sibling cloud sessions (`create_session`), never a queue. Only a run that needs another's result waits for it. Start in the background (`nohup python3 <script> > runs/<part>/<version>/log.txt 2>&1 &`): a run lives only while a command runs. Checkpoint every ~10 iterations and commit + push each checkpoint (`ckpt <n>`); a reclaimed box keeps the branch, not the files.
- Measure peak memory at the start, tag it, keep the sum over the runs on a box under 75 % of it. Report only the final numbers. Hard cap 6 h.
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
Every "ready to print" reply (and every reply that names a file to print) ends with one ready-to-paste prompt for a print and slicing session on the laptop (Spark 23 Oct), in a code block, filled in for that file:
```
Print and slicing session (opus, medium, computer control on the laptop). In the nox clone: git pull on main.
Part: <part> <version>, file print/<name> (<g> g <material>, <x> x <y> x <z> mm, solid_check PASS).
Bambu Studio, A1 mini, <material>: import the file as saved (<bed face> down, do not re-orient), tree(auto) supports 30° global<, brim 5 mm if bed contact < 400 mm²><, part rules>.
Ctrl+R slice; report time, filament g, support g (over ~25 % of the part mass: stop and tell Spark). Check <what to keep clear, e.g. the thread>.
Send to the A1 mini only after Spark says go. Then one row at the top of nox/print-log.md (State PRINTING), commit, push.
```
Slicing: tree(auto) supports, 30°, global; brim 5 mm under 400 mm² of bed contact; box parts 4 walls, 15 % gyroid. Report bed face, bed contact, support g, time; over ~25 % of the mass in supports goes back to the designer. A print that fails for orientation is re-sliced; one that snaps at a member is a design failure (`log.md` failure log). The moment a print is sent: one row at the top of `nox/print-log.md`, State PRINTING. A printed version is frozen; any change is a new version. Bambu Studio on the laptop: one window, never launch it while running, type paths, Ctrl+R slice, Ctrl+G export; set orientation in the `.3mf` transform if a remote session must re-orient.
