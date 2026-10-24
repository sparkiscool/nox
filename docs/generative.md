# Generative parts, heavy runs, save and print

Every generated part uses the topology engine `nox/topo` (default since 24 Oct, Spark). The old `noxopt.py`, `noxopt_local.py`, `printopt.py` and `noxopt_demo.py` stay in the repo for history and old part scripts only; no new part uses them. Method, settings and test results: `nox/topo/README.md`. Tags: T measured in a cloud run, E estimate.

## The rule
0. **System first, then cut (Spark 24 Oct).** The unit of design is the assembled system, never one piece. Applies to every part that is READY, BUILT, HOLD or in DESIGN, not only new versions.
   - **System pass (one per system, before any piece):** a system model places every printed and bought part in its assembled pose (the system model is `nox/<machine>/<system>.assembly.json` loaded with `nox/assembly.py`, `nox/ASSEMBLY.md`) (`interfaces.py` features, `s500_cad.py`, bought-part data in `dimensions.md` M); the motion artifact and fit checks use the same model. It carries the system cases: flight (hover, max thrust, yaw), crash (drop on an arm, a corner, the nose), swap and catch loads, handling and transport, masses from the parts list, each applied where it acts on the real machine (motor, pack, wheel, plunger). Solve the assembled system (a coarse `topo` run over the joined envelope, or an FE / frame model) and decide the cuts there: pieces that fit the 180 cube, print on their largest flat face, assemble in order and can be serviced, joints where the stress is low.
   - **Piece requirements (the system pass's output):** for every piece, in `dimensions.md` section "System <name>": envelope, interfaces (positions, fasteners, pins, pads), joint reactions per system case, its mass slot `budget_g` (a soft target, `dimensions.md` L Mass budget), the stiffness it must give at each joint, and the worst index it must reach so the assembled system meets its own target. These are the piece's `topo` parameters.
   - **System cases and targets:** `dimensions.md` L for every drone (hover, flight, landing, crash in 14 directions, swap, handling; cage margin, balance, sensor-mount mode, breaking order, index targets), plus the system's own section. Every case acts at the CG of the mass it moves.
   - **Mounted for what it carries (Spark 24 Oct):** a piece's supports are the faces and fasteners it really bolts to, its loads are the masses it holds times the L cases at their CG plus the joint reactions from the system pass. A heavy or one-sided payload (the L2: heavy, one mounting face) is modelled as its mass at its CG on a rigid link to its mounting face, so the piece sees the full moment, and gets a stiffness / first-mode target and a load path back into the core (triangulated, never a stalk).
   - **Piece pass (own session or chat, in parallel):** each piece runs its own `topo` (or drawn design) with those parameters: loads = the joint reactions, supports = the neighbour interfaces, targets from the table. Never an invented support or a guessed load on one piece.
   - **Assembled recheck:** the pieces go back into the system model: fit 0 mm³ between pieces and with bought parts, the assembled worst index per case with the joints modelled, mass and CG of the whole, first modes where it flies. A miss goes back to the system pass, not to a local patch.
   - **Compact layout (Spark 24 Oct, every design):** the system pass places the parts by an optimization that minimises the assembled bounding volume and the largest extent (with the guard / bumper offsets included) under the fit, view (lidar hemisphere, camera frustum), access (fasteners, battery swap, cables) and service constraints. Nest parts round the load path, heaviest on the support axis; no empty column or plate between parts that are only stacked or bolted on. The run reports bounding volume (L) and extents (mm) of the assembled system.
   - **Cage first (every drone, Spark 24 Oct):** the full cage (top, sides, bottom) is in the system model. Contact-first check on the assembled model: for every direction, the support distance of the cage exceeds that of every protected part (pack, cartridge, receiver, electronics, props, motors, camera) by ≥ 3 mm (DEFAULT; alt 1 mm); openings the box needs (cartridge path, head) are recessed behind cage members (margin ≥ 0 inside their cone). The cage layout comes from an optimisation (frame or `topo`) on the system cases, triangulated with load paths closed into the core; report its worst index, mass and lowest mode.
   - **Concept check (before any piece run or detailed CAD, 24 Oct DEFAULT):** the system pass ends with (1) the rule checks run on the layout (the CLAUDE.md "Every drone design" list: cage first, compact bounding volume and extents, access, fit, mass and CG pack in and out, sensor-mount modes; `python3 nox/system_check.py <manifest>`; the manifest's `access` paths (pack swap, connectors, USB, fastener key lines, swept and checked against every part) and the crash probes (edges and posts between cage members, not only flat walls) are part of it and gate like cage first); (2) two alternatives weighed in one line each (what each wins and loses, the numbers); (3) the `review` agent in concept mode on the layout; (4) the layout artifact published and one line to Spark. Then continue on the chosen layout as DEFAULT, never wait. This is where a flimsy, stacked or column layout is caught, before hours of runs.
   - **Three strikes:** a part whose third version in a row fails the gate, a check, the assembled recheck or Spark's look goes back to its system pass (requirements, layout, cuts), never to a fourth version of the same piece. The log line names the strike.
   - **Existing parts:** a READY, BUILT or HOLD part gets the assembled recheck against its system's requirements. Pass: it keeps its state and its summary gets the System line. Fail: State back to DESIGN, File cell marked do not print, a new version from the piece pass. A PRINTED or PRINTING part is reported (the result in its row) and fixed in its next version.
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
| Mass target | `gen_cm3`, generated volume; lower it while the as-built worst index stays at target; never below what the strength, stiffness and function targets need, whatever the budget slot says |
| Solver | Pardiso on active dofs; multigrid CG beyond the RAM budget, chosen automatically |

Known limits (T): the layout is a stiffness design; strength is reported per case, not optimised. A keep-out with a sharp edge that members must bend round sets the worst index (holder: rounding the battery channel edge r 3 mm took 0.569 to 0.493 at 11.7 g). Engine edit: `cd nox && python3 -m topo.test_topo` once, commit, one `log.md` line.

## Heavy runs (cloud box)
- Branch `run/<part>-<version>`, one `run-part` worker per run, each in its own worktree. Independent runs start together (Spark 24 Oct): as many as fit the box's memory budget below, the rest in sibling cloud sessions (`create_session`), never a queue. Only a run that needs another's result waits for it. Start in the background (`nohup python3 <script> > runs/<part>/<version>/log.txt 2>&1 &`): a run lives only while a command runs. Checkpoint every ~10 iterations and commit + push each checkpoint (`ckpt <n>`); a reclaimed box keeps the branch, not the files. Commit checkpoints to the run branch while running; the PR to `main` carries only the final mesh, preview, summary, `result.json` and log (drop `ck.npz`, `coarse_ck.npz`, `nohup*.txt`, videos and any `ckpt` subfolder from the PR; keep `density.npz` only if a later rebuild reads it). A run that a newer version supersedes (not in `print/`, not named in `current-parts.md`, `dimensions.md` or a script as an input) is slimmed: delete its meshes, `.npz`, `.npy`, `.mp4`, `nohup*.txt` and checkpoint folders, add `RETIRED.md` with the recovery command; summary, result, preview and log stay.
- Measure peak memory at the start, tag it, keep the sum over the runs on a box under 75 % of it. Report only the final numbers. Hard cap 6 h.
- A box that does not fit in RAM: run it coarse first (h 3.0), then h 2.0 with cells outside that region + 10 mm set void. Give the mass target as volume, not a share.
- Done: `runs/<part>/<version>/summary.md` in the form below, `result.json` (mass g M, worst index + case, solid_check pass, bed face, bed contact mm², peak memory GB, wall time), `preview.png`; `python3 tools/runs_index.py`; commit, push, PR.

## Save and print (done = saved)
A part is finished only when one turn does all of: `solid_check.py` PASS on the exact file; `print_orient.py` pass with the STL saved on its largest flat face (no first-layer island under 20 mm² lasting over 10 mm, no free-standing post over 10 mm); file at `print/<part>_<version>.<ext>` (zip over 50 MB; the File cell in `current-parts.md` is the only pointer to it); State READY; one `log.md` line; `summary.md` + `preview.png` (`render_preview.py <mesh> --mat PETG|TPU --prev <last g>`); committed, pushed, PR open. Spark sees the preview and this before "ready to print":
```
<part> <version>: ready to print
System: <system>, assembled worst index <x> in <case> (was <y>), fit <mm³> (M), mass <g> (M), CG <where>
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
