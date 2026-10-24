# Nox

Spark's drone project: the **box** (funnel, hook latch, swap arm, bays, stand), **Scout v3** (`nox/scout/v3/`) and the **Overseer** (`nox/overseer/`). This repo is the only source: nothing depends on a file that exists only on a PC, in a chat or in an artifact.

## Every session
1. `git pull` on `main`. Read only your part's row in `nox/current-parts.md`, its part doc and the `nox/dimensions.md` section it uses.
2. Branch `part/<part>` for design work, `run/<part>-<version>` for a heavy run (one `run-part` worker per run; several run at once). One prompt from brief to saved part: `/part <part> <version>: <brief>`.
3. Every turn that changes a design commits and pushes the same turn: script, results, the part's row, one line at the top of `nox/log.md` (≤ 400 chars: what changed and where; the detail lives in the part doc and `summary.md`). Unpushed work does not exist.
4. End with a small PR to `main`, one part. First `python3 tools/check_state.py` (no ERROR; fix the warnings your rows or lines caused). Then `python3 tools/pr.py open "<part> <version>: <what>" [body.md]`. Run the `review` agent on it; on PASS merge it yourself, `python3 tools/pr.py merge <n>`; on FAIL fix and re-review, never merge a FAIL. A tooling, docs, config or research PR is merged after the session's own check. A refused merge means main moved: merge `origin/main` into the branch, resolve, push, merge again. No PR waits on Spark.
5. A system session owns every piece cut from its system (one PR may carry them). Never edit a part outside it, its script or row; put what it must change in the PR description and say which part you kept.
6. A rule change (Spark) lands in one PR: one line in Hard rules, the method in `docs/generative.md`, and its check (a tool, or a `review` check). Skills and agents point at those and never restate a rule.

## Hard rules (Spark)
- **Solid check gates everything.** Nothing is saved, called BUILT or READY, or shown unless `python3 nox/solid_check.py <mesh>` reads PASS on the exact file. A FAIL is fixed in the design, never on the mesh. Every other check is reported, never blocks.
- **Attachments are drawn**, never generated: pads ≥ Ø12 with ≥ 2.0 web, posts ≥ 6 x 6, a gusset into the body, a boss wall ≥ 3 round every counterbore or access cut.
- **Tag every number:** P measured, M mesh, C CAD, E estimate, F frame model. A number lives only in `nox/dimensions.md`.
- **Ready to print = a slicing prompt.** Every reply that says a file is ready to print ends with the laptop slicing-session prompt (`docs/generative.md`, Save and print), filled in, in a code block.
- **PETG rigid, TPU feet and nodes, no PLA.** Nothing under 2.0 mm (flexures excepted). Rods 3.0 carbon, epoxied. Bambu A1 mini, 180 cube, tree supports, largest flat face down.
- **No old designs** (slide swap, slider, pusher, turret, turntable, carousel, groove seat, Z stage, top-mounted battery) unless Spark asks fresh. Simplest design wins; every placement comes from an optimization.
- **System first, then cut (Spark 24 Oct).** Never design or optimise a part alone: a system pass on the assembled thing (every part and bought part in pose, flight / crash / swap / handling cases, load path through every joint, the cuts) writes each piece's requirements to `dimensions.md` "System <name>"; each piece then runs its own topology with those as parameters; the assembled result is rechecked. Holds for every READY, BUILT, HOLD and DESIGN part. Method: `docs/generative.md` The rule, 0.
- **Every drone design, in this order (Spark 24 Oct).** Checked on the assembled model (`nox/system_check.py`), numbers in `dimensions.md` L, method in `docs/generative.md` The rule, 0.
  1. **Light.** Every structural part comes from topology on its real loads: the lightest that meets its index.
  2. **Compact and organised.** Parts nest round the load path (heaviest on the support axis), never stacked with the space between filled in; nothing wastes space. Everything that must be reached (pack swap, connectors, USB / SD, fasteners, switches, antennas) is reachable without taking another part off. The placement optimisation minimises bounding volume and largest extent; the summary reports both.
  3. **Cage first.** Whatever the drone hits (wall, floor, ceiling, any direction), a cage member touches first, ≥ 3 mm ahead (DEFAULT) of props, motors, electronics, camera, lidar and pack. Full cage top, sides and bottom, triangulated, load paths closed into the core; box openings recessed behind cage members. A stick or column layout is not shown as a design.
  4. **Balanced.** CG on the thrust centre within 3 mm horizontally (DEFAULT), with and without the pack: the pack sits on the CG axis so a swap does not move it.
  5. **Mounted for what it carries.** Each piece runs its own topology but is loaded from the system: supports are what it really bolts to, loads are what it holds (mass x the `dimensions.md` L cases, at that mass's CG) and the joint reactions. A heavy or one-sided payload (the L2 lidar: heavy, one mounting face) gets the crash cases at its CG, a stiffness target, and a load path back into the core, never a stalk.
  6. **Stiff where it senses** (added, DEFAULT). FC, camera and lidar mounts: first mode ≥ 100 Hz (E, until the vibration log gives the rotor band), reported in Hz.
  7. **Breaks in the right order** (added, DEFAULT). In a crash a cheap printed cage node or foot fails first, never the frame, a mount or the pack, and it is replaced without taking the drone apart.
  8. **Clear air, clear view, clean wiring** (added, DEFAULT). Nothing but arms and thin rods in the prop wash; lidar and camera fields of view free except thin rods (blocked share reported); every cable has a drawn route, strain relief and a connector it can be unplugged at; heat sources sit in airflow.
  9. **Fits the box** (added). The funnel catch interface, the hook latch zone and the swap path are part of every drone's system model.
- **Concept before detail; three strikes (24 Oct, DEFAULT).** No topology run or detailed CAD before the concept check (`docs/generative.md` The rule, 0): the system layout with the rule checks run on it, two alternatives weighed, the `review` agent in concept mode, the layout artifact and one line to Spark (then continue, never wait). A part whose third version in a row fails the gate, a check or Spark's look goes back to its system pass, never to a fourth patch.
- **Parallel by default (Spark 24 Oct).** Work that does not depend on other work never queues: independent runs, parts, checks, research and reviews go to separate workers at once (Agent tool, `isolation: "worktree"` for anything that commits; a sibling cloud session via `create_session` when this box is full), each on its own branch and PR. Only a real dependency (another's mesh, number or merge) waits.
- **Children stay in sync (Spark 24 Oct).** A session that controls other sessions or workers sends every Spark decision, rule change, design change, stop or supersede to each live child the same turn (send_message); children merge `origin/main` at the start of every step; the controller checks each child (`get_session`) before reporting, in one line; finished or superseded children are archived.
- **Never wait on Spark.** Decide, mark DEFAULT, write the choice and the alternative in one `log.md` line, continue; a shared number changes the same way (its `dimensions.md` row + the log line). Spark reverts what he disagrees with. A session never spends money (Spark orders from the cart link) or deletes work on `main` alone. Replies short, no em dashes.

## Model and effort
| Work | Model | Effort |
|---|---|---|
| Design or CAD session (`part/<part>`), the control conversation, the `review` agent | opus | high |
| Geometry or a fit that two opus turns could not solve | fable | high |
| Print and slicing session on the laptop (computer control) | opus | medium |
| Heavy run (`run-part` agent), research, docs, log and state edits | sonnet | medium |
Agents carry theirs in `.claude/agents/`; a session takes `.claude/settings.json`; a project thread is started with the model above (`claude --model <m> --effort <e>` locally). Never a weaker model than the row says.

## Where things are
- `README.md`: the start page for Spark: what is where, how to start each kind of session.
- `nox/current-parts.md`: every part's state, file and next step; open items and Spark questions.
- `nox/dimensions.md`: every number (section M: bought parts and mounting data). `nox/parts-list.md`: every bought and printed part with qty and status (on hand / ordered / to buy).
- `docs/generative.md`: heavy runs, the generative rule, save and print, done = saved.
- `nox/topo/`: the topology engine, the default for every generated part (`from topo import Part, Settings, optimise`; organic finish; `nox/topo/README.md`). After an engine edit run `cd nox && python3 -m topo.test_topo` once.
- `nox/*.py`: the tools (`solid_check`, `print_orient`, `render_preview`, `motion` for the motion artifact, `interfaces`); `tools/check_state.py` lints state (missing print files, orphans, long cells and log lines; `--branches` lists merged and idle branches). `noxopt`, `noxopt_local`, `printopt`, `noxopt_demo` are the retired engine, kept for old part scripts only; no new part uses them.
- `nox/box/`, `nox/scout/v3/`, `nox/overseer/`: part docs and scripts. `runs/<part>/<version>/`: one folder per run, never overwritten; board `runs/index.md` (`python3 tools/runs_index.py`). `print/<part>_<version>.<ext>`: print files; the File cell in `current-parts.md` points at them.
- `tools/pr.py`: open and merge PRs over REST. `tools/amazon-parts-mcp/` + `.mcp.json`: Amazon ASINs, prices and cart links (skill `drone-parts`). `cloud/`: environment setup script and the SessionStart hook; start every cloud session in the "Nox" environment with this repo selected.
- `nox/log.md`, `nox/print-log.md`: history, newest first. `archive/`: Scout v2 and old queues, never a current design. `claude/`: background research; cite it when a task needs it, never read it by default.

## Git
- Commit messages: `<part> <version>: <what>` or `ckpt <n>`. Push after every commit. Never force-push `main`; never rewrite history on a shared branch.
- No git LFS (blocked from cloud boxes): any single file under 50 MB (decimate a viewing mesh, zip a big file); commit a run's final mesh, not every checkpoint's.
- Cloud boxes block GitHub GraphQL: every `gh pr ...` subcommand fails with 403. `gh api repos/sparkiscool/nox/...` (REST) works; `tools/pr.py` wraps open and merge. Branch deletes are refused from cloud boxes; the repo's auto-delete removes merged branches.
- **A GitHub action a session cannot do** (branch deletes, repo settings, anything the proxy or the permission check refuses): give Spark one ready-to-paste prompt for Claude in Chrome in a code block (exact URLs, what to click, what to leave alone, what to check after), then carry on with the rest.
