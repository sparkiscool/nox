# Nox

Spark's drone project: the **box** (funnel, magnet hold, vertical swap wheel, two-head radial arm), **Scout v3** (`nox/scout/v3/`) and the **Overseer** (`nox/overseer/`). This repo is the only source: nothing depends on a file that exists only on a PC, in a chat or in an artifact.

## Every session
1. `git pull` on `main`. Read only your part's row in `nox/current-parts.md`, its part doc and the `nox/dimensions.md` section it uses.
2. Branch `part/<part>` for design work, `run/<part>-<version>` for a heavy run (one `run-part` worker per run; several run at once). One prompt from brief to saved part: `/part <part> <version>: <brief>`.
3. Every turn that changes a design commits and pushes the same turn: script, results, the part's row, one line at the top of `nox/log.md`. Unpushed work does not exist.
4. End with a small PR to `main`, one part: `python3 tools/pr.py open "<part> <version>: <what>" [body.md]`. Run the `review` agent on it; on PASS merge it yourself, `python3 tools/pr.py merge <n>`; on FAIL fix and re-review, never merge a FAIL. A tooling, docs, config or research PR is merged after the session's own check. A refused merge means main moved: merge `origin/main` into the branch, resolve, push, merge again. No PR waits on Spark.
5. A system session owns every piece cut from its system (one PR may carry them). Never edit a part outside it, its script or row; put what it must change in the PR description and say which part you kept.

## Hard rules (Spark)
- **Solid check gates everything.** Nothing is saved, called BUILT or READY, or shown unless `python3 nox/solid_check.py <mesh>` reads PASS on the exact file. A FAIL is fixed in the design, never on the mesh. Every other check is reported, never blocks.
- **Attachments are drawn**, never generated: pads ≥ Ø12 with ≥ 2.0 web, posts ≥ 6 x 6, a gusset into the body, a boss wall ≥ 3 round every counterbore or access cut.
- **Tag every number:** P measured, M mesh, C CAD, E estimate, F frame model. A number lives only in `nox/dimensions.md`.
- **Ready to print = a slicing prompt.** Every reply that says a file is ready to print ends with the laptop slicing-session prompt (`docs/generative.md`, Save and print), filled in, in a code block.
- **PETG rigid, TPU feet and nodes, no PLA.** Nothing under 2.0 mm (flexures excepted). Rods 3.0 carbon, epoxied. Bambu A1 mini, 180 cube, tree supports, largest flat face down.
- **No old designs** (slide swap, slider, pusher, turret, turntable, carousel, groove seat, Z stage, top-mounted battery) unless Spark asks fresh. Simplest design wins; every placement comes from an optimization.
- **System first, then cut (Spark 24 Oct).** Never design or optimise a part alone. First a system pass on the assembled thing (the Scout, the Overseer, or a box sub-assembly such as the arm, stand or catch): every part in its assembled pose with its neighbours and bought parts, the system's load cases (flight, crash, swap, handling), the load path through every joint, the cuts into printable pieces. It writes each piece's requirements (envelope, interfaces, joint reactions per case, stiffness and worst index needed for the assembled strength) to `dimensions.md`. Then each piece runs its own topology in its own session with those as its parameters, and the assembled result is rechecked (fit, worst index, mass and CG). Every READY, BUILT, HOLD and DESIGN part is held to it now. Method: `docs/generative.md` The rule, 0.
- **Compact by default (Spark 24 Oct).** Every design and layout is as compact as it can be: components nest around each other and around the load path (the heaviest on the support axis, the rest on its faces), never stacked or bolted on side by side with empty space between. The placement optimization minimises the assembled bounding volume and the largest extent under the fit, view, access and service constraints, and the summary reports both. Method: `docs/generative.md` The rule, 0.
- **Parallel by default (Spark 24 Oct).** A session never queues work that does not depend on other work: independent jobs (runs, parts, checks, research, reviews of different PRs) go to separate workers at the same time (Agent tool, `isolation: "worktree"` for anything that commits; a sibling cloud session via `create_session` when this box is full). Only a real dependency (needs the other's mesh, number or merge) waits. Each worker keeps its own branch and PR; the session collects the results.
- **Children stay in sync (Spark 24 Oct).** A session that starts or controls other sessions or workers keeps every one of them up to date: each Spark decision, rule change, design change, stop or supersede goes to every live child the same turn (send_message), each child merges `origin/main` at the start of every step, and before reporting the controller checks each child's state (`get_session`) and says it in one line. A child that finishes or is superseded is archived.
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
- `nox/current-parts.md`: every part's state, file and next step; open items and Spark questions.
- `nox/dimensions.md`: every number (section M: bought parts and mounting data). `nox/parts-list.md`: every bought and printed part with qty and status (on hand / ordered / to buy).
- `docs/generative.md`: heavy runs, the generative rule, save and print, done = saved.
- `nox/topo/`: the topology engine, the default for every generated part (`from topo import Part, Settings, optimise`; organic finish; `nox/topo/README.md`). After an engine edit run `cd nox && python3 -m topo.test_topo` once.
- `nox/*.py`: the tools (`solid_check`, `print_orient`, `render_preview`, `motion` for the motion artifact, `interfaces`). `noxopt`, `noxopt_local`, `printopt`, `noxopt_demo` are the retired engine, kept for old part scripts only; no new part uses them.
- `nox/box/`, `nox/scout/v3/`, `nox/overseer/`: part docs and scripts. `runs/<part>/<version>/`: one folder per run, never overwritten; board `runs/index.md` (`python3 tools/runs_index.py`). `print/<part>_<version>.<ext>`: print files; the File cell in `current-parts.md` points at them.
- `tools/pr.py`: open and merge PRs over REST. `tools/amazon-parts-mcp/` + `.mcp.json`: Amazon ASINs, prices and cart links (skill `drone-parts`). `cloud/`: environment setup script and the SessionStart hook; start every cloud session in the "Nox" environment with this repo selected.
- `nox/log.md`, `nox/print-log.md`: history, newest first. `archive/`: Scout v2 and old queues, never a current design. `claude/`: background research; cite it when a task needs it, never read it by default.

## Git
- Commit messages: `<part> <version>: <what>` or `ckpt <n>`. Push after every commit. Never force-push `main`; never rewrite history on a shared branch.
- No git LFS (blocked from cloud boxes): any single file under 50 MB (decimate a viewing mesh, zip a big file); commit a run's final mesh, not every checkpoint's.
- Cloud boxes block GitHub GraphQL: every `gh pr ...` subcommand fails with 403. `gh api repos/sparkiscool/nox/...` (REST) works; `tools/pr.py` wraps open and merge. Branch deletes are refused from cloud boxes; the repo's auto-delete removes merged branches.
- **A GitHub action a session cannot do** (branch deletes, repo settings, anything the proxy or the permission check refuses): give Spark one ready-to-paste prompt for Claude in Chrome in a code block (exact URLs, what to click, what to leave alone, what to check after), then carry on with the rest.
