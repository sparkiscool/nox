# Nox

Spark's drone project: the **box** (funnel, magnet hold, vertical swap wheel, two-head radial arm), **Scout v3** (`nox/scout/v3/`) and the **Overseer** (`nox/overseer/`). This repo is the only source.

## Every session
1. `git pull` on `main`. Read only your part's row in `nox/current-parts.md`, its part doc and the `nox/dimensions.md` section it uses.
2. Branch `part/<part>` for design work, `run/<part>-<version>` for a heavy run (one per session, `run-part` agent).
3. Every turn that changes a design commits and pushes the same turn: script, results, the part's row, one line at the top of `nox/log.md`. Unpushed work does not exist.
4. End with a small PR to `main`, one part. Never edit another part's script or row; put what it must change in the PR description.

## Hard rules (Spark)
- **Solid check gates everything.** Nothing is saved, called BUILT or READY, or shown unless `python3 nox/solid_check.py <mesh>` reads PASS on the exact file. A FAIL is fixed in the design, never on the mesh. Every other check is reported, never blocks.
- **Attachments are drawn**, never generated: pads ≥ Ø12 with ≥ 2.0 web, posts ≥ 6 x 6, a gusset into the body, a boss wall ≥ 3 round every counterbore or access cut.
- **Tag every number:** P measured, M mesh, C CAD, E estimate, F frame model. A number lives only in `nox/dimensions.md`.
- **PETG rigid, TPU feet and nodes, no PLA.** Nothing under 2.0 mm (flexures excepted). Rods 3.0 carbon, epoxied. Bambu A1 mini, 180 cube, tree supports, largest flat face down.
- **No old designs** (slide swap, slider, pusher, turret, turntable, carousel, groove seat, Z stage, top-mounted battery) unless Spark asks fresh. Simplest design wins; every placement comes from an optimization. Ask Spark only on costly or hard-to-undo choices; otherwise decide, mark DEFAULT, continue. Replies short, no em dashes.

## Model and effort
| Work | Model | Effort |
|---|---|---|
| Design or CAD session (`part/<part>`), the control conversation, the `review` agent | opus | high |
| Geometry or a fit that two opus turns could not solve | fable | high |
| Print and slicing session on the laptop (computer control) | opus | medium |
| Heavy run (`run-part` agent), research, docs, log and state edits | sonnet | medium |
Agents carry theirs in `.claude/agents/`; a single-repo cloud session takes `.claude/settings.json`; a project thread is started with the model above (`claude --model <m> --effort <e>` locally). Never a weaker model than the row says.

## Where things are
- `nox/current-parts.md`: every part's state, file and next step; open items and Spark questions.
- `nox/dimensions.md`: every number (section M: bought parts and mounting data).
- `docs/generative.md`: heavy runs, the generative rule, save and print, done = saved.
- `nox/log.md`, `nox/print-log.md`: history, newest first. `archive/`: Scout v2 and old queues, never a current design.
- `claude/`: background research (materials, frames, mounting dimensions, patents). Cite it when a task needs it, never read it by default.
