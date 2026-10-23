# Nox

Spark's drone project: the **box** (funnel, magnet hold, vertical swap wheel, two-head radial arm), **Scout v3** (`nox/scout/v3/`, starting) and the **Overseer** (`nox/overseer/`, starting). Scout v2 is retired to `archive/scout-v2/`: history, never a current design.

This repo is the only source. Nothing depends on a file that exists only on a PC, in a chat or in an artifact.

## Every session
1. `git pull` on `main`. Then read only what the task needs (table below). Nothing else is a must-read.
2. Branch: `part/<part>` for design work, `run/<part>-<version>` for a heavy run.
3. Every turn that changes a design commits and pushes, same turn: the script, the results, the part's row in `nox/current-parts.md`, one line at the top of `nox/log.md`. Unpushed work does not exist.
4. End with a small PR to `main`, one part. Never edit another part's script or row; put what it must change in the PR description and say which part you kept.

## Read when you need it
| Task | Read |
|---|---|
| Any part work | the part's row in `nox/current-parts.md`, its part doc (`nox/box/`, `nox/scout/`, `nox/overseer/`), the `nox/dimensions.md` section it uses |
| An open problem, or a question for Spark | `nox/open.md` |
| A bought part: qty, on hand, to buy | `nox/parts-list.md`; its mounting data in `nox/dimensions.md` section M |
| A generative, topology or FE run | `docs/generative.md` |
| Saving a print file, slicing, logging a print | `docs/printing.md` |
| A rule in detail (solid check, attachments, materials, printer, banned designs) | `docs/rules.md` |
| Git, cloud boxes, splitting work, state words, editing shared docs | `docs/workflow.md` |
| Where a file lives, what an engine script does | `docs/README.md` |
| What changed or failed, what was printed | `nox/log.md`, `nox/print-log.md` (newest first) |
| Background research | `claude/` (never a current design) |

## Hard rules (Spark)
- **Speed first.** One blocking gate: the solid check. Every other check is reported, never blocks.
- **Solid parts, always.** Nothing is saved, called BUILT or READY, or shown as a result unless `python3 nox/solid_check.py <mesh>` reads PASS on the exact file. A FAIL is fixed in the design, never patched on the mesh.
- **Attachments are drawn**, never generated: pads ≥ Ø12, posts ≥ 6 x 6, a gusset into the body.
- **Simplest design wins.** Justify any added complexity.
- **Every placement comes from an optimization.**
- **Tag every number:** P measured, M mesh, C CAD, E estimate, F frame model.
- **Ask Spark** only on costly or hard-to-undo choices, or before changing a shared number. Otherwise decide, mark DEFAULT, continue. A question goes in `nox/open.md` with its best answer in one line.
- **Replies short**, each problem plainly with the best fix. No em dashes. Everything we make is beautiful: part, preview, summary, doc.
- Drone parts: lightest that meets the loads. Box parts: strong and fits. PETG rigid, TPU feet and nodes, no PLA. Nothing under 2.0 mm (flexures excepted). Rods 3.0 carbon, epoxied.
- No old designs (slide swap, slider, pusher, turret, turntable, carousel, groove seat, Z stage, top-mounted battery) unless Spark asks fresh.
- Crash behaviour is proven by physical test. Buy and modify over building from scratch.

## Done = saved
A part is finished only when one turn does all of: solid check PASS; `print_orient.py` pass, STL on its largest flat face; file in `print/<part>_<version>.<ext>` and `print/latest/`; State READY in `current-parts.md`; a `log.md` line; `runs/<part>/<version>/summary.md` with the preview; committed, pushed, PR open. Form and detail: `docs/printing.md`.

## Docs
One fact, one place: a state only in `current-parts.md`, a number only in `dimensions.md`, an open item only in `open.md`, history only in `log.md`. Other docs link. A retired design is deleted, not described. Pull right before editing, change only your own rows, keep edits small.
