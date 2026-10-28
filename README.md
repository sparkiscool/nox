# Nox

Spark's drone project: the **box** (ground station that catches a drone, swaps its battery and charges it), **Scout v3** (`nox/scout/v3/`) and the **Overseer** (`nox/overseer/`, S500 with a Unitree L2 lidar). Rules for sessions: `CLAUDE.md`. This page is the map.

## Where to look
| I want | Open |
|---|---|
| State of every part, what is next, who owns it, what Spark must measure / buy / decide | `nox/current-parts.md` |
| A number (with its tag P / M / C / E / F) | `nox/dimensions.md` |
| What to buy, what is on hand | `nox/parts-list.md` (cart links: skill `drone-parts`) |
| A file to print | `print/` (only files a `current-parts.md` row names; `print/README.md` lists do-not-print ones) |
| Every generative run and its numbers | `runs/index.md` |
| What happened, newest first | `nox/log.md` (design), `nox/print-log.md` (prints) |
| How parts are designed, saved and printed | `docs/generative.md` |
| Research notes | `claude/` (cited when needed); Scout v2 and old designs: `archive/` |
| The box as a whole (sections to scale, every system, owners, change log) | the Nox Ground Unit artifact https://claude.ai/artifact/RX2CexkfhB5UoQHnGTVgCC, kept by the box master session (`nox/box/box.md` Master view); with the box assembled in 3D from its real mesh files at the top (every designed component; the viewer alone: https://claude.ai/artifact/VyTyzuieTjrbBnURYPDgmG) |

## Start a session
Every session runs in the cloud "Nox" environment with this repo selected. Paste one of these:

| Work | Prompt (model, effort) |
|---|---|
| Design one part | `/part <part> <version>: <brief>` (opus, high) |
| System pass (assembled layout, cuts, piece requirements) | `System pass for <Scout v3 / Overseer / box catch / box swap / box structure / scan stick>: <brief>. Follow docs/generative.md The rule, 0, end with the concept check.` (opus, high) |
| Heavy run | `Run <part> <version> with the run-part agent: <script>, <settings>.` (sonnet, medium) |
| Print and slicing on the laptop | the filled-in prompt every "ready to print" reply ends with (opus, medium, computer control) |
| Buy parts | `Order <what> for the <drone>` (skill `drone-parts`) |
| Organise, rules, workflow | `Workflow chat: <ask>` (opus, high) |

## Housekeeping
- **Organiser:** a session tidies the project every 6 h on its own (sessions, rules, overlaps, stale work, board): `docs/organiser.md`.
- **Progress:** no digests; sessions message Spark only for print-ready files or his decisions. The live view is the **Nox Board** (https://claude.ai/artifact/NKBNkPUSyKgwYrtZZBhMkw), refreshed by the organiser every 6 h (`docs/sessions.md`).
- `python3 tools/check_state.py` lints the state: missing or orphan print files, cells and log lines too long. `--branches` lists branches merged into main (safe to delete) and idle ones with unmerged work.
- Cloud boxes cannot delete branches: the session gives Spark a Claude in Chrome prompt for that.
- CI (`.github/workflows/checks.yml`) runs on every PR to main: `state` (check_state), `tests` (system_check, assembly; a warning when `nox/topo/` changed, since topo.test_topo needs pypardiso and runs locally), `solid` (solid_check on every added or modified `print/*.stl`), `lint-new-lines` (new log.md lines ≤ 400 chars). CI must be green before merge; `tools/pr.py merge` refuses otherwise (waits up to 15 min while pending; deps in `requirements-ci.txt`).
