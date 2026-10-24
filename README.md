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
- `python3 tools/check_state.py` lints the state: missing or orphan print files, cells and log lines too long. `--branches` lists branches merged into main (safe to delete) and idle ones with unmerged work.
- Cloud boxes cannot delete branches: the session gives Spark a Claude in Chrome prompt for that.
