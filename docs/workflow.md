# Workflow: sessions, git, live status, shared docs, cloud

## Every session
- Start: `git pull` on `main`. Read your part's row and the rows of the parts it touches in `nox/current-parts.md`. That is the newest design; a version in memory, in a chat attachment or in a local file is older unless that table says so. Before any fit check, pull so the mating parts' current scripts are on disk.
- Read only what the request needs (`CLAUDE.md`, "Read when you need it"). A CAD session reads its row, its part doc and the dimension section it uses; a research session reads no part docs. No research passes or subagents unless Spark asks.
- One session per part; Spark starts a fresh one when it gets long.
- Every turn that changes a design: same turn, the changed scripts, the print file, the row's cells, one `nox/log.md` line, all committed and pushed. Local copies (laptop clone, SD card, downloads) are mirrors; `main` on GitHub is the design. A session that ends without pushing left work nowhere.
- Someone else's part: never edit its file or row. Put what it must change in the PR description, and say which part you kept.

## Git
- Branch `part/<part>` for design work, `run/<part>-<version>` for a heavy run. Finish with a small PR to `main`, one part. Spark merges, or the Nox control conversation does.
- Commit messages: `<part> <version>: <what>` or `ckpt <n>`. Push after every commit. A rejected push means another session pushed first: `git pull --rebase`, resolve, push again.
- No git LFS (its uploads are blocked from cloud boxes). Meshes, previews and print files go in plain git; any single file stays under 50 MB (decimate a viewing mesh, zip a big 3mf); commit a run's final mesh, not every checkpoint's.
- Never force-push `main`. Never rewrite history on a shared branch.
- Laptop mirror: the CAD laptop holds a clone at `Documents\Nox`. A linked chat runs `git pull` there before slicing; nothing is edited on the laptop alone.

## Live status
- A part's state is written in exactly one cell: its State cell in `nox/current-parts.md`. No other doc restates it. A doc that disagrees with that cell is stale.
- State words, nothing else in the cell: DESIGN (being drawn or run) · BUILT (passes the Done checks, file not saved) · READY (file committed in `print/`, the only proof) · PRINTING · PRINTED (not tested) · TESTED (P numbers in the part doc) · FAILED (failure log) · HOLD (Spark said stop) · ON HAND (bought part).
- "Ready to print" without a file in `print/` on `main` is BUILT. Who moves a state: the design session to BUILT / READY; the print session to PRINTING when it sends; Spark's report to PRINTED / TESTED / FAILED, written by whichever session he tells. A sliced file on the SD card is still READY until the job starts on a printer.

## Editing shared docs (several sessions write at once)
1. `git pull` right before editing; change only your own rows; keep the edit small; commit and push at once.
2. Changing a number: its row in `nox/dimensions.md` plus one line at the top of `nox/log.md`. Superseded values leave the docs.
3. One rule, one place; one doc per part. Live docs hold current state only; a retired design is deleted, not described. History goes in `nox/log.md`, never into a title, a heading or a row.
4. One fact, one cell: a state only in `current-parts.md`, a number only in `dimensions.md`, an open problem or question only in `open.md`. Other docs link, never copy.
5. Open problems and Spark questions keep their numbers forever (other docs cite them). A closed one leaves `open.md`; its number goes on that file's Closed line, its outcome in `log.md`.

## Split the work
- At the start of every task, and whenever a run would wait on another, split the work into independent pieces. Each piece is its own cloud session on the repo, so pieces run at the same time on their own machines.
- One piece = one part, or one heavy run (one generative run per machine). Pieces that write the same doc rows or the same shared number stay in one session.
- How pieces start: the Nox control conversation (claude.ai/code, Projects, Nox) starts one thread per piece and watches them. Without it, Spark pastes a prompt into a new session (`claude --cloud "<prompt>"` from any clone, or a new session at claude.ai/code on the repo).
- A prompt holds: the goal, the files to read, the exact commands, what to report back, and the numbers it needs (tagged). Short, in a code block, ready to paste.
- The session that splits says which piece it kept and does not touch the others. Each piece ends in its own PR; the control conversation merges them.

## Cloud sessions
- The repo is the only source. A new or changed script is committed and pushed the same turn.
- Environment: the Claude Code cloud environment "Nox" (claude.ai/code, Environments) runs `cloud/setup.sh` once and snapshots the box, so the engine packages are installed; `cloud/session_start.sh` reinstalls them if a box lacks them and prints the box's cores and RAM.
- Engine change: edit, `python3 nox/test_noxopt.py`, commit, one `nox/log.md` line.
- A run's meshes, checkpoints and logs go in `runs/<part>/<version>/`, committed with every checkpoint, so a reclaimed box loses nothing. A print-ready file goes into `print/` the same turn. A cloud box and the chat itself are not storage.
- Limits: a cloud box is listed at ~4 vCPU, 16 GB RAM, 30 GB disk (the old chat boxes were 2 cores / 7 GB, P). Measure on the first run and tag it; keep peak under 75 % of RAM; run nothing else heavy beside a generative run. A run lives only while a command runs. Hard cap 6 h. One generative run per box; the rest is split (above).
- Control: one conversation at claude.ai/code (Projects, Nox) starts and watches every thread. A heavy run uses the `run-part` agent; a PR gets the `review` agent before merge. Scheduled or on-demand reruns are routines (claude.ai/code, Routines).
