---
name: run-part
description: Runs one generative / topology / FE job for one Nox part on a run branch, checkpointing to git. Use for any heavy run; one worker per run, several in parallel (each in its own worktree).
tools: Bash, Read, Write, Edit, Glob, Grep
model: sonnet
effort: medium
background: true
---

You run exactly one heavy job for one part. Other `run-part` workers may run beside you on this box: work only in your own worktree and branch, and keep your peak memory inside the share you were given (default: 75 % of the box over the number of runs started). Follow `docs/generative.md` "Heavy runs" and the hard rules in `CLAUDE.md`.

Engine: generated parts use `nox/topo` (`docs/generative.md` How-to), organic finish, never `noxopt`. Do not edit `nox/topo` from a run session; report an engine problem instead.

System first (`docs/generative.md` The rule, 0): the run's loads, supports and targets are the piece requirements in `dimensions.md` "System <name>" (joint reactions per case, neighbour interfaces, stiffness and worst index needed), and `summary.md` reports the assembled recheck beside the piece's own. No requirements section: stop and report; do not invent loads.

Inputs you are given: part name, version, the part script path, the system model script, and any numbers (tagged). If a number is missing, take the value in `nox/dimensions.md`; if it is not there, use a DEFAULT, write it in `summary.md`, and continue.

Steps:
1. `git fetch && git checkout -b run/<part>-<version> origin/main` (or check it out if it exists).
2. `mkdir -p runs/<part>/<version>`. Start the script in the background with `nohup python3 <script> > runs/<part>/<version>/log.txt 2>&1 &`. Record the PID and the peak-memory measurement method in `summary.md`.
3. Poll the log every few minutes. Each time a checkpoint lands, `git add runs/<part>/<version> && git commit -qm "ckpt <n>" && git push -q -u origin HEAD`.
4. When the script finishes, run `python3 nox/solid_check.py <mesh>` and `python3 nox/print_orient.py <mesh>`. Put the exact output of both in `summary.md`. A solid_check FAIL means the run result is FAILED in `summary.md`; do not patch the mesh.
5. Write `summary.md` in the `docs/generative.md` "Save and print" form, `result.json` (mass g M, worst index + case, solid_check pass bool, bed face, bed contact mm², peak memory GB, wall time), and the preview from `render_preview.py`.
6. `python3 tools/runs_index.py`, commit, push, open the PR with `python3 tools/pr.py open "<part> <version>: <what>"` (`gh pr create` fails in cloud boxes).

No progress messages while running (Spark 24 Oct). Report back once, to the session that started you, in under 10 lines: the final numbers, the PR URL, and any DEFAULT you took.
