# Session lifecycle and cleanup

Spark 24 Oct: no session stays alive or keeps getting updates after its work is over; cleanup runs on its own. Archiving is reversible (`unarchive_session`); it never deletes a branch, PR or file.

## Lifecycle
- **Start:** a session or worker has one job, one branch, and an owner row (`current-parts.md` System recheck or the part row) or a controller.
- **Done** = its PRs are merged or closed, its branch has no unmerged commits, nothing it started is still running (runs, workers, children), and its last reply says what is left for whom.
- **End:** a controller archives each child the turn it reports done or is superseded (CLAUDE.md Children stay in sync). A session that is not a child ends its last turn with the done line; the sweep below archives it.
- **Controllers:** before reporting, a controller checks each child (`get_session`) and says its state in one line; a child that finishes or is superseded is archived the same turn.
- **Updates go only to live sessions:** before a `send_message`, `get_session` the target; never send to an ARCHIVED one, and never to one whose job is done (archive it instead).

## Sessions, workers and reporting (Spark 24 Oct: no hub or digest session; "only use tokens when absolutely needed")
- **One task, one session.** The session Spark starts for a task owns it end to end: design, review agent, merge on PASS (CLAUDE.md step 4).
- **Workers inside it.** Parallel subtasks go to Agent-tool workers in the same session (`isolation: "worktree"` for anything that commits). They share the box (4 cores, 16 GB) and stop if its container restarts, so they commit and push every step.
- **A new cloud session only when needed:** a heavy run (hours, or more memory than the box can share) or a box already full: then at once, in the same turn as the runs that fit here (CLAUDE.md Parallel by default). Its brief is standalone; it merges its own PR after the review agent's PASS and is archived when done.
- **Reporting (Spark 24 Oct: "only notifications when something is done or needs me"):** a session messages or notifies Spark only when its task is done (one line: what, PR, what is left), a file is ready to print (pictures + slicing prompt) or a decision of his is needed (options, DEFAULT, visual). Nothing else: no digests, progress pings, "started" notes, timed check-ins or push notifications. A child session puts its result in its own final reply and, if its parent is live, one send_message to the parent.
- **Cage sessions keep their parts in sync (Spark 27 Oct: "let every cage chat know it should message all sessions making its parts and replace the artifact with the newest parts everytime").** A session that owns a drone's cage (Overseer cage, Scout v3) keeps a list of the sessions making that cage's parts (nodes, sensor head, arms, core, funnel). Every change to the cage or its links (geometry, link ends, sizes, loads, bores, materials) goes to each of those sessions in one message, the turn it is pushed. Each part session tells the cage session the turn it saves a part (file, commit, M mass). The cage session then puts the newest saved part of each piece into its assembly and republishes its artifact (same URL), so the artifact always shows the newest parts. Check: `review` item 15.
- **Board:** the organiser refreshes the Nox Board (https://claude.ai/artifact/NKBNkPUSyKgwYrtZZBhMkw) on its 6 h run: `python3 tools/board_data.py <dir>`, upload each JSON as an asset, then one ArtifactData `update` per `datasets/<name>`.

## Usage (Spark 24 Oct: "use tokens only when needed"; CLAUDE.md Small contexts)
Every tool call re-sends the whole context: a 700k session pays 700k tokens per call, and a wake after the 1 h cache rewrites all of it. 24 Oct audit (`docs/usage-plan.html`): $1,446 in 42 h, ~85 % in 11 opus sessions at 470-730k.
- **Cap:** `.claude/settings.json` `autoCompactWindow` 300k: a session compacts there. State lives in files (row, part doc, `summary.md`, pushed every turn), so a compact loses nothing; a session past ~250k that starts a new phase ends with a handoff line in its part doc.
- **Loops in workers:** CAD iterations, FE or topo retries, fix-and-rerun cycles, sweeps over many files and big outputs go to one Agent worker (fresh context; `run-part` on sonnet for runs) that returns ≤ 10 lines. The orchestrator decides; it never runs a long loop or reads a big log itself.
- **One wake per run:** a heavy job runs under `python3 tools/run_bg.py runs/<part>/<version> -- python3 <script>` started with Bash `run_in_background`: it commits checkpoints itself and the starter is woken once, when it exits. No polling, no sleep loops, no "still running" turns; several jobs that feed one decision run under one worker.
- **Sync by pull:** children merge `origin/main` every step. A controller messages a live child only for a change to that child's own inputs or task (a Spark decision on it, a stop or supersede), all of a turn's changes in one message, `priority: "later"`. A shared number changed elsewhere is not a message: the child reads it on its next merge.
- **Model on every spawn:** `create_session` copies the caller's model unless told: a heavy run passes `model` sonnet, a design child opus. The sweep reports an opus heavy-run session (`tools/sweep_eval.py`).
- **Read slices:** `python3 tools/dim.py <section>` (never all 160 KB of `dimensions.md`), `head` of `log.md`, the one row of `current-parts.md`. New `dimensions.md` lines stay ≤ 600 chars (`check_state` warns).
- **Review:** `python3 tools/review_precheck.py` first; a FAIL goes straight back to the author, no opus reviewer. The reviewer judges the rest; a re-review checks only the last findings and what changed.

## Sweep (every 12 h as part of the organiser run, `docs/organiser.md`; any session may run it by hand)
1. `list_sessions` (mine, limit 100). Consider only sessions whose sources or outcomes name `sparkiscool/nox`, not ARCHIVED, not this session. `python3 tools/sweep_eval.py <saved list_sessions output> <this session id>` applies the rules below and prints ARCHIVE / REPORT / keep per session.
2. Archive a session when all hold:
   - not RUNNING, and last update over 6 h ago;
   - `status_bucket` is completed (never failed or blocked: those are reported, not archived);
   - every branch in its outcomes / current_branches is merged into `origin/main` or gone (`git rev-list --count origin/main..origin/<branch>` is 0 or the branch does not exist), and no open PR has that head branch;
   - it controls no live child (no non-archived session it created), and it is not tagged `keep`.
3. Report, never archive: a failed or blocked session, a session idle over 24 h with unmerged commits or an open PR (its owner decides), branches merged into main (`python3 tools/check_state.py --branches`; deleting needs Spark's Claude in Chrome prompt).
4. Previous sweep sessions are archived by the same rules.
5. Output: one `log.md` line only when something was archived or reported (`- <date> (session sweep): archived <n>: <titles>; report: <items>`), committed on a `chore/sweep-<date>` branch and merged with `tools/pr.py`. Nothing changed: no commit, no message.
