# Session lifecycle and cleanup

Spark 24 Oct: no session stays alive or keeps getting updates after its work is over; cleanup runs on its own. Archiving is reversible (`unarchive_session`); it never deletes a branch, PR or file.

## Lifecycle
- **Start:** a session or worker has one job, one branch, and an owner row (`current-parts.md` System recheck or the part row) or a controller.
- **Done** = its PRs are merged or closed, its branch has no unmerged commits, nothing it started is still running (runs, workers, children), and its last reply says what is left for whom.
- **End:** a controller archives each child the turn it reports done or is superseded (CLAUDE.md Children stay in sync). A session that is not a child ends its last turn with the done line; the sweep below archives it.
- **Controllers:** before reporting, a controller checks each child (`get_session`) and says its state in one line; a child that finishes or is superseded is archived the same turn.
- **Updates go only to live sessions:** before a `send_message`, `get_session` the target; never send to an ARCHIVED one, and never to one whose job is done (archive it instead).

## Hub and reporting (Spark 24 Oct: no status updates from running parts; one session reports; 24 Oct later: only for print-ready files or his decisions)
- **Hub:** the control session, "Parallel session execution with workers" (`session_01BEtJ8Y7dCsuQ6ko5SnvzNr`). If it is archived, the newest running session titled as the control session is the hub; a new hub writes its id here in its first PR.
- **Everyone else reports to the hub, never to Spark:** a worker, run or child session sends the hub one `send_message` when it finishes (PR link, the summary block, DEFAULTs taken), fails (what and why) or is blocked (what it needs). No progress pings, no push notifications, no proactive `SendUserFile`. A session Spark talks to directly answers him when he writes, and still sends its results to the hub.
- **Reporting (Spark 24 Oct: "only use tokens when absolutely needed"):** no timed digests and no send_later check-ins. The hub messages Spark only when a file is ready to print (pictures + slicing prompt) or a decision of his is needed (options, DEFAULT, visual). Child reports are acted on (routed, reviewed, merged) without messaging Spark. The board is refreshed by the organiser.
- **Board:** with each digest the hub refreshes the Nox Board (https://claude.ai/artifact/NKBNkPUSyKgwYrtZZBhMkw): `python3 tools/board_data.py <dir>`, upload each JSON as an asset of the board (Artifact publish, `asset: true`, `url` the board), then one ArtifactData `update` per changed `datasets/<name>` with the new `source.url` and `updated: {at, by}`.

## Sweep (every 6 h as part of the organiser run, `docs/organiser.md`; any session may run it by hand)
1. `list_sessions` (mine, limit 100). Consider only sessions whose sources or outcomes name `sparkiscool/nox`, not ARCHIVED, not this session.
2. Archive a session when all hold:
   - not RUNNING, and last update over 6 h ago;
   - `status_bucket` is completed (never failed or blocked: those are reported, not archived);
   - every branch in its outcomes / current_branches is merged into `origin/main` or gone (`git rev-list --count origin/main..origin/<branch>` is 0 or the branch does not exist), and no open PR has that head branch;
   - it controls no live child (no non-archived session it created), and it is not tagged `keep`.
3. Report, never archive: a failed or blocked session, a session idle over 24 h with unmerged commits or an open PR (its owner decides), branches merged into main (`python3 tools/check_state.py --branches`; deleting needs Spark's Claude in Chrome prompt).
4. Previous sweep sessions are archived by the same rules.
5. Output: one `log.md` line only when something was archived or reported (`- <date> (session sweep): archived <n>: <titles>; report: <items>`), committed on a `chore/sweep-<date>` branch and merged with `tools/pr.py`. Nothing changed: no commit, no message.
