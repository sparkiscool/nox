# Session lifecycle and cleanup

Spark 24 Oct: no session stays alive or keeps getting updates after its work is over; cleanup runs on its own. Archiving is reversible (`unarchive_session`); it never deletes a branch, PR or file.

## Lifecycle
- **Start:** a session or worker has one job, one branch, and an owner row (`current-parts.md` System recheck or the part row) or a controller.
- **Done** = its PRs are merged or closed, its branch has no unmerged commits, nothing it started is still running (runs, workers, children), and its last reply says what is left for whom.
- **End:** a controller archives each child the turn it reports done or is superseded (CLAUDE.md Children stay in sync). A session that is not a child ends its last turn with the done line; the sweep below archives it.
- **Updates go only to live sessions:** before a `send_message`, `get_session` the target; never send to an ARCHIVED one, and never to one whose job is done (archive it instead).

## Sweep (scheduled every 6 h; any session may run it by hand)
1. `list_sessions` (mine, limit 100). Consider only sessions whose sources or outcomes name `sparkiscool/nox`, not ARCHIVED, not this session.
2. Archive a session when all hold:
   - not RUNNING, and last update over 6 h ago;
   - `status_bucket` is completed (never failed or blocked: those are reported, not archived);
   - every branch in its outcomes / current_branches is merged into `origin/main` or gone (`git rev-list --count origin/main..origin/<branch>` is 0 or the branch does not exist), and no open PR has that head branch;
   - it controls no live child (no non-archived session it created), and it is not tagged `keep`.
3. Report, never archive: a failed or blocked session, a session idle over 24 h with unmerged commits or an open PR (its owner decides), branches merged into main (`python3 tools/check_state.py --branches`; deleting needs Spark's Claude in Chrome prompt).
4. Previous sweep sessions are archived by the same rules.
5. Output: one `log.md` line only when something was archived or reported (`- <date> (session sweep): archived <n>: <titles>; report: <items>`), committed on a `chore/sweep-<date>` branch and merged with `tools/pr.py`. Nothing changed: no commit, no message.
