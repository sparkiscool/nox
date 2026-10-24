# Organiser (runs on its own)

Spark 24 Oct: "there should always be an organization agent occasionally without me ... I need the project to not get complicated and die." The scheduled task "Nox organiser (sweep + tidy)" fires every 12 h (24 Oct, was 6 h; paused 24 Oct 20:00 while Spark studies: he re-enables it) into the organiser session, "Project workflow and organization" (`session_011MGpg8zx6nzatcVg46CaCK`, tagged `keep` and `nox-organiser`), which has the repo and the session tools (a fresh session has neither: tested 24 Oct). If that session is archived or gone, any session that notices creates a new organiser session (`create_session` on this repo, tags `keep`, `nox-organiser`) and a 12 h scheduled task into it with the same prompt, and writes the new id here. Every run does the light pass; the first run after 20 h without a full pass (see the newest `(organiser)` line in `nox/log.md`) also does the full pass. Model opus, effort high (the session's own; a persistent session cannot switch per run); its context is capped at 300k (`docs/sessions.md` Usage).

## Limits (never)
- Never edit a part's design, script, mesh, row cells or `dimensions.md` numbers: those belong to their owner (`current-parts.md` System recheck). What an owner must fix goes to that owner session (one send_message, only if it is live) and in this run's report.
- Never delete work on `main` alone beyond what a rule already allows (`docs/sessions.md`, the repo slim rule in CLAUDE.md Git), never force-push, never rewrite history, never close another session's PR, never spend money.
- Never message Spark directly: one short reply in this session's own transcript is the report (`docs/sessions.md`); only a decision of Spark's or a print-ready file goes to him.
- Never change a rule's meaning. Merging duplicates, shortening and moving text to its one place are allowed; a rule that looks wrong is reported, not changed.

## Light pass (every run, ~10 min)
0. **Cheap exit:** `git log -1 --format=%cI origin/main` and one `list_sessions` call: no commit on main and no nox session updated since the newest `(organiser)` log line or this session's last run, and no full pass due: reply "nothing new" in one line and stop. Otherwise hand steps 1-5 to one `general-purpose` Agent worker on `model: sonnet` with this file's path; it returns ≤ 8 lines and the organiser only archives (step 2) and reports.
1. `git fetch origin`; work on a branch `chore/organiser-<date>-<hh>` from `origin/main`.
2. Session sweep: `docs/sessions.md` Sweep, exactly.
3. `python3 tools/check_state.py --branches`. Fix what is the organiser's own: a File cell pointing at a missing file that exists under another name in `print/` (report it, the owner fixes the cell), orphan print files (add the do-not-print line to `print/README.md` when the row says superseded).
4. Refresh the Nox Board if the Artifact and ArtifactData tools are available (`docs/sessions.md` Board); else skip and say so.
5. Commit only if something changed; one `nox/log.md` line `- <date> (organiser): ...` ≤ 300 chars; open and merge with `tools/pr.py` (docs / tooling PR).

## Full pass (once a day)
1. **Rules in one place.** Read CLAUDE.md Hard rules, `docs/generative.md` The rule, `docs/sessions.md`, `.claude/agents/*.md`, `.claude/skills/*/SKILL.md`. Any rule text restated outside its one place: replace the copy with a pointer (CLAUDE.md step 6). Two rules that say the same thing: merge into the newer wording. Contradictions: report them, change nothing.
2. **CLAUDE.md stays short.** Over ~10 KB or a rule over ~5 lines: move method detail to `docs/generative.md` / `docs/sessions.md` and leave the one-line rule.
3. **Ownership.** Every part row has one owner in System recheck; every live session maps to one owner. Two live sessions on the same part or system, or a part with no owner: report it (owner session if live, else this run's report) with the suggested owner.
4. **Stale work.** Open PRs with no push for 24 h, branches idle 24 h with unmerged commits, `DESIGN` rows whose Next cell has not changed in 3 days, three-strikes candidates (third failed version in a row): report each with its owner.
5. **Docs drift.** Part docs that the row says are out of date ("still describe superseded versions", "update on the next turn"): report to the owner. Root `README.md` map: fix broken paths.
6. **Weight.** `du -sh runs print archive .git`; a retired run folder with meshes over 5 MB that matches the slim rule: slim it the same way (RETIRED.md with the recovery commit) in this PR.
7. **Log.** `nox/log.md` over 200 KB: move lines older than 14 days to `archive/log-<yyyy-mm>.md` (moved, not deleted) and say so in the log line.
8. Report in this session's own final reply (≤ 12 lines): what was tidied (PR link), what each owner must do, what needs Spark.
