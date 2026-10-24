#!/usr/bin/env python3
"""Evaluate a saved list_sessions result against the sweep rules (docs/sessions.md Sweep).

python3 tools/sweep_eval.py <list_sessions output file> [self_session_id]
Prints one line per non-archived nox session: ARCHIVE (all sweep conditions hold), REPORT (failed / blocked, or idle
over 24 h with unmerged work or an open PR) or keep, with age, bucket, tags and each branch's unmerged count.
Only reads; archiving is done with archive_session by the session running the sweep.
"""
import datetime, json, subprocess, sys

src, me = sys.argv[1], (sys.argv[2] if len(sys.argv) > 2 else "")
t = open(src).read()
d, _ = json.JSONDecoder().raw_decode(t[t.index('{"ccr"'):])
now = datetime.datetime.now(datetime.timezone.utc)
git = lambda *a: subprocess.run(["git", *a], capture_output=True, text=True).stdout
git("fetch", "-q", "--prune", "origin")
remote = set(git("branch", "-r", "--format=%(refname:short)").split())
try:
    prs = {p["head"]["ref"] for p in json.loads(subprocess.run(
        ["gh", "api", "repos/sparkiscool/nox/pulls?state=open&per_page=100"], capture_output=True, text=True).stdout)}
except Exception:
    prs = set()
for x in d["ccr"]["data"]:
    status = x.get("session_status", "").replace("SESSION_STATUS_", "")
    sc = x.get("session_context") or {}
    if status == "ARCHIVED" or x["id"] == me or "sparkiscool/nox" not in json.dumps(sc):
        continue
    age = (now - datetime.datetime.fromisoformat(x["updated_at"].replace("Z", "+00:00"))).total_seconds() / 3600
    bucket = x.get("status_bucket", "").replace("SESSION_STATUS_BUCKET_", "")
    tags = x.get("tags") or []
    branches = {b for o in sc.get("outcomes") or [] for b in o.get("git_repository", {}).get("git_info", {}).get("branches", [])}
    branches |= set(((x.get("external_metadata") or {}).get("current_branches") or {}).values())
    unmerged, info = False, []
    for b in sorted(b for b in branches if b and b != "main"):
        if f"origin/{b}" in remote:
            n = int(git("rev-list", "--count", f"origin/main..origin/{b}").strip() or 0)
            unmerged |= n > 0 or b in prs
            info.append(f"{b}:+{n}{' PR' if b in prs else ''}")
        else:
            info.append(f"{b}:gone")
    if bucket in ("FAILED", "BLOCKED") or (age > 24 and unmerged):
        verdict = "REPORT"
    elif status != "RUNNING" and age > 6 and bucket == "COMPLETED" and not unmerged and "keep" not in tags:
        verdict = "ARCHIVE"   # live-child check: the sweeping session confirms it controls no non-archived session
    else:
        verdict = "keep"
    print(f"{verdict:7} {x['id']} {status} {bucket} {age:.1f}h | {(x.get('title') or '')[:45]} | {' '.join(info)}")
