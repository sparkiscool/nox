#!/usr/bin/env python3
"""Open and merge Nox pull requests over the GitHub REST API.

Cloud boxes block GitHub GraphQL, so every `gh pr ...` subcommand fails there (403).
`gh api` (REST) works, and this wraps the two calls a session needs:

    python3 tools/pr.py open "<part> <version>: <what>" [body.md]   # head = current branch, base = main
    python3 tools/pr.py merge <n> [--no-wait]                        # merge commit, then delete the head branch

Prints the PR URL (open) or "merged <n> (<sha>), deleted <branch>" (merge). Exit 1 on any failure.
merge is gated on CI (.github/workflows/checks.yml): it reads the check runs on the PR head commit and refuses while any
failed; while any is pending it polls every 30 s for up to 15 min (--no-wait: refuse at once). No check runs at all
(workflow not on main yet, or it never started): it waits up to 2 min for them if the workflow is on main, then merges
with a warning.
Branch deletion is refused from cloud boxes both ways (git push --delete and REST DELETE ref, proxy 403), as is
the repo-settings write that would turn on auto-delete. merge tries the delete (works on a PC), then checks the
branch and prints the true outcome; the repo setting "Automatically delete head branches" covers cloud merges.
"""
import json
import subprocess
import sys
import time

REPO = "sparkiscool/nox"


def gh(*args, check=True):
    r = subprocess.run(["gh", "api", *args], text=True, capture_output=True)
    if check and r.returncode:
        sys.exit(f"gh api {' '.join(args)}\n{r.stderr.strip() or r.stdout.strip()}")
    return r.stdout


def git(*args):
    return subprocess.run(["git", *args], text=True, capture_output=True, check=True).stdout.strip()


def cmd_open(title, body_file=None):
    head = git("branch", "--show-current")
    if not head or head == "main":
        sys.exit("open: check out the part/ or run/ branch first")
    args = ["-X", "POST", f"repos/{REPO}/pulls", "-f", f"title={title}", "-f", f"head={head}", "-f", "base=main"]
    if body_file:
        args += ["-F", f"body=@{body_file}"]
    pr = json.loads(gh(*args))
    print(pr["html_url"])


OK_CONCLUSIONS = {"success", "neutral", "skipped"}
POLL_S, WAIT_S, NONE_WAIT_S = 30, 15 * 60, 2 * 60


def check_runs(sha):
    """[(name, status, conclusion)] of every check run on commit sha."""
    out = json.loads(gh(f"repos/{REPO}/commits/{sha}/check-runs?per_page=100"))
    return [(c["name"], c["status"], c.get("conclusion")) for c in out.get("check_runs", [])]


def wait_for_ci(n, wait=True):
    """return when every check run on the PR head passed (or there are none); sys.exit while any failed or stays pending."""
    has_workflow = subprocess.run(["gh", "api", f"repos/{REPO}/contents/.github/workflows/checks.yml?ref=main"],
                                  text=True, capture_output=True).returncode == 0
    t0 = time.time()
    while True:
        sha = json.loads(gh(f"repos/{REPO}/pulls/{n}"))["head"]["sha"]
        runs = check_runs(sha)
        failed = [f"{a} ({c})" for a, s, c in runs if s == "completed" and c not in OK_CONCLUSIONS]
        pending = [a for a, s, c in runs if s != "completed"]
        if failed:
            sys.exit(f"merge: refused, CI failed on {sha[:7]}: {', '.join(failed)}. Fix and push, then merge again.")
        if runs and not pending:
            print(f"CI green on {sha[:7]}: {len(runs)} check run(s)")
            return
        waited = time.time() - t0
        if not runs and (not has_workflow or waited >= NONE_WAIT_S):
            print(f"merge: WARNING no check runs on {sha[:7]}"
                  f"{'' if has_workflow else ' (.github/workflows/checks.yml not on main yet)'}; merging without CI")
            return
        if not wait:
            sys.exit(f"merge: refused, CI pending on {sha[:7]}: {', '.join(pending) or 'not started'} (--no-wait)")
        if waited >= WAIT_S:
            sys.exit(f"merge: refused, CI still pending after {WAIT_S // 60} min on {sha[:7]}: {', '.join(pending)}")
        print(f"CI pending on {sha[:7]}: {', '.join(pending) or 'not started'}; next check in {POLL_S} s", flush=True)
        time.sleep(POLL_S)


def cmd_merge(n, wait=True):
    pr = json.loads(gh(f"repos/{REPO}/pulls/{n}"))
    if pr["state"] != "open":
        sys.exit(f"merge: PR {n} is {pr['state']}")
    head = pr["head"]["ref"]
    wait_for_ci(n, wait)
    res = json.loads(gh("-X", "PUT", f"repos/{REPO}/pulls/{n}/merge", "-f", "merge_method=merge"))
    if not res.get("merged"):
        sys.exit(f"merge: {res.get('message', res)}")
    sha = (res.get("sha") or "")[:7]
    # branch delete: see the docstring
    subprocess.run(["git", "push", "-q", "origin", "--delete", head], text=True, capture_output=True)
    for _ in range(10):  # the repo's auto-delete lands a few seconds after the merge
        gone = subprocess.run(["gh", "api", f"repos/{REPO}/branches/{head}"], text=True, capture_output=True).returncode != 0
        if gone:
            break
        time.sleep(2)
    if gone:
        print(f"merged {n} ({sha}), deleted {head}")
    else:
        print(f"merged {n} ({sha}); {head} left on origin (cloud boxes cannot delete branches: "
              "turn on Settings -> General -> Automatically delete head branches once)")


if __name__ == "__main__":
    a = sys.argv[1:]
    if len(a) >= 2 and a[0] == "open":
        cmd_open(a[1], a[2] if len(a) > 2 else None)
    elif len(a) in (2, 3) and a[0] == "merge" and a[1].isdigit() and a[2:] in ([], ["--no-wait"]):
        cmd_merge(a[1], wait="--no-wait" not in a)
    else:
        sys.exit(__doc__)
