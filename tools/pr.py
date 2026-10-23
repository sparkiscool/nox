#!/usr/bin/env python3
"""Open and merge Nox pull requests over the GitHub REST API.

Cloud boxes block GitHub GraphQL, so every `gh pr ...` subcommand fails there (403).
`gh api` (REST) works, and this wraps the two calls a session needs:

    python3 tools/pr.py open "<part> <version>: <what>" [body.md]   # head = current branch, base = main
    python3 tools/pr.py merge <n>                                    # merge commit, then delete the head branch

Prints the PR URL (open) or "merged <n> (<sha>), deleted <branch>" (merge). Exit 1 on any failure.
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


def cmd_merge(n):
    pr = json.loads(gh(f"repos/{REPO}/pulls/{n}"))
    if pr["state"] != "open":
        sys.exit(f"merge: PR {n} is {pr['state']}")
    head = pr["head"]["ref"]
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
    elif len(a) == 2 and a[0] == "merge" and a[1].isdigit():
        cmd_merge(a[1])
    else:
        sys.exit(__doc__)
