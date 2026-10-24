#!/usr/bin/env python3
"""Lint the project state so drift is caught by a command, not by a reader.

python3 tools/check_state.py            -> report; exit 1 on an ERROR (a File cell naming a missing file)
python3 tools/check_state.py --branches -> also list remote branches merged into main / idle with unmerged work

Checks:
  ERROR  a File cell in nox/current-parts.md names a print/ file that does not exist
  WARN   a print/ file no row and no print/README.md line names (orphan)
  WARN   a current-parts.md cell over CELL_MAX chars (history belongs in log.md, numbers in dimensions.md)
  WARN   one of the newest LOG_TOP log.md lines over LINE_MAX chars (details belong in the part doc / summary.md)
"""
import re, subprocess, sys, pathlib, time

ROOT = pathlib.Path(__file__).resolve().parents[1]
CELL_MAX = 600      # chars per current-parts.md cell
LINE_MAX = 400      # chars per log.md line
LOG_TOP = 10        # only the newest lines are held to LINE_MAX; older ones are history
FILE_RE = re.compile(r"`(?:print/)?([A-Za-z0-9_{},.\-]+\.(?:stl|3mf|obj|step|zip))`")


def expand(name):
    """box_finger_{A,B}_v4.stl -> both names."""
    m = re.search(r"\{([^}]*)\}", name)
    if not m:
        return [name]
    return [n for alt in m.group(1).split(",") for n in expand(name[:m.start()] + alt + name[m.end():])]


def rows():
    """(section, part, cells) for every table row of current-parts.md with a File column."""
    sec, head = "", None
    for line in (ROOT / "nox/current-parts.md").read_text().splitlines():
        if line.startswith("## "):
            sec, head = line[3:], None
        elif line.startswith("|") and not set(line) <= set("|-: "):
            cells = [c.strip() for c in line.strip("|").split("|")]
            if head is None:
                head = cells
            else:
                yield sec, head, cells


def main(branches=False):
    err, warn = [], []
    printed = {p.name for p in (ROOT / "print").iterdir() if p.name != "README.md"}
    named = set()
    for sec, head, cells in rows():
        part = cells[0][:50]
        for h, c in zip(head, cells):
            if len(c) > CELL_MAX:
                warn.append(f"{sec} / {part}: {h} cell {len(c)} chars (max {CELL_MAX})")
        if "File (`print/`)" in head:
            cell = cells[head.index("File (`print/`)")]
            for raw in FILE_RE.findall(cell):
                for n in expand(raw):
                    named.add(n)
                    if n not in printed:
                        err.append(f"{sec} / {part}: File cell names print/{n}, not in print/")
    readme = (ROOT / "print/README.md").read_text()
    for n in sorted(printed - named):
        if n not in readme:                        # README.md accounts for superseded / do-not-print files
            warn.append(f"orphan print/{n} (no current-parts row, not in print/README.md)")
    log = [l for l in (ROOT / "nox/log.md").read_text().splitlines() if l.startswith("- ")][:LOG_TOP]
    for l in log:
        if len(l) > LINE_MAX:
            warn.append(f"log.md line {len(l)} chars (max {LINE_MAX}): {l[:70]}...")
    for e in err:
        print("ERROR", e)
    for w in warn:
        print("WARN ", w)
    if branches:
        print_branches()
    print(f"check_state: {len(err)} error(s), {len(warn)} warning(s)")
    return 1 if err else 0


def print_branches():
    git = lambda *a: subprocess.run(["git", "-C", str(ROOT), *a], capture_output=True, text=True).stdout
    git("fetch", "-q", "--prune", "origin")
    now = time.time()
    for b in git("branch", "-r", "--format=%(refname:short)").split():
        if b in ("origin/main", "origin/HEAD", "origin"):
            continue
        ahead = int(git("rev-list", "--count", f"origin/main..{b}").strip() or 0)
        age_h = (now - int(git("log", "-1", "--format=%ct", b).strip())) / 3600
        if ahead == 0:
            print(f"BRANCH merged, safe to delete: {b}")
        elif age_h > 24:
            print(f"BRANCH idle {age_h:.0f} h with {ahead} unmerged commit(s): {b} (owner decides: PR, merge or archive)")


if __name__ == "__main__":
    sys.exit(main(branches="--branches" in sys.argv))
