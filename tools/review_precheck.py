#!/usr/bin/env python3
"""The mechanical half of the review agent's PR mode (.claude/agents/review.md), so the opus reviewer only judges design.

python3 tools/review_precheck.py [--base origin/main] [--body <pr body file>]
Checks the branch's diff against the base (review items 1, 2, 6, 9, 10, 11, 12):
  1 solid    every print/ file the PR adds or changes is named with a solid_check PASS in a summary.md
  2 state    a design PR changes nox/current-parts.md and adds a new top line to nox/log.md
  6 preview  every print/ file has a preview PNG in that run folder, named in its summary.md
  9 state    tools/check_state.py shows no ERROR
  10 visual  a design PR links an artifact (claude.ai/.../artifact/...) in a changed summary.md or the PR body
  11 carbon  a Scout / Overseer / cage / mount PR (nox/scout, nox/overseer, runs or print scout_ overseer_ cage motor_mount) lists carbon (rod list with any non-3.0 stock sizes)
  12 scout rods  a PR that changes the Scout assembly: its cage shell is 3.0 rod and at most 50 rods (Spark 27 Oct)
Prints PASS / FAIL / skip per check; exit 1 on any FAIL. Reads only.
"""
import argparse, pathlib, re, subprocess, sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
ap = argparse.ArgumentParser()
ap.add_argument("--base", default="origin/main")
ap.add_argument("--body", help="PR body file")
a = ap.parse_args()
git = lambda *g: subprocess.run(["git", "-C", str(ROOT), *g], capture_output=True, text=True).stdout
changed = [f for f in git("diff", "--name-only", "--diff-filter=AMR", f"{a.base}...HEAD").split() if f]
prints = [f for f in changed if f.startswith("print/") and not f.endswith("README.md")]
design = [f for f in changed if f.startswith(("runs/", "print/")) or re.match(r"nox/(box|scout|overseer)/", f)]
sums = {f: (ROOT / f).read_text() for f in changed if f.endswith("summary.md") and (ROOT / f).exists()}
body = pathlib.Path(a.body).read_text() if a.body else ""
res = []


def out(item, ok, msg):
    res.append(ok)
    print(f"{'PASS' if ok else 'FAIL' if ok is False else 'skip'}  {item:10} {msg}")


if not design:
    out("all", None, "no runs/, print/ or part-folder change: tooling / docs PR, review items not applicable")
    sys.exit(0)

if prints:
    for p in prints:
        name = pathlib.Path(p).name
        hit = [s for s, t in sums.items() if name in t and (re.search(re.escape(name) + r".{0,40}PASS", t)
                                                          or "solid_check PASS" in t)]
        out("1 solid", bool(hit), f"{name}: " + (hit[0] if hit else "no changed summary.md names it with solid_check PASS"))
        pngs = [s for s in hit if re.search(r"preview[^\s`]*\.png", sums[s])
                and any(q.suffix == ".png" for q in (ROOT / s).parent.glob("preview*.png"))]
        out("6 preview", bool(pngs), f"{name}: " + ("preview PNG named in " + pngs[0] if pngs else "no preview*.png named in its summary"))
else:
    out("1 solid", None, "no print/ file in the PR")

base_log = git("show", f"{a.base}:nox/log.md")
top = next((l for l in (ROOT / "nox/log.md").read_text().splitlines() if l.startswith("- ")), "")
out("2 state", "nox/current-parts.md" in changed and top and top not in base_log,
    "row changed and a new top log.md line" if "nox/current-parts.md" in changed and top not in base_log
    else "needs the part's current-parts.md row and one new line at the top of nox/log.md")

cs = subprocess.run([sys.executable, str(ROOT / "tools/check_state.py")], capture_output=True, text=True)
out("9 state", cs.returncode == 0, cs.stdout.strip().splitlines()[-1] if cs.stdout.strip() else cs.stderr[-200:])

art = re.compile(r"claude\.ai/(?:code/)?artifact/\w+")
out("10 visual", bool(art.search(body) or any(art.search(t) for t in sums.values())),
    "artifact link found" if art.search(body) or any(art.search(t) for t in sums.values())
    else "no artifact link in a changed summary.md or the PR body")

drone = [f for f in design if re.match(r"nox/(scout|overseer)/|(runs|print)/(scout|overseer|cage|motor_mount)", f)]
if drone:
    ok = any(re.search(r"carbon", t, re.I) for t in sums.values())
    out("11 carbon", ok, "carbon list in summary" if ok else "drone / cage / mount PR without the carbon rods list (stock size and length of every non-3.0 rod) in summary.md")
else:
    out("11 carbon", None, "not a drone, cage or mount PR")

if "nox/scout/v3/scout.assembly.json" in changed:                     # Scout cage (Spark 27 Oct): every rod 3 mm, about half the rods
    import json as _j
    cs_ = _j.load(open(ROOT / "nox/scout/v3/scout.assembly.json")).get("cage_shell") or {}
    ok = abs(float(cs_.get("rod_r_mm", 0)) - 1.5) < 1e-6 and 0 < len(cs_.get("rods", [])) <= 50
    out("12 scout rods", ok, f"shell rods {len(cs_.get('rods', []))}, rod r {cs_.get('rod_r_mm')}" + ("" if ok else
        ": the Scout shell must be 3.0 rod (r 1.5) and at most 50 rods (half the full frequency-2 shell, Spark 27 Oct)"))
sys.exit(1 if False in res else 0)
