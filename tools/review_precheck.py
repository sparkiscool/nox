#!/usr/bin/env python3
"""The mechanical half of the review agent's PR mode (.claude/agents/review.md), so the opus reviewer only judges design.

python3 tools/review_precheck.py [--base origin/main] [--body <pr body file>]
Checks the branch's diff against the base (review items 1, 2, 6, 9, 10, 11, 12, 13, 14, 16):
  1 solid    every print/ file the PR adds or changes is named with a solid_check PASS in a summary.md
  2 state    a design PR changes nox/current-parts.md and adds a new top line to nox/log.md
  6 preview  every print/ file has a preview PNG in that run folder, named in its summary.md
  9 state    tools/check_state.py shows no ERROR
  10 visual  a design PR links an artifact (claude.ai/.../artifact/...) in a changed summary.md or the PR body
  11 carbon  a Scout / Overseer / cage / mount PR (nox/scout, nox/overseer, runs or print scout_ overseer_ cage motor_mount) lists carbon (rod list with any non-3.0 stock sizes)
  12 parallel the runs the PR adds started within 30 min of each other (run_bg.txt start lines), or the later run's summary.md says `Waited for:`
  13 scout rods  a PR that changes the Scout assembly: every cage rod 3.0 (shell and links) and at most 62 rods in all (cage_shell total_rods; Spark 27 Oct)
  14 estimate  every topo run the PR adds that started after 2026-10-27T08:00Z (run_bg.txt) has estimate.json from tools/run_estimate.py (Spark 27 Oct);
              after 2026-10-28T00:00Z its room audit must read PASS (loaded drawn features on the design room, Spark 27 Oct)
  16 drawn    a topo run whose part's previous version FAILED may not grow the drawn mass (result.json drawn_g) by > 5 % and > 1 g unless
              its summary.md has a `Drawn added:` line naming each added feature (the review checks each is an attachment; Spark 27 Oct)
Prints PASS / FAIL / skip per check; exit 1 on any FAIL. Reads only.
"""
import argparse, json, pathlib, re, subprocess, sys

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

import datetime as _dt
starts = {}
for f in changed:
    if f.endswith("run_bg.txt") and (ROOT / f).exists():
        m = re.search(r"run_bg: start (\S+?)Z", (ROOT / f).read_text())
        if m:
            starts[str(pathlib.Path(f).parent)] = _dt.datetime.fromisoformat(m.group(1))
if len(starts) >= 2:
    first = min(starts.values())
    late = [d for d, t in starts.items() if (t - first).total_seconds() > 1800
            and "Waited for:" not in ((ROOT / d / "summary.md").read_text() if (ROOT / d / "summary.md").exists() else "")]
    out("12 parallel", not late, "runs started together" if not late else
        f"{', '.join(late)} started over 30 min after the first run with no `Waited for:` line (CLAUDE.md Parallel by default)")
else:
    out("12 parallel", None, "fewer than 2 runs with a start time in the PR")

if "nox/scout/v3/scout.assembly.json" in changed:                     # Scout cage (Spark 27 Oct): every rod 3 mm, about half the rods
    import json as _j
    cs_ = _j.load(open(ROOT / "nox/scout/v3/scout.assembly.json")).get("cage_shell") or {}
    tot = int(cs_.get("total_rods", 0))
    ok = abs(float(cs_.get("rod_r_mm", 0)) - 1.5) < 1e-6 and abs(float(cs_.get("link_r_mm", 0)) - 1.5) < 1e-6 and 0 < tot <= 62
    out("13 scout rods", ok, f"shell {len(cs_.get('rods', []))} + links {cs_.get('links')} = {tot} rods, r {cs_.get('rod_r_mm')} / {cs_.get('link_r_mm')}"
        + ("" if ok else ": every Scout rod 3.0 (r 1.5) and at most 62 in all (half of cage v1's 123, Spark 27 Oct)"))
RULE14 = _dt.datetime(2026, 10, 27, 8, 0)                             # Measure before quoting (Spark 27 Oct)
topo_runs = [d for d, t in starts.items() if t >= RULE14 and "=== " in ((ROOT / d / "log.txt").read_text() if (ROOT / d / "log.txt").exists() else "")]
if topo_runs:
    miss = [d for d in topo_runs if not (ROOT / d / "estimate.json").exists()]
    RULE14B = _dt.datetime(2026, 10, 28, 0, 0)                          # the room audit (Spark 27 Oct: drawn filler on hub deck v7 / v9)
    miss += [f"{d} (room audit not PASS)" for d in topo_runs if starts[d] >= RULE14B and (ROOT / d / "estimate.json").exists()
             and json.loads((ROOT / d / "estimate.json").read_text()).get("room_audit") != "PASS"]
    out("14 estimate", not miss, f"{len(topo_runs)} topo run(s) measured" if not miss else
        f"{', '.join(miss)}: no estimate.json (tools/run_estimate.py before the run; CLAUDE.md Measure before quoting)")
else:
    out("14 estimate", None, "no topo run started after the rule in the PR")
# 16 drawn growth after a failed version (Spark 27 Oct: "the issue that happened here with the extra materials cannot happen again")
def _vnum(v):
    m_ = re.match(r"v(\d+)([a-z]?)$", v); return (int(m_.group(1)), m_.group(2)) if m_ else None
def _read(part, ver, name):
    f_ = ROOT / "runs" / part / ver / name
    if f_.exists(): return f_.read_text()
    return subprocess.run(["git", "-C", str(ROOT), "show", f"origin/run/{part}-{ver}:runs/{part}/{ver}/{name}"], capture_output=True, text=True).stdout
grown = []; seen16 = 0
for f in changed:
    m = re.match(r"runs/([^/]+)/(v\d+[a-z]?)/result\.json$", f)
    if not m or not (ROOT / f).exists(): continue
    part, ver = m.groups(); cur = json.loads((ROOT / f).read_text()).get("drawn_g")
    if cur is None or _vnum(ver) is None: continue
    vers = {d.name for d in (ROOT / "runs" / part).iterdir() if d.is_dir()}
    vers |= {b.split(f"origin/run/{part}-")[1] for b in git("branch", "-r", "--list", f"origin/run/{part}-*").split()}
    prev = sorted([v for v in vers if _vnum(v) and _vnum(v)[0] < _vnum(ver)[0] and _read(part, v, "result.json").strip()], key=_vnum)
    if not prev: continue
    pv = prev[-1]; seen16 += 1
    p_res = json.loads(_read(part, pv, "result.json")); p_sum = _read(part, pv, "summary.md").split("\n")[0]
    if "FAILED" not in p_sum: continue
    pd_ = p_res.get("drawn_g") or 0.0
    if cur > pd_ * 1.05 and cur - pd_ > 1.0 and "Drawn added:" not in ((ROOT / "runs" / part / ver / "summary.md").read_text() if (ROOT / "runs" / part / ver / "summary.md").exists() else ""):
        grown.append(f"{part} {ver}: drawn {cur:.1f} g after {pv} FAILED at {pd_:.1f} g")
if seen16:
    out("16 drawn", not grown, "no drawn growth after a failure" if not grown else
        "; ".join(grown) + " (fix a failed run in its setup: room, cuts, loads, volume; a real new attachment goes on a `Drawn added:` line)")
else:
    out("16 drawn", None, "no topo run with a previous version in the PR")
sys.exit(1 if False in res else 0)
