#!/usr/bin/env python3
"""Build the data files behind the Nox Board dashboard (https://claude.ai/artifact/NKBNkPUSyKgwYrtZZBhMkw).

python3 tools/board_data.py [out_dir]     -> writes parts.json, spark.json, log.json, runs.json, prs.json (default board/)

parts: one row per current-parts.md part row (system, part, version, state, ready file, mass, next step)
spark: one row per item Spark owns (Measure / On hand / Decide / Buy / Tests / Later)
log:   the newest 15 log.md lines (date, who, what)
runs:  latest run per part from runs/*/*/result.json
prs:   open PRs (GitHub REST via `gh api`; empty when offline)
Refresh steps: docs/sessions.md "Board".
"""
import json, pathlib, re, subprocess, sys, datetime

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from check_state import rows  # noqa: E402

BOLD = re.compile(r"\*\*(.*?)\*\*")


def plain(s, n=None):
    s = BOLD.sub(r"\1", s).replace("`", "").strip()
    return s if n is None or len(s) <= n else s[: n - 1].rstrip() + "…"


def parts():
    out = []
    for sec, head, cells in rows():
        if sec.startswith("System") or "State" not in head:
            continue
        c = dict(zip(head, cells))
        state = re.search(r"\*\*([A-Z ]+)\*\*", c.get("State", ""))
        file_cell = c.get("File (`print/`)", "")
        out.append({
            "system": sec, "part": plain(c.get("Part", ""), 80), "version": plain(c.get("Version", ""), 60),
            "state": state.group(1).strip() if state else plain(c.get("State", ""), 20),
            "ready_file": "yes" if (".stl" in file_cell and "do not print" not in file_cell) else "no",
            "mass": plain(c.get("g", ""), 40), "next": plain(c.get("Next (owner)", ""), 220),
        })
    return out


def spark():
    text = (ROOT / "nox/current-parts.md").read_text()
    sec = text.split("## Spark", 1)[1] if "## Spark" in text else ""
    out = []
    for line in sec.splitlines():
        m = re.match(r"- \*\*(.+?)\*\*:?\s*(.*)", line)
        if not m:
            continue
        kind = m.group(1).rstrip(":")
        for item in re.split(r";\s+(?![^()]*\))", m.group(2)):
            if item.strip():
                out.append({"kind": kind, "item": plain(item, 200)})
    return out


def log():
    out = []
    for l in (ROOT / "nox/log.md").read_text().splitlines():
        m = re.match(r"- (\d+ \w+) \(([^),:]+)[^)]*\):?\s*(.*)", l)
        if m:
            out.append({"date": m.group(1), "who": m.group(2), "what": plain(m.group(3), 240)})
        if len(out) == 15:
            break
    return out


def runs():
    latest = {}
    for rj in sorted((ROOT / "runs").glob("*/*/result.json")):
        try:
            d = json.loads(rj.read_text())
        except Exception:
            continue
        if not isinstance(d, dict):
            continue
        part, ver = rj.parts[-3], rj.parts[-2]
        date = str(d.get("date") or datetime.datetime.fromtimestamp(rj.stat().st_mtime).isoformat())[:16]
        row = {"part": part, "version": ver, "date": date,
               "mass_g": d.get("mass_g") if isinstance(d.get("mass_g"), (int, float)) else None,
               "worst_index": d.get("worst_index") if isinstance(d.get("worst_index"), (int, float)) else None,
               "solid": {True: "PASS", False: "FAIL"}.get(d.get("solid_ok"), "")}
        if part not in latest or row["date"] >= latest[part]["date"]:
            latest[part] = row
    return sorted(latest.values(), key=lambda r: r["date"], reverse=True)


def prs():
    try:
        r = subprocess.run(["gh", "api", "repos/sparkiscool/nox/pulls?state=open&per_page=50"],
                           capture_output=True, text=True, timeout=30)
        return [{"number": p["number"], "title": p["title"], "branch": p["head"]["ref"], "updated": p["updated_at"][:16]}
                for p in json.loads(r.stdout)]
    except Exception:
        return []


if __name__ == "__main__":
    out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ROOT / "board")
    out.mkdir(parents=True, exist_ok=True)
    for name, fn in (("parts", parts), ("spark", spark), ("log", log), ("runs", runs), ("prs", prs)):
        data = fn()
        (out / f"{name}.json").write_text(json.dumps(data, indent=1, ensure_ascii=False))
        print(f"{name}: {len(data)} rows")
