#!/usr/bin/env python3
"""Rebuild runs/index.md from every runs/<part>/<version>/result.json.

result.json keys (all optional, missing shows as "·"):
  date (ISO), status (RUNNING | DONE | FAILED), mass_g, mass_tag, worst_index,
  worst_case, solid_ok (bool), bed_face, bed_contact_mm2, peak_mem_gb,
  wall_time_min, pr, note
"""
import json, pathlib, datetime

ROOT = pathlib.Path(__file__).resolve().parents[1]
RUNS = ROOT / "runs"


def cell(v, tag=None, nd=1):
    if v is None or v == "":
        return "·"
    if isinstance(v, bool):
        return "PASS" if v else "FAIL"
    if isinstance(v, float):
        v = f"{v:.{nd}f}"
    return f"{v} ({tag})" if tag else str(v)


def load():
    rows = []
    for rj in sorted(RUNS.glob("*/*/result.json")):
        part, ver = rj.parts[-3], rj.parts[-2]
        try:
            d = json.loads(rj.read_text())
        except Exception as e:  # keep the board alive on one bad file
            d = {"status": f"bad result.json: {e}"}
        d.setdefault("date", datetime.datetime.fromtimestamp(rj.stat().st_mtime).strftime("%Y-%m-%d %H:%M"))
        d["_part"], d["_ver"], d["_dir"] = part, ver, rj.parent.relative_to(ROOT).as_posix()
        d["_preview"] = (rj.parent / "preview.png").exists()
        rows.append(d)
    rows.sort(key=lambda d: d["date"], reverse=True)
    return rows


def main():
    rows = load()
    out = ["# Runs", "", f"_{len(rows)} runs · rebuilt {datetime.datetime.now():%Y-%m-%d %H:%M}_", ""]
    latest = {}
    for d in rows:
        latest.setdefault(d["_part"], d)
    out += ["## Latest per part", "", "| Part | Version | Status | Mass | Worst index | Solid | Summary |", "|---|---|---|---|---|---|---|"]
    for p in sorted(latest):
        d = latest[p]
        out.append(
            f"| {p} | {d['_ver']} | {cell(d.get('status'))} | {cell(d.get('mass_g'), d.get('mass_tag', 'M'))} | "
            f"{cell(d.get('worst_index'), None, 2)} {('in ' + d['worst_case']) if d.get('worst_case') else ''} | "
            f"{cell(d.get('solid_ok'))} | [{d['_dir']}]({d['_dir']}/summary.md) |"
        )
    out += ["", "## All runs, newest first", "", "| Date | Part | Version | Status | Mass | Worst index | Solid | Bed face | Peak mem | Wall | PR |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    for d in rows:
        pr = f"[PR]({d['pr']})" if d.get("pr") else "·"
        out.append(
            f"| {d['date']} | {d['_part']} | [{d['_ver']}]({d['_dir']}/summary.md) | {cell(d.get('status'))} | "
            f"{cell(d.get('mass_g'), d.get('mass_tag', 'M'))} | {cell(d.get('worst_index'), None, 2)} | {cell(d.get('solid_ok'))} | "
            f"{cell(d.get('bed_face'))} | {cell(d.get('peak_mem_gb'), 'P')} GB | {cell(d.get('wall_time_min'), None, 0)} min | {pr} |"
        )
    out.append("")
    (RUNS / "index.md").write_text("\n".join(out))
    print(f"runs/index.md: {len(rows)} runs")


if __name__ == "__main__":
    RUNS.mkdir(exist_ok=True)
    main()
