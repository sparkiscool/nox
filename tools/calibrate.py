#!/usr/bin/env python3
"""Turn test readings into calibration factors (docs/tests.md). The predicted value comes from the test's row in
nox/dimensions.md section T (the one place the number lives); the script prints the stats, the factor and the exact row
text to paste over that row, plus the derived P number (MPa) where the row gives a bond or section area ("A <n> mm2").

  python3 tools/calibrate.py post_pull 6x6 38.0 36.5 4x3 9.1 8.8     # both post sizes in one call (copy A)
  python3 tools/calibrate.py post_pull_6x6 38.0 36.5                 # one key
  python3 tools/calibrate.py layer_pull 290 301 276
  python3 tools/calibrate.py glue_epoxy 240 255 231 --kg              # readings in kg (luggage scales read kg)
  python3 tools/calibrate.py --list                                   # the keys and their predicted values

Readings are peak loads in N (or kg with --kg, x 9.81). Factor = measured / predicted; the min factor is the one to use
(a part's real index = reported index / factor)."""
import re, sys, statistics, datetime, pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
DIM = ROOT / "nox" / "dimensions.md"
G = 9.81


def rows():
    """{key: (cells, line)} from the dimensions.md T table"""
    out, on = {}, False
    for ln in DIM.read_text().splitlines():
        if ln.startswith("## "):
            on = ln.startswith("## T.")
            continue
        if on and ln.startswith("| `"):
            cells = [c.strip() for c in ln.strip().strip("|").split("|")]
            out[cells[0].strip("`")] = (cells, ln)
    return out


def num(s):
    m = re.search(r"[-+]?\d+(?:\.\d+)?", s)
    return float(m.group()) if m else None


def one(key, vals, R, date):
    if key not in R:
        sys.exit(f"calibrate: no row `{key}` in nox/dimensions.md T (python3 tools/calibrate.py --list)")
    cells, line = R[key]
    pred = num(cells[2])
    n = len(vals)
    mean, lo = statistics.mean(vals), min(vals)
    sd = statistics.stdev(vals) if n > 1 else 0.0
    fm, fl = mean / pred, lo / pred
    print(f"{key}: n {n}, mean {mean:.1f} N, min {lo:.1f} N, sd {sd:.1f} N ({sd / mean:.0%})")
    print(f"  predicted {pred:g} N ({cells[2]})")
    print(f"  factor = measured / predicted: min {fl:.2f}, mean {fm:.2f}  -> use {fl:.2f}")
    derived = ""
    a = re.search(r"A (\d+(?:\.\d+)?) mm", cells[1])
    if a:
        A = float(a.group(1))
        derived = f"; {lo / A:.1f} MPa min, {mean / A:.1f} MPa mean (P, A {A:g} mm²)"
        print(f"  derived strength over A {A:g} mm2: min {lo / A:.1f} MPa, mean {mean / A:.1f} MPa (P)")
    if fl < 0.8:
        print("  NOTE: min factor under 0.8: the method over-predicts strength, so every index of this kind is optimistic "
              "(real = reported / factor); bring it into nox/topo (material.py or the attachment rule) in a design session")
    meas = f"{lo:.1f} min, {mean:.1f} mean, sd {sd:.1f} (n {n}: {', '.join(f'{v:.1f}' for v in vals)}; {date}){derived}"
    new = cells[:3] + [meas, f"{fl:.2f} (min), {fm:.2f} (mean)", "P"]
    row = "| " + " | ".join(new) + " |"
    print("\nPaste over the row in nox/dimensions.md T:\n" + row)
    print(f"\nnox/log.md line:\n- {date} (tests): {key} measured {lo:.1f} min / {mean:.1f} mean N (n {n}) vs {pred:g} predicted, "
          f"factor {fl:.2f} (min){derived.replace('; ', ', ')}; dimensions.md T.\n")
    return row


def main(argv):
    R = rows()
    if not argv or argv[0] in ("-h", "--help"):
        print(__doc__)
        return 0
    if argv[0] == "--list":
        for k, (c, _) in R.items():
            print(f"{k:22s} predicted {c[2]}  | measured {c[3]}")
        return 0
    kg = "--kg" in argv
    date = datetime.date.today().strftime("%d %b").lstrip("0")
    if "--date" in argv:
        date = argv[argv.index("--date") + 1]
        argv = [a for i, a in enumerate(argv) if a != "--date" and (i == 0 or argv[i - 1] != "--date")]
    argv = [a for a in argv if a != "--kg"]
    base, rest = argv[0], argv[1:]
    groups, cur = [], None
    for a in rest:                                           # "post_pull 6x6 v v 4x3 v v": sub-keys start a group
        if num(a) is None or re.fullmatch(r"\d+x\d+(_B)?", a):
            cur = (f"{base}_{a}", [])
            groups.append(cur)
        else:
            if cur is None:
                cur = (base, [])
                groups.append(cur)
            cur[1].append(float(a) * (G if kg else 1.0))
    if not groups or not all(g[1] for g in groups):
        sys.exit("calibrate: give at least one reading per key")
    for k, v in groups:
        one(k, v, R, date)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
