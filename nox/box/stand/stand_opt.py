"""Foot placement optimization for the free-standing box.

Variables (symmetric in X): back pair (+-xb, yb), front pair (+-xf, yf);
tripod variants with one centred foot front or back.
Cost: printed leg mass (strut 10 x 10 PETG from the nearest structure to the
foot) + pads. Constraints (DEFAULT):
  S1 static tilt to tip >= 10 deg in every load state (IEC 62368-1 style)
  S2 arm drive torque 2.4 N m (C) never lifts an edge
  D1 landing gate (+-30, h <= 100, drift 0.15 out, yaw 5, mu 0.05/0.4):
     peak box rock <= 1.0 deg, drone seated on rolling wheels (mu 0.05;
     mu 0.4 is the stuck-wheel case), magnet on at the seat.
     Walk is reported only: an off-centre hit on one 45 deg face pushes the
     box sideways as hard as down, so it slides unless floor mu > ~1; it was
     the same (1.8 to 3.3 mm) on every footprint and floor mu 0.5 to 1.2.
     (A foot-lift limit is not usable: an off-centre impulse lifts the far
     feet a few mm on any footprint, wider feet lift more; 1 mm failed even
     on the largest layout. Tilt and walk are what matter.)
  D2 extended (+-40, h 150): box never tips (tilt < 10 deg)
Envelope: |X| <= 160, Y -64..286 (nothing outside the deployed funnel shadow).
"""
import itertools, json, time, sys
import numpy as np
from stand_model import configs, static_report, ZF
from stand_landing import build, drop

RHO_L = 0.127          # g per mm of 10 x 10 PETG strut (1.27 g/cm3), E
PAD = 4.0              # g per TPU pad, E
TILT_OK, SLIDE_OK = 1.0, 1e9   # deg DEFAULT; walk reported, not a placement constraint (see below)


def _seg_dist(p, a, b):
    a, b = np.array(a, float), np.array(b, float)
    t = np.clip(np.dot(p - a, b - a) / np.dot(b - a, b - a), 0, 1)
    return np.linalg.norm(p - (a + t * (b - a)))


def attach_dist(x, y):
    """Shortest strut from the foot top (x, y, ZF+6) to existing structure."""
    p = np.array([x, y, ZF + 6.0])           # (X, Y, Z)
    best = 1e9
    for th in np.radians(np.linspace(-180, 0, 91)):          # ring outer, lower half
        for yy in (88.0, 131.0, 174.0):
            q = np.array([154 * np.cos(th), yy, 154 * np.sin(th)])
            yc = np.clip(y, 88, 174); q[1] = yc
            best = min(best, np.linalg.norm(p - q))
    for s in (-1, 1):                                          # spider spokes and ties
        c = np.array([s * 146 * 0.7071, -3, -146 * 0.7071])
        best = min(best, _seg_dist(p, (s * 28.3, -3, -28.3), c))
        best = min(best, _seg_dist(p, c, c + [0, 91, 0]))
    best = min(best, _seg_dist(p, (-107, -23, -112), (107, -23, -112)))   # PSU plate edge (E)
    return best


def layout_feet(kind, xb, yb, xf, yf):
    if kind == "quad":
        return [(xb, yb), (-xb, yb), (xf, yf), (-xf, yf)]
    if kind == "tri_front1":
        return [(xb, yb), (-xb, yb), (0.0, yf)]
    if kind == "tri_back1":
        return [(0.0, yb), (xf, yf), (-xf, yf)]


def cost(feet):
    return sum(RHO_L * attach_dist(x, y) + PAD for x, y in feet)


def gate_drops(ext=False):
    out = []
    offs = (-40, 0, 40) if ext else (-30, 0, 30)
    hs = (150,) if ext else (50, 100)
    for dx, dy in itertools.product(offs, offs):
        n = np.hypot(dx, dy)
        drift = (0, 0) if n == 0 else (0.15 * dx / n, 0.15 * dy / n)
        for h in hs:
            for mu in (0.05, 0.4):
                out.append(dict(dx=dx, dy=dy, h=h, drift=drift, yaw=5, mu=mu))
    return out


def landing_check(feet, ext=False, stop_on_fail=True):
    worst = dict(lift=0, tilt=0, case=None)
    models = {}
    for c in gate_drops(ext):
        mu = c.pop("mu")
        if mu not in models:
            models[mu] = build(feet, mu_face=mu)[0]
        lift, tilt, slide, seated = drop(models[mu], **c)
        bad = (ext and tilt > 10) or (not ext and (tilt > TILT_OK or slide > SLIDE_OK or (mu < 0.1 and not seated)))
        if tilt > worst["tilt"] or bad:
            worst = dict(lift=lift, tilt=tilt, case={**c, "mu": mu}, slide=slide, seated=seated)
        if bad and stop_on_fail:
            return False, worst
    return True, worst


def main():
    t0 = time.time()
    m, s, lab, occ = configs()
    xs = (80, 100, 120, 140, 160)
    ybs = (-64, -45, -25, -5, 15)
    yfs = (150, 175, 200, 225, 250, 275)
    cands = []
    for kind in ("quad", "tri_front1", "tri_back1"):
        for xb, yb, xf, yf in itertools.product(xs, ybs, xs, yfs):
            if kind == "tri_front1" and xf != xs[0]:
                continue
            if kind == "tri_back1" and xb != xs[0]:
                continue
            feet = layout_feet(kind, xb, yb, xf, yf)
            r = static_report(np.array(feet, float), m, s)
            tilt, arm = r["tilt"].min(), r["arm_margin"].min()
            cands.append(dict(kind=kind, xb=xb, yb=yb, xf=xf, yf=yf, feet=feet,
                              tilt=float(tilt), arm=float(arm), push=float(r["push"].min()),
                              cost=cost(feet)))
    stat = [c for c in cands if c["tilt"] >= 10 and c["arm"] >= 0]
    stat.sort(key=lambda c: c["cost"])
    print(f"{len(cands)} layouts, {len(stat)} pass static, {time.time()-t0:.0f} s", flush=True)
    passed, tried = [], []
    done_kinds = set()
    for c in stat:
        if c["kind"] in done_kinds:
            continue
        ok, w = landing_check(c["feet"])
        c["gate"] = dict(ok=ok, **w)
        tried.append(c)
        print(f'{c["kind"]:10s} xb {c["xb"]:4} yb {c["yb"]:4} xf {c["xf"]:4} yf {c["yf"]:4} '
              f'cost {c["cost"]:5.1f} g tilt {c["tilt"]:5.1f} gate {"OK " if ok else "no "}'
              f'peak tilt {w["tilt"]:5.2f} slide {w.get("slide",0):4.1f}', flush=True)
        if ok:
            ok2, w2 = landing_check(c["feet"], ext=True, stop_on_fail=False)
            c["ext"] = dict(ok=ok2, **w2)
            print("   extended:", ok2, round(w2["lift"], 2), round(w2["tilt"], 2), flush=True)
            if ok2:
                passed.append(c); done_kinds.add(c["kind"])
                if {p["kind"] for p in passed} == {"quad", "tri_front1", "tri_back1"}:
                    break
        if len(tried) > 900:
            break
    json.dump(dict(cands=cands, tried=tried, passed=passed), open("out_opt.json", "w"), default=float, indent=1)
    print("done", time.time() - t0)


if __name__ == "__main__":
    main()
