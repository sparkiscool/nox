"""Hub v6 on the topo engine (24 Oct 2026, swap arm system worker): hub_v6.hub_v6_parts(), build +y (sleeve end on the bed, as hub v5).
The hub is cut from the arm system at its joints (swap_system.py, docs/generative.md The rule 0): supports = the two 6010 seats; loads =
the joint reactions the system frame puts on the hub's interfaces in every system case (runs/arm/v6/system.json "frame" -> "hub"),
the receiver-guided swap cases both with the receiver's 0.3 gaps and free (no receiver), so the hub carries the worse of the two:
  - motor seats A / B / axle: force + moment (the pinion / 20T loads and the motor weights at their CGs), moments as force couples on
    the seat ring's halves;
  - rail pads A / B at r 40 / 78: force + moment from each rail foot (one M5 + the foot on the pad), moments as couples on the pad halves;
  - the detent: pivot 22 N and anchor 8.4 N (the springs' preload, detent.CHOSEN_V6), sustained.
Classes: swap -> flight (repeated working load, SF 4), misuse / handling -> misuse (2.5), sustained -> sustained (10).
python3 hub_v6_topo.py [h] [gen_cm3] [out_dir] -> <out_dir>/box_hub_v6.stl (W) + result.json (topo engine). NOX_SETUP_ONLY=1: setup only."""
import sys, os, math, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
import arm_v7_layout as S7
if os.environ.get("NOX_ARM", "v7") == "v7": S7.use()                         # the v7 stack (ties, R 113 loads); NOX_ARM=v6 for the v6 hub
import arm_layout as L, arm_cad as A, detent as DT
import hub_v6 as HV
from arm_cad import box
from topo import Part, Settings, optimise

H = float(sys.argv[1]) if len(sys.argv) > 1 else 1.5
GEN = float(sys.argv[2]) if len(sys.argv) > 2 else 40.0
SYSTEM = os.path.join(HV.REPO, "runs", "arm", os.environ.get("NOX_ARM", "v7"), "system.json")
CLS = dict(swap="flight", misuse="misuse", handling="misuse", sustained="sustained")
BIG = 400.0


def halves(region, c, axis, d):
    """region split by the plane through c normal to axis (0 X, 2 Z; 1 Y unused): (minus half, plus half)"""
    lo = [c[0] - BIG, c[1] - BIG, c[2] - BIG]; hi = [c[0] + BIG, c[1] + BIG, c[2] + BIG]
    lo_p, hi_m = list(lo), list(hi); lo_p[axis] = c[axis]; hi_m[axis] = c[axis]
    m = region ^ box(lo[0], hi_m[0], lo[1], hi_m[1], lo[2], hi_m[2])
    p = region ^ box(lo_p[0], hi[0], lo_p[1], hi[1], lo_p[2], hi[2])
    return m, p


def iface_pieces(region, c, F, M, d):
    """[(region, vec)] carrying force F at c and moment M (N.mm, arm frame) as couples over the region's halves, d = the halves'
    centroid spacing (C). Mx: Y forces across Z halves; Mz: Y forces across X halves; My: X forces across Z halves (r x F)."""
    out = [(region, tuple(F))]
    Mx, My, Mz = M
    zm, zp = halves(region, c, 2, d); xm, xp = halves(region, c, 0, d)
    if abs(Mx) > 1e-6:
        f = -Mx / d; out += [(zp, (0, f, 0)), (zm, (0, -f, 0))]
    if abs(Mz) > 1e-6:
        f = Mz / d; out += [(xp, (0, f, 0)), (xm, (0, -f, 0))]
    if abs(My) > 1e-6:
        f = My / d; out += [(zp, (f, 0, 0)), (zm, (-f, 0, 0))]
    return [(r, v) for r, v in out if not r.is_empty()]


def regions():
    """interface name (swap_system node) -> (region, centre, couple spacing)"""
    out = {}
    for nm, M in HV.MOTORS6.items():
        ring = A.cylY(M["xc"], M["zc"], L.Y_PLATE0, L.Y_PLATE1, 21.0) - A.cylY(M["xc"], M["zc"], L.Y_PLATE0 - 1, L.Y_PLATE1 + 1, 12.0)
        out[f"seat {nm}"] = (ring, (M["xc"], 52.0, M["zc"]), 2 * 4 * (21.0 ** 3 - 12.0 ** 3) / (3 * math.pi * (21.0 ** 2 - 12.0 ** 2)))
    for h, r, p, hole in HV.pads():
        c = (0.0, 52.0, r) if h == "A" else (-r, 52.0, 0.0)
        out[f"pad{h} {r:g}"] = (p - hole, c, 2 * 0.5 * HV.PAD_L)
    for h in ("A", "B"):                                                     # v7: the tie sockets (both rods of a head as one interface)
        b, c = HV.tie_sockets((h,))
        if b is not None:
            t = HV.S6.TIE; ctr = (0.0, t["hub"][0], t["hub"][1]) if h == "A" else (-t["hub"][1], t["hub"][0], 0.0)
            out[f"tie hub {h}"] = (b - c, ctr, 2 * t["x"])
    return out


def setup(h):
    P = HV.hub_v6_parts(); drawn, cuts, env0 = P["drawn"], P["cuts"], P["env0"]
    pp = Part("box_hub_v6", h, drawn, cuts, env0, build="+y")
    pp.support("default", A.cylY(0, 0, 8, 40, 42.0) - A.cylY(0, 0, 7, 41, 39.5))          # the bearing seats, as v5
    R = regions()
    sysres = json.load(open(SYSTEM))
    n = 0
    for case in sysres["frame"] + [dict(c, case=c["case"] + " [Overseer]") for c in sysres.get("frame_overseer", [])]:
        # every case, the power cut carrying included: the fallback until the head brake (box electronics) is built and bench-tested
        parts = []
        for node, v in case["hub"].items():
            if node not in R: continue
            reg, c, d = R[node]
            parts += iface_pieces(reg, c, v["F"], v["M"], d)
        if parts:
            pp.load_multi(f"{case['case']} [{case['cls']}]", parts, cls=CLS[case["cls"]]); n += 1
    D, S = P["detent"]
    Pv, Av = D["P"], D["A"]; dA = (D["H"] - Av) / np.linalg.norm(D["H"] - Av)
    pp.load_multi("detent springs (pivot 22 N, anchor 8.4 N)", [(A.cylY(Pv[0], Pv[1], 66.3, 71.3, 4.0), (22 * D["u"][0], 0, 22 * D["u"][1])),
                                                                (A.cylY(Av[0], Av[1], 66.3, 74.1, 4.0), (8.4 * dA[0], 0, 8.4 * dA[1]))], cls="sustained")
    pp.log(f"  {n} system cases from {os.path.relpath(SYSTEM, HV.REPO)} + the detent springs")
    return pp


if __name__ == "__main__":
    out = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HV.REPO, "runs", "hub", "v6")
    if os.environ.get("NOX_SETUP_ONLY"):
        p = setup(H); print(p.summary() if hasattr(p, "summary") else "setup ok"); sys.exit(0)
    res = optimise(setup, H, GEN, os.path.abspath(out), "box_hub_v6", settings=Settings(), git=bool(os.environ.get("NOX_GIT")))
    print({k: res.get(k) for k in ("name", "solid_ok", "mass_g", "total_s")}, res["strength"]["worst"])
