"""Box swap arm v7 drawn parts (24 Oct 2026, swap arm system worker): rails A / B v3 = rail v2 (arm_v6_cad.rail_v2) on the v7 stack
(arm_v7_layout: r 29..151) with the carbon tie sockets (two Ø3.3 x 10 bores rising into the bottom face at r 128, arm_v6_layout.TIE), and
the tie rods (2 x 3.0 carbon per rail, from the hub's lower rim at Y 22.5, r 78). Pinion16 v2, detent ring v3, V-wheel v1 and bush v1 carry
over unchanged. python3 arm_v7_cad.py [checks] -> print/box_arm_rail_{A,B}_v3.stl, runs/arm_rail/v3/ (W meshes, checks.json)"""
import os, sys, math, json
import numpy as np, trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
import arm_v7_layout as S7
S7.use()
import arm_v6_cad as C6, arm_v5_cad as C5, arm_v5_layout as V
from arm_cad import tm
C6.TIES = True
ROOT = os.path.normpath(os.path.join(HERE, "..", "..")); RUN = os.path.join(ROOT, "runs", "arm_rail", "v3"); os.makedirs(RUN, exist_ok=True)


def rail_v3(h="A"):
    return C6.rail_v2(h)


def tie_rods(h="A"):
    """the two rods of rail h in head A's frame (B is mirrored with its rail)"""
    m = None
    for p0, p1, d in C6.tie_geom(h):
        c = C6.cyl_between(p0, p1, C6.T["rod_d"] / 2, 24); m = c if m is None else m + c
    return m


def main(checks=False):
    from solid_check import solid_check
    res = {}
    for h in ("A", "B"):
        m = rail_v3(h); tm(m).export(os.path.join(RUN, f"arm_rail_{h}_v3_W.stl"))
        t = C5.pose_rail(m); path = os.path.join(ROOT, "print", f"box_arm_rail_{h}_v3.stl"); t.export(path)
        r = dict(file=os.path.relpath(path, ROOT), mass_g=round(t.volume / 1000 * 1.27, 2), extents=np.round(t.extents, 1).tolist(), bodies=len(m.decompose()))
        if checks:
            ok, info = solid_check(trimesh.load(path, force="mesh"), pitch=0.25)
            r["solid_check"] = "PASS" if ok else "FAIL"; r["solid_info"] = {k: info[k] for k in ("bodies_eroded", "necks", "thin_walls") if k in info}
        rods = tie_rods(h)
        r["rods_in_rail_mm3"] = round((m ^ rods).volume(), 2); r["rod_len"] = round(float(np.linalg.norm(np.subtract(*C6.tie_geom(h)[0][:2]))), 1)
        res[f"rail_{h}_v3"] = r; print(h, r)
    json.dump(res, open(os.path.join(RUN, "checks.json"), "w"), indent=1, default=str)
    return res


if __name__ == "__main__":
    main("checks" in sys.argv)
