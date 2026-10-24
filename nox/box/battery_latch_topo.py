"""Battery latch receiver block v2 candidate: topology on the real latch loads (24 Oct 2026, battery latch worker).

The receiver latch block v1 (`battery_latch.py receiver()`) is drawn plates. Here only its must-haves are drawn, each at
least 6 thick (the gate's 2.0 erosion keeps a 2 core): per side three locating pads on the cartridge rails (closed end,
tab line, lead end), two 30 deg lead-in feet (ends) and the latch pocket block (tab line, with its foot); two end pads
with the z lead-ins; four TPU stop pad seats over the cartridge top (ends); the XT60E1-M float pocket with its pull lips
and lid inserts; four M3 mount bosses (o 12, heat-set from the mount plane). Everything else is generated inside the v1
envelope widened to x +-(RF + 7) (Scout: clipped to the collar planes at the v1 inset). Nothing of it in the cartridge
path (with its lead-in capture) or over the pack between the stop pads above the deck line (the partner sits on Y_M).

Supports: the four boss tops (the mount plane: drone belly frame, Overseer sled or bay body).
Loads (dimensions L at the swap CG, cartridge coords, Scout 210.9 g / Overseer 747.5 g swap mass, E):
  pull-out (hover 1 g, flight 4 g + 2 g x, catch 6 g, crash -y, crash corner (x, -y, z)/sqrt3; crash = CRASH_G): the bolts bear
  on the pocket floors (-y; the corner's x on the locating pads, z on an end pad);
  crash +y and swap insert (60 N: 45 on the XT60 lid seat, 15 on the stop pads): +y on the stop seats / XT60 seat;
  crash +-x: the rails on one side's locating pads; crash +-z: an end pad;
  swap pull 50 N: 45 N -y on the XT60 pull lips (bolts retracted, the head holds the cartridge).

python3 nox/box/battery_latch_topo.py scout|overseer [--h 1.0] [--gen 14] [--out runs/battery_latch_receiver_topo/<pack>-v1] [--git]
python3 nox/box/battery_latch_topo.py scout --check   -> reachability + cell counts at the coarse grid, no solve
"""
import argparse, math, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [HERE, os.path.join(ROOT, "nox")]
import battery_latch as BL
from battery_latch import box, cyl, U, prism_xy, prism_zy

T_MIN = 6.0                        # drawn body thickness (gate: 2 core at 2.0 erosion)
ENV_X = 7.0                        # envelope half width past the locating face
STN = 10.0                         # station length in z (locating pads, feet, stop seats)
BOSS_D = 12.0
G0 = 9.81
CRASH_G = {"scout": 15.3, "overseer": 35.0}   # Scout (Spark / Scout v3 24 Oct): the cage's TPU suspension gives above
                                             # ~110 N, so the core decelerates at ~110 N / 0.72 kg = 15.3 g (E); the
                                             # Overseer stays on dimensions L (35 g) until its own pass


def stations(r):
    """z centres of the three side stations: closed end, tab line, lead end (clear of the lid and the XT60 pocket)."""
    zc0 = r["Z_E0"] + STN / 2
    zc2 = r["Z_P1"] - STN / 2 + 1.0
    return [zc0, r["Z_T"], zc2]


def bodies(pack):
    B = BL.build(pack)
    g, r = B["g"], B["r"]
    rf, zt = r["RF"], r["Z_T"]
    xe = rf + ENV_X
    z0, z1 = r["Z_E0"] - r["END_T"], r["Z_E1"] + r["END_T"]          # v1's length: the Overseer's 179.6 fits the 180 bed
    yt = r["Y_DK0"] + T_MIN                                         # top of the stop seats = the deck line
    zs = stations(r)
    D = {k: [] for k in ("loc", "feet", "pocket", "endpad", "stop", "xt60", "boss")}
    for s in (-1, 1):
        def X(a, b): return sorted((s * a, s * b))
        for zc in zs:                                               # locating pads on the rails, up to the deck line
            a0, a1 = X(rf, rf + T_MIN)
            D["loc"].append(box(a0, a1, r["RAIL_Y0"], yt, zc - STN / 2, zc + STN / 2))
        for zc in (zs[0], zs[2]):                                   # feet with the 30 deg lead-in
            poly = [(rf + BL.CAP, BL.Y_RB), (rf + T_MIN, BL.Y_RB), (rf + T_MIN, BL.Y_RB + BL.HCAP + 2.0),
                    (rf, BL.Y_RB + BL.HCAP + 2.0), (rf, BL.Y_RB + BL.HCAP)]
            D["feet"].append(prism_xy([(s * x, y) for x, y in poly], zc - STN / 2, zc + STN / 2))
        pw = max(r["PK_D"] + BL.PK_WALL, T_MIN + 0.5)               # pocket block (+ its foot) at the tab line
        a0, a1 = X(rf, rf + pw)
        pb = box(a0, a1, BL.Y_RB, r["PK_Y1"] + BL.PK_WALL, zt - r["PK_HZ"] - BL.PK_WALL, zt + r["PK_HZ"] + BL.PK_WALL)
        poly = [(rf + BL.CAP, BL.Y_RB), (rf + pw, BL.Y_RB), (rf + pw, r["PK_Y0"]), (rf, r["PK_Y0"]), (rf, BL.Y_RB + BL.HCAP)]
        pb = (pb - box(a0 - 1, a1 + 1, BL.Y_RB - 1, r["PK_Y0"], zt - 50, zt + 50)) + \
            prism_xy([(s * x, y) for x, y in poly], zt - r["PK_HZ"] - BL.PK_WALL, zt + r["PK_HZ"] + BL.PK_WALL)
        D["pocket"].append(pb)
        for zc in (zs[0], zs[2]):                                   # stop pad seats over the cartridge top (TPU pads)
            a0, a1 = X(r["X_IN"] - 1.6, rf + T_MIN)
            D["stop"].append(box(a0, a1, r["Y_DK0"], yt, zc - STN / 2, zc + STN / 2))
    for zf, sg in ((r["Z_E0"], -1), (r["Z_E1"], 1)):                # end pads with the z lead-in at the cartridge corners
        y0 = r["RAIL_Y0"]                                           # (its end bears there only: battery_latch.end_relief)
        et = r["END_T"]                                             # thinner than T_MIN: held by its cover (gate ties)
        pts = [(zf + sg * BL.CAP, y0), (zf + sg * et, y0), (zf + sg * et, yt), (zf, yt), (zf, y0 + BL.HCAP)]
        D["endpad"].append(U(prism_zy(pts, r["X_IN"] - 1.0, rf + T_MIN), prism_zy(pts, -(rf + T_MIN), -(r["X_IN"] - 1.0))))
    zx = r["Z_X"]                                                    # XT60E1-M float pocket body (lid inserts in it)
    D["xt60"].append(box(-r["PKX_XW"] - 2.0, r["PKX_XW"] + 2.0, r["LIP_Y0"], r["Y_DK1"], zx - r["PKX_ZW"] - 2.0, zx + r["PKX_ZW"] + 2.0))
    for (bx, bz) in r["MOUNT"]:
        top = r["Y_DK1"] if abs(bz - zx) < 1e-6 else r["Y_M"]
        D["boss"].append(cyl("y", bx, bz, BOSS_D / 2, top - BL.INS_DEPTH - 2.5, top))
    drawn = U(*[m for v in D.values() for m in v])
    # cuts: the cartridge path (exact, with the capture), the pockets, the XT60 pocket and male path, insert holes,
    # the stop pad recesses, and the space over the pack above the deck line between the stations (partner side)
    cuts = [BL.cart_path(r), BL.end_relief(r)]
    for s in (-1, 1):
        pk, _ = BL.pocket_cut(r); cuts.append(pk if s > 0 else BL.mirx(pk))
    cuts.append(box(-r["PKX_XW"], r["PKX_XW"], r["Y_XM"], r["Y_DK1"] + 10, zx - r["PKX_ZW"], zx + r["PKX_ZW"]))
    cuts.append(box(-8.5, 8.5, -1.0, r["Y_XM"] + 0.01, zx - 4.8, zx + 4.8))
    for (bx, bz) in r["MOUNT"]:
        top = r["Y_DK1"] if abs(bz - zx) < 1e-6 else r["Y_M"]
        cuts.append(cyl("y", bx, bz, BL.BOLT_D / 2, -5.0, top + 1))
    for s in (-1, 1):
        for zc in (zs[0], zs[2]):
            a0, a1 = sorted((s * (r["X_IN"] - 1.6), s * r["RF"]))
            cuts.append(box(a0, a1, r["Y_DK0"] - 1, r["Y_DK0"] + BL.STRIP_REC, zc - STN / 2 + 1, zc + STN / 2 - 1))
    lid_zone = box(-(r["BX"] + BL.BOSS_R + 0.5), r["BX"] + BL.BOSS_R + 0.5, r["Y_DK1"], r["Y_M"] + 1, zx - 10.5, zx + 10.5)
    cuts.append(lid_zone)
    cuts = U(*cuts)
    env = box(-xe, xe, BL.Y_RB, yt, z0, z1) + U(*D["boss"]) + U(*D["xt60"])
    if B["clip"] is not None:
        env = env ^ B["clip"]
        drawn = drawn ^ B["clip"]
    return dict(B=B, g=g, r=r, D=D, drawn=drawn, cuts=cuts, env=env, yt=yt, zs=zs)


def add_cases(p, Q, h, reg=lambda m: m):
    """supports and the L load cases on Part p; reg maps each region (identity for the topology run, ^ the v1 mesh for
    the as-drawn FE check)."""
    g, r, D = Q["g"], Q["r"], Q["D"]
    seats = U(*[cyl("y", bx, bz, BOSS_D / 2, (r["Y_DK1"] if abs(bz - r["Z_X"]) < 1e-6 else r["Y_M"]) - 1.5 * h,
                    (r["Y_DK1"] if abs(bz - r["Z_X"]) < 1e-6 else r["Y_M"]) + 1.0) for (bx, bz) in r["MOUNT"]])
    p.support("default", reg(seats))
    W = g["SWAP_M"] / 1000 * G0
    cg = CRASH_G[g["pack"]]
    pk = U(*D["pocket"])
    floors = U(*[box(*sorted((s * r["RF"], s * (r["RF"] + r["PK_D"]))), r["PK_Y0"] - 1.5, r["PK_Y0"],
                     r["Z_T"] - r["PK_HZ"], r["Z_T"] + r["PK_HZ"]) for s in (-1, 1)]) ^ pk
    locp = U(*[m for m in D["loc"] if m.bounding_box()[0] > 0]); locn = U(*[m for m in D["loc"] if m.bounding_box()[0] < 0])
    stop = U(*D["stop"]); end0, end1 = D["endpad"]
    lips = U(*D["xt60"])
    fn = "flight"                                                  # SF 4 (topo.material); crashes misuse SF 2.5
    floors, locp, locn, stop, end0, end1, lips = [reg(m) for m in (floors, locp, locn, stop, end0, end1, lips)]
    p.load("hover 1 g", floors, (0, -W, 0), cls=fn)
    p.load_multi("flight 4 g + 2 g x", [(floors, (0, -4 * W, 0)), (locp, (2 * W, 0, 0))], cls=fn)
    p.load("catch 6 g", floors, (0, -6 * W, 0), cls=fn)
    p.load("crash -y", floors, (0, -cg * W, 0))
    p.load("crash +y", stop, (0, cg * W, 0))
    p.load("crash +x", locp, (cg * W, 0, 0))
    p.load("crash -x", locn, (-cg * W, 0, 0))
    p.load("crash +z", end1, (0, 0, cg * W))
    p.load("crash -z", end0, (0, 0, -cg * W))
    c = cg * W / math.sqrt(3)
    p.load_multi("crash corner (+x -y +z)", [(floors, (0, -c, 0)), (locp, (c, 0, 0)), (end1, (0, 0, c))])
    p.load_multi("crash corner (-x -y -z)", [(floors, (0, -c, 0)), (locn, (-c, 0, 0)), (end0, (0, 0, -c))])
    p.load_multi("swap insert 60 N", [(lips, (0, 45.0, 0)), (stop, (0, 15.0, 0))], cls=fn)
    p.load("swap pull 50 N", lips, (0, -45.0, 0), cls=fn)
    p.load("handling 50 N on a foot", reg(U(*D["feet"][:1])), (0, 0, 50.0))
    return p


def setup_for(pack):
    from topo import Part

    def setup(h):
        Q = bodies(pack)
        p = Part(f"battery_receiver_{pack}", h, Q["drawn"], Q["cuts"], Q["env"], build="+z", material="PETG")
        return add_cases(p, Q, h)
    return setup


def fe_v1(pack, h=1.0, log=print):
    """as-built voxel FE of the drawn receiver v1 (`battery_latch.receiver()`) on the same cases and supports, regions cut
    to its own material: worst index per case (topo.build.strength), the number a topology v2 must beat (rule 6)."""
    from types import SimpleNamespace as NS
    from topo import Part
    from topo.part import tm as ttm
    from topo.material import MATERIALS, AXES, stiffness
    from topo.build import strength
    Q = bodies(pack)
    R = Q["B"]["recv"]
    far = box(1e4, 1e4 + 1, 0, 1, 0, 1)
    p = Part(f"battery_receiver_{pack}_v1", h, R, far, R, build="+z", material="PETG", log=lambda s_: None)
    add_cases(p, Q, h, reg=lambda m: m ^ R)
    mat = MATERIALS["PETG"]; ax = AXES["+z"][0]
    cases = [NS(name=c.name, F=c.F, cls=c.cls, support=c.support, nodes=c.nodes, tie=False, fe=None) for c in p.cases]
    lay = NS(part=p, cases=cases, mat=mat, build_axis=ax, C=stiffness(mat, ax), solid=p.solid)
    return strength(lay, ttm(R), log=log)


def first_mode(pack, h=1.0, log=print):
    """lowest first mode (Hz, E) of the seated swap mass (pack + cartridge + sliders, rigid) on the drawn receiver v1, fixed
    at its bosses: for each axis a unit load through the regions that carry it (floors -y and +y via the stop seats, the
    locating pads x, the end pads z), stiffness k = F / mean displacement of the loaded nodes, f = sqrt(k / m) / 2 pi."""
    from topo import Part
    from topo.material import MATERIALS, AXES, stiffness
    from topo.fem import Solver
    from topo.part import voxelise, tm as ttm
    Q = bodies(pack); g, r, D = Q["g"], Q["r"], Q["D"]; R = Q["B"]["recv"]
    far = box(1e4, 1e4 + 1, 0, 1, 0, 1)
    p = Part(f"battery_receiver_{pack}_v1_mode", h, R, far, R, build="+z", material="PETG", log=lambda s_: None)
    seats = U(*[cyl("y", bx, bz, BOSS_D / 2, (r["Y_DK1"] if abs(bz - r["Z_X"]) < 1e-6 else r["Y_M"]) - 1.5 * h,
                    (r["Y_DK1"] if abs(bz - r["Z_X"]) < 1e-6 else r["Y_M"]) + 1.0) for (bx, bz) in r["MOUNT"]])
    p.support("default", seats ^ R)
    pk = U(*D["pocket"])
    floors = U(*[box(*sorted((s * r["RF"], s * (r["RF"] + r["PK_D"]))), r["PK_Y0"] - 1.5, r["PK_Y0"],
                     r["Z_T"] - r["PK_HZ"], r["Z_T"] + r["PK_HZ"]) for s in (-1, 1)]) ^ pk ^ R
    loc = U(*D["loc"]) ^ R
    ends = U(*D["endpad"]) ^ R
    stop = U(*D["stop"]) ^ R
    p.load("y-", floors, (0, -1.0, 0)); p.load("y+", stop, (0, 1.0, 0))
    p.load_multi("x", [(U(*[m for m in D["loc"] if m.bounding_box()[0] > 0]) ^ R, (0.5, 0, 0)),
                       (U(*[m for m in D["loc"] if m.bounding_box()[0] < 0]) ^ R, (0.5, 0, 0))])
    p.load("z", ends, (0, 0, 1.0))
    mat = MATERIALS["PETG"]; C = stiffness(mat, AXES["+z"][0])
    A = voxelise(ttm(R), *p.axes).ravel()
    sol = Solver(p.shape, p.h, C, p.supports, active=A | p.solid, backend="auto", log=lambda s_: None)
    F = np.stack([c.F for c in p.cases], 1)
    Uu = sol.solve(np.where(A, 1.0, 1e-9), F, ["default"] * len(p.cases), keys=[c.name for c in p.cases])
    m = g["SWAP_M"] / 1000.0
    out = {}
    for q, c in enumerate(p.cases):
        nd = c.nodes; d = np.asarray(F[:, q]).reshape(-1, 3)[nd]; dirv = d.sum(0) / np.linalg.norm(d.sum(0))
        u = Uu[:, q].reshape(-1, 3)[nd] @ dirv
        k = 1.0 / max(float(np.mean(u)), 1e-12) * 1000.0            # N/m (u in mm per N)
        out[c.name] = dict(k_N_per_mm=round(k / 1000.0, 1), f_Hz=round(math.sqrt(k / m) / (2 * math.pi), 0))
    out["first_mode_Hz"] = min(v["f_Hz"] for v in out.values())
    log(f"{pack} receiver as a pack mount: first mode ~{out['first_mode_Hz']} Hz (E, rigid pack {g['SWAP_M']:.0f} g)")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pack", choices=("scout", "overseer"))
    ap.add_argument("--h", type=float, default=1.0)
    ap.add_argument("--gen", type=float, default=None, help="generated volume target cm3")
    ap.add_argument("--out", default=None)
    ap.add_argument("--git", action="store_true")
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--fe-v1", action="store_true", help="as-built FE of the drawn receiver v1 on the same cases")
    ap.add_argument("--mode", action="store_true", help="first mode (Hz) of the seated swap mass on receiver v1")
    ap.add_argument("--time", type=float, default=5.5 * 3600)
    a = ap.parse_args()
    if a.mode:
        import json
        print(json.dumps(first_mode(a.pack, a.h), indent=1))
        return
    if a.fe_v1:
        import json
        res = fe_v1(a.pack, a.h)
        print(json.dumps(res, indent=1))
        return
    if a.check:
        from topo.layout import Layout, Settings
        p = setup_for(a.pack)(2.0 * a.h)
        Q = bodies(a.pack)
        print("drawn", round(Q["drawn"].volume() / 1000, 2), "cm3; env", round(Q["env"].volume() / 1000, 2), "cm3")
        Layout(p, gen_cm3=10.0, settings=Settings())
        print("reachability ok")
        return
    from topo import Settings, optimise
    gen = a.gen if a.gen is not None else {"scout": 12.0, "overseer": 30.0}[a.pack]
    out = a.out or os.path.join(ROOT, "runs", "battery_latch_receiver_topo", f"{a.pack}-v1")
    optimise(setup_for(a.pack), h=a.h, gen_cm3=gen, out_dir=out, name=f"battery_receiver_{a.pack}",
             settings=Settings(time_limit_s=a.time), git=a.git)


if __name__ == "__main__":
    main()
