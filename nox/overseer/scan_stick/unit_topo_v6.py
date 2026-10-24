"""Scan stick unit v6: the v5 system (unit_topo_v5.py: layout, cage, head, joint, cases) with the base cut in two so every piece
fits the A1 mini (180 cube). Base v27 passed strength (worst 0.41) but stood 224 tall (z -25..199): no pose fits the bed.
Cut at z 28 (DEFAULT, 24 Oct; alt: split the spine at mid height, a joint in the most bent section):
  - pole adapter (z -25..31): the Ø28.3 sleeve 27 over the Ø27.5 pole cylinder, the thread cut 26 (open at the top: the tip
    stops 1 under the spine floor), the low cage sleeves, a 10 mm flange z 18..28 and a Ø36 x 3 spigot into the spine floor;
    4x M3 x 12 from below (heads sunk 4 into the 10 mm flange) into Ruthex M3 inserts (Ø4.0 x 7) in the spine floor. Reprinting the adapter alone
    retunes the thread (the fit coupon v3 is this socket).
  - spine (z 28..199, 171 tall, prints upright on its floor): pocket tube, top ring + pin, Pi standoffs, ledge + lips, top and
    mid cage sleeves; floor z 28..38 (10: the inserts + 3 over them), a Ø36.6 x 3 recess for the spigot.
The head (module) is unchanged: head v17. Head mass now measured 168 g (v17 M, v5 used 90 g E).
   python3 unit_topo_v6.py --check | --piece spine|adapter [h] [gen] --ver vN | --reactions"""
import sys, os, json
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(ROOT, "nox"))
_argv = sys.argv; sys.argv = [sys.argv[0]]
import unit_topo_v5 as V
sys.argv = _argv
import numpy as np, manifold3d as mf
from topo import Part, Settings, optimise
from unit_topo_v5 import box, cyl, cylx, B6, G, KG, PACK, SPINE, PI_KEEP, EL_KEEP, PI_X0, Z_MOUTH, BOSS_R, JOINT_C, DIRS, DIAG

V.HEAD_KG = 0.168                                               # head v17 (M)
VER = sys.argv[sys.argv.index("--ver") + 1] if "--ver" in sys.argv else None
ARGS = [a for a in sys.argv[1:] if not a.startswith("-") and a != VER]
CUT_Z = 28.0                                                    # the cut plane (C)
FLANGE = (-30.0, 30.0, -31.5, 31.5, CUT_Z - 10.0, CUT_Z)        # adapter flange = the spine footprint, 10 thick (adapter v1: 6, worst 1.77 at its root), C
SPIGOT_R, SPIGOT_H = 18.0, 3.0                                  # Ø36 x 3 into a Ø36.6 recess (0.3 radial, C)
JBOLTS = [(sx * 22.0, sy * 24.0) for sx in (-1, 1) for sy in (-1, 1)]   # 4x M3 x 12 (C): 6 flange + 5.7 insert
INS_R, INS_D, CLR_R = 2.0, 7.0, 1.7                             # Ruthex M3 insert hole Ø4.0 x 7 (spec), M3 clearance Ø3.4
SPINE_KG, SPINE_CG = 0.40, np.array((-5.0, 0.0, 110.0))         # E (base v27 494.6 g M less the adapter)
ADAPTER_KG = 0.09                                               # E

def upper(z0=CUT_Z):
    return box(-200, 200, -200, 200, z0, 400)
def lower(z1=CUT_Z):
    return box(-200, 200, -200, 200, -100, z1)

def spine_bodies():
    S = V.spine_bodies()
    x0, x1, y0, y1, _, _ = SPINE
    floor = box(x0, x1, y0, y1, CUT_Z, PACK[4])
    inserts = sum((cyl(x, y, CUT_Z - 1.0, CUT_Z + INS_D, INS_R, 24) for x, y in JBOLTS), mf.Manifold())
    recess = cyl(0, 0, CUT_Z - 1.0, CUT_Z + SPIGOT_H, SPIGOT_R + 0.3, 96)
    drawn = (S["drawn"] ^ upper()) + floor
    cuts = (S["cuts"] ^ upper(CUT_Z - 2.0)) + inserts + recess + lower()
    env = S["env"] ^ upper()
    sleeves = S["sleeves"] ^ upper()
    grip = sum((cyl(x, y, CUT_Z, CUT_Z + INS_D + 1.0, INS_R + 2.5, 24) for x, y in JBOLTS), mf.Manifold()) \
        + (cyl(0, 0, CUT_Z, CUT_Z + SPIGOT_H, SPIGOT_R + 3.0, 96) - recess)
    return dict(S, drawn=drawn, cuts=cuts, env=env, floor=floor, sleeves=sleeves, grip=grip)

def adapter_bodies():
    S = V.spine_bodies()
    flange = B6(FLANGE)
    spigot = cyl(0, 0, CUT_Z - 1.0, CUT_Z + SPIGOT_H, SPIGOT_R, 96)
    drawn = (S["drawn"] ^ lower()) + flange + spigot
    holes = sum((cyl(x, y, FLANGE[4] - 1.0, CUT_Z + 1.0, CLR_R, 24) for x, y in JBOLTS), mf.Manifold())
    heads = sum((cyl(x, y, FLANGE[4] - 30.0, FLANGE[4] + 4.0, 3.2, 24) for x, y in JBOLTS), mf.Manifold())   # head counterbore 4 deep (M3 x 12: 6 clamp + 5.7 insert) + driver access from below
    top = cyl(0, 0, 26.0, CUT_Z + SPIGOT_H + 1.0, 10.4, 64)                                      # the thread runs out open at the top
    cuts = (S["cuts"] ^ lower(CUT_Z + SPIGOT_H + 1.0)) + holes + heads + top + (upper(CUT_Z) - cyl(0, 0, CUT_Z - 1, CUT_Z + SPIGOT_H, SPIGOT_R, 96))
    env = (S["env"] ^ lower()) + spigot
    sleeves = S["sleeves"] ^ lower()
    pads = [cyl(x, y, CUT_Z - 2.0, CUT_Z, 4.5, 24) - cyl(x, y, CUT_Z - 3, CUT_Z + 1, CLR_R, 24) for x, y in JBOLTS]
    ring = cyl(0, 0, CUT_Z, CUT_Z + SPIGOT_H, SPIGOT_R, 96) - cyl(0, 0, CUT_Z - 1, CUT_Z + SPIGOT_H + 1, 14.0, 96)
    return dict(drawn=drawn, cuts=cuts, env=env, sleeves=sleeves, pads=pads, ring=ring)

def cut_reactions():
    """(name, accel, F, M) the spine puts on the adapter at the cut centre O = (0, 0, CUT_Z), per case: the head (v5 joint
    resultants with the 168 g head), the pack, Pi, UBEC / buck and the spine itself at their CGs, the spine's cage bumps"""
    O = np.array((0.0, 0.0, CUT_Z))
    Hd = V.head_bodies(); out = []
    c = lambda b: np.array(((b[0] + b[1]) / 2, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2))
    masses = [(KG["pack"], c(PACK)), (KG["pi"], c(PI_KEEP)), (KG["el"], c(EL_KEEP)), (SPINE_KG, SPINE_CG)]
    for name, parts, accel, Fv, Mv in V.head_cases(Hd):
        F = np.array(Fv, float); M = np.array(Mv, float) + np.cross(JOINT_C - O, F)
        if accel is not None:
            for m, p in masses:
                f = m * G * np.asarray(accel); F = F + f; M = M + np.cross(p - O, f)
        out.append((name, accel, F, M))
    for nm, p, v in (("cage bump back 100 N (spine share)", (-66.0, 0, 108.0), (50.0, 0, 0)), ("cage bump +y 100 N (spine share)", (0, 36.0, 150.0), (0, -50.0, 0)),
                     ("cage bump top 100 N (spine share)", (0, 0, 192.0), (0, 0, -50.0)), ("cage bump front 100 N (spine share)", (30.0, 0, 150.0), (-50.0, 0, 0)),
                     ("hand push top ring -z 100 N", (0, 0, 196.0), (0, 0, -100.0))):
        f = np.array(v); out.append((nm, None, f, np.cross(np.array(p) - O, f)))
    return out

def bolt_split(F, M):
    """the cut load on the adapter: shear (Fx, Fy) and torsion on the spigot ring, Fz + Mx + My as axial forces on the 4 bolt
    pads (positive z = the spine pulls up, negative = it bears down: the flange carries both at the pads, E)"""
    xs = np.array([b[0] for b in JBOLTS]); ys = np.array([b[1] for b in JBOLTS])
    fz = F[2] / 4 + M[0] * ys / np.sum(ys ** 2) - M[1] * xs / np.sum(xs ** 2)
    rr = np.hypot(xs, ys); ft = M[2] / np.sum(rr)                      # torsion as tangential bolt shear
    pads = [np.array((-ft * y / r, ft * x / r, z)) for x, y, z, r in zip(xs, ys, fz, rr)]
    return np.array((F[0], F[1], 0.0)), pads

def setup_piece(piece, h):
    if piece == "adapter":
        A = adapter_bodies()
        p = Part("scan_stick_adapter", h, A["drawn"], A["cuts"], A["env"], build="+z", material="PETG")
        p.support("default", V.pole_grip() ^ lower())
        for name, accel, F, M in cut_reactions():
            sh, pads = bolt_split(F, M)
            parts = [(A["ring"], sh)] + [(r, f) for r, f in zip(A["pads"], pads)]
            if accel is not None:
                parts.append((A["drawn"] ^ lower(CUT_Z - 6.0), ADAPTER_KG * G * np.asarray(accel)))
            p.load_multi(name, parts)
        for nm, v in (("cage bump low back 100 N", (50.0, 0, 0)), ("cage bump low +y 100 N", (0, -50.0, 0)), ("cage bump low front 100 N", (-50.0, 0, 0))):
            p.load(nm, A["sleeves"], np.array(v))
        return p
    Sb, Hd = spine_bodies(), V.head_bodies()
    p = Part("scan_stick_spine", h, Sb["drawn"], Sb["cuts"], Sb["env"], build="+z", material="PETG")
    p.support("default", Sb["grip"])                                    # held by the adapter: the 4 insert bosses and the spigot recess
    pads = V.joint_regions()
    pack_r = Sb["floor"] + Sb["back"] + Sb["front"] + Sb["ring"]
    el_r = box(PI_X0, PI_X0 + 2, -30, 30, 45, 101)
    for name, parts, accel, Fv, Mv in V.head_cases(Hd):
        jf, err = V.joint_forces(Fv, Mv)
        joint = [(r, f) for r, f in zip(pads, jf) if np.linalg.norm(f) > 1e-6]
        if accel is None:
            p.load_multi(name + " (joint)", joint)
        else:
            gs = np.linalg.norm(accel); d = accel / gs
            p.load_multi(name, joint + [(pack_r, KG["pack"] * G * gs * d), (Sb["standoffs"], KG["pi"] * G * gs * d), (el_r, KG["el"] * G * gs * d)], accel=accel)
    for nm, v in (("cage bump back 100 N (spine share)", (50.0, 0, 0)), ("cage bump +y 100 N (spine share)", (0, -50.0, 0)),
                  ("cage bump top 100 N (spine share)", (0, 0, -50.0)), ("cage bump front 100 N (spine share)", (-50.0, 0, 0))):
        p.load(nm, Sb["sleeves"], np.array(v))
    p.load("hand push top ring -z 100 N", Sb["ring"], np.array((0, 0, -100.0)))
    return p

def check():
    from topo.part import tm
    Sb, A, Hd = spine_bodies(), adapter_bodies(), V.head_bodies()
    s, a, hd = tm(Sb["drawn"] - Sb["cuts"]), tm(A["drawn"] - A["cuts"]), tm(Hd["drawn"] - Hd["cuts"])
    for n, m in (("spine", s), ("adapter", a), ("head", hd)):
        e = m.extents; print(f"{n}: bounds {np.round(m.bounds, 1).tolist()} extents {np.round(e, 1).tolist()} fits 180: {bool(max(e) <= 180.0)}")
    print("spine x adapter overlap mm3", round((Sb["drawn"] - Sb["cuts"]).__xor__(A["drawn"] - A["cuts"]).volume(), 2))
    for name, accel, F, M in cut_reactions():
        if name.startswith("unit 50") or "hit" in name:
            sh, pads = bolt_split(F, M)
            print(f"cut {name}: F {np.round(F, 0).tolist()} N, M {np.round(M / 1000, 2).tolist()} N m, bolt axial {np.round([q[2] for q in pads], 0).tolist()} N")

if __name__ == "__main__":
    if "--check" in sys.argv:
        check()
    elif "--reactions" in sys.argv:
        for r in cut_reactions(): print(r[0], np.round(r[2], 1).tolist(), np.round(r[3] / 1000, 3).tolist())
    elif "--piece" in sys.argv:
        piece = ARGS[0]
        h = float(ARGS[1]) if len(ARGS) > 1 else 2.0
        gen = float(ARGS[2]) if len(ARGS) > 2 else (60.0 if piece == "spine" else 25.0)
        name = "scan_stick_" + piece
        ver = VER or "v1"
        out = os.path.join(ROOT, "runs", name, ver)
        if os.path.exists(os.path.join(out, "result.json")): sys.exit(f"runs/{name}/{ver} is finished: never overwritten, pass --ver")
        if os.environ.get("NOX_SETUP_ONLY"):
            print(setup_piece(piece, h).summary())
        else:
            optimise(lambda hh: setup_piece(piece, hh), h=h, gen_cm3=gen, out_dir=out, name=name, settings=Settings(), git="--no-git" not in sys.argv)
