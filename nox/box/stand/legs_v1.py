"""Box legs v1 (17 Oct 2026, nox/box-stand.md): two bolt-on front legs, one design printed as a mirror pair.
python legs_v1.py [h 2.0] [generated cm3 150]   (NOX_SETUP_ONLY=1: build the problem and stop; NOX_TLIM layout s, NOX_MAXIT)
Run from nox/box/stand/ with the engine on PYTHONPATH; needs legs_loads.json (python legs_loads.py). Lean backend (noxopt_local).
v1 result 17 Oct: 195.3 g (M) per leg, worst 0.52 (landing +X face), one body, function 0/0/0, thin 0, 0 overlaps.

Each leg replaces the four link plates (near and far faces) of the lower 45 deg module it hugs (the +X leg: joints -22.5 and -67.5, between the
0 / -45 / -90 modules; the -X leg mirrored: -157.5 / -112.5), so every bay module stays identical: the leg's two root
pads are link plates: near pads Y 95.5..98.5 (cut to the link pockets) and far pads Y 184.3..187.3 on the far-face
bosses (Spark 17 Oct, arm chat adds them), 16 countersunk M3, and it carries the front foot (X +-160, Y 225, floor Z -160, F) and that side's stub-spider tie boss (r 146 on
the -45 deg diagonal, along Y, E). The upper two tie bosses (45 / 135 deg) cannot be on a leg: they go on tie links.
Drawn (mounting only): 2 root pads with their 8 holes, the tie boss (M3 x 5.7 heat-set, hole 4.0 x 8 along +Y, face
Y 85.5, DEFAULT), the TPU foot pad seat (nox/box/stand/pad_v1.py: pocket 30.6 x 30.6 x 1 in the land face Z -157,
4 floor with a 6.4 hole, 11 x 6 cavity). Everything else generated.
Keep-outs (from arm_cad.bay_module_parts): every module position (r 154.5 cylinder Y 93.5..184.5, all 7 positions incl.
the reserved Overseer bays, less the two link pockets the pads sit in), the bars' sweep (r < 136.5 for Y < 95.5; it
also holds both heads swept at every position), driver access to the 8 screws (r 3.6 along -Y), the spider tie
(r 6 along the tie axis, Y < 85.5), the floor (nothing under the land face). Spider side: nothing at Y < 82 (DEFAULT).
Loads: legs_loads.py (box weight on 4 feet, each front foot off the floor in turn, 13 N rim push in 8 directions at
two masses, Overseer landing 1 kN at 45 deg on each face at two masses; foot reaction on the pad + spider force on the
tie boss). Supports: the 8 screws. Print: land face down (build +Z), no supports (overhang filter on), A1 mini.
"""
import os, sys, math, json
import numpy as np
import manifold3d as mf
import trimesh
_here = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(_here, ".."), os.path.join(_here, "lib")]   # nox/box (arm_layout, arm_cad, arm_topo); lib = local copies
import arm_layout as L, arm_cad as A
from arm_cad import box, cylY, cylZ, hull, tm, man
from arm_topo import PartProblem, run_part, log

H = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
GEN = float(sys.argv[2]) if len(sys.argv) > 2 else 150.0
DZ = 1.0                         # grid shift: puts the bed layer (layer 0) on the land face Z -157 (grid lines at even Z')

FOOT = (160.0, 225.0)            # F (box-stand.md)
Z_LAND = -157.0                  # pad 3 proud of the land face -> floor -160 (pad_v1.py, C)
TIE = (146 / math.sqrt(2), -146 / math.sqrt(2))   # (X, Z) on the -45 deg diagonal, r 146 (E, spider chat)
TIE_Y = 85.5                     # tie boss face (DEFAULT: insert 8 deep ends 0.5 short of the module near face Y 94)
JOINTS = (-22.5, -67.5)          # this leg's two joints (W angles from +X in XZ)
PAD_R = (119.0, 152.0); PAD_W = 8.7; PAD_Y = (95.5, 98.5)   # inside the modules' link pockets (r 118..153, 9 per side)
DOM = dict(x=(30.0, 178.5), y=(82.0, 243.5), z=(Z_LAND, -35.0))
R_MOD, Y_MOD = 154.5, (93.5, 184.2)                # module envelope r 154 + 0.5, Y 94..184 (-0.5 near, +0.2 far)
FAR_Y = (184.3, 187.3)                              # far pads on the modules' far-face bosses (Spark 17 Oct: far-face bosses), C


def rot_to(m, j):
    """the +22.5 deg joint of the top position (W angle 67.5) -> joint angle j (manifold rotate: phi' = phi - theta)."""
    return m.rotate([0, 67.5 - j, 0])


def hole_points():
    a = math.radians(22.5); pts = []
    for sx in (-1, 1):
        for r in L.LINK_R:
            pts.append((r * math.sin(a) + sx * 6.0 * math.cos(a), r * math.cos(a) - sx * 6.0 * math.sin(a)))
    return pts


def root_pad(far=False):
    """link plate cut to the link pocket (the arm chat's plate is 11 per side, the pocket 9): 8.7 per side, r 119..152."""
    a = math.radians(22.5); px, pz = math.cos(a), -math.sin(a)
    ys = FAR_Y if far else PAD_Y
    p = hull([[r * math.sin(a) + s * PAD_W * px, y, r * math.cos(a) + s * PAD_W * pz] for r in PAD_R for s in (-1, 1) for y in ys])
    holes = None
    for x, z in hole_points():
        if far:   # countersunk from +Y (heads face out of the box)
            h = cylY(x, z, ys[0] - 1.0, ys[1] + 1.0, 1.7, 32) + mf.Manifold.cylinder(1.7, 1.7, 3.3, 32).rotate([-90, 0, 0]).translate([x, ys[1] - 1.7, z])
        else:
            h = cylY(x, z, 94.0, 100.0, 1.7, 32) + mf.Manifold.cylinder(1.7, 1.7, 3.3, 32).rotate([90, 0, 0]).translate([x, 97.2, z])
        holes = h if holes is None else holes + h
    return p, holes


def pocket():
    a = math.radians(22.5); px, pz = math.cos(a), -math.sin(a)
    return hull([[r * math.sin(a) + s * 9.0 * px, y, r * math.cos(a) + s * 9.0 * pz] for r in (118.0, 153.0) for s in (-1, 1) for y in (90.0, 98.5)])


def rsq(cx, cy, half, r, z0, z1):
    return hull([[cx + sx * (half - r) + r * math.cos(t), cy + sy * (half - r) + r * math.sin(t), z]
                 for sx in (-1, 1) for sy in (-1, 1) for t in np.linspace(0, 2 * math.pi, 16, endpoint=False) for z in (z0, z1)])


def seat():
    """TPU pad seat (pad_v1.py): 36.6 square block, 3 walls, land face Z -157."""
    cx, cy = FOOT
    blk = rsq(cx, cy, 18.3, 4.0, Z_LAND, Z_LAND + 14.0)
    cut = box(cx - 15.3, cx + 15.3, cy - 15.3, cy + 15.3, Z_LAND - 1.0, Z_LAND + 1.0) + \
        cylZ(cx, cy, Z_LAND + 0.9, Z_LAND + 5.0, 3.2, 32) + cylZ(cx, cy, Z_LAND + 4.99, Z_LAND + 11.0, 5.5, 32)
    return blk, cut


def tie_boss():
    x, z = TIE
    b = cylY(x, z, TIE_Y, Y_MOD[0], 5.0, 48)
    c = cylY(x, z, TIE_Y - 1.0, TIE_Y + 8.0, 2.0, 32)
    return b, c


def parts():
    pads = None; holes = None; pockets = None; scr = []
    for j in JOINTS:
        for far in (False, True):
            p, h = root_pad(far)
            pads = rot_to(p, j) if pads is None else pads + rot_to(p, j)
            holes = rot_to(h, j) if holes is None else holes + rot_to(h, j)
        pockets = rot_to(pocket(), j) if pockets is None else pockets + rot_to(pocket(), j)
        th = math.radians(67.5 - j)
        for x, z in hole_points():
            scr.append((x * math.cos(th) + z * math.sin(th), -x * math.sin(th) + z * math.cos(th)))
    sblk, scut = seat()
    tb, tc = tie_boss()
    drawn = pads + sblk + tb
    access = None
    for x, z in scr:
        c = cylY(x, z, 60.0, 95.6, 3.6, 32) + cylY(x, z, FAR_Y[1] - 0.1, 260.0, 3.6, 32)
        access = c if access is None else access + c
    tx, tz = TIE
    keep = (cylY(0, 0, Y_MOD[0], Y_MOD[1], R_MOD, 240) - pockets) + cylY(0, 0, 0.0, 95.5, 136.5, 240) + access + \
        cylY(tx, tz, 0.0, TIE_Y, 6.0, 48)
    cuts = holes + scut + tc + access + cylY(tx, tz, 0.0, TIE_Y, 6.0, 48)
    dom = box(DOM["x"][0], DOM["x"][1], DOM["y"][0], DOM["y"][1], DOM["z"][0], DOM["z"][1])
    env = (dom - keep) + drawn
    reg = dict(
        screws=[cylY(x, z, 94.0, 100.0, 5.5, 24) for x, z in scr] + [cylY(x, z, FAR_Y[0] - 1.0, FAR_Y[1] + 1.0, 5.5, 24) for x, z in scr],
        foot=box(FOOT[0] - 18.3, FOOT[0] + 18.3, FOOT[1] - 18.3, FOOT[1] + 18.3, Z_LAND, Z_LAND + 5.0),
        tie=cylY(tx, tz, TIE_Y - 0.5, Y_MOD[0], 5.5, 32))
    return dict(drawn=drawn, cuts=cuts, env=env, keep=keep, regions=reg, screws=scr, pads=pads)


def shift(m): return m.translate([0, 0, DZ])


def checks(part_m, P_parts):
    """overlaps (mm3) of the built leg with everything it must clear, both legs fitted."""
    ann = cylY(0, 0, 94.0, L.Y_FAR_WALL, 154.0, 240) - cylY(0, 0, 93.0, L.Y_FAR_WALL + 1, 99.5, 240)
    mods = None
    for a in tuple(L.SCOUT_BAYS) + tuple(L.OVERSEER_BAYS):
        w = (ann ^ A.wedge(22.5, L.JOINT_GAP, 93.0, L.Y_FAR_WALL + 1)).rotate([0, 90.0 - a, 0])
        mods = w if mods is None else mods + w
    pk = None
    for j in (-22.5, -67.5, -112.5, -157.5):
        q = rot_to(pocket(), j); pk = q if pk is None else pk + q
    mods = mods - pk                                       # the link pockets are outside every module
    plates = None
    for j in (22.5, 157.5):                                # the remaining link plates (arm chat; 67.5 / 112.5 face the drone gap)
        pl = rot_to(A.link_plate(), j); plates = pl if plates is None else plates + pl
    mirror = part_m.mirror([1, 0, 0])
    tx, tz = TIE
    out = dict(
        modules_all_7=(part_m ^ mods).volume(),
        bars_and_heads_sweep=(part_m ^ cylY(0, 0, 0.0, 95.5, 136.5, 240)).volume(),
        other_link_plates=(part_m ^ plates).volume(),
        spider_ties=((part_m + mirror) ^ (cylY(tx, tz, -3.0, TIE_Y, 6.0) + cylY(-tx, tz, -3.0, TIE_Y, 6.0))).volume(),
        floor=(part_m ^ box(-300, 300, -300, 400, -400, Z_LAND - 0.01)).volume(),
        other_leg=(part_m ^ mirror).volume(),
        screw_access=(part_m ^ sum([cylY(x, z, 60.0, 95.4, 3.5) for x, z in P_parts["screws"]], start=box(0, 0.01, 0, 0.01, 0, 0.01))).volume())
    return {k: round(v, 2) for k, v in out.items()}


if __name__ == "__main__":
    P = parts()
    drawn, cuts, env = shift(P["drawn"]), shift(P["cuts"]), shift(P["env"])
    bb = env.bounding_box(); log(f"env bbox {[round(v, 1) for v in bb]} (Z shifted +{DZ})")
    pp = PartProblem("legs_v1", H, drawn, cuts, env, build="+z", overhang=True, pad=0)
    sup = None
    for s in P["regions"]["screws"]: sup = shift(s) if sup is None else sup + shift(s)
    pp.support("default", sup)
    LC = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "legs_loads.json")))
    for c in LC["cases"]:
        f, t = np.array(c["foot"]), np.array(c["tie"])
        F = np.zeros(pp.ndof)
        if np.linalg.norm(f) > 0.05:
            el = pp.elems(shift(P["regions"]["foot"]), pp.solid | pp.design); nd = pp.nodes_of(el)
            for a in range(3): F[3 * nd + a] += f[a] / len(nd)
        if np.linalg.norm(t) > 0.05:
            el = pp.elems(shift(P["regions"]["tie"]), pp.solid | pp.design); nd = pp.nodes_of(el)
            for a in range(3): F[3 * nd + a] += t[a] / len(nd)
        if not F.any(): continue
        import printopt as PO
        pp.cases.append(PO.LoadCase(c["name"], F, cls=c["cls"], support="default"))
    log(f"{len(pp.cases)} load cases")
    if os.environ.get("NOX_SETUP_ONLY"): sys.exit(0)
    import noxopt as NO, noxopt_local as NL
    NO.Layout = NL.Layout          # lean backend: printopt's COO indices need > 7 GB here; 28 cases = 6 basis loads (superposition)
    # build envelope: 0.6 below the land face so the polish (clip 0.5) can reach the bed; the exact trim at Z -157 follows
    slab = (box(DOM["x"][0], DOM["x"][1], DOM["y"][0], DOM["y"][1], Z_LAND - 0.6, Z_LAND + 0.01) - P["keep"])
    env_b = env + shift(slab)
    res, mesh = run_part(pp, drawn, cuts, env_b, GEN, "out/legs_v1_shifted.stl", maxit=int(os.environ.get("NOX_MAXIT", 200)),
                         time_limit=float(os.environ.get("NOX_TLIM", 5400)))   # NOX_MAXIT=121: build from the checkpoint
    trim = box(-400, 400, -400, 600, Z_LAND + DZ, 400)
    mesh = tm(man(mesh) ^ trim)                                    # envelope trim: flat bed face at the land face (exact)
    res["mass_g"] = mesh.volume / 1000 * 1.27; res["bodies"] = len(man(mesh).decompose())
    os.makedirs("out", exist_ok=True)
    mesh.apply_translation([0, 0, -DZ])
    mesh.export("out/box_leg_R_v1_W.stl")                         # in W, for fit checks
    left = mesh.copy(); left.apply_transform(np.diag([-1, 1, 1, 1])); left.invert()
    left.export("out/box_leg_L_v1_W.stl")
    ch = checks(man(mesh), P); log("CHECKS " + json.dumps(ch))
    for nm, m in (("R", mesh), ("L", left)):                      # print orientation: land face on the bed (Z up), at origin
        t = m.copy(); t.apply_translation(-t.bounds[0]); t.export(f"out/box_leg_{nm}_v1.stl")
    bb = mesh.bounds; res["bbox"] = [round(float(v), 1) for v in (bb[1] - bb[0])]; res["checks"] = ch
    json.dump(res, open("legs_v1_result.json", "w"), indent=1)
    log("DONE " + json.dumps({k: res[k] for k in ("mass_g", "worst", "worst_case", "bodies", "function_check", "thin", "bbox")}))
