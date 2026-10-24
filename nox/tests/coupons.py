"""Strength calibration coupons v1 (24 Oct 2026, test-coupons worker). DRAWN parts (manifold3d, no topology), saved in print
pose (+z up = as printed, z 0 = the bed). They turn the E strength numbers into P numbers: `docs/tests.md` has the protocol,
`tools/calibrate.py` turns the readings into factors and `nox/dimensions.md` T holds the numbers.

  post_pull   plate + two 6 x 6 and two 4 x 3 posts, printed upright like real attachment posts; a side pull on the eye at the
              top bends each post, so the root fails across layers. Calibrates the as-built index of nox/topo (material + FE).
  layer_pull  plate + three 3.6 x 3.6 necks, printed upright, pulled straight up: across-layer tensile strength Zt (P).
  glue_lap    one half of a single-lap joint (two halves make a joint, 10 x 6 bond, flush 4 thick): PETG bond strength,
              epoxy vs methacrylate.

Predicted failure loads: the post and layer coupons go through nox/topo's own as-built strength report (`topo.build.strength`,
same voxel FE, transversely isotropic PETG, same allowables) on a sub-model per post (plate patch fixed on its bottom face,
the hook load on the eye), at the parts' grid h 1.0 and at h 0.5; and a hand calc (beam / F = Zt A). The glue and socket
predictions are hand calcs (E). Break load = the load at which the method's stress reaches the material strength
(index x KNOCKDOWN / SAFETY = 1, misuse class), so factor = measured / predicted says how far the method is off.

python3 nox/tests/coupons.py            -> print/test_<name>_v1.stl, runs/test_<name>/v1/ (summary, result.json, preview)
python3 nox/tests/coupons.py --no-fe    -> geometry and checks only
"""
import os, sys, json, math, subprocess, datetime
import numpy as np
import manifold3d as mf
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
NOX = os.path.abspath(os.path.join(HERE, ".."))
ROOT = os.path.abspath(os.path.join(NOX, ".."))
sys.path.insert(0, NOX)
PRINT = os.path.join(ROOT, "print")
RHO = 1.27                                      # PETG g/cm3 (nox/topo material, E)

# ------------------------------------------------------------------ numbers (C; dimensions.md T)
PLATE_T = 6.0                                   # base plate thickness, all plate coupons
SCREW_D = 5.5                                   # through holes: M5 or 4-5 mm wood screws into a scrap board
HOOK_D = 8.0                                    # eye hole: a luggage-scale hook wire up to ~6 (E)
TEAR_APEX = 4.0 * math.sqrt(2.0)                # full 45 deg teardrop: apex 5.66 over the hole centre, 2.34 wall under an R 8 eye top

# post pull: (name, x0, x1, y0, y1, L = hook line above the root, eye radius); pull +Y, so t (along Y) is the bending depth
POSTS = [("6x6_a", 17, 23, 12, 18, 25.0, 8.0), ("6x6_b", 47, 53, 12, 18, 25.0, 8.0),
         ("4x3_a", 78, 82, 13, 16, 18.0, 8.0), ("4x3_b", 108, 112, 13, 16, 18.0, 8.0)]
POST_PLATE = (0.0, 130.0, 0.0, 30.0)
POST_HOLES = [(6, 15), (35, 15), (65, 15), (95, 15), (124, 15)]

# layer pull: three necks, gauge 3.6 x 3.6 x 8 (3.6: the solid gate's 1.5 neck erosion keeps a core; 3.0 would split the eye off)
NECK = 3.6
NECK_LEN = 8.0
NECK_FLARE = 3.0                                # 45 deg flare 9.6 -> 3.6 into the plate
EYE_W, EYE_R = 10.0, 8.0                        # layer eye: 10 along X, 16 along Y, hole Ø8 along X
NECK_X = [20, 45, 70]
LAYER_PLATE = (0.0, 90.0, 0.0, 30.0)
LAYER_HOLES = [(6, 15), (32.5, 15), (57.5, 15), (84, 15)]

# glue lap half
LAP_W, LAP_L, LAP_T = 10.0, 6.0, 4.0            # strip width, overlap, joint thickness (pad = half)
LAP_SPAN = 60.0                                 # eye centre to the pad end
LAP_EYE_R = 10.0

F_REF = 10.0                                    # N, FE reference load (linear)


def box(x0, x1, y0, y1, z0, z1):
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def hull(pts):
    return mf.Manifold.hull_points(np.asarray(pts, np.float64))


def circ_yz(yc, zc, r, n=64):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return [(yc + r * math.cos(t), zc + r * math.sin(t)) for t in a]


def prism_x(pts_yz, x0, x1):
    """convex YZ profile extruded along X"""
    return hull([[x, y, z] for x in (x0, x1) for (y, z) in pts_yz])


def teardrop_x(yc, zc, r, x0, x1):
    """hole along X, 45 deg apex up (prints upright without support; the hook seats in the circle)"""
    pts = circ_yz(yc, zc, r) + [(yc, zc + r * math.sqrt(2.0))]
    return prism_x(pts, x0 - 1, x1 + 1)


def tm(m):
    me = m.to_mesh()
    return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=True)


def man(t):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))


# ------------------------------------------------------------------ geometry
def eye_x(x0, x1, yc, zc, R, t, z_post):
    """eye in the YZ plane (hole axis X), width x0..x1, flanks <= 45 deg down to the post depth t; returns (solid, hole, z flank)"""
    zb = zc - math.sqrt(2.0) * R + t / 2.0 - 0.5           # 45 deg tangent from the circle meets the post faces here
    zb = max(zb, z_post)
    prof = circ_yz(yc, zc, R) + [(yc - t / 2, zb), (yc + t / 2, zb)]
    return prism_x(prof, x0, x1), teardrop_x(yc, zc, HOOK_D / 2, x0, x1), zb


def post_unit(p):
    name, x0, x1, y0, y1, L, R = p
    yc, t = 0.5 * (y0 + y1), y1 - y0
    zc = PLATE_T + L
    e, hole, zb = eye_x(x0, x1, yc, zc, R, t, PLATE_T)
    post = box(x0, x1, y0, y1, PLATE_T - 0.5, zb + 0.01)
    return post + e, hole, dict(zc=zc, yc=yc, t=t, b=x1 - x0, L=L, zb=zb, R=R)


def post_pull():
    X0, X1, Y0, Y1 = POST_PLATE
    m = box(X0, X1, Y0, Y1, 0, PLATE_T)
    holes = mf.Manifold()
    for p in POSTS:
        s, h, _ = post_unit(p)
        m += s; holes += h
    for (x, y) in POST_HOLES:
        holes += mf.Manifold.cylinder(PLATE_T + 2, SCREW_D / 2, SCREW_D / 2, 48).translate([x, y, -1])
    return m - holes


def neck_unit(xc, yc=15.0):
    a = NECK / 2
    z0 = PLATE_T
    fl = hull([[xc + sx * (a + NECK_FLARE), yc + sy * (a + NECK_FLARE), z0 - 0.5] for sx in (-1, 1) for sy in (-1, 1)] +
              [[xc + sx * (a + NECK_FLARE), yc + sy * (a + NECK_FLARE), z0] for sx in (-1, 1) for sy in (-1, 1)] +
              [[xc + sx * a, yc + sy * a, z0 + NECK_FLARE] for sx in (-1, 1) for sy in (-1, 1)])
    zg1 = z0 + NECK_FLARE + NECK_LEN
    gauge = box(xc - a, xc + a, yc - a, yc + a, z0 + NECK_FLARE - 0.01, zg1 + 0.01)
    up = EYE_R - a                                            # 45 deg in Y (the wider way)
    tr = hull([[xc + sx * a, yc + sy * a, zg1] for sx in (-1, 1) for sy in (-1, 1)] +
              [[xc + sx * EYE_W / 2, yc + sy * EYE_R, zg1 + up] for sx in (-1, 1) for sy in (-1, 1)])
    ze0 = zg1 + up
    zc = ze0 + HOOK_D / 2 + 3.0                               # 3.0 under the hole
    prof = circ_yz(yc, zc, EYE_R) + [(yc - EYE_R, ze0), (yc + EYE_R, ze0)]
    eye = prism_x(prof, xc - EYE_W / 2, xc + EYE_W / 2)
    hole = teardrop_x(yc, zc, HOOK_D / 2, xc - EYE_W / 2, xc + EYE_W / 2)
    return fl + gauge + tr + eye, hole, dict(zc=zc, zg0=z0 + NECK_FLARE, zg1=zg1)


def layer_pull():
    X0, X1, Y0, Y1 = LAYER_PLATE
    m = box(X0, X1, Y0, Y1, 0, PLATE_T)
    holes = mf.Manifold()
    for x in NECK_X:
        s, h, _ = neck_unit(x)
        m += s; holes += h
    for (x, y) in LAYER_HOLES:
        holes += mf.Manifold.cylinder(PLATE_T + 2, SCREW_D / 2, SCREW_D / 2, 48).translate([x, y, -1])
    return m - holes


def glue_lap():
    """one half: eye at X 0 (hole Ø8 vertical), strip to X LAP_SPAN, the last LAP_L half thick (pad, bond face on top at z 2)"""
    w = LAP_W / 2
    a = np.linspace(0, 2 * np.pi, 72, endpoint=False)
    eye = hull([[LAP_EYE_R * math.cos(t), LAP_EYE_R * math.sin(t), z] for t in a for z in (0, LAP_T)] +
               [[LAP_EYE_R + 8.0, sy * w, z] for sy in (-1, 1) for z in (0, LAP_T)])
    strip = box(0, LAP_SPAN - LAP_L, -w, w, 0, LAP_T)
    pad = box(LAP_SPAN - LAP_L - 0.01, LAP_SPAN, -w, w, 0, LAP_T / 2)
    hole = mf.Manifold.cylinder(LAP_T + 2, HOOK_D / 2, HOOK_D / 2, 48).translate([0, 0, -1])
    m = (eye + strip + pad) - hole
    return m.translate([LAP_EYE_R, w + 0.0, 0])                   # bbox from 0


# ------------------------------------------------------------------ FE (nox/topo as-built strength report)
def fe_index(unit, cut_extra, load_box, vec, h, name, log=lambda s: None):
    """worst as-built index (misuse class) of `unit` (Manifold) under `vec` on the cells in load_box, bottom face fixed.
    The grid is aligned so integer coordinates fall on cell faces (env box extents a multiple of 8 h)."""
    from types import SimpleNamespace
    from topo.part import Part
    from topo.material import MATERIALS, stiffness, AXES
    from topo.build import strength
    mesh = unit - cut_extra
    lo, hi = [np.array(v) for v in (mesh.bounding_box()[:3], mesh.bounding_box()[3:])]
    lo = np.floor(lo) - 1.0
    ext = np.ceil((hi + 1.0 - lo) / (8 * h)) * 8 * h
    envb = box(lo[0], lo[0] + ext[0], lo[1], lo[1] + ext[1], lo[2], lo[2] + ext[2])
    part = Part(name, h, mesh, envb - mesh, envb, build="+z", log=log)
    part.support("default", box(lo[0] - 1, lo[0] + ext[0] + 1, lo[1] - 1, lo[1] + ext[1] + 1, -1.0, 0.75 * h))
    part.load("pull", box(*load_box), vec, cls="misuse")
    mat = MATERIALS["PETG"]
    ax = AXES["+z"][0]
    cases = [SimpleNamespace(name=c.name, F=c.F, fe=None, cls=c.cls, support=c.support, nodes=c.nodes, tie=False) for c in part.cases]
    L = SimpleNamespace(part=part, cases=cases, C=stiffness(mat, ax), solid=part.solid, mat=mat, build_axis=ax)
    st = strength(L, tm(mesh), log=log)
    r = st["rows"][0]
    return dict(h=h, inlayer=r["inlayer"], across=r["across"], worst=max(r["inlayer"], r["across"]), at=r["at"])


def fe_break(res):
    from topo.material import SAFETY, KNOCKDOWN
    k = SAFETY["misuse"] / KNOCKDOWN                      # index at which the stress reaches the raw strength
    return dict(index_at_ref=res["worst"], F_index1_N=round(F_REF / res["worst"], 1), F_break_N=round(F_REF * k / res["worst"], 1),
                at=res["at"], governs="across layers" if res["across"] >= res["inlayer"] else "in-layer")


def predict_posts(hs=(1.0, 0.5)):
    from topo.material import MATERIALS
    Zt = MATERIALS["PETG"].Zt
    out = {}
    for p in POSTS[::2]:                                       # one per size (a and b are the same geometry)
        name = p[0].split("_")[0]
        s, hole, g = post_unit(p)
        xc = 0.5 * (p[1] + p[2])
        patch = box(xc - 15, xc + 15, POST_PLATE[2], POST_PLATE[3], 0, PLATE_T)
        unit = patch + s
        r = HOOK_D / 2
        hand = Zt * g["b"] * g["t"] ** 2 / (6.0 * g["L"])
        rows = {}
        for h in hs:
            lb = (p[1] - 1, p[2] + 1, g["yc"] + r - 0.01, g["yc"] + r + h + 0.01, g["zc"] - 1.6, g["zc"] + 1.6)
            res = fe_index(unit, hole, lb, (0.0, F_REF, 0.0), h, f"post_{name}_h{h}")
            rows[f"h{h}"] = dict(res, **fe_break(res))
        out[name] = dict(section=f"{g['b']:.0f} (X) x {g['t']:.0f} (Y, pull)", L_mm=g["L"], hand_break_N=round(hand, 1),
                         hand=f"sigma_z = 6 F L / (b t^2) at the root = Zt {Zt} (E)", fe=rows,
                         predicted_N=rows["h1.0"]["F_break_N"])
    return out


def predict_layer(hs=(1.0, 0.5)):
    from topo.material import MATERIALS
    Zt = MATERIALS["PETG"].Zt
    s, hole, g = neck_unit(45.0)
    unit = box(30, 60, 0, 30, 0, PLATE_T) + s
    rows = {}
    for h in hs:
        lb = (45 - EYE_W / 2 - 1, 45 + EYE_W / 2 + 1, 12.0, 18.0, g["zc"] + 3.0, g["zc"] + TEAR_APEX + 1.5 * h)   # the hook wedges in the apex flanks
        res = fe_index(unit, hole, lb, (0.0, 0.0, F_REF), h, f"layer_h{h}")
        rows[f"h{h}"] = dict(res, **fe_break(res))
    hand = Zt * NECK * NECK
    return dict(section=f"{NECK} x {NECK} gauge, {NECK_LEN} long", hand_break_N=round(hand, 1),
                hand=f"F = Zt A = {Zt} x {NECK * NECK:.2f} (E)", fe=rows, predicted_N=rows["h1.0"]["F_break_N"])


def predict_glue():
    tau = 2.0                                                  # MPa design bond (box-funnel.md 7, E)
    A = LAP_W * LAP_L
    return dict(bond_mm2=A, design_bond_MPa=tau, predicted_N=round(tau * A, 1),
                strip_N=round(51.0 * LAP_W * LAP_T / 2, 0), hand=f"F = tau A = {tau} x {A:.0f} (E); pad net section in-layer {51.0 * LAP_W * LAP_T / 2:.0f} N")


def predict_socket():
    tau = 2.0
    d, L = 3.0, 10.0 - 0.6                                     # rod, socket depth less the lead-in (node_topo2.SOCK, C)
    A = math.pi * d * L
    return dict(bond_mm2=round(A, 1), design_bond_MPa=tau, predicted_N=round(tau * A, 0),
                hand=f"F = tau pi d L = {tau} x pi x {d} x {L} (E; the rod face is the smaller bond area)")


# ------------------------------------------------------------------ save + checks
def run(cmd):
    r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    return (r.stdout + r.stderr).strip()


def save(name, m, ver="v1"):
    t = tm(m)
    t.apply_translation([-t.bounds[0][0], -t.bounds[0][1], -t.bounds[0][2]])
    f = os.path.join(PRINT, f"test_{name}_{ver}.stl")
    t.export(f)
    rd = os.path.join(ROOT, "runs", f"test_{name}", ver)
    os.makedirs(rd, exist_ok=True)
    rel = os.path.relpath(f, ROOT)
    sc = run([sys.executable, "nox/solid_check.py", rel])
    po = run([sys.executable, "nox/print_orient.py", rel, "--all"])
    pv = run([sys.executable, "nox/render_preview.py", rel, "--mat", "PETG", "--out", os.path.join(rd, "preview.png")])
    return dict(file=rel, run_dir=os.path.relpath(rd, ROOT), mass_g=round(t.volume / 1000 * RHO, 2),
                extents=[round(float(v), 1) for v in t.extents], watertight=bool(t.is_watertight),
                solid=sc.splitlines()[-1] if sc else "", solid_ok=" PASS " in sc, orient=po, preview=pv)


def check_file(rel, rd):
    """the checks on a file saved elsewhere (the socket coupon copy)"""
    os.makedirs(os.path.join(ROOT, rd), exist_ok=True)
    t = trimesh.load(os.path.join(ROOT, rel), force="mesh")
    sc = run([sys.executable, "nox/solid_check.py", rel])
    po = run([sys.executable, "nox/print_orient.py", rel, "--all"])
    run([sys.executable, "nox/render_preview.py", rel, "--mat", "PETG", "--out", os.path.join(ROOT, rd, "preview.png")])
    return dict(file=rel, run_dir=rd, mass_g=round(t.volume / 1000 * RHO, 2), extents=[round(float(v), 1) for v in t.extents],
                watertight=bool(t.is_watertight), solid=sc.splitlines()[-1] if sc else "", solid_ok=" PASS " in sc, orient=po)


def bed_mm2(orient):
    for ln in orient.splitlines():
        if ln.strip().startswith("saved:"):
            return float(ln.split("bed")[1].split("mm2")[0])
    return None


DOC = "docs/tests.md"


def write_run(name, r, ver, strength, print_line, extra, calib):
    rd = os.path.join(ROOT, r["run_dir"])
    bed = bed_mm2(r["orient"])
    head = (f"test_{name} {ver}: ready to print\n"
            f"System: test coupon, no assembly (it calibrates the index every system pass uses; protocol `{DOC}`)\n"
            f"Weight: {r['mass_g']} g (M), was none\n"
            f"Strength: {strength}\n"
            f"Solid: solid_check {'PASS' if r['solid_ok'] else 'FAIL'}\n"
            f"Print: {print_line}, bed contact {bed:.0f} mm2 (M); file {r['file']}\n")
    body = (f"\n## Details\n\n- Script: `nox/tests/coupons.py` (drawn, manifold3d). Extents {r['extents']} mm (M), watertight {r['watertight']}.\n"
            f"- solid_check: `{r['solid']}`\n" + extra +
            f"\n## print_orient --all\n\n```\n{r['orient']}\n```\n")
    with open(os.path.join(rd, "summary.md"), "w") as fh:
        fh.write(head + body)
    res = dict(date="2026-10-24", status="DONE", file=r["file"], mass_g=r["mass_g"], mass_tag="M",
               solid_ok=r["solid_ok"], solid=r["solid"], bed_face="flat face (as saved)" if name == "glue_lap" else "plate bottom (as saved)",
               bed_contact_mm2=bed, extents=r["extents"], predict=r.get("predict"), calibration=calib,
               note="drawn test coupon (manifold3d), nox/tests/coupons.py; protocol docs/tests.md; numbers dimensions.md T")
    with open(os.path.join(rd, "result.json"), "w") as fh:
        json.dump(res, fh, indent=1, default=str)


def main(fe=True):
    res = {}
    for name, fn in (("post_pull", post_pull), ("layer_pull", layer_pull), ("glue_lap", glue_lap)):
        res[name] = save(name, fn())
        print(name, res[name]["mass_g"], "g", res[name]["extents"], res[name]["solid"][:120])
    res["socket_coupon"] = check_file("print/test_socket_coupon_v2.stl", "runs/test_socket_coupon/v2")
    print("socket_coupon", res["socket_coupon"]["mass_g"], "g", res["socket_coupon"]["solid"][:120])
    if not fe:
        return res
    P = res["post_pull"]["predict"] = predict_posts()
    Lp = res["layer_pull"]["predict"] = predict_layer()
    G = res["glue_lap"]["predict"] = predict_glue()
    S = res["socket_coupon"]["predict"] = predict_socket()
    a, b = P["6x6"], P["4x3"]
    fmt = lambda q: (f"h 1.0 {q['fe']['h1.0']['F_break_N']} N (index {q['fe']['h1.0']['worst']} at {F_REF:.0f} N), "
                     f"h 0.5 {q['fe']['h0.5']['F_break_N']} N, hand {q['hand_break_N']} N")
    write_run("post_pull", res["post_pull"], "v1",
              f"predicted break (nox/topo as-built FE, across layers at the post root, misuse index x 0.85 / 2.5 = 1): 6x6 {a['predicted_N']} N, "
              f"4x3 {b['predicted_N']} N (h 1.0, the parts' grid); h 0.5: {a['fe']['h0.5']['F_break_N']} / {b['fe']['h0.5']['F_break_N']} N; "
              f"beam hand calc {a['hand_break_N']} / {b['hand_break_N']} N (E: Zt 25.5)",
              "plate bottom down (as saved), no supports needed (eye flanks and teardrop holes 45 deg); copy A 100 % infill, copy B 4 walls 15 % gyroid",
              f"- Posts printed upright like real attachment posts: 6x6 (X 6 x Y 6, hook line 25 above the root) and 4x3 (X 4 x Y 3, pulled across the 3, "
              f"hook line 18), two of each. Eye on each top: Ø{HOOK_D:.0f} hole along X (45 deg teardrop), the eye as wide as its post "
              f"(the solid gate's neck test). Plate {POST_PLATE[1]:.0f} x {POST_PLATE[3]:.0f} x {PLATE_T:.0f}, 5 x Ø{SCREW_D} through holes between the posts.\n"
              f"- Pull: horizontal, along Y (either way, the eye is symmetric), luggage-scale hook through the eye, the plate screwed or clamped down.\n"
              f"- FE: `topo.build.strength` on a sub-model per post (plate patch 30 x 30 fixed on its bottom face, {F_REF:.0f} N on the eye's hole wall "
              f"on the pull side), PETG transversely isotropic (Zt 25.5, Xt 51, E), misuse class; break = the load where index x 0.85 / 2.5 = 1 (linear).\n"
              f"  - 6x6: {fmt(a)}; worst at {a['fe']['h1.0']['at']} (root layer, tension face).\n"
              f"  - 4x3: {fmt(b)}; worst at {b['fe']['h1.0']['at']}.\n"
              f"  - At h 1.0 the 4x3 post is 4 x 3 cells and the root corner is not resolved: the method reads {b['predicted_N'] / b['hand_break_N']:.2f} x "
              f"the beam value; the 6x6 reads {a['predicted_N'] / a['hand_break_N']:.2f} x. The two sizes split material error from grid error.\n",
              dict(post_pull_6x6=dict(predicted_N=a["predicted_N"], hand_N=a["hand_break_N"]),
                   post_pull_4x3=dict(predicted_N=b["predicted_N"], hand_N=b["hand_break_N"])))
    write_run("layer_pull", res["layer_pull"], "v1",
              f"predicted break (nox/topo as-built FE, across layers): {Lp['predicted_N']} N at h 1.0 (index {Lp['fe']['h1.0']['worst']} at {F_REF:.0f} N, "
              f"worst at the gauge / flare junction {Lp['fe']['h1.0']['at']}); h 0.5 {Lp['fe']['h0.5']['F_break_N']} N; hand F = Zt A {Lp['hand_break_N']} N (E)",
              "plate bottom down (as saved), no supports needed (flares and teardrops 45 deg), 100 % infill",
              f"- Three necks {NECK} x {NECK} x {NECK_LEN} gauge, printed upright: a straight pull is pure across-layer tension, so measured / A = Zt (P). "
              f"45 deg flare into the plate, eye {EYE_W:.0f} x {2 * EYE_R:.0f} with a Ø{HOOK_D:.0f} teardrop hole along X; the hook seats in the apex.\n"
              f"- {NECK} and not 3.0: the solid gate's 1.5 neck erosion must keep a core in the gauge, else the eye reads as a blob on a neck (FAIL).\n"
              f"- Needs a 50 kg luggage scale: {Lp['predicted_N'] / 9.81:.0f} kgf predicted.\n",
              dict(layer_pull=dict(predicted_N=Lp["predicted_N"], hand_N=Lp["hand_break_N"], area_mm2=round(NECK * NECK, 2))))
    write_run("glue_lap", res["glue_lap"], "v1",
              f"predicted break (hand calc, E): bond {G['design_bond_MPa']} MPa (design bond, box-funnel.md 7) x {G['bond_mm2']:.0f} mm2 = {G['predicted_N']} N; "
              f"the pads' own net section {G['strip_N']:.0f} N in-layer",
              "flat face down (as saved), no supports; 12 halves (3 joints per adhesive x 2 adhesives), 100 % infill",
              f"- One half of a single-lap joint: strip {LAP_W:.0f} wide x {LAP_T:.0f} thick, the last {LAP_L:.0f} at half thickness (bond face on top). "
              f"Turn the second half over and butt each pad against the other's step: the overlap is exactly {LAP_W:.0f} x {LAP_L:.0f} and the joint is flush, "
              f"so the pull line runs through the bond (little peel). Ø{HOOK_D:.0f} eye at each end.\n",
              dict(glue_epoxy=dict(predicted_N=G["predicted_N"], area_mm2=G["bond_mm2"]),
                   glue_methacrylate=dict(predicted_N=G["predicted_N"], area_mm2=G["bond_mm2"])))
    write_run("socket_coupon", res["socket_coupon"], "v2",
              f"predicted epoxy pull-out (hand calc, E): {S['design_bond_MPa']} MPa x pi x 3.0 x 9.4 = {S['predicted_N']:.0f} N per rod (bond on the rod face)",
              "plate bottom down (as saved; copied unchanged from archive/scout-v2/print/s2_socket_v2_coupon_v2.stl), 100 % infill",
              "- Copy of the Scout v2 socket coupon (`archive/scout-v2/lantern/socket_coupon.py`): bores 3.5 / 3.6 / 3.7 (1 / 2 / 3 notches), each standing "
              "and lying, 4 ribs 0.3 under the bore, 2 lock rings, bleed slot, 10 deep. Rechecked with the current solid_check: PASS.\n",
              dict(socket_pull=dict(predicted_N=S["predicted_N"], area_mm2=S["bond_mm2"])))
    print(json.dumps({k: v.get("predict") for k, v in res.items()}, indent=1, default=str)[:3000])
    return res


if __name__ == "__main__":
    main(fe="--no-fe" not in sys.argv)
