"""Scout v3 electronics tail mount v3 (28 Oct 2026): the round-5 mount (electronics_system.py, PR 130, runs/scout_electronics/v2/)
refit to scout_layout v13 (PR 169: the catch re-trimmed to 2.5 aft of the thrust centre, 7.5 aft of v12; the Radxa and the UBEC in
v13's poses, riding the catch; the XR2 placed here) with the FC on top of the plate (Spark 27 Oct; hard on 3 standoffs, the belly
mount v3 pass, nox/scout/v3/belly_mount_v3.py).
`python3 nox/scout/v3/electronics_v3.py` -> runs/scout_electronics/v3/. Seeded: one run gives one set of numbers.

What changes from round 5 (everything else is electronics_system.py's, imported and never changed):
  - the model: main's assembly (layout v13), v13's shell and links, the catch at v13's 2.5 aft; the layout's placeholder boxes for
    this mount dropped; the belly-mount placeholder cut to the requirement (its tail face x -31.0 over |y| <= 22, plate-fixed);
  - the FC on top of the plate (plate-fixed): the board on 3 M3 standoffs x 12 over the front and side AIO holes (PCB z 34.0..35.6,
    board z 30.8..38.8, the belly mount v3 pass's DEFAULT), the FC's leads on the plate top; the cables run to its pads; the layout's
    under-plate FC USB-C path is gone (the FC on top's own path is checked in the belly mount v3 pass);
  - the wash (CLAUDE.md 8): with the catch 7.5 further aft the tail block's block, gusset and fin sit behind the plate's tail edge
    (x -50); every printed piece is cut to >= 3 outside the swept disks wherever it is not in the plate's shadow (under the carbon,
    inside its outline); round 5 needed none of this at v12;
  - the plate interfaces kept plate-fixed: the 2 M3 at (-42, +-16) in the tail tabs, the foot's tip at x -14, the web to x -31.
Drone frame: x nose, y left, z up, plate underside 18.5, top 22.0, thrust centre x 0. Tags: C computed on v13's geometry, E estimate.
"""
import os, sys, io, json, math, time
import numpy as np, trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import electronics_system as es
from electronics_system import (ROOT, ASM, box, cyl, union, diff, centroid, describe, cables, item_meshes, keepclear_plug, sweep_out,
                                checks, fmt_checks, fit_table, cg_final, slide_wide, loads_mesh, modes_mesh, one_body, run_system_check,
                                funnel_line, funnel_z, surf, wash_dist, under_plate, ubec_region, pose_json, preview, place_xr2, bay_poses,
                                printed_g, MASS, GAP, AIR, TOOL, DISK_R, WASH, MOTORS, PLATE_BOLTS, FOOT_X1, WEB_X1, BLK_Z, BOLT_DX, CONTACT_MM3, INFILL)

V13 = os.path.join(ROOT, "runs", "scout_layout", "v13")
RUN = os.path.join(ROOT, "runs", "scout_electronics", "v3")
es.RUN = RUN
es.LOG = io.StringIO()
DC = -json.load(open(os.path.join(V13, "result.json")))["tab_x"]      # +2.5: the catch 2.5 aft (C)
DC_V12 = -json.load(open(os.path.join(ROOT, "runs", "scout_layout", "v12", "result.json")))["tab_x"]
PLATE_TAIL = -50.0                                                       # the plate's tail edge (C, plate mesh)
CAGE_WORST_V13 = 9.13          # the cage's worst crash index on v13: shell rod buckling 9.13 x Pcr in the 35 g crash (F, frame model v4,
                               # runs/scout_frame/v4/summary.md; Spark 27 Oct: the 3.0 rods are the fuse). Round 5's 0.59 was the 6/4 cage's
es.CAGE_CRASH_WORST = CAGE_WORST_V13
OLD_CAGE_LINE = 0.59 / 1.5     # round 5's stricter order line (the 6/4 cage), reported and preferred
# the FC on top (the belly mount v3 pass, DEFAULT; plate-fixed): numbers from that pass, rechecked there on this mount
FC_C, FC_YAW, FC_W = (1.9, 0.0), 45.0, 36.0
FC_PCB, FC_ENV = (34.0, 35.6), (30.8, 38.8)
FC_STO = [(1.9, -18.031), (19.931, 0.0), (1.9, 18.031)]                 # the front and side AIO holes (P: the diamond at (1.9, 0))
STO_D = 5.5 / math.cos(math.radians(30))
WIRES_G, WIRES_CG = 20.0, [3.0, 0.0, 29.0]                               # the FC's leads on the plate top (E, the belly mount v3 pass)


def say(*a):
    es.say(*a)


def fc_on_top():
    a = math.radians(FC_YAW); h = FC_W / 2
    fc = box([-h, -h, FC_ENV[0]], [h, h, FC_ENV[1]])
    T = trimesh.transformations.rotation_matrix(a, [0, 0, 1]); T[:3, 3] = [FC_C[0], FC_C[1], 0]; fc.apply_transform(T)
    sto = union([cyl([x, y, 22.0], [x, y, FC_PCB[0]], STO_D / 2, 6) for x, y in FC_STO] + [cyl([x, y, 16.1], [x, y, 18.5], STO_D / 2, 6) for x, y in FC_STO])
    return fc, sto


def wash_cutter(M):
    """the region a printed piece of this mount may not occupy: inside a swept disk + 3 (plan), except under the carbon plate inside
    its outline (the plate's shadow); above the plate top every point counts"""
    import shapely.geometry as sg, shapely.ops as so
    disks = so.unary_union([sg.Point(mx, my).buffer(DISK_R + WASH + 0.05, 128) for mx, my in MOTORS])
    sec = M["P"]["frame plate (Bee35 carbon)"]["mesh"].section(plane_origin=[0, 0, 20.0], plane_normal=[0, 0, 1])
    plate = so.unary_union([sg.Polygon(e[:, :2]) for e in sec.discrete]).buffer(0)
    out = []
    for pg, z0, z1 in ((disks.difference(plate), -100.0, 22.0), (disks, 22.0, 150.0)):
        for g in (pg.geoms if pg.geom_type == "MultiPolygon" else [pg]):
            m = trimesh.creation.extrude_polygon(g, z1 - z0); m.apply_translation([0, 0, z0]); out.append(m)
    return out


_mount_v12 = es.mount_v12


def mount_v13(P, dc, xr2=None, strut=False):
    """round 5's mount (electronics_system.mount_v12) with every printed piece cut out of the wash (see wash_cutter); the cut volume
    is kept for the log"""
    parts = _mount_v12(P, dc, xr2, strut)
    cut = es.MODEL.setdefault("_wash_cut", wash_cutter(es.MODEL)); out = {}
    for k, (m, g, c, note) in parts.items():
        v0 = abs(m.volume); m2 = diff(m, cut)
        if len(m2.split(only_watertight=False)) > 1:                                     # a sliver left over: keep the largest body
            m2 = max(m2.split(only_watertight=False), key=lambda q: abs(q.volume))
        solid = k.startswith("belly-mount tail block") and es.BLOCK_SOLID
        g2 = printed_g(m2, 1.0 if solid else INFILL)
        out[k] = (m2, g2, centroid(m2)[0], note + f"; cut out of the wash ({v0 - abs(m2.volume):.0f} mm3)")
        es.MODEL.setdefault("_wash_cut_mm3", {})[k] = round(v0 - abs(m2.volume), 1)
    return out


es.mount_v12 = mount_v13                    # place_xr2 and the loops below use it


def load_v13():
    M = es.load_model(ASM, V13); M["dc_ref"] = DC
    bm = M["P"]["mount + nose legs (Scout)"]                                           # the belly mount placeholder cut to the requirement
    bm["mesh"] = diff(bm["mesh"], [box([-300.0, -22.0, 0.0], [WEB_X1, 22.0, 30.0])])
    fc, sto = fc_on_top()
    a = math.radians(FC_YAW)
    M["P"]["FC"].update(mesh=fc, g=10.0, c=np.array([FC_C[0], FC_C[1], sum(FC_PCB) / 2]), catch=False, tag="P")
    M["P"]["FC standoffs (3, M3 aluminium)"] = dict(mesh=sto, g=4.5, tag="E", swap=False, role="bought", protect=False,
                                                    c=np.asarray(sto.center_mass), how="volume", catch=False, spec={})
    M["P"]["wires"].update(mesh=box([35.8, -0.2, 24.0], [36.2, 0.2, 24.4]), g=WIRES_G, c=np.array(WIRES_CG), catch=False)
    M["A"]["access"] = [x for x in M["A"]["access"] if not x["name"].startswith("FC USB-C")]   # the under-plate path: gone with the FC
    M["A"]["contact_pairs"] = [c for c in M["A"]["contact_pairs"] if "FC" not in c[:2]]
    return M


def fc_point(dc): return np.array([FC_C[0], FC_C[1], FC_PCB[1]])            # the FC's pads on top (C)


es.fc_point = fc_point
es.usb_path = lambda dc, M=None: (np.array([1e4, 1e4, 1e4]), np.array([1.0, 0, 0]))   # no under-plate FC USB-C path any more


def manifest_v13(res, dc, mp):
    man, keys = es.manifest_v12(res, dc, mp)
    for q in man["parts"]:
        if q["name"] in ("FC", "wires"):
            q.pop("mesh", None); q.pop("box", None); q["trimesh"] = es.MODEL["P"][q["name"]]["mesh"]
            q["mass_g"] = es.MODEL["P"][q["name"]]["g"]; q["cg"] = np.round(es.MODEL["P"][q["name"]]["c"], 3).tolist()
    man["parts"].append(dict(name="FC standoffs (3, M3 aluminium)", role="bought", trimesh=es.MODEL["P"]["FC standoffs (3, M3 aluminium)"]["mesh"],
                             mass_g=4.5, tag="E", geom_tag="C", cg=np.round(es.MODEL["P"]["FC standoffs (3, M3 aluminium)"]["c"], 3).tolist()))
    man["contact_pairs"] += [["FC", "FC standoffs (3, M3 aluminium)", 1e9], ["FC standoffs (3, M3 aluminium)", "frame plate (Bee35 carbon)", 1e9]]
    man["name"] = "Scout v3 with the tail electronics mount v3 (layout v13, the FC on top)"
    return man, keys


def run_v13():
    t0 = time.time(); np.random.seed(0)
    os.makedirs(RUN, exist_ok=True)
    es.MODEL = load_v13(); M = es.MODEL; dc = DC
    fl = funnel_line(dc)
    say("Scout v3 electronics tail mount v3 (28 Oct 2026): round 5's mount (nox/scout/v3/electronics_system.py, PR 130) refit to scout_layout v13 "
        f"(PR 169: the catch {dc:+.1f}, {dc - DC_V12:+.1f} from v12's {DC_V12:+.1f}: everything on the catch moved {dc - DC_V12:.1f} aft against the plate), the Radxa "
        "and UBEC in v13's poses, the FC on top of the plate (hard on 3 standoffs, the belly mount v3 pass); the belly-mount placeholder cut to x -31.0 over |y| <= 22")
    say(f"model: {len(M['P'])} parts in pose (the layout's 3 placeholder boxes for this mount dropped); shell {len(M['nodes'])} nodes / {len(M['rods'])} rods + "
        f"{len(M['links'])} links; funnel line from v13's ring tail node and tail point (+3): (%.1f, %.1f) -> (%.1f, %.1f)" % fl
        + f"; plate tail edge x {PLATE_TAIL}, notch front x {es.notch_front():.1f}")
    an = es.anchors(dc)
    say(f"anchors at v13 (C): receiver tail face x {an['rec_tail']:.1f} ({an['rec_tail'] - PLATE_TAIL:+.1f} against the plate's tail edge), the block x {an['xb1']:.1f}..{an['xb0']:.1f}, "
        f"the belly placeholder's tail x {an['belly_tail']:.1f}; the plate bolts {PLATE_BOLTS} stay plate-fixed in the tail tabs (x -50..-40, |y| 9..30)")
    P = bay_poses()
    for n, (o, t) in P.items(): say("   " + describe(n, o, t))
    o, t = P["Radxa Zero 3W"]; up = o["R"] @ np.array([0, -1.0, 0]); tilts = []
    for ang in range(0, 41, 5):
        for sd in (1, -1):
            d = np.array([0.0, sd * math.sin(math.radians(ang)), math.cos(math.radians(ang))]) if ang else up
            es.USB1_DIR = d; v = sweep_out(dict(poses=P))["USB-C 1"]; tilts.append((ang, sd, v["least_clear_mm"]))
            if v["least_clear_mm"] >= TOOL / 2: break
        else: continue
        break
    say("   USB-C 1 tool line (Ø10): " + ", ".join(f"{a} deg {'+y' if s_ > 0 else '-y'}: {c:.1f}" for a, s_, c in tilts) + f" -> dir {np.round(es.USB1_DIR, 3).tolist()}")
    mp0 = mount_v13(P, dc)
    xr = place_xr2(P, dc, mp0)
    if xr is None: raise SystemExit("no XR2 pose")
    P["XR2"] = xr
    say("   " + describe("XR2", *xr))
    cab = cables(dc, {n: v + P["Radxa Zero 3W"][1] for n, v in P["Radxa Zero 3W"][0]["pts"].items()},
                 {n: v + P["UBEC"][1] for n, v in P["UBEC"][0]["pts"].items()}, {n: v + xr[1] for n, v in xr[0]["pts"].items()})
    lo = np.min([np.min([b[0] for b in o["body"]], 0) + t for o, t in P.values()], 0); hi = np.max([np.max([b[1] for b in o["body"]], 0) + t for o, t in P.values()], 0)
    res = dict(poses=P, cab=[(n, float(g), np.asarray(m), float(L)) for n, g, m, L in cab], lo=lo, hi=hi, dc=dc)
    base = dict(SPINE_SKIN=True, SPINE_TUBE=20.0, UBOX=99.0, BLOCK_SOLID=True, FOOT_TOP=45.0, LOW_AFT=20.0)
    VARS = [("V0 round 4", {}), ("V1 tube 14", dict(SPINE_TUBE=14.0)), ("V2 tube 14 + UBEC box 6", dict(SPINE_TUBE=14.0, UBOX=6.0)),
            ("V3 flanges only + UBEC box 6", dict(SPINE_SKIN=False, UBOX=6.0)),
            ("V4 V2 + block walls 2.4 / 20 %", dict(SPINE_TUBE=14.0, UBOX=6.0, BLOCK_SOLID=False)),
            ("V5 V4 + foot to z 34", dict(SPINE_TUBE=14.0, UBOX=6.0, BLOCK_SOLID=False, FOOT_TOP=34.0)),
            ("V6 V5 + lower back 6 aft", dict(SPINE_TUBE=14.0, UBOX=6.0, BLOCK_SOLID=False, FOOT_TOP=34.0, LOW_AFT=6.0)),
            ("V7 V3 + block 2.4 / 20 % + foot 34 + lower back 6", dict(SPINE_SKIN=False, UBOX=6.0, BLOCK_SOLID=False, FOOT_TOP=34.0, LOW_AFT=6.0)),
            ("V8 V5 + lower back 14 aft", dict(SPINE_TUBE=14.0, UBOX=6.0, BLOCK_SOLID=False, FOOT_TOP=34.0, LOW_AFT=14.0))]
    say("variants (round 5's), sized on dimensions.md L (crash 35 g, 6 faces + 8 corners, <= 1.0; flight 4 g / 2 g and landing 6 g, <= 0.5; break order in the "
        "crash cases: the yoke bolts or the tail block >= 1.5 x before the carrier); the 50 N bay-corner case (E) reported; every piece cut out of the wash:")
    vrows = []
    for vn, cfg in VARS:
        c_ = dict(base, **cfg); es.__dict__.update(c_)
        mpv = mount_v13(P, dc, xr); ldv = loads_mesh(res, dc, mpv); cgv = cg_final(res, dc, mpv); bk = ldv["by_kind"]
        r_ = dict(name=vn, cfg=c_, mass=round(sum(v[1] for v in mpv.values()), 1), pieces={k: v[1] for k, v in mpv.items()},
                  cage_first_order=bool(bk["crash"]["worst"] <= es.CAGE_CRASH_WORST / 1.5),
                  cg=[round(cgv["hin"], 2), round(cgv["hout"], 2)], crash=bk["crash"]["worst"], flight=bk["flight"]["worst"], landing=bk["landing"]["worst"],
                  E=bk["E"]["worst"], old_line=bool(bk["crash"]["worst"] <= OLD_CAGE_LINE), order_ok=bk["crash"]["order_ok"], order_fails=bk["crash"]["order_fails"], l_pass=ldv["l_pass"],
                  one_body=all(one_body(v[0]) == 1 for v in mpv.values()), wash_cut=dict(M.get("_wash_cut_mm3", {})))
        vrows.append(r_)
        say(f"   {vn}: mount {r_['mass']} g ({', '.join(f'{k} {g:.1f}' for k, g in r_['pieces'].items())}); CG {r_['cg'][0]:.2f} / {r_['cg'][1]:.2f} at {dc:+.1f}; "
            f"worst crash {r_['crash']:.2f}, flight {r_['flight']:.2f}, landing {r_['landing']:.2f} (L pass {r_['l_pass']}); L breaking order (mount <= cage "
            f"{es.CAGE_CRASH_WORST} / 1.5, v13's rods) {r_['cage_first_order']} (round 5's line {OLD_CAGE_LINE:.2f}: {r_['old_line']}); yoke / block before the carrier {r_['order_ok']} {r_['order_fails']}; "
            f"50 N bay corner (E) {r_['E']:.2f}; one body {r_['one_body']}; wash cut {r_['wash_cut']} mm3")
    ok_ = [r for r in vrows if r["l_pass"] and r["cage_first_order"] and r["one_body"] and max(r["cg"]) <= 3.0]
    pick = min(ok_, key=lambda r: (not r["old_line"], len(r["order_fails"]), r["mass"])) if ok_ else vrows[0]   # round 5's stricter line first
    es.__dict__.update(pick["cfg"])
    say(f"   picked (passes L, the breaking order against v13's cage and the balance at {dc:+.1f}; round 5's stricter line where one holds it; then the fewest cases where the yoke or block is not 1.5 x before the carrier; "
        f"then the lightest): {pick['name']}" + ("" if ok_ else " (NONE passes: V0 shown)"))
    out = {}
    for label, strut in (("A", False), ("B", True)):
        mp = mount_v13(P, dc, xr, strut=strut)
        say(f"concept {label} ({'A + a strut to the tail point, clip fuse' if strut else 'yoke + spine on the belly-mount tail block'}):")
        bodies = {}
        for k, (m, g, c, note) in mp.items():
            bodies[k] = one_body(m)
            say(f"   {k}: {g:.1f} g (E), CG ({c[0]:.1f}, {c[1]:.1f}, {c[2]:.1f}), x {m.bounds[0][0]:.1f}..{m.bounds[1][0]:.1f}, y {m.bounds[0][1]:.1f}..{m.bounds[1][1]:.1f}, "
                f"z {m.bounds[0][2]:.1f}..{m.bounds[1][2]:.1f}; bodies {bodies[k]}, watertight {m.is_watertight}; {note}")
        ck = checks(res, dc, mp)
        for l in fmt_checks(ck): say(l)
        fp = fit_table(res, mp, dc)
        say(f"   fit table ({len(fp)} pairs within 10; a declared bearing contact passes at overlap <= {CONTACT_MM3} mm3, every other pair needs >= 2.0): "
            f"{sum(r['ok'] for r in fp)} / {len(fp)} pass")
        for r in fp: say(f"      {'ok  ' if r['ok'] else 'FAIL'} {r['a']} / {r['b']}: overlap {r['overlap_mm3']} mm3, gap {r['gap_mm']:.2f} ({r['kind']})")
        sw = sweep_out(res)
        cg = cg_final(res, dc, mp)
        sc = slide_wide(res, dc, mp, True); bs = min(sc, key=lambda r: max(r[1], r[2]))
        ld = loads_mesh(res, dc, mp)
        st = None
        if strut:
            free = M["nodes"][M["nring"]:]; node = free[np.argmin(free[:, 0])]
            o_, t_ = P["Radxa Zero 3W"]; bR_ = o_["body"][0] + t_
            top = np.array([bR_[0][0] + 6.0, (es.SPINE_Y[0] + es.SIDE_Y[0]) / 2, es.RADXA_POSE["ztop"] - 3.5])
            u_ = (node - top) / np.linalg.norm(node - top); st = (top, np.abs(u_), es.E_PETG * math.pi * 9.0 / np.linalg.norm(node - top))
        md = modes_mesh(res, dc, mp, ld, st)
        g_mount = sum(v[1] for v in mp.values())
        man, keys = manifest_v13(res, dc, mp)
        r_sc, txt = run_system_check(man, os.path.join(RUN, f"system_check{'' if label == 'A' else '_alt_B'}.txt"))
        say(f"   mass: mount {g_mount:.1f} g, cables {sum(g for _, g, _, _ in res['cab']):.1f} g (E); CG pack in {cg['hin']:.2f}, out {cg['hout']:.2f} mm at catch {dc:+.1f} "
            f"({cg['mass']:.0f} g); best catch with the bay riding it {bs[0]:+.2f}: {bs[1]:.2f} / {bs[2]:.2f}")
        say(f"   modes (E, from the sections): lateral {md['lateral']:.0f} Hz, vertical {md['vertical']:.0f} Hz ({md['carried_g']} g carried at {md['carried_cg']}, from the plate bolts x {md['root_x']})")
        for kind, v in ld["by_kind"].items():
            say(f"   loads, {kind} (limit {v['limit']}): worst {v['worst']:.2f} (fuse {v['fuse']:.2f}, tail block {v['root']:.2f}, carrier {v['carrier']:.2f}), "
                f"pass {v['pass']}; a replaceable piece 1.5 x before the carrier in every case {v['order_ok']} {v['order_fails']}")
        for k, v in ld["cases"].items(): say(f"      {k}: " + json.dumps(v))
        say("   weakest sections (least I): " + json.dumps(ld["weakest_sections"]))
        for nm, part, p, d, cl in keys: say(f"   key {nm}: from {np.round(p, 1).tolist()} along {np.round(d, 3).tolist()}, clear of rods / nodes (0.5) and prop disks (0) {cl:.2f}")
        for k, v in sw.items(): say(f"   tool Ø10, {k}: from {v['start']} along {v['dir']}, out of the shell at {v['exits_shell_at_mm']} mm, least clear of a rod / node {v['least_clear_mm']} (need >= 5)")
        say("   " + txt.splitlines()[0]); say("   " + txt.splitlines()[-1][:900])
        out[label] = dict(mp=mp, ck=ck, fp=fp, sw=sw, sc_txt=txt, cg=cg, sc=sc, bs=bs, ld=ld, md=md, g=g_mount, sc_line=txt.splitlines()[-1], r=r_sc, bodies=bodies,
                          keys=[(nm, np.round(p, 1).tolist(), np.round(d, 3).tolist(), round(cl, 1)) for nm, part, p, d, cl in keys])
    A = out["A"]; mp = A["mp"]
    o, t = P["Radxa Zero 3W"]; ou, tu = P["UBEC"]
    bR = o["body"][0] + t; bU = ou["body"][0] + tu
    fm = {n: round(v["funnel"], 1) for n, v in A["ck"].items()}
    say(f"funnel line at catch {dc:+.1f}: (%.1f, %.1f) -> (%.1f, %.1f); margins (part's lowest point over the line): " % fl + json.dumps(fm))
    wsh = {n: round(v["wash"], 1) for n, v in A["ck"].items()}
    say("wash (plan distance outside the swept disks, the plate's shadow excluded; need >= 3): " + json.dumps(wsh))
    ur = dict(at_layout_catch=ubec_region(res, dc, mp))
    say("UBEC pose region (balance <= 3 pack in / out at this catch, funnel >= 0, wash >= 3; everything else fixed): " + json.dumps(ur))
    gapRU = bU[0][1] - bR[1][1]; kp = keepclear_plug(o, t)
    say(f"pose check: UBEC to Radxa component face {gapRU:.1f} mm (need >= {GAP:.0f} and {AIR:.0f} of air over the SoC)")
    allm = trimesh.util.concatenate([es.at_dc(n, dc) for n in M["P"] if n not in es.MOVED and n not in es.PLACEHOLDERS] + list(item_meshes(res).values()) + [v[0] for v in mp.values()])
    ext = allm.bounds[1] - allm.bounds[0]
    say(f"drone bounding box with the mount: {ext[0]:.0f} x {ext[1]:.0f} x {ext[2]:.0f} mm, {np.prod(ext) / 1e6:.1f} L")
    preview([(f"A (picked): yoke + spine on the belly-mount tail block, v13 poses, catch {dc:+.1f}; CG {A['cg']['hin']:.1f} / {A['cg']['hout']:.1f} mm", res, dc, mp, M),
             (f"B: A + strut to the tail point (clip fuse); CG {out['B']['cg']['hin']:.1f} / {out['B']['cg']['hout']:.1f} mm", res, dc, out["B"]["mp"], M)],
            os.path.join(RUN, "preview.png"))
    for f in os.listdir(RUN):
        if f.endswith(".stl"): os.remove(os.path.join(RUN, f))
    files = {}
    for k, (m, g, c, note) in mp.items():
        fn = "".join(ch_ if ch_.isalnum() else "_" for ch_ in k.lower())[:40] + ".stl"; m.export(os.path.join(RUN, fn)); files[k] = os.path.relpath(os.path.join(RUN, fn), ROOT)
    fit_items = dict(item_meshes(res)); fit_items["Radxa Zero 3W"] = es.radxa_envelope(o, t, plugs=True)   # the items as fitted (for the system pass)
    for k, m in fit_items.items():
        fn = "".join(ch_ if ch_.isalnum() else "_" for ch_ in k.lower())[:30] + "__as_fitted.stl"; m.export(os.path.join(RUN, fn)); files[k] = os.path.relpath(os.path.join(RUN, fn), ROOT)
    env = dict(round="v3", layout="scout_layout v13 (PR 169, on main)", catch=dc, funnel_line=np.round(fl, 1).tolist(), files=files,
               note="drone frame (x nose, y left, z up, thrust centre x 0), catch 2.5 aft (v13); boxes [[x0, y0, z0], [x1, y1, z1]]; masses E unless tagged",
               items={}, mount={}, keep_clear=dict(plugs=np.round(kp, 1).tolist(), tool_lines={k: dict(start=v["start"], dir=v["dir"], d=TOOL) for k, v in A["sw"].items()},
                                                   keys={nm: dict(start=p, dir=d, d=4) for nm, p, d, cl in A["keys"]}))
    for n, (o_, t_) in P.items():
        env["items"][n] = dict(box=np.round(np.array([np.min([b[0] for b in o_["body"]], 0), np.max([b[1] for b in o_["body"]], 0)]) + t_, 1).tolist(),
                               mass_g=MASS[n][0], tag=MASS[n][1], cg=np.round(o_["cg"] + t_, 1).tolist(), funnel_margin=round(A["ck"][n]["funnel"], 1),
                               rods=round(A["ck"][n]["rods"], 1), inset=round(A["ck"][n]["inset"], 1), wash=round(A["ck"][n]["wash"], 1))
    for k, (m, g, c, note) in mp.items():
        fm_ = A["ck"][k]["funnel"]
        env["mount"][k] = dict(box=np.round(m.bounds, 1).tolist(), mass_g=g, tag="E", cg=np.round(c, 1).tolist(), note=note, wash=round(A["ck"][k]["wash"], 1),
                               funnel_margin=None if not np.isfinite(fm_) else round(fm_, 1), rods=round(A["ck"][k]["rods"], 1), inset=round(A["ck"][k]["inset"], 1))
    cb = [(n, g, c) for n, g, c, _ in res["cab"]]
    env["cables"] = dict(mass_g=round(sum(g for _, g, _ in cb), 2), tag="E", cg=np.round(sum(g * c for _, g, c in cb) / sum(g for _, g, _ in cb), 1).tolist(),
                         runs={n: dict(g=round(g, 2), mid=np.round(c, 1).tolist(), L=round(L, 0)) for n, g, c, L in res["cab"]})
    gm = sum(v["mass_g"] for v in env["mount"].values())
    env["mount_total"] = dict(mass_g=round(gm, 1), cg=np.round(sum(v["mass_g"] * np.array(v["cg"]) for v in env["mount"].values()) / gm, 1).tolist())
    env["cg"] = dict(pack_in=np.round(A["cg"]["cin"], 2).tolist(), pack_out=np.round(A["cg"]["cout"], 2).tolist(), h_in=round(A["cg"]["hin"], 2),
                     h_out=round(A["cg"]["hout"], 2), mass=round(A["cg"]["mass"], 1), best_catch_bay_riding=dict(catch=A["bs"][0], h_in=round(A["bs"][1], 2), h_out=round(A["bs"][2], 2)))
    env["ubec_region"] = ur
    env["requirements"] = dict(belly_mount="its tail face at x -31.0 over |y| <= 22, z 9.6..18.5 (plate-fixed): the tail block's web and the 2 plate-bolt bosses "
                                            "sit there under the plate; the block's gusset bears on the receiver's tail face",
                               plate="2 x Ø3.4 at (-42.0, +-16.0) in the tail tabs (8.0 from the tail edge, 7.0 from the notch edge), unchanged from round 5")
    json.dump(env, open(os.path.join(RUN, "envelope.json"), "w"), indent=1)
    res_out = dict(round="v3", catch=dc, poses=pose_json(res), cables=env["cables"], concepts={
        k: dict(mount={kk: dict(g=g, cg=np.round(c, 2).tolist(), bounds=np.round(m.bounds, 2).tolist(), note=note, bodies=v["bodies"][kk]) for kk, (m, g, c, note) in v["mp"].items()},
                mount_g=round(v["g"], 1), checks=v["ck"], fit=v["fp"], sweeps=v["sw"], loads={kk: vv for kk, vv in v["ld"].items() if kk != "secs"}, modes=v["md"],
                cg=dict(pack_in=np.round(v["cg"]["cin"], 3).tolist(), pack_out=np.round(v["cg"]["cout"], 3).tolist(),
                h_in=round(v["cg"]["hin"], 3), h_out=round(v["cg"]["hout"], 3), mass=round(v["cg"]["mass"], 1)), slide_riding=v["sc"], best_riding=v["bs"],
                keys=v["keys"], system_check=v["sc_line"][:900], by_kind=v["ld"]["by_kind"]) for k, v in out.items()},
        funnel=dict(line=np.round(fl, 2).tolist(), margins=fm), wash=wsh, ubec_region=ur, variants=vrows, picked=pick["name"],
        pose_findings=dict(ubec_radxa_gap=round(float(gapRU), 2), plug_keepclear=np.round(kp, 1).tolist()), usb1_tilts=tilts,
        usb1_dir=np.round(es.USB1_DIR, 3).tolist(), bbox=dict(extents=np.round(ext, 0).tolist(), L=round(float(np.prod(ext) / 1e6), 1)))
    json.dump(res_out, open(os.path.join(RUN, "result.json"), "w"), indent=1, default=lambda x: x.tolist() if hasattr(x, "tolist") else str(x))
    say(f"done in {time.time() - t0:.0f} s")
    open(os.path.join(RUN, "run.log"), "w").write(es.LOG.getvalue())


if __name__ == "__main__":
    run_v13()
