"""Box structure v2 layout (24 Oct 2026, box structure system session; docs/generative.md The rule 0, concept step).

Spark 24 Oct: every cartridge slot is a bay, 4 Overseer bays at the bottom (-60, -90, -120, -150) and 5 Scout bays
(30, 0, -30, -180, -210), drone at 90. Swap arm v6 (PR 71): every seated face at R 113, Overseer modules 172 long at
Y_C 175, arm lane >= 159 in the drone opening. Battery latch v1 (PR 69): one receiver per bay, the bay body bolts to its
mount plane. This script places the receivers, cartridges and their install paths, the arm's turning envelopes, the
funnel panels v7 (catch) and their 8 seat landings at the new seat height, sizes the enclosure and the floor round them,
and writes the joined design envelope + keep-outs + drawn interfaces for the coarse system topology run
(runs/box_system/v2/layout/). Nothing here is a part: the pieces are cut from the topology result.

python3 box_layout_v2.py [--R 113] [--clip]   ->  runs/box_system/v2/layout/ (layout.json, *.stl)
--clip: receivers clipped to their bay's sector planes less 0.3 (the requirement sent to the latch, PR 69 comment).
W frame (box-arm.md). C geometry, M meshes (latch v1 receivers from runs/box_system/v2/latch_ctx/), E where tagged."""
import os, sys, json, math, argparse
import numpy as np, trimesh, manifold3d as mf

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
for p in (os.path.join(HERE, "stand"), HERE, os.path.join(ROOT, "nox"), os.path.join(HERE, "catch_v4")):
    if p not in sys.path: sys.path.insert(0, p)
import box_system as B

CTX = os.path.join(ROOT, "runs", "box_system", "v2", "latch_ctx")
BAYS = {30: "scout", 0: "overseer", -30: "scout", -60: "overseer", -90: "scout", -120: "overseer", -150: "scout",
        -180: "overseer", -210: "scout"}     # hub DEFAULT 24 Oct (arm v7): no two Overseer bays adjacent; set A (mirror-symmetric, CG on X 0)
BAYS_BOTTOM = {30: "scout", 0: "scout", -30: "scout", -60: "overseer", -90: "overseer", -120: "overseer", -150: "overseer",
               -180: "scout", -210: "scout"}  # the catch preview's placement (alternative, needs clipped receivers or R 128)
HALF = 15.0; GAP = 0.3                          # sector half angle; each body / receiver stays 0.3 inside its plane
Y_C = 165.0                                     # common grip line of both cartridges (arm PR 79 DEFAULT, hub 24 Oct; was 175, then 138)
Y_FUN = Y_C                                     # funnel centre = the grip line = each drone's catch centre = its latch set's centre (hub)


def grip_z(pk, meta):
    """the grip line in cartridge z: the latch set's centre (hub 24 Oct: each drone's catch centre on its set's centre,
    the arm grips there), taken as the receiver's z mid (C from the latch v1 meshes; E until the latch republishes)."""
    t = trimesh.load(os.path.join(CTX, f"{pk}_receiver_c.stl"), force="mesh"); return float(t.bounds[:, 2].mean())
PATH = 70.0                                     # cartridge install path kept clear, radially inward from the seat (E: park r ~ R - 15)
ARM_PARK_R = 1.5                                # turning envelope of the parked heads + cartridges: r <= R - 1.5 (arm v6: limit R - 2.5)
LANE = dict(r=159.0, half_deg=25.0, y=(62.0, 86.5))   # drone opening: nothing inside r 159 within 90 +- 25 over the carriage's Y (arm v6:
                                                # wheel rim r 156.9 on wheels Y 63..73, carriage +-20.3 deg, hardware to Y 85.5 (+1); rails below, r 151 (disc), C)
MOTOR_SWEEP = dict(r=106.5, y=(2.0, 50.0))      # motors on the arm (v1 box_system)
HUB = dict(r=88.0, y=(7.0, 74.0))               # hub + rails on the plate (arm v6 hub envelope r 110 clipped to +-88)
GRIP_LANE = dict(r=159.6, half_deg=13.0, y0=77.0, dy1=52.0)  # the head's grabber beside the cartridge at the drone insert: X +-33.5,
                                                # u to 61 (cartridge top) -> r 158.6 at R 113 (+1), Y 78..Y_C + 51 (+1) (arm v6 / v7 head envelope, C)
RAIL_DISC = dict(r=151.0, y=(38.0, 73.0))       # arm round 3 (24 Oct): the rails' keel sweeps Y 38..73 out to r 151 (C)
BRACKET = 12.0                                  # ring envelope to sheet inner face (U-clip + flange, v1 brackets 3.5..10)
CRADLE = 48.1                                   # bottom bay's outer face to the floor (v1 cradle_keel: Z_TOP -152.9 to -201)
FUNNEL_V8_GROW_Y = 11.5                         # funnel v8 plain (catch PR 81): opening 191 along drone x (= W Y) x 168: the +-Y faces move out
                                                # (191 - 168) / 2 = 11.5 (C); the sets are centred on the grip line now (hub), receiver +-89.8


def man(t): return B.man(t)
def tm(m): return B.tmesh(m)


def place(m, zT, ang, R):
    """cartridge coords -> W at bay angle ang (90 = drone): X = x, Y = Y_C - (z - z_T), Z = R + y, then turned."""
    return m.transform([[1, 0, 0, 0], [0, 0, -1, Y_C + zT], [0, 1, 0, R]]).rotate([0, 90 - ang, 0])


def wedge(ang, r0, r1, y0, y1, gap=GAP, half=HALF):
    """sector solid between the bay's two planes (each moved 'gap' inward), r0..r1, Y y0..y1."""
    pts = [(0.0, 0.0)] + [(1.2 * r1 * math.cos(math.radians(ang + a)), 1.2 * r1 * math.sin(math.radians(ang + a)))
                          for a in np.linspace(-half - 1, half + 1, 25)]
    w = mf.Manifold.extrude(mf.CrossSection([pts]), y1 - y0).rotate([90, 0, 0]).translate([0, y1, 0])   # XY -> XZ, along Y
    ring = mf.Manifold.cylinder(y1 - y0, r1, r1, 128).rotate([-90, 0, 0]).translate([0, y0, 0]) - \
        mf.Manifold.cylinder(y1 - y0 + 2, r0, r0, 128).rotate([-90, 0, 0]).translate([0, y0 - 1, 0])
    return _sector_planes(ring ^ w, ang, gap, half)


def _sector_planes(m, ang, gap, half):
    for s in (1, -1):
        t = math.radians(ang + s * half)
        n = np.array([-s * math.sin(t), 0.0, s * math.cos(t)])                          # points into the sector
        c = np.array([math.cos(math.radians(ang)), 0.0, math.sin(math.radians(ang))])
        if np.dot(n, c) < 0: n = -n
        m = m.trim_by_plane([float(n[0]), 0.0, float(n[2])], gap)
    return m


def load_ctx(pk):
    return {k: man(trimesh.load(os.path.join(CTX, f"{pk}_{k}_c.stl"), force="mesh")) for k in ("receiver", "receiver_lid", "cartridge", "xt60e1m")}


def seat_landings(R):
    """the 8 funnel seat landings (panel seat blocks, funnel_panel.py v6/v7: SEAT_Y 36 along the edge, SEAT_V 14 up the
    slope, landing disc o 16 at CLAMP 6 behind the contact face, post along +n, rib-free r 9.5) in W at the R seat."""
    import funnel_panel as F
    land = F.P_FACE + F.CLAMP * F.N_IN; n = F.N_IN
    pts = []
    for k in range(4):
        c, s = math.cos(k * math.pi / 2), math.sin(k * math.pi / 2); Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
        for sy in (1, -1):
            p = land * np.array([1, sy, 1]); pts.append((Rz @ p, Rz @ n))
    T = B.SEAT_TO_W.copy(); T[2, 3] += R - 100.0; T[1, 3] = Y_FUN
    out = []
    for p, nn in pts:
        pw = T[:3, :3] @ p + T[:3, 3]; nw = T[:3, :3] @ nn; out.append((pw, nw))
    return out


def layout(R=113.0, clip=False, bays=None):
    global BAYS
    if bays is not None: BAYS = bays
    P, rows = {}, {}
    meta = json.load(open(os.path.join(CTX, "meta.json")))
    for ang, pk in BAYS.items():
        c = load_ctx(pk); zT = grip_z(pk, meta)
        rec = place(c["receiver"] + c["receiver_lid"], zT, ang, R)
        if clip: rec = _sector_planes(rec, ang, GAP, HALF)
        cart = place(c["cartridge"], zT, ang, R)
        d = np.array([math.cos(math.radians(ang)), 0, math.sin(math.radians(ang))])
        path = mf.Manifold.batch_hull([cart, cart.translate(list(-PATH * d))])
        P[ang] = dict(pack=pk, rec=rec, cart=cart, path=path, xt60=place(c["xt60e1m"], zT, ang, R))
        t = tm(rec); r = np.hypot(t.vertices[:, 0], t.vertices[:, 2])
        rows[ang] = dict(pack=pk, rec_r=[round(float(r.min()), 1), round(float(r.max()), 1)], rec_Y=np.round(t.bounds[:, 1], 1).tolist(),
                         mount_r=round(R + meta[pk]["Y_M"], 1))
    # mount pads (the bay body's face on each receiver's mount plane, cartridge coords x +-(BX + 5), y Y_M..Y_M + 3,
    # z -4.3..L_C + 4.3; BX = RF + 5.3, latch table) and the drone's own receiver + path at 90 deg (both packs)
    for ang, q in P.items():
        g = meta[q["pack"]]; bx = g["RF"] + 5.3 + 5.0
        pad = mf.Manifold.cube([2 * bx, 3.0, g["L_C"] + 8.6]).translate([-bx, g["Y_M"], -4.3])
        q["pad"] = place(pad, grip_z(q["pack"], meta), ang, R)
        if clip: q["pad"] = _sector_planes(q["pad"], ang, GAP, HALF)
    drone = {}
    for pk in ("scout", "overseer"):
        c = load_ctx(pk); zT = grip_z(pk, meta); cart = place(c["cartridge"], zT, 90, R)
        drone[pk] = place(c["receiver"] + c["receiver_lid"], zT, 90, R) + mf.Manifold.batch_hull([cart, cart.translate([0, 0, -PATH])])
    # neighbours
    angs = sorted(BAYS); nb = {}
    for a, b in zip(angs[:-1], angs[1:]):
        nb[f"{a} | {b}"] = dict(receivers_mm3=round((P[a]["rec"] ^ P[b]["rec"]).volume(), 1),
                                cart_in_neighbour_mm3=round((P[a]["path"] ^ P[b]["rec"]).volume() + (P[b]["path"] ^ P[a]["rec"]).volume(), 1))
    # catch at the new seat height
    import box_system as BS
    T = BS.SEAT_TO_W.copy(); T[2, 3] += R - 100.0; T[1, 3] = Y_FUN
    import assembly
    A = assembly.load(os.path.join(HERE, "box_catch.assembly.json"))
    ms = [(p, t) for p, t in zip(A.parts, A.meshes()) if not p["name"].startswith("drone")]
    in_w = min(t.bounds[0][2] for _, t in ms) > 100.0                     # one test per file: seat frame (v7 file) or W (funnel v8 on, at R 113)
    if in_w:                                                              # a W file: shift it to this layout's R and funnel centre (its own centre from the panels)
        pb = np.array([t.bounds for p, t in ms if "panel" in p["name"]]); yc = (pb[:, 0, 1].min() + pb[:, 1, 1].max()) / 2
        T = np.eye(4); T[1, 3] = Y_FUN - yc; T[2, 3] = R - 113.0
    catch = {}
    for p, t in ms:
        t = t.copy(); t.apply_transform(T); catch[p["name"]] = man(t)
    cb = np.array([tm(m).bounds for m in catch.values()])
    rim = float(cb[:, 1, 2].max()); front = float(cb[:, 1, 1].max()); back = float(cb[:, 0, 1].min()); side = float(np.abs(cb[:, :, 0]).max())
    lands = seat_landings(R)
    # bay body envelopes (C, DEFAULT): the sector from the receiver's inner corner to 12 over its mount plane, the
    # receiver's Y range + 4 end walls; the end bays (30, -210) also reach the funnel seat landings over their free end
    env = {}
    for ang, q in P.items():
        t = tm(q["rec"]); mr = rows[ang]["mount_r"]
        env[ang] = wedge(ang, R - ARM_PARK_R + 0.5, mr + 12.0, t.bounds[0, 1] - 4.0, t.bounds[1, 1] + 4.0)
    # service: a bay body leaves along +Y with the front sheet off (the door, DEFAULT), so it keeps out of every
    # neighbour's receiver swept toward -Y (a wider Overseer receiver reaches into its Scout neighbour's sector near r R)
    angs_ = sorted(P)
    for i_, ang in enumerate(angs_):
        for nb_ in (angs_[i_ - 1] if i_ > 0 else None, angs_[i_ + 1] if i_ + 1 < len(angs_) else None):
            if nb_ is None: continue
            r_ = P[nb_]["rec"]; env[ang] = env[ang] - mf.Manifold.batch_hull([r_, r_.translate([0, -400.0, 0])])
    disc = mf.Manifold.cylinder(RAIL_DISC["y"][1] + 1.0 - (RAIL_DISC["y"][0] - 1.0), RAIL_DISC["r"] + 1.0, RAIL_DISC["r"] + 1.0, 128).rotate([-90, 0, 0]).translate([0, RAIL_DISC["y"][0] - 1.0, 0])
    for ang in env: env[ang] = env[ang] - disc                                  # 1.0 off the arm's rail disc (arm round 3)
    ring = None
    for m in env.values(): ring = m if ring is None else ring + m
    rt = tm(ring)
    ring_x = float(np.abs(rt.vertices[:, 0]).max()); ring_zmin = float(rt.vertices[:, 2].min())
    X_IN = math.ceil(max(ring_x, side) + BRACKET)
    FLOOR = math.floor(ring_zmin - CRADLE)
    Z_TOP = math.ceil(rim + 0.5)
    Y_FRONT = math.ceil(max(front + FUNNEL_V8_GROW_Y + 1.0, float(rt.bounds[1, 1]) + BRACKET))
    Y_BACK = -66.0
    assert back - FUNNEL_V8_GROW_Y > Y_BACK + 1.0, "funnel v8 reaches the rear sheet"
    H = Z_TOP - FLOOR
    out = dict(R=R, clip=clip, bays=rows, neighbours=nb, catch=dict(rim_Z=round(rim, 1), front_Y=round(front, 1), back_Y=round(back, 1),
                                                                     side_X=round(side, 1), seat_W_Z=round(T[2, 3], 1)),
               landings=[dict(p=np.round(p, 1).tolist(), n=np.round(n, 3).tolist()) for p, n in lands],
               ring=dict(max_abs_X=round(ring_x, 1), min_Z=round(ring_zmin, 1), Y=np.round(rt.bounds[:, 1], 1).tolist()),
               enclosure=dict(X_IN=X_IN, Y_BACK=Y_BACK, Y_FRONT=Y_FRONT, FLOOR=FLOOR, Z_TOP=Z_TOP,
                              sheets_mm={"side x2": [round(Y_FRONT - Y_BACK - 6), H], "front / rear x2": [2 * X_IN - 6, H]},
                              post_segments=dict(n=3, length=round(H / 3, 2)), outer=[2 * X_IN + 6, Y_FRONT - Y_BACK + 6, H],
                              volume_L=round((2 * X_IN + 6) * (Y_FRONT - Y_BACK + 6) * H * 1e-6, 1)))
    out["drone_keepout"] = "drone receivers (Scout, Overseer) + cartridge paths at 90 deg: drone_W.stl"
    return out, P, catch, env, lands, drone


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--R", type=float, default=113.0); ap.add_argument("--clip", action="store_true")
    ap.add_argument("--bottom", action="store_true", help="Overseer bays at the 4 bottom slots (the catch preview), not alternating")
    a = ap.parse_args()
    tag = ("" if a.R == 113 else f"_R{a.R:g}") + ("_clip" if a.clip else "") + ("_bottom" if a.bottom else "")
    od = os.path.join(ROOT, "runs", "box_system", "v2", "layout" + tag)
    os.makedirs(od, exist_ok=True)
    out, P, catch, env, lands, drone = layout(a.R, a.clip, BAYS_BOTTOM if a.bottom else None)
    json.dump(out, open(os.path.join(od, "layout.json"), "w"), indent=1, default=float)
    print(json.dumps({k: out[k] for k in ("neighbours", "catch", "ring", "enclosure")}, indent=1, default=float))
    rec = None; path = None; envm = None
    for q in P.values():
        rec = q["rec"] if rec is None else rec + q["rec"]; path = q["path"] if path is None else path + q["path"]
    for m in env.values(): envm = m if envm is None else envm + m
    pads = None
    for q in P.values(): pads = q["pad"] if pads is None else pads + q["pad"]
    for n, m in (("receivers", rec), ("cartridge_paths", path), ("bay_envelopes", envm), ("mount_pads", pads),
                 ("drone", drone["scout"] + drone["overseer"])):
        tm(m).export(os.path.join(od, f"{n}_W.stl"))
    c = None
    for m in catch.values(): c = m if c is None else c + m
    tm(c).export(os.path.join(od, "catch_W.stl"))
    print(write_concept(od, out, P, env, a.R))


def arm_envelopes(R):
    """the swap arm's turning envelopes (arm v6 numbers at the top of this file, C): hub + rails, motors on the arm,
    parked heads with cartridges, the drone-insert lane."""
    cyl = lambda r, y0, y1: mf.Manifold.cylinder(y1 - y0, r, r, 96).rotate([-90, 0, 0]).translate([0, y0, 0])
    lane = wedge(90, 1.0, LANE["r"], LANE["y"][0], LANE["y"][1], gap=0.0, half=LANE["half_deg"]) + \
        wedge(90, 1.0, GRIP_LANE["r"], GRIP_LANE["y0"], Y_C + GRIP_LANE["dy1"], gap=0.0, half=GRIP_LANE["half_deg"])
    return {"arm: hub + rails (turning)": cyl(HUB["r"], *HUB["y"]) + cyl(RAIL_DISC["r"], *RAIL_DISC["y"]),
            "arm: motors (turning)": cyl(MOTOR_SWEEP["r"], *MOTOR_SWEEP["y"]) - cyl(HUB["r"] - 0.01, MOTOR_SWEEP["y"][0] - 1, MOTOR_SWEEP["y"][1] + 1),
            "arm: parked heads + cartridges (turning)": cyl(R - ARM_PARK_R, HUB["y"][1], Y_C + 122.0),
            "arm: drone insert lane": lane}


def write_concept(od, out, P, env, R):
    """system_check manifest of the v2 layout (concept check) + its motion scene: per bay its receiver (latch,
    protected), cartridge (swap part) and body envelope; the catch, the drone keep-out, the arm's turning envelopes,
    the v1 mast and the PSU (stand and electronics context), the 4 sheets (cage); access paths: every bay's cartridge
    out along its install axis to park (the head's move) and every bay body out along +Y (service)."""
    import assembly
    e = out["enclosure"]; pd = os.path.join(od, "parts"); os.makedirs(pd, exist_ok=True)
    rel = lambda n: os.path.relpath(os.path.join(od, n), ROOT)
    parts, access, contact = [], [], []
    for ang, q in P.items():
        for k, m, role in (("receiver", q["rec"], "protected"), ("cartridge", q["cart"], "bought"), ("body envelope", env[ang], "structure")):
            fn = f"parts/bay{ang}_{k.replace(' ', '_')}.stl"; tm(m).export(os.path.join(od, fn))
            p = dict(name=f"bay {ang} {k}", role=role, mesh=rel(fn), material="PETG", tag="C" if k == "body envelope" else "M", owner="box structure" if k == "body envelope" else "box swap: latch")
            if role == "protected": p["protect"] = True
            if k == "cartridge": p.update(mass_g=round(json.load(open(os.path.join(CTX, "meta.json")))[q["pack"]]["SWAP_M"], 1), swap=True)
            parts.append(p)
        contact += [[f"bay {ang} receiver", f"bay {ang} body envelope", 1e9], [f"bay {ang} cartridge", f"bay {ang} body envelope", 1e9],
                    [f"bay {ang} cartridge", f"bay {ang} receiver", 1e9]]
        d = [-math.cos(math.radians(ang)), 0.0, -math.sin(math.radians(ang))]
        access.append(dict(name=f"bay {ang}: cartridge out to park ({PATH:g} along -r)", part=f"bay {ang} cartridge", dir=d, length_mm=PATH, tool="part",
                           ignore=[f"bay {ang} receiver", f"bay {ang} body envelope", "arm: parked heads + cartridges (turning)"]))
        access.append(dict(name=f"bay {ang}: body out along +Y, front sheet off (service)", part=f"bay {ang} body envelope", dir=[0, 1, 0], length_mm=270.0, tool="part",
                           ignore=[f"bay {ang} receiver", f"bay {ang} cartridge", "sheetF"]))
    for n, m in arm_envelopes(R).items():
        fn = f"parts/{n.split(':')[1].strip().split(' ')[0]}_{len(parts)}.stl"; tm(m).export(os.path.join(od, fn))
        parts.append(dict(name=n, role="bought", mesh=rel(fn), mass_g=0.0, tag="C", owner="box swap: arm", note="turning envelope (keep-out)"))
    parts += [dict(name="catch (panels v7, hook latch v2) at the R seat", role="structure", mesh=rel("catch_W.stl"), material="PETG", tag="M", owner="box catch"),
              dict(name="drone receivers + paths (keep-out)", role="bought", mesh=rel("drone_W.stl"), mass_g=0.0, tag="C", owner="drones"),
              dict(name="mast v6 (stand context, v1)", role="structure", mesh="runs/box_system/v1/W/box_mast_v6_W.stl", material="PETG", tag="M", owner="box structure"),
              dict(name="PSU LRS-350-24", role="bought", protect=True, box=[42, 157, -66, -36, -170, 45], mass_g=750.0, tag="E", owner="box electronics")]
    contact += [["arm: hub + rails (turning)", "arm: motors (turning)", 1e9], ["arm: hub + rails (turning)", "arm: parked heads + cartridges (turning)", 1e9],
                ["arm: parked heads + cartridges (turning)", "arm: drone insert lane", 1e9], ["arm: hub + rails (turning)", "arm: drone insert lane", 1e9],
                ["drone receivers + paths (keep-out)", "arm: drone insert lane", 1e9], ["drone receivers + paths (keep-out)", "arm: parked heads + cartridges (turning)", 1e9],
                ["arm: motors (turning)", "arm: drone insert lane", 1e9]]
    X, YB, YF, FL, ZT = e["X_IN"], e["Y_BACK"], e["Y_FRONT"], e["FLOOR"], e["Z_TOP"]
    for n, b in {"sheet+X": [X, X + 3, YB + 3, YF - 3, FL, ZT], "sheet-X": [-X - 3, -X, YB + 3, YF - 3, FL, ZT],
                 "sheetF": [-X + 3, X - 3, YF, YF + 3, FL, ZT], "sheetR": [-X + 3, X - 3, YB - 3, YB, FL, ZT]}.items():
        parts.append(dict(name=n, role="cage", box=b, material="acrylic", mass_g=round((b[1] - b[0]) * (b[3] - b[2]) * (b[5] - b[4]) * 1.19e-3, 1), tag="C", owner="box structure"))
    A = assembly.new("Box structure v2 layout", "box", parts=parts, probes=False, fit_tol_mm3=0.5, contact_pairs=contact, access=access,
                     note="v2 layout concept (box_layout_v2.py). Bay body envelopes are design spaces (their System-line mass is envelope volume x PETG, not a design mass); each receiver and cartridge sits in its own envelope by design (contact pairs); the arm's turning envelopes overlap each other by construction. Probes off (floor-standing box, DEFAULT).")
    A.save(os.path.join(od, "concept.assembly.json"))
    res = A.check(); json.dump(dict(summary=res.get("summary"), access=res.get("access"), fit=res.get("fit")), open(os.path.join(od, "concept_check.json"), "w"), indent=1, default=str)
    A.scene().write(os.path.join(od, "layout.html"))
    return res.get("summary")

if __name__ == "__main__":
    main()
