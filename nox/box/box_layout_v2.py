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
BAYS = {30: "scout", 0: "scout", -30: "scout", -60: "overseer", -90: "overseer", -120: "overseer", -150: "overseer",
        -180: "scout", -210: "scout"}
HALF = 15.0; GAP = 0.3                          # sector half angle; each body / receiver stays 0.3 inside its plane
Y_C, Y_FUN = 175.0, 177.0                       # tab line (arm v6, both cartridges); funnel centre (stand, unchanged)
PATH = 70.0                                     # cartridge install path kept clear, radially inward from the seat (E: park r ~ R - 15)
ARM_PARK_R = 1.5                                # turning envelope of the parked heads + cartridges: r <= R - 1.5 (arm v6: limit R - 2.5)
LANE = dict(r=159.0, half_deg=25.0)             # drone opening: nothing inside r 159 within 90 +- 25 (arm v6 wheel rim 156.9, carriage +-20.3)
MOTOR_SWEEP = dict(r=106.5, y=(2.0, 50.0))      # motors on the arm (v1 box_system)
HUB = dict(r=88.0, y=(7.0, 74.0))               # hub + rails on the plate (arm v6 hub envelope r 110 clipped to +-88)
BRACKET = 12.0                                  # ring envelope to sheet inner face (U-clip + flange, v1 brackets 3.5..10)
CRADLE = 48.1                                   # bottom bay's outer face to the floor (v1 cradle_keel: Z_TOP -152.9 to -201)


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
    T = B.SEAT_TO_W.copy(); T[2, 3] += R - 100.0
    out = []
    for p, nn in pts:
        pw = T[:3, :3] @ p + T[:3, 3]; nw = T[:3, :3] @ nn; out.append((pw, nw))
    return out


def layout(R=113.0, clip=False):
    P, rows = {}, {}
    meta = json.load(open(os.path.join(CTX, "meta.json")))
    for ang, pk in BAYS.items():
        c = load_ctx(pk); zT = meta[pk]["Z_T"]
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
        q["pad"] = place(pad, g["Z_T"], ang, R)
        if clip: q["pad"] = _sector_planes(q["pad"], ang, GAP, HALF)
    drone = {}
    for pk in ("scout", "overseer"):
        c = load_ctx(pk); zT = meta[pk]["Z_T"]; cart = place(c["cartridge"], zT, 90, R)
        drone[pk] = place(c["receiver"] + c["receiver_lid"], zT, 90, R) + mf.Manifold.batch_hull([cart, cart.translate([0, 0, -PATH])])
    # neighbours
    angs = sorted(BAYS); nb = {}
    for a, b in zip(angs[:-1], angs[1:]):
        nb[f"{a} | {b}"] = dict(receivers_mm3=round((P[a]["rec"] ^ P[b]["rec"]).volume(), 1),
                                cart_in_neighbour_mm3=round((P[a]["path"] ^ P[b]["rec"]).volume() + (P[b]["path"] ^ P[a]["rec"]).volume(), 1))
    # catch at the new seat height
    import box_system as BS
    T = BS.SEAT_TO_W.copy(); T[2, 3] += R - 100.0
    import assembly
    A = assembly.load(os.path.join(HERE, "box_catch.assembly.json"))
    catch = {}
    for p, t in zip(A.parts, A.meshes()):
        if p["name"].startswith("drone"): continue
        t = t.copy(); t.apply_transform(T); catch[p["name"]] = man(t)
    cb = np.array([tm(m).bounds for m in catch.values()])
    rim = float(cb[:, 1, 2].max()); front = float(cb[:, 1, 1].max()); back = float(cb[:, 0, 1].min()); side = float(np.abs(cb[:, :, 0]).max())
    lands = seat_landings(R)
    # bay body envelopes (C, DEFAULT): the sector from the receiver's inner corner to 12 over its mount plane, the
    # receiver's Y range + 4 end walls; the end bays (30, -210) also reach the funnel seat landings over their free end
    env = {}
    for ang, q in P.items():
        t = tm(q["rec"]); mr = rows[ang]["mount_r"]
        env[ang] = wedge(ang, R - ARM_PARK_R, mr + 12.0, t.bounds[0, 1] - 4.0, t.bounds[1, 1] + 4.0)
    ring = None
    for m in env.values(): ring = m if ring is None else ring + m
    rt = tm(ring)
    ring_x = float(np.abs(rt.vertices[:, 0]).max()); ring_zmin = float(rt.vertices[:, 2].min())
    X_IN = math.ceil(max(ring_x, side) + BRACKET)
    FLOOR = math.floor(ring_zmin - CRADLE)
    Z_TOP = math.ceil(rim + 0.5)
    Y_FRONT = math.ceil(max(front + 1.0, float(rt.bounds[1, 1]) + BRACKET))
    Y_BACK = -66.0
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
    a = ap.parse_args()
    od = os.path.join(ROOT, "runs", "box_system", "v2", "layout" + ("" if a.R == 113 and a.clip else f"_R{a.R:g}{'_clip' if a.clip else '_noclip'}"))
    os.makedirs(od, exist_ok=True)
    out, P, catch, env, lands, drone = layout(a.R, a.clip)
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


if __name__ == "__main__":
    main()
