"""Box-side operation sweep (17 Oct 2026, sweep chat): arm_kinematics' swap cycle with every part as built today.
Adds to arm_kinematics.build_parts():
  - hub v5 as printed (Nox Print Files box_hub_v5, brought back to W; drawn features missing 0.01 mm3, M)
  - heads A / B at their v5 generation ENVELOPE (env - cuts, the most material the v5 run could place: an upper bound;
    the v5 meshes live only in the head chats), or --heads drawn for the lower bound
  - bay module v4i (box_baymod_v4i_W.obj, M) at the bays of the chosen config
  - harness keep-outs (box-wiring.md Problem 4) as arm bodies: saddle Ø10 on axis Y 78..86, drop lane Ø8 at 135 deg
    (arm frame, A = 90 deg) r 32 Y 50..86, hub lacing Ø6 at r 46 Y 8..50 taken as the full annulus r 43..49 (conservative)
  - stand v4 wood (stand_v4.py pieces, C / E) and the funnel as a 4.4 thick 45 deg frustum (open bottom 168 at Z 143.6,
    mouth 320, centre Y 177; E from stand_v4's funnel check) as fixed bodies
Setup: fetch nox/box/*.py + models/ from Nox Source, box_hub_v5.stl.zip.b64.txt from Nox Print Files (base64 -d, unzip).\nRecords the minimum distance of every checked pair over the whole cycle, not only the per-state minimum.
python3 sweep_box.py <dt> <config: built|ring9> <heads: env|drawn|v5> -> out/sweep_<config>_<heads>.json"""
import sys, os, json, math, time
import numpy as np, trimesh

CONFIG = sys.argv[2] if len(sys.argv) > 2 else "built"
HEADS = sys.argv[3] if len(sys.argv) > 3 else "env"
DT = float(sys.argv[1]) if len(sys.argv) > 1 else 0.2

import arm_layout as L
if CONFIG == "built":
    L.SCOUT_BAYS = (0.0, -30.0); L.BAY_ANGLES = list(L.SCOUT_BAYS)     # box-case.md: 2 bays printed, stand v4 holds 0 and -30
import arm_cad as A
import arm_kinematics as K
from arm_cad import box, cylY, tm, man

OUT = "out"


def head_mesh(h):
    if HEADS == "v5":                                   # the head chat's run output (head frame, glide inner end at u 0), Nox Source models/
        return K.load_dec(f"models/box_head_{h}_v5.stl", 40000)
    H = A.head_parts(h)
    m = (H["env"] - H["cuts"]) if HEADS == "env" else (H["drawn"] - H["cuts"])
    return tm(m)


def hub_from_print(src, dst="out/box_hub_v5.stl"):
    """Nox Print Files box_hub_v5 (print pose, sleeve end down) back to W: W = (Px, Pz, -Py) + (-84, 7, 84).
    Checked 17 Oct: drawn features missing 0.01 mm3, material in cuts 0.004 mm3 (M)."""
    t = trimesh.load(src, force="mesh"); t.merge_vertices(); v = t.vertices
    w = trimesh.Trimesh(np.c_[v[:, 0], v[:, 2], -v[:, 1]] + [-84.0, 7.0, 84.0], t.faces, process=False)
    w.export(dst); return w


def harness():
    ang = math.radians(135.0)
    saddle = cylY(0, 0, 78.0, 86.0, 5.0)
    drop = cylY(32 * math.cos(ang), 32 * math.sin(ang), 50.0, 86.0, 4.0)
    lacing = cylY(0, 0, 8.0, 50.0, 49.0, 128) - cylY(0, 0, 7.0, 51.0, 43.0, 128)
    return saddle + drop + lacing


def prism(poly_xz, y0, y1):
    p = np.asarray(poly_xz, float)
    v = np.vstack([np.c_[p[:, 0], np.full(len(p), y0), p[:, 1]], np.c_[p[:, 0], np.full(len(p), y1), p[:, 1]]])
    return trimesh.convex.convex_hull(v)


def stand_parts():
    c45 = math.sqrt(.5); T = 18.0; BT = -152.0
    out = {}
    out["upright"] = prism([(-250, BT), (250, BT), (250, 250), (-250, 250)], -T, 0)
    out["base"] = prism([(-250, BT - T), (250, BT - T), (250, BT), (-250, BT)], -T, 300 - T)
    s = np.array([98.14, 151.46]); u = np.array([c45, c45]); n = np.array([c45, -c45])
    for sgn, nm in ((1, "fblock_R"), (-1, "fblock_L")):
        uu, nn, ss = u * [sgn, 1], n * [sgn, 1], s * [sgn, 1]
        out[nm] = prism([ss - 10 * uu, ss + 79 * uu, ss + 79 * uu + 38 * nn, ss - 10 * uu + 38 * nn], 0, 230)
    rot = lambda th: np.array([math.cos(th), math.sin(th)])
    d, m = rot(math.radians(15)), rot(math.radians(105))
    out["bayblock"] = prism([d * 155 + m * .3, d * 244 + m * .3, d * 244 + m * 38.3, d * 155 + m * 38.3], 0, 230)
    mm = rot(math.radians(-135)); a = rot(math.radians(-45)) * 158 + mm * .3; b = rot(math.radians(-45)) * 200 + mm * .3
    out["wedge"] = prism([a, b, (b[0], BT), (a[0], BT)], 135, 220)
    return out


def funnel():
    """4 flat 45 deg panels: inner faces through (|X| or |Y - 177|) = 84 at Z 143.6, mouth at Z 219.6; 4.4 thick outward."""
    z0, z1, yc = 143.6, 219.6, 177.0
    def frustum(off):
        pts = []
        for z in (z0 - (off if off else 0) * 0, z1):
            w = 84 + (z - z0) + off * math.sqrt(2)
            pts += [(sx * w, yc + sy * w, z) for sx in (-1, 1) for sy in (-1, 1)]
        return man(trimesh.convex.convex_hull(np.array(pts)))
    shell = frustum(4.4) - frustum(0.0).translate([0, 0, 0]) - box(-300, 300, -300, 600, z1, z1 + 50) - box(-300, 300, -300, 600, z0 - 50, z0)
    return tm(shell)


def build(mode="all"):
    P = K.build_parts(mode)
    for h in ("A", "B"):
        P[f"head{h}"] = K.Part(f"head {h} v5 {HEADS}", head_mesh(h), f"head{h}", "#d35400")
    bm = man(K.load_dec("models/box_baymod_v4i_W.obj", 40000))
    for k_ in [k for k in P if k.startswith(("bay", "strip"))]: del P[k_]
    for k_, a in enumerate(L.SCOUT_BAYS):
        P[f"bay{k_}"] = K.Part(f"bay module v4i {k_} ({a:.0f} deg)", tm(bm.rotate([0, 90.0 - a, 0])), "fixed", "#5b8ac9")
        P[f"strip{k_}"] = K.Part(f"TPU strip {k_}", tm(A.bay_strip().rotate([0, 90.0 - a, 0])), "fixed", "#2e4053")
    P["harness"] = K.Part("harness keep-outs", tm(harness()), "arm", "#c0392b", printed=False)
    if CONFIG == "built":
        for nm, m in stand_parts().items():
            P[f"wood_{nm}"] = K.Part(f"stand {nm}", m, "fixed", "#a0522d", printed=False)
        P["funnel"] = K.Part("funnel v6 (frustum, E)", funnel(), "fixed", "#5d6d7e")
    K.PARTS_CACHE.clear(); K.PARTS_CACHE.update(P)
    return P


ARM_GROUPS = ("arm", "armB", "headA", "headB")


def main():
    import trimesh.collision as tc, fcl
    os.makedirs(OUT, exist_ok=True)
    if not os.path.exists(f"{OUT}/box_hub_v5.stl"): hub_from_print(sys.argv[4] if len(sys.argv) > 4 else "box_hub_v5.stl")
    if not os.path.exists(f"{OUT}/box_baymod_v4_W.stl"): trimesh.load("models/box_baymod_v4i_W.obj", force="mesh").export(f"{OUT}/box_baymod_v4_W.stl")
    t_b = time.time()
    P = build(); names = list(P)
    # harness: bound to the arm; only arm-vs-fixed counts
    for nm, p in P.items():
        if p.group in ARM_GROUPS and nm != "harness": K.ALLOWED.add(("harness", nm))
    # bay modules in this config: neighbours bolted (K built ALLOWED from the 9-bay list at import)
    for i, ai in enumerate(L.SCOUT_BAYS):
        K.ALLOWED.add((f"bay{i}", f"strip{i}"))
        for j, aj in enumerate(L.SCOUT_BAYS):
            if i != j and abs(((ai - aj) + 180) % 360 - 180) < L.PITCH + 1: K.ALLOWED.add((f"bay{i}", f"bay{j}"))
    # stand: bolted / screwed to each other, the bays, the stub and the funnel
    wood = [n for n in names if n.startswith("wood_")]
    for a in wood:
        for b in wood + ["stub", "funnel", "bay0", "bay1"]:
            if a != b: K.ALLOWED.add((a, b))
    K.ALLOWED |= {("stub", "wood_upright")}
    print(f"parts {len(names)} built in {time.time() - t_b:.0f} s; config {CONFIG}, heads {HEADS}, bays {L.SCOUT_BAYS}", flush=True)
    mgr = tc.CollisionManager()
    for nm in names: mgr.add_object(nm, P[nm].mesh, np.eye(4))
    cart_meshes = {sq: tm(K.cartridge_local(sq)) for sq in K.SQ_STEPS}
    for cn in ("pack charged", "pack dead"):
        for sq in K.SQ_STEPS: mgr.add_object(f"{cn}|{sq}", cart_meshes[sq], K.FAR)
    moving = [n for n in names if P[n].group != "fixed"]
    ts = K.sample_times(DT)
    rows, pairmin = [], {}
    t0 = time.time(); req = fcl.DistanceRequest()
    for t in ts:
        T, carts, s = K.transforms(t)
        T["harness"] = K.rotY(90.0 - s["phi"])
        for nm in names: mgr.set_transform(nm, T.get(nm, np.eye(4)))
        owners, active = {}, {}
        for cn, (Tc, sq, own) in carts.items():
            sqr = min([v for v in K.SQ_STEPS if v >= sq - 1e-9] or [K.SQ_STEPS[-1]])
            for v in K.SQ_STEPS: mgr.set_transform(f"{cn}|{v}", Tc if v == sqr else K.FAR)
            owners[f"{cn}|{sqr}"] = own; active[f"{cn}|{sqr}"] = (cart_meshes[sqr], Tc)
        own_plain = {k.split("|")[0]: v for k, v in owners.items()}
        _, pairs = mgr.in_collision_internal(return_names=True)
        pairs = {p for p in pairs if all((not n.startswith("pack")) or n in active for n in p)}
        base = lambda n: n.split("|")[0]
        bad = [p for p in pairs if not K.allowed(base(p[0]), base(p[1]), own_plain)]
        objs = {nm: (P[nm].mesh, T.get(nm, np.eye(4))) for nm in names}; objs.update(active)
        q = moving + list(active); allq = list(objs)
        def aabb(nm):
            m, Tr = objs[nm]; c = trimesh.transform_points(trimesh.bounds.corners(m.bounds), Tr); return c.min(0), c.max(0)
        boxes = {nm: aabb(nm) for nm in allq}
        dmin, dpair = 1e9, None
        for i, a in enumerate(q):
            for b in allq:
                if b == a or (b in q and q.index(b) < i): continue
                aa, bb = base(a), base(b)
                if K.allowed(aa, bb, own_plain): continue
                if P.get(aa) is not None and P.get(bb) is not None and P[aa].group == P[bb].group and P[aa].group in ARM_GROUPS[:2]:
                    continue                                                     # rigid on the same body: fixed gap, checked once below
                lo1, hi1 = boxes[a]; lo2, hi2 = boxes[b]
                if (np.maximum(lo1, lo2) - np.minimum(hi1, hi2) > 30.0).any(): continue
                res = fcl.DistanceResult()
                d = fcl.distance(mgr._objs[a]["obj"], mgr._objs[b]["obj"], req, res)
                key = tuple(sorted((aa, bb)))
                if d < pairmin.get(key, (9e9,))[0]: pairmin[key] = (round(float(d), 3), round(t, 3), round(s["phi"], 1))
                if d < dmin: dmin, dpair = float(d), (aa, bb)
        overlaps = []
        for a, b in bad:
            va = man(objs[a][0].copy().apply_transform(objs[a][1])); vb = man(objs[b][0].copy().apply_transform(objs[b][1]))
            v = (va ^ vb).volume()
            if v > 1e-3: overlaps.append((base(a), base(b), round(v, 2)))
        rows.append(dict(t=round(t, 3), phi=round(s["phi"], 1), zA=round(s["zA"], 1), zB=round(s["zB"], 1), fA=round(s["fA"], 2),
                         fB=round(s["fB"], 2), clearance=round(dmin, 3), pair=list(dpair) if dpair else None, overlaps=overlaps))
        print(f"t {t:6.2f} phi {s['phi']:7.1f} zA {s['zA']:5.1f} zB {s['zB']:5.1f} min {dmin:6.2f} {dpair} ov {overlaps}", flush=True)
    out = dict(config=CONFIG, heads=HEADS, dt=DT, bays=list(L.SCOUT_BAYS), states=len(rows), time_s=round(time.time() - t0),
               rows=rows, pairs=sorted([dict(a=k[0], b=k[1], gap=v[0], t=v[1], phi=v[2]) for k, v in pairmin.items()], key=lambda r: r["gap"]))
    json.dump(out, open(f"{OUT}/sweep_{CONFIG}_{HEADS}.json", "w"), indent=0)
    nov = sum(1 for r in rows if r["overlaps"])
    print(f"DONE {len(rows)} states in {out['time_s']} s; states with overlaps {nov}; min clearance {min(r['clearance'] for r in rows):.3f}", flush=True)
    for r in out["pairs"][:15]: print("  ", r)


if __name__ == "__main__":
    main()
