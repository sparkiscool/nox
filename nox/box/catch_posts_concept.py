"""Inside-out catch (concept, 27 Oct, Spark: "lets reimagine the funnel, is there a way to make a funnel that works for
both and only depend on the bottom opening"). Concept check only (docs/generative.md The rule, 0); the box catch
session owns the catch and draws it if Spark keeps this.

The box carries 4 inclined posts (45 deg in the diagonal planes, r 6, PTFE-sleeved, E) rising into the drone's bottom
opening; each drone's cage ends in the same square rim (rim nodes at z 0 of the seat frame, rollers R 5 on its 4
corners at (+-RIM, +-RIM)). A landing off by up to c lands the rim on the posts' upper ends and slides it down to the
seat; nothing else of the drone may touch a post on the way. The drone's outside is free (no mouth to clear); the only
shared numbers are the rim and the post lines.

Check: for each drone (Scout layout D from nox/scout/v3/bottom_concept.py; the Overseer layout v5 meshes, shell left out
and its bottom opening redrawn as the same rim at its cartridge bottom), offsets c in 8 directions at yaw -5 / 0 / +5:
descend to the first contact, which must be the rim; slide straight to the seat (45 deg: horizontal and vertical go
together); every other point keeps 1.0 off the posts all the way. Seated: the cartridge flat bottom lands 1.0 under
the rim centres for both drones, so one head stop serves both.
python3 nox/box/catch_posts_concept.py   (writes runs/box_catch/posts_concept/result.json)
"""
import os, sys, json, math, numpy as np, trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, "nox")); sys.path.insert(0, os.path.join(REPO, "nox", "scout", "v3"))
import bottom_concept as bc
from system_check import load_part, pose_matrix
OUT = os.path.join(REPO, "runs", "box_catch", "posts_concept"); os.makedirs(OUT, exist_ok=True)

RIM, ROLLER_R, POST_R, CLEAR = bc.RIM_D, 5.0, 6.0, 1.0      # C (concept): shared rim, R 5 rollers, posts r 6, 1.0 to everything else
TOP = float(os.environ.get("POSTS_TOP", 72.0))              # post tops at (+-TOP, +-TOP); capture ~ RIM - TOP - post r - rim tube r
                                                             # (27 Oct runs: 72 / 45 deg holds +-30 for both; 61 / 35 deg only +-20)
ALPHA = float(os.environ.get("POSTS_ALPHA", 45.0))          # post slope from horizontal in its diagonal plane (45 = the funnel's faces)
U = np.array([1, 1, 0]) / math.sqrt(2)

def posts():
    """4 segments; the post's surface touches each corner roller at the seat (contact normal 45 deg up-out)"""
    P = []
    for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
        a = math.radians(ALPHA); u = np.array([sx, sy, 0]) / math.sqrt(2)
        n = math.sin(a) * u + [0, 0, math.cos(a)]; d = -math.cos(a) * u + [0, 0, math.sin(a)]   # contact normal, axis (up-in)
        c0 = np.array([sx * RIM, sy * RIM, 0.0]) - (ROLLER_R + POST_R) * n
        L = (np.hypot(c0[0], c0[1]) - TOP * math.sqrt(2)) / math.cos(a)
        P.append((c0 - 15 * d, c0 + L * d))
    return P

def seg_dist(Q, a, b):
    ab = b - a; t = np.clip(((Q - a) @ ab) / (ab @ ab), 0, 1)
    return np.linalg.norm(Q - (a + t[..., None] * ab), axis=-1)

def clearance(Q, rad, PS):
    return np.min([seg_dist(Q, a, b) for a, b in PS], axis=0) - POST_R - rad

def rim_ring(z, xc=0.0):
    """the shared rim: 8 nodes (corners with rollers), 6 mm tubes between them"""
    pts = [(RIM, 0), (RIM, RIM), (0, RIM), (-RIM, RIM), (-RIM, 0), (-RIM, -RIM), (0, -RIM), (RIM, -RIM)]
    N = np.array([[x + xc, y, z] for x, y in pts])
    return N, [(i, (i + 1) % 8) for i in range(8)]

def sample(N, R, rad_rod, rim_ids, extra):
    """points (seat frame), radius, is_rim"""
    P, rad, rim = [], [], []
    for i, p in enumerate(N):
        P.append(p); rad.append(ROLLER_R if i in rim_ids else 4.0); rim.append(i in rim_ids)
    for a, b in R:
        for t in np.linspace(0, 1, 31)[1:-1]:
            P.append(N[a] + t * (N[b] - N[a])); rad.append(rad_rod(a, b)); rim.append(a in rim_ids and b in rim_ids)
    for Q, r in extra:
        P += list(Q); rad += [r] * len(Q); rim += [False] * len(Q)
    return np.array(P), np.array(rad), np.array(rim)

def landing(P, rad, rim, PS, c):
    """worst over 8 directions x 3 yaws at offset c: first contact must be rim; then min clearance of the rest on the slide"""
    out = dict(first_rim=True, inward=True, min_clear=1e9, where=None)
    for k in range(8):
        a = 2 * math.pi * k / 8; off = np.array([c * math.cos(a), c * math.sin(a), 0.0])
        for yd in (-5.0, 0.0, 5.0):
            y = math.radians(yd); Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
            Q0 = P @ Rz.T + off
            Hs = np.arange(c * 1.6 + 10, -0.01, -0.5); Hc = None
            for H in Hs:
                cl = clearance(Q0 + [0, 0, H], rad, PS)
                if (cl <= 0).any():
                    Hc = H; hit = cl <= 0; break
            if Hc is None: continue
            if not rim[hit].all(): out["first_rim"] = False; out["where"] = f"first contact not the rim at c {c}, dir {k}, yaw {yd}"
            Qh = (Q0 + [0, 0, Hc])[hit]                     # the post must push the rim back toward the centre
            for a_, b_ in PS:
                ab = b_ - a_; t = np.clip(((Qh - a_) @ ab) / (ab @ ab), 0, 1); v = Qh - (a_ + t[:, None] * ab)
                near = np.linalg.norm(v, axis=1) <= POST_R + rad[hit] + 1e-6
                if near.any() and ((v[near, :2] @ off[:2]) > 0).any():
                    out["inward"] = False; out["where"] = f"a post pushes the rim outward at c {c}, dir {k}, yaw {yd}"
            for s in np.linspace(0, 1, 41):                  # slide: offset and height go to zero together
                y2 = math.radians(yd * (1 - s)); R2 = np.array([[math.cos(y2), -math.sin(y2), 0], [math.sin(y2), math.cos(y2), 0], [0, 0, 1]])
                Q = P @ R2.T + off * (1 - s) + [0, 0, Hc * (1 - s)]
                cl = clearance(Q[~rim], rad[~rim], PS)
                m = float(cl.min()) - CLEAR
                if m < out["min_clear"]: out["min_clear"] = round(m, 1); out["at"] = dict(c=c, dir=k, yaw=yd, s=round(float(s), 2), idx=int(np.argmin(cl)))
    return out

def scout():
    rec, N, R, tag, ids, kinds, links, dz = bc.layout("D")
    Ns = N + [0, 0, dz]
    rim_ids = set(ids["rings"][2])
    extra = []
    for nm, p0, p1 in links:
        extra.append((np.array([np.add(p0, t * np.subtract(p1, p0)) + [0, 0, dz] for t in np.linspace(0, 1, 21)]), 1.5))
    boxes = [b for _, b in rec["boxes"]] + [bc.cart_box(rec["cart_top"]), bc.PLATE, bc.FC]
    for b in boxes:
        g = np.array([[x, y, z + dz] for x in np.linspace(b[0], b[1], 9) for y in np.linspace(b[2], b[3], 6) for z in np.linspace(b[4], b[5], 6)])
        extra.append((g, 0.0))
    for nm, p0, p1, r in rec["struts"]:
        extra.append((np.array([np.add(p0, t * np.subtract(p1, p0)) + [0, 0, dz] for t in np.linspace(0, 1, 11)]), r))
    for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1)):                 # arms, motors, prop sweep
        extra.append((np.array([[sx * bc.MOTOR * t, sy * bc.MOTOR * t, 20.25 + dz] for t in np.linspace(0.5, 1, 9)]), 6.0))
        extra.append((np.array([[sx * bc.MOTOR, sy * bc.MOTOR, z + dz] for z in (22, 29, 36)]), 14.0))
    P, rad, rim = sample(Ns, R, lambda a, b: kinds[(a, b)] / 2, rim_ids, extra)
    return dict(P=P, rad=rad, rim=rim, cart_bottom_seat=round(rec["z_cart_bottom"] + dz, 1), dz=dz,
                nodes=Ns.tolist(), rods=[[int(a), int(b), kinds[(a, b)]] for a, b in R], links=[[list(np.add(p, [0, 0, dz])), list(np.add(q, [0, 0, dz]))] for _, p, q in links],
                cart=bc.cart_box(rec["cart_top"]), boxes=[b for _, b in rec["boxes"]])

def overseer(skip_sled=False):
    man = json.load(open(bc.OVR)); cb = -54.0; zr = cb + 1.0; dz = -zr; xc = bc.OVR_XC
    extra, parts = [], {}
    for p in man["parts"]:
        if p["name"].startswith(("cage shell", "spokes", "L2 guard", "TPU feet")): continue
        if skip_sled and p["name"].startswith(("belly sled", "down camera", "flow")): continue
        r = load_part(p, os.path.dirname(bc.OVR)); m = (r[0] if isinstance(r, tuple) else r).copy()
        if "pose" in p: m.apply_transform(pose_matrix(p["pose"]))
        Q = np.vstack([m.vertices, trimesh.sample.sample_surface(m, 2500, seed=2)[0]]) + [-xc, 0, dz]
        Q = Q[Q[:, 2] < 90]                                  # only what can reach a post (tops at seat z ~35)
        if len(Q): extra.append((Q, 0.0)); parts[p["name"]] = (m.bounds + [[-xc, 0, dz]]).tolist()
    N, R = rim_ring(0.0)
    P, rad, rim = sample(N, R, lambda a, b: 3.0, set(range(8)), extra)
    names = []
    for nm in parts: names.append(nm)
    sh = json.load(open(os.path.join(REPO, "runs", "overseer_layout", "v5", "cage_shell.json")))
    shell = dict(nodes=(np.array(sh["nodes"]) + [-xc, 0, dz]).round(1).tolist(), rods=sh["rods"])
    return dict(P=P, rad=rad, rim=rim, cart_bottom_seat=round(cb + dz, 1), dz=dz, parts=parts, shell=shell, rim_nodes=N.tolist())

def main():
    PS = posts()
    res = dict(rim=RIM, top=TOP, alpha=ALPHA, capture_geom=round(RIM - TOP - POST_R - 3.0 - 1.0, 1), post_r=POST_R, posts=[[a.tolist(), b.tolist()] for a, b in PS],
               post_height=round(float(PS[0][1][2]), 1))
    for name, d in (("scout", scout()), ("overseer", overseer())):
        seated = float((clearance(d["P"][~d["rim"]], d["rad"][~d["rim"]], PS) - CLEAR).min())
        caps = {}
        for c in (20, 30, 35, 40):
            caps[c] = landing(d["P"], d["rad"], d["rim"], PS, c)
        best = max([c for c, v in caps.items() if v["first_rim"] and v["inward"] and v["min_clear"] >= 0] or [0])
        res[name] = dict(seated_clear=round(seated, 1), capture=caps, capture_ok_mm=best, cart_bottom_seat=d["cart_bottom_seat"],
                         **{k: d[k] for k in ("nodes", "rods", "links", "cart", "boxes", "parts", "shell", "rim_nodes") if k in d})
        print(name, "seated", round(seated, 1), "capture ok to", best, {c: (v["first_rim"], v["inward"], v["min_clear"], v["where"]) for c, v in caps.items()})
    enc = lambda o: o.tolist() if hasattr(o, "tolist") else float(o)
    json.dump(res, open(os.path.join(OUT, "result.json"), "w"), default=enc)
    bcj = json.load(open(os.path.join(REPO, "runs", "scout_bottom", "concept", "concept.json")))      # run bottom_concept.py first
    tpl = open(os.path.join(REPO, "nox", "scout", "v3", "bottom_concept_view.html")).read()
    open(os.path.join(REPO, "runs", "scout_bottom", "concept", "concept.html"), "w").write(
        tpl.replace("/*DATA*/null", json.dumps(dict(bc=bcj, posts=res), default=enc)))       # the artifact

if __name__ == "__main__":
    main()
