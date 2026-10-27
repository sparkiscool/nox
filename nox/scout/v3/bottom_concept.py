"""Scout v3 bottom + landing concept (27 Oct, Spark: "the current bottom dosent look good ... might be wasting
performance if we can make it more compact"). Concept check only (docs/generative.md The rule, 0): no piece run.

Three layouts in drone = seat frame (x fwd, y left, z up; dimensions C / G / J / System box swap: latch):
  A  today: receiver v3 box (94.4 g, M3 x 35 columns) hung under the FC, cartridge below it (bottom z -44.5); a
     rule-compliant geodesic shell round it for a fair count (bottom ring under the cartridge).
  B  shell catch (DEFAULT): the same shared catch (8 wheels on the 45 deg faces, box unchanged), the receiver box
     replaced by two latch rails + the XT60 carrier hung on the plate and braced into the shell's bottom ring, the
     cartridge tucked up to the FC (top 1.0 under it); the shell's lower half is the funnel's negative (45 deg band
     inside the faces, a knee ring at the face edges, a short skirt to the bottom ring = floor feet + swap opening).
  C  belly catch (alternative): the wheels on the bottom ring and a new smaller funnel round it (box change).
Writes runs/scout_bottom/concept/: concept.json (geometry + numbers for the artifact), manifest_B.json,
system_check_B.json, concept.html (the artifact), summary lines on stdout.  python3 nox/scout/v3/bottom_concept.py
"""
import os, sys, json, math, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); REPO = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, os.path.join(REPO, "nox"))
OUT = os.path.join(REPO, "runs", "scout_bottom", "concept"); os.makedirs(OUT, exist_ok=True)

# ---- shared numbers (dimensions.md; tags in the comments)
PLATE = [-50, 50, -47, 47, 18.5, 22.0]            # C: Bee35 plate, underside 18.5, top 22
MOTOR = 54.1; PROP_R = 44.45; PROP_Z = 41.2; SWEEP = 4.0   # P / C
FC = [1.9 - 18, 1.9 + 18, -18, 18, 9.5, 17.5]     # P: 36 x 36, lowest z 9.5 (H)
PACK = (70, 32, 33, 165.0)                          # P
CART = (93.0, 40.0, 45.6, 50.5)                    # C / M: cartridge v7 set (z x x x y), 50.5 g per drone set
FLOOR = 12.0                                        # C: pack floor above the flat bottom (grabber recess)
FACE_K = 82.7      # C (G Opening limit): face planes |x| = |y| = 82.7 + z (bottom edges: +-y at z 1.3, half 84; +-x at 12.8, half 95.5)
EDGE_Y, HALF_Y, EDGE_X, HALF_X = 1.3, 84.0, 12.8, 95.5
WHEEL_R = 5.0; WHEEL_AT = 112.0                     # C: R 5 wheels, centres at the shared points (+-112, +-100) / (+-100, +-112)
Z_WHEEL = WHEEL_AT - FACE_K + WHEEL_R * math.sqrt(2)  # wheel centre height on the face (C) = 36.4
YAW = 5.0; SIDE = 3.0                               # G (2): +-5 deg yaw, 3 a side below the edges, 1.0 off the faces above
TUBE6_GPM, ROD3_GPM = 25.0, 11.0                   # E: 6 x 4 carbon tube ~25 g/m; 3.0 solid rod 11.0 g/m (G)
CORE_G = 400.0 - 165.0 + 10 + 20                   # P: 400 g with pack, no cage (scout.md); + Radxa 10, UBEC 20 (E)

def cart_box(z_top):
    L, W, H, _ = CART
    return [-L / 2, L / 2, -W / 2, W / 2, z_top - H, z_top]

def pack_box(z_cart_bottom):
    L, W, H, _ = PACK
    return [-L / 2, L / 2, -W / 2, W / 2, z_cart_bottom + FLOOR, z_cart_bottom + FLOOR + H]

# ---- receivers
def receiver_A():
    """receiver v3 as placed today (System box swap: latch): mount plane z 9.6, body to the cartridge bottom, 65.5 wide."""
    zb = -44.5
    return dict(boxes=[("receiver v3 (box)", [-53, 53, -32.75, 32.75, zb + 0.2, 9.6])], mass=114.9, z_cart_bottom=zb,
                cart_top=zb + CART[2], tag="M")

def receiver_B():
    """two latch rails (pocket blocks + lead-ins) on four 8 x 8 posts to the plate beside the FC, XT60 carrier at the
    nose end between the cartridge top and the plate; the rails end in the shell's bottom ring (braced, not a stalk)."""
    ct = FC[4] - 1.0                                  # cartridge top 1.0 under the FC (TPU preload strip on the rail ledges)
    zb = ct - CART[2]
    py0, py1 = zb + 5.75, zb + 11.85                  # pocket mouth (latch row) in drone z
    r0, r1 = py0 - 3.0, py1 + 3.0                     # rail with 3.0 walls round the pocket
    bx = []
    for s in (1, -1):
        yy = sorted([s * 20.3, s * 28.3])
        bx.append((f"latch rail {'L' if s > 0 else 'R'}", [-50, 50, yy[0], yy[1], r0, r1]))
        for x0 in (-44, 36):
            bx.append((f"rail post {'L' if s > 0 else 'R'}{'f' if x0 > 0 else 't'}", [x0, x0 + 8, yy[0], yy[1], r1, PLATE[4]]))
    bx.append(("XT60 carrier + cap", [31, 47, -9.25, 9.25, ct, PLATE[4]]))
    # E: rails 2 x (100 x 8 x 12.1 at 0.8 fill), posts 4 x 8 x 8 x ~26, carrier + cap 10.3 (M, latch v4), 2 TPU strips 2.0 (M)
    m_rail = 2 * 100 * 8 * (r1 - r0) * 0.8e-3 * 1.27
    m_post = 4 * 8 * 8 * (PLATE[4] - r1) * 0.8e-3 * 1.27
    return dict(boxes=bx, mass=round(m_rail + m_post + 10.3 + 2.0, 1), z_cart_bottom=zb, cart_top=ct, tag="E",
                parts=dict(rails=round(m_rail, 1), posts=round(m_post, 1), carrier_cap=10.3, strips=2.0))

# ---- shell: apex, top ring 4, equator 8 (wheels), knee 8 (face edges), bottom 8 (opening + feet); antiprism bands
def az(k, off=0.0):
    a = math.radians(off + 45.0 * k); return math.cos(a), math.sin(a)

def shell(z_bottom, knee=True, low_equator=False):
    N, tag = [], []
    def add(p, t): N.append([round(float(v), 2) for v in p]); tag.append(t); return len(N) - 1
    apex = add((0, 0, 60.0), "apex")
    T = [add((118 * c, 118 * s, 56.0), "top") for c, s in (az(k * 2 + 1) for k in range(4))]  # over the prop tips (r 121), 9.3 over the sweep
    E = []
    for k in range(8):
        c, s = az(k)
        if low_equator:                                   # C: the widest ring at the prop plane, no wheels on it
            E.append(add(((106 * math.copysign(1, c), 106 * math.copysign(1, s)) if k % 2 else (112 * c, 112 * s)) + (40.0,), "equator"))
            continue
        if k % 2 == 1:                                    # corner node across the chamfer, carries 2 wheels
            E.append(add((106 * math.copysign(1, c), 106 * math.copysign(1, s), Z_WHEEL), "wheel node"))
        elif k in (0, 4):                                 # +-x mid raised above the pad top (G (8) probe keep-out)
            E.append(add((104 * c, 0, 52.0), "equator"))
        else:                                             # +-y mid raised the same (crown): guards the props from above-side
            E.append(add((0, 104 * s, 52.0), "equator"))
    K = []
    if knee:
        for k in range(8):
            c, s = az(k, 22.5)
            near_x = abs(c) > abs(s)                      # under the +-x faces' edge (12.8) / the +-y faces' edge (1.3)
            K.append(add((72 * math.copysign(1, c), 36 * math.copysign(1, s), 7.0) if near_x else   # 12 off the +-x faces (G (8) pins)
                         (38 * math.copysign(1, c), 68 * math.copysign(1, s), -3.0), "knee"))
    B = []
    for k in range(8):
        c, s = az(k)
        if k in (0, 4): p = (86 * c, 0, z_bottom)
        elif k in (2, 6): p = (0, 66 * s, z_bottom)
        else: p = (64 * math.copysign(1, c), 60 * math.copysign(1, s), z_bottom)
        if low_equator: p = (p[0] * 1.25, p[1] * 1.55, z_bottom)       # C: the wheel ring
        B.append(add(p, "foot"))
    R = []
    for i in range(4):
        R += [(apex, T[i]), (T[i], T[(i + 1) % 4])]
        R += [(T[i], E[2 * i]), (T[i], E[2 * i + 1]), (T[i], E[(2 * i + 2) % 8])]
    R += [(E[i], E[(i + 1) % 8]) for i in range(8)]
    low = K if knee else B
    for i in range(8):                                    # antiprism E (k) - K (k + 22.5)
        R += [(E[i], low[i]), (E[(i + 1) % 8], low[i])] if knee else [(E[i], B[i])]
    if knee:
        R += [(K[i], K[(i + 1) % 8]) for i in range(8)]
        for i in range(8):
            R += [(K[i], B[i]), (K[i], B[(i + 1) % 8])]
    else:
        R += [(E[i], B[(i + 1) % 8]) for i in range(0, 8, 2)] + [(E[(i + 1) % 8], B[i]) for i in range(1, 8, 2)]
    R += [(B[i], B[(i + 1) % 8]) for i in range(8)]
    R = sorted({tuple(sorted(r)) for r in R})
    return np.array(N, float), R, tag, dict(apex=apex, T=T, E=E, K=K, B=B)

def rod_kind(tag, a, b):
    """6 mm tubes on the equator ring (catch wheels, side hits); 3.0 rods elsewhere (Spark 27 Oct: 6 and 3 only; the frame model sizes them)"""
    return 6.0 if {tag[a], tag[b]} <= {"wheel node", "equator"} else 3.0

def node_g(t):
    return {"wheel node": 3.5, "foot": 2.0, "apex": 1.5}.get(t, 1.5)    # E: v2 corner node 3.08 + 2 wheels 0.34 (M); TPU feet ~2

def shell_mass(N, R, tag):
    L6 = sum(np.linalg.norm(N[a] - N[b]) for a, b in R if rod_kind(tag, a, b) == 6.0) / 1000
    L3 = sum(np.linalg.norm(N[a] - N[b]) for a, b in R if rod_kind(tag, a, b) == 3.0) / 1000
    return dict(tube6_m=round(L6, 2), rod3_m=round(L3, 2), nodes=len(N), g=round(L6 * TUBE6_GPM + L3 * ROD3_GPM + sum(node_g(t) for t in tag) + 6.0, 1))

# ---- catch checks in the seat frame (G (1), (2), (7), (8)), each point at +-5 deg yaw
def face_clear(p, rad):
    """min margin (mm) of a sphere at p: above an edge it must stay 1.0 inside the face, below it 3 inside the opening"""
    worst = 1e9
    for yd in (-YAW, 0, YAW):
        a = math.radians(yd); x = p[0] * math.cos(a) - p[1] * math.sin(a); y = p[0] * math.sin(a) + p[1] * math.cos(a); z = p[2]
        for v, edge, half in ((abs(x), EDGE_X, HALF_X), (abs(y), EDGE_Y, HALF_Y)):
            if z >= edge: m = (FACE_K + z - v) / math.sqrt(2) - rad - 1.0
            else: m = half - v - rad - SIDE                                                         # below: inside the opening
            worst = min(worst, m)
    return worst

def plunger_clear(p, rad):
    """G (7): extended plunger 12 (x 94..106) x 8 (slope 50.1..58.1 from the +-y edge) x 12 off the face"""
    worst = 1e9
    for sy in (1, -1):
        for sx in (1, -1):
            v = sy * p[1]; s = ((v - HALF_Y) + (p[2] - EDGE_Y)) / math.sqrt(2); n = (FACE_K + p[2] - v) / math.sqrt(2)
            dx = max(94 - sx * p[0], 0, sx * p[0] - 106); ds = max(50.1 - s, 0, s - 58.1); dn = max(-n, 0, n - 12)
            worst = min(worst, math.sqrt(dx * dx + ds * ds + dn * dn) - rad - 1.0)
    return worst

def probe_clear(p, rad):
    """G (8): nothing within 9.0 of the +-x faces over y -35..+69 (+x) / -69..+35 (-x) below the pad top (z 35.8)"""
    worst = 1e9
    for sx in (1, -1):
        yy = sx * p[1]
        if -35 <= yy <= 69 and p[2] - rad < 33 + 4 / math.sqrt(2) and p[2] + rad > EDGE_X:   # where the +-x face exists
            n = (FACE_K + p[2] - sx * p[0]) / math.sqrt(2)
            worst = min(worst, n - rad - 9.0)
    return worst

def catch_checks(N, R, tag, kinds):
    out = dict(face=1e9, plunger=1e9, probe=1e9, where={})
    pts = []
    for i, p in enumerate(N):
        if tag[i] != "wheel node": pts.append((p, 5.0, f"node {i} {tag[i]}"))
    for a, b in R:
        if "wheel node" in (tag[a], tag[b]):                      # the wheel nodes own their first 12 mm (v2 corner node practice)
            pass
        r = kinds[(a, b)] / 2
        for t in np.linspace(0, 1, 41)[1:-1]:
            q = N[a] + t * (N[b] - N[a])
            if min(np.linalg.norm(q - N[a]), np.linalg.norm(q - N[b])) < 12 and "wheel node" in (tag[a] if np.linalg.norm(q - N[a]) < 12 else tag[b],): continue
            pts.append((q, r, f"rod {a}-{b}"))
    for p, r, nm in pts:
        for k, fn in (("face", face_clear), ("plunger", plunger_clear), ("probe", probe_clear)):
            m = fn(p, r)
            if m < out[k]: out[k] = m; out["where"][k] = nm
    for k in ("face", "plunger", "probe"): out[k] = round(out[k], 1)
    return out

def opening_inner(N, ids):
    P = N[ids]; return dict(x=round(float(P[:, 0].max() * 2 - 2 * 5), 1), y=round(float(P[:, 1].max() * 2 - 2 * 5), 1))

# ---- the system_check manifest of B (concept geometry: boxes, rods as cylinders, nodes as spheres)
def manifest(name, N, R, tag, kinds, rec, cart_top, extra):
    parts = [dict(name="frame plate", box=PLATE, role="structure", mass_g=28, tag="E"),
             dict(name="wiring, UBEC, XR2, camera, flow (lump)", box=[-30, 30, -20, 20, 22, 34], role="bought", mass_g=round(CORE_G - 176, 1), tag="E"),
             dict(name="FC MicoAir743", box=FC, role="protected", protect=True, mass_g=10, tag="P"),
             dict(name="Radxa Zero 3W (top)", box=[-32.5, 32.5, -15, 15, 23.0, 30.0], role="protected", protect=True, mass_g=10, tag="E"),
             dict(name="pack 4S 1500", box=pack_box(cart_top - CART[2]), role="protected", protect=True, mass_g=165, tag="P", swap=True),
             dict(name="cartridge", box=cart_box(cart_top), role="protected", protect=True, mass_g=CART[3], tag="M", swap=True)]
    for i, (sx, sy) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1))):
        c = [sx * MOTOR, sy * MOTOR]
        parts.append(dict(name=f"motor {i+1}", cylinder=dict(r=14, p0=c + [22.0], p1=c + [36.0]), role="protected", protect=True, mass_g=28, tag="E"))
        parts.append(dict(name=f"prop {i+1}", cylinder=dict(r=PROP_R, p0=c + [PROP_Z - SWEEP], p1=c + [PROP_Z + SWEEP]), role="protected", protect=True, mass_g=4, tag="P"))
        parts.append(dict(name=f"arm {i+1}", cylinder=dict(r=6, p0=[sx * 30, sy * 30, 20.25], p1=c + [20.25]), role="structure", mass_g=0, tag="E"))
    vol = [(b[1] - b[0]) * (b[3] - b[2]) * (b[5] - b[4]) for _, b in rec["boxes"]]
    for (nm, bx), v in zip(rec["boxes"], vol):                        # receiver mass spread by volume (E)
        parts.append(dict(name=nm, box=bx, role="structure", mass_g=round(rec["mass"] * v / sum(vol), 2), tag=rec["tag"]))
    for i, p in enumerate(N):
        parts.append(dict(name=f"node {i} {tag[i]}", sphere=dict(r=5.0, center=p.tolist()), role="cage", mass_g=node_g(tag[i]), tag="E"))
    for a, b in R:
        d = kinds[(a, b)]; L = float(np.linalg.norm(N[a] - N[b]))
        parts.append(dict(name=f"rod {a}-{b}", cylinder=dict(r=d / 2, p0=N[a].tolist(), p1=N[b].tolist()), role="cage",
                          mass_g=round(L / 1000 * (TUBE6_GPM if d == 6 else ROD3_GPM), 2), tag="E"))
    contact = [[f"motor {i}", f"prop {i}", 1e9] for i in range(1, 5)] + [[f"arm {i}", f"motor {i}", 1e9] for i in range(1, 5)]
    contact += [[f"arm {i}", "frame plate", 1e9] for i in range(1, 5)] + [["pack 4S 1500", "cartridge", 1e9]]
    contact += [[nm, "pack 4S 1500", 1e9] for nm, _ in rec["boxes"] if "box" in nm]      # A: the stand-in box is the receiver's envelope
    names = [p["name"] for p in parts]
    for i, (a, b) in enumerate(R):                                    # rods meeting in one node
        for c_, d_ in R[i + 1:]:
            if {a, b} & {c_, d_}: contact.append([f"rod {a}-{b}", f"rod {c_}-{d_}", 1e9])
    contact += [["wiring, UBEC, XR2, camera, flow (lump)", n2, 1e9] for n2 in names if not n2.startswith(("rod", "node"))]
    for nm in names:                                                  # designed joints: rods in their nodes, rails on the plate
        if nm.startswith("rod "):
            a, b = nm[4:].split("-"); contact += [[nm, n2, 1e9] for n2 in names if n2.startswith(f"node {a} ") or n2.startswith(f"node {b} ")]
    for nm, _ in rec["boxes"]:
        contact += [[nm, "frame plate", 1e9], [nm, "cartridge", 1e9]] + [[nm, n2, 1e9] for n2, _ in rec["boxes"] if n2 != nm]
    ids = extra["B"]
    return dict(name=name, machine="scout", parts=parts, contact_pairs=contact, cg_ref=[0, 0, PROP_Z], thrust_axis=[0, 0, 1],
                cg_tol_mm=3.0, cage_margin_mm=3.0, mass_budget_g=500, probes=False,
                openings=[dict(axis=[0, 0, -1], half_angle_deg=40.0, min_mm=0.0)],
                cage_shell=dict(nodes=N.tolist(), rods=[list(r) for r in R], open=[dict(ring=ids)], mirror="y", rod_r_mm=1.5))

def build(which):
    rec = receiver_A() if which == "A" else receiver_B()
    zb = rec["z_cart_bottom"] + 2.0                              # bottom ring node centre: its r 5 foot face 3.0 under the flat bottom
    N, R, tag, ids = shell(zb, knee=True, low_equator=(which == "C"))
    if which == "C":
        N, R, tag, ids = shell(zb, knee=False, low_equator=True)
    kinds = {r: rod_kind(tag, *r) for r in R}
    return rec, N, R, tag, ids, kinds

def main():
    res = {}
    for which in ("A", "B", "C"):
        rec, N, R, tag, ids, kinds = build(which)
        sm = shell_mass(N, R, tag)
        cc = catch_checks(N, R, tag, kinds) if which != "C" else None
        zlo = float(N[:, 2].min() - 5.0); zhi = float(N[:, 2].max() + 5.0)
        drone_g = CORE_G + PACK[3] + CART[3] + rec["mass"] + sm["g"]
        res[which] = dict(receiver=dict(boxes=rec["boxes"], mass=rec["mass"], tag=rec["tag"], parts=rec.get("parts")),
                          cart=cart_box(rec["cart_top"]), pack=pack_box(rec["z_cart_bottom"]),
                          nodes=N.tolist(), rods=[[int(a), int(b), kinds[(a, b)]] for a, b in R], tags=tag, ids={k: (v if isinstance(v, list) else [v]) for k, v in ids.items()},
                          shell=sm, catch=cc, z_low=round(zlo, 1), z_high=round(zhi, 1), height=round(zhi - zlo, 1),
                          below_plate=round(PLATE[4] - zlo, 1), opening=opening_inner(N, ids["B"]),
                          swap_on_drone_g=round(CART[3] + rec["mass"], 1), drone_g=round(drone_g, 1),
                          pack_cz=round((pack_box(rec["z_cart_bottom"])[4] + pack_box(rec["z_cart_bottom"])[5]) / 2, 1),
                          wheel_to_cart_bottom=round(Z_WHEEL - rec["z_cart_bottom"], 1) if which != "C" else round(float(N[ids["B"], 2].mean()) - rec["z_cart_bottom"], 1))
    a, b = res["A"], res["B"]
    b["vs_A"] = dict(mass_g=round(b["drone_g"] - a["drone_g"], 1), height=round(b["height"] - a["height"], 1),
                     below_plate=round(b["below_plate"] - a["below_plate"], 1),
                     hover_power_pct=round(((b["drone_g"] / a["drone_g"]) ** 1.5 - 1) * 100, 1))
    # C: the wheels on the bottom ring; a 45 deg funnel round it with the same +-40 capture (E)
    c = res["C"]; Bn = np.array(c["nodes"])[c["ids"]["B"]]; rc = float(np.max(np.abs(Bn[:, :2])))
    c["funnel"] = dict(contact_half=round(rc, 1), mouth=round(2 * (rc + 40 + 10), 0), height=round(40 + 10 + 6, 0),
                       note="E: mouth = contact + 40 capture + 10 at 45 deg; the box catch redraws panels, hook latch and probes")
    # system_check on B
    from system_check import run
    _, N, R, tag, ids, kinds = build("B")
    rec = receiver_B()
    man = manifest("Scout v3 bottom concept B (shell catch)", N, R, tag, kinds, rec, rec["cart_top"], ids)
    json.dump(man, open(os.path.join(OUT, "manifest_B.json"), "w"))
    sc = run(man, probes=False)
    json.dump(sc, open(os.path.join(OUT, "system_check_B.json"), "w"), default=str, indent=1)
    b["system_check"] = sc["summary"]
    rec_a = receiver_A(); _, Na, Ra, taga, idsa, kindsa = build("A")
    man_a = manifest("Scout v3 bottom concept A (today, shell for the count)", Na, Ra, taga, kindsa, rec_a, rec_a["cart_top"], idsa)
    sca = run(man_a, probes=False); a["system_check"] = sca["summary"]
    res["numbers"] = dict(Z_WHEEL=round(Z_WHEEL, 2), CORE_G=CORE_G, FC=FC, PLATE=PLATE, face_k=FACE_K, edges=[EDGE_Y, HALF_Y, EDGE_X, HALF_X],
                          motor=MOTOR, prop_r=PROP_R, prop_z=PROP_Z)
    json.dump(res, open(os.path.join(OUT, "concept.json"), "w"), default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    data = json.dumps(res, default=lambda o: o.tolist() if hasattr(o, "tolist") else float(o) if isinstance(o, np.floating) else str(o))
    html = open(os.path.join(HERE, "bottom_concept_view.html")).read().replace("/*DATA*/null", data)
    open(os.path.join(OUT, "concept.html"), "w").write(html)                # the artifact (source: bottom_concept_view.html)
    for w in ("A", "B", "C"):
        r = res[w]
        print(f"{w}: height {r['height']} (z {r['z_low']}..{r['z_high']}), below plate {r['below_plate']}, swap parts on the drone {r['swap_on_drone_g']} g, "
              f"shell {r['shell']}, drone {r['drone_g']} g E, pack centre z {r['pack_cz']}, opening {r['opening']}, catch {r['catch']}, wheel-cart {r['wheel_to_cart_bottom']}")
    print("B vs A:", b["vs_A"]); print("A", a["system_check"]); print("B", b["system_check"]); print("C funnel", c["funnel"])

if __name__ == "__main__":
    main()
