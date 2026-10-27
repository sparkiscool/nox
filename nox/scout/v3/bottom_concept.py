"""Scout v3 bottom + landing concept, and how the Overseer shares the catch (27 Oct, Spark: "the current bottom dosent look
good ... might be wasting performance if we can make it more compact"; "if we go with b how will the overseer fit").
Concept check only (docs/generative.md The rule, 0): no piece run.

Geometry in the drone frame (x fwd, y left, z up; plate top z 22; dimensions C / G / J / System box swap: latch);
every catch check in the seat frame = the funnel's (G: face planes |x| = |y| = 82.7 + z, +-y edge z 1.3 half 84, +-x
edge z 12.8 half 95.5, mouth z 77.3 half 160), the seated drone placed by its wheels.
  A  today's latch set: receiver v3 box (94.4 g, M3 x 35 columns) under the FC, cartridge bottom z -44.5, wheels high
     (seat = drone); a shell round it for a fair count.
  B  shell catch: wheels on the shell's crown at the prop plane (seat = drone), the pack 1.0 under the FC, two latch
     rails for the receiver box, the lower shell a 45 deg pyramid inside the capture cone. Scout only: the Overseer's
     171 cartridge cannot hang that low (overseer_fit), so the box would need two head stops.
  C  belly crown (DEFAULT 27 Oct): the 8 wheels on the cage's bottom ring for both drones, the same pack tuck and rails,
     everything outside the 320 mouth above it; both cartridges seat at the same height (one head stop); the box
     seats the drone 79 lower against the swap wheel than today.
Then nox/box/catch_posts_concept.py: D, the inside-out catch (4 posts into a shared rim), and the artifact.
Capture (G (3), +-40 at +-5 deg yaw): every point not a wheel stays 1.0 inside the infinite face planes, or above the
mouth, or (below the edges at 40 off) inside the throat with 3 a side. Writes runs/scout_bottom/concept/.
python3 nox/scout/v3/bottom_concept.py
"""
import os, sys, json, math, numpy as np, trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); REPO = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, os.path.join(REPO, "nox"))
OUT = os.path.join(REPO, "runs", "scout_bottom", "concept"); os.makedirs(OUT, exist_ok=True)
from system_check import run, load_part, pose_matrix

# ---- shared numbers (dimensions.md; tags in the comments)
PLATE = [-50, 50, -47, 47, 18.5, 22.0]            # C: Bee35 plate, underside 18.5, top 22
MOTOR = 54.1; PROP_R = 44.45; PROP_Z = 41.2; SWEEP = 4.0   # P / C
FC = [1.9 - 18, 1.9 + 18, -18, 18, 9.5, 17.5]     # P: 36 x 36, lowest z 9.5 (H)
PACK = (70, 32, 33, 165.0)                          # P
CART = (93.0, 40.0, 45.6, 50.5)                    # C / M: cartridge set (length, width, height), 50.5 g per drone set
FLOOR = 12.0                                        # C: pack floor above the flat bottom (grabber recess)
FACE_K, MOUTH_Z, MOUTH_H = 82.7, 77.3, 160.0       # C (G): face planes, mouth top and half width
EDGE_Y, HALF_Y, EDGE_X, HALF_X = 1.3, 84.0, 12.8, 95.5
WHEEL_R = 5.0; WHEEL_AT = 111.3                     # C: R 5, contact 43.6 up the slope (hook latch v2 datum, review 27 Oct)
Z_WHEEL = WHEEL_AT - FACE_K + WHEEL_R * math.sqrt(2)  # seat z of the wheel centres = 35.67
YAW, SIDE, CAPTURE, LIFT = 5.0, 3.0, 40.0, 5.7      # G (2), (3), (7)
TUBE6_GPM, ROD3_GPM = 25.0, 11.0                   # E: 6 x 4 carbon tube; 3.0 solid rod 11.0 g/m (G)
CORE_G = 400.0 - 165.0 + 10 + 20                   # P: 400 g with pack, no cage (scout.md); + Radxa 10, UBEC 20 (E), no pack
RIM_D = 110.0                                       # D: shared bottom rim, corner rollers at (+-110, +-110) (catch_posts_concept.py)
OVR = os.path.join(REPO, "nox", "overseer", "overseer.assembly.json"); OVR_XC = -30.0   # Overseer catch centre (catch 30 aft, C)
S2 = math.sqrt(2)

def cart_box(z_top):
    L, W, H, _ = CART
    return [-L / 2, L / 2, -W / 2, W / 2, z_top - H, z_top]

def pack_box(z_cart_bottom):
    L, W, H, _ = PACK
    return [-L / 2, L / 2, -W / 2, W / 2, z_cart_bottom + FLOOR, z_cart_bottom + FLOOR + H]

# ---- receivers
def receiver_A():
    """receiver v3 as placed today (System box swap: latch): mount plane z 9.6, body to the cartridge bottom, 65.5 wide
    (its ends are clipped to the funnel inset with TPU end pieces; the box here is its envelope)."""
    zb = -44.5
    return dict(boxes=[("receiver v3 (envelope)", [-53, 53, -32.75, 32.75, zb + 0.2, 9.6])], struts=[], mass=114.9,
                z_cart_bottom=zb, cart_top=zb + CART[2], tag="M")

def receiver_tucked():
    """two latch rails (pocket blocks, 3.0 walls) on two A-frames each to the plate beside the FC (a post and a strut),
    XT60 carrier at the nose end between the cartridge top and the plate; cartridge top 1.0 under the FC (TPU strip)"""
    ct = FC[4] - 1.0
    zb = ct - CART[2]
    r0, r1 = zb + 5.75 - 3.0, zb + 11.85 + 3.0
    bx, st = [], []
    for s in (1, -1):
        y0, y1 = sorted([s * 20.3, s * 28.3]); yc = (y0 + y1) / 2
        bx.append((f"latch rail {'L' if s > 0 else 'R'}", [-50, 50, y0, y1, r0, r1]))
        for sx in (1, -1):
            bx.append((f"rail post {'L' if s > 0 else 'R'}{'f' if sx > 0 else 't'}", sorted([sx * 36, sx * 44]) + [y0, y1, r1, PLATE[4]]))
            st.append((f"rail strut {'L' if s > 0 else 'R'}{'f' if sx > 0 else 't'}", [sx * 48, yc, r1 - 2], [sx * 12, yc, PLATE[4]], 4.0))
    bx.append(("XT60 carrier + cap", [31, 47, -9.25, 9.25, ct, PLATE[4]]))
    m_rail = 2 * 100 * 8 * (r1 - r0) * 0.8e-3 * 1.27
    m_post = 4 * 8 * 8 * (PLATE[4] - r1) * 0.8e-3 * 1.27
    m_strut = sum(np.linalg.norm(np.subtract(b, a)) * math.pi * r * r for _, a, b, r in st) * 0.6e-3 * 1.27
    return dict(boxes=bx, struts=st, mass=round(m_rail + m_post + m_strut + 10.3 + 2.0, 1), z_cart_bottom=zb, cart_top=ct, tag="E",
                parts=dict(rails=round(m_rail, 1), posts=round(m_post, 1), struts=round(m_strut, 1), carrier_cap=10.3, strips=2.0))

# ---- shells: rings of 8 (azimuth k*45 or 22.5 + k*45) stitched as antiprisms; mirror-exact about xz by construction
def oct8(pts):
    """8 points from first-octant representatives: {'x': (r, z) on +x, 'd': (x, y, z) diagonal, 'y': (r, z) on +y} or
    an offset ring {'a': (x, y, z) near +x, 'b': (x, y, z) near +y}"""
    if "d" in pts:
        (rx, zx), (dx, dy, dz), (ry, zy) = pts["x"], pts["d"], pts["y"]
        return [(rx, 0, zx), (dx, dy, dz), (0, ry, zy), (-dx, dy, dz), (-rx, 0, zx), (-dx, -dy, dz), (0, -ry, zy), (dx, -dy, dz)]
    (ax, ay, az), (bx, by, bz) = pts["a"], pts["b"]
    return [(ax, ay, az), (bx, by, bz), (-bx, by, bz), (-ax, ay, az), (-ax, -ay, az), (-bx, -by, bz), (bx, -by, bz), (ax, -ay, az)]

def shell(rings, wheel_ring, opening_ring, pads):
    """rings top to bottom after the apex and the 4-node top ring: alternating k*45 / 22.5 + k*45 rings of 8"""
    N, tag = [], []
    def add(p, t): N.append([round(float(v), 2) for v in p]); tag.append(t); return len(N) - 1
    apex = add((0, 0, 60.0), "apex")
    T = [add((83.4 * sx, 83.4 * sy, 56.0), "top") for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1))]   # over the prop tips
    ids = []
    for i, (name, pts) in enumerate(rings):
        ids.append([add(p, ("wheel node" if (i == wheel_ring and k % 2 == 1) else name)) for k, p in enumerate(oct8(pts))])
    R = []
    for i in range(4):
        R += [(apex, T[i]), (T[i], T[(i + 1) % 4]), (T[i], ids[0][2 * i]), (T[i], ids[0][2 * i + 1]), (T[i], ids[0][(2 * i + 2) % 8])]
    for i, ring in enumerate(ids):
        R += [(ring[k], ring[(k + 1) % 8]) for k in range(8)]
        if i + 1 < len(ids):
            nxt = ids[i + 1]
            if "d" in rings[i][1]:                       # k*45 above a 22.5 + k*45 ring: node k between k and k+1
                R += [(ring[k], nxt[k]) for k in range(8)] + [(ring[(k + 1) % 8], nxt[k]) for k in range(8)]
            else:
                R += [(ring[k], nxt[k]) for k in range(8)] + [(ring[k], nxt[(k + 1) % 8]) for k in range(8)]
    P = []
    for p, (a, b) in pads:                               # pad carriers: a node per pad spot on both sides (mirror), 2 links up-in
        j = add(p, "pad"); P.append(j); R += [(j, a(ids, T)), (j, b(ids, T))]
    R = sorted({tuple(sorted(r)) for r in R})
    return np.array(N, float), R, tag, dict(apex=[apex], T=T, rings=ids, wheel=ids[wheel_ring], opening=ids[opening_ring], pads=P)

def rod_kind(tag, a, b):
    """6 mm tubes on the wheel ring (catch) and at the widest ring; 3.0 rods elsewhere (Spark 27 Oct: 6 and 3 only)"""
    return 6.0 if {tag[a], tag[b]} <= {"wheel node", "crown", "equator"} else 3.0

def node_g(t):
    return {"wheel node": 3.5, "pad": 2.5, "foot": 2.0}.get(t, 1.5)   # E: v2 corner node 3.08 + 2 wheels 0.34 (M)

def shell_mass(N, R, tag, links):
    L6 = sum(np.linalg.norm(N[a] - N[b]) for a, b in R if rod_kind(tag, a, b) == 6.0) / 1000
    L3 = (sum(np.linalg.norm(N[a] - N[b]) for a, b in R if rod_kind(tag, a, b) == 3.0) + sum(np.linalg.norm(np.subtract(q, p)) for _, p, q in links)) / 1000
    return dict(tube6_m=round(L6, 2), rod3_m=round(L3, 2), nodes=len(N), g=round(L6 * TUBE6_GPM + L3 * ROD3_GPM + sum(node_g(t) for t in tag) + 6.0, 1))

def layout(which):
    if which == "A":
        rec = receiver_A()
    else:
        rec = receiver_tucked()
    cb = rec["z_cart_bottom"]
    pad_x = lambda zc: [((111.0 * sx, 17.0 * sy, zc), None) for sx in (1, -1) for sy in (1, -1)]
    if which in ("A", "B"):                              # crown at the prop plane (seat = drone), lower 45 deg pyramid
        zb = cb + 1.0                                    # bottom ring centres: r 4 nodes reach 3.0 under the flat bottom
        rings = [("crown", {"x": (104, 52.0), "d": (105.0, 105.0, Z_WHEEL), "y": (104, 52.0)}),
                 ("knee", {"a": (55, 20, -8.0), "b": (20, 55, -8.0)}),
                 ("foot", {"x": (48, zb), "d": (44.0, 24.3, zb), "y": (36, zb)})]
        wheel, opening, dz, zp = 0, 2, 0.0, 36.0
        pads = [(p, (lambda ids, T, sx=p[0], sy=p[1]: ids[0][0 if sx > 0 else 4],
                     lambda ids, T, sx=p[0], sy=p[1]: T[{(1, 1): 0, (-1, 1): 1, (-1, -1): 2, (1, -1): 3}[(int(np.sign(sx)), int(np.sign(sy)))]]))
                for p, _ in pad_x(zp)]
    else:                                                # C / D: crown on the belly, a drum flank up to the equator at the props
        zw = cb + 1.0                                    # wheel / crown centres: R 5 wheels reach 4.0 under the flat bottom
        dz = Z_WHEEL - zw if which == "C" else -zw       # seat = drone + dz (D: seat z 0 = the rim, catch_posts_concept.py)
        crown = {"x": (90, zw), "d": (105.0, 105.0, zw), "y": (104, zw)} if which == "C" else \
                {"x": (RIM_D, zw), "d": (RIM_D, RIM_D, zw), "y": (RIM_D, zw)}     # D: the shared rim, rollers on the corners
        rings = [("equator", {"x": (112, 40.0), "d": (106.0, 106.0, 40.0), "y": (112, 40.0)}),
                 ("knee", {"a": (110, 45, 0.0), "b": (45, 110, 0.0)} if which == "C" else
                          {"a": (110, 45, 12.0), "b": (45, 110, 12.0)}),           # D: above the post tops (seat 35.4 + 6)
                 ("crown", crown)]
        wheel, opening, zp = 2, 2, zw + 0.3
        pads = [(p, (lambda ids, T, sx=p[0], sy=p[1]: ids[2][0 if sx > 0 else 4],
                     lambda ids, T, sx=p[0], sy=p[1]: ids[0][0 if sx > 0 else 4])) for p, _ in pad_x(zp)] if which == "C" else []
    N, R, tag, ids = shell(rings, wheel, opening, pads)
    kinds = {r: rod_kind(tag, *r) for r in R}
    links = []                                           # 3 mm links into the core: knee / ring nodes to the arm tips, rails to the bottom ring
    tips = [(sx * MOTOR, sy * MOTOR, 20.25) for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
    for k in ids["rings"][1]:
        t = min(tips, key=lambda q: np.linalg.norm(np.subtract(q[:2], N[k][:2])))
        links.append((f"spoke {k}", N[k].tolist(), list(t)))
    if which != "A":
        low = ids["rings"][2]
        for s in (1, -1):
            for sx in (1, -1):
                end = [sx * 50.0, s * 24.3, cb + 9.0]
                for k in low:
                    if np.sign(N[k][0]) == sx and (abs(N[k][1]) < 1 or (np.sign(N[k][1]) == s and which != "D")) and (abs(N[k][0]) > 1):
                        if np.linalg.norm(N[k] - end) > 8: links.append((f"rail tie {k} {sx:+.0f}{s:+.0f}", end, N[k].tolist()))
    return rec, N, R, tag, ids, kinds, links, dz

# ---- catch checks (seat frame)
def faces(p, yd):
    a = math.radians(yd); x = p[0] * math.cos(a) - p[1] * math.sin(a); y = p[0] * math.sin(a) + p[1] * math.cos(a)
    return ((abs(x), EDGE_X, HALF_X), (abs(y), EDGE_Y, HALF_Y))

def capture_clear(p, rad):
    """margin (mm) of a sphere at seat point p over every landing within +-40 at +-5 deg yaw: inside the infinite face
    planes by 1.0, or above the mouth by 3, or (below the edges at 40 off) in the throat with 3 a side"""
    if p[2] - rad >= MOUTH_Z + 3: return p[2] - rad - MOUTH_Z - 3
    worst = 1e9
    for yd in (-YAW, 0, YAW):
        for v, edge, half in faces(p, yd):
            m = (FACE_K + p[2] - v) / S2 - rad - 1.0
            if p[2] + rad < edge - CAPTURE: m = max(m, half - CAPTURE - SIDE - v - rad)
            worst = min(worst, m)
    return worst

def seated_clear(p, rad):
    worst = 1e9
    for yd in (-YAW, 0, YAW):
        for v, edge, half in faces(p, yd):
            worst = min(worst, (FACE_K + p[2] - v) / S2 - rad - 1.0 if p[2] >= edge else half - v - rad - SIDE)
    return worst

def plunger_clear(p, rad):
    """G (7): extended plunger x 94..106, slope 50.1..58.1 from the +-y edge, 12 off the face; drone lift 0..5.7"""
    worst = 1e9
    for lift in (0.0, 2.85, LIFT):
        z = p[2] + lift
        for sy in (1, -1):
            v = sy * p[1]; s = ((v - HALF_Y) + (z - EDGE_Y)) / S2; n = (FACE_K + z - v) / S2
            for sx in (1, -1):
                dx = max(94 - sx * p[0], 0, sx * p[0] - 106); ds = max(50.1 - s, 0, s - 58.1); dn = max(-n, 0, n - 12)
                worst = min(worst, math.sqrt(dx * dx + ds * ds + dn * dn) - rad - 1.0)
    return worst

def probe_clear(p, rad):
    """G (8): nothing but the pads within 9.0 of the +-x faces over y -35..+69 (+x) / -69..+35 (-x) between the pins'
    lowest pass (probe line z 33 - 40 at 40 off) and the pad top (z 35.8)"""
    worst = 1e9
    for sx in (1, -1):
        if -35 <= sx * p[1] <= 69 and p[2] - rad < 33 + 4 / S2 and p[2] + rad > 33 - CAPTURE - 1.5:
            worst = min(worst, (FACE_K + p[2] - sx * p[0]) / S2 - rad - 9.0)
    return worst

def sample_parts(N, R, tag, kinds, links, rec, dz, which):
    """(seat point, radius, name, kind) for every non-wheel cage point, link, latch part and the drone body"""
    S = []
    near = lambda q, j, d: np.linalg.norm(q - N[j]) < d
    for i, p in enumerate(N):
        if tag[i] != "wheel node": S.append((p + [0, 0, dz], 4.0, f"node {i} {tag[i]}", tag[i]))
    for a, b in R:
        for t in np.linspace(0, 1, 41)[1:-1]:
            q = N[a] + t * (N[b] - N[a]); own = [j for j in (a, b) if tag[j] in ("wheel node", "pad") and near(q, j, 12)]
            S.append((q + [0, 0, dz], kinds[(a, b)] / 2, f"rod {a}-{b}", "pad" if any(tag[j] == "pad" for j in own) else ("wheel" if own else "rod")))
    for nm, p0, p1 in links:
        for t in np.linspace(0, 1, 21):
            S.append((np.add(p0, t * np.subtract(p1, p0)) + [0, 0, dz], 1.5, nm, "link"))
    boxes = [(nm, b) for nm, b in rec["boxes"]] + [("cartridge", cart_box(rec["cart_top"])), ("frame plate", PLATE), ("FC", FC)]
    for nm, b in boxes:
        for x in np.linspace(b[0], b[1], 7):
            for y in np.linspace(b[2], b[3], 5):
                for z in np.linspace(b[4], b[5], 5):
                    S.append((np.array([x, y, z + dz]), 0.0, nm, "body"))
    for nm, p0, p1, r in rec["struts"]:
        for t in np.linspace(0, 1, 11): S.append((np.add(p0, t * np.subtract(p1, p0)) + [0, 0, dz], r, nm, "body"))
    for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
        for a in np.linspace(0, 2 * math.pi, 24, endpoint=False):
            S.append((np.array([sx * MOTOR + PROP_R * math.cos(a), sy * MOTOR + PROP_R * math.sin(a), PROP_Z - SWEEP + dz]), 0.0, "prop", "body"))
    return S

def catch_checks(S, which):
    out = dict(capture=1e9, seated=1e9, plunger=1e9, probe=1e9, where={})
    for p, r, nm, kind in S:
        if kind == "wheel": continue
        m = dict(capture=capture_clear(p, r), seated=seated_clear(p, r) if p[2] - r < MOUTH_Z + 3 else 1e9,
                 plunger=plunger_clear(p, r), probe=probe_clear(p, r) if kind != "pad" else 1e9)
        for k, v in m.items():
            if v < out[k]: out[k] = v; out["where"][k] = nm
    for k in ("capture", "seated", "plunger", "probe"): out[k] = round(out[k], 1)
    out["pass"] = all(out[k] >= 0 for k in ("capture", "seated", "plunger", "probe"))
    return out

# ---- system_check manifest (concept geometry: boxes, rods and links as cylinders, nodes as spheres)
def manifest(name, N, R, tag, kinds, rec, links, ids):
    parts = [dict(name="frame plate", box=PLATE, role="structure", mass_g=28, tag="E"),
             dict(name="wiring, UBEC, XR2, camera, flow (lump)", box=[-30, 30, -20, 20, 22, 34], role="bought", mass_g=round(CORE_G - 176, 1), tag="E"),
             dict(name="FC MicoAir743", box=FC, role="protected", protect=True, mass_g=10, tag="P"),
             dict(name="Radxa Zero 3W (top)", box=[-32.5, 32.5, -15, 15, 23.0, 30.0], role="protected", protect=True, mass_g=10, tag="E"),
             dict(name="pack 4S 1500", box=pack_box(rec["z_cart_bottom"]), role="protected", protect=True, mass_g=165, tag="P", swap=True),
             dict(name="cartridge", box=cart_box(rec["cart_top"]), role="protected", protect=True, mass_g=CART[3], tag="M", swap=True)]
    for i, (sx, sy) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1))):
        c = [sx * MOTOR, sy * MOTOR]
        parts.append(dict(name=f"motor {i+1}", cylinder=dict(r=14, p0=c + [22.0], p1=c + [36.0]), role="protected", protect=True, mass_g=28, tag="E"))
        parts.append(dict(name=f"prop {i+1}", cylinder=dict(r=PROP_R, p0=c + [PROP_Z - SWEEP], p1=c + [PROP_Z + SWEEP]), role="protected", protect=True, mass_g=4, tag="P"))
        parts.append(dict(name=f"arm {i+1}", cylinder=dict(r=6, p0=[sx * 30, sy * 30, 20.25], p1=c + [20.25]), role="structure", mass_g=0, tag="E"))
    vol = [(b[1] - b[0]) * (b[3] - b[2]) * (b[5] - b[4]) for _, b in rec["boxes"]]
    for (nm, bx), v in zip(rec["boxes"], vol):
        parts.append(dict(name=nm, box=bx, role="structure", mass_g=round((rec["mass"] - 12.3) * v / sum(vol) + (12.3 if "XT60" in nm else 0), 2), tag=rec["tag"]))
    for nm, p0, p1, r in rec["struts"]:
        parts.append(dict(name=nm, cylinder=dict(r=r, p0=list(p0), p1=list(p1)), role="structure", mass_g=0, tag="E"))
    for i, p in enumerate(N):
        parts.append(dict(name=f"node {i} {tag[i]}", sphere=dict(r=5.0 if tag[i] == "wheel node" else 4.0, center=p.tolist()), role="cage", mass_g=node_g(tag[i]), tag="E"))
    for a, b in R:
        d = kinds[(a, b)]; L = float(np.linalg.norm(N[a] - N[b]))
        parts.append(dict(name=f"rod {a}-{b}", cylinder=dict(r=d / 2, p0=N[a].tolist(), p1=N[b].tolist()), role="cage",
                          mass_g=round(L / 1000 * (TUBE6_GPM if d == 6 else ROD3_GPM), 2), tag="E"))
    for nm, p0, p1 in links:
        parts.append(dict(name=nm, cylinder=dict(r=1.5, p0=list(p0), p1=list(p1)), role="cage", mass_g=round(np.linalg.norm(np.subtract(p1, p0)) / 1000 * ROD3_GPM, 2), tag="E"))
    names = [p["name"] for p in parts]
    contact = [[f"motor {i}", f"prop {i}", 1e9] for i in range(1, 5)] + [[f"arm {i}", f"motor {i}", 1e9] for i in range(1, 5)]
    contact += [[f"arm {i}", "frame plate", 1e9] for i in range(1, 5)] + [["pack 4S 1500", "cartridge", 1e9]]
    contact += [[nm, "pack 4S 1500", 1e9] for nm, _ in rec["boxes"] if "envelope" in nm]
    contact += [["wiring, UBEC, XR2, camera, flow (lump)", n2, 1e9] for n2 in names if not n2.startswith(("rod", "node", "spoke", "rail tie"))]
    for i, (a, b) in enumerate(R):                                    # rods meeting in one node
        for c_, d_ in R[i + 1:]:
            if {a, b} & {c_, d_}: contact.append([f"rod {a}-{b}", f"rod {c_}-{d_}", 1e9])
    for nm in names:                                                  # rods in their nodes
        if nm.startswith("rod "):
            a, b = nm[4:].split("-"); contact += [[nm, n2, 1e9] for n2 in names if n2.startswith(f"node {a} ") or n2.startswith(f"node {b} ")]
    latch = [nm for nm, _ in rec["boxes"]] + [nm for nm, *_ in rec["struts"]]
    for nm in latch:
        contact += [[nm, "frame plate", 1e9], [nm, "cartridge", 1e9]] + [[nm, n2, 1e9] for n2 in latch if n2 != nm]
    for nm, p0, p1 in links:                                          # links end in a node and an arm tip or a rail end
        contact += [[nm, n2, 1e9] for n2 in names if n2.startswith(("node ", "rod ", "arm ", "latch rail", "spoke", "rail tie", "rail post", "rail strut", "motor"))]
    return dict(name=name, machine="scout", parts=parts, contact_pairs=contact, cg_ref=[0, 0, PROP_Z], thrust_axis=[0, 0, 1],
                cg_tol_mm=3.0, cage_margin_mm=3.0, mass_budget_g=500, probes=False,
                openings=[dict(axis=[0, 0, -1], half_angle_deg=40.0, min_mm=0.0)],
                cage_shell=dict(nodes=N.tolist(), rods=[list(r) for r in R], open=[dict(ring=ids["opening"])], mirror="y", rod_r_mm=1.5))

# ---- the Overseer (layout v5 / cage v3 meshes) in the same funnel
def overseer_fit(cart_bottom_seat):
    """d = seat z - Overseer drone z. Its body (shell left out: it is redrawn round the catch) must clear the capture;
    the cartridge bottom (drone z -54) must sit where the Scout's does (one head stop)"""
    man = json.load(open(OVR)); body, swap = [], []
    for p in man["parts"]:
        if p["name"].startswith(("cage shell", "spokes", "L2 guard", "TPU feet")): continue
        r = load_part(p, os.path.dirname(OVR)); m = (r[0] if isinstance(r, tuple) else r).copy()
        if "pose" in p: m.apply_transform(pose_matrix(p["pose"]))
        pts = np.vstack([m.vertices, trimesh.sample.sample_surface(m, 3000, seed=1)[0]])
        (swap if p["name"] in ("pack", "cartridge", "receiver") else body).append((p["name"], pts, m.bounds.tolist()))
    def worst(group, d):
        out = []
        for nm, P, _ in group:
            Q = np.c_[P[:, 0] - OVR_XC, P[:, 1], P[:, 2] + d]
            out.append((round(min(capture_clear(q, 0.0) for q in Q[::7]), 1), nm))
        return sorted(out)
    def dmin(group):
        for d in np.arange(-20, 200, 0.5):
            if all(capture_clear(q, 0.0) >= 0 for _, P, _ in group for q in np.c_[P[::7, 0] - OVR_XC, P[::7, 1], P[::7, 2] + d]): return float(d)
    cb = man and -54.0
    sh = json.load(open(os.path.join(REPO, "runs", "overseer_layout", "v5", "cage_shell.json")))
    d_need = cart_bottom_seat - cb
    Ns = np.array(sh["nodes"])
    def shell_low(d):
        Q = np.c_[Ns[:, 0] - OVR_XC, Ns[:, 1], Ns[:, 2] + d]
        bad = [i for i, q in enumerate(Q) if capture_clear(q, 4.0) < 0]
        return dict(nodes_out=len(bad), of=len(Ns), max_rise=round(max([MOUTH_Z + 7 - Q[i][2] for i in bad] or [0]), 1))
    return dict(d_body=dmin(body), d_swap=dmin(swap), cart_bottom_drone=cb, d_for_shared=round(d_need, 1),
                worst_at_shared=worst(body, d_need)[:4], shell_at_shared=shell_low(d_need),
                parts={nm: dict(bounds=b) for nm, _, b in body + swap}, shell=dict(nodes=sh["nodes"], rods=sh["rods"]))

def main():
    res = {}
    for which in ("A", "B", "C", "D"):
        rec, N, R, tag, ids, kinds, links, dz = layout(which)
        sm = shell_mass(N, R, tag, links)
        S = sample_parts(N, R, tag, kinds, links, rec, dz, which)
        cc = catch_checks(S, which) if which != "D" else None          # D: catch_posts_concept.py
        man = manifest(f"Scout v3 bottom concept {which}", N, R, tag, kinds, rec, links, ids)
        sc = run(man, probes=False)
        json.dump(man, open(os.path.join(OUT, f"manifest_{which}.json"), "w"))
        json.dump(sc, open(os.path.join(OUT, f"system_check_{which}.json"), "w"), default=str, indent=1)
        zlo = float(min(N[:, 2].min() - 5.0, rec["z_cart_bottom"])); zhi = float(N[:, 2].max() + 4.0)
        drone_g = CORE_G + PACK[3] + CART[3] + rec["mass"] + sm["g"]
        res[which] = dict(receiver=dict(boxes=rec["boxes"], struts=rec["struts"], mass=rec["mass"], tag=rec["tag"], parts=rec.get("parts")),
                          cart=cart_box(rec["cart_top"]), pack=pack_box(rec["z_cart_bottom"]), dz=round(dz, 1),
                          nodes=N.tolist(), rods=[[int(a), int(b), kinds[(a, b)]] for a, b in R], links=[[p, q] for _, p, q in links],
                          tags=tag, ids={k: v for k, v in ids.items() if k != "rings"}, rings=ids["rings"],
                          shell=sm, catch=cc, system_check=sc["summary"], system_pass=bool(sc["pass"]),
                          z_low=round(zlo, 1), z_high=round(zhi, 1), height=round(zhi - zlo, 1), below_plate=round(PLATE[4] - zlo, 1),
                          opening=dict(x=round(2 * float(np.max(N[ids["opening"], 0])) - 8, 1), y=round(2 * float(np.max(N[ids["opening"], 1])) - 8, 1)),
                          swap_on_drone_g=round(CART[3] + rec["mass"], 1), drone_g=round(drone_g, 1),
                          pack_cz=round((pack_box(rec["z_cart_bottom"])[4] + pack_box(rec["z_cart_bottom"])[5]) / 2, 1),
                          cart_bottom_seat=round(rec["z_cart_bottom"] + dz, 1))
    a = res["A"]
    for w in ("B", "C", "D"):
        r = res[w]
        r["vs_A"] = dict(mass_g=round(r["drone_g"] - a["drone_g"], 1), height=round(r["height"] - a["height"], 1),
                         hover_power_pct=round(((r["drone_g"] / a["drone_g"]) ** 1.5 - 1) * 100, 1),
                         head_stop=round(r["cart_bottom_seat"] - a["cart_bottom_seat"], 1))
    res["overseer"] = dict(B=overseer_fit(res["B"]["cart_bottom_seat"]), C=overseer_fit(res["C"]["cart_bottom_seat"]))
    res["numbers"] = dict(Z_WHEEL=round(Z_WHEEL, 2), CORE_G=CORE_G, FC=FC, PLATE=PLATE, face_k=FACE_K, mouth=[MOUTH_Z, MOUTH_H],
                          edges=[EDGE_Y, HALF_Y, EDGE_X, HALF_X], motor=MOTOR, prop_r=PROP_R, prop_z=PROP_Z, ovr_xc=OVR_XC)
    enc = lambda o: o.tolist() if hasattr(o, "tolist") else float(o) if isinstance(o, np.floating) else str(o)
    json.dump(res, open(os.path.join(OUT, "concept.json"), "w"), default=enc)
    for w in ("A", "B", "C", "D"):
        r = res[w]
        print(f"{w}: h {r['height']} (z {r['z_low']}..{r['z_high']}), swap parts {r['swap_on_drone_g']} g, shell {r['shell']}, drone {r['drone_g']} g E, "
              f"cart bottom seat {r['cart_bottom_seat']}, opening {r['opening']}\n   catch {r['catch']}\n   {r['system_check']}")
    for w in ("B", "C", "D"): print(w, "vs A", res[w]["vs_A"])
    for w in ("B", "C"):
        o = res["overseer"][w]; print(f"Overseer with {w}: d_body {o['d_body']}, d_swap {o['d_swap']}, d for the shared head stop {o['d_for_shared']}, worst {o['worst_at_shared']}, shell {o['shell_at_shared']}")

if __name__ == "__main__":
    main()
