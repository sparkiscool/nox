"""Ramp catch (concept E, 27 Oct, Spark: "lets do no rim, just make it so it clears all current cages, the scout one will
have no trapezoids at either end and just be the rectangle ... 2 sides need ramps ... wide enough for a big chunk of the
whole side not getting in the way of the nodes ... the overseer will have 4 ... make sure all ramps clear the cages its
okay if its short just needs to be long enough for it to center"; "give the scout one 4 as well ... make sure it clears for
the overseer if those are shorter thats okay then tell me the error tolerance we get"). Concept check only; the box
catch session owns the catch.

The drones keep their cages; each lands with its bottom opening ring on 45 deg ramps that rise into the opening:
  side ramps (2, shared, each two pads x +-(12..55)): under the opening's long sides (y +-104.3), clear of the corner
  nodes and the Scout's side mid node; 7 high (the Overseer's belly sled and pods at y +-72, 9 above the ring);
  Overseer end ramps (2): under its trapezoid tips (x +-176 about the catch centre), y +-25, clear of the tip nodes;
  Scout end ramps (2, each two pads beside the Overseer's receiver): under the Scout rectangle's short ends (x +-72.9),
  y +-(54..88), clear of the Overseer's receiver and TPU feet, low enough to pass under it.
Seat frame: z 0 = the ring tube centres; both cartridges sit 3.0 above them (one head stop). A ramp's capture along its
axis is about its height. The check drops each drone (Scout layout E, nox/scout/v3/bottom_concept.py; the Overseer
layout v5 with its cage v3 as it is) at offsets along x, y and the diagonals, yaw -5 / 0 / +5: the first contact must be
its ring on a ramp's slope (pushing it back), then it slides to the seat with everything else 1.0 off every ramp.
python3 nox/box/catch_ramps_concept.py   (writes runs/box_catch/ramps_concept/result.json and the artifact)
"""
import os, sys, json, math, numpy as np, trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, "nox")); sys.path.insert(0, os.path.join(REPO, "nox", "scout", "v3"))
import bottom_concept as bc
from system_check import load_part, pose_matrix
OUT = os.path.join(REPO, "runs", "box_catch", "ramps_concept"); os.makedirs(OUT, exist_ok=True)

RX, RY = bc.RECT_E                     # C: middle rectangle of the Overseer's opening (node centres), about its catch centre
TIP_X = 176.0                          # C: its trapezoid tips (drone x 146 / -206, catch centre -30)
TUBE_R, CLEAR, BOTTOM = 3.0, 1.0, -25.0
H_SIDE = float(os.environ.get("RAMP_SIDE_H", 25.0))       # ramp heights above the seat (= capture along that axis, 45 deg)
H_OEND = float(os.environ.get("RAMP_OEND_H", 25.0))
H_SEND = float(os.environ.get("RAMP_SEND_H", 20.0))
SEND_Y = (0, 0)                                            # unused (per end below)
SEND_NOSE = (52.0, 96.0)    # Scout nose pads |y|: outside the Overseer's nose TPU feet (|y| 39 + yaw + capture), inside its side nodes
SEND_TAIL = (52.0, 96.0)    # Scout tail pads |y|: outside the Overseer's tail TPU feet, inside the corner nodes (the Scout's ties left the band in v10)
TIP_LEN = 50.0              # Overseer nose V: 30 + 50 along each tip edge
TIP_LEN_TAIL = float(os.environ.get("TIP_LEN_TAIL", 25.0))   # Overseer tail V: 30 + 25 only, so it stays behind the Scout's tail low node and its rods
SLED_Y = float(os.environ.get("OVR_SLED_Y", 0.0))         # 0 = the Overseer's belly sled and pods as they are
ALT_SIDE, ALT_SLED_Y = 35.0, 46.0                          # what-if: taller side ramps with the Overseer's belly pods inside |y| 46
S2 = math.sqrt(2)

def wedge(A, B, c, s0, s1, h, name, v_tip=None):
    """45 deg ramp under the ring tube from A to B (plan, seat frame; tube centre at z 0), rising inward (away from the
    plan centre c); along the edge from s0 to s1 mm from A; half-spaces n.p <= d"""
    A, B, c = (np.array(v, float) for v in (A, B, c))
    t2 = (B - A) / np.linalg.norm(B - A); e2 = np.array([t2[1], -t2[0]])
    if e2 @ (A - c) < 0: e2 = -e2                             # outward
    e = np.array([e2[0], e2[1], 0.0]); t = np.array([t2[0], t2[1], 0.0])
    k = e2 @ A - TUBE_R * S2                                   # slope plane: e.p + z = k (touches a seated r-3 tube)
    H = [((e + [0, 0, 1]) / S2, k / S2), (np.array([0, 0, 1.0]), h), (-e, -(k - h)), (e, k - BOTTOM),
         (np.array([0, 0, -1.0]), -BOTTOM),
         ((t + [0, 0, 1]) / S2, (t2 @ A + s1) / S2), (t, t2 @ A + s1 - BOTTOM)]                      # 45 deg end chamfer (top edge to s1)
    if v_tip is None:
        H += [((-t + [0, 0, 1]) / S2, (-(t2 @ A + s0)) / S2), (-t, -(t2 @ A + s0) - BOTTOM)]
    else:                                                    # one half of a V under a tip: meets its mirror on y 0, cut back from the tip
        side, xcut, sx = v_tip                               # side +1 / -1 (its y), x of the cut, +1 front / -1 tail
        H += [(np.array([0, -side, 0.0]), 0.0), (np.array([sx, 0, 1.0]) / S2, sx * xcut / S2), (np.array([sx, 0, 0.0]), sx * xcut - BOTTOM)]
    return dict(name=name, planes=[(n.tolist(), float(d)) for n, d in H], A=A.tolist(), B=B.tolist(), e=e2.tolist(), t=t2.tolist(),
                k=float(k), span=[s0, s1], h=h)

# the current Overseer base loop (cage v5, PR 122, confirmed 27 Oct) about the swap centre (x -30), seat z 0 = the loop plane
HEX = dict(Ln=(85.8, 116.1), Rn=(85.8, -116.1), Lt=(-68.9, 119.7), Rt=(-68.9, -119.7), F=(191.2, 0.0), T=(-212.6, 0.0))
INSET = 20.0                                                     # clear of the r 10.15 nodes by ~10 at both ends
TIP_INSET = 30.0                                                 # the V under each tip is cut back 30 (in x) from the tip node

def ramps(h_side=None, h_tip=None, h_send=None, send_y=None):
    h_side = H_SIDE if h_side is None else h_side; h_tip = H_OEND if h_tip is None else h_tip; h_send = H_SEND if h_send is None else h_send
    y0, y1 = SEND_Y if send_y is None else send_y
    X = HEX; c = (0.0, 0.0); R = []
    for a, b, nm in (("Lt", "Ln", "side +y"), ("Rt", "Rn", "side -y")):            # shared: the long sides
        L = float(np.linalg.norm(np.subtract(X[b], X[a]))); R.append(wedge(X[a], X[b], c, INSET, L - INSET, h_side, nm))
    for a, b, nm in (("F", "Ln", "Overseer tip +x +y"), ("F", "Rn", "Overseer tip +x -y"), ("T", "Lt", "Overseer tip -x +y"), ("T", "Rt", "Overseer tip -x -y")):
        sx = 1 if a == "F" else -1; side = 1 if b.startswith("L") else -1
        R.append(wedge(X[a], X[b], c, 0.0, TIP_INSET + (TIP_LEN if sx > 0 else TIP_LEN_TAIL), h_tip, nm, v_tip=(side, X[a][0] - sx * TIP_INSET, sx)))
        # a V under each tip, its two halves joined on the centreline, cut back TIP_INSET from the tip node, TIP_LEN + TIP_INSET along each tip edge
    for a, b, nm, (y0, y1) in (("Rn", "Ln", "Scout end +x", SEND_NOSE), ("Rt", "Lt", "Scout end -x", SEND_TAIL)):   # the Scout's short ends
        A, B = X[a], X[b]; L = float(np.linalg.norm(np.subtract(B, A)))
        for lo, hi in ((L / 2 + y0, L / 2 + y1), (L / 2 - y1, L / 2 - y0)):
            R.append(wedge(A, B, c, lo, hi, h_send, f"{nm} {'+' if lo > L / 2 else '-'}y"))
    return R

def sdf(Q, R):
    """signed distance-ish (max plane distance: exact inside and on faces, conservative near edges) and the face index"""
    best = np.full(len(Q), 1e9); face = np.zeros((len(Q), 3)); which = np.full(len(Q), -1)
    for j, r in enumerate(R):
        N = np.array([n for n, _ in r["planes"]]); d = np.array([d for _, d in r["planes"]])
        v = Q @ N.T - d; m = v.max(1); a = v.argmax(1)
        upd = m < best; best[upd] = m[upd]; face[upd] = N[a[upd]]; which[upd] = j
    return best, face, which

def landing(P, rad, ring, R, off, yd, names=None):
    """drop at offset off (x, y) and yaw yd. Going down, contacts may hold it (a side ramp squaring the yaw) but every
    contact is its ring and none pushes outward, until a ring contact pushes it back along the offset (>= 0.3 of the
    push); from there it slides straight to the seat with everything else 1.0 off every ramp. Returns (ok, why, clear)"""
    y = math.radians(yd); Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    Q0 = P @ Rz.T + [off[0], off[1], 0]
    o = np.array([off[0], off[1]], float); u = o / np.linalg.norm(o) if np.linalg.norm(o) > 0.5 else None
    nm = lambda idx: sorted({names[i] for i in idx}) if names else "?"
    Hr = None
    for H in np.arange(60.0, -0.01, -0.5):
        dist, face, which = sdf(Q0 + [0, 0, H], R)
        c = dist - rad <= 0
        if not c.any(): continue
        bad = np.where(c & ~ring)[0]
        if len(bad): return False, f"{nm(bad)} meets {sorted({R[which[i]]['name'] for i in bad})} before it is centred", -1
        if u is None: Hr = H; break
        push = face[c][:, :2] @ u
        net = float(push.mean())                                   # all contacts together (several touch at once under yaw)
        if net > 0.1: return False, "the ramps push the ring outward (landed past a ramp top)", -1
        if (-push).max() >= 0.3 and net < 0: Hr = H; break
    if Hr is None: return False, "nothing pushes it back along the offset (it would sit off centre)", -1
    worst, who = 1e9, ""
    for s in np.linspace(0, 1, 33):
        y2 = math.radians(yd * (1 - s)); R2 = np.array([[math.cos(y2), -math.sin(y2), 0], [math.sin(y2), math.cos(y2), 0], [0, 0, 1]])
        Q = P[~ring] @ R2.T + [off[0] * (1 - s), off[1] * (1 - s), Hr * (1 - s)]
        dist, _, w = sdf(Q, R); m = dist - rad[~ring] - CLEAR
        if m.min() < worst:
            worst = float(m.min()); j = int(m.argmin())
            who = (np.array(names, object)[~ring][j] if names else "?") + " on " + R[w[j]]["name"]
    return worst >= 0, ("ok" if worst >= 0 else f"{who} meets a ramp on the slide ({worst:.1f})"), round(worst, 1)

def capture(d, R, cs=(5, 10, 15, 20, 25, 30, 35, 40)):
    """largest offset along x, along y and on the diagonals that lands at yaw -5 / 0 / +5"""
    out = {}
    for name, dirs in (("x", [(1, 0), (-1, 0)]), ("y", [(0, 1), (0, -1)]), ("diag", [(1, 1), (-1, 1), (-1, -1), (1, -1)])):
        best, why = 0, None
        for c in cs:
            ok_all = True
            for dx, dy in dirs:
                u = np.array([dx, dy], float); u /= np.linalg.norm(u)
                for yd in (-5.0, 0.0, 5.0):
                    ok, w, _ = landing(d["P"], d["rad"], d["ring"], R, u * c, yd, d.get("names"))
                    if not ok: ok_all = False; why = f"{c} along {name}: {w}"; break
                if not ok_all: break
            if not ok_all: break
            best = c
        out[name] = dict(ok_mm=best, first_fail=why)
    return out

def seated(d, R):
    dist, _, which = sdf(d["P"], R)
    m = dist - d["rad"]
    return round(float((m[~d["ring"]] - CLEAR).min()), 1), round(float(m[d["ring"]].min()), 2)

# ---- the drones in the seat frame
def tube_points(N, rods, r, ring_ids, every=3.0):
    P, rad, ring = [], [], []
    tube_points.names = []
    for a, b in rods:
        L = np.linalg.norm(N[b] - N[a]); n = max(int(L / every), 2)
        for t in np.linspace(0, 1, n + 1)[1:-1]:
            P.append(N[a] + t * (N[b] - N[a])); rad.append(r(a, b)); ring.append(a in ring_ids and b in ring_ids)
            tube_points.names.append(f"cage tube {a}-{b}")
    return P, rad, ring

SCOUT_REF = os.environ.get("SCOUT_REF", "c5b57903")            # Scout v3 layout v10 (PR 131 head: bottom on the Overseer loop's side nodes, ties out of the band, plane -49.62)
SCOUT_DIR = "runs/scout_layout/v10"
SCOUT_CATCH_X, SCOUT_PLANE_Z = -7.5, -49.62                      # C (Scout session): catch (tab line) x, bottom plane z as pushed; cartridge flat bottom -44.5
SCOUT_PLANE_FIX = -49.62                                         # the plane that seats its r 1.5 ring like the Overseer's r 3 (cartridge -3 -2.12; asked of the Scout session)
OVR_REF = os.environ.get("OVR_REF", "c65d96a")                   # Overseer cage v5 (PR 122 head; loop unchanged since 20fbb52e, confirmed 27 Oct)
OVR_CATCH_X, OVR_PLANE_Z = -30.0, -57.0                          # C (Overseer session): swap centre x, loop plane (cartridge bottom -54)

def git_bytes(ref, path):
    import subprocess
    return subprocess.check_output(["git", "-C", REPO, "show", f"{ref}:{path}"])

def load_drone(ref, assembly, shell, off, skip=("cage shell",), move=None, ring_r=None):
    """one drone from a git ref in the seat frame: the cage graph (ring = its bottom opening) and every part mesh"""
    man = json.loads(git_bytes(ref, assembly)); sh = json.loads(git_bytes(ref, shell))
    N = np.array(sh["nodes"], float); ring_ids = set(sh["open"][0]["ring"])
    if move:
        for i, p in move.items(): N[i] = p
    N = N + off
    rr = sh["rod_r_mm"]
    P, rad, ring = tube_points(N, sh["rods"], lambda a, b: rr, ring_ids); names = list(tube_points.names)
    for i, p in enumerate(N):
        P.append(p); rad.append(10.15 if i in ring_ids else sh["node_r_mm"]); ring.append(i in ring_ids); names.append(f"cage node {i}")
    meshes = []
    for p in man["parts"]:
        if p["name"].startswith(skip): continue
        if "mesh" in p: m = trimesh.load(trimesh.util.wrap_as_stream(git_bytes(ref, p["mesh"])), file_type="stl")
        else:
            r = load_part(p, REPO); m = r[0] if isinstance(r, tuple) else r
        m = m.copy()
        if "pose" in p: m.apply_transform(pose_matrix(p["pose"]))
        m.apply_translation(off)
        Q = np.vstack([m.vertices, trimesh.sample.sample_surface(m, 1500, seed=4)[0]]); Q = Q[Q[:, 2] < 70]
        if SLED_Y and p["name"].startswith(("belly sled", "flow")):  # what-if: the belly pods inside |y| SLED_Y
            Q = Q.copy(); Q[:, 1] = np.clip(Q[:, 1], -SLED_Y, SLED_Y)
        P.extend(Q); rad.extend([0.0] * len(Q)); ring.extend([False] * len(Q)); names.extend([p["name"]] * len(Q))
        dm = m.simplify_quadric_decimation(face_count=300) if len(m.faces) > 300 else m
        meshes.append(dict(name=p["name"], v=dm.vertices.round(1).tolist(), f=dm.faces.tolist()))
    return dict(P=np.array(P), rad=np.array(rad), ring=np.array(ring), names=names,
                draw=dict(nodes=N.round(1).tolist(), rods=sh["rods"], ring=sorted(ring_ids), meshes=meshes, rod_r=rr)), rr

def scout(plane=None):
    """the Scout (PR 131) seated on the ramps: its r 1.5 ring sits (3 - 1.5) * sqrt 2 below where an r 3 tube would; its
    plane is modelled at SCOUT_PLANE_FIX (the cartridge then lands where the Overseer's does) unless plane= is given"""
    sh = json.loads(git_bytes(SCOUT_REF, f"{SCOUT_DIR}/cage_shell.json")); drop = (TUBE_R - sh["rod_r_mm"]) * S2
    plane = SCOUT_PLANE_FIX if plane is None else plane
    # only the 4 ring nodes move to `plane`; everything else keeps its drone-frame place
    move = {i: [sh["nodes"][i][0], sh["nodes"][i][1], plane] for i in sh["open"][0]["ring"]}
    off = np.array([-SCOUT_CATCH_X, 0.0, -plane - drop])
    d, rr = load_drone(SCOUT_REF, "nox/scout/v3/scout.assembly.json", f"{SCOUT_DIR}/cage_shell.json", off, move=move)
    d.update(dz=off[2], cart_bottom_seat=round(-44.5 + off[2], 2), seat_drop=round(drop, 2), plane=plane)
    return d

def overseer():
    off = np.array([-OVR_CATCH_X, 0.0, -OVR_PLANE_Z])
    d, rr = load_drone(OVR_REF, "nox/overseer/overseer.assembly.json", "nox/overseer/cage_v5_shell.json", off)
    d.update(dz=off[2], cart_bottom_seat=round(-54.0 + off[2], 2))
    return d

def wedge_mesh(r):
    """6 corners for drawing: per end [top inner, bottom outer, bottom inner]"""
    A, e, t, k, (s0, s1), h = np.array(r["A"]), np.array(r["e"]), np.array(r["t"]), r["k"], r["span"], r["h"]
    pts = []
    for sv in (s0, s1):
        base = A + t * sv - e * (e @ A)
        for (u, z) in ((k - h, h), (k - BOTTOM, BOTTOM), (k - h, BOTTOM)):
            q = base + e * u; pts.append([float(q[0]), float(q[1]), z])
    return pts

def run_config(h_side, h_oend, h_send, sled_y=0.0):
    global SLED_Y
    SLED_Y = sled_y
    R = ramps(h_side, h_oend, h_send)
    out = dict(heights=dict(side=h_side, overseer_end=h_oend, scout_end=h_send, overseer_pods_y=sled_y or None),
               ramps=[dict(r, corners=wedge_mesh(r)) for r in R])
    for name, f in (("scout", scout), ("overseer", overseer)):
        d = f()
        out[name] = dict(seated_clear=seated(d, R), cart_bottom_seat=d["cart_bottom_seat"], capture=capture(d, R), draw=d["draw"])
        print(name, out[name]["seated_clear"], out[name]["capture"])
    return out

def main():
    res = run_config(H_SIDE, H_OEND, H_SEND)                       # DEFAULT: every cage and part as it is
    alt = run_config(ALT_SIDE, H_OEND, H_SEND, sled_y=ALT_SLED_Y)   # what-if: the Overseer's belly pods inside |y| ALT_SLED_Y
    res["whatif"] = [dict(label=f"Scout, side ramps {ALT_SIDE:.0f}", capture=alt["scout"]["capture"]),
                     dict(label=f"Overseer, belly pods inside |y| {ALT_SLED_Y:.0f}, side ramps {ALT_SIDE:.0f}", capture=alt["overseer"]["capture"])]
    enc = lambda o: o.tolist() if hasattr(o, "tolist") else float(o)
    json.dump(res, open(os.path.join(OUT, "result.json"), "w"), default=enc)
    build_html(res)

def build_html(res=None):
    """the artifact from result.json: text from the numbers"""
    res = res or json.load(open(os.path.join(OUT, "result.json")))
    S_, O_ = res["scout"], res["overseer"]; m = lambda d: min(d["capture"][k]["ok_mm"] for k in ("x", "y", "diag"))
    hs = res["heights"]; wi = {w["label"].split(",")[0]: w for w in res.get("whatif", [])}
    mw = lambda w: min(w["capture"][k]["ok_mm"] for k in ("x", "y", "diag"))
    res["lede"] = (f"Built on the cages the two sessions confirmed: Overseer cage v5 (PR 122, hexagon bottom) and Scout cage v9 (PR 127) "
                   f"with its 4 bottom nodes moved onto the hexagon's side nodes, so the Scout's opening is the middle of the Overseer's. "
                   f"Ramps rise into the opening: 2 shared side ramps, 4 Overseer ramps near its tips and 4 small Scout pads at its ends. "
                   f"Landing tolerance today: Scout ±{m(S_)} mm, Overseer ±{m(O_)} mm.")
    res["notes"] = [
        f"Ramp heights above the seat (a 45° ramp catches about its own height): side {hs['side']:.0f}, Overseer tips {hs['overseer_end']:.0f}, Scout pads {hs['scout_end']:.0f}. Side ramps run the long sides 20 clear of each node; the Overseer ramps sit on the tip edges, 20 to 75 from the tip; the Scout pads sit beside the Overseer's receiver and feet at the nose (|y| 40..96) and between the Scout's tail-bay ties and its corner nodes at the tail (|y| 92.5..99.5).",
        "What limits the Scout: at the tail its own tail-bay ties sit at ring height (x ≈ −60, |y| 50..91), and the Overseer's TPU feet and receiver fill the middle, so its tail pads are 7 mm wide slivers. Its nose and tail tubes also run low over where the Overseer's ramps would continue, so those stop near the tips.",
        f"With the Overseer's belly pods inside |y| {ALT_SLED_Y:.0f} the side ramps can be {ALT_SIDE:.0f} tall: see the last two table rows.",
        "Seat height: the Scout's ring rods are 3 mm, the Overseer's 6 mm, so on the same ramps the Scout sits 2.1 mm lower and its cartridge is 2.1 mm below the Overseer's at the swap. A 6 mm Scout ring (or its plane 2.1 higher) removes it (asked of the Scout session).",
        "Box: the ramps stand on a deck round the swap column. The hold (a latch over a ring tube) and keep-alive (pins on ring pads) move to the ramp bases. Still open: low-friction ramp faces (PTFE, μ ≤ 0.2) and the drop sim with tilt and drift.",
    ]
    bcj = json.load(open(os.path.join(REPO, "runs", "scout_bottom", "concept", "concept.json")))
    tpl = open(os.path.join(REPO, "nox", "box", "catch_ramps_view.html")).read()
    enc = lambda o: o.tolist() if hasattr(o, "tolist") else float(o)
    html = tpl.replace("/*DATA*/null", json.dumps(dict(bc=bcj, ramps=res), default=enc))
    open(os.path.join(OUT, "ramps.html"), "w").write(html)
    open(os.path.join(REPO, "runs", "scout_bottom", "concept", "concept.html"), "w").write(html)   # the published artifact

if __name__ == "__main__":
    build_html() if "--html" in sys.argv else main()
