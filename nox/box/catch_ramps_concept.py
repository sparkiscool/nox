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
H_SIDE = float(os.environ.get("RAMP_SIDE_H", 7.0))       # ramp heights above the seat (= capture along that axis, 45 deg)
H_OEND = float(os.environ.get("RAMP_OEND_H", 30.0))
H_SEND = float(os.environ.get("RAMP_SEND_H", 8.0))
SLED_Y = float(os.environ.get("OVR_SLED_Y", 0.0))         # 0 = the Overseer's belly sled and pods as they are
ALT_SIDE, ALT_SLED_Y = 26.0, 46.0                          # what-if: taller side ramps with the Overseer's pods pulled in
S2 = math.sqrt(2)

def wedge(axis, sign, c_line, span, h, name):
    """45 deg ramp under a ring tube along the other axis at axis-coordinate sign*c_line (tube centre), rising toward
    the centre; span = (lo, hi) along the tube; half-spaces n.p <= d"""
    e = np.zeros(3); e[axis] = sign                          # outward horizontal
    t = np.zeros(3); t[1 - axis] = 1.0                       # along the tube
    k = c_line - TUBE_R * S2                                 # slope plane: e.p + z = k (surface touching the seated tube)
    H = [((e + [0, 0, 1]) / S2, k / S2),                     # slope (outside = above it)
         (np.array([0, 0, 1.0]), h),                         # top
         (-e, -(k - h)),                                     # back face (inner vertical)
         (e, k - BOTTOM),                                    # outer bound at the bottom
         (np.array([0, 0, -1.0]), -BOTTOM),
         (t, span[1]), (-t, -span[0])]
    return dict(name=name, planes=[(n.tolist(), float(d)) for n, d in H], axis=axis, sign=sign, line=c_line, span=list(span), h=h)

def ramps(h_side=H_SIDE, h_oend=H_OEND, h_send=H_SEND):
    R = [wedge(1, s, RY, span, h_side, f"side {'+' if s > 0 else '-'}y {'+' if span[0] > 0 else '-'}x")   # two pads a side: clear of
         for s in (1, -1) for span in ((12, 55), (-55, -12))]                                                # the corners and the Scout's mid node
    R += [wedge(0, s, TIP_X, (-25, 25), h_oend, f"Overseer end {'+' if s > 0 else '-'}x") for s in (1, -1)]
    for s in (1, -1):
        for lo, hi in ((54, 88), (-88, -54)):              # beside the Overseer's receiver and its TPU feet (|y| <= 39 + yaw + capture)
            R.append(wedge(0, s, RX, (lo, hi), h_send, f"Scout end {'+' if s > 0 else '-'}x {'+' if lo > 0 else '-'}y"))
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
        if (push > 1e-6).any(): return False, "a ramp pushes the ring outward (landed past a ramp top)", -1
        if (-push).max() >= 0.3: Hr = H; break
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

SCOUT_REF = os.environ.get("SCOUT_REF", "64d6210")             # Scout v3 layout v9 / cage v2 (PR 127 head, confirmed by the Scout session 27 Oct)
SCOUT_CATCH_X, SCOUT_PLANE_Z = -8.5, -47.5                       # C (Scout session): catch (tab line) x, bottom plane z; cartridge flat bottom -44.5

def git_bytes(ref, path):
    import subprocess
    return subprocess.check_output(["git", "-C", REPO, "show", f"{ref}:{path}"])

def scout():
    """the Scout as its session has it (PR 127 head): cage graph + every part mesh, seat frame = catch centre, z 0 at the
    bottom plane; its ring rods are r 1.5 (the ramps are set for the Overseer's r 3), so it seats (3 - 1.5) * sqrt 2 lower"""
    man = json.loads(git_bytes(SCOUT_REF, "nox/scout/v3/scout.assembly.json"))
    sh = json.loads(git_bytes(SCOUT_REF, "runs/scout_layout/v9/cage_shell.json"))
    drop = (TUBE_R - sh["rod_r_mm"]) * S2
    off = np.array([-SCOUT_CATCH_X, 0.0, -SCOUT_PLANE_Z - drop])
    N = np.array(sh["nodes"], float) + off; ring_ids = set(sh["open"][0]["ring"])
    P, rad, ring = tube_points(N, sh["rods"], lambda a, b: sh["rod_r_mm"], ring_ids); names = list(tube_points.names)
    for i, p in enumerate(N):
        P.append(p); rad.append(10.15 if i in ring_ids else sh["node_r_mm"]); ring.append(i in ring_ids); names.append(f"cage node {i}")
    meshes, parts = [], {}
    for p in man["parts"]:
        if p["name"].startswith(("cage shell",)): continue
        if "mesh" in p:
            m = trimesh.load(trimesh.util.wrap_as_stream(git_bytes(SCOUT_REF, p["mesh"])), file_type="stl")
        else:
            r = load_part(p, REPO); m = r[0] if isinstance(r, tuple) else r
        m = m.copy()
        if "pose" in p: m.apply_transform(pose_matrix(p["pose"]))
        m.apply_translation(off)
        Q = np.vstack([m.vertices, trimesh.sample.sample_surface(m, 1500, seed=4)[0]]); Q = Q[Q[:, 2] < 70]
        P.extend(Q); rad.extend([0.0] * len(Q)); ring.extend([False] * len(Q)); names.extend([p["name"]] * len(Q))
        dm = m.simplify_quadric_decimation(face_count=300) if len(m.faces) > 300 else m
        meshes.append(dict(name=p["name"], v=dm.vertices.round(1).tolist(), f=dm.faces.tolist())); parts[p["name"]] = m.bounds.tolist()
    cb = -44.5 + off[2]
    return dict(P=np.array(P), rad=np.array(rad), ring=np.array(ring), names=names, dz=off[2], cart_bottom_seat=round(cb, 2), seat_drop=round(drop, 2),
                draw=dict(nodes=N.round(1).tolist(), rods=sh["rods"], ring=sorted(ring_ids), meshes=meshes, node_r=sh["node_r_mm"]))

def overseer():
    man = json.load(open(bc.OVR)); sh = json.load(open(os.path.join(REPO, "runs", "overseer_layout", "v5", "cage_shell.json")))
    ringz = -57.0; dz = -ringz; xc = bc.OVR_XC                      # C: its opening ring plane; cartridge bottom 3.0 above
    N = np.array(sh["nodes"]) + [-xc, 0, dz]; ring_ids = set(sh["open"][0]["ring"])
    P, rad, ring = tube_points(N, sh["rods"], lambda a, b: 3.0, ring_ids); names = list(tube_points.names)
    for i, p in enumerate(N):
        P.append(p); rad.append(10.15); ring.append(i in ring_ids); names.append(f"cage node {i}")
    meshes = []
    for p in man["parts"]:
        if p["name"].startswith(("cage shell",)): continue
        r = load_part(p, os.path.dirname(bc.OVR)); m = (r[0] if isinstance(r, tuple) else r).copy()
        if "pose" in p: m.apply_transform(pose_matrix(p["pose"]))
        m.apply_translation([-xc, 0, dz])
        Q = np.vstack([m.vertices, trimesh.sample.sample_surface(m, 1500, seed=3)[0]]); Q = Q[Q[:, 2] < 70]
        if SLED_Y and p["name"].startswith(("belly sled", "down camera", "flow")):   # what-if: the belly pods pulled in to |y| <= SLED_Y
            Q = Q.copy(); Q[:, 1] = np.clip(Q[:, 1], -SLED_Y, SLED_Y)
        P.extend(Q); rad.extend([0.0] * len(Q)); ring.extend([False] * len(Q)); names.extend([p["name"]] * len(Q))
        dm = m.simplify_quadric_decimation(face_count=360) if len(m.faces) > 360 else m
        meshes.append(dict(name=p["name"], v=dm.vertices.round(1).tolist(), f=dm.faces.tolist()))
    return dict(P=np.array(P), rad=np.array(rad), ring=np.array(ring), names=names, dz=dz, cart_bottom_seat=round(-54.0 + dz, 1),
                draw=dict(nodes=N.round(1).tolist(), rods=sh["rods"], ring=sorted(ring_ids), meshes=meshes))

def wedge_mesh(r):
    """the ramp's 6 corners for drawing (seat frame)"""
    ax, s, k, (lo, hi), h = r["axis"], r["sign"], r["line"] - TUBE_R * S2, r["span"], r["h"]
    pts = []
    for t in (lo, hi):
        for (u, z) in ((k - h, h), (k - BOTTOM, BOTTOM), (k - h, BOTTOM)):
            p = [0.0, 0.0, z]; p[ax] = s * u; p[1 - ax] = t; pts.append(p)
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
    S_, O_ = res["scout"], res["overseer"]; m = lambda d: min(d["capture"][k]["ok_mm"] for k in ("x", "y", "diag"))
    res["whatif"] = [dict(label=f"Scout, side ramps {ALT_SIDE:.0f}", capture=alt["scout"]["capture"]),
                     dict(label=f"Overseer, belly pods in to |y| {ALT_SLED_Y:.0f}, side ramps {ALT_SIDE:.0f}", capture=alt["overseer"]["capture"])]
    res["lede"] = (f"No rim and no change to either cage. The Scout's bottom opening is just the Overseer's middle rectangle "
                   f"({2 * RX:.0f} × {2 * RY:.0f} between node centres). Ramps rise into the opening: two shared side ramps under the long sides, "
                   f"two Overseer end ramps under its trapezoid tips, and two Scout end ramps under the Scout's short ends, split around the Overseer's "
                   f"cartridge and kept {H_SEND:.0f} mm low so the Overseer passes over them. Both cartridges seat 3 mm above the ring, so one head stop. "
                   f"Landing tolerance as the cages are today: Scout ±{m(S_)} mm, Overseer ±{m(O_)} mm.")
    res["notes"] = [
        f"Ramp heights (mm above the seat; along its axis a 45° ramp catches about its own height): side {H_SIDE:.0f}, Overseer ends {H_OEND:.0f}, Scout ends {H_SEND:.0f}. The side ramps are two pads a side (x ±12..55) so they miss the corner nodes and the Scout's mid node; the Overseer end ramps sit between its tip nodes (y ±25); the Scout end ramps are pads at |y| 54..88, beside the Overseer's receiver and its TPU feet.",
        f"What limits the Overseer sideways: its belly sled and the down camera / flow pods reach y ±72 at 9 mm above the ring, right over the side ramps. With the pods pulled in to |y| {ALT_SLED_Y:.0f} the side ramps can be {ALT_SIDE:.0f} tall: Overseer ±{min(alt['overseer']['capture'][k]['ok_mm'] for k in ('x', 'y', 'diag'))} mm, Scout ±{min(alt['scout']['capture'][k]['ok_mm'] for k in ('x', 'y', 'diag'))} mm (table).",
        "Scout: its cage ends in the rectangle ring (6 mm tubes, nodes at the 4 corners and the 4 mid sides), cartridge 3 mm above the ring as the Overseer's, pack 1.0 under the FC, two latch rails for the receiver box. Overseer: cage v3 unchanged.",
        "Box: the 6 ramps stand on a deck round the swap column (both cartridges drop straight through between them). The funnel panels and hook latch v2 retire; the hold (a latch over a ring tube) and keep-alive (pins against pads on the ring) move to the ramp bases.",
        "Still open (concept review 27 Oct): the ring tubes slide on the ramps, so the slopes need a low-friction face (PTFE, μ ≤ 0.2) and the drop sim with tilt and drift before anything is printed; the 45° slope and these heights are concept numbers.",
    ]
    enc = lambda o: o.tolist() if hasattr(o, "tolist") else float(o)
    json.dump(res, open(os.path.join(OUT, "result.json"), "w"), default=enc)
    bcj = json.load(open(os.path.join(REPO, "runs", "scout_bottom", "concept", "concept.json")))
    tpl = open(os.path.join(REPO, "nox", "box", "catch_ramps_view.html")).read()
    html = tpl.replace("/*DATA*/null", json.dumps(dict(bc=bcj, ramps=res), default=enc))
    open(os.path.join(OUT, "ramps.html"), "w").write(html)
    open(os.path.join(REPO, "runs", "scout_bottom", "concept", "concept.html"), "w").write(html)   # the published artifact

if __name__ == "__main__":
    main()
