"""Push-bar catch (concept F, 27 Oct, Spark: "the most error-tolerant most well-designed one-motor latch system that
works for both current drone cages. Take your time and research first"). Concept check only; the box catch session
owns the catch. Research: claude/research/2026-10-27-{dock-centering-mechanisms,one-motor-sequencing-and-locks,
precision-landing-accuracy}.md (budget: +-80 mm, +-10 deg, survive +-15 deg).

The drone lands anywhere on a flat deck (nothing stands up inside the landing room). One stepper (NEMA 17, T8 x 2 lead
screw, self-locking) moves one carriage under the deck. Two phases, set by force (lost motion), not by a cam:
  phase 1  two low end pads (H_PAD tall) close along x at a 1 : K_T ratio (nose : tail) through a preloaded spring
           (the pad slide). Each pad face is a profile along y: a notch (|y| <= NOTCH_Y) that cradles a tip node, a
           ramp beside it receding RAMP_DX (the Overseer's tip rods run there at deck height), and V wings
           (WING_Y, WING_DEG, outer corners leading) that push an off-centre node inward and carry the Scout's straight
           end tube on their corners. The pads stop where the drone stops them: the Overseer on its two tip nodes after
           S_O of nose travel, the Scout on its two end tubes after S_S; K_T puts both receivers on the swap centre.
  phase 2  the carriage runs S2 further against the pad spring; that relative motion alone drives two short side bars
           (BAR_X long, L-section with lips) from park onto the shared side tubes to a rigid stop (gap GAP), squaring
           yaw and centring y at the drone's seated x (the side tubes taper 3.6 over 155, so a fixed gap fits at one x;
           and the Overseer's side rods leave the side nodes at 16 deg, so a bar near a node before x is centred gets a
           rod across its top). The nodes ride the pad ramps into the notches, the spring gives (stroke LM). The lips
           reach over the tubes (hold-down for the 74 N head stall). The motor stalls on the bars' stop: StallGuard and
           the step count read seated, and which drone. Release: back out.
Seat frame: catch centre, z 0 = the Overseer's ring tube centres seated; its TPU feet reach z -4.0 (M), so the deck is
at -4.0 and its ring floats 1.0 above it. The Scout rests on 6 mm printed sleeves on its side rods (SLEEVE = 1, DEFAULT:
one gap, one lip height for both) or on its bare r 1.5 rods (SLEEVE = 0: 3.5 of y slack); its cartridge height is reported.

Check: the drone lands at an offset (x, y) and yaw; the bodies move in STEP mm of nose-pad travel; every step the drone
is pushed the least that resolves every contact (translation + yaw, least-norm; in phase 2 the pad slide is a 4th
unknown, sprung within its stroke). Sphere-vs-body contacts are exact (every body an orthogonal box in its own frame).
A pushing face may only touch ring tubes, nodes and the tip-node structure; a lip touching anything, a body running
under anything, a rod climbing a top edge by more than RIDE, or a landing on a parked body fails that landing. Both pads
pinching = the x clamp (phase 2 starts); the carriage at s_clamp + S2 = seated. Capture = the landings that end seated,
with the pose error, the clearances and the stall travel.
python3 nox/box/catch_bars_concept.py                   full sweep (both drones) -> runs/box_catch/bars_concept/result.json + page
python3 nox/box/catch_bars_concept.py --drone scout      one drone -> result_scout.json (run the two in parallel), then --merge
python3 nox/box/catch_bars_concept.py --smoke            27 landings a drone
python3 nox/box/catch_bars_concept.py --html             rebuild the page from result.json
"""
import os, sys, json, math, time, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
import catch_ramps_concept as cr                      # the two drones from git refs (SCOUT_REF / OVR_REF at its top)
OUT = os.path.join(REPO, "runs", "box_catch", "bars_concept"); os.makedirs(OUT, exist_ok=True)

DECK = float(os.environ.get("DECK", -4.0))                 # M: the Overseer's TPU feet bottom (its ring tube centre at 0, r 3)
TUBE_R = 3.0                # Overseer ring tube (C); the Scout's is 1.5 (C)
GAP = 0.5                   # closed side gap over the two tube surfaces (E: ring width tolerance)
Y_WAIT = None               # (phase 1 cam removed: the side bars stay parked until the x clamp; the lost-motion stage alone drives them)
LM = float(os.environ.get("LM", 80.0))                   # pad spring stroke (the slide may lag the carriage this much): S2 plus the x the nodes need to ride the pad ramps into the notches (17, C)
S2 = float(os.environ.get("S2", 60.0))                   # phase 2 carriage travel after the x clamp: drives the side bars from park to the gap (ratio 2.3)
EXTRA_Y = float(os.environ.get("PARK_EXTRA_Y", 137.0))  # side bars parked this far outside the seated tube surface (|y| 258): the Overseer's 16 deg side rods clear the bar top with a node anywhere inside the +-80 landing room (C)

SLEEVE = int(os.environ.get("SLEEVE", 1))            # 1: 6 mm printed sleeves on the Scout's side tubes over the bar span (DEFAULT); 0: bare 3 mm rods
SLEEVE_HALF = 32.0          # sleeve half length along x (bar 50 + yaw)
BAR_X = (-25.0, 25.0)       # side bar span along x (C: inside both drones' clear tube spans at +-40 landing)
BAR_T = 6.0                 # bar thickness (outward), >= 6 x 6 (attachments rule)
LIP_Z0, LIP_T = 4.5, 3.0    # lip underside above the seated Overseer tube top (3.0) by 1.5; lip 3 thick
LIP_IN = 2.0                # lip edge this far inside the tube centre line (the Overseer's inner link is at +5..8)
H = LIP_Z0 + LIP_T - DECK   # bar height from the deck (11.5)
NOTCH_Y = 6.0               # notch half width: the Overseer's tip rods leave the node surface at |y| 7.9 (C), so the notch (contact at |y| r sin V = 5.1) only ever touches the node sphere
PAD_T = 8.0
RIDE = 2.0                  # a pushed rod met by a body's top edge with the normal tilted past the friction angle (centre > 0.3 r above the top) climbs onto it; the climb (r - height above the top) may be this much (reported), more is a crash
H_PAD = 3.0                 # pad height from the deck (top at z -1): pushes the Overseer's node 1 under its equator and the Scout's end tube on centre; the tip rods (5.7 deg up) pass over it from 20 mm out of the node
V_DEG = 30.0                # notch flank angle from the y axis (120 deg notch): a node seats r / cos 30 ahead of the apex, centred in y
TV = math.tan(math.radians(V_DEG)); CV = math.cos(math.radians(V_DEG))
WING_Y = (28.0, 80.0)       # V wings each side, 18 deg flanks, outer end leading: they push a node inward (squaring a yawed Overseer), the Scout's straight end tube rests on their outer corners; the tip rods pass over them (>= 0.6 above the pad top)
WING_DEG = float(os.environ.get("WING_DEG", 18.0))             # wing flank angle from y: squares the Overseer (tan > 33 / 191) and the Scout (tan < 87 / 60)
TW = math.tan(math.radians(WING_DEG))
RAMP_Y, RAMP_DX = 28.0, 14.0  # from the notch edge the face recedes to RAMP_DX behind the apex at |y| RAMP_Y: the tip rods run there at deck height (x apex + 11 at |y| 28); an off-centre node slides out to the wings
STEP = 0.5                  # nose-pad travel per step
EXTRA = float(os.environ.get("PARK_EXTRA", 90.0))     # parked bars this far outside the seated Overseer's features: the landing room (research 27 Oct: real touchdown error 7..14 cm, budget +-80)

# the shared hexagon (Overseer cage v5 base loop) about the catch centre; the Scout's rectangle is its 4 side nodes
HEX = cr.HEX
NODE_R = 10.15
WING_C = (NODE_R + GAP / 2) / math.cos(math.radians(WING_DEG)) + (WING_Y[1] - WING_Y[0]) / 2 * TW - (NODE_R + GAP / 2) / CV   # wing inner end behind the apex so a node mid-wing sits at its notch x (2.9)
D_WING = -(WING_C - (WING_Y[1] - WING_Y[0]) * TW)   # the wings' outer corners lead the apex by this (the Scout's end tube rests there, 7.5)
def side_line(sign):
    """the +-y side tube centre line: point, unit direction (tail -> nose), outward unit normal"""
    A = np.array([HEX["Lt"][0], sign * HEX["Lt"][1]]); B = np.array([HEX["Ln"][0], sign * HEX["Ln"][1]])
    t = (B - A) / np.linalg.norm(B - A); n = np.array([-t[1], t[0]])
    if n[1] * sign < 0: n = -n
    return A, t, n

# ---- travel schedule (nose-pad travel s, mm)
# pad positions are the notch apex; a node of radius r seats with its centre r / cos V ahead of the apex, a straight tube on
# the wings, D_WING ahead of it
NOSE_SEAT = HEX["F"][0] + (NODE_R + GAP / 2) / CV          # nose apex when the Overseer is clamped (205.9)
TAIL_SEAT = HEX["T"][0] - (NODE_R + GAP / 2) / CV          # (-227.3)
SCOUT_NOSE = HEX["Ln"][0] + 1.5 + GAP / 2 + D_WING         # nose apex when the Scout's end tube is on the wings (96.05)
SCOUT_TAIL = HEX["Lt"][0] - 1.5 - GAP / 2 - D_WING         # (-79.15)
K_T = (SCOUT_TAIL - TAIL_SEAT) / (NOSE_SEAT - SCOUT_NOSE)  # tail : nose travel so both drones clamp centred (1.361)
NOSE_PARK = NOSE_SEAT + EXTRA
S_O = NOSE_PARK - NOSE_SEAT                                 # nose travel to the Overseer clamp (= EXTRA)
S_S = NOSE_PARK - SCOUT_NOSE                                # to the Scout clamp
TAIL_PARK = TAIL_SEAT - K_T * S_O
Y_OUT = TUBE_R + GAP / 2                                    # side face outside the tube centre line when closed (3.25; the Scout with sleeves the same)
Y_LINE0 = HEX["Lt"][1] + (0 - HEX["Lt"][0]) * (HEX["Ln"][1] - HEX["Lt"][1]) / (HEX["Ln"][0] - HEX["Lt"][0])   # the side line's |y| at x 0 (118.1)
Y_PARK = Y_OUT + EXTRA_Y

def side_out(s, s_clamp=None):
    """side face offset outside the tube centre line: parked until the x clamp, then (phase 2, the lost motion) to Y_OUT over LM"""
    if s_clamp is None or s <= s_clamp: return Y_PARK
    return Y_PARK - (Y_PARK - Y_OUT) * min((s - s_clamp) / S2, 1.0)

def box(planes, work, name, allow, ends, disp=(0.0, 0.0)):
    """convex body: planes (n, d), inside = n.p <= d; work = indices of the pushing faces (empty: no contact allowed);
    ends = plane indices a tube running along the bar slides past (not tested for tube points); disp = the body's plan
    displacement per mm of pad-slide advance (the pads are sprung in phase 2)"""
    return dict(N=np.array([p[0] for p in planes], float), D=np.array([p[1] for p in planes], float), work=set(work), name=name, allow=allow,
                ends=list(ends), disp=np.array(disp, float))

def rbox(o, u, t, depth, L, ztop, name, allow, disp):
    """an orthogonal box: face through o with outward normal u (toward the drone), lateral direction t along the face
    from o over length L, depth behind the face, up to ztop. Planes: face, back, lateral low, lateral high, top"""
    o = np.array([o[0], o[1], 0.0]); u3 = np.array([u[0], u[1], 0.0]); t3 = np.array([t[0], t[1], 0.0])
    planes = [(u3, u3 @ o), (-u3, depth - u3 @ o), (-t3, -(t3 @ o)), (t3, t3 @ o + L), (np.array([0, 0, 1.0]), ztop)]
    return box(planes, (0,), name, allow, (2, 3), disp)

def pad_bodies(xa, sgn, name, disp):
    """one end pad: local X = sgn (x - apex) grows into the pad; the drone is at X < 0"""
    out = []; top = DECK + H_PAD
    e0 = -NOTCH_Y * TV                                                   # X of the notch edge
    rl = math.hypot(RAMP_DX - e0, RAMP_Y - NOTCH_Y)                      # ramp face length
    wl = math.hypot((WING_Y[1] - WING_Y[0]) * TW, WING_Y[1] - WING_Y[0]) # wing face length
    for sy in (1, -1):
        feats = [  # (origin X, origin y, lateral direction (X, y), outward normal (X, y), length)
            (0.0, 0.0, (-TV, sy), (-1.0, -sy * TV), NOTCH_Y * math.hypot(1, TV)),                                         # notch flank
            (e0, sy * NOTCH_Y, (RAMP_DX - e0, sy * (RAMP_Y - NOTCH_Y)), (-(RAMP_Y - NOTCH_Y), sy * (RAMP_DX - e0)), rl),  # ramp
            (WING_C, sy * WING_Y[0], (-TW, sy), (-1.0, -sy * TW), wl)]                                                    # wing flank
        for X0, y0, t, u, L in feats:
            t = np.array(t, float) / np.linalg.norm(t); u = np.array(u, float) / np.linalg.norm(u)
            o = (xa + sgn * X0, y0); tg = (sgn * t[0], t[1]); ug = (sgn * u[0], u[1])
            out.append(rbox(o, ug, tg, PAD_T, L, top, name, "ends", disp))
    return out

def bars_at(s, s_clamp=None, s_slide=None):
    """bodies at nose travel s; the pads sit at the slide position s_slide (= s until the clamp, then sprung); s None: pads only"""
    B = []; ex, ey, ez = np.eye(3); sp = s if s_slide is None else s_slide
    out = side_out(s, s_clamp) if s is not None else None
    for sign, nm in (((1, "side +y"), (-1, "side -y")) if s is not None else ()):
        A, t, n = side_line(sign); n3 = np.array([n[0], n[1], 0.0])
        d_face = n @ A + out                                   # face plane n.p = d_face; the tube side is n.p <= d_face
        body = [(-n3, -d_face), (n3, d_face + BAR_T), (-ex, -BAR_X[0]), (ex, BAR_X[1]), (ez, DECK + H)]
        B.append(box(body, (0,), nm, "ring", (2, 3)))
        lip = [(n3, d_face), (-n3, -(d_face - (Y_OUT + LIP_IN))), (-ex, -BAR_X[0]), (ex, BAR_X[1]), (-ez, -LIP_Z0), (ez, LIP_Z0 + LIP_T)]
        B.append(box(lip, (), nm + " lip", None, (2, 3)))
    # end pads, each a face profile along |y| (6 orthogonal boxes in rotated frames, so the sphere test is exact): a notch
    # (|y| <= NOTCH_Y, apex at xn / xt, flanks V_DEG from y) for a tip node; a ramp (NOTCH_Y..RAMP_Y) receding to RAMP_DX
    # behind the apex, clear of the tip rods; V wings (WING_Y, WING_DEG) whose outer corners lead
    xn = NOSE_PARK - sp; xt = TAIL_PARK + K_T * sp
    B += pad_bodies(xn, 1, "nose pad", (-1.0, 0.0)) + pad_bodies(xt, -1, "tail pad", (K_T, 0.0))
    return B

# ---- the drones on the flat deck
def prep(d, kind):
    """keep the points that can reach a bar (surface under z 30), masks of what each bar kind may push"""
    P, rad, ring = d["P"], d["ring"] * 0 + d["rad"], d["ring"]; names = np.array(d["names"], object)
    node_c = np.array([HEX[k] for k in (("Ln", "Rn", "Lt", "Rt", "F", "T") if kind == "overseer" else ("Ln", "Rn", "Lt", "Rt"))])
    node_r = NODE_R if kind == "overseer" else 6.0
    dn = np.min(np.linalg.norm(P[:, None, :2] - node_c[None], axis=2), axis=1)
    is_tube = np.char.startswith(names.astype(str), "cage tube")
    keep = ((P[:, 2] - rad) < DECK + H + 0.5) & ~(is_tube & (dn < node_r))          # under the lip top; no rod sample inside its node sphere
    P, rad, ring, names, dn = P[keep], rad[keep], ring[keep], names[keep], dn[keep]
    tube = ring & np.char.startswith(names.astype(str), "cage tube")
    node = np.char.startswith(names.astype(str), "cage node")                       # a hub standing on the deck: tested in plan only
    near_node = dn < NODE_R + 10
    tipc = np.array([HEX["F"], HEX["T"]]); dtip = np.min(np.linalg.norm(P[:, None, :2] - tipc[None], axis=2), axis=1)
    fitting = np.char.startswith(names.astype(str), "spokes") & ((dn < NODE_R + 10) | (dtip < 70))   # node fittings; the nose and tail structure at the tip nodes
    cage = np.char.startswith(names.astype(str), "cage")
    return dict(P=P, rad=rad, names=names, allow=dict(ring=ring, ends=cage | fitting), tube=tube, node=node, kind=kind, draw=d["draw"], cart=d["cart_bottom_seat"])

def overseer():
    return prep(cr.overseer(), "overseer")

def scout():
    """the Scout resting on its side tubes (6 mm sleeves over the bar span, or bare r 1.5): ring centre at DECK + r"""
    r = 3.0 if SLEEVE else 1.5
    sh = json.loads(cr.git_bytes(cr.SCOUT_REF, f"{cr.SCOUT_DIR}/cage_shell.json"))
    plane = cr.SCOUT_PLANE_Z
    move = {i: [sh["nodes"][i][0], sh["nodes"][i][1], plane] for i in sh["open"][0]["ring"]}
    off = np.array([-cr.SCOUT_CATCH_X, 0.0, -plane + DECK + r])
    d, _ = cr.load_drone(cr.SCOUT_REF, "nox/scout/v3/scout.assembly.json", f"{cr.SCOUT_DIR}/cage_shell.json", off, move=move)
    d["cart_bottom_seat"] = round(-44.5 + off[2], 2)
    p = prep(d, "scout")
    if SLEEVE:   # 6 mm sleeves on the two side tubes over |x| <= SLEEVE_HALF
        A, t, n = side_line(1); P = p["P"]
        side = p["tube"] & (np.abs(np.abs(P[:, 1]) - (A[1] + (P[:, 0] - A[0]) * t[1] / t[0])) < 4) & (np.abs(P[:, 0]) <= SLEEVE_HALF)
        p["rad"] = p["rad"].copy(); p["rad"][side] = 3.0; p["sleeve_pts"] = int(side.sum())
    return p

def pose_pts(P, pose):
    dx, dy, th = pose; c, s_ = math.cos(th), math.sin(th)
    Q = P.copy(); Q[:, 0] = c * P[:, 0] - s_ * P[:, 1] + dx; Q[:, 1] = s_ * P[:, 0] + c * P[:, 1] + dy
    return Q

RG = 90.0   # least-norm metric: 1 mm of slide ~ RG mm of yaw lever (the footprint's radius of gyration, E)
WS = 0.3    # and ~ WS mm of pad-spring travel: the spring gives before the drone is dragged (soft spring vs rigid contacts, E)

def contacts(d, Q, B):
    """per body: sphere vs convex body. Outside the body the nearest point comes from the summed plane violations (exact
    for a box), so a corner or edge pushes along its real normal; inside, the shallowest face. For ring tube points the end
    planes only bound the hit (a tube slides past an end). Returns (fail reason or None, list of (p, n_xy, pen, body),
    {body name: max penetration}, max climb)"""
    cons = []; pushers = {}; ride = 0.0
    for b in B:
        v = Q @ b["N"].T - b["D"]
        vr = v.copy(); vr[np.ix_(d["tube"], b["ends"])] = -1e9; vr[d["node"], -1] = -1e9
        pos = np.maximum(vr, 0.0); outside = (vr > 0).any(1)
        g = pos @ b["N"]; gn = np.linalg.norm(g, axis=1)
        dist = np.where(outside, gn, vr.max(1)); hit = dist < d["rad"]
        if b["ends"]: hit &= v[:, b["ends"]].max(1) < d["rad"]            # a tube slides past an end: beyond it (by a radius) nothing touches
        if not hit.any(): continue
        if not b["work"]: i = np.where(hit)[0][0]; return f"{b['name']} meets {d['names'][i]}", None, None, None
        idx = np.where(hit)[0]; W = sorted(b["work"])
        wf = np.array(W)[vr[idx][:, W].argmax(1)]                            # inside the body: out through the working face it came in by
        nrm = np.where(outside[idx, None], g[idx] / np.maximum(gn[idx], 1e-9)[:, None], b["N"][wf])
        pen = np.where(outside[idx], d["rad"][idx] - dist[idx], d["rad"][idx] - vr[idx, wf]); top = v[idx, -1]
        lateral = np.linalg.norm(nrm[:, :2], axis=1)
        over = (top > 0) & (nrm[:, 2] > 0.3)                                 # met mostly from above: the body runs under the point
        climb = np.where(over, d["rad"][idx] - top, 0.0)
        allowed = d["allow"][b["allow"]][idx]
        bad = ~allowed
        if bad.any(): i = idx[np.where(bad)[0][0]]; return f"{b['name']} pushes {d['names'][i]}", None, None, None
        if (climb > RIDE).any():
            i = idx[np.where(climb > RIDE)[0][0]]
            if os.environ.get("DEBUG"): print("   DEBUG top", b["name"], d["names"][i], Q[i].round(1), "rad", d["rad"][i], "v", v[i].round(1))
            return f"{b['name']} (top face) meets {d['names'][i]}", None, None, None
        ride = max(ride, float(climb.max()))
        push = (~over) & (lateral > 0.3) & (pen > 0.02)                       # the rest rides on the top edge: no push
        if not push.any(): continue
        pushers[b["name"]] = max(pushers.get(b["name"], 0.0), float(pen[push].max()))
        for k in np.where(push)[0]:
            n2 = nrm[k, :2] / lateral[k]
            cons.append((Q[idx[k], :2].copy(), n2, pen[k] * lateral[k], b))
    return None, cons, pushers, ride

def resolve(d, pose, B, slide=None):
    """push the drone until no body penetrates it (least-norm translation + yaw). With slide = [p, lo, hi] the pad slide
    is a 4th unknown (sprung: it gives way or advances within [lo, hi], weighted WS). Returns (pose, slide p,
    {body: residual}, {bodies touched}, fail)"""
    touched = set(); ride_max = 0.0; p = None if slide is None else slide[0]
    for it in range(14):
        Q = pose_pts(d["P"], pose); fail, cons, pushers, ride = contacts(d, Q, B)
        if fail: return pose, p, {}, touched, fail
        touched |= set(pushers); ride_max = max(ride_max, ride)
        resolve.ride = max(getattr(resolve, "ride", 0.0), ride_max)
        if not cons: return pose, p, {}, touched, None
        Pc = np.array([c[0] for c in cons]); Nc = np.array([c[1] for c in cons]); pen = np.array([c[2] for c in cons])
        r = Pc - pose[:2]; cols = [Nc[:, 0], Nc[:, 1], (r[:, 0] * Nc[:, 1] - r[:, 1] * Nc[:, 0]) / RG]
        if slide is not None: cols.append(-np.array([c[3]["disp"] @ c[1] for c in cons]) / WS)     # a pad moving with the slide closes its gap
        A = np.column_stack(cols); A2 = (A * A).sum(1); dq = np.zeros(A.shape[1]); p0 = p
        for _ in range(150):                                   # projected Kaczmarz on the gap inequalities, least-norm in (x, y, RG yaw, WS slide)
            worst = 0.0
            for i in range(len(A)):
                g = A[i] @ dq - pen[i]
                if g < -1e-3: dq += (-g) * A[i] / A2[i]; worst = max(worst, -g)
            if slide is not None:                              # the spring stroke bounds the slide
                pn = min(max(p + dq[3] / WS, slide[1]), slide[2]); dq[3] = (pn - p) * WS
            if worst < 1e-3: break
        if slide is not None: p = min(max(p + dq[3] / WS, slide[1]), slide[2])
        if worst > 0.05 or np.abs(dq).max() < 1e-3:
            if os.environ.get("DEBUG"): print(f"   DEBUG stuck worst {worst:.2f} dq {dq.round(3)} slide {p0} bounds {slide}")
            return pose, p0, pushers, touched, None                # opposed contacts: cannot be resolved by a rigid move (nothing moves)
        pose = pose + np.array([dq[0], dq[1], dq[2] / RG])
        if slide is not None: B = rebuild_pads(B, p)
    Q = pose_pts(d["P"], pose); fail, cons, pushers, ride = contacts(d, Q, B)
    return pose, p, pushers, touched | set(pushers), fail

def rebuild_pads(B, p):
    """the pad bodies at slide position p (the side bars stay)"""
    new = [b for b in B if not b["name"].endswith("pad")]
    return new + [b for b in bars_at(None, None, p) if b["name"].endswith("pad")]

def landing(d, off, yaw_deg):
    """one landing: dict(ok, why, s_clamp, s_slide, pose_err=(dx, dy, yaw deg), clear, clear_who, ride)"""
    pose = np.array([off[0], off[1], math.radians(yaw_deg)]); s = 0.0; s_clamp = None; s_slide = None; resolve.ride = 0.0
    while s <= (S_S + 2.0 if s_clamp is None else s_clamp + S2):
        if s_clamp is None:
            pose, _p, resid, touched, fail = resolve(d, pose, bars_at(s))
        else:
            # phase 2: the pad slide is sprung to the carriage: each step the spring pushes it fully forward and the
            # contacts push it back as far as they must (within the stroke)
            p_start = min(s, s_slide + 1.0)
            pose, s_slide, resid, touched, fail = resolve(d, pose, bars_at(s, s_clamp, p_start), slide=[p_start, max(s - LM, 0.0), s])
        if fail and s == 0.0: return dict(ok=False, why=f"lands on the {fail.split(' (')[0].split(' pushes')[0].split(' meets')[0]}: {fail.split(' ')[-1] if ' pushes ' not in fail and ' meets ' not in fail else fail.split('pushes ' if 'pushes ' in fail else 'meets ')[1]}", s=s, pose=pose.round(2).tolist(), phase=0)
        if fail: return dict(ok=False, why=f"{fail} at s {s:.1f}", s=s, pose=pose.round(2).tolist(), phase=1 if s_clamp is None else 2)
        if abs(pose[0]) > 400 or abs(pose[1]) > 300: return dict(ok=False, why="runaway", s=s, pose=pose.round(2).tolist())
        pads = {k: v for k, v in resid.items() if k.endswith("pad") and v > 0.1}
        sides = {k: v for k, v in resid.items() if k.startswith("side") and v > 0.1}
        if s_clamp is None:
            if len(pads) == 2: s_clamp = s; s_slide = s                 # the x clamp: from here the pads are sprung and the side bars close
            elif sides: return dict(ok=False, why=f"jammed at s {s:.1f} (phase 1): {sorted(resid)} cannot resolve ({max(resid.values()):.2f})", s=s, pose=pose.round(2).tolist())
        else:
            if pads and s - s_slide >= LM - 0.01: return dict(ok=False, why=f"pad spring bottomed at s {s:.1f}: { {k: round(v, 2) for k, v in pads.items()} }", s=s, pose=pose.round(2).tolist())
            if sides and s < s_clamp + S2 - STEP / 2:
                return dict(ok=False, why=f"side bars jam before their stop at s {s:.1f} (phase 2): {sides}", s=s, pose=pose.round(2).tolist())
            if s >= s_clamp + S2 - STEP / 2: return finish(d, pose, s, s_clamp, s_slide)
        s += STEP
    return dict(ok=False, why="no x clamp before the end of travel", s=s, pose=pose.round(2).tolist())

def finish(d, pose, s, s_clamp, s_slide):
    """seated: lips and bars' clearance to everything that is not a ring tube, the pose error"""
    Q = pose_pts(d["P"], pose); clear = 1e9; who = ""
    for b in bars_at(s, s_clamp, s_slide):
        v = Q @ b["N"].T - b["D"]; m = v.max(1) - d["rad"]
        mask = ~d["allow"][b["allow"]] if b["work"] else np.ones(len(m), bool)   # what this body must never touch
        j = int(np.argmin(np.where(mask, m, 1e9)))
        if m[j] < clear: clear, who = float(m[j]), f"{b['name']} to {d['names'][j]}"
    err = (round(float(pose[0]), 2), round(float(pose[1]), 2), round(math.degrees(pose[2]), 2))
    return dict(ok=True, s_clamp=round(s_clamp, 1), s_slide=round(s_slide, 2), spring=round(s - s_slide, 2), s_stall=round(s, 1), pose_err=err,
                clear=round(clear, 2), clear_who=who, ride=round(resolve.ride, 2))

def sweep(d, offs, yaws, offs_x=None):
    res = []; t0 = time.time()
    for ox in (offs_x or offs):
        for oy in offs:
            for yw in yaws:
                r = landing(d, (ox, oy), yw); r.update(off=[ox, oy], yaw=yw); res.append(r)
                print(f"  {d['kind']:8s} off ({ox:4.0f},{oy:4.0f}) yaw {yw:4.0f}: " +
                      (f"ok clamp {r['s_clamp']} slide {r['s_slide']} err {r['pose_err']} clear {r['clear']} ({r['clear_who']}) ride {r['ride']}" if r["ok"] else f"FAIL {r['why']} pose {r.get('pose')}"), flush=True)
    print(f"  {d['kind']} {len(res)} landings in {time.time() - t0:.0f} s", flush=True)
    return res

def capture(res):
    """largest square of offsets (every yaw) with no failure, per-axis reach, the yaws that work at offset 0, worst errors"""
    oks = {(r["off"][0], r["off"][1], r["yaw"]): r["ok"] for r in res}
    offs = sorted({abs(r["off"][0]) for r in res}); yaws = sorted({r["yaw"] for r in res}); allo = sorted({r["off"][0] for r in res})
    best = -1
    for a in offs:
        if all(oks.get((ox, oy, y), False) for ox in allo for oy in allo for y in yaws if abs(ox) <= a and abs(oy) <= a): best = a
        else: break
    per_axis = {}
    for ax, i in (("x", 0), ("y", 1)):
        okd = {}
        for r in res:
            if r["off"][1 - i] == 0 and r["yaw"] == 0: okd[r["off"][i]] = r["ok"]
        plus = max([v for v in okd if v >= 0 and all(okd[w] for w in okd if 0 <= w <= v)], default=0)
        minus = min([v for v in okd if v <= 0 and all(okd[w] for w in okd if v <= w <= 0)], default=0)
        per_axis[ax] = [minus, plus]
    yaw_ok = [y for y in yaws if all(r["ok"] for r in res if r["yaw"] == y and r["off"] == [0, 0])]
    good = [r for r in res if r["ok"]]
    fails = {}
    for r in res:
        if not r["ok"]: k = r["why"].split(" at s")[0]; fails[k] = fails.get(k, 0) + 1
    return dict(square_mm=best, per_axis=per_axis, yaw_ok_deg=yaw_ok,
                worst_pose_err_mm=max((max(abs(r["pose_err"][0]), abs(r["pose_err"][1])) for r in good), default=None),
                worst_yaw_err_deg=max((abs(r["pose_err"][2]) for r in good), default=None),
                min_clear_mm=min((r["clear"] for r in good), default=None), s_clamp=sorted({r["s_clamp"] for r in good}),
                n=len(res), n_ok=len(good), fails=dict(sorted(fails.items(), key=lambda kv: -kv[1])[:6]))

def main(smoke=False, only=None, tag=""):
    offs = [-20, 0, 20] if smoke else [-80, -60, -40, -20, 0, 20, 40, 60, 80]
    yaws = [-10, 0, 10] if smoke else [-15, -10, -5, 0, 5, 10, 15]
    offs_x = [float(v) for v in os.environ["OFFS_X"].split(",")] if os.environ.get("OFFS_X") else offs   # a chunk of the x rows (parallel runs)
    res = dict(deck=DECK, sleeve=SLEEVE, gap=GAP, lm=LM, s2=S2, wing_deg=WING_DEG, wing_c=round(WING_C, 2), bar_x=BAR_X, bar_t=BAR_T, lip=[LIP_Z0, LIP_T, LIP_IN], notch_y=NOTCH_Y, wing_y=WING_Y, d_wing=round(D_WING, 2),
               ramp=[RAMP_Y, RAMP_DX],
               pad_t=PAD_T, h_pad=H_PAD, v_deg=V_DEG, height=H,
               park_extra=EXTRA, park_extra_y=EXTRA_Y,
               schedule=dict(nose_park=round(NOSE_PARK, 2), tail_park=round(TAIL_PARK, 2), y_park=round(Y_LINE0 + Y_PARK, 2), k_t=round(K_T, 4),
                             s_overseer=round(S_O, 1), s_scout=round(S_S, 1), tail_travel_scout=round(K_T * S_S, 1),
                             y_travel=round(Y_PARK - Y_OUT, 1), y_ratio=round((Y_PARK - Y_OUT) / S2, 2), carriage_overseer=round(S_O + S2, 1), carriage_scout=round(S_S + S2, 1)),
               seats=dict(nose_overseer=round(NOSE_SEAT, 2), tail_overseer=round(TAIL_SEAT, 2), nose_scout=round(SCOUT_NOSE, 2), tail_scout=round(SCOUT_TAIL, 2), y_out=Y_OUT),
               hex=HEX, offs=offs, yaws=yaws)
    for name, f in (("overseer", overseer), ("scout", scout)):
        if only and name != only: continue
        d = f(); print(f"{name}: {len(d['P'])} low points, cartridge bottom {d['cart']} above z 0 ({d['cart'] - DECK:.2f} above the deck)", flush=True)
        r = sweep(d, offs, yaws, offs_x)
        res[name] = dict(cart_bottom_seat=d["cart"], landings=r, capture=capture(r), draw=d["draw"], sleeve_pts=d.get("sleeve_pts"))
        print(name, json.dumps(res[name]["capture"]), flush=True)
    enc = lambda o: o.tolist() if hasattr(o, "tolist") else float(o)
    fn = ("smoke" if smoke else "result") + (f"_{only}" if only else "") + tag + ".json"
    json.dump(res, open(os.path.join(OUT, fn), "w"), default=enc)
    if not smoke and not only: build_html(res)

def merge():
    """join result_overseer*.json and result_scout*.json (chunks of x rows) into result.json and build the page"""
    import glob
    out = None
    for name in ("overseer", "scout"):
        parts = sorted(glob.glob(os.path.join(OUT, f"result_{name}*.json")))
        land = []; base = None
        for fp in parts:
            j = json.load(open(fp)); base = base or j; land += j[name]["landings"]
        if out is None: out = base
        out[name] = dict(base[name]); out[name]["landings"] = land; out[name]["capture"] = capture(land)
        print(name, len(parts), "files,", len(land), "landings:", json.dumps(out[name]["capture"]))
    json.dump(out, open(os.path.join(OUT, "result.json"), "w")); build_html(out)

def build_html(res=None):
    res = res or json.load(open(os.path.join(OUT, "result.json")))
    tpl_path = os.path.join(HERE, "catch_bars_view.html")
    if not os.path.exists(tpl_path): print("no view template yet"); return
    enc = lambda o: o.tolist() if hasattr(o, "tolist") else float(o)
    html = open(tpl_path).read().replace("/*DATA*/null", json.dumps(res, default=enc))
    open(os.path.join(OUT, "bars.html"), "w").write(html)

if __name__ == "__main__":
    only = sys.argv[sys.argv.index("--drone") + 1] if "--drone" in sys.argv else None
    tag = sys.argv[sys.argv.index("--tag") + 1] if "--tag" in sys.argv else ""
    build_html() if "--html" in sys.argv else merge() if "--merge" in sys.argv else main(smoke="--smoke" in sys.argv, only=only, tag=tag)
