"""Hub v6 (24 Oct 2026; swap arm system worker, from the hub + heads worker's PR 43 draft): hub v5 rebuilt for the arm v6 stack
(swap_system.py, arm_v6_layout.py: printed V-rails on the motor plate, rack and pinion heads). Changes from hub v5 (arm_cad.hub_parts),
every number C unless tagged E:
  - motor seats at the v5 poses (arm_v5_layout.MOTORS_V5 + the axle pose of its balance search, runs/arm/v5-layout/result.json):
    A (53.5, 55) 45.8 deg, B (-55, -53.5) -135.8 deg, axle (53.9, -43.7) -39 deg, axle seat slotted +-1 for the loop as before;
  - no posts (compact stack): 4 rail pads on the plate top (X +-16 x +-10 along the rail, Y 50..54) at r 40 / 78 on head A's and
    head B's axes, each with ONE M5 heat-set insert hole Ø6.4 x 7 at X 0 in a boss r 6.5 from Y 44 (the rail bolts down from its top);
  - detent tower at detent.chosen(): v6 (21.2, -30.3), v7 (31.1, -31.1) (roller -135, ring v3; every gap 1.0, C);
  - the harness drop lane (arm_v5_layout.DROP, r 34 at 135 deg) goes through the plate: a Ø9 hole (C);
  - the hub v5 solid_check FAIL fixed in the design: the 1.0 lip (r 29..30, Y 53..54) is gone (web disc top at Y 53), the bore
    cavity gets a drawn roof (Y 47.5..50) and wall (r 33..43, Y 41..47.5) instead of the generated 0.3 films (box-arm Problem 9);
    the wall sits on bearing 2's outer race only (relief r 37.5 to Y 41: 6010 housing shoulder <= Ø75.4, spec; v5's r 36 sat on the seal).
python3 hub_v6.py  -> drawn-only checks (solid, mass) + every gap to the v5 parts over the stroke (C)."""
import math, json, os, itertools
import numpy as np, manifold3d as mf
import arm_layout as L, arm_cad as A, detent as DT, arm_v5_layout as V
from arm_cad import box, cylY, hull, rsquare, rot_pt

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))

import arm_v6_layout as S6
S6.use("v6")                          # rails on the motor plate (compact stack, swap_system.py): no posts, rail pads + inserts on the plate

# ------------------------------------------------------------------ v5 poses (read, never copied)
def motors():
    """head motors from arm_v5_layout, the axle motor from its balance search (result.json; rerun the search if it is missing)"""
    m = {k: dict(v) for k, v in V.MOTORS_V5.items()}
    res = os.path.join(REPO, "runs", "arm", "v5-layout", "result.json")
    if os.path.exists(res):
        ax = json.load(open(res))["motors"]["axle"]
    else:
        b = V.axle_search(); ax = dict(xc=b["xc"], zc=b["zc"], rot=b["th"])
    m["axle"] = dict(xc=float(ax["xc"]), zc=float(ax["zc"]), rot=float(ax["rot"]))
    return m

MOTORS6 = motors()

# detent v5 placement: the mechanism worker's search result if it is on main, else the box-arm.md v5 candidate (DEFAULT)
DET_CAND = dict(th=-45.0, s=-1, a=15.0, b=39.0, phi_d=45.0, psi=90.0, Q=(16.0, -38.0))
DET6 = dict(DT.CHOSEN_V6)                     # roller -135 (arm v2), tower (21.2, -30.3) on the v6 stack; v7: DT.chosen() (tower (31.1, -31.1))

def detent6():
    c = DT.chosen()
    D = DT.design(c["th"], c["s"], c["a"], c["b"], c["phi_d"], c["psi"])
    S = DT.arm_solids(D, np.array(c["Q"]))
    return D, S

# rail pads on the motor plate (v6 stack): one M5 insert per pad at X 0 (A: along +Z, B: along -X), A and B the same radii
PAD_W, PAD_L = 16.0, 10.0             # X +-16 (the rail foot), +-10 along the rail about the bolt (clipped at r 30: 2.5 off the 80T flange)
PAD_R0 = 30.0
BOSS_R, BOSS_Y = 6.5, 44.0            # insert boss r 6.5 (3.3 wall round the Ø6.4 hole) from Y 44 up to the plate top
INSERT = dict(d=6.4, depth=7.0)       # M5 x 5.8 heat-set (OD 7.0, E) in Ø6.4 x 7 from the plate top Y 54 (Y 47..54)
ROOF_Y = (47.5, 50.0)                 # drawn roof over the bore cavity (2.5, joins the motor plate)
CAV_R = 33.0                          # cavity r: sun post flange r 28.5 / bearing 2 inner race shoulder r 29.9 (spec 59.7) + 3.1 (DEFAULT; v5 36)
SEAL_R, SEAL_Y = 37.5, 41.0           # the shoulder on bearing 2's outer race only: relief r 37.5 to Y 41 (6010 housing shoulder <= Ø75.4, spec; v5's r 36 sat on the seal)
SH_R = 43.0                           # shoulder / cavity wall outer r (= the sleeve)


COL_FLARE = (7.5, 6.0)                # detent tower column: r 7.5 at the plate top tapering to its r 5 over 6 (hub v6 run 2: the sustained springs 1.33 at the root, F)
TOWER_LEGS = float(os.environ.get("NOX_TOWER_LEGS", "4.0"))   # leg r (0 = none): the single column bent at 1.08 sustained in run 3 (F)
SKIRT = float(os.environ.get("NOX_SKIRT", "6.0"))   # 45 deg skirt r 43 -> 49 under the plate (run 3: swap 0.523 at the sleeve-to-plate corner r 43.4, Y 47.6, F)
ENV_OVERLAP = float(os.environ.get("NOX_ENV_OVERLAP", "1.0"))                     # the generated region runs 1.0 into the plate (runs 1 / 2: the field met the plate's underside edge-on at Y 50
                                      # and left 15 sub-0.25 mm lips there: solid_check's face-connected body count 13 / 16)
LANE_Y = 53.0                         # arm v7 round 3d: the wheel lanes' floor (the axle tails at Y 54.75 pass 1.75 over it)
SEAT_SKIP = {("axle", -1, -1)}      # the axle motor's (-1, -1) M3 sits under the detent springs (key line blocked, swap_system access): 3 x M3
                                    # hold it (85 N + 0.6 N.m worst, E) and its tension slide stays free (DEFAULT; turning the square 40 deg to
                                    # clear it puts a corner at r 39.6 into the sleeve r 43)


def pads():
    """[(head, r, pad Manifold, insert hole)] in the arm frame (A along +Z, B along -X)"""
    out = []
    for h in ("A", "B"):
        for r in V.RAIL_BOLTS["r_" + h]:
            z0, z1 = max(r - PAD_L, PAD_R0), r + PAD_L
            if h == "A":
                p = box(-PAD_W, PAD_W, L.Y_PLATE0, L.Y_PLATE1, z0, z1) + cylY(0, r, BOSS_Y, L.Y_PLATE1, BOSS_R, 48)
                hole = cylY(0, r, L.Y_PLATE1 - INSERT["depth"], L.Y_PLATE1 + 1, INSERT["d"] / 2, 48)
            else:
                p = box(-z1, -z0, L.Y_PLATE0, L.Y_PLATE1, -PAD_W, PAD_W) + cylY(-r, 0, BOSS_Y, L.Y_PLATE1, BOSS_R, 48)
                hole = cylY(-r, 0, L.Y_PLATE1 - INSERT["depth"], L.Y_PLATE1 + 1, INSERT["d"] / 2, 48)
            out.append((h, r, p, hole))
    return out

TIE_BOSS_R = 5.0                    # boss round each Ø3.3 tie bore (1.85 + a 3.15 wall, the boss rule), from 3 behind the rod end to its 10 deep end


def tie_sockets(heads=("A", "B")):
    """v7 (arm_v7_layout active): the carbon tie rods' hub ends (arm_v6_layout.TIE, Y 22.5, r 78, X +-5 on each head's axis): a boss and a
    Ø3.3 x 10 bore along each rod, and a 1.0 keep-out round the rest of the rod. Returns (bosses, cuts) or (None, None) without v7."""
    import arm_v7_layout as S7
    if not S7._DONE: return None, None
    import arm_v6_cad as C6
    T = S6.TIE; bosses = cuts = None
    for h in heads:
        for sd in (-1, 1):
            p0, p1 = (np.asarray(p, float) for p in S6.tie_rod(h, sd)); d = (p1 - p0) / np.linalg.norm(p1 - p0)
            b = C6.cyl_between(p0 - 3.0 * d, p0 + T["depth"] * d, TIE_BOSS_R, 32)
            c = C6.cyl_between(p0 - 0.5 * d, p0 + T["depth"] * d, T["bore_d"] / 2, 24) + C6.cyl_between(p0 + T["depth"] * d, p1 + 2 * d, T["rod_d"] / 2 + 1.0, 24)
            bosses = b if bosses is None else bosses + b; cuts = c if cuts is None else cuts + c
    return bosses, cuts


def hub_v6_parts():
    """(drawn, cuts, env, allowed, ...) in W (arm at phi 90). Same build as hub v5: bed at Y 7 (sleeve end), plate up."""
    y0 = L.Y_HUB0
    bore = cylY(0, 0, y0 - 1, 40.0, L.BRG_OD / 2 + 0.10, 256)
    sleeve = cylY(0, 0, y0, L.Y_BRG[1] + L.BRG_W, SH_R) - cylY(0, 0, y0 - 1, 41, L.BRG_OD / 2 + 0.10, 256)
    wall = cylY(0, 0, 40.0, ROOF_Y[0] + 0.01, SH_R) - cylY(0, 0, 39, ROOF_Y[0] + 0.5, CAV_R) - cylY(0, 0, 39, SEAL_Y, SEAL_R, 128)   # outer-race shoulder + cavity wall
    roof = cylY(0, 0, ROOF_Y[0], ROOF_Y[1] + 0.01, SH_R, 128)                                    # cavity roof (v5: generated films)
    if SKIRT:                                                                                    # system pass: a 45 deg skirt from the sleeve wall to the
        rg = lambda r, y: [[r * math.cos(t), y, r * math.sin(t)] for t in np.linspace(0, 2 * math.pi, 128, endpoint=False)]   # plate's underside
        roof += hull(rg(SH_R, ROOF_Y[1] - SKIRT) + rg(SH_R + SKIRT, ROOF_Y[1] - 2.5) + rg(SH_R + SKIRT, ROOF_Y[1]) + rg(SH_R, ROOF_Y[1]))   # 2.5 rim
    plate_c = cylY(0, 0, ROOF_Y[1], L.HUB_RECESS_Y, 30.0)                                        # web disc up to the recess floor Y 53 (v5 to 54: the 1.0 lip)
    for M in MOTORS6.values():                                                                    # spokes 36 wide to each seat (as v5)
        r = math.hypot(M["xc"], M["zc"]); ux, uz = M["xc"] / r, M["zc"] / r; px, pz = -uz, ux
        plate_c += hull([[s * px * 18.0, y, s * pz * 18.0] for s in (-1, 1) for y in (50.0, 54.0)] +
                        [[M["xc"] + s * px * 18.0, y, M["zc"] + s * pz * 18.0] for s in (-1, 1) for y in (50.0, 54.0)])
    P = pads()
    pst = None; holes = None
    for h, r, p, hole in P:
        pst = p if pst is None else pst + p
        holes = hole if holes is None else holes + hole
    seats = None; seat_cuts = None
    for nm, M in MOTORS6.items():
        xc, zc = M["xc"], M["zc"]
        sl = L.AX_SLOT if nm == "axle" else 0.0
        rr = math.hypot(xc, zc); ux, uz = xc / rr, zc / rr
        def slot(f):
            return f(0.0, 0.0) if sl == 0 else (f(-sl * ux, -sl * uz) + f(sl * ux, sl * uz)).hull()
        s = cylY(xc, zc, L.Y_PLATE0, L.Y_PLATE1, 26.5)
        c = slot(lambda dx, dz: cylY(xc + dx, zc + dz, L.Y_PLATE0 - 1, L.Y_PLATE1 + 1, L.NEMA_BOSS / 2 + 0.5))
        body_k = rsquare(M, L.NEMA / 2 + 1.0, 0.0, L.Y_PLATE0)
        c += body_k if sl == 0 else (body_k.translate([-sl * ux, 0, -sl * uz]) + body_k.translate([sl * ux, 0, sl * uz]))
        for sx in (-1, 1):
            for sz in (-1, 1):
                if (nm, sx, sz) in SEAT_SKIP: continue
                hx, hz = rot_pt(M, sx * L.NEMA_HOLES / 2, sz * L.NEMA_HOLES / 2)
                c += slot(lambda dx, dz: cylY(hx + dx, hz + dz, L.Y_PLATE0 - 1, L.Y_PLATE1 + 1, 1.7))
                s += slot(lambda dx, dz: cylY(hx + dx, hz + dz, L.Y_PLATE0, L.Y_PLATE1, 4.5))
        seats = s if seats is None else seats + s
        seat_cuts = c if seat_cuts is None else seat_cuts + c
    post_k = cylY(0, 0, 39.0, 56.0, L.POST_HOLE_R, 128) + cylY(0, 0, L.HUB_RECESS_Y, 80.0, L.HUB_RECESS_R, 128)
    D, S = detent6()
    Q = DT.chosen()["Q"]
    tower = S["tower"] + cylY(Q[0], Q[1], L.Y_PLATE0, L.Y_PLATE1, 7.0, 48)      # + a plate pad under the column (joins the spoke)
    ring = lambda r, y: [[Q[0] + r * math.cos(t), y, Q[1] + r * math.sin(t)] for t in np.linspace(0, 2 * math.pi, 48, endpoint=False)]
    tower += hull(ring(COL_FLARE[0], L.Y_PLATE1 - 0.5) + ring(5.0, L.Y_PLATE1 + COL_FLARE[1]))   # column base flare (hub v6 run 2: sustained 1.33 at its root)
    w0 = DT.WEB_Y[0]
    for pt in ((D["P"], D["A"]) if TOWER_LEGS else ()):                         # system pass (three strikes, runs 1..3): the tower becomes a frame,
        tower += cylY(pt[0], pt[1], L.Y_PLATE0, w0 + 0.01, TOWER_LEGS, 32) + cylY(pt[0], pt[1], L.Y_PLATE0, L.Y_PLATE1, 6.0, 48)   # legs under the pivot
                                                                                 # boss and the anchor post, each on a plate pad like the column's
    mov = S["arm"] + S["spacer"] + S["nut"] + S["roller"] + S["springs"][0] + S["springs"][1]
    ties, tie_cuts = tie_sockets()
    drawn = (sleeve + wall + roof + plate_c + seats + pst + tower + (ties if ties is not None else mf.Manifold())) - post_k
    cav = cylY(0, 0, 39.9, ROOF_Y[0], CAV_R, 128) + cylY(0, 0, 39.9, SEAL_Y, SEAL_R, 128)
    rails = None                                                                 # the rails sit on the pads: keep the hub off them elsewhere
    for nm, poly, yy in V.rail_bodies("A") + V.rail_bodies("B"):
        m = prism(poly, *yy); rails = m if rails is None else rails + m
    drop = cylY(V.DROP["x"], V.DROP["z"], L.Y_PLATE0 - 6.0, L.Y_PLATE1 + 1, V.DROP["r"] + 0.5, 32)   # harness drop through the plate (box-wiring: Ø9, C)
    lanes = None                                                                 # arm v7 round 3d (low stack): the wheel-axle tails (Y 54.75) and thin
    if getattr(V, "RAIL_KEEL", None):                                            # nuts pass over the plate: lanes X +-(13..31) along each rail at Y 53
        for h in ("A", "B"):
            for sx in (-1, 1):
                x0, x1 = sorted((sx * 13.0, sx * 31.0))
                m = prism(V.to_arm(h, V.rect(x0, x1, V.RAIL_R[0] - 10.0, 92.0)), LANE_Y, L.Y_PLATE1 + 6.0)
                lanes = m if lanes is None else lanes + m
    cuts = bore + cav + seat_cuts + post_k + mov + holes + rails + drop + (tie_cuts if tie_cuts is not None else mf.Manifold()) + (lanes if lanes is not None else mf.Manifold())
    if lanes is not None: drawn -= lanes
    env = cylY(0, 0, y0, L.Y_PLATE0 + ENV_OVERLAP, 110.0) ^ box(-88.0, 88.0, y0 - 1, L.Y_PLATE0 + 1 + ENV_OVERLAP, -88.0, 88.0)
    env -= bore + cav + holes + drop
    env -= seat_cuts
    env0 = env - post_k                                                          # the generated region as given (topo organic finish)
    env = (env + pst + plate_c + seats + tower + roof + wall + (ties if ties is not None else mf.Manifold())) - post_k
    return dict(ties=ties, drawn=drawn, cuts=cuts, env=env, env0=env0, allowed=env - cuts, pads=P, rails=rails, seat_cuts=seat_cuts, detent=(D, S), tower=tower)

# ------------------------------------------------------------------ v5 parts as solids (arm frame), for the checks
def prism(poly, y0, y1):
    """2D polygon in (X, Z) extruded along Y"""
    pts = [(x, -z) for x, z in poly]
    cs = mf.CrossSection([pts], mf.FillRule.NonZero)
    return mf.Manifold.extrude(cs, y1 - y0).rotate([-90, 0, 0]).translate([0, y0, 0])

def v5_static():
    ax = (MOTORS6["axle"]["xc"], MOTORS6["axle"]["zc"])
    out = {}
    for nm, poly, y in V.rail_bodies("A") + V.rail_bodies("B") + V.arm_bodies(ax) + V.fixed_bodies():
        if nm.startswith("post"): continue                                   # the hub's own posts
        m = prism(poly, *y); out[nm] = out[nm] + m if nm in out else m
    for nm, M in MOTORS6.items():                                            # motor bodies Y 2..50 and shafts
        out[f"motor{nm}"] = rsquare(M, L.NEMA / 2, 2.0, 50.0)
    return out

def v5_heads(zA, zB):
    out = {}
    for h, z in (("A", zA), ("B", zB)):
        for nm, poly, y in V.head_bodies(h, z):
            m = prism(poly, *y); out[nm] = out[nm] + m if nm in out else m
    return out

def checks(H=None, verbose=True):
    """every gap of the hub v6 drawn bodies (posts, tower, plate) and the detent's moving parts to the v5 parts over 4 x 4 stroke
    states (C). Returns {pair: min gap}."""
    H = H or hub_v6_parts()
    D, S = H["detent"]
    hubm = H["drawn"] - H["cuts"]
    above = hubm ^ box(-200, 200, 50.0, 200, -200, 200)                      # the hub above the plate's underside (posts, tower)
    det = dict(arm=S["arm"] + S["spacer"], hook=S["hook"], springs=S["springs"][0] + S["springs"][1], anchor=S["anchor"],
               roller=S["roller"] + S["nut"])
    st = v5_static()
    touch = {("roller", "ring"), ("hub", "motorA"), ("hub", "motorB"), ("hub", "motoraxle"), ("hub", "railA:body"), ("hub", "railB:body")}
    touch |= {("hub", f"rail{h}:foot{r:g}") for h in "AB" for r in V.RAIL_BOLTS["r_" + h]}   # seats carry the motors, pads the rails (contact at Y 54)
    gaps = {}
    def rec(a, b, g):
        k = (a, b); gaps[k] = min(gaps.get(k, 1e9), g)
    for nm, m in st.items():
        if ("hub", nm) not in touch: rec("hub", nm, above.min_gap(m, 6.0))
        for dn, dm in det.items():
            if (dn, nm) in touch: continue
            rec(dn, nm, dm.min_gap(m, 6.0))
    for zA in V.Z0S:
        for zB in V.Z0S:
            for nm, m in v5_heads(zA, zB).items():
                rec("hub", nm, above.min_gap(m, 6.0))
                for dn, dm in det.items(): rec(dn, nm, dm.min_gap(m, 6.0))
    for dn, dm in det.items():                                               # the turning detent parts vs the hub's own tower / posts
        if dn in ("anchor", "hook"): continue
        tw = H["tower"] - cylY(D["P"][0], D["P"][1], 0, 200, 6.0) if dn == "arm" else H["tower"]
        rec(dn, "tower", (tw - H["cuts"]).min_gap(dm, 6.0))
    if verbose:
        for (a, b), g in sorted(gaps.items(), key=lambda kv: kv[1])[:16]:
            print(f"  {g:6.2f}  {a} / {b}")
    return gaps

if __name__ == "__main__":
    H = hub_v6_parts()
    d = H["drawn"] - H["cuts"]
    print("motors v6", {k: (round(v["xc"], 2), round(v["zc"], 2), round(v["rot"], 1)) for k, v in MOTORS6.items()})
    print("detent", {k: v for k, v in DET6.items()}, "roller C", np.round(H["detent"][0]["C"], 1), "P", np.round(H["detent"][0]["P"], 1),
          "H", np.round(H["detent"][0]["H"], 1), "anchor", np.round(H["detent"][0]["A"], 1))
    print(f"drawn {d.volume() * 1.27e-3:.1f} g, bodies {len(d.decompose())}, env {H['env'].volume() / 1000:.0f} cm3")
    import trimesh
    import sys; sys.path.insert(0, os.path.join(REPO, "nox"))
    from solid_check import solid_check
    t = A.tm(d)
    ok, info = solid_check(t)
    print("drawn solid_check", "PASS" if ok else "FAIL", info)
    print("gaps (C):")
    checks(H)
