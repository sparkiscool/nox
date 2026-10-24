"""Hub v6 (24 Oct 2026, hub + heads worker): hub v5 rebuilt for arm v5 (printed V-rails on the posts, rack and pinion heads).
Changes from hub v5 (arm_cad.hub_parts), every number C unless tagged E:
  - motor seats at the v5 poses (arm_v5_layout.MOTORS_V5 + the axle pose of its balance search, runs/arm/v5-layout/result.json):
    A (53.5, 55) 45.8 deg, B (-55, -53.5) -135.8 deg, axle (53.9, -43.7) -39 deg, seats slotted +-1 for the axle loop as before;
  - detent tower moved for the v5 heads: roller at arm angle -45 (ring v1 unchanged, notches every 30), pivot side -1, the
    candidate of box-arm.md v5 (until the mechanism worker lands its search: arm_v5_layout.DETENT or detent.CHOSEN_V5 wins);
  - rail posts as v5 (A r 35 / 76, B r 38.75 / 76, 32 wide, Y 52..74) but 14 long along the rail (+2 outward, the inner faces
    stay 1.5 off the 80T flange / 1.04 off the loop: 3.8 wall round the insert, DEFAULT) with ONE M5 heat-set insert hole each at
    X 0 (arm_v5_layout.RAIL_BOLTS, the arm-mechanism worker's 24 Oct interface; v5 had 2 blind M5 holes at X +-10);
  - the hub v5 solid_check FAIL fixed in the design: the 1.0 lip (r 29..30, Y 53..54, the web disc above the 80T recess floor)
    is gone (web disc top at Y 53), and the bore cavity gets a drawn roof (Y 47.5..50) and wall (r 36..43, Y 40..47.5) instead
    of the generated 0.3 films at Y 48..49 (box-arm Problem 9).
python3 hub_v6.py  -> drawn-only checks (solid, mass) + every gap to the v5 parts over the stroke (C)."""
import math, json, os, itertools
import numpy as np, manifold3d as mf
import arm_layout as L, arm_cad as A, detent as DT, arm_v5_layout as V
from arm_cad import box, cylY, hull, rsquare, rot_pt

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))

# the arm-mechanism worker's 24 Oct layout changes (relayed by the lead; its PR lands arm_v5_layout.RAIL_BOLTS with them). Until that
# file is on main the same numbers are set here, so every check below runs on the layout the rails are drawn to.
if not hasattr(V, "RAIL_BOLTS"):
    V.RAIL_Y = (74.0, 93.0); V.RAIL_R = (29.0, 140.0)
    V.RAIL_CHAN = dict(x=(-2.0, 2.0), y=(89.0, 93.0), mouth=3.2)
    V.RAIL_BOLTS = dict(x=0.0, r_A=(35.0, 76.0), r_B=(38.75, 76.0), cbore_r=4.5, cbore_y=84.0, hole_r=2.75, bolt="M5 x 16 ISO 4762",
                        post_insert=dict(hole_d=6.4, depth=8.0, insert="M5 x 5.8 heat-set (OD 7.0, E)"))
    V.RAIL_INSERTS = V.RAIL_BOLTS
    V.WEB_U = (-47.0, 30.0); V.RACK_LEN = 77.0
    V.DRONE_PUSH = 6.0
    V.Z0S = tuple(V.Z0S) + (L.Z0_BAY + 5.0 + 6.0,)
    V.ECC = dict(x=-22.0, u=24.0, shank_r=5.5, bore_in_plate_r=5.6, plate_boss_r=7.6, flange_af=13.0, flange_y=(98.0, 100.5), throw=0.8, bolt_r=2.65)
    V.AXLE_KEEP = dict(r=5.0, y=(98.0, 106.0)); V.ECC_KEEP = dict(r=13.0, y=(98.0, 101.0))

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
DET6 = dict(getattr(V, "DETENT", None) or getattr(DT, "CHOSEN_V5", None) or DET_CAND)

def detent6():
    c = DET6
    D = DT.design(c["th"], c["s"], c["a"], c["b"], c["phi_d"], c["psi"])
    S = DT.arm_solids(D, np.array(c["Q"]))
    return D, S

# rail posts (v5 positions, 14 long along the rail, inner face kept)
POST_W = 16.0                         # X +-16 (as v5)
POST_Y = (52.0, 74.0)                 # as v5: rail foot on top at Y 74
POST_U = dict(inner=(-6.0, 8.0), outer=(-7.0, 7.0))    # along the rail about the bolt row (inner: face fixed at r - 6)
_RB = V.RAIL_BOLTS                    # arm-mech 24 Oct: ONE M5 x 16 per post from the rail top at X 0 into an M5 heat-set in the post top
RAIL_BOLTS = dict(x=(float(_RB["x"]),), insert_d=float(_RB["post_insert"]["hole_d"]), insert_l=float(_RB["post_insert"]["depth"]))   # hole Ø6.4 x 8 (E)
ROOF_Y = (47.5, 50.0)                 # drawn roof over the bore cavity (2.5, joins the motor plate)
CAV_R = 36.0                          # cavity r (sun post flange r 28.5 + bearing 2 inner race), as v5
SH_R = 43.0                           # shoulder / cavity wall outer r (= the sleeve)

def posts():
    """[(head, r, Manifold, footprint)] posts in the arm frame (A along +Z, B along -X)"""
    out = []
    for h, rs in (("A", L.POSTS_A), ("B", L.POSTS_B)):
        for k, r in enumerate(rs):
            u0, u1 = POST_U["inner" if k == 0 else "outer"]
            if h == "A":
                p = box(-POST_W, POST_W, *POST_Y, r + u0, r + u1)
                for x in RAIL_BOLTS["x"]:
                    p -= cylY(x, r, POST_Y[1] - RAIL_BOLTS["insert_l"], POST_Y[1] + 1, RAIL_BOLTS["insert_d"] / 2, 48)
                fp = hull([[x, y, zz] for x in (-18.0, 18.0) for y in (50.0, 52.0) for zz in (r + u0 - 2.0, r + u1 + 2.0)] +
                          [[x, y, zz] for x in (-18.0, 18.0) for y in (50.0, 52.0) for zz in (-12.0, 12.0)])
            else:
                p = box(-r - u1, -r - u0, *POST_Y, -POST_W, POST_W)
                for z in RAIL_BOLTS["x"]:
                    p -= cylY(-r, z, POST_Y[1] - RAIL_BOLTS["insert_l"], POST_Y[1] + 1, RAIL_BOLTS["insert_d"] / 2, 48)
                fp = hull([[xx, y, z] for xx in (-r - u1 - 2.0, -r - u0 + 2.0) for y in (50.0, 52.0) for z in (-18.0, 18.0)] +
                          [[xx, y, z] for xx in (-12.0, 12.0) for y in (50.0, 52.0) for z in (-18.0, 18.0)])
            out.append((h, r, p, fp))
    return out

def hub_v6_parts():
    """(drawn, cuts, env, allowed, ...) in W (arm at phi 90). Same build as hub v5: bed at Y 7 (sleeve end), plate up."""
    y0 = L.Y_HUB0
    bore = cylY(0, 0, y0 - 1, 40.0, L.BRG_OD / 2 + 0.10, 256)
    sleeve = cylY(0, 0, y0, L.Y_BRG[1] + L.BRG_W, SH_R) - cylY(0, 0, y0 - 1, 41, L.BRG_OD / 2 + 0.10, 256)
    wall = cylY(0, 0, 40.0, ROOF_Y[0] + 0.01, SH_R) - cylY(0, 0, 39, ROOF_Y[0] + 0.5, CAV_R)     # shoulder + cavity wall (v5: 40..44)
    roof = cylY(0, 0, ROOF_Y[0], ROOF_Y[1] + 0.01, SH_R, 128)                                    # cavity roof (v5: generated films)
    plate_c = cylY(0, 0, ROOF_Y[1], L.HUB_RECESS_Y, 30.0)                                        # web disc up to the recess floor Y 53 (v5 to 54: the 1.0 lip)
    for M in MOTORS6.values():                                                                    # spokes 36 wide to each seat (as v5)
        r = math.hypot(M["xc"], M["zc"]); ux, uz = M["xc"] / r, M["zc"] / r; px, pz = -uz, ux
        plate_c += hull([[s * px * 18.0, y, s * pz * 18.0] for s in (-1, 1) for y in (50.0, 54.0)] +
                        [[M["xc"] + s * px * 18.0, y, M["zc"] + s * pz * 18.0] for s in (-1, 1) for y in (50.0, 54.0)])
    P = posts()
    pst = None
    for h, r, p, fp in P:
        plate_c += fp
        pst = p if pst is None else pst + p
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
                hx, hz = rot_pt(M, sx * L.NEMA_HOLES / 2, sz * L.NEMA_HOLES / 2)
                c += slot(lambda dx, dz: cylY(hx + dx, hz + dz, L.Y_PLATE0 - 1, L.Y_PLATE1 + 1, 1.7))
                s += slot(lambda dx, dz: cylY(hx + dx, hz + dz, L.Y_PLATE0, L.Y_PLATE1, 4.5))
        seats = s if seats is None else seats + s
        seat_cuts = c if seat_cuts is None else seat_cuts + c
    post_k = cylY(0, 0, 39.0, 56.0, L.POST_HOLE_R, 128) + cylY(0, 0, L.HUB_RECESS_Y, 80.0, L.HUB_RECESS_R, 128)
    D, S = detent6()
    Q = DET6["Q"]
    tower = S["tower"] + cylY(Q[0], Q[1], L.Y_PLATE0, L.Y_PLATE1, 7.0, 48)      # + a plate pad under the column (joins the spoke)
    mov = S["arm"] + S["spacer"] + S["nut"] + S["roller"] + S["springs"][0] + S["springs"][1]
    drawn = (sleeve + wall + roof + plate_c + seats + pst + tower) - post_k
    cav = cylY(0, 0, 39.9, ROOF_Y[0], CAV_R, 128)
    cuts = bore + cav + seat_cuts + post_k + mov
    env = cylY(0, 0, y0, L.Y_PLATE0, 110.0) ^ box(-88.0, 88.0, y0 - 1, L.Y_PLATE0 + 1, -88.0, 88.0)
    env -= bore + cav
    env -= seat_cuts
    env0 = env - post_k                                                          # the generated region as given (topo organic finish)
    env = (env + pst + plate_c + seats + tower + roof + wall) - post_k
    return dict(drawn=drawn, cuts=cuts, env=env, env0=env0, allowed=env - cuts, posts=P, seat_cuts=seat_cuts, detent=(D, S), tower=tower)

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
    touch = {("hub", "railA:foot35"), ("hub", "railA:foot76"), ("hub", "railB:foot38.75"), ("hub", "railB:foot76"),
             ("roller", "ring"), ("hub", "motorA"), ("hub", "motorB"), ("hub", "motoraxle")}   # seats carry the motors, posts the rail feet
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
    from solid_check import solid_check
    t = A.tm(d)
    ok, info = solid_check(t)
    print("drawn solid_check", "PASS" if ok else "FAIL", info)
    print("gaps (C):")
    checks(H)
