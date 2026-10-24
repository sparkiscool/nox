"""Heads A / B v7 body on the topo engine (24 Oct 2026, heads v7 part session). One print: the carriage (plate, wing, rack web with
its m1 teeth, the spine under the wing) and the column that carries the grabber and the servo, cut from the arm system at its joints
(docs/generative.md The rule, 0; requirements dimensions "System box swap: arm", layout head_v7_layout.py):
  - envelope = the layout's design space (the arm's free space over 4 swap cycles, the park bisector, minus every moving part + 1.0,
    the cartridge zone above u 15.4, the tool and wire paths);
  - drawn = the interfaces: plate + wing + web + spine, the eccentric boss, the deck with the push pads and the hall pockets, the rod
    walls with blind sockets, the spring rod walls, the servo ear pads with M3 heat-set holes, the sync pinion's post;
  - cuts = axle bores and countersinks, the eccentric bore and flange counterbore, the rack's tooth gaps, the rod sockets, the hall
    pockets, the inserts, the pinion pin, the bumper notch, the wire hole and every keep-out;
  - supports = the three wheel bores and the rack teeth where the pinion meets them in the case (drone insert u -56, bay u -47,
    park u 23);
  - loads = the head's interface loads per case (head_v7_layout.loads_table, E): insert 60 N (swap) / 74 N stall (misuse) on the
    pads, the pull 57.3 N (swap) on the four rod sockets, handling 3 g + 2 g with the Overseer cartridge (pads + the pinion post),
    the servo's stall torque 1.96 N.m on the ear pads (misuse).
Classes as hub_v6_topo: swap -> flight (SF 4), misuse / handling -> misuse (2.5). Build +x (the part on its -X flank: the column's
bending along Y and the plate's along u both lie in the layers; the upper bound reads 0.44 in the swap class against 0.61..0.70 with Y
or u up; the rack teeth face +X, up). Head B is solved in the A frame with the MG995 cradle and B's shorter rack tail, then mirrored.
python3 head_v7_topo.py [A|B] [h] [gen_cm3] [out_dir]  -> <out_dir>/box_head_v7_<A|B>.stl (head frame) + result.json. NOX_SETUP_ONLY=1."""
import sys, os, math, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
import head_v7_layout as H
from arm_cad import box, cylX, cylY, cylZ, hull
from topo import Part, Settings, optimise

HEAD = sys.argv[1] if len(sys.argv) > 1 else "A"
SERVO = dict(A="DS3218", B="MG995")[HEAD]
HH = float(sys.argv[2]) if len(sys.argv) > 2 else 1.5
GEN = float(sys.argv[3]) if len(sys.argv) > 3 else 30.0
CLS = dict(swap="flight", misuse="misuse", handling="misuse")
PIN_U = dict(drone=55.0 - 111.3, bay=55.0 - 101.8, park=55.0 - 32.0)        # pinion u = PIN_Z - z0 (arm_v5_layout), commanded strokes


def web_u():
    return H.WEB["uB"] if SERVO == "MG995" else H.WEB["uA"]


def rack_gaps():
    """the m1 rack's tooth gaps on the web's +X face (as arm_v5_cad.carriage: tips X 46.5, root 44.25, phase from u -63)"""
    import arm_v5_layout as V
    p = math.pi * 1.0; xt, xr = H.WEB["x"][1], 45.5 - 1.25
    g = None; u = H.WEB["uA"][0] + p / 2
    while u < H.WEB["uA"][1] - p / 2:
        wt = p / 2 + 0.1 + 2 * 1.0 * math.tan(math.radians(20)); wr = p / 2 + 0.1 - 2 * 0.25 * math.tan(math.radians(20))
        if u > web_u()[0]:
            k = hull([[xt + 0.01, y, u + s * wt / 2] for s in (-1, 1) for y in (H.WEB["teeth_y"][0] - 0.01, H.WEB["teeth_y"][1])] +
                     [[xr, y, u + s * wr / 2] for s in (-1, 1) for y in (H.WEB["teeth_y"][0] - 0.01, H.WEB["teeth_y"][1])])
            g = k if g is None else g + k
        u += p
    return g


def drawn_and_cuts():
    W, P = H.WEB, H.PLATE
    wu = web_u()
    plate = box(*P["x"], *P["y"], *P["u"]) + box(*H.WING["x"], *P["y"], *H.WING["u"])
    web = box(*W["x"], W["y"][0], W["teeth_y"][1], *wu) - box(W["x"][0] - 1, W["x"][1] + 1, W["y"][0] - 1, W["tail_y0"], wu[0] - 1, W["tail_u"])
    spine = box(35.0, W["x"][0] + 0.01, W["y"][0], P["y"][0] + 0.01, 0.0, P["u"][1])
    ecc = cylY(H.WHEELS[2][0], H.WHEELS[2][1], *P["y"], H.WHEEL["ecc_boss_r"], 48)
    deck = box(-H.DECK["x"], H.DECK["x"], H.DECK["y"][0], H.DECK["y"][1], *H.DECK["u"])
    for s in (-1, 1):
        a, b = sorted((s * H.DECK["pad"]["x"][0], s * H.DECK["pad"]["x"][1]))
        deck += box(a, b, *H.DECK["pad"]["y"], H.DECK["u"][1] - 0.01, H.DECK["pad"]["u_top"])
    walls = None
    for s in (-1, 1):
        a, b = sorted((s * H.RODS["wall_in"], s * H.RODS["wall_out"]))
        w = box(a, b, H.BOSS["y"][0], H.BOSS["y"][1], H.BOSS["u"][0], H.BOSS["u"][1])
        a, b = sorted((s * H.SPRING_ROD["wall_in"], s * H.SPRING_ROD["wall_out"]))
        w += box(a, b, H.SPRING_ROD["y"] - 4.5, H.SPRING_ROD["y"] + 4.5, H.SPRING_ROD["u"] - 3.5, H.SPRING_ROD["u"] + 4.0)
        walls = w if walls is None else walls + w
    pads, pad_cuts = H.cradle(SERVO)
    post = cylZ(H.PINION["x"], H.PINION["y"], H.U_C - 9.0, H.PINION["u"][0] - 0.5, H.PINION["post_r"])
    drawn = plate + web + spine + ecc + deck + walls + pads + post
    # cuts
    cuts = []
    for (x, u) in H.WHEELS[:2]:
        cuts.append(cylY(x, u, P["y"][0] - 1, P["y"][1] + 1, H.WHEEL["axle_r"], 40))
        cuts.append(hull([[x + 5.2 * math.cos(a), P["y"][1] + 0.01, u + 5.2 * math.sin(a)] for a in np.linspace(0, 2 * math.pi, 32, endpoint=False)] +
                         [[x + 2.6 * math.cos(a), P["y"][1] - 2.6, u + 2.6 * math.sin(a)] for a in np.linspace(0, 2 * math.pi, 32, endpoint=False)]))   # 90 deg countersink
    x, u = H.WHEELS[2]
    cuts.append(cylY(x, u, P["y"][0] - 1, P["y"][1] + 1, H.WHEEL["ecc_bore_r"], 48))
    cuts.append(cylY(x, u, H.WHEEL["ecc_flange"][0], P["y"][1] + 1, 7.6, 48))                             # flange AF 13 (round 15.2) counterbore
    cuts.append(rack_gaps())
    for yr in H.RODS["y"]:
        cuts.append(cylX(yr, H.RODS["u"], -H.RODS["wall_out"] + 1.0, H.RODS["wall_out"] + 1.0, H.ROD_D / 2 + 0.05, 24))   # -X blind, +X open
    cuts.append(cylX(H.SPRING_ROD["y"], H.SPRING_ROD["u"], -H.SPRING_ROD["wall_out"] + 1.0, H.SPRING_ROD["wall_out"] + 1.0, H.ROD_D / 2 + 0.05, 24))
    for (xh, yh) in H.HALL_POS:
        cuts.append(box(xh - 1.6, xh + 1.6, yh - 1.6, yh + 1.6, H.DECK["u"][0] - 1.0, H.U_BOT - H.LATCH["hall"]["gap"]))
        cuts.append(box(xh - 2.5, xh + 2.5, yh - 3.5, yh + 1.0, H.DECK["u"][0] - 6.0, H.DECK["u"][0] + 0.01))   # the hall board (5 x 6.5) under the deck
    cuts.append(pad_cuts)
    cuts.append(cylZ(H.PINION["x"], H.PINION["y"], H.U_C - 6.0, H.PINION["u"][0] + 0.01, H.PINION["pin_d"] / 2 + 0.05, 24))
    cuts.append(box(*H.BUMPER["x"], P["y"][0] - 1, P["y"][1] + 1, H.BUMPER["u"][0] - 1, H.BUMPER["u"][1]))
    cuts += H.keepouts(SERVO)
    cut = cuts[0]
    for c in cuts[1:]: cut = cut + c
    return drawn, cut


def setup(h):
    drawn, cuts = drawn_and_cuts()
    env = H.design_space(SERVO)
    pp = Part(f"box_head_v7_{HEAD}", h, drawn, cuts, env, build=os.environ.get("NOX_BUILD", "+x"))
    bores = None
    for (x, u) in H.WHEELS:
        r = H.WHEEL["ecc_bore_r"] if (x, u) == H.WHEELS[2] else H.WHEEL["axle_r"]
        b = cylY(x, u, H.PLATE["y"][0], H.PLATE["y"][1], r + 2.0) - cylY(x, u, H.PLATE["y"][0] - 1, H.PLATE["y"][1] + 1, r)
        bores = b if bores is None else bores + b
    for nm, pu in PIN_U.items():
        if pu < web_u()[0] + 3: pu = web_u()[0] + 3
        pp.support(nm, bores + box(44.0, H.WEB["x"][1], H.WEB["teeth_y"][0], H.WEB["teeth_y"][1], pu - 4.0, pu + 4.0))
    pads = None
    for s in (-1, 1):
        a, b = sorted((s * H.DECK["pad"]["x"][0], s * H.DECK["pad"]["x"][1]))
        p = box(a, b, *H.DECK["pad"]["y"], H.DECK["u"][1], H.DECK["pad"]["u_top"])
        pads = p if pads is None else pads + p
    sock = None
    for s in (-1, 1):
        a, b = sorted((s * H.RODS["wall_in"], s * H.RODS["wall_out"]))
        for yr in H.RODS["y"]:
            c = cylX(yr, H.RODS["u"], a, b, H.ROD_D / 2 + 1.2, 24)
            sock = c if sock is None else sock + c
    post = cylZ(H.PINION["x"], H.PINION["y"], H.U_C - 6.0, H.PINION["u"][0] - 0.5, H.PINION["post_r"])
    L = H.LOADS
    for where in (("drone", "insert at the drone"), ("bay", "insert at a bay")):
        pp.load(f"{where[1]} 60 N [swap]", pads, (0, 0, -L["insert"]), cls=CLS["swap"], support=where[0])
        pp.load(f"{where[1]}, stall 74 N [misuse]", pads, (0, 0, -L["stall"]), cls=CLS["misuse"], support=where[0])
        pp.load(f"pull at the {where[0]} 57.3 N while the XT60 unmates [swap]", sock, (0, 0, L["pull_unmate"]), cls=CLS["swap"], support=where[0])
    Ov = L["cart_g"]["overseer"] / 1000 * 9.81
    lever = 48.5 - H.U_BOT; cpl = 2 * Ov * lever / (2 * 18.25)                 # the 2 g lateral at the cartridge CG as a couple on the pads
    padp = box(H.DECK["pad"]["x"][0], H.DECK["pad"]["x"][1], *H.DECK["pad"]["y"], H.DECK["u"][1], H.DECK["pad"]["u_top"])
    padm = box(-H.DECK["pad"]["x"][1], -H.DECK["pad"]["x"][0], *H.DECK["pad"]["y"], H.DECK["u"][1], H.DECK["pad"]["u_top"])
    for sx in (1, -1):
        pp.load_multi(f"handling 3 g in + 2 g along {'+' if sx > 0 else '-'}X, Overseer cartridge carried [handling]",
                      [(pads, (0, 0, -3 * Ov)), (post, (sx * 2 * Ov, 0, 0)), (padp if sx > 0 else padm, (0, 0, -cpl)), (padm if sx > 0 else padp, (0, 0, cpl))],
                      cls=CLS["handling"], support="park")
    pp.load(f"handling 3 g out, Overseer cartridge carried (on the rods) [handling]", sock, (0, 0, 3 * Ov), cls=CLS["handling"], support="park")
    sv = H.SERVOS[SERVO]; T = sv["kgcm"] * 98.07; ye = H.Y_SERVO_TOP - sv["H"] + sv["ear_y"]
    hs = H.ear_holes(SERVO); f = T / (hs[2][0] - hs[0][0]) / 2
    pp.load_multi(f"{SERVO} stall torque {T / 1000:.2f} N.m on the ear pads [misuse]",
                  [(cylY(x, u, ye - 3.0, ye, 5.0, 24), (0, 0, f if x < 10 else -f)) for (x, u) in hs], cls=CLS["misuse"], support="park")
    pp.log(f"  head {HEAD} ({SERVO}): {len(pp.cases)} cases, supports {list(pp.supports)}")
    return pp


def bound(h=1.0):
    """the upper bound: the as-built FE (topo.build.strength) of the whole design space filled solid (+ drawn - cuts) on this setup's
    cases: if a case misses here, no body inside the envelope meets it (concept review r1: the 4.0 carriage plate)"""
    from topo.layout import Layout
    from topo.build import strength
    from arm_cad import tm
    p = setup(h)
    drawn, cuts = drawn_and_cuts()
    solid = tm((H.design_space(SERVO) + drawn) - cuts)
    L = Layout(p, volfrac=0.5, settings=Settings(reach_check="off", auto_sym=False, finish="drawn"), log=lambda s_: None)
    st = strength(L, solid)
    out = dict(h=h, mass_g=round(solid.volume * 1.27e-3, 1), worst=st["worst"], worst_case=st["worst_case"], worst_at=st["worst_at"], rows=st["rows"])
    out["build"] = os.environ.get("NOX_BUILD", "+x")
    json.dump(out, open(os.path.join(H.ROOT, "runs", "head", "v7", f"bound_{HEAD}_h{h:g}_{out['build']}.json"), "w"), indent=1)
    return out


if __name__ == "__main__":
    out = sys.argv[4] if len(sys.argv) > 4 else os.path.join(H.ROOT, "runs", "head", "v7", HEAD)
    if os.environ.get("NOX_BOUND"):
        r = bound(float(os.environ["NOX_BOUND"])); print(json.dumps({k: v for k, v in r.items() if k != "rows"})); [print(x) for x in r["rows"]]; sys.exit(0)
    if os.environ.get("NOX_SETUP_ONLY"):
        p = setup(HH); print("setup ok", p.shape, len(p.cases)); sys.exit(0)
    res = optimise(setup, HH, GEN, os.path.abspath(out), f"box_head_v7_{HEAD}", settings=Settings(), git=bool(os.environ.get("NOX_GIT")))
    print({k: res.get(k) for k in ("name", "solid_ok", "mass_g", "total_s")}, res["strength"]["worst"])
