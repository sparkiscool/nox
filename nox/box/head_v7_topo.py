"""Heads A / B v7 body on the topo engine (24 Oct 2026, heads v7 part session). One print: the carriage (plate, wing, the web the
separate m1 rack bar is epoxied to, the spine under the wing) and the column that carries the grabber and the servo, cut from the arm
system at its joints (docs/generative.md The rule, 0; requirements dimensions "System box swap: arm", layout head_v7_layout.py):
  - envelope = the layout's design space (the arm's free space over 4 swap cycles, the park bisector, minus every moving part + 1.0,
    the cartridge zone above u 15.4, the tool and wire paths, the rack bar);
  - drawn = the interfaces: plate + wing + web + spine, the eccentric boss, the deck with the push pads and the hall pockets, the rod
    walls with blind sockets, the spring rod walls, the carriers' inward stops, the 2 mm bed land at X -34, the tail rib, the retainer's insert bosses, the servo cradle with its four Ø12 ear pads;
  - cuts = axle bores and countersinks, the eccentric bore and flange counterbore, the rod sockets, the hall pockets and boards,
    the ear screw holes, the bumper notch and every keep-out (horn and cam channel, drivers, rod insertion bores, detent);
  - supports = the real interfaces: the three wheel bores hold X and Y, the rack bar's bonded face holds u and X over the pinion's
    patch in the case (drone insert u -56, bay u -47, park u 23);
  - loads = the head's interface loads per case (head_v7_layout.loads_table, E): insert 60 N (swap) / 74 N stall (misuse) on the
    pads, the pull 57.3 N (swap) on the four rod sockets, handling 3 g + 2 g with the Overseer cartridge (pads, stops, sockets),
    43.5 N across at a seat (swap, on the stops), the centring 7.6 N / 1.05 N.m (swap), the servo's stall torque on the ear pads
    (misuse); power cut carrying is braked (3 N on the bumper, the system's required head brake), under the handling cases.
Classes as hub_v6_topo: swap -> flight (SF 4), misuse / handling -> misuse (2.5). Build +x (the part on its -X flank: the column's
bending along Y and the plate's along u both lie in the layers). Head B is solved in the A frame with the MG995 cradle and B's
shorter web tail, then mirrored.
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
if HEAD == "B": PIN_U["drone"] = -47.6                                      # B only pulls at the drone (the dead pack), at u -47.6 (concept review r4)


def web_u():
    return H.WEB["uB"] if SERVO == "MG995" else H.WEB["uA"]


def drawn_and_cuts():
    W, P = H.WEB, H.PLATE
    wu = web_u()
    plate = box(*P["x"], *P["y"], *P["u"]) + box(*H.WING["x"], *P["y"], *H.WING["u"]) + H.prism_xu(H.FILLET, *P["y"])
    web = box(*W["body_x"], W["y"][0], W["y"][1], *wu) - box(W["body_x"][0] - 1, W["body_x"][1] + 1, W["y"][0] - 1, W["tail_y0"], wu[0] - 1, W["tail_u"])   # the rack bar is bonded to its +X face
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
    rb, rc = H.retainer_bosses()
    drawn = plate + web + spine + ecc + deck + walls + pads + H.stops() + H.bed_land() + H.tail_rib(SERVO) + rb
    drawn = drawn + H.rack_bar(SERVO)          # the epoxied rack bar rides in the FE as bonded (13.5 x 93 glue line); removed from the print after
    # cuts
    cuts = []
    for (x, u) in H.WHEELS[:2]:
        cuts.append(cylY(x, u, P["y"][0] - 1, P["y"][1] + 1, H.WHEEL["axle_r"], 40))
        cuts.append(hull([[x + 5.2 * math.cos(a), P["y"][1] + 0.01, u + 5.2 * math.sin(a)] for a in np.linspace(0, 2 * math.pi, 32, endpoint=False)] +
                         [[x + 2.6 * math.cos(a), P["y"][1] - 2.6, u + 2.6 * math.sin(a)] for a in np.linspace(0, 2 * math.pi, 32, endpoint=False)]))   # 90 deg countersink
    x, u = H.WHEELS[2]
    cuts.append(cylY(x, u, P["y"][0] - 1, P["y"][1] + 1, H.WHEEL["ecc_bore_r"], 48))
    cuts.append(cylY(x, u, H.WHEEL["ecc_flange"][0], P["y"][1] + 1, 7.6, 48))                             # flange AF 13 (round 15.2) counterbore
    for yr in H.RODS["y"]:
        cuts.append(cylX(yr, H.RODS["u"], -H.RODS["wall_out"] + 1.0, H.RODS["wall_out"] + 1.0, H.ROD_D / 2 + 0.05, 24))   # -X blind, +X open
    cuts.append(cylX(H.SPRING_ROD["y"], H.SPRING_ROD["u"], -H.SPRING_ROD["wall_out"] + 1.0, H.SPRING_ROD["wall_out"] + 1.0, H.ROD_D / 2 + 0.05, 24))
    for (xh, yh) in H.HALL_POS:
        cuts.append(box(xh - 1.6, xh + 1.6, yh - 1.6, H.DECK["slot"][2] + 0.05, H.DECK["u"][0] - 1.0, H.U_BOT - H.LATCH["hall"]["gap"]))   # the package's hole, joined to the blade slot
    # (the hall boards' pockets and every wire space are in H.keepouts(): wiring())
    cuts.append(pad_cuts)
    cuts.append(rc)                                                           # the retainer's heat-set insert holes
    cuts.append(box(*H.BUMPER["x"], P["y"][0] - 1, P["y"][1] + 1, H.BUMPER["u"][0] - 1, H.BUMPER["u"][1]))
    cuts += H.keepouts(SERVO)
    cut = cuts[0]
    for c in cuts[1:]: cut = cut + c
    return drawn, cut


def setup(h):
    drawn, cuts = drawn_and_cuts()
    env = H.design_space(SERVO) + H.rack_bar(SERVO)
    pp = Part(f"box_head_v7_{HEAD}", h, drawn, cuts, env, build=os.environ.get("NOX_BUILD", "+x"))
    bores = None
    for (x, u) in H.WHEELS:
        r = H.WHEEL["ecc_bore_r"] if (x, u) == H.WHEELS[2] else H.WHEEL["axle_r"]
        b = cylY(x, u, H.PLATE["y"][0], H.PLATE["y"][1], r + 2.0) - cylY(x, u, H.PLATE["y"][0] - 1, H.PLATE["y"][1] + 1, r)
        bores = b if bores is None else bores + b
    # supports as the real interfaces (concept review r2): the wheels hold X and Y at their bores (they roll along u); the pinion holds
    # u (its tangential force) on the bonded rack bar's teeth over its patch in the case; its separating force (tangential x tan 20,
    # toward -X) is a load in every case with a u component (concept review r3; system.json "rack on pinion A": 60.8 / 22.1 N)
    pp.support("_bores", bores)
    nb = pp.supports.pop("_bores"); cb = pp.support_cells.pop("_bores")
    for nm, pu in PIN_U.items():
        if pu < web_u()[0] + 3: pu = web_u()[0] + 3
        pp.support("_t", box(44.25, H.WEB["bar_x"][1], H.WEB["y"][0], H.WEB["teeth_y"][1], pu - 4.0, pu + 4.0))   # the bar's teeth at the pinion
        nt = pp.supports.pop("_t"); ct = pp.support_cells.pop("_t")
        fx = np.zeros_like(nb)
        fx[nb.any(1), 0] = True; fx[nb.any(1), 1] = True
        fx[nt.any(1), 2] = True                                              # the teeth hold u only; X is the wheels' (r3)
        pp.supports[nm] = fx; pp.support_cells[nm] = np.unique(np.concatenate([cb, ct]))
        pp.log(f"  support {nm}: bores X Y, rack teeth u at u {pu:.1f}")
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
    stopp = H.stops() ^ box(0, 40, 0, 300, -50, 50); stopm = H.stops() ^ box(-40, 0, 0, 300, -50, 50)
    L = H.LOADS
    T20 = math.tan(math.radians(20.0))

    def teeth(sup):                                                           # the rack teeth at the pinion in this case
        pu = max(PIN_U[sup], web_u()[0] + 3)
        return box(44.25, H.WEB["bar_x"][1], H.WEB["y"][0], H.WEB["teeth_y"][1], pu - 4.0, pu + 4.0)

    def case(name, parts, cls, sup):                                          # + the pinion's separating force for the case's net u load
        fu = sum(f[2] for _, f in parts)
        if abs(fu) > 1e-6: parts = parts + [(teeth(sup), (-abs(fu) * T20, 0, 0))]
        pp.load_multi(name, parts, cls=cls, support=sup)

    for where in (("drone", "insert at the drone"), ("bay", "insert at a bay")):
        case(f"{where[1]} 60 N [swap]", [(pads, (0, 0, -L["insert"]))], CLS["swap"], where[0])
        case(f"{where[1]}, stall 74 N [misuse]", [(pads, (0, 0, -L["stall"]))], CLS["misuse"], where[0])
        case(f"pull at the {where[0]} 57.3 N while the XT60 unmates [swap]", [(sock, (0, 0, L["pull_unmate"]))], CLS["swap"], where[0])
    Ov = L["cart_g"]["overseer"] / 1000 * 9.81
    lever = 48.5 - H.U_BOT; cpl = 2 * Ov * lever / (2 * 18.25)                 # the 2 g lateral at the cartridge CG as a couple on the pads
    padp = box(H.DECK["pad"]["x"][0], H.DECK["pad"]["x"][1], *H.DECK["pad"]["y"], H.DECK["u"][1], H.DECK["pad"]["u_top"])
    padm = box(-H.DECK["pad"]["x"][1], -H.DECK["pad"]["x"][0], *H.DECK["pad"]["y"], H.DECK["u"][1], H.DECK["pad"]["u_top"])
    yc = sum(H.DECK["pad"]["y"]) / 2
    padf = pads ^ box(-50, 50, 0, yc, -50, 50); padb = pads ^ box(-50, 50, yc, 300, -50, 50)   # pad halves along Y
    dyc = 170.2 - yc                                                          # the Overseer CG sits 5.2 past the pads' / sockets' centre in Y
    cgp = 3 * Ov * dyc / (yc - H.DECK["pad"]["y"][0])                         # 3 g at the CG: a couple on the pad halves
    for sx in (1, -1):
        case(f"handling 3 g in + 2 g along {'+' if sx > 0 else '-'}X, Overseer cartridge carried (CG 5.2 off in Y) [handling]",
             [(pads, (0, 0, -3 * Ov)), (stopm if sx > 0 else stopp, (sx * 2 * Ov, 0, 0)), (padp if sx > 0 else padm, (0, 0, -cpl)), (padm if sx > 0 else padp, (0, 0, cpl)),
              (padb, (0, 0, -cgp)), (padf, (0, 0, cgp))], CLS["handling"], "park")
    sockf = sock ^ box(-60, 60, 0, yc, -50, 50); sockb = sock ^ box(-60, 60, yc, 300, -50, 50)
    cgs = 3 * Ov * dyc / (H.RODS["y"][1] - H.RODS["y"][0])
    case(f"handling 3 g out, Overseer cartridge carried (on the rods; CG 5.2 off in Y) [handling]",
         [(sock, (0, 0, 3 * Ov)), (sockb, (0, 0, cgs)), (sockf, (0, 0, -cgs))], CLS["handling"], "park")
    cy = 2 * Ov * (48.5 - H.RODS["u"]) / (yc - H.DECK["pad"]["y"][0])        # 2 g along Y at the CG (u 48.5) to the sockets (u 6.75): a couple on the pad halves
    for sy in (1, -1):
        case(f"handling 2 g along {'+' if sy > 0 else '-'}Y, Overseer cartridge carried (rib -> blades -> rods, couple on the pads) [handling]",
             [(sock, (0, sy * 2 * Ov, 0)), (padb if sy > 0 else padf, (0, 0, -cy)), (padf if sy > 0 else padb, (0, 0, cy))], CLS["handling"], "park")
    sockp = sock ^ box(0, 60, 0, 300, -50, 50); sockm = sock ^ box(-60, 0, 0, 300, -50, 50)
    ca = 43.5 * (H.BLADE["rib"]["u"] - sum(H.STOP["u"]) / 2) / (2 * H.RODS["wall_out"])   # the rib (u 19.4) to the stop: a couple through the rods to the walls
    for sx in (1, -1):                                                        # the receiver's gaps closed: 43.5 N across (latch v2 load set)
        # +X on the cartridge pushes the -X carrier inward onto its stop, and the reverse
        case(f"across {'+' if sx > 0 else '-'}X 43.5 N at a seat, receiver gaps closed (+ rib-to-stop couple) [swap]",
             [(stopm if sx > 0 else stopp, (sx * 43.5, 0, 0)), (sockp, (0, 0, -sx * ca)), (sockm, (0, 0, sx * ca))], CLS["swap"], "bay")
    cm = 1.05e3 / (2 * 18.25)                                                 # the lead-ins' centring 7.6 N / 1.05 N.m (arm position error, latch v2 load set)
    for sx in (1, -1):
        case(f"centring {'+' if sx > 0 else '-'}X 7.6 N + 1.05 N.m at a seat, driver on [swap]",
             [(stopm if sx > 0 else stopp, (sx * 7.6, 0, 0)), (padp, (0, 0, -sx * cm)), (padm, (0, 0, sx * cm))], CLS["swap"], "bay")
    # power cut carrying: the head brake is required (System box swap: arm, row Head brake): braked creep 7.8 mm/s puts 3 N on the bumper
    # (runs/arm/v7/system.json brake), under the handling cases; unbraked (409 / 630 N) is outside the design, gated by the brake self-test
    sv = H.SERVOS[SERVO]; T = sv["kgcm"] * 98.07; ye = H.Y_SERVO_TOP - sv["H"] + sv["ear_y"]
    hs = H.ear_holes(SERVO); f = T / (hs[2][0] - hs[0][0]) / 2
    fr = T / (2 * H.ROLLER["x_sq"])                                           # the reaction: cam -> rollers -> carriers -> rods -> the wall sockets
    pp.load_multi(f"{SERVO} stall torque {T / 1000:.2f} N.m on the ear pads, reacted at the rod sockets through the rollers [misuse]",
                  [(cylY(x, u, ye - 3.0, ye, 5.0, 24), (0, 0, f if x < 10 else -f)) for (x, u) in hs] + [(sockp, (0, 0, fr)), (sockm, (0, 0, -fr))],
                  cls=CLS["misuse"], support="park")
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
    solid = tm((H.design_space(SERVO) + drawn) - cuts)                     # drawn carries the bonded rack bar
    L = Layout(p, volfrac=0.5, settings=Settings(reach_check="off", auto_sym=False, finish="drawn"), log=lambda s_: None)
    st = strength(L, solid)
    out = dict(h=h, mass_g=round(solid.volume * 1.27e-3, 1), worst=st["worst"], worst_case=st["worst_case"], worst_at=st["worst_at"], rows=st["rows"])
    out["build"] = os.environ.get("NOX_BUILD", "+x")
    json.dump(out, open(os.path.join(H.ROOT, "runs", "head", "v7", f"bound_{HEAD}_h{h:g}_{out['build']}.json"), "w"), indent=1)
    return out


def split_bar(out):
    """the print: the optimised body less the rack bar (its own print, head_v7_parts.py); the bonded pair is kept as _bonded.stl"""
    import trimesh
    from arm_cad import tm, man
    p = os.path.join(out, f"box_head_v7_{HEAD}.stl")
    whole = trimesh.load(p, force="mesh"); whole.export(os.path.join(out, f"box_head_v7_{HEAD}_bonded.stl"))
    body = tm(man(whole) - H.rack_bar(SERVO)); body.export(p)
    print("split: body", round(body.volume * 1.27e-3, 1), "g")


if __name__ == "__main__":
    out = sys.argv[4] if len(sys.argv) > 4 else os.path.join(H.ROOT, "runs", "head", "v7", HEAD)
    if os.environ.get("NOX_BOUND"):
        r = bound(float(os.environ["NOX_BOUND"])); print(json.dumps({k: v for k, v in r.items() if k != "rows"})); [print(x) for x in r["rows"]]; sys.exit(0)
    if os.environ.get("NOX_SETUP_ONLY"):
        p = setup(HH); print("setup ok", p.shape, len(p.cases)); sys.exit(0)
    res = optimise(setup, HH, GEN, os.path.abspath(out), f"box_head_v7_{HEAD}", settings=Settings(), git=bool(os.environ.get("NOX_GIT")))
    print({k: res.get(k) for k in ("name", "solid_ok", "mass_g", "total_s")}, res["strength"]["worst"])
    split_bar(os.path.abspath(out))

