"""Box swap arm v6 axial stack (24 Oct 2026, swap arm system worker). Compact by default (Spark 24 Oct) + Spark's Y_C 118 answer for
the next module version: the rails leave hub v5's posts and sit on the motor plate (stack C2 of `arm_v5_layout.stacks()`), which
removes 4 empty 22-tall posts between the plate and the rails. The rack teeth and the 16T pinion keep their Y (64..73): the rack
web shrinks from Y 64..94 to Y 64..78 and pinion v1 stays.
Everything else is arm_v5_layout's geometry: `use("v6")` patches that module's constants in place, so arm_v5_cad (rails, carriage
proxy), arm_v5_layout (2D clearances, detent candidates), detent.obstacles_v5() and arm_kinematics build the v6 stack unchanged.
Numbers C unless tagged. python3 arm_v6_layout.py -> the stack search (2D clearance per rail base Y) + runs/arm/v6/layout.json"""
import math, json, os, copy
import arm_layout as L
import arm_v5_layout as V

HERE = os.path.dirname(os.path.abspath(__file__)); REPO = os.path.normpath(os.path.join(HERE, "..", ".."))
_KEYS = ("AXLE_BOLT_L", "RACK_TAIL", "Z0S", "RACK_TEETH_Y", "PIN_TEETH_Y", "PIN_BOSS_Y", "WEB_U", "RAIL_Y", "RAIL_R", "RAIL_STOP", "RIDGE_Y", "RAIL_CHAN", "RAIL_BOLTS", "RAIL_INSERTS", "WHEEL_Y", "NUT_Y", "TAIL_Y", "SPACER_Y",
         "PLATE_Y", "ECC", "AXLE_KEEP", "ECC_KEEP", "WEB_Y", "POSTS_A", "POSTS_B", "POST_Y", "ALLOWED")
V5 = {k: copy.deepcopy(getattr(V, k)) for k in _KEYS}
STACK = "v5"

# ------------------------------------------------------------------ v6 = rails on the motor plate (C)
PLATE_TOP = L.Y_PLATE1                 # 54: hub motor plate top = rail foot face
P80_CLR = 1.5                          # rail inner end off the 80T flange (r 27.5) and the loop (r 27.0) at Y 54..64
NUT_H = 5.0                            # DIN 985 M5 nyloc (E): 5.0 (v5 drew 4)
PIN_V2 = dict(teeth_y=(62.0, 72.5), boss_y=(56.5, 60.5), flat_y0=59.0)   # pinion v2 (C): 10.5 face (Lewis 0.44 at 60 N, v1 8 face 0.57, swap_system),
                                       # boss under it; the NEMA D-flat is the shaft's last 15 (dimensions M: Y 59..74), so the bore is round below Y 59.5
RACK_V2_Y = (61.5, 73.0)               # rack teeth (heads v6): 0.5 past the pinion face each side; web Y 61.5..78
WEB_U_V2 = (-50.0, 30.0)               # rack tail u -50 (was -47): the XT60 lag at the drone insert (swap_system: 3 mm at 60 N with ties) stays inside the
                                       # rack: pinion u at z0 99 = -44 >= -50 + 6
AXLE_BOLT_L = dict(fixed=20.0, ecc=25.0)  # M5 socket heads (C): fixed wheels plate 4 + shim 1 + wheel 10 + nyloc 5 = 20 (tail flush Y 58);
                                       # the eccentric from its bush flange (Y 80.5): x 25, tail Y 55.5, 1.5 over the hub plate
RACK_TAIL = dict(u_max=-40.0, y0=63.0)  # heads v6: the rack below u -40 has its teeth from Y 63 (1.0 over the axle loop's top Y 62, which its
                                       # tail crosses at park, X 40.5..45.3, sweep C); only the drone insert meshes there (pinion face 9.5 of 10.5)
Z0_INSERT_MAX = 97.6                   # the commanded drone insert may run 2 past 95.6 for the arm's lag under the 60 N (swap_system; DEFAULT):
                                       # wheel rim r 143.3 at X 22, 2.2 inside the stand's r 145.5 arm lane
# carbon tie under each rail (swap_system alternatives: cartridge pitch 2.61 -> 1.85 deg at Y_C 175): 2 x 3.0 carbon rods at X +-8 in the
# rail's frame from a socket in the hub's lower rim into the rail's bottom face near its outer end (mouth Y 54 at r 120, 36.9 deg up the
# rail); rod ends 10 deep in Ø3.3 bores, epoxied (CLAUDE.md). X +-5: 2.35 wall to the rail's sides, 6.7 between the bores (C)
TIE = dict(x=5.0, hub=(22.5, 78.0), rail=(60.0, 128.0), depth=10.0, bore_d=3.3, rod_d=3.0, wall=3.0)   # (Y, r) of the rod ends (C); rod 62.5
RAIL_BOLT_R = (40.0, 78.0)             # one M5 per pad, A and B the same (DEFAULT): inner r 40 (insert hole Ø6.4 inside the cavity wall
                                       # r 36..43, solid below); outer r 78 (pad to r 88 = the plate's edge). Span 38 (v5 posts 41 / 37.25)


def v6_values(dy=-20.0):
    """the v5 rail / carriage stack moved by dy (the rack teeth and the pinion stay); dy -20 = rails on the plate."""
    sh = lambda t: (t[0] + dy, t[1] + dy)
    r0 = max(V5["RAIL_R"][0], L.P80_FL / 2 + P80_CLR) if V5["RAIL_Y"][0] + dy < L.Y_P80[1] else V5["RAIL_R"][0]   # beside the 80T: off its flange
    v = dict(RAIL_Y=sh(V5["RAIL_Y"]), RAIL_R=(r0, V5["RAIL_R"][1]), RIDGE_Y=V5["RIDGE_Y"] + dy, WHEEL_Y=sh(V5["WHEEL_Y"]), SPACER_Y=sh(V5["SPACER_Y"]),
             PLATE_Y=sh(V5["PLATE_Y"]))
    st = dict(V5["RAIL_STOP"]); st["y"] = sh(st["y"]); v["RAIL_STOP"] = st                 # r 27..31 kept: overhangs the rail start over the 80T top
    ch = dict(V5["RAIL_CHAN"]); ch["y"] = sh(ch["y"]); v["RAIL_CHAN"] = ch
    w0 = v["WHEEL_Y"][0]
    v["NUT_Y"] = (w0 - NUT_H, w0)                                                          # nyloc under the wheel
    # fixed wheels: M5 x 20 (plate 4 + shim 1 + wheel 10 + nyloc 5 = 20, flush, DEFAULT); eccentric: M5 x 25 from the bush flange top
    v["TAIL_Y"] = (v["PLATE_Y"][1] + 2.5 - 25.0, w0 - NUT_H)                                # the eccentric's tail (the lowest), Y 55.5..58
    e = dict(V5["ECC"]); e["flange_y"] = (v["PLATE_Y"][1], v["PLATE_Y"][1] + 2.5); v["ECC"] = e
    v["AXLE_KEEP"] = dict(r=5.0, y=(v["PLATE_Y"][1], v["PLATE_Y"][1] + 8.0)); v["ECC_KEEP"] = dict(r=13.0, y=(v["PLATE_Y"][1], v["PLATE_Y"][1] + 3.0))
    v["RACK_TEETH_Y"] = RACK_V2_Y if dy < -9.0 else V5["RACK_TEETH_Y"]
    v["PIN_TEETH_Y"] = PIN_V2["teeth_y"] if dy < -9.0 else V5["PIN_TEETH_Y"]; v["PIN_BOSS_Y"] = PIN_V2["boss_y"] if dy < -9.0 else V5["PIN_BOSS_Y"]
    v["WEB_U"] = WEB_U_V2 if dy < -9.0 else V5["WEB_U"]
    v["Z0S"] = tuple(V5["Z0S"]) + ((Z0_INSERT_MAX,) if dy < -9.0 else ())
    v["AXLE_BOLT_L"] = AXLE_BOLT_L if dy < -9.0 else V5["AXLE_BOLT_L"]; v["RACK_TAIL"] = RACK_TAIL if dy < -9.0 else V5["RACK_TAIL"]
    v["WEB_Y"] = (v["RACK_TEETH_Y"][0], v["PLATE_Y"][1])                                    # rack web from its teeth to the plate top
    b = copy.deepcopy(V5["RAIL_BOLTS"])
    on_plate = V5["RAIL_Y"][0] + dy <= PLATE_TOP + 1e-6
    v["POST_Y"] = (V5["POST_Y"][0], V5["POST_Y"][1] + dy)                                  # posts shrink with the rails (gone on the plate)
    v["POSTS_A"], v["POSTS_B"] = copy.deepcopy(V5["POSTS_A"]), copy.deepcopy(V5["POSTS_B"])
    if on_plate:                                                                           # on the plate: pads at RAIL_BOLT_R, inserts in the hub plate
        b["r_A"] = RAIL_BOLT_R; b["r_B"] = RAIL_BOLT_R
        b["post_insert"] = dict(hole_d=6.4, depth=8.0, insert="M5 x 5.8 heat-set (OD 7.0, E) in a hub plate boss Y 46..54")
        v["POSTS_A"], v["POSTS_B"] = [], []
    b["cbore_y"] = V5["RAIL_BOLTS"]["cbore_y"] + dy
    v["RAIL_BOLTS"] = b; v["RAIL_INSERTS"] = b
    al = set(V5["ALLOWED"]) - {p for p in V5["ALLOWED"] if any(s.startswith("rail") and "foot" in s for s in p)}
    for h in ("A", "B"):
        for r in b["r_" + h]:
            al.add((f"rail{h}:foot{r:g}", f"post{h}"))
    v["ALLOWED"] = al
    return v


def tie_rod(h="A", side=1):
    """(p0, p1) rod ends in the arm frame (hub end, rail end) for head h, side +-1 (rail X)"""
    x = side * TIE["x"]; (yh, rh), (yr, rr) = TIE["hub"], TIE["rail"]
    f = (lambda X, Y, R: (X, Y, R)) if h == "A" else (lambda X, Y, R: (-R, Y, -X))
    return f(x, yh, rh), f(x, yr, rr)


def use(stack="v6", dy=-20.0):
    """patch arm_v5_layout in place: 'v5' restores the posts stack, 'v6' the compact one (rails on the plate)."""
    global STACK
    vals = V5 if stack == "v5" else v6_values(dy)
    for k, val in vals.items():
        setattr(V, k, copy.deepcopy(val))
    STACK = stack
    return vals


def stack_search(tab=L.Z_TABC):
    """the axial-stack optimisation: the rail / carriage block moved down from the posts (dy 0) to the plate (dy -20) in 1 mm steps; for
    each, the 2D clearance over 5 x 5 stroke states (arm_v5_layout.clearance_sweep), the tail-to-plate gap and the extent of the arm at
    the demo Y_C and with modules at the lowest Y_C the carriage hardware allows. Returns rows (C)."""
    ax = (53.93, -43.67)
    rows = []
    for dy in [float(-k) for k in range(0, 21)]:
        vals = use("v6", dy) if dy < 0 else use("v5")
        w = V.clearance_sweep(ax)
        gmin = min(w.values()); worst = min(w, key=w.get)
        plate_ok = (V.TAIL_Y[0] - PLATE_TOP) >= 1.0                                            # the wheel tails over the hub plate
        hw_top = max(V.AXLE_KEEP["y"][1] - 3.0, V.ECC["flange_y"][1] + 5.0)                     # socket head 5 tall on the plate / on the bush flange
        yc_min = hw_top + 1.0 + tab                                                            # cartridge closed end 1.0 over the carriage hardware (tab: closed end to the tab line)
        rows.append(dict(dy=dy, rail_y=V.RAIL_Y, plate_y=V.PLATE_Y, gmin=round(gmin, 2), worst=list(worst), tail_to_plate=round(V.TAIL_Y[0] - PLATE_TOP, 2),
                         posts=bool(V.POSTS_A), ok=bool(gmin >= 1.0 - 1e-6 and plate_ok), hw_top=hw_top, yc_min=round(yc_min, 1)))
    use("v5")
    return rows


if __name__ == "__main__":
    rows = stack_search()
    for r in rows:
        print(f"dy {r['dy']:5.0f}: rails Y {r['rail_y'][0]:.0f}..{r['rail_y'][1]:.0f} plate {r['plate_y'][0]:.0f}..{r['plate_y'][1]:.0f} posts {r['posts']!s:5s} "
              f"gmin {r['gmin']:6.2f} {r['worst']} tail-plate {r['tail_to_plate']:6.1f} -> {'OK' if r['ok'] else 'no'}; Y_C min {r['yc_min']}")
    os.makedirs(os.path.join(REPO, "runs", "arm", "v6"), exist_ok=True)
    json.dump(rows, open(os.path.join(REPO, "runs", "arm", "v6", "stack_search.json"), "w"), indent=1, default=list)
