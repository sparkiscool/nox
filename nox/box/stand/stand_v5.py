"""Box stand v5 (18 Oct 2026): one wood base + four 2040 posts, every joint an M5. Geometry in W (X across, Y along the
axle away from the stub posts, Z up, axle at X 0 Z 0), clearance checks, tip-over, layout figure.
Tags: C CAD, E estimate, P measured."""
import numpy as np, json
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon

T = 20.0                      # base = 2040 rails lying flat (v5d): 20 tall, 40 wide
BASE_TOP = -181.0             # v5b: the -90 bay's bottom lugs (rho 179) sit 2 over the board; bar sweep clear 44.5 (C)
BASE = dict(x=(-210, 210), y=(-80, 237))         # v5e: H frame on the floor: rail 1 (240) Y -80..-40 behind the stub post, rail 2 (420) Y 197..237 in front of the side posts and the cradle, tie (237) X 50..90 Y -40..197; every post and the cradle stand on the floor (C)
RAILS = dict(cross1=dict(x=(-120, 120), y=(-80, -40)), cross2=dict(x=(-210, 210), y=(197, 237)), tie=dict(x=(50, 90), y=(-40, 197)))
SWEEP_BAR, SWEEP_HEADS = 136.5, 152.5           # C / E
FL_R, FL_BOLT_R = 45.0, 38.0                    # stub flange Ø90, 4 x M5 at r 38 on the 45° diagonals (C, arm_cad.stub)
XB = FL_BOLT_R * np.sqrt(0.5)                   # 26.87: bolt X and Z
LUG_RHO, LUG_Y = (165.0, 179.0), (145.0, 209.0)  # bay module free-side lugs (C, arm_layout)
LUG_W = 14.0
BAYS = (-90.0, -60.0)                           # v5b (Spark 23:49): bays at the bottom on the board; 30 deg pitch keeps the detent (C)
CLR = 1.0

# ---- 2040 posts (20 x 40 section). Each: X range, Y range, Z range, which face carries the T-nuts
posts = {}
# stub posts: 20 face (one slot, slot centre at the post's X centre) toward +Y, flange on plane Y 0
# v5e: ONE stub post, 40 face toward +Y (two slots at X +-10); stub v3 flange holes at (+-10, +-30)
posts["stub"] = dict(x=(-20.0, 20.0), y=(-40.0, 0.0), z=(BASE_TOP - T, 63.0), slot="+Y", cut=63.0 - BASE_TOP + T)   # 264, stands on the floor, bracketed to rail 1 behind it
STUB_V3_BOLTS = [(sx * 10.0, sz * 30.0) for sx in (1, -1) for sz in (1, -1)]
# side posts: inner face at X = lug outer rho + CLR (the 0° bay's +15° lug reaches rho 179 at X 173 .. the post must sit past the
# whole module: max X of the 0° module = max over its lug corners)
lug0 = max(LUG_RHO) * np.cos(np.radians(15.0))              # 172.9: the +15° lug's outer end, X
lug0_t = lug0 + 9.0 * np.sin(np.radians(15.0))              # nut-side lug 9 thick along the plane normal: 175.2 (C)
X_POST = 180.0                                              # v5: 1 clear + 3 past a 0-deg bay's lug; kept (bays now at the bottom) (C)
for s, nm in ((1, "side_R"), (-1, "side_L")):
    posts[nm] = dict(x=tuple(sorted((s * X_POST, s * (X_POST + 20)))), y=(157.0, 197.0), z=(BASE_TOP - T, 178.0), slot="inner",
                     cut=178.0 - BASE_TOP + T)   # 379, stands on the floor, bracketed to rail 2 in front of it

# ---- printed parts (envelopes, C): funnel seat pads, bay lug adapters. All on the side posts' inner faces (X +-X_POST).
SEAT = np.array([98.14, 151.46]); SEAT_N = np.array([np.sqrt(.5), -np.sqrt(.5)]); SEAT_U = np.array([np.sqrt(.5), np.sqrt(.5)])
brackets = {}
for s, nm in ((1, "seat_R"), (-1, "seat_L")):
    p0 = SEAT * [s, 1]; n0 = SEAT_N * [s, 1]; u0 = SEAT_U * [s, 1]; xi = s * X_POST
    A = p0 - 10 * u0 + 0.3 * n0; B = p0 + 10 * u0 + 0.3 * n0          # 20 pad on the collar seat face (M3 into the insert)
    poly = [A, B, B + 12 * n0, (xi, 165.0), (xi, 140.0), A + 12 * n0]
    brackets[nm] = dict(poly=np.array(poly, float), y=((141 - 10, 141 + 10), (213 - 10, 213 + 10)), mass_E=2 * 11, n=2)
def lug_adapter(a_deg, lo_side, z_top, z_bot):
    """bar along Y 137..217 replacing the neighbour's lug half on the free joint plane at a_deg: mating half 9 thick on the
    free side, body to the post's inner face. lo_side: the free side is at lower angles."""
    a = np.radians(a_deg); d = np.array([np.cos(a), np.sin(a)])
    n = np.array([np.sin(a), -np.cos(a)]) if lo_side else np.array([-np.sin(a), np.cos(a)])
    P1 = d * LUG_RHO[0] + 0.3 * n; P2 = d * LUG_RHO[1] + 0.3 * n
    return dict(poly=np.array([P1, P2, (X_POST, z_top), (X_POST, z_bot)], float), y=((137, 217),), mass_E=24, n=1)
# v5c (Spark 23:57): only the -90 module is mounted, on a printed cradle under its back plate; every other module hangs on
# its neighbour's lugs + dovetail. Cradle top = the r 152.5 cylinder between the lugs (X +-35), 4 M5 tapped up into the
# module's generated struts at X +-29, Y 150 / 200 (solid 14..18 deep, M: box_baymod_v4i_W.obj probe 18 Oct), 4 M5 through the board.
CRADLE_X, CRADLE_Y = 35.0, (137.0, 217.0)
th = np.linspace(np.radians(-90 - 13.2), np.radians(-90 + 13.2), 24)   # asin(35/152.5)
top = np.c_[152.5 * np.cos(th), 152.5 * np.sin(th)]
poly = np.vstack([top, [(CRADLE_X, BASE_TOP - T), (-CRADLE_X, BASE_TOP - T)]])   # v5e: stands on the floor, 48.5 tall
brackets["bay_cradle"] = dict(poly=poly, y=(CRADLE_Y,), mass_E=52, n=1,
                              bolts_up=[(-29, 150), (29, 150), (-29, 200), (29, 200)], bolts_down=[(-25, 145), (25, 145), (-25, 209), (25, 209)])
BAYS = (-90.0,)                                                       # mounted; growth: -60, -30, 0, 30 and -120, -150, -180, -210
CHAIN = (-60.0, -30.0, 0.0, 30.0, -120.0, -150.0, -180.0, -210.0)

# ---- checks
def box_pts(p, n=25):
    xs = np.linspace(*p["x"], n); zs = np.linspace(*p["z"], 4 * n)
    X, Z = np.meshgrid(xs, zs); return np.c_[X.ravel(), Z.ravel()]
def poly_pts(poly, n=60):
    pts = []
    for i in range(len(poly)):
        A, B = poly[i], poly[(i + 1) % len(poly)]
        for t in np.linspace(0, 1, n): pts.append(A + (B - A) * t)
    return np.array(pts)
def in_bay_sector(pts):
    a = np.degrees(np.arctan2(pts[:, 1], pts[:, 0])); r = np.hypot(pts[:, 0], pts[:, 1])
    hit = np.zeros(len(pts), bool)
    for b in BAYS: hit |= (np.abs(((a - b) + 180) % 360 - 180) < 15 - 0.1) & (r > 99) & (r < 180)
    return hit.any()
rep = {}
print("part              rmin   |X|min above Z102   in a bay sector?")
for k, p in posts.items():
    q = box_pts(p); r = np.hypot(q[:, 0], q[:, 1])
    hi = q[q[:, 1] > 102]; ox = np.abs(hi[:, 0]).min() if len(hi) else None
    rr = r.min() if p["y"][1] > 6 else None          # stub posts sit behind the arm (Y <= 0): no radial check applies
    rep[k] = dict(rmin=rr, ox=ox, bay=in_bay_sector(q) if p["y"][1] > 6 else False)
    print(f"{k:16s} {'-' if rr is None else round(rr, 1):>6}  {'-' if ox is None else round(ox, 1):>6}   {rep[k]['bay']}")
for k, b in brackets.items():
    q = poly_pts(b["poly"]); r = np.hypot(q[:, 0], q[:, 1])
    hi = q[q[:, 1] > 102]; ox = np.abs(hi[:, 0]).min() if len(hi) else None
    rep[k] = dict(rmin=r.min(), ox=ox, bay=in_bay_sector(q))
    print(f"{k:16s} {round(r.min(), 1):>6}  {'-' if ox is None else round(ox, 1):>6}   {rep[k]['bay']}")
print("base top", BASE_TOP, "clear to the bar sweep", -SWEEP_BAR - BASE_TOP)
print("side post inner face X", X_POST, "clear to the heads' sweep", X_POST - SWEEP_HEADS)
print("-90 module lowest point (lug rho 179 at the -75 / -105 planes, Z", round(-179 * np.cos(np.radians(15)), 1), ") over the floor:", round(-179 * np.cos(np.radians(15)) - BASE_TOP + T, 1), "; over the rail tops:", round(-179 * np.cos(np.radians(15)) - BASE_TOP, 1))
# chain: moment at the root joint (-75 plane, rho 172) from n hanging modules of 0.23 kg (module 0.083 + pack 0.15, E) at CG r 130
root = np.array([172 * np.cos(np.radians(-75)), 172 * np.sin(np.radians(-75))])
for n in (1, 2, 3, 4):
    Mw = sum(0.23 * 9.81 * abs(130 * np.cos(np.radians(a)) - root[0]) / 1000 for a in CHAIN[:n])
    print(f"chain of {n} on the right: weight moment at the root joint {Mw:.2f} N.m; lug + dovetail couple (15 lever) {Mw / 0.015:.0f} N")
print("20 N insert push at the 0 deg bay: root moment", round(20 * abs(0 - root[1]) / 1000, 2), "N.m ->", round(20 * abs(root[1]) / 1000 / 0.015), "N on the lug / dovetail pair (E)")
# funnel interior: frustum inner faces through (84, 143.6) at 45°, mouth Z 219.6, centre Y 177
for k in ("seat_R", "seat_L"):
    q = poly_pts(brackets[k]["poly"]); w = 84 + (q[:, 1] - 143.6)
    inside = (np.abs(q[:, 0]) < w + 4.4 - 0.5) & (q[:, 1] > 143.6) & (q[:, 1] < 219.6)
    print(k, "points inside the funnel panel outer surface:", int(inside.sum()))
# the 30° module's body vs the seat bracket: module sector 15..45°, r 99..180 -> closest bracket point to the sector
q = poly_pts(brackets["seat_R"]["poly"]); a = np.degrees(np.arctan2(q[:, 1], q[:, 0])); r = np.hypot(q[:, 0], q[:, 1])


# ---- tip-over (E): non-stand mass 4.9 kg at Y 76, 174 over the v4 floor (stand_model FIXED less the spider and ring, + 2 bay
# modules with packs, arm, Scout); board 0.65e-6 kg/mm3; 2040 0.95 kg/m
m_rest, cg_rest = 4.9, np.array([76.0, 174 + BASE_TOP - T])
m_base = (0.24 + 0.42 + 0.237) * 0.95                      # four 2040 rails, kg (E)
pm = [(p["cut"] / 1000 * 0.95, np.mean(p["y"]), np.mean(p["z"])) for p in posts.values()]
m_posts = sum(q[0] for q in pm)
m_prints = sum(b["mass_E"] for b in brackets.values()) / 1000
m = m_rest + m_base + m_posts + m_prints
cg_y = (m_rest * cg_rest[0] + m_base * np.mean(BASE["y"]) + sum(q[0] * q[1] for q in pm) + m_prints * 177) / m
cg_z = (m_rest * (cg_rest[1] - 60) + m_base * (BASE_TOP - T / 2) + sum(q[0] * q[2] for q in pm) + m_prints * 100) / m
h = cg_z - (BASE_TOP - T)
tilt_back = np.degrees(np.arctan2(cg_y - BASE["y"][0], h)); tilt_fwd = np.degrees(np.arctan2(BASE["y"][1] - cg_y, h))
W = m * 9.81; h_rim = (234 - (BASE_TOP - T)) / 1000
rest = W * min(cg_y - BASE['y'][0], BASE['y'][1] - cg_y) / 1000
print(f"tip-over E: total {m:.2f} kg (base rails {m_base:.2f}, posts {m_posts:.2f}, prints {m_prints:.2f}), CG Y {cg_y:.0f}, {h:.0f} over the floor; tilt back {tilt_back:.0f} deg, forward {tilt_fwd:.0f} deg; 13 N rim push {13 * h_rim:.1f} N.m vs {rest:.1f} N.m restoring (margin {rest / (13 * h_rim):.1f})")

json.dump(dict(posts={k: {kk: (list(v) if isinstance(v, tuple) else v) for kk, v in p.items()} for k, p in posts.items()},
               brackets={k: dict(poly=b["poly"].round(2).tolist(), y=b["y"], mass_E=b["mass_E"]) for k, b in brackets.items()},
               base=dict(**BASE, top=BASE_TOP, t=T), x_post=float(X_POST), checks=rep), open("stand_v5_parts.json", "w"), indent=1, default=float)

# ---- figure: front view (XZ) and top view (XY)
fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 6.5), dpi=150)
INK, AL, WOOD, PR, MUTED = "#1d1d1f", "#6b7280", "#b45309", "#0f766e", "#6b7280"
for ax in (a1, a2): ax.set_aspect("equal"); ax.axis("off")
a1.set_title("front view (from +Y, the arm side)", fontsize=9, color=INK, loc="left")
a1.add_patch(Rectangle((-210, BASE_TOP - T), 420, T, fc=AL, ec=INK, lw=.5, alpha=.35))
for k, p in posts.items():
    a1.add_patch(Rectangle((p["x"][0], p["z"][0]), p["x"][1] - p["x"][0], p["z"][1] - p["z"][0], fc=AL, ec=INK, lw=.5, alpha=.9 if "side" in k else .55))
for r_, c_, ls in ((SWEEP_HEADS, MUTED, ":"), (100, MUTED, "--")):
    a1.add_patch(Circle((0, 0), r_, fc="none", ec=c_, lw=.7, ls=ls))
a1.add_patch(Circle((0, 0), FL_R, fc="#e5e7eb", ec=INK, lw=.6))
for bx, bz in STUB_V3_BOLTS: a1.add_patch(Circle((bx, bz), 2.75, fc="none", ec=INK, lw=.6))
for b in BAYS + CHAIN[:3] + CHAIN[4:7]:
    th = np.radians(np.linspace(b - 15, b + 15, 20)); ring = np.r_[np.c_[99 * np.cos(th), 99 * np.sin(th)], np.c_[180 * np.cos(th[::-1]), 180 * np.sin(th[::-1])]]
    a1.add_patch(Polygon(ring, fc="#dbeafe" if b == -90 else "none", ec="#1d4ed8", lw=.6, ls="-" if b == -90 else (0, (2, 2))))
for k, b in brackets.items(): a1.add_patch(Polygon(b["poly"], fc=PR, ec=INK, lw=.5, alpha=.85))
fun = np.array([(-84, 143.6), (84, 143.6), (160, 219.6), (-160, 219.6)]); a1.add_patch(Polygon(fun, fc="none", ec=INK, lw=.7))
a1.text(0, 236, "funnel", ha="center", fontsize=7, color=INK)
a1.text(-225, BASE_TOP - 4, f"rail 2 (behind the cradle, in front of the side posts), top Z {BASE_TOP:.0f}; posts and cradle on the floor", ha="left", va="top", fontsize=7, color=WOOD)
a1.text(26, 66, "2040 stub post ×1, 40 face to +Y\nstub v3 flange 4 × M5 at (±10, ±30), T-nuts", fontsize=6.5, color=INK, va="bottom")
a1.text(X_POST + 24, 150, f"2040 side posts ×2\ninner face X ±{X_POST:.0f}", fontsize=6.5, color=INK)
a1.text(140, 160, "seat bracket", fontsize=6.5, color=PR, ha="right")
a1.text(44, -168, "cradle, 4 × M5 up + 4 down", fontsize=6.5, color=PR, ha="left")

a1.set_xlim(-250, 300); a1.set_ylim(-225, 250)
a2.set_title("top view", fontsize=9, color=INK, loc="left")
for rr in RAILS.values(): a2.add_patch(Rectangle((rr["x"][0], rr["y"][0]), rr["x"][1] - rr["x"][0], rr["y"][1] - rr["y"][0], fc=AL, ec=INK, lw=.5, alpha=.5))
for k, p in posts.items():
    a2.add_patch(Rectangle((p["x"][0], p["y"][0]), p["x"][1] - p["x"][0], p["y"][1] - p["y"][0], fc=AL, ec=INK, lw=.5))
a2.add_patch(Rectangle((-152.5, 74), 305, 148, fc="#e5e7eb", ec=MUTED, lw=.5, ls=":")); a2.text(0, 148, "arm sweep r 152.5, Y 74..222", ha="center", fontsize=6.5, color=MUTED)
a2.add_patch(Rectangle((-164.4, 177 - 80), 328.8, 160, fc="none", ec=INK, lw=.7)); a2.text(0, 262, "funnel mouth", ha="center", fontsize=6.5, color=INK)
a2.add_patch(Rectangle((-107, -100), 214, 55, fc="none", ec=MUTED, lw=.6, ls="--")); a2.text(0, -72, "PSU + SKR + chargers on the floor behind (E)", ha="center", fontsize=6.5, color=MUTED)
for s in (1, -1):
    for yy in (141, 213): a2.add_patch(Rectangle((min(s * 98, s * X_POST), yy - 10), abs(X_POST - 98), 20, fc=PR, ec=INK, lw=.4, alpha=.85))
a2.add_patch(Rectangle((-50, 137), 185, 80, fc="#dbeafe", ec="#1d4ed8", lw=.5, alpha=.6)); a2.text(42, 230, "−90° bay on the cradle; others chain on (dashed)", fontsize=6.5, color="#1d4ed8", ha="center")
a2.set_xlim(-270, 300); a2.set_ylim(-120, 290)
fig.suptitle("Nox box stand v5e: all 2040, 6 pieces (H base + 3 posts), one cradle; bays chain round the ring (C / E)", fontsize=10, color=INK, x=0.02, ha="left")
fig.savefig("stand_v5_layout.png", bbox_inches="tight")
print("fig saved")
