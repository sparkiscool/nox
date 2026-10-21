#!/usr/bin/env python3
"""Box stand v6: all printed. Layout, tip-over search, mast section pick, clearances,
print sizes, mass budget. Writes stand_v6_parts.json and stand_v6_layout.png.
W coords (box-arm.md): X across, Y along the axle (+ away from the stub face),
Z up, axle at X 0 Z 0, floor Z -201. All numbers C unless tagged E.
"""
import json, math, itertools
import numpy as np

FLOOR = -201.0            # kept from v5e (DEFAULT)
BED = 176.0               # A1 mini 180 cube, 4 margin
BED_DIAG = 254.0          # flat part on the bed diagonal: L + w <= 254
G = 9.81

# ---------------------------------------------------------------- fixed interfaces
STUB_BOLTS = [(x, z) for x in (-10, 10) for z in (-30, 30)]   # stub v3 flange, M5 button
STUB_PLATE = dict(disc_d=54.0, lobe_d=14.4)                   # spider plate outline (arm doc)
PAD = dict(x=98.14, z=151.46, y=(141.0, 213.0), n=(0.707, 0.0, -0.707))  # funnel seat pads
CRADLE = dict(y=(140.0, 197.0), top=-152.9, body_x=20.4, lip_x=30.0, bolts_y=(154.0, 189.0),
              back_bolts=dict(x=(-10, 10), z=-191.0, y=197.0))      # cradle v5e, unchanged
RING_R_IN, RING_R_ENV, RING_R_LUG = 148.0, 163.0, 179.0       # frame band (DEFAULT), module env, lugs
RING_Y = (133.0, 221.0)
BAY_POS = [30, 0, -30, -60, -90, -120, -150, -180, -210]       # 12 positions at 30°, 90° opening
BAYS_NOW = [-90, -60]                                         # printed bays (-60 DEFAULT for the 2nd)
HEAD_SWEEP_R, BAR_SWEEP_R = 152.5, 136.5                      # heads (Y 97..222), bars (Y 74..94)
HUB_R, HUB_Y = 86.0, (8.0, 67.0)
DRONE_COL_X, DRONE_COL_Z = 84.0, 102.0                        # |X| < 84 above Z 102 must stay open

# ---------------------------------------------------------------- stand v6 pieces (C)
# vertical structure: spigot (part of KB) + sleeve (the mast) + stub head on the sleeve
GUSSET_TOP = -140.0       # 4 under the sleeve's bottom edge (Z -136)
MAST = dict(x=40.0, y=60.0, wall=1.7, head_x=60.0, head_z=(-40.0, 40.0),
            spigot_z=(FLOOR + 3.0, -60.0),      # KB's spigot, printed with KB
            sleeve_z=(-136.0, 40.0),            # the mast piece, 176 tall
            clearance=0.2, bolts=[("X", -80.0), ("Y", -120.0)])
JOINT_Y = (60.0, 80.0)                          # KB/KF lap + cross bar seat, one bolt group
KB = dict(y=(-96.0, 80.0), x=20.0, z=(FLOOR, FLOOR + 20.0))          # 176 long, keel + spigot
KF = dict(y=(60.0, 194.0), x=20.0, z=(FLOOR, FLOOR + 20.0),
          tongue=dict(y=(140.0, 194.0), x=17.8, z=(FLOOR + 2.4, FLOOR + 15.0)))   # inside the cradle
BAR = dict(len=200.0, w=20.0, h=(16.0, 8.0), x_tip=180.0, lap=40.0)  # 2 identical halves, X -20..180
TOE = dict(y=(197.0, 245.0), x=20.0, h=10.0)                          # behind the cradle, same 2 M5
FRAME = dict(pitch=30, r=(RING_R_IN, RING_R_ENV), y=RING_Y, mass_e=25.0)
ADAPTER = dict(mass_e=20.0)

# ---------------------------------------------------------------- masses (E) and CGs (W)
# (name, kg, (x, y, z))
masses = [
    ("arm (hub, heads, motors, bars, 2 packs, stub)", 2.00, (0, 60, 0)),
    ("mast sleeve + head", 0.09, (0, -30, -50)),
    ("KB keel + spigot + gussets", 0.13, (0, -20, -150)),
    ("KF keel", 0.06, (0, 130, -192)),
    ("cross bar halves x2", 0.056, (0, 70, -193)),
    ("toe", 0.012, (0, 221, -196)),
    ("cradle v5e", 0.0375, (0, 168, -175)),
    ("bay module -90", 0.0835, (0, 177, -124)),
    ("bay module -60", 0.0835, (62, 177, -107)),
    ("ring frames x7", 0.175, (0, 177, -19)),
    ("ring end adapters x2", 0.04, (0, 177, 140)),
    ("funnel (8 panels, hub, blocks)", 0.65, (0, 177, 180)),
    ("Scout seated", 0.52, (0, 177, 160)),
]
ELECTRONICS = ("PSU + SKR + chargers on the keel (zip-tied)", 1.00, (0, -70, -180))

def cg(ms):
    m = sum(k for _, k, _ in ms)
    c = np.array([sum(k * np.array(p) for _, k, p in ms)]).reshape(3) / m
    return m, c

# ---------------------------------------------------------------- tip-over
def margin(ms, contacts, pushes):
    """Min over push cases of restoring / overturning about every hull edge (E)."""
    m, c = cg(ms)
    W = m * G
    pts = np.array(contacts, float)
    hull = _hull(pts)
    worst = (1e9, None)
    for (name, p, F) in pushes:
        p = np.array(p, float); F = np.array(F, float)
        for i in range(len(hull)):
            a, b = hull[i], hull[(i + 1) % len(hull)]
            u = np.array([b[0] - a[0], b[1] - a[1], 0.0]); u /= np.linalg.norm(u)
            # restoring: W x lever of the CG from the edge (inside the hull is positive)
            d = ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) / np.linalg.norm(b - a)
            r = np.array([p[0] - a[0], p[1] - a[1], p[2] - FLOOR])
            Mo = np.dot(np.cross(r, F), u)           # overturning, positive tips outward over this edge
            if d <= 0:
                continue
            Mr = W * d / 1000.0
            if Mo > 1e-6:
                k = Mr / (Mo / 1000.0)
                if k < worst[0]:
                    worst = (k, f"{name} over the edge ({a[0]:.0f}, {a[1]:.0f})-({b[0]:.0f}, {b[1]:.0f}): {Mr:.2f} vs {Mo/1000:.2f} N·m")
    return worst, m, c

def _hull(P):
    P = sorted(set(map(tuple, P)))
    if len(P) <= 2: return P
    def cross(o, a, b): return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lo, up = [], []
    for p in P:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0: lo.pop()
        lo.append(p)
    for p in reversed(P):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0: up.pop()
        up.append(p)
    return [np.array(q, float) for q in lo[:-1] + up[:-1]]

RIM_Z = 237.0   # funnel rim (E, v5e used 13 N at 438 over the floor)
PUSH = 13.0
pushes = []
for nm, pt in [("rim", (0, 177, RIM_Z)), ("wheel top", (0, 60, 150.0))]:
    for dn, F in [("+X", (PUSH, 0, 0)), ("-X", (-PUSH, 0, 0)), ("+Y", (0, PUSH, 0)), ("-Y", (0, -PUSH, 0))]:
        pushes.append((f"{PUSH:.0f} N {dn} on the {nm}", pt, F))

def contacts_for(yb, ys, yf, xs):
    return [(0, yb), (0, yf), (xs, ys), (-xs, ys), (20, yb), (-20, yb)]

# search: back end of KB (yb), cross bar Y (ys), toe end (yf); KB length <= BED, KF length <= BED
best = None
for yb in np.arange(-110, -50, 2.0):
    for ys in np.arange(40, 130, 2.0):
        kb_len = ys + 10 - yb                   # KB spans yb .. ys+10 (the joint zone is ys±10)
        if kb_len > BED: continue
        for yf in (197.0, 230.0, 245.0, 260.0):
          for xs in (180.0, 200.0):
            k_w, m_w, _ = margin(masses + [ELECTRONICS], contacts_for(yb, ys, yf, xs), pushes)
            k_n, _, _ = margin(masses, contacts_for(yb, ys, yf, xs), pushes)
            score = (min(k_w[0], 1.6), min(k_n[0], 1.6), -(yf - 197) - (ys + 10 - yb) / 10 - (xs - 180) / 4)   # margins first, then least extra length
            if best is None or score > best[0]:
                best = (score, dict(yb=yb, ys=ys, yf=yf, xs=xs, k_with=k_w, k_without=k_n))
opt = best[1]
# apply the chosen layout
KB["y"] = (opt["yb"], opt["ys"] + 10); JOINT_Y = (opt["ys"] - 10, opt["ys"] + 10); KF["y"] = (opt["ys"] - 10, 194.0)
TOE["y"] = (197.0, opt["yf"]) if opt["yf"] > 197 else None
BAR["x_tip"] = opt["xs"]; BAR["len"] = opt["xs"] + 20.0
contacts = contacts_for(opt["yb"], opt["ys"], opt["yf"], opt["xs"])
k_with, m_with, c_with = margin(masses + [ELECTRONICS], contacts, pushes)
k_without, m_without, c_without = margin(masses, contacts, pushes)
# static tilt (sideways and fore/aft) with electronics
def static_tilt(ms, half_footprint):
    m, c = cg(ms); return math.degrees(math.atan2(half_footprint, c[2] - FLOOR))
tilt_side = static_tilt(masses + [ELECTRONICS], opt["xs"])
tilt_fwd = static_tilt(masses + [ELECTRONICS], opt["yf"] - c_with[1])
tilt_back = static_tilt(masses + [ELECTRONICS], c_with[1] - opt["yb"])

# ---------------------------------------------------------------- mast section (E: PETG E 2.0 GPa, G 0.75 GPa)
E_PETG, G_PETG = 2000.0, 750.0
M_LEAN = 1.2 + 3.5          # N·m: 2 kg at Y 60 + 20 N head push at Y 175 (-90° bay) (E)
M_SIDE = 20 * 0.20          # 20 N push at the 0° bay, lever to the floor-level root (E)
T_YAW = 20 * 0.175          # 20 N at Y 175 (E)
L_MAST = MAST["sleeve_z"][1] - MAST["spigot_z"][0]
def section(a, b, t):
    Ix = (a * b**3 - (a - 2*t) * (b - 2*t)**3) / 12
    Iy = (b * a**3 - (b - 2*t) * (a - 2*t)**3) / 12
    Am = (a - t) * (b - t); J = 4 * Am**2 * t / (2 * (a - t + b - t))
    A = a * b - (a - 2*t) * (b - 2*t)
    return Ix, Iy, J, A
cands = []
for a, b, t in itertools.product((40, 50), (44, 52, 60, 70), (1.7, 2.1)):
    Ix, Iy, J, A = section(a, b, t)
    th = math.degrees(M_LEAN * 1000 * L_MAST / (E_PETG * Ix))
    th_s = math.degrees(M_SIDE * 1000 * L_MAST / (E_PETG * Iy))
    ph = math.degrees(T_YAW * 1000 * L_MAST / (G_PETG * J))
    sig = M_LEAN * 1000 / (Ix / (b / 2))
    cands.append(dict(a=a, b=b, t=t, A=A, lean_deg=th, side_deg=th_s, yaw_deg=ph, stress_MPa=sig,
                      drop_at_Y222_mm=math.radians(th) * 222, g_per_mm=A * 1.27e-3))
ok = [c for c in cands if c["lean_deg"] <= 0.25 and c["yaw_deg"] <= 0.4]
mast_pick = min(ok, key=lambda c: c["A"])
MAST["x"], MAST["y"], MAST["wall"] = mast_pick["a"], mast_pick["b"], mast_pick["t"]

# socket / spigot root (E): spigot wall 3.0, box section at the keel
sp_a, sp_b = MAST["x"] - 2 * (MAST["wall"] + MAST["clearance"]), MAST["y"] - 2 * (MAST["wall"] + MAST["clearance"])
Ixs, Iys, Js, As = section(sp_a, sp_b, 3.0)
spigot = dict(outer=(sp_a, sp_b), wall=3.0, root_stress_MPa=M_LEAN * 1000 / (Ixs / (sp_b / 2)),
              root_side_MPa=M_SIDE * 1000 / (Iys / (sp_a / 2)), g=(As * (MAST["spigot_z"][1] - MAST["spigot_z"][0])) * 1.27e-3)
engage = MAST["spigot_z"][1] - MAST["sleeve_z"][0]
joint_slop_deg = math.degrees(math.atan(2 * MAST["clearance"] / engage))

# ---------------------------------------------------------------- cross bar root (E, misuse: full weight on one tip)
F_tip = m_with * G
bar_M = F_tip * (BAR["x_tip"] - 20) / 1000
h0, w, t = BAR["h"][0], BAR["w"], 1.7
W_bar = (w * h0**3 - (w - 2*t) * (h0 - 2*t)**3) / 12 / (h0 / 2)
bar_stress = bar_M * 1000 / W_bar
bar_g = ((w * (h0 + BAR["h"][1]) / 2 - (w - 2*t) * ((h0 + BAR["h"][1]) / 2 - 2*t)) * BAR["len"]) * 1.27e-3

# ---------------------------------------------------------------- ring chain (E)
F_top = (0.65 + 0.52 + 0.04) * G          # funnel + drone + adapters
chain_axial = F_top / 2
push_couple = PUSH * (RIM_Z) / 1000 / (2 * 0.121)   # 13 N at the rim vs the two top lugs at X ±121
root_M = PUSH * (RIM_Z - CRADLE["top"]) / 1000 / 2  # half the rim push moment per chain root (E, worst)
lug_F = root_M / 0.015 + chain_axial                 # 15 mm lug / dovetail couple

# ---------------------------------------------------------------- clearances (C)
checks = {
    "cross bar vs bar sweep": f"bar top Z {FLOOR + BAR['h'][0]:.0f} vs bars' lowest Z {-BAR_SWEEP_R:.1f} at Y 74..94: {(-BAR_SWEEP_R) - (FLOOR + BAR['h'][0]):.1f} clear",
    "KB keel vs hub": f"keel top Z {FLOOR + 20:.0f} vs hub lowest Z {-HUB_R:.0f} (Y 8..67): {-HUB_R - (FLOOR + 20):.0f} clear",
    "mast face": f"flat plane Y 0 over the hub's r {HUB_R:.0f} and the motors (bodies start Y 2, arm doc): nothing proud of Y 0 (head pocket is a recess)",
    "mast head vs motors": f"head X ±{MAST['head_x']/2:.0f}, Z ±40 all behind Y 0; motors to the face 2.0 as in v5e",
    "frames vs parked heads": f"frame band r >= {RING_R_IN:.0f} vs parked packs' corners r 97.5 and bars r {BAR_SWEEP_R} (Y 74..94, outside the ring's Y {RING_Y})",
    "frames vs extended heads": f"heads extend only at a bay or the drone (|X| <= 19 beyond r 98.5); frame slots never see an extended head",
    "adapters vs drone column": f"pad arms from the 45°/135° lugs (X ±121.6, Z 121.6) to the pads (X ±{PAD['x']}, Z {PAD['z']}): every point |X| >= {PAD['x']:.0f} > {DRONE_COL_X}",
    "ring vs cross bar": f"ring Y {RING_Y[0]:.0f}..{RING_Y[1]:.0f} vs cross bar Y {JOINT_Y[0]:.0f}..{JOINT_Y[1]:.0f}: {RING_Y[0] - JOINT_Y[1]:.0f} apart",
    "toe vs heads": f"toe Y {TOE['y'][0]:.0f}..{TOE['y'][1]:.0f} at Z {FLOOR:.0f}..{FLOOR + TOE['h']:.0f}; heads reach Y 222 at r <= 152.5: {-152.5 - (FLOOR + TOE['h']):.1f} clear",
}

# ---------------------------------------------------------------- print sizes (C)
sleeve_len = MAST["sleeve_z"][1] - MAST["sleeve_z"][0]
prints = [
    # name, qty, bbox (mm), bed face, g (E), notes
    ("mast sleeve + stub head", 1, (MAST["head_x"], MAST["y"], sleeve_len), "bottom face, standing",
     (section(MAST["x"], MAST["y"], MAST["wall"])[3] * sleeve_len + 15000) * 1.27e-3, "stub pocket + 4 nut traps at the top, 45° flare to the 60 head"),
    ("KB: keel + spigot + 2 gussets", 1, (2 * KB["x"] + 2 * 3, KB["y"][1] - KB["y"][0], MAST["spigot_z"][1] - FLOOR), "floor face, standing",
     spigot["g"] + (KB["y"][1] - KB["y"][0]) * 222 * 1.27e-3 + 10, f"keel channel 40 x 20; spigot to Z -60; gussets fore/aft at X 0, 3 thick, 45 long, top Z {GUSSET_TOP:.0f}; 2 zip-tie slots behind the spigot"),
    ("KF: keel + cradle tongue", 1, (2 * KF["x"], KF["y"][1] - KF["y"][0], 20), "floor face, flat",
     (KF["y"][1] - KF["y"][0]) * 222 * 1.27e-3 + 8, "tongue 35.6 x 12.6 slides into cradle v5e's open front; 2 M5 nut pockets at its end"),
    ("cross bar half", 2, (BAR["len"], BAR["w"], BAR["h"][0]), "bottom face, flat (diagonal on the bed)", bar_g,
     "tapered 16 -> 8; inner 40 half-lapped; 2 M5 x 35 through both halves + KF + KB"),
    ("toe", 1, (TOE["y"][1] - TOE["y"][0] if TOE["y"] else 0, 2 * TOE["x"], 20.0), "flat", 12.0, "L: 4-thick upstand 40 x 20 on the cradle's back wall (its 2 M5 at X ±10, Z -191) + floor bar 10 tall"),
    ("ring frame 30°", len(BAY_POS) - len(BAYS_NOW), (90.0, 40.0, RING_Y[1] - RING_Y[0]), "near end (Y 133) down, 3 per plate", FRAME["mass_e"],
     "generated; lugs + dovetails copied from arm_cad.bay_module_parts; band r 148..163"),
    ("ring end adapter (R, L mirrored)", 2, (60.0, 90.0, 30.0), "lug plate face down", ADAPTER["mass_e"],
     "2 x M5 on the chain's top lug pair + 2 funnel seat pads (P1 geometry, M3 x 25)"),
]
fits = []
for n, q, (a, b, c), face, g, note in prints:
    dims = sorted((a, b, c), reverse=True)
    ok_box = dims[0] <= BED
    ok_diag = (dims[0] + dims[1]) <= BED_DIAG and dims[2] <= BED
    fits.append((n, q, dims, face, g, ok_box or ok_diag, "fits" if ok_box else ("fits on the diagonal" if ok_diag else "TOO BIG"), note))
g_total = sum(q * g for _, q, _, _, g, *_ in fits)

# ---------------------------------------------------------------- hardware (C)
hardware = {
    "stub to mast head": "4 x M5 x 12 ISO 7380 button head (was x 8 for the T-nuts) + 4 M5 nuts in side traps",
    "sleeve to spigot": "2 x M5 x 55 + nuts (one along X at Z -80, one along Y at Z -120)",
    "base joint": "4 x M5 x 35 + nuts at (X ±10, Y %.0f / %.0f): bar halves + KF tongue + KB tongue" % (JOINT_Y[0] + 5, JOINT_Y[1] - 5),
    "cradle + toe": "2 x M5 x 20 + nuts (the cradle's own back holes at X ±10, Z -191): toe + back wall + KF tongue",
    "cradle to the -90° module": "4 x M5 x 16 as v5e",
    "ring": "2 x M5 x 16 + nut per joint (module pattern): 8 joints + 2 adapters = 20",
    "funnel": "4 x M3 x 25 into the collar inserts as v5e",
    "electronics": "2 zip ties through slots in the KB keel behind the spigot (PSU as ballast)",
}

out = dict(layout=dict(floor=FLOOR, mast=MAST, spigot=spigot, engage_mm=engage, joint_slop_deg=joint_slop_deg,
                       KB=KB, KF=KF, joint_y=JOINT_Y, bar=BAR, toe=TOE, frames=FRAME, bays_now=BAYS_NOW,
                       contacts=contacts),
           tipover=dict(mass_with_kg=m_with, cg_with=c_with.tolist(), margin_with=k_with,
                        mass_without_kg=m_without, cg_without=c_without.tolist(), margin_without=k_without,
                        tilt_side_deg=tilt_side, tilt_fwd_deg=tilt_fwd, tilt_back_deg=tilt_back, pushes="13 N at the rim (Z 237) and the wheel top (Z 150), ±X ±Y"),
           mast_pick=mast_pick, mast_candidates=cands,
           bar=dict(root_M_Nm=bar_M, stress_MPa=bar_stress, tip_force_N=F_tip),
           ring=dict(chain_axial_N=chain_axial, push_couple_N=push_couple, root_M_Nm=root_M, lug_F_N=lug_F),
           checks=checks, prints=[dict(name=n, qty=q, dims=d, face=f, g=g, fit=s, note=nt) for n, q, d, f, g, _, s, nt in fits],
           g_total=g_total, hardware=hardware)
json.dump(out, open("stand_v6_parts.json", "w"), indent=1, default=float)

print(f"layout: KB Y {KB['y'][0]:.0f}..{KB['y'][1]:.0f}, joint/cross bar Y {JOINT_Y[0]:.0f}..{JOINT_Y[1]:.0f} tips X ±{opt['xs']:.0f}, KF Y {KF['y'][0]:.0f}..194, toe to Y {opt['yf']:.0f}")
print(f"tip-over: {m_with:.2f} kg with electronics, CG Y {c_with[1]:.0f} Z {c_with[2]-FLOOR:.0f} over the floor: min margin {k_with[0]:.2f} ({k_with[1]})")
print(f"          {m_without:.2f} kg without: {k_without[0]:.2f} ({k_without[1]}); static tilt side {tilt_side:.0f}°, fwd {tilt_fwd:.0f}°, back {tilt_back:.0f}°")
print(f"mast: {mast_pick['a']:.0f} x {mast_pick['b']:.0f} wall {mast_pick['t']}: lean {mast_pick['lean_deg']:.2f}° ({mast_pick['drop_at_Y222_mm']:.1f} mm at Y 222), side {mast_pick['side_deg']:.2f}°, yaw {mast_pick['yaw_deg']:.2f}°, {mast_pick['stress_MPa']:.1f} MPa; spigot {sp_a:.1f} x {sp_b:.1f} wall 3: root {spigot['root_stress_MPa']:.1f} / {spigot['root_side_MPa']:.1f} MPa; engage {engage:.0f}, slop {joint_slop_deg:.2f}°")
print(f"bar root (misuse, {F_tip:.0f} N on one tip): {bar_M:.1f} N·m, {bar_stress:.0f} MPa; ring: chain {chain_axial:.0f} N, top couple ±{push_couple:.0f} N, root {root_M:.1f} N·m -> lug {lug_F:.0f} N")
for k, v in checks.items(): print(f"  {k}: {v}")
for n, q, d, f, g, _, s, nt in fits: print(f"  print {q}x {n}: {d[0]:.0f} x {d[1]:.0f} x {d[2]:.0f} {s}, ~{g:.0f} g ({f})")
print(f"printed total ~{g_total:.0f} g (E) new; retired: 4 seat pads (99 g) and 1.92 m of 2040")

# ---------------------------------------------------------------- layout figure
def figure(path="stand_v6_layout.png"):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Polygon, Circle, Wedge, Arc
    NEW, OLD, BUY, ENV, SWP = "#1b6f8a", "#9c7a4a", "#8e98a3", "#c9ced4", "#d0862e"
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 8.5, "axes.edgecolor": "#444", "axes.linewidth": 0.6})
    fig = plt.figure(figsize=(17, 9.6), dpi=150, facecolor="white")
    gs = fig.add_gridspec(2, 3, width_ratios=[1.25, 1.0, 0.95], height_ratios=[1.35, 1.0], wspace=0.08, hspace=0.12,
                          left=0.035, right=0.985, top=0.93, bottom=0.05)
    axS, axF, axT = fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[:, 2])
    axP = fig.add_subplot(gs[1, 0:2])
    yb, ys, yf, xs = opt["yb"], opt["ys"], opt["yf"], opt["xs"]
    mx, my = MAST["x"], MAST["y"]
    def box(ax, x0, y0, w, h, c, a=0.9, ls="-", lw=0.7, fill=True, z=3):
        ax.add_patch(Rectangle((x0, y0), w, h, facecolor=c if fill else "none", edgecolor=c if fill else c, alpha=a, lw=lw, ls=ls, zorder=z))
    # ---- side view (Y-Z, from +X)
    ax = axS; ax.set_title("side  (Y along the axle, Z up; from +X)", loc="left", fontsize=9.5, color="#222")
    ax.axhline(FLOOR, color="#777", lw=1.0, zorder=1); ax.axhline(0, color="#bbb", lw=0.5, ls=":")
    box(ax, HUB_Y[0], -HUB_R, HUB_Y[1] - HUB_Y[0], 2 * HUB_R, BUY, 0.35)                       # hub
    box(ax, 74, -134, 20, 268, BUY, 0.5)                                                        # bars
    box(ax, 97.3, -98.5, 222 - 97.3, 197, BUY, 0.22)                                            # heads parked
    box(ax, RING_Y[0], -RING_R_LUG, RING_Y[1] - RING_Y[0], 2 * RING_R_LUG, OLD, 0.12, fill=True)  # ring envelope
    box(ax, RING_Y[0], -RING_R_ENV, RING_Y[1] - RING_Y[0], RING_R_ENV - 100, OLD, 0.55)        # -90 bay
    box(ax, RING_Y[0], RING_R_IN, RING_Y[1] - RING_Y[0], RING_R_ENV - RING_R_IN, OLD, 0.55)   # top of the ring (adapters' frames)
    ax.add_patch(Polygon([(93, 102), (261, 102), (284, RIM_Z), (70, RIM_Z)], closed=True, facecolor="none", edgecolor="#777", lw=0.8, ls="--", zorder=4))  # funnel (E)
    box(ax, 107, 100, 140, 110, "#777", 0.9, fill=False, ls=":")                                 # drone (E)
    ax.plot([PAD["y"][0], PAD["y"][1]], [PAD["z"], PAD["z"]], "o", ms=3.5, color=NEW, zorder=6)
    box(ax, yb, FLOOR, ys + 10 - yb, 20, NEW)                                                   # KB keel
    box(ax, -my, FLOOR + 3, my, MAST["spigot_z"][1] - FLOOR - 3, NEW, 0.9)                       # spigot
    for gy in (-my - 45, 0):                                                                     # gussets
        ax.add_patch(Polygon([(gy, FLOOR + 20), (gy + 45, FLOOR + 20), (gy + 45 if gy < 0 else gy, GUSSET_TOP)], closed=True, facecolor=NEW, alpha=0.55, lw=0, zorder=3))
    box(ax, -my, MAST["sleeve_z"][0], my, MAST["sleeve_z"][1] - MAST["sleeve_z"][0], NEW, 0.95)  # sleeve
    box(ax, -my, MAST["head_z"][0], my, 80, NEW, 1.0)
    for (_, z) in STUB_BOLTS: ax.plot(0, z, "o", ms=2.5, color="white", zorder=7)
    box(ax, ys - 10, FLOOR, 20, BAR["h"][0], NEW, 0.9)                                           # cross bar (end-on)
    box(ax, KF["y"][0], FLOOR, 194 - KF["y"][0], 20, NEW, 0.75)                                 # KF
    box(ax, CRADLE["y"][0], FLOOR, 57, CRADLE["top"] - FLOOR, OLD, 0.8)                         # cradle v5e
    box(ax, 197, FLOOR, yf - 197, TOE["h"], NEW, 0.9)                                           # toe
    box(ax, -104, FLOOR + 20, 64, 40, "#555", 1, fill=False, ls="--")                           # PSU (E)
    ax.text(-72, FLOOR + 62, "PSU (ballast)", ha="center", va="bottom", fontsize=7.5, color="#555")
    for (y, z, t, ha) in [(-my/2, 60, f"mast {mx:.0f}x{my:.0f}", "center"), (177, RIM_Z + 6, "funnel + Scout", "center"), (177, -185, "cradle v5e", "center"),
                          (ys, FLOOR - 14, "cross bar", "center"), (yf - 5, FLOOR - 14, "toe", "right"), (yb + 4, FLOOR - 14, "KB", "left"), (120, FLOOR - 14, "KF", "center"),
                          (-my - 48, -160, "gusset", "right"), (RING_Y[1] + 4, 0, "ring", "left")]:
        ax.text(y, z, t, ha=ha, va="bottom" if z > FLOOR - 10 else "top", fontsize=7.5, color="#333")
    ax.set_xlim(-135, 300); ax.set_ylim(-230, 262); ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
    # ---- front view (X-Z, from +Y)
    ax = axF; ax.set_title("front  (X across, Z up; from +Y)", loc="left", fontsize=9.5, color="#222")
    ax.axhline(FLOOR, color="#777", lw=1.0, zorder=1)
    box(ax, -mx/2, FLOOR, mx, MAST["head_z"][0] - FLOOR, NEW, 0.35, z=2)                         # mast behind the wheel
    box(ax, -MAST["head_x"]/2, MAST["head_z"][0], MAST["head_x"], 80, NEW, 0.45, z=2)
    ax.add_patch(Circle((0, 0), HUB_R, facecolor=BUY, alpha=0.3, lw=0, zorder=3))
    for ang in (0, 90):
        for s in (1, -1):
            ax.plot([0, s*134*math.cos(math.radians(ang))], [0, s*134*math.sin(math.radians(ang))], color=BUY, lw=5, alpha=0.6, zorder=3, solid_capstyle="round")
    ax.add_patch(Circle((0, 0), 100, facecolor="none", edgecolor=SWP, lw=0.7, ls="--", zorder=4))
    ax.add_patch(Circle((0, 0), HEAD_SWEEP_R, facecolor="none", edgecolor=SWP, lw=0.7, ls="--", zorder=4))
    for p in BAY_POS:                                                                            # ring: frames and bays
        a0, a1 = p - 15, p + 15
        if p in BAYS_NOW:
            ax.add_patch(Wedge((0, 0), RING_R_ENV, a0, a1, width=RING_R_ENV - 100, facecolor=OLD, alpha=0.7, lw=0, zorder=5))
        else:
            ax.add_patch(Wedge((0, 0), RING_R_ENV, a0, a1, width=RING_R_ENV - RING_R_IN, facecolor=OLD, alpha=0.45, lw=0, zorder=5))
        for a in (a0, a1):                                                                        # lugs on the side planes
            ca, sa = math.cos(math.radians(a)), math.sin(math.radians(a))
            ax.plot([165*ca, 179*ca], [165*sa, 179*sa], color=OLD, lw=3.5, alpha=0.8, zorder=5, solid_capstyle="butt")
    for s in (1, -1):                                                                             # adapters: top lug -> pads
        ax.plot([s*121.6, s*PAD["x"]], [121.6, PAD["z"]], color=NEW, lw=4, zorder=6, solid_capstyle="round")
        ax.plot(s*PAD["x"], PAD["z"], "o", ms=4, color=NEW, zorder=7)
    ax.add_patch(Polygon([(-DRONE_COL_X, DRONE_COL_Z), (DRONE_COL_X, DRONE_COL_Z), (130, RIM_Z), (-130, RIM_Z)], closed=True, facecolor="none", edgecolor="#777", lw=0.8, ls="--", zorder=4))
    box(ax, -70, 100, 140, 110, "#777", 1, fill=False, ls=":")
    ax.add_patch(Polygon([(-xs, FLOOR), (xs, FLOOR), (xs, FLOOR + BAR["h"][1]), (20, FLOOR + BAR["h"][0]), (-20, FLOOR + BAR["h"][0]), (-xs, FLOOR + BAR["h"][1])], closed=True, facecolor=NEW, alpha=0.9, lw=0, zorder=6))
    box(ax, -CRADLE["lip_x"], FLOOR, 2*CRADLE["lip_x"], CRADLE["top"] - FLOOR, OLD, 0.8)
    for (x, z, t, ha) in [(0, FLOOR - 14, f"cross bar  X ±{xs:.0f}", "center"), (0, RIM_Z + 6, "funnel", "center"), (0, 185, "Scout", "center"),
                          (-175, 10, "frames", "right"), (120, -150, "bay", "left"), (-128, 160, "adapter", "right"), (0, 50, "mast + hub", "center"), (0, FLOOR + 30, "cradle", "center")]:
        ax.text(x, z, t, ha=ha, va="bottom" if z > FLOOR - 10 else "top", fontsize=7.5, color="#333", zorder=8)
    ax.set_xlim(-225, 225); ax.set_ylim(-230, 262); ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
    # ---- plan view (X-Y)
    ax = axP; ax.set_title("plan  (X across, Y along the axle; from above)   dashed = tip-over hull, × = CG", loc="left", fontsize=9.5, color="#222")
    hull = _hull(np.array(contacts, float))
    ax.add_patch(Polygon([tuple(h) for h in hull], closed=True, facecolor="#eef3f6", edgecolor="#777", lw=0.8, ls="--", zorder=1))
    box(ax, -100, -104, 200, 64, "#555", 1, fill=False, ls="--"); ax.text(0, -72, "PSU", ha="center", va="center", fontsize=7.5, color="#555")
    box(ax, -20, yb, 40, ys + 10 - yb, NEW, 0.8)                                                  # KB
    box(ax, -20, KF["y"][0], 40, 194 - KF["y"][0], NEW, 0.65)                                     # KF
    box(ax, -mx/2 - 0, -my, mx, my, NEW, 1.0)                                                     # mast
    box(ax, -MAST["head_x"]/2, -my, MAST["head_x"], my, NEW, 0.35)
    box(ax, -xs, ys - 10, 2*xs, 20, NEW, 0.9)                                                     # cross bar
    box(ax, -20, 197, 40, yf - 197, NEW, 0.9)                                                     # toe
    box(ax, -CRADLE["lip_x"], CRADLE["y"][0], 2*CRADLE["lip_x"], 57, OLD, 0.7)                    # cradle
    box(ax, -RING_R_LUG, RING_Y[0], 2*RING_R_LUG, RING_Y[1] - RING_Y[0], OLD, 0.25)               # ring
    box(ax, -HUB_R, HUB_Y[0], 2*HUB_R, HUB_Y[1] - HUB_Y[0], BUY, 0.35)                            # hub
    box(ax, -134, 74, 268, 20, BUY, 0.5)                                                          # bars
    box(ax, -130, 93, 260, 168, "#777", 1, fill=False, ls="--")                                   # funnel (E)
    ax.plot(c_with[0], c_with[1], "x", ms=9, mew=1.6, color="#222", zorder=9)
    for (x, y) in contacts[:4]: ax.plot(x, y, "o", ms=4, color="#222", zorder=9)
    for (x, y, t, ha) in [(0, yb - 6, f"KB back  Y {yb:.0f}", "center"), (0, yf + 6, f"toe  Y {yf:.0f}", "center"), (xs + 6, ys, f"X {xs:.0f}", "left"), (-xs - 6, ys, f"X −{xs:.0f}", "right"),
                          (195, 177, "ring + funnel", "left"), (100, 40, "hub", "left"), (-(mx/2) - 6, -my/2, "mast", "right"), (c_with[0] + 6, c_with[1] + 4, f"CG  Y {c_with[1]:.0f}", "left")]:
        ax.text(x, y, t, ha=ha, va="center", fontsize=7.5, color="#333")
    ax.set_xlim(-235, 240); ax.set_ylim(-130, 290); ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
    for a in (axS, axF, axP):
        for sp in a.spines.values(): sp.set_visible(False)
    # ---- text panel
    axT.axis("off")
    L = [("Box stand v6: all printed", 10.5, "bold"), ("", 4, ""),
         ("Pieces (new, PETG)", 9, "bold")]
    for n, q, d, f, g, _, s, nt in fits:
        L.append((f"{q}× {n}: {d[0]:.0f}×{d[1]:.0f}×{d[2]:.0f}, ~{g:.0f} g (E)", 8, ""))
    L += [(f"new prints ~{g_total:.0f} g (E); kept: cradle v5e, bay module v4i, stub v3", 8, ""),
          ("retired: 4 seat pads, 1.92 m of 2040, 8 brackets", 8, ""), ("", 4, ""),
          ("Tip-over (13 N push on the rim Z 237 and the wheel top, ±X ±Y; E)", 9, "bold"),
          (f"{m_with:.1f} kg with the PSU on the keel: min margin {k_with[0]:.2f}", 8, ""),
          (f"{m_without:.1f} kg without it: {k_without[0]:.2f}  (v5e: 1.8 at 6.8 kg)", 8, ""),
          (f"static tilt: side {tilt_side:.0f}°, forward {tilt_fwd:.0f}°, back {tilt_back:.0f}°", 8, ""),
          (f"feet from the search: back Y {yb:.0f}, cross bar Y {ys:.0f} to X ±{xs:.0f}, toe Y {yf:.0f}", 8, ""), ("", 4, ""),
          ("Mast (E, PETG 2 GPa)", 9, "bold"),
          (f"sleeve {mx:.0f}×{my:.0f} wall {MAST['wall']} over KB's spigot {sp_a:.1f}×{sp_b:.1f} wall 3, engaged {engage:.0f}", 8, ""),
          (f"2 kg arm + 20 N head push: lean {mast_pick['lean_deg']:.2f}° = {mast_pick['drop_at_Y222_mm']:.1f} mm at Y 222; yaw {mast_pick['yaw_deg']:.2f}°", 8, ""),
          (f"stress {mast_pick['stress_MPa']:.1f} MPa; spigot root {spigot['root_stress_MPa']:.1f} MPa; joint slop {joint_slop_deg:.2f}°", 8, ""), ("", 4, ""),
          ("Ring (E)", 9, "bold"),
          ("7 frames + 2 bays chain from the cradle both ways; the 2 end", 8, ""),
          ("adapters carry the funnel pads and close the ring into an arch", 8, ""),
          (f"13 N on the rim: {root_M:.1f} N·m per chain root, ~{lug_F:.0f} N on a lug", 8, ""), ("", 4, ""),
          ("Hardware", 9, "bold")]
    import textwrap
    for k, v in hardware.items():
        for i, ln in enumerate(textwrap.wrap(f"{k}: {v}", 78)): L.append((("" if i == 0 else "   ") + ln, 7.5, ""))
    y = 0.995
    for t, fs, w in L:
        if t: axT.text(0.0, y, t, fontsize=fs, fontweight=w or "normal", va="top", ha="left", color="#222", wrap=True, transform=axT.transAxes)
        y -= (fs + 3.2) / 560
    fig.text(0.035, 0.965, "Nox box stand v6  (all printed; W coords; new parts teal, existing printed parts tan, bought arm parts grey, sweeps dashed orange)", fontsize=11, color="#222")
    fig.savefig(path, facecolor="white"); print("figure", path)

figure()
