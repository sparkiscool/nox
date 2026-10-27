"""Scout v3 electronics in the pointed tail (concept, 27 Oct 2026).

Spark 27 Oct: "check the new scout cage design it should have a pointy tail and you need to fit the stuff there".
Context: the race-car cage (Scout v3 design session, artifact WHdjYkzKPifXuHT1dWeqoR v5), meshes and rods saved in
runs/scout_electronics/v1/ctx/ (E until that layout run is on main). Electronics list: nox/scout/v3/README.md
"Electronics mount" (battery side not touched).

What moves to the tail: Radxa Zero 3W, the 5 V UBEC and the XR2. The FC stays on the plate centre (IMU, motor leads) and the
flow stays under the belly (its own view, PX4 offset). Two layouts are drawn and checked the same way:
  A  tail fin: one printed fin on the centre plane hung from the plate tail; Radxa on edge on its -y face, UBEC on its +y face,
     XR2 at the top aft corner, antenna up into the free air behind the props
  B  tail tray: a flat tray in the tail cone behind the rear prop disks, Radxa flat on top, UBEC under it, XR2 at the back
Checks (each reported, the system pass decides): cage first (every rod surface >= 3 outside the electronics and the box
inside the shell), rear prop wash (>= 3 outside the swept disks' columns, rule 8), receiver / cartridge / swap column
clearance, CG pack in / out against the layout's own (rule 4, gate 3), cable runs.

Run: python3 nox/scout/v3/electronics_tail.py [ctx dir] [out dir]
"""
import json, os, sys
import numpy as np, trimesh
from scipy.spatial import ConvexHull, Delaunay

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
CTX = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "runs/scout_electronics/v1/ctx")
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.join(ROOT, "runs/scout_electronics/v1")
ctx = json.load(open(os.path.join(CTX, "ctx.json")))
P = {n: (trimesh.load(os.path.join(CTX, v["file"])), v["mass"], v["group"]) for n, v in ctx["parts"].items()}
RODS = [(np.array(r["a"]), np.array(r["b"]), r["r"], r["set"]) for r in ctx["rods"]]

# ---------------------------------------------------------------- numbers (dimensions.md M / H; E where open)
RADXA = (65.0, 30.0, 6.31)          # board L x W, body thickness without header (H, P); SD end 7.62 (H)
RADXA_SD = 7.62
STANDOFF = 3.0                       # PCB back off the fin: clears the SD slot (2.86 under the PCB, H) (C)
UBEC = (50.0, 25.0, 10.0)           # worst of the makers' range (M Q4: 43-50 x 17-25 x 7-10) (E, DEFAULT until measured)
XR2 = (16.0, 12.0, 6.0)             # M spec; tower antenna ~14 above the board (E)
XR2_ANT = 14.0
FIN_T = 2.4                          # fin web (>= 2.0 rule) (C)
FIN_MASS = 6.0                       # fin + 4 bosses + root lug, PETG, before topology (E)
TRAY_MASS = 9.0                      # option B tray + hanger (E)
WIRES_TAIL = 4.0                     # extra lead length to the tail (UART, 5 V, XR2, camera USB) (E)
CAGE_FIRST, WASH, GAP = 3.0, 3.0, 2.0
MOVED = ("Radxa Zero 3W", "UBEC", "XR2")
PACK_OUT = ("pack 4S 1500", "pack XT60 plug", "cartridge v6 (latch)", "cartridge v6 strips (latch, TPU)",
            "cartridge skid closed end (latch, TPU)", "cartridge skid lead end (latch, TPU)")

# rear prop disks: from the rear motors, swept radius from the props mesh (E: layout model)
pb = P["props (swept)"][0].bounds
R_DISK = pb[1][0] - P["motor 1"][0].bounds.mean(0)[0]
Z_DISK = pb[0][2]
REAR = [P[m][0].bounds.mean(0)[:2] for m in ("motor 3", "motor 4")]

# shell: hull of the shell node centres (cage first: the electronics live inside it)
nodes = P["cage shell nodes"][0]
cent = np.array([c.centroid for c in nodes.split(only_watertight=False)])
SHELL = Delaunay(cent)


def box(lo, hi):
    return trimesh.creation.box(bounds=[lo, hi])


def surf_pts(m, n=3000):
    p, _ = trimesh.sample.sample_surface_even(m, n, seed=1)
    return np.vstack([p, m.vertices])


def rod_gap(pts):
    """least rod-surface distance from the points, and the rod"""
    best = (1e9, None)
    for a, b, r, s in RODS:
        d = b - a; t = np.clip(((pts - a) @ d) / (d @ d), 0, 1)
        g = np.linalg.norm(pts - (a + np.outer(t, d)), axis=1).min() - r
        if g < best[0]: best = (g, (s, a.round(1).tolist(), b.round(1).tolist()))
    return best


def wash_gap(pts):
    """least plan distance outside the rear disks' columns (points under the disk plane only)"""
    q = pts[pts[:, 2] < Z_DISK]
    if not len(q): return 1e9
    return min((np.linalg.norm(q[:, :2] - c, axis=1) - R_DISK).min() for c in REAR)


def part_gap(pts, names):
    out = {}
    for n in names:
        out[n] = float(trimesh.proximity.ProximityQuery(P[n][0]).signed_distance(pts).max() * -1)
    return out


def cg(items):
    m = sum(w for _, w in items); return np.sum([c * w for c, w in items], axis=0) / m, m


def mass_cent(mesh):
    return mesh.center_mass if mesh.is_watertight and mesh.volume > 1 else mesh.bounds.mean(0)


BASE = [(mass_cent(m), w) for n, (m, w, g) in P.items() if w > 0]


def layout_cg(placed, extra):
    """CG pack in / pack out with MOVED replaced by the placed boxes and the mount added"""
    it = [(mass_cent(m), w) for n, (m, w, g) in P.items() if w > 0 and n not in MOVED]
    it += [(mesh.bounds.mean(0), P[n][1]) for n, mesh in placed.items()] + extra
    out = [(mass_cent(m), w) for n, (m, w, g) in P.items() if w > 0 and n in PACK_OUT]
    c_in, m_in = cg(it)
    m_out = m_in - sum(w for _, w in out)
    c_out = (c_in * m_in - np.sum([c * w for c, w in out], axis=0)) / m_out
    return c_in, c_out, m_in


# ---------------------------------------------------------------- option A: tail fin
def fin_mesh(yf, poly):
    """the fin: a side-view polygon (x, z) extruded FIN_T about the plane y = yf"""
    from shapely.geometry import Polygon
    m = trimesh.creation.extrude_polygon(Polygon(poly), FIN_T)        # in (x, z), thickness along +w
    T = np.array([[1, 0, 0, 0], [0, 0, -1, yf + FIN_T / 2], [0, 1, 0, 0], [0, 0, 0, 1]], float)
    return m.apply_transform(T)


def option_a():
    yf = 5.0                                      # fin plane (C): the faces 2.3 clear of the tail core tie (centre plane, r 1.5)
    x0 = -65.0                                    # Radxa nose end: 5 behind the receiver TPU end pieces (-60) (C)
    rad_y0 = yf - FIN_T / 2 - STANDOFF - 1.6 - 3.16   # PCB back on 3.0 standoffs, component side out, USB-C tops (H)
    rad = box([x0 - RADXA[0], rad_y0, -24.0], [x0, yf - FIN_T / 2 - STANDOFF, -24.0 + RADXA[1]])
    ub = box([-88.0 - UBEC[0], yf + FIN_T / 2, -12.0], [-88.0, yf + FIN_T / 2 + UBEC[2], -12.0 + UBEC[1]])
    xr = box([-132.0, yf + FIN_T / 2, 16.0], [-132.0 + XR2[1], yf + FIN_T / 2 + XR2[2], 16.0 + XR2[0]])
    ant = box([-128.0, yf + FIN_T / 2 + 1.0, 32.0], [-124.0, yf + FIN_T / 2 + 5.0, 32.0 + XR2_ANT])
    # side view: root lug over the receiver top onto the belly mount's tail face (x -56, z 11.6..18.5, under the plate),
    # front edge 2 behind the receiver end pieces, Radxa land z -26..6, rising aft to the XR2 at the top
    fin = fin_mesh(yf, [(-56.0, 11.6), (-56.0, 18.5), (-66.0, 18.5), (-133.0, 34.0), (-133.0, -26.0), (-62.0, -26.0), (-62.0, 11.6)])
    return dict(name="A tail fin", placed={"Radxa Zero 3W": rad, "UBEC": ub, "XR2": trimesh.util.concatenate([xr, ant])},
                mount={"tail fin (PETG)": fin}, extra=[(fin.center_mass, FIN_MASS), (np.array([-80.0, 0, 0]), WIRES_TAIL)],
                note="fin plane y %.1f; Radxa on edge on the -y face, ports up, SD end aft; UBEC on the +y face behind the disks' "
                     "wash; XR2 top aft on the +y face, antenna up" % yf)


# ---------------------------------------------------------------- option B: tail tray in the cone
def option_b():
    xb = -104.0                                   # tray nose: behind the rear disks (x -98.5) + wash margin (C)
    # the lengthwise Radxa (65 along x) does not fit the pointed cone (rods at x -150..-169), so it lies crosswise
    rad = box([xb - RADXA[1], -RADXA[0] / 2, 6.0], [xb, RADXA[0] / 2, 6.0 + RADXA_SD])
    ub = box([xb - UBEC[1], -UBEC[0] / 2, 6.0 - 2.4 - UBEC[2]], [xb, UBEC[0] / 2, 6.0 - 2.4])
    xr = box([xb - RADXA[1] - 2 - XR2[1], -XR2[0] / 2, 6.0], [xb - RADXA[1] - 2, XR2[0] / 2, 6.0 + XR2[2] + XR2_ANT])
    tray = box([xb - RADXA[1] - 16, -RADXA[0] / 2 - 2, 3.6], [-58.0, RADXA[0] / 2 + 2, 6.0])
    return dict(name="B tail tray", placed={"Radxa Zero 3W": rad, "UBEC": ub, "XR2": xr},
                mount={"tail tray (PETG)": tray}, extra=[(tray.bounds.mean(0), TRAY_MASS), (np.array([-100.0, 0, 0]), WIRES_TAIL)],
                note="tray z 3.6..6.0 from the plate tail; Radxa flat and crosswise behind the disks, component side up; UBEC under; XR2 behind")


def check(opt):
    r = {"option": opt["name"], "note": opt["note"], "parts": {}}
    keep = ("receiver v3 (latch)", "receiver TPU end pieces (latch, E)", "cartridge v6 (latch)", "FC", "frame plate (Bee35 carbon)",
            "mount + nose legs (Scout)", "flow MTF-02P")
    for n, m in {**opt["placed"], **opt["mount"]}.items():
        pts = surf_pts(m)
        g, rod = rod_gap(pts)
        inside = bool(SHELL.find_simplex(pts).min() >= 0)
        r["parts"][n] = {"bounds": m.bounds.round(1).tolist(), "rod_gap": round(float(g), 1), "nearest_rod": rod,
                         "inside_shell": inside, "wash_gap": round(float(wash_gap(pts)), 1),
                         "part_gap": {k: round(v, 1) for k, v in part_gap(pts, keep).items()}}
    c_in, c_out, mass = layout_cg(opt["placed"], opt["extra"])
    r["cg_in"], r["cg_out"], r["mass"] = c_in.round(2).tolist(), c_out.round(2).tolist(), round(mass, 1)
    r["cg_off_in"], r["cg_off_out"] = round(float(np.hypot(*c_in[:2])), 2), round(float(np.hypot(*c_out[:2])), 2)
    # the cage session's balance scan slides the catch set (receiver, cartridge, pack, belly mount, FC, flow) fore-aft;
    # estimate the slide that centres pack in and pack out with the tail bay as placed (E: the scan decides)
    cs = [n for n in P if n.startswith(("receiver", "cartridge", "slider", "pack", "mount + nose", "XT60")) or n in ("FC", "flow MTF-02P")]
    m_cs_in = sum(P[n][1] for n in cs); m_cs_out = m_cs_in - sum(P[n][1] for n in PACK_OUT)
    m_out = mass - sum(P[n][1] for n in PACK_OUT)
    # x_in + d m_cs_in / mass and x_out + d m_cs_out / m_out: the d that minimises the larger |x|
    ds = np.linspace(-30, 30, 1201)
    worst = [max(abs(c_in[0] + d * m_cs_in / mass), abs(c_out[0] + d * m_cs_out / m_out)) for d in ds]
    k = int(np.argmin(worst)); r["reslide"] = {"catch_set_dx": round(float(ds[k]), 1), "cg_x_worst": round(float(worst[k]), 2)}
    el = [n for n in opt["placed"]]
    r["ok"] = {"cage_first": all(r["parts"][n]["rod_gap"] >= CAGE_FIRST and r["parts"][n]["inside_shell"] for n in el),
               "wash": all(r["parts"][n]["wash_gap"] >= WASH for n in r["parts"]),
               "fit": all(min(p["part_gap"].values()) >= GAP for n, p in r["parts"].items() if n in el),
               "balance": max(r["cg_off_in"], r["cg_off_out"]) <= 3.0}
    return r


def picture(opts, res, path):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    col = {"Radxa Zero 3W": "#2e7d32", "UBEC": "#c62828", "XR2": "#6a1b9a"}
    fig, ax = plt.subplots(len(opts), 2, figsize=(15, 6.3 * len(opts)))
    for k, (opt, rr) in enumerate(zip(opts, res)):
        for v, (i, j, lab) in enumerate(((0, 2, "side (from -y): x fwd, z up"), (0, 1, "top: x fwd, y left"))):
            a = ax[k][v]
            for A, B, rad, s in RODS:
                a.plot([A[i], B[i]], [A[j], B[j]], color="#333" if s == "cage " else "#888", lw=1.1)
            for n in ("frame plate (Bee35 carbon)", "receiver v3 (latch)", "cartridge v6 (latch)", "FC", "flow MTF-02P",
                      "mount + nose legs (Scout)", "camera", "camera mount (boom + seat)", "pack 4S 1500"):
                b = P[n][0].bounds
                a.add_patch(plt.Rectangle((b[0][i], b[0][j]), b[1][i] - b[0][i], b[1][j] - b[0][j], fc="#c8ced6", ec="#8a94a0", lw=0.6, alpha=0.6))
            if j == 1:
                for c in REAR + [P[m][0].bounds.mean(0)[:2] for m in ("motor 1", "motor 2")]:
                    a.add_patch(Circle(c, R_DISK, fc="#f2c94c", ec="#d9a400", alpha=0.25))
                    a.add_patch(Circle(c, R_DISK + WASH, fill=False, ec="#d9a400", ls="--", lw=0.8))
            else:
                a.plot([pb[0][0], pb[1][0]], [Z_DISK, Z_DISK], color="#d9a400", lw=3, alpha=0.6)
            for n, m in {**opt["mount"], **opt["placed"]}.items():
                b = m.bounds; q = m.vertices[:, [i, j]]; q = q[ConvexHull(q).vertices]
                a.add_patch(plt.Polygon(q, fc=col.get(n, "#90a4ae"), ec="k", lw=0.8, alpha=0.85 if n in col else 0.45))
                if n in col: a.text(b.mean(0)[i], b[1][j] + 2, n.split()[0], ha="center", fontsize=8)
            c_in = np.array(rr["cg_in"]); a.plot(c_in[i], c_in[j], "k+", ms=12, mew=2)
            a.set_aspect("equal"); a.set_xlim(-200, 190); a.set_ylim(-60, 75) if j == 2 else a.set_ylim(-145, 145)
            a.set_title(f"{opt['name']}: {lab}", fontsize=10); a.grid(alpha=0.2)
    fig.suptitle("Scout v3 electronics in the pointed tail (concept): green Radxa, red UBEC (worst-size envelope), purple XR2 + antenna,"
                 " grey-blue = mount; yellow = prop disks, dashed = disk + 3 wash margin; + = CG pack in", fontsize=10)
    fig.tight_layout(); fig.savefig(path, dpi=110); plt.close(fig)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    c0_in, c0_out, m0 = layout_cg({n: trimesh.creation.box(bounds=P[n][0].bounds) for n in MOVED}, [])
    opts = [option_a(), option_b()]
    res = [check(o) for o in opts]
    base = {"cg_in": c0_in.round(2).tolist(), "cg_out": c0_out.round(2).tolist(), "mass": round(m0, 1),
            "R_disk": round(float(R_DISK), 1), "rear_disk_centres": [c.round(1).tolist() for c in REAR]}
    json.dump({"layout_as_drawn": base, "options": res}, open(os.path.join(OUT, "result.json"), "w"), indent=1)
    picture(opts, res, os.path.join(OUT, "preview.png"))
    print("layout as drawn (tail bay of the cage session):", base)
    for r in res:
        print(f"\n{r['option']}: {r['note']}\n  CG pack in {r['cg_in']} ({r['cg_off_in']}), pack out {r['cg_out']} ({r['cg_off_out']}), {r['mass']} g; "
              f"catch set re-slide {r['reslide']}; ok {r['ok']}")
        for n, p in r["parts"].items():
            print(f"  {n:18s} rod {p['rod_gap']:6.1f} inside {p['inside_shell']}  wash {p['wash_gap']:7.1f}  min part gap "
                  f"{min(p['part_gap'].values()):6.1f} ({min(p['part_gap'], key=p['part_gap'].get)})  {p['bounds']}")
