"""Motion artifact of one landing on the push-bar catch (concept F): the drone descends onto the flat deck, the end pads
close along x and clamp it, the side bars close with their lips, stall. Built from a trace of catch_bars_concept.py
(`--trace <drone> <x> <y> <yaw>`): every 0.5 mm of carriage travel is one frame at 30 fps, about the real carriage speed
(15 mm/s, E). Views: iso, the nose pad close up, the side bar close up, from below the belly.
python3 nox/box/catch_bars_motion.py runs/box_catch/bars_concept/trace_overseer_+40_+30_+10.json [--video]
-> runs/box_catch/bars_concept/motion_<drone>_<x>_<y>_<yaw>.html (+ .mp4 / .gif / .png with --video)
"""
import json, math, os, sys, numpy as np, trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, "nox"))
from motion import Scene

def box_mesh(o, u, t, depth, L, z0, z1):
    """the orthogonal box of catch_bars_concept.rbox: face through o with outward normal u, lateral t over L, depth behind, z0..z1"""
    o = np.array([o[0], o[1], 0.0]); u = np.array([u[0], u[1], 0.0]); t = np.array([t[0], t[1], 0.0])
    c = o - u * depth / 2 + t * L / 2 + [0, 0, (z0 + z1) / 2]
    T = np.eye(4); T[:3, 0] = -u; T[:3, 1] = t; T[:3, 2] = [0, 0, 1]; T[:3, 3] = c
    return trimesh.creation.box(extents=[depth, L, z1 - z0], transform=T)

def pad_mesh(g, xa, sgn):
    """one end pad at apex xa (local X = sgn (x - xa) into the pad): notch, relief, wings, 6 boxes"""
    TV = math.tan(math.radians(g["v_deg"])); TW = math.tan(math.radians(g["wing_deg"])); ny, (ry, rdx), (w0, w1), wc, t = g["notch_y"], g["ramp"], g["wing_y"], g["wing_c"], g["pad_t"]
    e0 = -ny * TV; z0, z1 = g["deck"], g["deck"] + g["h_pad"]; parts = []
    for sy in (1, -1):
        feats = [(0.0, 0.0, (-TV, sy), (-1.0, -sy * TV), ny * math.hypot(1, TV)),
                 (e0, sy * ny, (rdx - e0, sy * (ry - ny)), (-(ry - ny), sy * (rdx - e0)), math.hypot(rdx - e0, ry - ny)),
                 (wc, sy * w0, (-TW, sy), (-1.0, -sy * TW), math.hypot((w1 - w0) * TW, w1 - w0))]
        for X0, y0, tt, uu, L in feats:
            tt = np.array(tt, float) / np.linalg.norm(tt); uu = np.array(uu, float) / np.linalg.norm(uu)
            parts.append(box_mesh((xa + sgn * X0, y0), (sgn * uu[0], uu[1]), (sgn * tt[0], tt[1]), t, L, z0, z1))
    # a back plate joining the six (for the eye only)
    back = box_mesh((xa + sgn * (rdx + t), 0.0), (-sgn, 0.0), (0.0, 1.0), 4.0, 2 * w1, z0, z1)
    back.apply_translation([0, -w1, 0])
    return trimesh.util.concatenate(parts + [back])

def bar_mesh(g, sign, out):
    """one side bar (L-section) with its face `out` outside the tube centre line"""
    H = g["hex"]; A = np.array([H["Lt"][0], sign * H["Lt"][1]]); B = np.array([H["Ln"][0], sign * H["Ln"][1]])
    t = (B - A) / np.linalg.norm(B - A); n = np.array([-t[1], t[0]]);
    if n[1] * sign < 0: n = -n
    xa, xb = g["bar_x"]; L = xb - xa; o = A + t * (xa - A[0]) / t[0] + n * out
    body = box_mesh(o, -n, t, -g["bar_t"], L, g["deck"], g["deck"] + g["h_bar"])          # depth negative: the body is outside the face
    lz0, lt, lin = g["lip"]
    lip = box_mesh(o - n * (g["y_out"] + lin), -n, t, -(g["y_out"] + lin), L, lz0, lz0 + lt)
    return trimesh.util.concatenate([body, lip])

def drone_meshes(tr):
    d = tr["draw"]; N = np.array(d["nodes"], float); rr = d["rod_r"]; node_r = 10.15 if tr["drone"] == "overseer" else 6.0
    rods = []
    for a, b in d["rods"]:
        seg = trimesh.creation.cylinder(radius=rr, segment=[N[a], N[b]], sections=10); rods.append(seg)
    nodes = [trimesh.creation.icosphere(subdivisions=1, radius=node_r if i in d["ring"] else d.get("node_r", 6.0)).apply_translation(p) for i, p in enumerate(N)]
    belly = {}
    for m in d["meshes"]:
        if not m["f"]: continue
        belly[m["name"]] = trimesh.Trimesh(vertices=np.array(m["v"], float), faces=np.array(m["f"], int), process=False)
    return trimesh.util.concatenate(rods), trimesh.util.concatenate(nodes), belly

def build(trace_path, video=False):
    tr = json.load(open(trace_path)); g = tr["geo"]; F = tr["frames"]; name = tr["drone"]
    sc = Scene(f"Deck catch: {name.capitalize()} lands {tr['off'][0]:+.0f} / {tr['off'][1]:+.0f}, yaw {tr['yaw']:+.0f}°",
               f"concept F, one stepper: pads clamp x, then the side bars; {len(F)} carriage steps of 0.5 mm at 30 fps, {tr['status']}")
    deck = trimesh.creation.box(extents=[700, 560, 6], transform=trimesh.transformations.translation_matrix([-30, 0, g["deck"] - 3]))
    sc.part("deck", deck, group="box", color="#b9c0bb", printed=False)
    sc.part("nose pad", pad_mesh(g, g["nose_park"], 1), group="box", color="#b8512b", printed=True)
    sc.part("tail pad", pad_mesh(g, g["tail_park"], -1), group="box", color="#b8512b", printed=True)
    sc.part("side bar +y", bar_mesh(g, 1, g["y_park"]), group="box", color="#c2612f", printed=True)
    sc.part("side bar -y", bar_mesh(g, -1, g["y_park"]), group="box", color="#c2612f", printed=True)
    rods, nodes, belly = drone_meshes(tr)
    col = "#4f77a6" if name == "overseer" else "#1d8676"
    sc.part("cage rods", rods, group=name, color=col, printed=False); sc.part("cage nodes", nodes, group=name, color="#2b3a33", printed=True)
    for k, m in belly.items(): sc.part(k, m, group=name, color="#8a9490" if "feet" not in k else "#444a47", printed=False)
    drone_parts = ["cage rods", "cage nodes"] + list(belly.keys())
    fps = 30; t_desc, t_hold = 1.5, 1.5; n = len(F); dur = t_desc + n / fps + t_hold
    H = g["hex"]; sgn_n = {}
    for sign in (1, -1):
        A = np.array([H["Lt"][0], sign * H["Lt"][1]]); B = np.array([H["Ln"][0], sign * H["Ln"][1]]); t = (B - A) / np.linalg.norm(B - A); nn = np.array([-t[1], t[0]])
        if nn[1] * sign < 0: nn = -nn
        sgn_n[sign] = nn
    def poses(t):
        if t < t_desc: f = F[0]; h = 60.0 * (1 - t / t_desc) ** 2
        else: f = F[min(n - 1, int((t - t_desc) * fps))]; h = 0.0
        dx, dy, th = f["pose"]; c, s = math.cos(th), math.sin(th)
        Td = np.array([[c, -s, 0, dx], [s, c, 0, dy], [0, 0, 1, h], [0, 0, 0, 1]])
        P = {p: Td for p in drone_parts}
        P["nose pad"] = trimesh.transformations.translation_matrix([f["xn"] - g["nose_park"], 0, 0])
        P["tail pad"] = trimesh.transformations.translation_matrix([f["xt"] - g["tail_park"], 0, 0])
        for sign, nm in ((1, "side bar +y"), (-1, "side bar -y")):
            d_out = f["out"] - g["y_park"]; P[nm] = trimesh.transformations.translation_matrix([sgn_n[sign][0] * d_out, sgn_n[sign][1] * d_out, 0])
        P["deck"] = np.eye(4); return P
    sc.timeline(poses, duration_s=dur, fps=fps)
    nf = max(2, int(round(dur * fps)))
    def at(i):
        t = i * dur / nf
        return F[0] if t < t_desc else F[min(n - 1, int((t - t_desc) * fps))]
    sc.series("x off", [at(i)["pose"][0] for i in range(nf)], unit="mm")
    sc.series("y off", [at(i)["pose"][1] for i in range(nf)], unit="mm")
    sc.series("yaw", [math.degrees(at(i)["pose"][2]) for i in range(nf)], unit="deg")
    sc.series("phase", [at(i)["phase"] for i in range(nf)], unit="")
    sc.view("iso", (420, -420, 320), (0, 0, 0))
    sc.view("nose pad close up", (330, -150, 90), (205, 0, 0))
    sc.view("side bar close up", (60, 300, 110), (0, 125, 0))
    sc.view("tail pad close up", (-360, 150, 90), (-225, 0, 0))
    sc.view("from below", (0, -5, -900), (0, 0, 0))
    out = os.path.join(os.path.dirname(trace_path), os.path.basename(trace_path).replace("trace_", "motion_").replace(".json", ""))
    sc.write(out + ".html"); print("wrote", out + ".html")
    if video: sc.video(out + ".mp4"); print("wrote", out + ".mp4")

if __name__ == "__main__":
    build(sys.argv[1], video="--video" in sys.argv)
