"""Motion artifact for funnel panel v9 plain (CLAUDE.md "Every idea comes with a visual"): the box catch in box W
(nox/box/box_catch.assembly.json: panels v7 on +-y, v9 plain on +-x with the keep-alive probes and caps, hook latch v2) and a
seated drone's catch context (wheels + ring, hook_latch.drone_parts, E) carrying the two keep-alive pads (copper 22 x 12 x 1.6
on the lower ring, face 2.0 off the funnel face seated, y +6..+28 on +x / -28..-6 on -x, 8 below to 4 above the probe line
along the slope; drone side, agreed, E): the drone lands (30 above its seat, straight down), sits, the XT60 push lifts it
5.7 and lets it back, then it takes off. The 4 probe pins follow the pads (tip = the pad face, within free 8.0 .. solid
0.5 off the face); the strip reads the tip and the spring force per frame. The arm's drone insert lane (carriage) and
head / grabber keep-outs (box_structure_v2.assembly.json, M) are shown under the -x face against its notch.
python3 probe_motion.py [--no-video] -> runs/funnel_panel/v9_plain/motion.html (+ .mp4 / .gif / .png)"""
import json, math, os, sys
import numpy as np, trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
sys.path[:0] = [HERE, os.path.dirname(HERE), os.path.join(ROOT, "nox")]
import catch_system as CS
import assembly as AS
from motion import Scene

RUN = os.path.join(ROOT, "runs", "funnel_panel", "v9_plain")
FPS, DUR = 10, 12.0
S2 = math.sqrt(2)
TIP_FREE, TIP_SOLID, SEAT_OFF, LIFT = 8.0, 0.5, 2.0, 5.7      # C (funnel_panel_v9.py, keep-alive concept)
K, X0 = 0.045, 12.0                                            # spring rate N/mm, preload compression (E)
H0 = 30.0                                                      # drone start / end height over its seat (shown)


def dz_at(t):
    s = lambda a, b: min(max((t - a) / (b - a), 0.0), 1.0)
    e = lambda x: x * x * (3 - 2 * x)
    if t < 3.0: return H0 * (1 - e(s(0, 3)))
    if t < 4.0: return 0.0
    if t < 5.0: return LIFT * e(s(4, 5))
    if t < 6.5: return LIFT
    if t < 7.5: return LIFT * (1 - e(s(6.5, 7.5)))
    if t < 9.0: return 0.0
    return H0 * e(s(9, 12))


def pad_mesh(face):
    """drone keep-alive pad in the seat frame (drone side, E): 22 along the edge x 12 along the slope x 1.6, its face
    SEAT_OFF off the funnel face, 8 below .. 4 above the probe line (z 33) along the slope."""
    v0 = (33.0 - 1.3) * S2
    b = trimesh.creation.box(extents=[1.6, 22.0, 12.0])               # (n, u, v) local
    b.apply_translation([-(SEAT_OFF + 0.8), 17.0, -2.0])             # n < 0: in front of the face (into the mouth)
    nI = np.array([1.0, 0.0, -1.0]) / S2; eV = np.array([1.0, 0.0, 1.0]) / S2; eU = np.array([0.0, 1.0, 0.0])
    p0 = np.array([84.0 + v0 / S2, 0.0, 1.3 + v0 / S2])
    M = np.eye(4); M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = nI, eU, eV, p0
    b.apply_transform(M)
    if face == "-x": b.apply_transform(np.diag([-1.0, -1.0, 1.0, 1.0]))
    return b


def main(video=True):
    a = AS.load(os.path.join(os.path.dirname(HERE), "box_catch.assembly.json"))
    W = CS.SEAT_TO_W
    up = W[:3, :3] @ np.array([0.0, 0.0, 1.0])
    st = json.load(open(os.path.join(os.path.dirname(HERE), "box_structure_v2.assembly.json")))
    sc = Scene("Funnel v9 plain: keep-alive probes and the arm notch",
               "box W. Panels v9 plain on +-x (2 probes in each A panel: B+ on +x, B- on -x; -x pair notched r >= 161 over the "
               "carriage lane), v7 + hook latch v2 on +-y. A drone's catch context with its 2 copper pads lands, sits, rides the "
               "5.7 XT60 lift and leaves; the pins follow the pads (free 8.0, seated 2.0, solid 0.5 off the face; 0.54..0.88 N)")
    pins = [n for n in a.names if n.startswith("probe pin")]
    for p, m in zip(a.parts, a.meshes()):
        nm = p["name"]
        if nm.startswith("drone"):
            sc.part(nm, m, group="drone (context, E)", color="#555555", printed=False, faces=20000); continue
        plain = "v9 plain" in str(p.get("version", "")) and nm.startswith("funnel")
        if nm.startswith("probe pin"): grp, col, pr = "keep-alive probes", "#d4a017", False
        elif nm.startswith("probe cap"): grp, col, pr = "keep-alive probes", "#3a6fd8", True
        elif plain: grp, col, pr = "panels v9 plain (+-x)", "#b9c2cc", True
        elif nm.startswith("funnel"): grp, col, pr = "panels v7 (+-y)", "#7fa7d9", True
        else: grp, col, pr = "hook latch v2", "#cc6644", True
        sc.part(nm, m, group=grp, color=col, printed=pr, faces=20000)
    for face in ("+x", "-x"):
        t = pad_mesh(face); t.apply_transform(W)
        sc.part(f"drone pad {face} ({'B+' if face == '+x' else 'B-'})", t, group="drone (context, E)", color="#c87533", printed=False)
    for p in st["parts"]:
        if "insert lane" in p["name"]:
            t = trimesh.load(os.path.join(ROOT, p["mesh"]), force="mesh")
            sc.part(p["name"] + " keep-out", t, group="arm keep-outs (M)", color="#e05050" if "carriage" in p["name"] else "#f0a0a0",
                    printed=False, faces=8000)
    n = int(DUR * FPS)
    frames, tip_s, f_s, dz_s, lat = [], [], [], [], []
    plungers = [nm for nm in a.names if nm.startswith("latch plunger")]
    for i in range(n):
        t = i / FPS
        dz = dz_at(t)
        T = np.eye(4); T[:3, 3] = up * dz
        pad_off = SEAT_OFF + dz / S2                                   # pad face off the funnel face (straight down approach)
        tip = min(max(pad_off, TIP_SOLID), TIP_FREE)
        push = TIP_FREE - tip
        fr = {nm: T for nm in ("drone wheels (seated)", "drone cage ring (seated)", "drone pad +x (B+)", "drone pad -x (B-)")}
        for nm in pins: fr[nm] = a.move(nm, push - 6.0) @ np.linalg.inv(a.pose(nm))      # relative to the posed mesh
        latch = 1.0 if 3.5 <= t <= 8.5 else 0.0
        for nm in plungers: fr[nm] = a.move(nm, latch) @ np.linalg.inv(a.pose(nm))
        frames.append(fr)
        tip_s.append(tip); f_s.append(K * (X0 + push)); dz_s.append(dz); lat.append(latch)
    sc.timeline(frames, duration_s=DUR)
    sc.series("pin tip off the face", tip_s, unit="mm", bad_below=TIP_SOLID)
    sc.series("pin force (each)", f_s, unit="N", bad_above=1.0)
    sc.series("drone over its seat", dz_s, unit="mm")
    sc.series("hook latch out", lat, unit="")
    c = W[:3, 3].copy()
    px = (W @ np.array([115.7, 17.0, 33.0, 1.0]))[:3]
    nx = (W @ np.array([-104.0, 0.0, 4.0, 1.0]))[:3]
    sc.view("+x probes from inside the mouth", eye=tuple(px + np.array([110.0, -230.0, 170.0])), target=tuple(px))
    sc.view("-x notch over the carriage lane", eye=tuple(nx + np.array([160.0, -60.0, -40.0])), target=tuple(nx))
    sc.view("whole catch", eye=(c[0] + 330, c[1] - 260, c[2] + 300), target=tuple(c))
    print("tip range", round(min(tip_s), 2), round(max(tip_s), 2), "force", round(min(f_s), 3), round(max(f_s), 3))
    out = sc.write(os.path.join(RUN, "motion.html"))
    if video: sc.video(os.path.join(RUN, "motion.mp4"), view="+x probes from inside the mouth")
    return out


if __name__ == "__main__":
    main(video="--no-video" not in sys.argv)
