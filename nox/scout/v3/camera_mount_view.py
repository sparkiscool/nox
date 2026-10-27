"""Scout camera mount v1: the motion artifact (nox/motion.py) and stills. The mount on the v12 cage with the sized tubes
(6/4 on the frame model's 39, 3.0 on the rest), the camera envelope and the nose of the drone; one cycle: the mount pushed on
along -x (4 clips snap onto rods 7-8 and 14-15), the stop collars slid in along the bottom rod, then held.

python3 nox/scout/v3/camera_mount_view.py [--video] [--stills]   -> runs/scout_camera_mount/v1/motion.html (+ motion.mp4/.gif/.png)
"""
import os, sys, json, math
import numpy as np, trimesh

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
from motion import Scene

RUN = os.path.join(ROOT, "runs", "scout_camera_mount", "v1"); LAY = os.path.join(ROOT, "runs", "scout_layout", "v12")
cg = json.load(open(os.path.join(LAY, "cage_shell.json"))); res = json.load(open(os.path.join(LAY, "result.json")))
up6 = set(json.load(open(os.path.join(ROOT, "runs", "scout_frame", "v2", "alt_sized", "up6.json"))))
N = np.array(cg["nodes"], float); nR = len(cg["open"][0]["ring"])


def rod(p, q, r):
    p, q = np.asarray(p, float), np.asarray(q, float); L = float(np.linalg.norm(q - p))
    c = trimesh.creation.cylinder(radius=r, height=L, sections=16)
    c.apply_transform(trimesh.geometry.align_vectors([0, 0, 1], (q - p) / L)); c.apply_translation((p + q) / 2); return c


tubes6, rods3 = [], []
for a, b in cg["rods"]:
    (tubes6 if f"rod {a}-{b}" in up6 else rods3).append(rod(N[a], N[b], 3.0 if f"rod {a}-{b}" in up6 else 1.5))
for k, (p, q) in res["links"].items():
    (tubes6 if k in up6 else rods3).append(rod(p, q, 3.0 if k in up6 else 1.5))
nodes = trimesh.util.concatenate([trimesh.creation.icosphere(2, radius=10.15 if i < nR else 6.0).apply_translation(N[i]) for i in range(len(N))])
mount = trimesh.load(os.path.join(RUN, "camera_mount_assembled.stl")); col = trimesh.load(os.path.join(RUN, "stop_collar_assembled.stl"))
col_m = col.copy(); col_m.apply_transform(np.diag([1, -1, 1, 1])); col_m.invert()
cam = trimesh.creation.box(extents=[14, 38, 38]); cam.apply_translation([142, 0, 42])
lens = trimesh.creation.cylinder(radius=6.0, height=4.0, sections=32); lens.apply_transform(trimesh.geometry.align_vectors([0, 0, 1], [1, 0, 0])); lens.apply_translation([147.0, 0, 42])
ctx = []
for f in ("frame_plate__bee35_carbon", "motor_1", "motor_2", "motor_3", "motor_4", "prop_1", "prop_2", "prop_3", "prop_4", "fc", "receiver_v3__latch", "pack_4s_1500", "tie_saddle__printed"):
    pth = os.path.join(LAY, "parts", f + ".stl")
    if os.path.exists(pth): ctx.append((f, trimesh.load(pth, force="mesh")))

res_m = json.load(open(os.path.join(RUN, "result.json")))
s = Scene("Scout camera mount v1", f"one PETG print {res_m['mass_mount_g']:.1f} g (M) on the 6/4 tubes: lens hood 4.0 ahead, chevron trusses, 4 snap clips (push-on {res_m['snap']['install_N']:.0f} N each, E)")
s.part("camera mount (PETG)", mount, group="mount", color="#c9ced6", printed=True)
s.part("stop collar +y", col, group="mount", color="#e0a040", printed=True)
s.part("stop collar -y", col_m, group="mount", color="#e0a040", printed=True)
s.part("camera (DECXIN 2633V2, envelope)", cam, group="camera", color="#2d5a2d", printed=False)
s.part("lens (r 6, E)", lens, group="camera", color="#101820", printed=False)
s.part("6/4 carbon tubes (sized)", trimesh.util.concatenate(tubes6), group="cage", color="#3a3f47", printed=False)
s.part("3.0 carbon rods", trimesh.util.concatenate(rods3), group="cage", color="#6a7079", printed=False)
s.part("cage nodes (PETG)", nodes, group="cage", color="#8899aa", printed=False)
for f, m in ctx: s.part(f, m, group="drone", color="#556677", printed=False, faces=4000)

T, PUSH, SLIDE = 8.0, 40.0, 10.0
def pose(t):
    a = min(1.0, t / 3.0); a = 0.5 - 0.5 * math.cos(math.pi * a)                 # 0..3 s: pushed on along -x
    b = min(1.0, max(0.0, (t - 3.5) / 2.0)); b = 0.5 - 0.5 * math.cos(math.pi * b)   # 3.5..5.5 s: collars slid in
    dx = PUSH * (1 - a); dy = SLIDE * (1 - b)
    return {"camera mount (PETG)": [dx, 0, 0, 0, 0, 0], "camera (DECXIN 2633V2, envelope)": [dx, 0, 0, 0, 0, 0], "lens (r 6, E)": [dx, 0, 0, 0, 0, 0],
            "stop collar +y": [0, dy, 0, 0, 0, 0], "stop collar -y": [0, -dy, 0, 0, 0, 0]}
s.timeline(pose, T, fps=30)
s.view("close-up", eye=(230, 120, 110), target=(128, 0, 38))
s.view("side", eye=(150, 230, 70), target=(125, 0, 38))
s.view("from inside the cage", eye=(40, 70, 60), target=(128, 0, 38))
s.write(os.path.join(RUN, "motion.html"))
if "--video" in sys.argv:
    s.video(os.path.join(RUN, "motion.mp4"), view="close-up")
if "--stills" in sys.argv:                                  # seated pose, one PNG per view
    s.timeline(lambda t: pose(T), 1.0, fps=2)
    for v, tag in (("close-up", "closeup"), ("side", "side"), ("from inside the cage", "inside")):
        s.video(os.path.join(RUN, f"still_{tag}.mp4"), view=v, gif=False, png=True); os.remove(os.path.join(RUN, f"still_{tag}.mp4"))
