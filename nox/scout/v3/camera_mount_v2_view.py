"""Scout camera mount v2: the motion artifact (nox/motion.py) and stills. The topology mount on the v12 cage with every rod 3.0,
the camera, the plate, props, belly mount and the nose of the drone; one cycle: the mount (camera on) pushed on along -x by the
110 mm stroke through the bottom-front cell (2 clips snap onto rod 14-15 next to nodes 14 / 15, the foot slides onto the plate
top), then the 2 longer M3 of the belly mount's front pair go down through the foot's slots, then held.

python3 nox/scout/v3/camera_mount_v2_view.py [--video] [--stills]   -> runs/scout_camera_mount/v2/motion.html (+ .mp4/.gif/.png)
"""
import os, sys, json, math
import numpy as np, trimesh

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox")); sys.path.insert(0, HERE)
from motion import Scene
import camera_mount_v2 as cm

RUN = os.path.join(ROOT, "runs", "scout_camera_mount", "v2"); LAY = cm.LAY
N = cm.N


def rod(p, q, r):
    p, q = np.asarray(p, float), np.asarray(q, float); L = float(np.linalg.norm(q - p))
    c = trimesh.creation.cylinder(radius=r, height=L, sections=12)
    c.apply_transform(trimesh.geometry.align_vectors([0, 0, 1], (q - p) / L)); c.apply_translation((p + q) / 2); return c


rods = [rod(N[a], N[b], 1.5) for a, b in cm.CG["rods"]] + [rod(p, q, 1.5) for p, q in cm.RES["links"].values()]
nodes = trimesh.util.concatenate([trimesh.creation.icosphere(2, radius=10.0 if i < cm.NR else 6.0).apply_translation(N[i]) for i in range(len(N))])
mount = trimesh.load(os.path.join(RUN, "camera_mount_assembled.stl"), force="mesh")
cam = trimesh.creation.box(extents=[14, 38, 38]); cam.apply_translation([142, 0, 42])
lens = trimesh.creation.cylinder(radius=6.0, height=4.0, sections=32); lens.apply_transform(trimesh.geometry.align_vectors([0, 0, 1], [1, 0, 0])); lens.apply_translation([147.0, 0, 42])
bolts = []
for (x, y) in cm.FOOT_BOLTS:                                          # M3 x 12 socket head + washer
    b = trimesh.util.concatenate([trimesh.creation.cylinder(radius=1.5, height=12, sections=16).apply_translation([x, y, cm.PLATE_TOP + cm.PAD[2] - 6.0 + 0.5]),
                                  trimesh.creation.cylinder(radius=2.75, height=3.0, sections=16).apply_translation([x, y, cm.PLATE_TOP + cm.PAD[2] + 2.0]),
                                  trimesh.creation.cylinder(radius=3.5, height=0.5, sections=16).apply_translation([x, y, cm.PLATE_TOP + cm.PAD[2] + 0.25])])
    bolts.append(b)
ctx = []
for f in ("frame_plate__bee35_carbon", "motor_1", "motor_2", "prop_1", "prop_2", "receiver_v3__latch", "xt60e1_m", "tie_saddle__printed"):
    pth = os.path.join(LAY, "parts", f + ".stl")
    if os.path.exists(pth): ctx.append((f, trimesh.load(pth, force="mesh")))
bm = os.path.join(ROOT, "runs", "scout_belly_mount", "v1", "belly_mount__petg_.stl")
if os.path.exists(bm): ctx.append(("belly mount v1 (concept)", trimesh.load(bm, force="mesh")))

chk = json.load(open(os.path.join(RUN, "checks.json")))
fr = chk["frame"]
s = Scene("Scout camera mount v2",
          f"topology, one PETG print {chk['mass_g']:.1f} g (M); core tie to the plate + 2 snap clips on rod 14-15 next to nodes 14 / 15; "
          f"first mode {fr['first_mode_hz']:.0f} Hz (F) on 3 mm rods; hood {chk['view']['hood_lead_mm']:.1f} ahead of the lens")
s.part("camera mount v2 (PETG)", mount, group="mount", color="#c9ced6", printed=True)
s.part("camera (DECXIN 2633V2, envelope)", cam, group="camera", color="#2d5a2d", printed=False)
s.part("lens (r 6, E)", lens, group="camera", color="#101820", printed=False)
for i, b in enumerate(bolts): s.part(f"M3 x 12 + washer {i + 1}", b, group="bolts", color="#d0a040", printed=False)
s.part("3.0 carbon rods (every rod)", trimesh.util.concatenate(rods), group="cage", color="#4a5059", printed=False)
s.part("cage nodes (PETG)", nodes, group="cage", color="#8899aa", printed=False)
for f, m in ctx: s.part(f, m, group="drone", color="#556677", printed=False, faces=4000)

T, PUSH, DROP = 9.0, cm.STROKE, 25.0


def ease(a): return 0.5 - 0.5 * math.cos(math.pi * min(1.0, max(0.0, a)))


def pose(t):
    dx = PUSH * (1 - ease(t / 4.0)); dz = DROP * (1 - ease((t - 4.6) / 2.0))
    out = {k: [dx, 0, 0, 0, 0, 0] for k in ("camera mount v2 (PETG)", "camera (DECXIN 2633V2, envelope)", "lens (r 6, E)")}
    for i in range(len(bolts)): out[f"M3 x 12 + washer {i + 1}"] = [0, 0, dz + (60 if t < 4.5 else 0), 0, 0, 0]
    return out


s.timeline(pose, T, fps=30)
s.view("close-up", eye=(250, 140, 120), target=(110, 0, 30))
s.view("side", eye=(90, 260, 40), target=(90, 0, 30))
s.view("from above", eye=(70, 20, 260), target=(85, 0, 25))
s.write(os.path.join(RUN, "motion.html"))
if "--video" in sys.argv:
    s.video(os.path.join(RUN, "motion.mp4"), view="close-up")
if "--stills" in sys.argv:
    s.timeline(lambda t: pose(T), 1.0, fps=2)
    for v, tag in (("close-up", "closeup"), ("side", "side"), ("from above", "above")):
        s.video(os.path.join(RUN, f"still_{tag}.mp4"), view=v, gif=False, png=True); os.remove(os.path.join(RUN, f"still_{tag}.mp4"))
