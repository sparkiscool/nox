"""Scan stick assembly visual: every printed part in place on the pole with the bought parts as context (L2, camera board,
Pi 5, 4S 6000, pole), as one motion artifact (slow turntable) + mp4 / gif / png.   python3 assembly_view.py [out_dir]
World = the base's print frame: pole axis Z, pole tip boss bottom at z = 0 (the thread), base flange top at z = 168,
head flange on it (head frame z 0 = world 168), lidar seat at world 202. Numbers C from the run scripts / section S."""
import os, sys, math
import numpy as np, trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
from motion import Scene

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "runs", "scan_stick", "v4")
TOP, SEAT = 168.0, 34.0                                   # base flange top; head seat above the head flange
CAM_Y, CAM_Z = 21.0, 10.0                                 # bracket origin in the head frame (holes 20 apart at z 17, bracket holes at z 7)
PI_X0, PI_SO, PI_Z0 = 20.0, 7.0, 10.0
BAT_X1, BAT_Z0 = -20.0, 10.0

def rot(axis, deg, t=(0, 0, 0)):
    M = trimesh.transformations.rotation_matrix(math.radians(deg), axis); M[:3, 3] = t; return M
def load(p, M=None):
    m = trimesh.load(p, force="mesh")
    if M is not None: m.apply_transform(M)
    return m
def box(x0, x1, y0, y1, z0, z1):
    b = trimesh.creation.box([x1 - x0, y1 - y0, z1 - z0]); b.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]); return b

base = load(os.path.join(ROOT, "print", "scan_stick_base_v2.stl"))
head = load(os.path.join(ROOT, "print", "scan_stick_head_v4.stl"), rot([0, 0, 1], 0, (0, 0, TOP)))
# cam print frame: plate in XY, tab at +Y, lens +z. Mount: plate against the block's +Y face (world: plate in XZ, lens +Y),
# tab holes at world z TOP+17, the plate hanging down.  print (x, y, z) -> world (x, z + CAM_Y, TOP + 17 - (y - 53))
cam = load(os.path.join(ROOT, "print", "scan_stick_cam_v2.stl"), rot([1, 0, 0], 90, (0, CAM_Y, TOP + 17 + 53)))
ring = load(os.path.join(ROOT, "print", "scan_stick_thread_test_v1.stl"), rot([0, 0, 1], 0, (-95, 20, -60)))

pole = trimesh.creation.cylinder(12.7, 420, sections=64); pole.apply_translation([0, 0, -6 - 210])
tip = trimesh.creation.cylinder(19.05 / 2, 27, sections=48); tip.apply_translation([0, 0, 13.5])
lidar = box(-37.5, 37.5, -37.5, 37.5, TOP + SEAT, TOP + SEAT + 39)
dome = trimesh.creation.icosphere(3, 35.5); dome.apply_scale([1, 1, 26 / 35.5]); dome.apply_translation([0, 0, TOP + SEAT + 39])
dome = dome.slice_plane([0, 0, TOP + SEAT + 39], [0, 0, 1], cap=True)
cam_cz = TOP + 17 - 25
camera = box(-19, 19, CAM_Y + 6 + 0.1, CAM_Y + 6 + 1.7, cam_cz - 19, cam_cz + 19)
lens = trimesh.creation.cylinder(6, 8, sections=32); lens.apply_transform(rot([1, 0, 0], -90, (0, CAM_Y + 7.8 + 4, cam_cz)))
pi = box(PI_X0 + PI_SO, PI_X0 + PI_SO + 1.6, -28, 28, PI_Z0, PI_Z0 + 85)
pack = box(BAT_X1 - 45, BAT_X1, -24, 24, BAT_Z0, BAT_Z0 + 148)

s = Scene("Scan stick", "one bolted unit on the 3/4-5 ACME pole tip: base v2 (Pi 5 + 4S 6000 near the hand), head v4 (Unitree L2 + DECXIN camera), nox/topo parts")
s.part("base v2", base, group="printed", color="#8899aa", printed=True)
s.part("head v4", head, group="printed", color="#9aa8b8", printed=True)
s.part("cam bracket v2", cam, group="printed", color="#aab4c0", printed=True)
s.part("thread test v1", ring, group="printed", color="#b8c0c8", printed=True, note="fit ring, beside the pole for scale")
s.part("Unitree L2", trimesh.util.concatenate([lidar, dome]), group="bought", color="#222222", printed=False, note="75 x 75 x 65, 230 g spec")
s.part("DECXIN camera", trimesh.util.concatenate([camera, lens]), group="bought", color="#2d5a2d", printed=False)
s.part("Raspberry Pi 5", pi, group="bought", color="#2e7d32", printed=False)
s.part("4S 6000 pack", pack, group="bought", color="#2f4fbf", printed=False, note="148 x 48 x 45 P")
s.part("pole", trimesh.util.concatenate([pole, tip]), group="bought", color="#777777", printed=False, note="EXTEND-A-REACH 1.5-3 ft")
names = ["base v2", "head v4", "cam bracket v2", "thread test v1", "Unitree L2", "DECXIN camera", "Raspberry Pi 5", "4S 6000 pack", "pole"]
s.timeline(lambda t: {n: rot([0, 0, 1], 360 * t / 16) for n in names}, duration_s=16, fps=30)
s.view("iso", (520, -620, 260), (0, 0, 80)); s.view("front", (0, -800, 80), (0, 0, 80)); s.view("head", (220, -260, 260), (0, 0, 200))
os.makedirs(OUT, exist_ok=True)
s.write(os.path.join(OUT, "motion.html"))
try:
    s.video(os.path.join(OUT, "motion.mp4"))
except Exception as e:
    print("video skipped:", str(e)[:200])
print("wrote", OUT)
