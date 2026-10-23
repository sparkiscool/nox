"""Scan stick assembly visual: every printed part in place on the pole with the bought parts as context (L2, camera board,
Pi 5, 4S 6000, pole), as one motion artifact (slow turntable) + png.   python3 assembly_view.py [out_dir]
Frames: world = head run frame (pole axis Z, lidar seat z = +6). Numbers C from scan_stick.scad / section S unless tagged."""
import os, sys, math
import numpy as np, trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
from motion import Scene

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "runs", "scan_stick", "v2")
ROD_D, HUB_R = 25.4, (25.4 + 0.5) / 2 + 5          # C (rod_d E until measured)
SOCK_H, BOSS_R, PLATE_T = 34.0, 16.0, 6.0            # C
CRADLE_Z = -230.0                                    # C: cradle hub bottom below the lidar seat plane
CAM_Y, CAM_Z = BOSS_R + 5, -SOCK_H + (SOCK_H - 17) - 7   # C: bracket origin (boss holes 17 above the boss bottom, holes at z 7 on the bracket)

def rot(axis, deg, t=(0, 0, 0)):
    M = trimesh.transformations.rotation_matrix(math.radians(deg), axis); M[:3, 3] = t; return M

def load(p, M=None):
    m = trimesh.load(p, force="mesh")
    if M is not None: m.apply_transform(M)
    return m

def box(x0, x1, y0, y1, z0, z1):
    b = trimesh.creation.box([x1 - x0, y1 - y0, z1 - z0]); b.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]); return b

# ---- printed parts (print files -> world)
head = load(os.path.join(ROOT, "runs", "scan_stick", "v2", "scan_stick_head_v2_runframe.stl"))
cam = load(os.path.join(ROOT, "print", "scan_stick_cam_v1.stl"), rot([1, 0, 0], -90, (0, CAM_Y, CAM_Z)))
cradle = load(os.path.join(ROOT, "print", "scan_stick_cradle_v1.stl"), rot([0, 0, 1], 0, (0, 0, CRADLE_Z)))

# ---- bought context (spec / P sizes, section M and O)
pole = trimesh.creation.cylinder(ROD_D / 2, 420, sections=64); pole.apply_translation([0, 0, -7 - 210])
tip = trimesh.creation.cylinder(19.05 / 2, 27, sections=48); tip.apply_translation([0, 0, -34 + 13.5])
lidar = box(-37.5, 37.5, -37.5, 37.5, PLATE_T, PLATE_T + 39)
dome = trimesh.creation.icosphere(3, 35.5); dome.apply_scale([1, 1, 26 / 35.5]); dome.apply_translation([0, 0, PLATE_T + 39])
dome = dome.slice_plane([0, 0, PLATE_T + 39], [0, 0, 1], cap=True)
camera = box(-19, 19, CAM_Y + 1.5 + 3, CAM_Y + 1.5 + 3 + 1.6, CAM_Z - 25 - 19, CAM_Z - 25 + 19)
lens = trimesh.creation.cylinder(6, 8, sections=32); lens.apply_transform(rot([1, 0, 0], -90, (0, CAM_Y + 6.1 + 4, CAM_Z - 25)))
pi = box(HUB_R + 7, HUB_R + 8.6, -28, 28, CRADLE_Z + 4, CRADLE_Z + 89)
pack = box(-HUB_R - 45, -HUB_R, -24, 24, CRADLE_Z + 4, CRADLE_Z + 152)

s = Scene("Scan stick", "Unitree L2 + DECXIN camera + Pi 5 + 4S 6000 on the 3/4-5 ACME paint pole; head v2 (topology), cam v1, cradle v1")
s.part("head v2", head, group="printed", color="#8899aa", printed=True, note="61.9 g M, topology run, solid_check PASS")
s.part("cam bracket v1", cam, group="printed", color="#9aa8b8", printed=True)
s.part("cradle v1", cradle, group="printed", color="#8fa3b5", printed=True)
s.part("Unitree L2", trimesh.util.concatenate([lidar, dome]), group="bought", color="#222222", printed=False, note="75 x 75 x 65, 230 g spec")
s.part("DECXIN camera", trimesh.util.concatenate([camera, lens]), group="bought", color="#2d5a2d", printed=False)
s.part("Raspberry Pi 5", pi, group="bought", color="#2e7d32", printed=False)
s.part("4S 6000 pack", pack, group="bought", color="#2f4fbf", printed=False, note="148 x 48 x 45 P")
s.part("pole", trimesh.util.concatenate([pole, tip]), group="bought", color="#777777", printed=False, note="EXTEND-A-REACH 1.5-3 ft, lower tube OD E")

names = ["head v2", "cam bracket v1", "cradle v1", "Unitree L2", "DECXIN camera", "Raspberry Pi 5", "4S 6000 pack", "pole"]
s.timeline(lambda t: {n: rot([0, 0, 1], 360 * t / 16) for n in names}, duration_s=16, fps=30)
s.view("iso", (520, -620, 120), (0, 0, -110)); s.view("front", (0, -800, -110), (0, 0, -110)); s.view("head", (220, -260, 60), (0, 0, -10))
os.makedirs(OUT, exist_ok=True)
s.write(os.path.join(OUT, "motion.html"))
try:
    s.video(os.path.join(OUT, "motion.mp4"))
except Exception as e:
    print("video skipped:", str(e)[:200])
print("wrote", OUT)
