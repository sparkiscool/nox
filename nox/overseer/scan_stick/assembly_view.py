"""Scan stick assembly visual: every printed part in place on the pole with the bought parts as context (L2, camera board,
Pi 5, 4S 6000, pole), as one motion artifact (slow turntable) + mp4 / gif / png.   python3 assembly_view.py [out_dir]
World = the base's print frame: pole axis Z, pole tip boss bottom at z = 0 (the thread), the joint face at z = 168,
head on it (head print frame z 0 = world 168); the L2 forward (+X) 15° down and the camera below it (unit_topo_v1.py). Numbers C from the run scripts / section S."""
import os, sys, math
import numpy as np, trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
from motion import Scene

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "runs", "scan_stick", "assembly")
TOP = 168.0                                              # the joint face (unit_topo_v1)
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

import importlib.util
UNIT = os.environ.get("SCAN_UNIT", "v3")                                                     # which unit_topo_v*.py layout to show
_spec = importlib.util.spec_from_file_location("unit", os.path.join(HERE, f"unit_topo_{UNIT}.py"))
U1 = importlib.util.module_from_spec(_spec); _argv = sys.argv; sys.argv = [sys.argv[0]]; _spec.loader.exec_module(U1); sys.argv = _argv
def piece(name):                                                                           # the saved print file, else the run's split piece (not saved yet)
    p = os.path.join(ROOT, "print", name + ".stl")
    return p if os.path.exists(p) else os.path.join(U1.OUT, name + ".stl")
SAVED = os.path.exists(os.path.join(ROOT, "print", "scan_stick_head_v14.stl"))
base = load(piece("scan_stick_base_v19"))
head = load(piece("scan_stick_head_v14"), rot([0, 0, 1], 0, (0, 0, TOP)))                  # printed flange down: head z 0 = world 168
ring = load(os.path.join(ROOT, "print", "scan_stick_thread_test_v1.stl"), rot([0, 0, 1], 0, (-95, 20, -60)))

pole = trimesh.creation.cylinder(12.7, 420, sections=64); pole.apply_translation([0, 0, -6 - 210])
tip = trimesh.creation.cylinder(19.05 / 2, 27, sections=48); tip.apply_translation([0, 0, 13.5])
# sensor frame (unit_topo_v1): x' = f (the L2 axis, forward 15° down), z' = u; L2 base face at U1.O
SENS = trimesh.transformations.rotation_matrix(math.radians(U1.TILT), [0, 1, 0]); SENS[:3, 3] = U1.O
def sens(m):
    m.apply_transform(SENS); return m
lidar = box(0.0, 39.0, -37.5, 37.5, -37.5, 37.5)
dome = trimesh.creation.icosphere(3, 35.5); dome.apply_scale([26 / 35.5, 1, 1]); dome.apply_translation([39.0, 0, 0])
dome = dome.slice_plane([39.0, 0, 0], [1, 0, 0], cap=True)
lidar = sens(trimesh.util.concatenate([lidar, dome]))
if hasattr(U1, "CAM_XF"):                                                                  # v2+: level camera, world frame
    camera = box(U1.CAM_XF - 1.6, U1.CAM_XF, -19, 19, U1.CAM_Z - 19, U1.CAM_Z + 19)          # board on the post tops, 1.6 thick (E)
    lens = trimesh.creation.cylinder(6, U1.LENS, sections=32); lens.apply_transform(rot([0, 1, 0], 90, (U1.CAM_XF + U1.LENS / 2, 0, U1.CAM_Z)))   # Ø12 x 16 (E, Q3)
else:                                                                                      # v1: tilted with the L2
    camera = sens(box(U1.CAM_F - 1.6, U1.CAM_F, -19, 19, U1.CAM_U - 19, U1.CAM_U + 19))
    lens = trimesh.creation.cylinder(6, 16, sections=32); lens.apply_transform(rot([0, 1, 0], 90, (U1.CAM_F + 8, 0, U1.CAM_U))); sens(lens)
fov = sens(box(0.0, 120.0, -150, 150, -150, 150))                                            # the L2's 90° hemisphere near the unit
pi = box(PI_X0 + PI_SO, PI_X0 + PI_SO + 1.6, -28, 28, PI_Z0, PI_Z0 + 85)
pack = box(BAT_X1 - 45.5, BAT_X1 - 0.5, -24, 24, BAT_Z0 + 0.5, BAT_Z0 + 148.5)            # in its keep-out, 0.5 off the back plate and the ledge

def overlap(a, b):
    """overlap volume (mm3, M) of two meshes, an exact manifold boolean (the engine's own)"""
    from topo.part import man
    return (man(a) ^ man(b)).volume()
for an, am, bn, bm in (("base", base, "head", head), ("base", base, "Unitree L2", lidar), ("head", head, "Unitree L2", lidar),
                       ("base", base, "camera board", camera), ("head", head, "camera board", camera), ("head", head, "camera lens", lens),
                       ("base", base, "Pi", pi), ("base", base, "pack", pack), ("head", head, "Pi", pi)):
    print(f"fit: {an} x {bn}: {overlap(am, bm):.0f} mm3 overlap (M; 0 = clear)")
print(f"fov: head in the L2 hemisphere: {overlap(head, fov):.0f} mm3 (M; 0 = the head is out of the lidar's view)")
s = Scene("Scan stick", f"one bolted unit on the 3/4-5 ACME pole tip, optimised as one (unit {UNIT}{'' if SAVED else ', NOT READY: failed checks, see the run summary'}): base v19 (Pi 5 + 4S 6000 near the hand), head v14 (Unitree L2 forward 15° down + DECXIN camera below it); nox/topo")
s.part("base v19", base, group="printed", color="#8899aa", printed=True)
s.part("head v14", head, group="printed", color="#9aa8b8", printed=True, note="L2 ring + camera mount, 4x M3 to the base")
s.part("thread test v1", ring, group="printed", color="#b8c0c8", printed=True, note="fit ring, beside the pole for scale")
s.part("Unitree L2", lidar, group="bought", color="#222222", printed=False, note="75 x 75 x 65, 230 g spec")
s.part("DECXIN camera", trimesh.util.concatenate([camera, lens]), group="bought", color="#2d5a2d", printed=False)
s.part("Raspberry Pi 5", pi, group="bought", color="#2e7d32", printed=False)
s.part("4S 6000 pack", pack, group="bought", color="#2f4fbf", printed=False, note="148 x 48 x 45 P")
s.part("pole", trimesh.util.concatenate([pole, tip]), group="bought", color="#777777", printed=False, note="EXTEND-A-REACH 1.5-3 ft")
names = ["base v19", "head v14", "thread test v1", "Unitree L2", "DECXIN camera", "Raspberry Pi 5", "4S 6000 pack", "pole"]
s.timeline(lambda t: {n: rot([0, 0, 1], 360 * t / 16) for n in names}, duration_s=16, fps=30)
s.view("iso", (520, -620, 260), (0, 0, 80)); s.view("front", (0, -800, 80), (0, 0, 80)); s.view("head", (300, -300, 300), (10, 0, 230))
os.makedirs(OUT, exist_ok=True)
s.write(os.path.join(OUT, "motion.html"))
try:
    s.video(os.path.join(OUT, "motion.mp4"))
except Exception as e:
    print("video skipped:", str(e)[:200])
print("wrote", OUT)
