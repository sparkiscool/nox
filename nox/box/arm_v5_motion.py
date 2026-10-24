"""Arm v5 motion artifact (24 Oct 2026, arm-mech): one swap cycle at the -90 deg bay with the v5 drawn parts (rails A / B v1, V-wheels,
16T pinions rolling on the carriage racks, eccentric bushes, the detent at CHOSEN_V5), the drone insert +6 and the slow last 12 of each
insert. Parts and poses come from arm_kinematics (mode v5): the same states the sweep checks.
python3 arm_v5_motion.py [--video]  -> runs/arm/v5/motion.html (+ .mp4 / .gif / .png)"""
import os, sys, json
import numpy as np, trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
os.chdir(HERE)
import arm_kinematics as K
from motion import Scene

OUT = os.path.join(HERE, "..", "..", "runs", "arm", "v5")
K.KEYS = K.KEYS_V5
P = K.build_parts("v5")
packs = {"pack charged": K.tm(K.cartridge_local(0.0)), "pack dead": K.tm(K.cartridge_local(0.0))}
GROUPS = {"fixed": "fixed", "arm": "arm", "armB": "arm", "headA": "head A", "headB": "head B"}
s = Scene("Box swap arm v5: V-rails, V-wheels, rack and pinion", "one swap cycle at the -90 deg bay; drone insert +6, last 12 of each insert at 20 mm/s (E); "
          "hub = drawn v6 proxy, heads = drawn carriages + v4 gripper")
for nm, p in P.items():
    grp = "bay modules" if nm.startswith(("bay", "strip")) else GROUPS.get(p.group, p.group)
    s.part(nm, p.mesh, group=grp, color=p.color, printed=p.printed, faces=12000 if nm.startswith("bay") else 20000, note=p.name)
for nm, m in packs.items():
    s.part(nm, m, group="packs", color="#27ae60" if "charged" in nm else "#7f8c8d", printed=False, note="Scout cartridge v3 (solid)")
FAR = np.diag([1e-4, 1e-4, 1e-4, 1.0])                          # hidden pack: shrunk to a point on the axle (a far pose spoils the camera framing)
zA, zB = [], []


def poses(t):
    T, carts, st = K.transforms(t)
    out = {nm: T.get(nm, np.eye(4)) for nm in P}
    for nm in packs:
        out[nm] = carts[nm][0] if nm in carts else FAR
    zA.append(st["zA"]); zB.append(st["zB"])
    return out


s.timeline(poses, duration_s=K.T_END, fps=15)
s.series("head A z0", zA, unit="mm"); s.series("head B z0", zB, unit="mm")
s.view("rails", eye=(260, 330, 220), target=(0, 80, 0))
s.view("drone insert", eye=(160, 160, 260), target=(0, 120, 110))
os.makedirs(OUT, exist_ok=True)
s.write(os.path.join(OUT, "motion.html"))
if "--video" in sys.argv:
    s.video(os.path.join(OUT, "motion.mp4"))
