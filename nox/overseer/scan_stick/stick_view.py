"""Scan stick as generated so far: the topology pieces (head, base) from their runs, every other part from the v5 system
model (bought parts, cage rods, TPU nodes, pole), as one motion artifact.   python3 stick_view.py [head_run] [base_run] [out]"""
import os, sys, math
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(ROOT, "nox"))
_argv = sys.argv; sys.argv = [sys.argv[0]]
import unit_topo_v5 as U
sys.argv = _argv
import trimesh as T
from motion import Scene

HEAD = sys.argv[1] if len(sys.argv) > 1 else "runs/scan_stick_head/v17"
BASE = sys.argv[2] if len(sys.argv) > 2 else "runs/scan_stick_base/v23"
OUT = sys.argv[3] if len(sys.argv) > 3 else os.path.join(ROOT, "runs", "scan_stick_system", "v3")
os.makedirs(OUT, exist_ok=True)
def bx(b):
    m = T.creation.box([b[1] - b[0], b[3] - b[2], b[5] - b[4]]); m.apply_translation([(b[0] + b[1]) / 2, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2]); return m
head = T.load(os.path.join(ROOT, HEAD, "scan_stick_head_view.stl"))
base = T.load(os.path.join(ROOT, BASE, "scan_stick_base_view.stl"))
l2 = T.load(os.path.join(U.OUT_SYS, "l2.stl"))
rods = T.util.concatenate([T.creation.cylinder(1.5, segment=[p0, p1], sections=12) for _, p0, p1 in U.cage_rods()])
nodes = T.util.concatenate([bx((x - 6, x + 6, sy * U.YC - 6, sy * U.YC + 6, z - 6, z + 6)) for sy in (-1, 1)
                            for x, z in ((U.X_BACK, U.Z_LOW), (U.X_BACK, U.Z_TOP), (U.frame_x(U.Z_LOW), U.Z_LOW), (U.frame_x(U.Z_TOP), U.Z_TOP))])
pole = T.creation.cylinder(13.75, 300, sections=48); pole.apply_translation([0, 0, 2.0 - 150])          # Ø27.5 cylinder, shoulder at z 2
tip = T.creation.cylinder(19.05 / 2, 25.0, sections=48); tip.apply_translation([0, 0, 2.0 + 12.5])     # 25 thread to z 27
pole = T.util.concatenate([pole, tip])
hv, bv = os.path.basename(HEAD), os.path.basename(BASE)
s = Scene("Scan stick assembly", f"Generated so far: head {hv} and base {bv} are nox/topo runs (organic); L2, camera, Pi 5, UBEC / buck, pack, "
          "3 mm carbon cage, TPU nodes and the pole are the v5 system models. Base socket: Ø28.3 sleeve 27 over the Ø27.5 pole, then the thread.")
s.part(f"base {bv} (generated)", base, group="printed", color="#8899aa", printed=True, note=f"{BASE}: spine, pack pocket, thread boss with the 27 sleeve down the pole")
s.part(f"head {hv} (generated)", head, group="printed", color="#c0cad6", printed=True, note=f"{HEAD}: front module, L2 ring + bosses, camera posts")
sides = U.tm(sum((U.box(U.PACK[0] - 0.5, U.PACK[1] + 0.5, *sorted((sy * (U.PACK[3] + 0.5), sy * (U.PACK[3] + 6.5))), 32.0, U.SPINE[5]) for sy in (-1, 1)), U.mf.Manifold())
              - U.spine_bodies()["cuts"])
s.part("side walls (new in base v26)", sides, group="printed", color="#e0592a", printed=True, note="drawn 6 mm walls y ±26..32 close the pocket into a tube; in the v26 run now, not in the v23 mesh shown")
s.part("cage rods", rods, group="bought", color="#111111", printed=False, note="3 mm carbon")
s.part("TPU nodes", nodes, group="printed", color="#d4a017", printed=False, note="drawn 12 mm nodes (model)")
for n, b, col in (("pack", U.PACK, "#2f4fbf"), ("Pi 5", U.PI_KEEP, "#2e7d32"), ("UBEC + buck", U.EL_KEEP, "#b07d2b"),
                  ("camera", (U.CAM_PLATE[1], U.CAM_XF + U.LENS, -19, 19, U.CAM_Z - 19, U.CAM_Z + 19), "#2d5a2d")):
    s.part(n, bx(b), group="bought", color=col, printed=False)
s.part("Unitree L2", l2, group="bought", color="#222222", printed=False)
s.part("pole", pole, group="bought", color="#777777", printed=False)
names = [p for p in (f"base {bv} (generated)", f"head {hv} (generated)", "side walls (new in base v26)", "cage rods", "TPU nodes", "pack", "Pi 5", "UBEC + buck", "camera", "Unitree L2", "pole")]
s.timeline(lambda t: {n: T.transformations.rotation_matrix(2 * math.pi * t / 16, [0, 0, 1]) for n in names}, duration_s=16, fps=30)
s.view("iso", (420, -480, 300), (20, 0, 90)); s.view("side", (0, -700, 90), (20, 0, 90)); s.view("front", (700, 0, 90), (20, 0, 90))
s.view("back", (-700, 0, 90), (20, 0, 90))
s.write(os.path.join(OUT, "motion.html"))
print(os.path.join(OUT, "motion.html"))
