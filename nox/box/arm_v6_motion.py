"""Motion / layout artifact for the swap arm v6 system (24 Oct 2026, swap arm system worker): one full swap cycle (arm_kinematics v6
keys, 22 s) with the exact parts and poses the sweep checks (build_parts_v6 + transforms): hub v6 (topo mesh when runs/hub/v6 has it,
else the drawn bodies), rails / pinions v2, carriages (drawn proxies until heads v6), V-wheels, bushes, v4 grippers and servos, the
detent (ring v3, arm v2, springs), motors, the axle loop, the latch v1 cartridges, the demo bay as the latch receiver and the other
eight v4i bay modules. Readouts: head A / B z0 and phi.
python3 arm_v6_motion.py [layout|motion] [v6 | v7:<pack>:<pick>:<ret>] [--video] -> runs/arm/<v6|v7>/<layout|motion>[_<pack>].html (+ mp4 / gif / png)"""
import sys, os, math
import numpy as np, trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
import arm_kinematics as K, arm_layout as L
from motion import Scene

args = [a for a in sys.argv[1:] if not a.startswith("--")]
KIND = args[0] if args else "motion"
MODE = args[1] if len(args) > 1 else "v6"
STK = "v7" if MODE.startswith("v7") else "v6"
OUT = os.path.normpath(os.path.join(HERE, "..", "..", "runs", "arm", STK)); os.makedirs(OUT, exist_ok=True)
if STK == "v7":
    import arm_v7_layout as S7; S7.use()
    _, PK, PI, RE = MODE.split(":")
    P = K.build_parts(MODE); K.ZP = L.Z0_A_HOME; K.KEYS = K.keys_v7(float(PI), float(RE))
else:
    PK = "scout"; K.KEYS = K._keys_v6(); P = K.build_parts("v6")
GROUPS = dict(arm="arm", armB="arm", headA="head A", headB="head B", fixed="fixed")

hub_nm = P["hub"].name
s = Scene(f"Nox swap arm {STK}" + (" layout (concept check)" if KIND == "layout" else "") + (f", {PK}" if STK == "v7" else ""),
          (f"one {PK} swap cycle (pick {PI}, return {RE}) at R 113 with every bay's receiver and seated pack; " if STK == "v7" else "one swap cycle at the -90 bay: ")
          + f"rails on the motor plate, 3-wheel carriages, 16T rack and pinion; {hub_nm}; latch v1 cartridges")
for nm, p in P.items():
    if STK == "v6" and nm.startswith(("bay", "strip")) and nm not in (f"bay{K.demo_idx()}",) and KIND == "layout": continue      # layout: the demo bay only
    s.part(nm, p.mesh, group=GROUPS.get(p.group, p.group), color=p.color, printed=p.printed, note=p.name)
cart = K.tm(K.CART_LOCAL(4.0))
for cn in ("pack charged", "pack dead"):
    s.part(cn, cart, group="pack", color="#44aa66" if cn == "pack charged" else "#aa5544", printed=False, note="latch v1 cartridge + Scout pack")
FAR = K.trans(0, 0, -5000.0)


def poses(t):
    T, carts, st = K.transforms(t)
    out = {nm: T[nm] for nm in P if nm in T}
    for cn in ("pack charged", "pack dead"):
        out[cn] = carts[cn][0] if cn in carts else (K.trans(0, 0, L.R + 5.0) if cn == "pack dead" else FAR)
    return out


DUR = K.T_END
s.timeline(poses, DUR, fps=20)
n = int(DUR * 20)
st = [K.state(i * DUR / max(n - 1, 1)) for i in range(n)]
s.series("head A z0", [x["zA"] for x in st], unit="mm"); s.series("head B z0", [x["zB"] for x in st], unit="mm")
s.series("phi (head A)", [x["phi"] for x in st], unit="deg")
s.view("front (along the axle)", (0, -650, 0), (0, 120, 0))
path = os.path.join(OUT, f"{KIND}.html" if STK == "v6" else f"{KIND}_{PK}.html")
s.write(path)
if "--video" in sys.argv: s.video(path[:-5] + ".mp4")
print("->", path)
