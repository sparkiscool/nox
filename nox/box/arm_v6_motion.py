"""Motion / layout artifact for the swap arm v6 system (24 Oct 2026, swap arm system worker): one full swap cycle (arm_kinematics v6
keys, 22 s) with the exact parts and poses the sweep checks (build_parts_v6 + transforms): hub v6 (topo mesh when runs/hub/v6 has it,
else the drawn bodies), rails / pinions v2, carriages (drawn proxies until heads v6), V-wheels, bushes, v4 grippers and servos, the
detent (ring v3, arm v2, springs), motors, the axle loop, the latch v1 cartridges, the demo bay as the latch receiver and the other
eight v4i bay modules. Readouts: head A / B z0 and phi.
python3 arm_v6_motion.py [layout|motion] [--video] -> runs/arm/v6/layout.html (concept check) or runs/arm/v6/motion.html (+ mp4 / gif / png)"""
import sys, os, math
import numpy as np, trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
import arm_kinematics as K, arm_layout as L
from motion import Scene

KIND = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else "motion"
OUT = os.path.normpath(os.path.join(HERE, "..", "..", "runs", "arm", "v6")); os.makedirs(OUT, exist_ok=True)
K.KEYS = K._keys_v6()
P = K.build_parts("v6")
GROUPS = dict(arm="arm", armB="arm", headA="head A", headB="head B", fixed="fixed")

hub_nm = P["hub"].name
s = Scene("Nox swap arm v6" + (" layout (concept check)" if KIND == "layout" else ""),
          f"one swap cycle at the -90 bay: rails on the motor plate, 3-wheel carriages, 16T rack and pinion; {hub_nm}; latch v1 cartridges")
for nm, p in P.items():
    if nm.startswith(("bay", "strip")) and nm not in (f"bay{K.demo_idx()}",) and KIND == "layout": continue      # layout: the demo bay only
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
path = os.path.join(OUT, f"{KIND}.html")
s.write(path)
if "--video" in sys.argv: s.video(os.path.join(OUT, f"{KIND}.mp4"))
print("->", path)
