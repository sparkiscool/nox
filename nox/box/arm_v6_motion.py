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
          + f"rails on the motor plate, 3-wheel carriages, 16T rack and pinion; {hub_nm}; latch v1 cartridges"
          + (f"; Y_C {L.Y_C:g}, low carriage stack, rails v4 (keel), orange = the insert-lane keep-out at the drone" if STK == "v7" else ""))
for nm, p in P.items():
    if STK == "v6" and nm.startswith(("bay", "strip")) and nm not in (f"bay{K.demo_idx()}",) and KIND == "layout": continue      # layout: the demo bay only
    s.part(nm, p.mesh, group=GROUPS.get(p.group, p.group), color=p.color, printed=p.printed, note=p.name)
cart = K.tm(K.CART_LOCAL(4.0))
for cn in ("pack charged", "pack dead"):
    s.part(cn, cart, group="pack", color="#44aa66" if cn == "pack charged" else "#aa5544", printed=False, note="latch v1 cartridge + Scout pack")
FAR = K.trans(0, 0, -5000.0)
KEEPS = {}
if STK == "v7":                                  # the insert lane at the drone (phi 90, head A along +Z): what the catch's funnel notch and the
    def _box(x0, x1, y0, y1, z0, z1):            # structure keep clear (dimensions "System box swap: arm", Insert-lane keep-out)
        return trimesh.creation.box(extents=(x1 - x0, y1 - y0, z1 - z0), transform=K.trans((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
    KEEPS["keep: carriage lane"] = _box(-35.0, 47.5, 53.75, 73.75, 100.0, 159.0)
    KEEPS["keep: head and grabber"] = _box(-47.5, 47.5, 73.75, L.Y_C + 52.0, 111.5, 159.6)
    for nm, m in KEEPS.items():
        s.part(nm, m, group="keep-out (insert lane)", color="#e67e22", printed=False, note="keep clear: catch funnel notch, structure")


def poses(t):
    T, carts, st = K.transforms(t)
    out = {nm: T[nm] for nm in P if nm in T}
    for cn in ("pack charged", "pack dead"):
        out[cn] = carts[cn][0] if cn in carts else (K.trans(0, 0, L.R + 5.0) if cn == "pack dead" else FAR)
    for nm in KEEPS: out[nm] = np.eye(4)
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
