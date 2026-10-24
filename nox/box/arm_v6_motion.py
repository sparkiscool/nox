"""Motion artifact for hub v6 + heads A / B v6 (24 Oct 2026): head A picks at the -90 bay, the arm turns, head A inserts at the drone
seat (to z0 89.6, then the +6 drone insert push to 95.6), retracts; then head B rises under the drone and reaches the seat.
Meshes: hub v6 (W) and the heads v6 (head frame) from the runs; rails / wheels / pinions from the arm-mechanism worker's meshes when
they are in print/, else drawn from arm_v5_layout as context; the bay module v4i (M) at -90 and a copy at the top standing in for the
drone receiver (the receiver is the same drawn part, box-case.md). Poses: arm frame = W at phi 90; W = R_Y(phi - 90) arm; head A at
stroke z0: arm Z = u + z0; head B (A mirrored): arm X = -(u + z0), arm Z = X_B.
python3 arm_v6_motion.py <hub_W.stl> <head_A_W.stl> [--video]  -> runs/head/v6/motion.html (+ mp4 / gif / png)"""
import sys, os, math
import numpy as np, trimesh, manifold3d as mf
import arm_layout as L, arm_cad as A, head_v6 as HD, hub_v6 as HV
from arm_cad import tm, man, box, cylY, mirror_x, rsquare
from motion import Scene

V = HV.V
hub_p, head_p = sys.argv[1], sys.argv[2]
OUT = os.path.join(HV.REPO, "runs", "head", "v6"); os.makedirs(OUT, exist_ok=True)
hub = trimesh.load(hub_p, force="mesh") if os.path.exists(hub_p) else tm(HV.hub_v6_parts()["drawn"] - HV.hub_v6_parts()["cuts"])
hA = trimesh.load(head_p, force="mesh")
hB = hA.copy(); hB.apply_transform(np.diag([-1.0, 1, 1, 1]))
if hB.volume < 0: hB.invert()

def R(phi):
    d = math.radians(phi - 90.0); c, s = math.cos(d), math.sin(d)
    T = np.eye(4); T[0, 0], T[0, 2], T[2, 0], T[2, 2] = c, -s, s, c
    return T
def Tz(z): T = np.eye(4); T[2, 3] = z; return T
TB = np.array([[0, 0, -1, 0], [0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1.0]])
def headT(h, z0, phi): return R(phi) @ (Tz(z0) if h == "A" else TB @ Tz(z0))

s = Scene("Nox swap arm v6: hub v6 + heads A / B v6", "head A picks at the -90 bay and inserts at the drone seat (5 over-travel + the 6 drone push: the stand-in receiver rides up with the drone); head B reaches the seat")
s.part("hub v6", hub, group="arm", color="#8899aa")
s.part("head A v6", hA, group="head A", color="#cc6644")
s.part("head B v6", hB, group="head B", color="#4477cc")
# context: rails (arm frame), pinions, motors, wheels (head frame), cartridge, bay modules
def prism(poly, y0, y1): return HV.prism(poly, y0, y1)
for h in ("A", "B"):
    rp = os.path.join(HV.REPO, "print", f"box_rail_{h}_v1_W.stl")
    if os.path.exists(rp): rail = trimesh.load(rp, force="mesh")
    else:
        m = None
        for nm, poly, y in V.rail_bodies(h):
            p = prism(poly, *y); m = p if m is None else m + p
        rail = tm(m)
    s.part(f"rail {h} (arm-mech part)", rail, group="arm", color="#556677", printed=False, note="arm-mechanism worker's part, drawn from arm_v5_layout here")
    M = HV.MOTORS6[h]
    s.part(f"pinion {h}", tm(cylY(M["xc"], M["zc"], *V.PIN_TEETH_Y, V.PIN_R + 0.5) + cylY(M["xc"], M["zc"], *V.PIN_BOSS_Y, V.PIN_BOSS_R)), group="arm", color="#f5b041", printed=False)
for nm, M in HV.MOTORS6.items():
    s.part(f"NEMA17 {nm}", tm(rsquare(M, L.NEMA / 2, 2.0, 50.0) + cylY(M["xc"], M["zc"], 50.0, 74.0, 2.5)), group="arm", color="#1f2a33", printed=False)
wheels = None
for x, u in V.WHEELS:
    w = cylY(x, u, *V.WHEEL_Y, V.WHEEL_OD / 2); wheels = w if wheels is None else wheels + w
s.part("V-wheels A", tm(wheels), group="head A", color="#aab4be", printed=False)
s.part("V-wheels B", tm(mirror_x(wheels)), group="head B", color="#aab4be", printed=False)
cart = tm(box(-L.CART_W / 2, L.CART_W / 2, L.Y_C - L.Z_TABC, L.Y_C - L.Z_TABC + L.CART_L, L.CART_BOT_DZ, L.CART_BOT_DZ + L.CART_H))
s.part("cartridge (Scout pack)", cart, group="pack", color="#44aa66", printed=False)
bm = A.man(trimesh.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), "models", "box_baymod_v4i_W.obj"), force="mesh"))
s.part("bay module v4i at -90", tm(bm.rotate([0, 90.0 - L.DEMO_BAY, 0])), group="fixed", color="#5b8ac9", printed=False)
s.part("drone receiver (stand-in: the same receiver)", tm(bm), group="fixed", color="#9955bb", printed=False)
s.part("80T sun (fixed)", tm(cylY(0, 0, 54, 64, L.P80_FL / 2)), group="fixed", color="#33aaaa", printed=False)

ZP, ZBAY, ZDR, ZPUSH, ZW = 32.0, L.Z0_BAY + 1.5, L.Z0_DRONE, max(HD.Z0S), L.Z0_B_WAIT
def lerp(a, b, t0, t1, t): return a if t <= t0 else b if t >= t1 else a + (b - a) * 0.5 * (1 - math.cos(math.pi * (t - t0) / (t1 - t0)))
KEY = 20.0
def state(t):
    phi = lerp(-90.0, 90.0, 5.5, 8.0, t) if t < 14 else lerp(90.0, 0.0, 14.0, 16.0, t)
    zA = ZP
    if t < 5.5: zA = lerp(ZP, ZBAY, 1.0, 3.0, t) if t < 3.5 else lerp(ZBAY, ZP, 3.5, 5.5, t)
    elif t < 14: zA = lerp(ZP, ZDR, 8.0, 10.0, t) if t < 10 else (lerp(ZDR, ZPUSH, 10.0, 11.0, t) if t < 11.5 else lerp(ZPUSH, ZP, 11.5, 13.5, t))
    zB = ZP if t < 16 else (lerp(ZP, ZW, 16.0, 17.5, t) if t < 17.5 else (lerp(ZW, ZDR, 17.5, 18.5, t) if t < 18.7 else lerp(ZDR, ZP, 18.7, 19.9, t)))
    return phi, zA, zB
def poses(t):
    phi, zA, zB = state(t)
    out = {"hub v6": R(phi), "rail A (arm-mech part)": R(phi), "rail B (arm-mech part)": R(phi), "pinion A": R(phi), "pinion B": R(phi),
           "NEMA17 A": R(phi), "NEMA17 B": R(phi), "NEMA17 axle": R(phi),
           "head A v6": headT("A", zA, phi), "V-wheels A": headT("A", zA, phi), "head B v6": headT("B", zB, phi), "V-wheels B": headT("B", zB, phi)}
    if t < 3.5: out["cartridge (Scout pack)"] = R(-90.0) @ Tz(ZBAY)                    # seated in the -90 bay
    elif t < 11.5: out["cartridge (Scout pack)"] = headT("A", zA, phi)                 # carried
    else: out["cartridge (Scout pack)"] = R(90.0) @ Tz(ZPUSH)                         # left in the drone seat
    lift = max(0.0, (zA if 8.0 <= t < 11.5 else ZPUSH if 11.5 <= t else ZBAY) - ZBAY)    # the drone rides up on the over-travel and the push
    out["drone receiver (stand-in: the same receiver)"] = Tz(lift)
    return out
s.timeline(poses, KEY, fps=25)
st = [state(i * KEY / 500) for i in range(500)]
s.series("head A z0", [z for _, z, _ in st], unit="mm"); s.series("head B z0", [z for _, _, z in st], unit="mm")
s.view("front (along the axle)", (0, 700, 0), (0, 120, 0))
s.write(os.path.join(OUT, "motion.html"))
if "--video" in sys.argv: s.video(os.path.join(OUT, "motion.mp4"))
print("->", OUT)
