"""Connectivity and material distribution of the box system topology (density.npz). python3 box_system_topo_v2_analyse.py"""
import os, sys
import numpy as np
from scipy import ndimage as ndi
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
import box_system_topo_v2 as T
from topo.part import tm, voxelise

OUT = os.path.join(T.ROOT, "runs", "box_system", "v2", "topo")
d = np.load(os.path.join(OUT, "density.npz")); h = float(d["h"]); sh = tuple(d["shape"]); o = np.asarray(d["origin"])
rho = d["rho_n"].reshape(sh); solid = d["solid"].reshape(sh)
ax = [o[k] + (np.arange(sh[k]) + 0.5) * h for k in range(3)]
X, Y, Z = np.meshgrid(*ax, indexing="ij")
EN = T.EN; Xi, Y0, Z0 = EN["X_IN"], EN["Y_BACK"], EN["FLOOR"]
sheet = {"+X": X > Xi - h, "-X": X < -Xi + h, "rear": Y < Y0 + h}
anysheet = sheet["+X"] | sheet["-X"] | sheet["rear"]
floor = Z < Z0 + h
mat_struct = ((rho > 0.5) | solid) & ~anysheet
st = ndi.generate_binary_structure(3, 1)
lab, n = ndi.label(mat_struct, st)
sizes = np.bincount(lab.ravel()); sizes[0] = 0
order = np.argsort(sizes)[::-1]
print("components", n, "largest cm3", [round(sizes[i] * h ** 3 / 1e3, 1) for i in order[:6]], "total struct cm3", round(mat_struct.sum() * h ** 3 / 1e3, 1))
main = int(order[0])
setup, G = T.setup_factory(lambda *a: None, {})
setup(h)
bod = {}
for ang, m in G["pads"].items(): bod[f"pad{ang}({T.L2.BAYS[ang][0].upper()})"] = voxelise(tm(m), *ax)
for i, m in enumerate(G["posts"]): bod[f"post{i}"] = voxelise(tm(m), *ax)
bod["stub"] = voxelise(tm(G["stub"]), *ax)
for k, m in bod.items():
    ids = np.unique(lab[m & (lab > 0)])
    print(k, ids.tolist(), "main" if main in ids else "NOT IN MAIN", "cells", int(m.sum()))
M = lab == main
for nm, m in list(sheet.items()) + [("floor", floor)]:
    near = ndi.binary_dilation(m, st, iterations=1) & M
    print("main body touches", nm, ":", int(near.sum()), "cells (", round(near.sum() * h * h / 100, 1), "cm2 of faces)")
r = np.hypot(X, Z); ang = np.degrees(np.arctan2(Z, X))
vol = lambda m: round((m & mat_struct).sum() * h ** 3 / 1e3, 1)
print("r 110..200:", vol((r > 110) & (r < 200)), " r>=200:", vol(r >= 200), " r<=110:", vol(r <= 110))
print("Y slabs (cm3):", {f"{y0}": vol((Y >= y0) & (Y < y0 + 50)) for y0 in range(-66, 340, 50)})
print("Z slabs (cm3):", {f"{z0}": vol((Z >= z0) & (Z < z0 + 50)) for z0 in range(-240, 240, 50)})
print("base Z<-150:", vol(Z < -150), " Y<74:", vol(Y < 74), " Y<20:", vol(Y < 20))
for a in range(-210, 60, 30):
    sec = np.abs(((ang - a + 180) % 360) - 180) < 15
    print("bay", a, "material in sector r>110:", vol(sec & (r > 110)))
# tracked: material between rear sheet and the ring (Y<74), and between ring and side sheets
print("Y 0..74 (mast zone) material cm3:", vol((Y > 0) & (Y < 74)), " X>200:", vol(np.abs(X) > 180))
