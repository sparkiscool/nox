"""Where the generated structure meets the sheets and the floor, and the density along each pad's path to the stub.
TOPO_OUT=<run dir> python3 box_system_topo_v2_contacts.py"""
import os, sys
import numpy as np
from scipy import ndimage as ndi
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
import box_system_topo_v2 as T

OUT = os.environ.get("TOPO_OUT")
d = np.load(os.path.join(OUT, "density.npz")); h = float(d["h"]); sh = tuple(d["shape"]); o = np.asarray(d["origin"])
rho = d["rho_n"].reshape(sh); solid = d["solid"].reshape(sh)
ax = [o[k] + (np.arange(sh[k]) + 0.5) * h for k in range(3)]
X, Y, Z = np.meshgrid(*ax, indexing="ij")
EN = T.EN; Xi, Y0, Z0 = EN["X_IN"], EN["Y_BACK"], EN["FLOOR"]
sheet = {"+X": X > Xi - h, "-X": X < -Xi + h, "rear": Y < Y0 + h}
anys = sheet["+X"] | sheet["-X"] | sheet["rear"]
struct = (rho > 0.5) & ~anys
st = ndi.generate_binary_structure(3, 1)
for nm, m in list(sheet.items()) + [("floor", Z < Z0 + h)]:
    c = ndi.binary_dilation(m, st) & struct & ~m
    lab, n = ndi.label(c, ndi.generate_binary_structure(3, 2))
    print(f"== contact with {nm}: {int(c.sum())} cells = {c.sum() * h * h / 100:.0f} cm2, {n} patches")
    for k in range(1, n + 1):
        idx = np.argwhere(lab == k)
        if len(idx) < 3: continue
        lo = np.array([X[tuple(idx.T)].min(), Y[tuple(idx.T)].min(), Z[tuple(idx.T)].min()]) - h / 2
        hi = np.array([X[tuple(idx.T)].max(), Y[tuple(idx.T)].max(), Z[tuple(idx.T)].max()]) + h / 2
        print(f"   patch {k}: {len(idx)} cells, X {lo[0]:.0f}..{hi[0]:.0f} Y {lo[1]:.0f}..{hi[1]:.0f} Z {lo[2]:.0f}..{hi[2]:.0f}")
# structure box by region
r = np.hypot(X, Z)
for nm, m in [("mast zone Y<0", Y < 0), ("hub zone 0<Y<74 r>110", (Y >= 0) & (Y < 74) & (r > 110)),
              ("top frame Z>110", Z > 110), ("bottom Z<-150", Z < -150)]:
    mm = struct & m
    if mm.any():
        i = np.argwhere(mm)
        print(f"{nm}: {mm.sum() * h ** 3 / 1e3:.0f} cm3, X {X[tuple(i.T)].min():.0f}..{X[tuple(i.T)].max():.0f} Y {Y[tuple(i.T)].min():.0f}..{Y[tuple(i.T)].max():.0f} Z {Z[tuple(i.T)].min():.0f}..{Z[tuple(i.T)].max():.0f}")
# radial band per bay angle: density of material in the 'link' between neighbouring bays (r 150..200, angle between pads)
ang = np.degrees(np.arctan2(Z, X))
print("ring band r 150..200, material per 15 deg wedge (cm3) [angle: vol]:")
out = {}
for a in range(-225, 75, 15):
    sec = np.abs(((ang - a + 180) % 360) - 180) < 7.5
    out[a] = round(float((struct & sec & (r > 150) & (r < 200) & (Y > 60) & (Y < 270)).sum() * h ** 3 / 1e3), 1)
print(out)
