"""Cut proposal from the density field of one run (TOPO_OUT): each material cell goes to one piece by priority, then envelope,
volume, mass (E, 1.27 g/cm3 PETG), 180 cube check. Rule-based first reading, not a final cut. -> cuts.json + cuts.txt"""
import os, sys, json, math
import numpy as np
from scipy import ndimage as ndi
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import box_system_topo_v2r2 as R2

OUT = os.environ["TOPO_OUT"]
d = np.load(os.path.join(OUT, "density.npz")); h = float(d["h"]); sh = tuple(d["shape"]); o = np.asarray(d["origin"])
mat = ((d["rho_n"].reshape(sh) > 0.5) | d["solid"].reshape(sh))
ax = [o[k] + (np.arange(sh[k]) + 0.5) * h for k in range(3)]
X, Y, Z = np.meshgrid(*ax, indexing="ij"); r = np.hypot(X, Z); ang = np.degrees(np.arctan2(Z, X))
piece = np.full(sh, "", dtype=object)
free = mat.copy()


def take(name, m):
    global free
    m = m & free; piece[m] = name; free = free & ~m


take("Mast + stub face (Y < 8, |X| < 70)", (Y < 8.0) & (np.abs(X) < 70))
take("Rear frame +X (Y < 8, X > 70)", (Y < 8.0) & (X >= 70))
take("Rear frame -X (Y < 8, X < -70)", (Y < 8.0) & (X <= -70))
take("Seat-post bridge (Y > 270)", Y > 270.0)
for a in range(30, -211, -30):
    sec = np.abs(((ang - a + 180) % 360) - 180) < 15
    take(f"Bay body {a} ({R2.L2.BAYS[a][0].upper()})", sec & (r > 110) & (Y > 60))
take("Top frame +X half (Z > 110)", (Z > 110) & (X >= 0))
take("Top frame -X half (Z > 110)", Z > 110)
take("Base + feet (Z < -150)", Z < -150)
take("Hub links (rest, r < 110 or between)", free)
rows = []
for nm in sorted(set(piece[mat].tolist())):
    m = (piece == nm)
    i = np.argwhere(m)
    lo = np.array([X[tuple(i.T)].min(), Y[tuple(i.T)].min(), Z[tuple(i.T)].min()]) - h / 2
    hi = np.array([X[tuple(i.T)].max(), Y[tuple(i.T)].max(), Z[tuple(i.T)].max()]) + h / 2
    ext = hi - lo; lab, n = ndi.label(m, ndi.generate_binary_structure(3, 2))
    v = m.sum() * h ** 3 / 1e3
    rows.append(dict(piece=nm, vol_cm3=round(float(v), 1), mass_g_E=round(float(v) * 1.27), x=[round(float(lo[0])), round(float(hi[0]))],
                     y=[round(float(lo[1])), round(float(hi[1]))], z=[round(float(lo[2])), round(float(hi[2]))],
                     longest_mm=round(float(ext.max())), over_180=bool(ext.max() > 180), islands=int(n)))
json.dump(rows, open(os.path.join(OUT, "cuts.json"), "w"), indent=1)
with open(os.path.join(OUT, "cuts.txt"), "w") as f:
    for q in rows:
        f.write(f"{q['piece']}: {q['vol_cm3']} cm3 = {q['mass_g_E']} g (E); X {q['x']} Y {q['y']} Z {q['z']}; longest {q['longest_mm']}"
                f"{' OVER 180' if q['over_180'] else ''}; {q['islands']} islands\n")
print(open(os.path.join(OUT, "cuts.txt")).read())
