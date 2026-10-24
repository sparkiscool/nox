"""Cross-section area profiles of the main members of the topology (where the density is low = candidate joints).
TOPO_OUT=<run dir> python3 box_system_topo_v2_profiles.py"""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
import box_system_topo_v2 as T

OUT = os.environ.get("TOPO_OUT")
d = np.load(os.path.join(OUT, "density.npz")); h = float(d["h"]); sh = tuple(d["shape"]); o = np.asarray(d["origin"])
rho = d["rho_n"].reshape(sh)
ax = [o[k] + (np.arange(sh[k]) + 0.5) * h for k in range(3)]
X, Y, Z = np.meshgrid(*ax, indexing="ij")
EN = T.EN; Xi, Y0 = EN["X_IN"], EN["Y_BACK"]
struct = (rho > 0.5) & (np.abs(X) < Xi - h) & (Y > Y0 + h)


def prof(mask, axis, step=16):
    c = [X, Y, Z][axis]; out = {}
    for a in np.arange(np.floor(c[mask].min() / step) * step, c[mask].max(), step):
        m = mask & (c >= a) & (c < a + step)
        out[int(a)] = round(float(m.sum() * h ** 3 / (step * 100)), 1)         # mean area cm2 over the slab
    return out


r = np.hypot(X, Z)
print("mast (Y<0): area cm2 vs Z", prof(struct & (Y < 0), 2))
print("mast (Y<0): area cm2 vs X", prof(struct & (Y < 0), 0))
print("top frame front beam (Y>285, Z>90): area vs X", prof(struct & (Y > 285) & (Z > 90), 0))
print("top frame side run X>60 (Z>110): area vs Y", prof(struct & (X > 60) & (Z > 110), 1))
print("top frame side run X<-60 (Z>110): area vs Y", prof(struct & (X < -60) & (Z > 110), 1))
print("top frame, area vs X (Z>110, Y 60..290)", prof(struct & (Z > 110) & (Y > 60) & (Y < 290), 0))
print("bottom (Z<-150): area vs X", prof(struct & (Z < -150), 0))
print("bottom (Z<-150): area vs Y", prof(struct & (Z < -150), 1))
print("diagonal/hub zone (0<Y<74, r>110): area vs X", prof(struct & (Y >= 0) & (Y < 74) & (r > 110), 0))
print("all: area vs Y", prof(struct, 1, 32))
