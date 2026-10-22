import numpy as np
import spec_top as S, geom_top as G
import matplotlib
matplotlib.use("Agg", force=True)
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy import ndimage as ndi
from skimage import measure
h = 1.25
pb, gr, geo = S.problem(h)
import sys
src = sys.argv[1] if len(sys.argv) > 1 else "runs/a/ck.npz"
zz = np.load(src)
x = zz["rho_phys"] if "rho_phys" in zz.files else zz["x"]
r = pb.R
X = (np.where(pb.void, 0, x).reshape(pb.shape) if "rho_phys" in zz.files else ndi.gaussian_filter(np.where(pb.void, 0, x).reshape(pb.shape), sigma=r / 2.0))
X = np.maximum(X, pb.solid.reshape(pb.shape).astype(float)); X[pb.void.reshape(pb.shape)] = 0
it = int(zz["it"]) if "it" in zz.files else -1
Xp = np.pad(X, 1)
v, f, _, _ = measure.marching_cubes(Xp, 0.5)
v = (v - 1) * h + np.asarray(pb.origin) + h / 2
vol = (X > 0.5).sum() * h ** 3 / 1000
def boxfaces(d):
    (x0, x1), (y0, y1), (z0, z1) = d
    P = np.array([[a, b, c] for a in (x0, x1) for b in (y0, y1) for c in (z0, z1)])
    F = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]
    return [P[i] for i in F]
comps = [((G.UBEC["x"], G.UBEC["y"], G.UBEC["z"]), "#f2b880"), ((G.XR2["x"], G.XR2["y"], G.XR2["z"]), "#a8d5a2"),
         (((-15, 16.75), (-32.5, 32.5), (G.R_LOW, G.R_BACK_PARTS)), "#7aa6d8"),
         (((-24, 28), (-25.5, 25.5), (23, 33)), "#bbbbbb")]
fig = plt.figure(figsize=(12, 4.6))
for i, (el, az, t) in enumerate([(22, -60, "3/4 from front right"), (0, -90, "from the right"), (90, -90, "from above")]):
    ax = fig.add_subplot(1, 3, i + 1, projection="3d")
    tri = v[f]; n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    L = np.array([0.4, -0.6, 0.7]); L /= np.linalg.norm(L); k = 0.35 + 0.65 * np.abs(n @ L)
    col = np.clip(np.array([0.85, 0.51, 0.17])[None] * k[:, None], 0, 1)
    ax.add_collection3d(Poly3DCollection(tri, facecolors=col, edgecolors="none"))
    for d, c in comps:
        ax.add_collection3d(Poly3DCollection(boxfaces(d), fc=c, ec="k", lw=0.3, alpha=0.18))
    ax.plot([-60, 60], [0, 0], [22, 22], color="k", lw=2)
    ax.set_xlim(-50, 50); ax.set_ylim(-50, 50); ax.set_zlim(18, 72); ax.set_box_aspect((100, 100, 54))
    ax.view_init(el, az); ax.set_title(t, fontsize=9); ax.set_axis_off()
fig.suptitle(f"S2 top case {src}: layout density (not the built part), {vol:.1f} cm3 above 0.5", fontsize=10)
plt.tight_layout(); plt.savefig(sys.argv[2] if len(sys.argv) > 2 else "snap.png", dpi=130)
print(it, vol)
