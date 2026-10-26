"""Own connectivity check for the xswap print files (same test as the repo's solid_check, written fresh): voxelise at
0.25 mm, erode by a 0.9 mm ball (a legal 2.0 wall survives), and require ONE connected body; also report how much of the
part vanished (features under ~1.8 mm thick). python3 xswap_solid.py [files...]"""
import glob, os, sys
import numpy as np, trimesh
from scipy import ndimage as ndi

def voxels(m, pitch):
    """inside test by ray parity along z, one ray per (x, y) cell centre (lean on memory)."""
    lo, hi = m.bounds
    xs = np.arange(lo[0] + pitch / 2, hi[0], pitch); ys = np.arange(lo[1] + pitch / 2, hi[1], pitch)
    zs = np.arange(lo[2] + pitch / 2, hi[2], pitch)
    A = np.zeros((len(xs), len(ys), len(zs)), bool)
    XX, YY = np.meshgrid(xs, ys, indexing="ij")
    org = np.stack([XX.ravel(), YY.ravel(), np.full(XX.size, lo[2] - 1.0)], 1)
    for c0 in range(0, len(org), 40000):
        o = org[c0:c0 + 40000]
        loc, ray, _ = m.ray.intersects_location(o, np.tile([0, 0, 1.0], (len(o), 1)), multiple_hits=True)
        if not len(ray): continue
        order = np.lexsort((loc[:, 2], ray)); ray = ray[order]; z = loc[order, 2]
        for r_ in np.unique(ray):
            zz = np.unique(np.round(z[ray == r_], 5))
            idx = c0 + r_; i, j = divmod(idx, len(ys))
            for k in range(0, len(zz) - 1, 2):
                A[i, j, (zs > zz[k]) & (zs < zz[k + 1])] = True
    return A

def check(path, pitch=0.3, erode=0.9):
    m = trimesh.load(path, force="mesh")
    a = voxels(m, pitch)
    r = int(round(erode / pitch))
    assert abs(r * pitch - erode) < 0.051, "pitch must divide the erosion (0.3 for 0.9)"
    g = np.mgrid[-r:r + 1, -r:r + 1, -r:r + 1]
    ball = (g ** 2).sum(0) <= r * r
    e = ndi.binary_erosion(np.pad(a, r + 1), structure=ball)
    lab, n = ndi.label(e, structure=np.ones((3, 3, 3)))
    sizes = np.bincount(lab.ravel())[1:] * pitch ** 3
    plain = ndi.label(a, structure=np.ones((3, 3, 3)))[1]
    ok = (n == 1) and (plain == 1)
    return ok, dict(bodies_plain=int(plain), bodies_eroded=int(n), eroded_mm3=[round(float(s), 1) for s in sorted(sizes, reverse=True)[:6]],
                    kept_frac=round(float(e.sum()) / max(1, a.sum()), 3))

if __name__ == "__main__":
    files = sys.argv[1:] or sorted(glob.glob(os.path.join(os.path.dirname(__file__), "..", "..", "..", "runs", "xswap", "v3", "stl", "*.stl")))
    bad = 0
    for f in files:
        ok, info = check(f)
        bad += not ok
        print(os.path.basename(f), "PASS" if ok else "FAIL", info)
    sys.exit(1 if bad else 0)
