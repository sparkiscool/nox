"""Off-grid thin-member recheck (hub v5, 17 Oct): noxopt.thin_check samples the part on one 0.4 grid; a thin member that
straddles that grid can be missed. This reruns the same test on grids shifted by fractions of the pitch.
python3 thin_offset.py out/box_hub_v5.stl -> members under 2.0 per offset (0 everywhere = OK)."""
import sys, json
import numpy as np, trimesh
from scipy import ndimage as ndi
from skimage.morphology import skeletonize
import noxopt as NO, arm_cad as A

def thin_at(part, drawn, off, floor_mm=2.0, pitch=0.4):
    ax = [np.arange(part.bounds[0][k] - pitch / 2 + off[k] * pitch, part.bounds[1][k] + pitch, pitch) for k in range(3)]
    M = NO._voxelise(part, *ax); E = ndi.distance_transform_edt(np.pad(M, 1))[1:-1, 1:-1, 1:-1] * pitch
    far = ndi.distance_transform_edt(~NO._voxelise(drawn, *ax)) * pitch > floor_mm
    thin = skeletonize(M).astype(bool) & (E < 0.45 * floor_mm) & far
    lab, n = ndi.label(thin, NO.S26); out = []
    for k in range(1, n + 1):
        idx = np.argwhere(lab == k); length = float(np.linalg.norm(idx.max(0) - idx.min(0)) * pitch)
        if length >= 2 * floor_mm:
            p = idx * pitch + np.array([a[0] for a in ax]); r = np.hypot(p[:, 0], p[:, 2])
            out.append(dict(len_mm=round(length, 1), radius_mm=round(float(np.median(E[lab == k])), 2),
                            at=p.mean(0).round(1).tolist(), r=[round(float(r.min()), 1), round(float(r.max()), 1)],
                            Y=[round(float(p[:, 1].min()), 1), round(float(p[:, 1].max()), 1)]))
    return out

if __name__ == "__main__":
    part = trimesh.load(sys.argv[1] if len(sys.argv) > 1 else "out/box_hub_v5.stl", force="mesh")
    H = A.hub_parts(); drawn = A.tm(H["drawn"] - H["cuts"])
    res = {}
    for off in ((0, 0, 0), (0.5, 0.5, 0.5), (0.25, 0.5, 0.75), (0.75, 0.25, 0.5)):
        r = thin_at(part, drawn, off); res[str(off)] = r
        print(f"offset {off}: {len(r)} member(s) under 2.0 {r if r else 'OK'}", flush=True)
    json.dump(res, open("out/hub_v5_thin_offsets.json", "w"), indent=1)
