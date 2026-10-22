"""As-built strength recheck of s2_receiver_v3.stl in its own process (the memory peak of the run)."""
import sys, copy, os
os.environ.setdefault("PYPARDISO_MKL_RT", "/usr/local/lib/libmkl_rt.so.3")
sys.argv = ["receiver_v3.py", "--none"]
src = open("receiver_v3.py").read().split('if __name__ == "__main__":')[0]
exec(src)
import trimesh
P = trimesh.load("s2_receiver_v3.stl")
A = NO._voxelise(P, xs, ys, zs).ravel().astype(float)
pbs = copy.copy(pb0); pbs.void = (A < 0.5) & ~pb0.solid; pbs.solid = np.zeros_like(pb0.solid)
rows, rep = NO.strength_report(pbs, np.where(pb0.solid, 1.0, A))
print(rep)
idx = lambda r: max(r["inlayer"], r["across"])
w = max(rows, key=idx)
real = [r for r in rows if r["cls"] not in ("misuse",)]
wr_ = max(real, key=idx)
print(f"RECHECK worst {idx(w):.2f} in {w['case']} | worst non-misuse {idx(wr_):.2f} in {wr_['case']}", flush=True)
for r in sorted(rows, key=idx, reverse=True)[:6]:
    print(f"  {r['case']:45s} {r['cls']:10s} in-layer {r['inlayer']:.2f} across {r['across']:.2f}")
