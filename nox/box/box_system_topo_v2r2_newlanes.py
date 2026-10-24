"""Overlap of a run's material (TOPO_OUT) with the arm's NEW lanes (heads v7, hub message 24 Oct): carriage r <= 159, Z >= 89, X -35..47.5,
Y 53.75..75.35; head + grabber r 95..159.6, X +-47.5, Y 73.75..217. Material cells (rho > 0.5) inside, cm3 (M)."""
import os, numpy as np
OUT = os.environ["TOPO_OUT"]
d = np.load(os.path.join(OUT, "density.npz")); h = float(d["h"]); sh = tuple(d["shape"]); o = np.asarray(d["origin"])
mat = ((d["rho_n"].reshape(sh) > 0.5) | d["solid"].reshape(sh))
ax = [o[k] + (np.arange(sh[k]) + 0.5) * h for k in range(3)]
X, Y, Z = np.meshgrid(*ax, indexing="ij"); r = np.hypot(X, Z)
car = (r <= 159) & (Z >= 89) & (X >= -35) & (X <= 47.5) & (Y >= 53.75) & (Y <= 75.35)
gra = (r >= 95) & (r <= 159.6) & (np.abs(X) <= 47.5) & (Y >= 73.75) & (Y <= 217)
cell = h ** 3 / 1e3
print(f"carriage lane overlap {(mat & car).sum() * cell:.1f} cm3 (M, lane volume {car.sum() * cell:.0f}); head+grabber overlap {(mat & gra).sum() * cell:.1f} cm3 (lane {gra.sum() * cell:.0f})")
