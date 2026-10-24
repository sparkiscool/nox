"""Round 2 analysis of one run: connectivity, floor contact, where the material is, profiles, per-bay links, views.
TOPO_OUT=<run dir> TOPO_RING=0|1 python3 box_system_topo_v2r2_analyse.py   (PYTHONPATH as run_topo_r2.sh)"""
import os, sys, json
import numpy as np
from scipy import ndimage as ndi
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import box_system_topo_v2r2 as R2
T = R2.T
from topo.part import tm, voxelise

OUT = os.environ["TOPO_OUT"]; R2.OPT["ring"] = os.environ.get("TOPO_RING", "0") == "1"
d = np.load(os.path.join(OUT, "density.npz")); h = float(d["h"]); sh = tuple(d["shape"]); o = np.asarray(d["origin"])
rho = d["rho_n"].reshape(sh); solid = d["solid"].reshape(sh)
ax = [o[k] + (np.arange(sh[k]) + 0.5) * h for k in range(3)]
X, Y, Z = np.meshgrid(*ax, indexing="ij")
EN = T.EN; Xi, Y0, Z0 = EN["X_IN"], EN["Y_BACK"], EN["FLOOR"]
mat = (rho > 0.5) | solid
st = ndi.generate_binary_structure(3, 1)
lab, n = ndi.label(mat, st)
sizes = np.bincount(lab.ravel()); sizes[0] = 0; order = np.argsort(sizes)[::-1]
floor = Z < Z0 + h
print("components", n, "largest cm3", [round(sizes[i] * h ** 3 / 1e3, 1) for i in order[:6]], "total cm3", round(mat.sum() * h ** 3 / 1e3, 1))
fl = np.unique(lab[mat & floor]); fl = fl[fl > 0]
main = int(order[0]); print("components on the floor:", fl.tolist(), "main", main)
setup, G = R2.setup_factory(lambda *a: None, {})
setup(h)
bod = {}
for ang, m in G["bosses"].items(): bod[f"bay{ang} bosses"] = voxelise(tm(m), *ax)
for ang, m in G["pads"].items(): bod[f"bay{ang} pad"] = voxelise(tm(m), *ax)
for i, m in enumerate(G["posts"]): bod[f"post{i}"] = voxelise(tm(m), *ax)
bod["stub"] = voxelise(tm(G["stub"]), *ax)
for i, m in enumerate(G["eboss"]): bod[f"elec boss{i}"] = voxelise(tm(m), *ax)
for i, m in enumerate(G["ring"]): bod["ring"] = voxelise(tm(m), *ax)
res = {}
for k, m in bod.items():
    ids = np.unique(lab[m & (lab > 0)])
    ok = bool(np.isin(ids, fl).any()); inmain = bool((ids == main).any())
    res[k] = dict(supported=ok, in_main=inmain)
    print(f"{k}: comps {ids.tolist()} {'JOINED to floor' if ok else 'FLOATING'} {'(main)' if inmain else ''}")
M = lab == main
fc = (M & floor)
print(f"floor contact (main body, Z<{Z0 + h:.0f} layer): {int(fc.sum())} cells = {fc.sum() * h * h / 100:.1f} cm2")
pad_floor = (mat & floor)
print(f"floor contact, any material: {pad_floor.sum() * h * h / 100:.1f} cm2")
if fc.any():
    i = np.argwhere(fc); lab2, n2 = ndi.label(fc, ndi.generate_binary_structure(3, 2))
    print("floor patches:", n2)
    for k in range(1, n2 + 1):
        idx = np.argwhere(lab2 == k)
        if len(idx) < 2: continue
        print(f"  patch {k}: {len(idx)} cells ({len(idx) * h * h / 100:.0f} cm2) X {X[tuple(idx.T)].min() - h / 2:.0f}..{X[tuple(idx.T)].max() + h / 2:.0f} Y {Y[tuple(idx.T)].min() - h / 2:.0f}..{Y[tuple(idx.T)].max() + h / 2:.0f}")
fs = (M & (X > Xi - h)).sum() * h * h / 100
print(f"main body on the +X face layer: {fs:.1f} cm2 (knocked-over support)")
r = np.hypot(X, Z); ang = np.degrees(np.arctan2(Z, X))
vol = lambda m: round((m & mat).sum() * h ** 3 / 1e3, 1)
print("r 110..200:", vol((r > 110) & (r < 200)), " r>=200:", vol(r >= 200), " r<=110:", vol(r <= 110))
print("Y slabs (cm3):", {f"{y0}": vol((Y >= y0) & (Y < y0 + 50)) for y0 in range(-66, 340, 50)})
print("Z slabs (cm3):", {f"{z0}": vol((Z >= z0) & (Z < z0 + 50)) for z0 in range(-240, 240, 50)})
print("X slabs (cm3):", {f"{x0}": vol((X >= x0) & (X < x0 + 50)) for x0 in range(-206, 206, 50)})
print("rear zone Y<-6 (behind the stub face):", vol(Y < -6), " mast zone -6<Y<0:", vol((Y >= -6) & (Y < 8)), " Y<74:", vol(Y < 74))
print("top frame Z>110:", vol(Z > 110), " base Z<-150:", vol(Z < -150))
for a in range(-210, 60, 30):
    sec = np.abs(((ang - a + 180) % 360) - 180) < 15
    print("bay", a, "material in sector r>110:", vol(sec & (r > 110)))
# bay-to-mast: does the field build bay <-> stub paths without passing the floor? path inside material only, floor layer removed
nofl = mat & ~floor & (X > -Xi + h) & (X < Xi - h)
lab3, n3 = ndi.label(nofl, st)
sid = np.unique(lab3[bod["stub"] & nofl]); sid = sid[sid > 0]
print("bay -> stub connection without the floor layer:")
for ang_, pk in R2.L2.BAYS.items():
    ids = np.unique(lab3[bod[f"bay{ang_} bosses"] & nofl]); ids = ids[ids > 0]
    print(f"  bay {ang_}: {'joined to the stub body' if np.isin(ids, sid).any() else 'NOT joined to the stub body'}")
# direct bay-to-bay links: material in the wedge between neighbouring bays r 120..200, Y 60..270
print("link band r 150..200 per 15 deg wedge (cm3):", {a: round(float((mat & (np.abs(((ang - a + 180) % 360) - 180) < 7.5) & (r > 150) & (r < 200) & (Y > 60) & (Y < 270)).sum() * h ** 3 / 1e3), 1) for a in range(-225, 75, 15)})
# profiles
def prof(mask, axis, step=16):
    c = [X, Y, Z][axis]; out = {}
    if not mask.any(): return out
    for a in np.arange(np.floor(c[mask].min() / step) * step, c[mask].max(), step):
        m = mask & (c >= a) & (c < a + step); out[int(a)] = round(float(m.sum() * h ** 3 / (step * 100)), 1)
    return out
print("area cm2 vs Z (Y<8, mast zone):", prof(mat & (Y < 8), 2))
print("area cm2 vs Z (Y<8, mast zone) by X:", prof(mat & (Y < 8), 0))
print("top frame front beam (Y>285, Z>90) area vs X:", prof(mat & (Y > 285) & (Z > 90), 0))
print("top frame X>60 (Z>110) area vs Y:", prof(mat & (X > 60) & (Z > 110), 1))
print("top frame X<-60 (Z>110) area vs Y:", prof(mat & (X < -60) & (Z > 110), 1))
print("all: area vs Y", prof(mat, 1, 32))
# electronics: does the field touch the keep-out boards? (it cannot) and how it passes: material in rear zone
json.dump(res, open(os.path.join(OUT, "connectivity.json"), "w"), indent=1)
