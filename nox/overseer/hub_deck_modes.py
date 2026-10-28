"""Hub deck first modes and keel socket stiffness (review 27 Oct, CLAUDE.md "Stiff where it senses": the Pi mount >= 100 Hz; the
keel sockets feed the L2 head's mode). Voxel FE of the saved mesh on nox/topo's brick element (PETG, nox/topo/material.py):
the 6 feet fixed on their plate faces (Ø14, clamped by the M3 and its insert), the Pi 5 + cooler 55 g lumped on its 4 post tops,
the UBEC 15 g and the buck 6 g on their seats; then each keel socket pushed along its keel axis on its bore wall for its axial
stiffness, against the keel's own EA/L (6/4 carbon tube, E 100 GPa, E).

python3 nox/overseer/hub_deck_modes.py <mesh.stl> [out.json] [pitch]      (~1 min at pitch 1.0)
"""
import json, os, sys
import numpy as np, scipy.sparse as sp, scipy.sparse.linalg as sla, trimesh

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
from topo.fem import hex_ke, LOCAL
from topo.material import MATERIALS, stiffness
from solid_check import voxels

MESH = sys.argv[1]; OUTJ = sys.argv[2] if len(sys.argv) > 2 else None; H = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
CORE = os.environ.get("CORE_VER", "v4")                                # v4 (27 Oct): no head load, no sockets; v3: the tower
R = json.load(open(os.path.join(ROOT, "runs", "overseer_core", CORE, "result.json")))
_RFP = os.path.join(ROOT, "runs", "overseer_core", CORE, "core_root_forces_low.json")
RF = json.load(open(_RFP)) if os.path.exists(_RFP) else {"points": {}, "cases": {}}
PTS = {k: (np.array(v["core"]), np.array(v["lug"])) for k, v in RF["points"].items()}   # core = the tube end (none in v4)
A = json.load(open(os.path.join(HERE, "overseer_core.assembly.json"))); PM = {p["name"]: p for p in A["parts"]}
mat = MATERIALS["PETG"]; RHO = mat.density * 1e-9                      # t/mm3 (mm, N, s)
E_TUBE, A_TUBE = 100e3, np.pi * (3.0 ** 2 - 2.0 ** 2)                  # 6/4 carbon tube (E), MPa and mm2

m = trimesh.load(MESH)
lo = m.bounds[0] - H; hi = m.bounds[1] + H
shape = tuple(int(np.ceil((hi[k] - lo[k]) / H)) for k in range(3))
V = voxels(m, H, lo, shape)
el = np.argwhere(V)                                                    # solid elements (i, j, k)
ny1, nz1 = shape[1] + 1, shape[2] + 1
nid = lambda i, j, k: (i * ny1 + j) * nz1 + k
en = np.stack([nid(el[:, 0] + di, el[:, 1] + dj, el[:, 2] + dk) for di, dj, dk in LOCAL], 1)
used, inv = np.unique(en, return_inverse=True); en = inv.reshape(en.shape); nn = len(used)
ui, uj, uk = used // (ny1 * nz1), (used // nz1) % ny1, used % nz1
X = lo + np.c_[ui, uj, uk] * H                                         # node coordinates (drone frame)
KE, _ = hex_ke(H, stiffness(mat, 2))
edof = (en[:, :, None] * 3 + np.arange(3)).reshape(len(el), 24)
rows = np.repeat(edof, 24, 1).ravel(); cols = np.tile(edof, (1, 24)).ravel()
K = sp.csr_matrix((np.tile(KE.ravel(), len(el)), (rows, cols)), shape=(3 * nn, 3 * nn))
mass = np.zeros(nn); np.add.at(mass, en.ravel(), RHO * H ** 3 / 8.0)    # lumped element mass

def add_mass(sel, g):
    if sel.sum() == 0: raise SystemExit(f"no nodes for a {g} g mass")
    mass[sel] += g * 1e-6 / sel.sum()

feet = [tuple(f) for f in R["feet_xy"]]
fixed = np.zeros(nn, bool)
for x, y in feet:
    fixed |= (np.hypot(X[:, 0] - x, X[:, 1] - y) <= 7.0) & (X[:, 2] <= 0.5 * H)
posts = [tuple(p) for p in R["pi_holes"]]; zp = R["coupon"]["z_pi"]
add_mass(np.any([(np.hypot(X[:, 0] - x, X[:, 1] - y) <= 3.6) & (X[:, 2] >= zp - 1.0 - 0.5 * H) for x, y in posts], 0), 55.0)
for nm, g in (("UBEC 5 V", 15.0), ("buck 12 V", 6.0)):
    b = trimesh.load(os.path.join(ROOT, PM[nm]["mesh"])).bounds
    inb = (X[:, 0] >= b[0][0]) & (X[:, 0] <= b[1][0]) & (X[:, 1] >= b[0][1]) & (X[:, 1] <= b[1][1]) & (X[:, 2] >= b[0][2] - 1.5 * H) & (X[:, 2] <= b[0][2] + H)
    add_mass(inb, g)

free = np.repeat(~fixed, 3); fd = np.flatnonzero(free)
Kf = K[fd][:, fd].tocsc(); Mf = sp.diags(np.repeat(mass, 3)[fd])
vals, vecs = sla.eigsh(Kf, k=4, M=Mf, sigma=0.0, which="LM")
f = np.sqrt(np.maximum(vals, 0)) / (2 * np.pi)
modes = []
for k in range(len(f)):
    u = np.zeros(3 * nn); u[fd] = vecs[:, k]; U = u.reshape(nn, 3); a = np.linalg.norm(U, axis=1); i = int(np.argmax(a))
    modes.append({"hz": round(float(f[k]), 1), "peak_at": np.round(X[i], 1).tolist(), "dir": np.round(U[i] / a[i], 2).tolist()})

lu = sla.splu(Kf)
sock = {}
for nm_, (a_, b_) in PTS.items():
    d = (b_ - a_) / np.linalg.norm(b_ - a_); L_s = 15.6 if nm_.startswith("L2") else 15.0
    rel = X - a_; t = rel @ d; r = np.linalg.norm(rel - np.outer(t, d), axis=1)
    wall = (t >= 0) & (t <= L_s) & (r >= 3.1 - 0.2) & (r <= 3.1 + 1.2 * H)
    F = np.zeros(3 * nn); F.reshape(nn, 3)[wall] = -d / wall.sum()     # 1 N pushing the socket in along its tube
    u = np.zeros(3 * nn); u[fd] = lu.solve(F[fd])
    defl = float(-(u.reshape(nn, 3)[wall] @ d).mean())
    k_s = 1.0 / defl; L_k = float(np.linalg.norm(b_ - a_)); k_t = E_TUBE * A_TUBE / L_k
    sock[nm_] = {"nodes": int(wall.sum()), "k_socket_N_per_mm": round(k_s), "k_tube_N_per_mm": round(k_t),
                 "series_freq_factor": round(float(np.sqrt(1.0 / (1.0 + k_t / k_s))), 3)}
# FE bolt reactions (review: "the topology run reports the FE value"): both keels push 387.3 N (F) and the deck's payload at 35 g up;
# each foot's reaction = the sum of K u over its fixed nodes (the plate + M3 + insert, rigid here)
DIRS = {0: [1, 0, 0], 4: [0, 0, 1]}
res_react = {}
for k_, dv in DIRS.items():
    case = next((c for c in RF["cases"] if c.startswith(f"crash {k_} ")), f"crash {k_} (35 g, payload only)")
    F = np.zeros(3 * nn)
    for nm_, n_ in RF["cases"].get(case, {}).items():
        a_, b_ = PTS[nm_]; d = (b_ - a_) / np.linalg.norm(b_ - a_); L_s = 15.6 if nm_.startswith("L2") else 15.0
        rel = X - a_; t = rel @ d; r = np.linalg.norm(rel - np.outer(t, d), axis=1)
        wall = (t >= 0) & (t <= L_s) & (r >= 2.9) & (r <= 3.1 + 1.2 * H)
        F.reshape(nn, 3)[wall] += d * n_ * 1.1 / wall.sum()             # SF 1.1 (crash), + tension pulls toward the head
    F.reshape(nn, 3)[:] -= np.outer(mass, np.array(dv, float)) * 35 * 9810.0 * 1.1
    u = np.zeros(3 * nn); u[fd] = lu.solve(F[fd]); Rf = (K @ u - F).reshape(nn, 3)
    react = {}
    for x, y in feet:
        sel = fixed & (np.hypot(X[:, 0] - x, X[:, 1] - y) <= 7.0)
        v = Rf[sel].sum(0); react[f"({x:g}, {y:g})"] = {"N": round(float(np.linalg.norm(v)), 1), "pull_N": round(float(-v[2]), 1)}
    res_react[case] = {"per_foot": react, "max_N": max(r_["N"] for r_ in react.values()), "max_pull_N": max(r_["pull_N"] for r_ in react.values())}
res = {"mesh": os.path.relpath(MESH, ROOT), "pitch_mm": H, "elements": int(len(el)), "dofs_free": int(len(fd)), "material": "PETG (nox/topo)",
       "masses_g": {"Pi 5 + cooler on the post tops": 55.0, "UBEC": 15.0, "buck": 6.0}, "modes": modes, "keel_sockets": sock, "bolt_reactions_fe": res_react,
       "note": "first modes of the deck with the Pi, UBEC and buck lumped on their seats, feet fixed (C on M mesh, E masses); socket stiffness along the keel"}
print(json.dumps(res, indent=1))
if OUTJ: json.dump(res, open(OUTJ, "w"), indent=1)
