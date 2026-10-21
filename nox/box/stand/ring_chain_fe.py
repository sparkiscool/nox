"""Ring arc as a CHAIN (19 Oct 2026 21:00): the right arc (bay module at -60 + frames -30, 0, endR) as one voxel FE with the
joints modelled as bolted contacts, not bonded: at every joint plane a 0.3 slab is removed, and the pieces touch only through
(a) the outer M5 pair (bonded cylinders r 4.5 at r 172, Y 145 / 209: the bolt + the clamped lug faces), (b) optionally an inner
M5 pair (r 150.5, Y 153 / 201: v7 frames), (c) the dovetail (bonded: the tongue is captured in the groove). The arc hangs from
the cradle (module lips fixed). Loads (E): funnel + Scout 6 N down + rim push 6.5 N sideways at the end frame's pads, a 20 N
hand on the 0 deg frame's rim. Configs: v6 joints / + inner pair / + inner pair + wall brackets (Y, Z fixed at the bracket lugs
at -15, 15, 45 deg: the sheet carries in-plane only). Reports tip drop, worst index (misuse), and the worst joint.
python3 ring_chain_fe.py -> ring_chain_fe.json, ring_chain_fe.png"""
import os, sys, json, math, numpy as np, trimesh, scipy.sparse as sp
os.environ.setdefault("PYPARDISO_MKL_RT", "/usr/local/lib/libmkl_rt.so.3")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../src/nox"))
import printopt as P, stand_v6_parts as S, manifold3d as mf, pypardiso
from stand_v6_fe import failure

MAT = P.MATERIALS["PETG"]; A = P.allowables(MAT, "misuse"); h = 2.0
def man(t): return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))
mod = trimesh.load("models/box_baymod_v4i_W.obj", force="mesh")
mod.apply_transform(trimesh.transformations.rotation_matrix(math.radians(90 + 60), [0, 1, 0]))   # bay 0 (+Z) -> -60
pieces = [mod]
for n, p in (("frame_mid", -30), ("frame_mid", 0), ("frame_endR", 30)):
    pieces.append(S.tm(S.place(man(trimesh.load(f"box_{n}_v6_W.stl")), p)))
arc = trimesh.util.concatenate(pieces)
vox = arc.voxelized(h).fill()
solid = vox.matrix.copy(); org = np.asarray(vox.transform[:3, 3] - h / 2, float)
nx, ny, nz = solid.shape
ex, ey, ez = [a.ravel() for a in np.meshgrid(np.arange(nx), np.arange(ny), np.arange(nz), indexing="ij")]
cent = np.stack([ex, ey, ez], 1) * h + org + h / 2
# joint planes at -45, -15, 15 deg (module/-30, -30/0, 0/endR); plane coordinate d = x cos(a) ... a plane through the axis at angle a
def plane_dist(pts, ang):
    a = math.radians(ang); n = np.array([-math.sin(a), 0, math.cos(a)])   # normal to the radial plane at angle ang
    return pts @ n
r = np.hypot(cent[:, 0], cent[:, 2]); ang = np.degrees(np.arctan2(cent[:, 2], cent[:, 0]))
def joint_mask(inner):
    cut = np.zeros(len(cent), bool)
    for jp in (-45, -15, 15):
        slab = (np.abs(plane_dist(cent, jp)) < 1.6) & (np.abs(ang - jp) < 20) & (r > 95)
        keep = np.zeros(len(cent), bool)
        for y in (145.0, 209.0):          # outer bolts
            keep |= (np.hypot(cent[:, 1] - y, r - 172.0) < 4.5)
        if inner:
            for y in (153.0, 201.0):
                keep |= (np.hypot(cent[:, 1] - y, r - 150.5) < 4.0)
        keep |= (r > 150.0) & (r < 163.5) & (((cent[:, 1] > 135) & (cent[:, 1] < 155)) | ((cent[:, 1] > 199) & (cent[:, 1] < 219)))   # dovetail segments
        cut |= slab & ~keep
    return cut

def run(inner, brackets):
    act = solid.ravel() & ~joint_mask(inner)
    nn = (nx + 1) * (ny + 1) * (nz + 1)
    nid = lambda i, j, k: (i * (ny + 1) + j) * (nz + 1) + k
    corners = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0), (0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)]
    enodes = np.stack([nid(ex + a, ey + b, ez + c) for a, b, c in corners], 1)
    edof = np.repeat(enodes * 3, 3, axis=1) + np.tile([0, 1, 2], 8)
    C = P.stiffness(MAT, 0); KE, B0 = P.hex_KE(h, C); CB = C @ B0      # frames print along ~X; the module along Y: one axis for all (E)
    iK = np.repeat(edof[act], 24, axis=1).ravel(); jK = np.tile(edof[act], (1, 24)).ravel()
    K = sp.coo_matrix((np.tile(KE.ravel(), act.sum()), (iK, jK)), shape=(3 * nn, 3 * nn)).tocsr()
    gi, gj, gk = np.meshgrid(np.arange(nx + 1), np.arange(ny + 1), np.arange(nz + 1), indexing="ij")
    X = np.stack([gi.ravel(), gj.ravel(), gk.ravel()], 1) * h + org
    used = np.zeros(nn, bool); used[np.unique(enodes[act])] = True
    # keep only the main connected body (loose voxels after the cut)
    import scipy.sparse.csgraph as cg
    adj = sp.coo_matrix((np.ones(len(iK)), (iK // 3, jK // 3)), shape=(nn, nn))
    ncomp, lab = cg.connected_components(adj, directed=False)
    main = np.bincount(lab[used]).argmax()
    rX = np.hypot(X[:, 0], X[:, 2]); aX = np.degrees(np.arctan2(X[:, 2], X[:, 0]))
    fixn = used & (X[:, 2] < -150) & (np.abs(X[:, 0]) < 31) & (X[:, 1] > 139) & (X[:, 1] < 198)   # cradle lips / plate zone of the -60.. wait: the module at -60: its back plate faces -60: fix its outer plate region
    fixn = used & (rX > 147) & (np.abs(aX + 60) < 12)                                             # the -60 module's back plate zone = where the cradle/neighbour holds it (E)
    fdofs = np.repeat(np.where(fixn)[0] * 3, 3) + np.tile([0, 1, 2], fixn.sum())
    if brackets:
        bn = np.zeros(nn, bool)
        for a in (-15, 15, 45):
            bn |= used & (rX > 170) & (np.abs(aX - a) < 3.5) & (((np.abs(X[:, 1] - 145) < 8)) | (np.abs(X[:, 1] - 209) < 8))
        fdofs = np.concatenate([fdofs, np.where(bn)[0] * 3 + 1, np.where(bn)[0] * 3 + 2])
    fdofs = np.unique(fdofs)
    free = np.setdiff1d(np.where(np.repeat(used & (lab == main), 3))[0], fdofs)
    F = np.zeros(3 * nn)
    pads = used & (rX > 183) & (aX > 35) & (aX < 60)
    hand = used & (rX > 178) & (np.abs(aX) < 12)
    def put(n, v):
        idx = np.where(n)[0]; v = np.asarray(v, float) / len(idx)
        F[idx * 3] += v[0]; F[idx * 3 + 1] += v[1]; F[idx * 3 + 2] += v[2]
    put(pads, [6.5, 0, -6.0]); put(hand, [0, 0, -20.0])
    Kf = sp.triu(K[free][:, free].tocsr(), format="csr")
    ps = pypardiso.PyPardisoSolver(); ps.set_matrix_type(2)
    U = np.zeros(3 * nn); U[free] = ps.solve(Kf, F[free]); ps.free_memory(everything=True)
    ue = U[edof[act]]; s = ue @ CB.T
    hot = fixn | pads | hand
    if brackets: hot |= bn
    smask = ~hot[enodes[act]].any(1)
    idx = np.where(smask, failure(s, A, 0), 0)
    c = cent[act]; k = np.argmax(idx)
    tip = U.reshape(-1, 3)[pads]
    out = dict(tip_drop_mm=round(float(-tip[:, 2].mean()), 3), tip_side_mm=round(float(tip[:, 0].mean()), 3),
               worst=round(float(idx.max()), 3), at=np.round(c[k], 1).tolist(), at_angle=round(float(math.degrees(math.atan2(c[k][2], c[k][0]))), 1),
               at_r=round(float(math.hypot(c[k][0], c[k][2])), 1), p99=round(float(np.quantile(idx[smask], 0.99)), 3), active=int(act.sum()))
    # per-joint worst
    for jp in (-45, -15, 15):
        near = np.abs(plane_dist(c, jp)) < 8
        out[f"joint_{jp}"] = round(float(idx[near].max()), 3) if near.any() else None
    return out, (c, idx)

res = {}; maps = {}
for name, inner, br in (("v6 joints (outer pair + dovetail)", False, False), ("+ inner pair", True, False), ("+ inner pair + wall brackets", True, True), ("v7: outer pair + wall brackets", False, True)):
    res[name], maps[name] = run(inner, br); print(name, res[name])
json.dump(res, open("ring_chain_fe.json", "w"), indent=1)
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
fig, axs = plt.subplots(1, 4, figsize=(22, 7), dpi=120)
for ax, (name, (c, idx)) in zip(axs, maps.items()):
    o = np.argsort(idx); sc = ax.scatter(c[o, 0], c[o, 2], c=idx[o], s=3, cmap="inferno", vmin=0, vmax=max(1, idx.max()))
    ax.set_aspect("equal"); ax.set_title(f"{name}\nworst {res[name]['worst']:.2f}, tip drop {res[name]['tip_drop_mm']:.2f} mm", fontsize=10); ax.set_xticks([]); ax.set_yticks([])
    plt.colorbar(sc, ax=ax, fraction=0.04)
fig.suptitle("Right arc as a bolted chain: failure index (misuse), funnel + Scout + rim push at the end pads, 20 N hand at 0°; fixed at the -60 module's back", fontsize=11)
fig.tight_layout(); fig.savefig("ring_chain_fe.png"); print("ring_chain_fe.png")
