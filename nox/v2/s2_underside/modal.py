"""First natural frequencies of the built part with the Radxa and the flow sensor on it (README: a part that carries a
board or sensor keeps its first mode outside 80 to 350 Hz). Model (voxels, h 0.75): the part in PETG Basic (layer-aware
stiffness, printopt); the Radxa PCB as FR4 cells (E ~18 GPa, 1.6 thick) bonded to the 4 boss tops, carrying the Radxa's
mass spread over the board; the sensor as a 2.0 g mass spread over the land; supports = the three bolt pads clamped.
usage: python modal.py part.stl"""
import sys
import numpy as np, scipy.sparse as sps
from scipy.sparse.linalg import eigsh
import trimesh, manifold3d as mf
import spec, geom as G
import printopt as PO


def modes(P, h=0.75, n=6, extra_clamp=None):
    """extra_clamp: list of boxes ((x0,x1),(y0,y1),(z0,z1)); the part's nodes inside are clamped too (what-if tests)."""
    modes.share = []
    pb, gr, geo = spec.problem(h)
    occ = (gr.frac(P) >= 0.5) | pb.solid
    occ &= ~(gr.frac(G.U(geo["cuts"])) >= 0.5)
    c = gr.ecent
    board = (np.abs(c[:, 0]) <= 32.5) & (c[:, 1] >= -15) & (c[:, 1] <= 15) & (c[:, 2] >= G.R_PCB_BOT) & (c[:, 2] <= G.R_PCB_TOP)
    act = occ | board
    sup = dict(pb.supports)
    if extra_clamp:
        extra = [gr.dofs(gr.nodes_of(gr.elems_in(bx, occ))) for bx in extra_clamp]
        sup["bolts"] = np.unique(np.concatenate([sup["bolts"]] + extra))
    pb1 = PO.Problem(shape=pb.shape, h=h, origin=pb.origin, solid=act, void=~act, supports=sup, cases=pb.cases[:1],
                     material=pb.material, build=pb.build, overhang=False, R=0.5, sym=None, comp_factor=None)
    po = PO.PrintOpt(pb1)
    Erel = np.where(board, 18000.0 / pb.material.E, 1.0)
    K = po.assemble(np.where(act, Erel, 1e-9))
    # mass (kg): PETG cells, board cells carry the Radxa, land carries the sensor
    m_el = np.where(occ & ~board, pb.material.density * 1e-6 * h ** 3, 0.0)
    m_el[board] += spec.M_RADXA / board.sum()
    x0, x1 = G.FB["x"]; y0, y1 = G.FB["y"]
    land = occ & (c[:, 0] >= x0 - 2.3) & (c[:, 0] <= x1 + 2.3) & (c[:, 1] >= y0 - 2.3) & (c[:, 1] <= y1 + 2.3) & \
        (c[:, 2] >= G.TAPE_Z - 0.1) & (c[:, 2] <= G.SEAT_TOP + 0.1)
    m_el[land] += spec.M_FLOW / land.sum()
    mn = np.zeros(po.nn); np.add.at(mn, po.enodes, (m_el / 8)[:, None])
    Md = np.repeat(mn, 3)
    fd = po.freedofs["bolts"]; fd = fd[Md[fd] > 0]
    Kf = K[fd][:, fd]; Mf = sps.diags(Md[fd])
    # K in N/mm, M in kg: K*1000 -> N/m
    w2, V = eigsh(Kf * 1000.0, M=Mf, k=n, sigma=0.0, which="LM")
    f = np.sqrt(np.maximum(w2, 0)) / (2 * np.pi)
    out = []
    for k in np.argsort(f):
        v = V[:, k].reshape(-1)
        u = np.zeros(po.ndof); u[fd] = v; u = u.reshape(-1, 3)
        mom = np.abs(u * Md.reshape(-1, 3)).sum(0)
        un = np.linalg.norm(u, axis=1); i = int(np.argmax(un))
        bn = np.unique(po.enodes[board]); ln = np.unique(po.enodes[land])
        modes.share = getattr(modes, "share", []) + [(float(un[bn].mean() / un.max()), float(un[ln].mean() / un.max()))]
        nodes = np.argsort(un)[-50:]
        out.append((float(f[k]), "xyz"[int(np.argmax(mom))], gr.ncoord[i].round(1).tolist(),
                    u[i].round(3).tolist()))
    return out


if __name__ == "__main__":
    t = trimesh.load(sys.argv[1])
    P = mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))
    for r in modes(P):
        print(f"{r[0]:7.0f} Hz  mostly {r[1]}  max at {r[2]} dir {r[3]}")
    for (f, *_), (b, l) in zip(modes(P), modes.share):
        print(f"{f:7.0f} Hz  board moves {b:.2f} of the max, sensor land {l:.2f}")
