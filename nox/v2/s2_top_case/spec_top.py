"""S2 top case v1: printopt.Problem (voxel masks, supports, load cases). 16 Oct 2026, top case chat.
Loads: v2 README (flight 4 g + 2 g, landing 6 g, 25 g on parts that hold a protected board, poke 5 N, pinch 20 N) plus
the upside-down floor landing on the boss tops (the top contact): 0.52 kg all-up (dimensions C, approx) x 6 g = 30.6 N.
Masses (E): Radxa + 4 screws 10.5 g, UBEC 20 g (scout.md, E), XR2 + tape 1.6 g.
Print: bed face = the part's right side (y min), build +y, 45 deg overhang filter (DEFAULT 16 Oct): every face that holds
something (plate pads, boss faces, tape land) is vertical in the printer and the spans over the FC are in-layer.
No mirror: a directional overhang filter can't be mirrored in y."""
import os
import numpy as np
import printopt as PO
import noxopt as NO
import geom_top as G

PETG_BASIC = PO.Material("PETG Basic (Bambu TDS)", E=1460.0, Ez=1120.0, nu=0.38, Xt=48.0, Zt=39.0, S=39.0, density=1.27)
g = 9.81
M_RADXA, M_UBEC, M_XR2 = 0.0105, 0.020, 0.0016          # E
M_DRONE = 0.52                                          # all-up, approx (dimensions C)
R_MM = float(os.environ.get("S2T_R_MM", 3.0))           # filter radius mm (underside v2 setting)
# cases trimmed 16 Oct (run a ran out of memory at 1.15M dof x 34 basis loads): one rolled-boss case, two pokes,
# stiffness as the pitch and roll couples only
BUILD = os.environ.get("S2T_BUILD", "+y")


def frac_vox(tm, xs, ys, zs, sub=3):
    h = xs[1] - xs[0]
    off = (np.arange(sub) + 0.5) / sub * h - h / 2
    zf = (zs[:, None] + off[None, :]).ravel()
    acc = np.zeros((len(xs), len(ys), len(zs)))
    for ox in off:
        for oy in off:
            A = NO._voxelise(tm, xs + ox, ys + oy, zf)
            acc += A.reshape(len(xs), len(ys), len(zs), sub).sum(-1)
    return acc / sub ** 3


class Grid:
    def __init__(self, x, y, z, h):
        self.h = h
        self.nx = int(round((x[1] - x[0]) / h)); self.ny = int(round((y[1] - y[0]) / h)); self.nz = int(round((z[1] - z[0]) / h))
        self.origin = (x[0], y[0], z[0]); self.shape = (self.nx, self.ny, self.nz)
        self.xs = x[0] + (np.arange(self.nx) + 0.5) * h; self.ys = y[0] + (np.arange(self.ny) + 0.5) * h
        self.zs = z[0] + (np.arange(self.nz) + 0.5) * h
        I, J, K = np.meshgrid(np.arange(self.nx + 1), np.arange(self.ny + 1), np.arange(self.nz + 1), indexing="ij")
        self.ncoord = np.stack([I.ravel(), J.ravel(), K.ravel()], 1) * h + np.asarray(self.origin)
        self.nn = len(self.ncoord); self.ndof = 3 * self.nn
        ex, ey, ez = [a.ravel() for a in np.meshgrid(np.arange(self.nx), np.arange(self.ny), np.arange(self.nz), indexing="ij")]
        nid = lambda i, j, k: (i * (self.ny + 1) + j) * (self.nz + 1) + k
        cs = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0), (0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)]
        self.enodes = np.stack([nid(ex + a, ey + b, ez + c) for a, b, c in cs], 1)
        self.ecent = np.stack([ex, ey, ez], 1) * h + np.asarray(self.origin) + h / 2

    def frac(self, m):
        o = np.asarray(self.origin); hi = o + np.array(self.shape) * self.h
        clip = G.box((o[0] - 1, hi[0] + 1), (o[1] - 1, hi[1] + 1), (o[2] - 0.5, hi[2] + 1))
        return frac_vox(G.to_trimesh(m ^ clip), self.xs, self.ys, self.zs).ravel()

    def nodes_of(self, emask):
        return np.unique(self.enodes[emask])

    def elems_in(self, box, mask):
        c = self.ecent; ok = mask.copy()
        for k in range(3):
            if box[k] is not None: ok &= (c[:, k] >= box[k][0]) & (c[:, k] <= box[k][1])
        return ok

    def force(self, nodes, vec):
        F = np.zeros(self.ndof)
        if len(nodes) == 0: raise ValueError("load region has no nodes")
        for k in range(3):
            if vec[k]: F[3 * nodes + k] = vec[k] / len(nodes)
        return F

    def dofs(self, nodes, comps=(0, 1, 2)):
        return np.concatenate([3 * nodes + k for k in comps])


def problem(h=1.0, report=False):
    E = G.ENV
    gr = Grid(E["x"], E["y"], E["z"], h)
    geo = G.parts()
    fd = gr.frac(G.U(geo["drawn"])); fc = gr.frac(G.U(geo["cuts"])); fk = gr.frac(G.U(geo["keep"]))
    cut = fc >= 0.5
    solid = (fd >= 0.3) & ~cut
    void = ((fk >= 0.5) | cut) & ~solid
    from scipy import ndimage as ndi
    lab, n = ndi.label((~void).reshape(gr.shape))
    lab = lab.ravel(); keep_ids = np.unique(lab[solid])
    void |= ~np.isin(lab, keep_ids)
    S = lambda box: gr.nodes_of(gr.elems_in(box, solid))
    r = G.PAD_R + 0.1
    # supports: the two side pads only (run e, 17 Oct). With the nose pad held too, nothing tied it to the part: runs b
    # and d (after the prune) left it detached. Free, it gets tie cases like any drawn feature and must be joined; the
    # real part is bolted there too, so every case is checked a little conservatively
    SUP = [b for b in G.bolts() if b[1] != 0] if os.environ.get("S2T_NOSE_HELD", "0") != "1" else G.bolts()
    pads = [S(((bx - r, bx + r), (by - r, by + r), (G.pad_z0(by), G.pad_z0(by) + G.PAD_T + 0.1))) for bx, by in SUP]
    supports = dict(bolts=gr.dofs(np.unique(np.concatenate(pads))))
    rb = G.BOSS_R + 0.1
    e = max(1.1, 0.6 * h)                       # load-layer depth: at least one cell
    boss = [S(((hx - rb, hx + rb), (hy - rb, hy + rb), (G.R_BACK - 0.1, G.R_BACK + e))) for hx, hy in G.R_HOLES]
    tops = [S(((hx - rb, hx + rb), (hy - rb, hy + rb), (G.BOSS_TOP - e, G.BOSS_TOP + 0.1))) for hx, hy in G.R_HOLES]
    BO, TO = np.unique(np.concatenate(boss)), np.unique(np.concatenate(tops))
    L = G.LAND
    UB = S(((L["x"][0] - 0.1, L["x"][0] + e), L["y"], L["z"]))                     # tape face, UBEC side
    XF = S(((L["x"][1] - e, L["x"][1] + 0.1), G.XR2["y"], G.XR2["z"]))             # tape face, XR2 side
    cases = []
    LC = lambda name, F, cls, acc=None: cases.append(
        PO.LoadCase(name, F, cls=cls, support="bolts", accel=None if acc is None else np.asarray(acc, float)))

    def payload(a):
        a = np.asarray(a, float)
        return gr.force(BO, M_RADXA * g * a) + gr.force(UB, M_UBEC * g * a) + gr.force(XF, M_XR2 * g * a)
    LC("flight 4 g down + 2 g x", payload((2, 0, -4)), "flight", (2, 0, -4))
    LC("flight 4 g down + 2 g y", payload((0, 2, -4)), "flight", (0, 2, -4))
    LC("landing 6 g", payload((0, 0, -6)), "flight", (0, 0, -6))
    for ax, nm in ((0, "x"), (1, "y"), (2, "z")):
        for s in (+1, -1):
            a = [0, 0, 0]; a[ax] = 25 * s
            LC(f"crash 25 g {'+' if s > 0 else '-'}{nm} (Radxa + UBEC + XR2)", payload(a), "crash", a)
    LC("upside-down landing 6 g on the boss tops", gr.force(TO, (0, 0, -M_DRONE * g * 6)), "crash")
    LC("upside-down landing on one boss (rolled)", gr.force(tops[0], (0, 0, -M_DRONE * g * 3)), "crash")
    LC("camera plug push 20 N -x on the board", gr.force(BO, (-20.0, 0, 0)), "misuse")
    LC("camera plug pull 20 N +x on the board", gr.force(BO, (20.0, 0, 0)), "misuse")
    LC("board push 20 N up (hand, from below)", gr.force(BO, (0, 0, 20.0)), "misuse")
    LC("UBEC press 20 N +x (zip tie / hand)", gr.force(UB, (20.0, 0, 0)), "misuse")
    for v in ((5, 0, 0), (0, 5, 0)):
        LC(f"poke 5 N boss tops {v}", gr.force(TO, v), "misuse")
    LC("stiff pitch couple", sum(gr.force(boss[k], (0, 0, 1.0 if G.R_HOLES[k][0] > 0 else -1.0)) for k in range(4)), None)
    LC("stiff roll couple", sum(gr.force(boss[k], (0, 0, 1.0 if G.R_HOLES[k][1] > 0 else -1.0)) for k in range(4)), None)
    pb = PO.Problem(shape=gr.shape, h=h, origin=gr.origin, solid=solid, void=void, supports=supports, cases=cases,
                    material=PETG_BASIC, build=BUILD, overhang=True, R=max(2.0, R_MM / h), sym=None, comp_factor=None)
    return pb, gr, geo


if __name__ == "__main__":
    import sys, time
    h = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
    t = time.time(); pb, gr, geo = problem(h)
    des = ~pb.solid & ~pb.void
    print(f"shape {pb.shape}, h {pb.h}, solid {pb.solid.sum()} ({pb.solid.sum()*pb.h**3/1000:.2f} cm3), design {des.sum()} "
          f"({des.sum()*pb.h**3/1000:.1f} cm3), active {(~pb.void).sum()}, cases {len(pb.cases)}, {time.time()-t:.0f}s")
    bodies, small = NO.interface_bodies(pb, 1)
    print("drawn voxel bodies:", [len(b[1]) for b in bodies], "fragments", len(small))
    from scipy import ndimage as ndi
    lab, n = ndi.label((~pb.void).reshape(pb.shape))
    print("active regions:", n, "sizes", np.bincount(lab.ravel())[1:][:10])
    np.savez_compressed(f"masks_h{h}.npz", solid=pb.solid, void=pb.void, shape=pb.shape, h=pb.h, origin=pb.origin)
