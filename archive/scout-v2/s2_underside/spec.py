"""S2 underside v1: printopt.Problem (voxel masks, supports, v2 load cases). 13 Oct 2026, underside chat.
Loads: v2 README (flight 4 g + 2 g, landing 6 g, 25 g on parts that hold a protected board, poke 5 N, pinch 20 N).
Masses (E): Radxa Zero 3W with 4 screws 10.5 g; flow sensor with tape 2.0 g."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "tools"))
import numpy as np
import printopt as PO
import noxopt as NO
import geom as G

PETG_BASIC = PO.Material("PETG Basic (Bambu TDS)", E=1460.0, Ez=1120.0, nu=0.38, Xt=48.0, Zt=39.0, S=39.0, density=1.27)
g = 9.81
M_RADXA = 0.0105          # E
M_FLOW = 0.0020           # 1.5 g (MicoAir) + tape, E
R_MM = float(os.environ.get("S2U_R_MM", 3.0))   # filter radius, mm (DEFAULT 13 Oct: 2.0 left flush members at 1.3 as built)


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
    fs = gr.frac(G.skin())
    cut = fc >= 0.5
    solid = (fd >= 0.3) & ~cut
    void = ((fk >= 0.5) | (fs >= 0.5) | cut) & ~solid
    # open pockets the keep-outs cut off from the drawn features carry no load path: void (13 Oct: one at the tail, behind
    # the FC box above the SD and header-lead lanes, made the stiffness matrix singular)
    from scipy import ndimage as ndi
    lab, n = ndi.label((~void).reshape(gr.shape))
    lab = lab.ravel(); keep_ids = np.unique(lab[solid])
    void |= ~np.isin(lab, keep_ids)
    c = gr.ecent
    S = lambda box: gr.nodes_of(gr.elems_in(box, solid))
    # support: the three bolt pads, clamped against the plate by the bolt (whole pad held, as the slider spec)
    pads = [S(((bx - G.FLANGE_R - 0.1, bx + G.FLANGE_R + 0.1), (by - G.FLANGE_R - 0.1, by + G.FLANGE_R + 0.1),
               (G.PLATE_UNDER - G.FLANGE_T - 0.1, G.PLATE_UNDER + 0.1))) for bx, by in G.pad_bolts()]
    supports = dict(bolts=gr.dofs(np.unique(np.concatenate(pads))))
    # load regions: the top ring of each boss (the board bears there), the sensor land under the tape face
    boss = [S(((hx - G.BOSS_R - 0.1, hx + G.BOSS_R + 0.1), (hy - G.BOSS_R - 0.1, hy + G.BOSS_R + 0.1),
               (G.R_PCB_BOT - 1.6, G.R_PCB_BOT + 0.1))) for hx, hy in G.R_HOLES]
    B = np.unique(np.concatenate(boss))
    x0, x1 = G.FB["x"]; y0, y1 = G.FB["y"]
    LAND = S(((x0 - G.LAND_OUT - 0.1, x1 + G.LAND_OUT + 0.1), (y0 - G.LAND_OUT - 0.1, y1 + G.LAND_OUT + 0.1),
              (G.TAPE_Z - 0.1, G.SEAT_TOP + 0.1)))
    TAB_N = S(((x1, x1 + 3), (-3, 3), (G.TAPE_Z - G.TAB_DROP - 0.1, G.TAPE_Z)))
    cases = []
    LC = lambda name, F, cls, acc=None: cases.append(
        PO.LoadCase(name, F, cls=cls, support="bolts", accel=None if acc is None else np.asarray(acc, float)))
    def payload(a):              # Radxa on the bosses + sensor on the land, accelerated at a (g, the direction pushed)
        a = np.asarray(a, float)
        return gr.force(B, M_RADXA * g * a) + gr.force(LAND, M_FLOW * g * a)
    LC("flight 4 g down + 2 g x", payload((2, 0, -4)), "flight", (2, 0, -4))
    LC("flight 4 g down + 2 g y", payload((0, 2, -4)), "flight", (0, 2, -4))
    LC("landing 6 g", payload((0, 0, -6)), "flight", (0, 0, -6))
    for ax, nm in ((0, "x"), (1, "y"), (2, "z")):
        for s in (+1, -1):
            a = [0, 0, 0]; a[ax] = 25 * s
            LC(f"crash 25 g {'+' if s > 0 else '-'}{nm} (Radxa + sensor)", payload(a), "crash", a)
    # handling and plugs
    LC("camera plug push 20 N +y on the board", gr.force(B, (0, 20.0, 0)), "misuse")
    LC("camera plug pull 20 N -y on the board", gr.force(B, (0, -20.0, 0)), "misuse")
    LC("board pinch 20 N down (hand)", gr.force(B, (0, 0, -20.0)), "misuse")
    LC("sensor press 20 N up", gr.force(LAND, (0, 0, 20.0)), "misuse")
    LC("sensor pull 10 N down", gr.force(LAND, (0, 0, -10.0)), "flight")
    for v in ((5, 0, 0), (0, 5, 0), (0, -5, 0), (0, 0, 5)):
        LC(f"poke 5 N nose tab {v}", gr.force(TAB_N, v), "misuse")
    # bare boss pokes (Radxa off): one front, one rear
    for k in (0, 3):
        for v in ((5, 0, 0), (0, 5, 0)):
            LC(f"poke 5 N boss {k} {v} (board off)", gr.force(boss[k], v), "misuse")
    # stiffness only (13 Oct, run g): run f's first mode was 92 Hz, a Radxa corner bouncing / the board pitching about the
    # two-bolt line (README: outside 80 to 350 Hz). Unit loads on each boss and a pitch and a roll couple, full weight
    for k in range(4):
        LC(f"stiff boss {k} z", gr.force(boss[k], (0, 0, 1.0)), None)
    F = sum(gr.force(boss[k], (0, 0, 1.0 if G.R_HOLES[k][0] > 0 else -1.0)) for k in range(4))
    LC("stiff pitch couple", F, None)
    F = sum(gr.force(boss[k], (0, 0, 1.0 if G.R_HOLES[k][1] > 0 else -1.0)) for k in range(4))
    LC("stiff roll couple", F, None)
    pb = PO.Problem(shape=gr.shape, h=h, origin=gr.origin, solid=solid, void=void, supports=supports, cases=cases,
                    material=PETG_BASIC, build="-z", overhang=False, R=max(2.0, R_MM / h), sym=None, comp_factor=None)
    return pb, gr, geo


if __name__ == "__main__":
    import time
    t = time.time(); pb, gr, geo = problem()
    des = ~pb.solid & ~pb.void
    print(f"shape {pb.shape}, h {pb.h}, solid {pb.solid.sum()} ({pb.solid.sum()*pb.h**3/1000:.2f} cm3), design {des.sum()} "
          f"({des.sum()*pb.h**3/1000:.1f} cm3), active {(~pb.void).sum()}, cases {len(pb.cases)}, {time.time()-t:.0f}s")
    bodies, small = NO.interface_bodies(pb, 1)
    print("drawn voxel bodies:", [len(b[1]) for b in bodies], "fragments", len(small))
    np.savez_compressed("masks.npz", solid=pb.solid, void=pb.void, shape=pb.shape, h=pb.h, origin=pb.origin)
