"""A part as the designer gives it: exact bodies (manifold3d) plus supports and load cases on regions.

  drawn = mounting / must-have features (passive solid), cuts = holes and keep-outs (passive void),
  env   = the region generated material may occupy (the drawn bodies are added to it automatically).

The grid is centred on the envelope's bounding box (mirror planes of a symmetric part fall on a cell boundary, so the
voxel masks come out mirrored) and sized to a multiple of `mult` cells per axis so the multigrid solver can coarsen.
Same call pattern as the old `arm_topo.PartProblem` (support / load / accel_case), so existing part setups work.
"""
from dataclasses import dataclass
import numpy as np


@dataclass
class LoadCase:
    name: str
    F: np.ndarray                 # (ndof,) nodal forces, N
    cls: str = "misuse"           # allowable class for the strength report; None = stiffness only
    support: str = "default"
    accel: np.ndarray = None      # (3,) inertial load in g on the part's own material (density-weighted body force)
    nodes: np.ndarray = None      # loaded node ids (for the strength report's exclusion set)


def tm(m):
    import trimesh
    me = m.to_mesh()
    return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=True)


def man(t):
    import manifold3d as mf
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32),
                               tri_verts=np.asarray(t.faces, np.uint32)))


def voxelise(mesh, xs, ys, zs):
    """inside test of a closed trimesh at the grid points xs x ys x zs (cell centres): ray parity along +z, one ray
    per (x, y) column. Returns bool (len xs, len ys, len zs)."""
    XX, YY = np.meshgrid(xs, ys, indexing="ij")
    org = np.stack([XX.ravel(), YY.ravel(), np.full(XX.size, zs[0] - 5.0)], 1)
    # a tiny irrational offset keeps rays off mesh edges / vertices that sit exactly on grid lines
    org[:, 0] += 1.234e-5
    org[:, 1] += 2.345e-5
    A = np.zeros((len(xs) * len(ys), len(zs)), bool)
    if len(mesh.faces) == 0:
        return A.reshape(len(xs), len(ys), len(zs))
    loc, ray, _ = mesh.ray.intersects_location(org, np.tile([0, 0, 1.0], (len(org), 1)), multiple_hits=True)
    if len(ray):
        order = np.argsort(ray, kind="stable")
        ray = ray[order]
        z = loc[order][:, 2]
        st = np.r_[0, np.where(np.diff(ray))[0] + 1, len(ray)]
        for a, b in zip(st[:-1], st[1:]):
            zz = np.sort(z[a:b])
            zz = zz[np.r_[True, np.diff(zz) > 1e-6]]
            for p, q in zip(zz[0::2], zz[1::2]):
                zq = zs + 3.4e-5                                  # a centre exactly on a face counts the same way always
                A[ray[a], (zq > p) & (zq < q)] = True
    return A.reshape(len(xs), len(ys), len(zs))


class Part:
    def __init__(self, name, h, drawn, cuts, env, build="+z", overhang=False, material="PETG", pad=1, mult=8,
                 log=print):
        self.name, self.h, self.build, self.material, self.log = name, float(h), build, material, log
        self.overhang = overhang
        env = env + drawn
        self.drawn_m, self.cuts_m, self.env_m = drawn, cuts, env
        bb = np.array(env.bounding_box(), float)
        lo, hi = bb[:3], bb[3:]
        n = np.ceil((hi - lo) / h).astype(int) + 2 * pad
        n = (np.ceil(n / mult) * mult).astype(int)
        c = 0.5 * (lo + hi)
        self.shape = tuple(int(v) for v in n)
        self.origin = tuple(c - n * h / 2.0)
        self.axes = [self.origin[k] + (np.arange(self.shape[k]) + 0.5) * h for k in range(3)]
        self.nel = int(np.prod(self.shape))
        nx, ny, nz = self.shape
        self.nshape = (nx + 1, ny + 1, nz + 1)
        self.nn = int(np.prod(self.nshape))
        self.ndof = 3 * self.nn
        self.env_t, self.drawn_t, self.cuts_t = tm(env), tm(drawn), tm(cuts)
        inside_env = self.vox(self.env_t)
        inside_cut = self.vox(self.cuts_t)
        inside_drawn = self.vox(self.drawn_t)
        self.solid = (inside_drawn & ~inside_cut).ravel()
        self.void = ((~inside_env) | inside_cut).ravel() & ~self.solid
        self.design = ~self.solid & ~self.void
        self.supports = {}            # name -> bool (nn, 3) fixed dofs
        self.support_cells = {}       # name -> element ids the support region covers
        self.cases = []
        log(f"{name}: grid {self.shape} h {h} = {self.nel:,} cells; design {int(self.design.sum()):,} "
            f"({self.design.sum() * h ** 3 / 1000:.1f} cm3), drawn {int(self.solid.sum()):,}, void {int(self.void.sum()):,}")

    # ---------------------------------------------------------------------------------------------- regions
    def vox(self, t):
        return voxelise(t, *self.axes)

    def elems(self, region, within=None):
        m = self.vox(tm(region)).ravel()
        if within is not None:
            m &= within
        return np.where(m)[0]

    def nodes_of(self, el):
        nx, ny, nz = self.shape
        ex, ey, ez = np.unravel_index(el, self.shape)
        nid = lambda i, j, k: (i * (ny + 1) + j) * (nz + 1) + k
        return np.unique(np.concatenate([nid(ex + a, ey + b, ez + c) for a in (0, 1) for b in (0, 1) for c in (0, 1)]))

    # ------------------------------------------------------------------------------------- supports and loads
    def support(self, name, region):
        el = self.elems(region, self.solid | self.design)
        if len(el) == 0:
            raise ValueError(f"support {name}: the region holds no part cells")
        nd = self.nodes_of(el)
        fx = np.zeros((self.nn, 3), bool)
        fx[nd] = True
        self.supports[name] = fx
        self.support_cells[name] = el
        self.log(f"  support {name}: {len(el)} cells, {len(nd)} nodes")

    def load(self, name, region, vec, cls="misuse", support="default", accel=None):
        el = self.elems(region, self.solid | self.design)
        if len(el) == 0:
            raise ValueError(f"load {name}: empty region")
        nd = self.nodes_of(el)
        F = np.zeros(self.ndof)
        for a in range(3):
            F[3 * nd + a] = vec[a] / len(nd)
        self.cases.append(LoadCase(name, F, cls=cls, support=support,
                                   accel=None if accel is None else np.asarray(accel, float), nodes=nd))

    def load_multi(self, name, parts, cls="misuse", support="default", accel=None):
        """one load case made of several (region, vector) pieces, e.g. a torque as two opposite tangential pushes."""
        F = np.zeros(self.ndof)
        alln = []
        for region, vec in parts:
            el = self.elems(region, self.solid | self.design)
            if len(el) == 0:
                raise ValueError(f"load {name}: empty region")
            nd = self.nodes_of(el)
            for a in range(3):
                F[3 * nd + a] += vec[a] / len(nd)
            alln.append(nd)
        self.cases.append(LoadCase(name, F, cls=cls, support=support,
                                   accel=None if accel is None else np.asarray(accel, float),
                                   nodes=np.unique(np.concatenate(alln))))

    def accel_case(self, name, g, cls="misuse", support="default"):
        self.cases.append(LoadCase(name, np.zeros(self.ndof), cls=cls, support=support, accel=np.asarray(g, float),
                                   nodes=np.zeros(0, int)))

    # ------------------------------------------------------------------------------------------------ misc
    def cell_centres(self):
        X, Y, Z = np.meshgrid(*self.axes, indexing="ij")
        return np.stack([X.ravel(), Y.ravel(), Z.ravel()], 1)

    def summary(self):
        return dict(name=self.name, h=self.h, shape=list(self.shape), origin=[round(o, 4) for o in self.origin],
                    cells=self.nel, design=int(self.design.sum()), drawn=int(self.solid.sum()),
                    cases=[c.name for c in self.cases], supports=list(self.supports))
