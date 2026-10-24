"""Box structure system model (Spark 24 Oct "System first, then cut"; docs/generative.md The rule, 0).

One whole-box model in W (box-arm.md: X across, Y along the axle from the stub face, Z up, floor Z -201): every
structure piece from its exact print file (stand, ring frames, enclosure, bay modules), the swap arm and catch
assemblies as context (their meshes or envelopes), bought parts in pose (sheets, PSU, motors' sweep, stub).
  1. fit: exact overlaps between every pair of placed bodies (manifold), arm sweep and collar keep-outs
  2. mass and CG of the whole box, tip-over angles
  3. system FE: piece-labelled voxel model of the assembled structure (each piece keeps its own nodes; pieces join
     only where their surfaces touch within one sample, ring joints only at the lug + dovetail band r >= 150, where
     they are bolted), the system cases below, joint reactions per piece and case, stiffness at the interfaces
  4. piece rechecks: each piece alone at its own grid with the assembled joint reactions as its loads (largest joint
     held), worst index per case (topo.material allowables, layer-aware, class SF per case)
python3 box_system.py [--h 4] [--fine 1.5] [--only fit|fe|pieces]  ->  runs/box_system/v1/
Tags: C geometry, M mesh, E estimate, F this frame / voxel model."""
import os, sys, json, math, time, hashlib, subprocess, argparse
import numpy as np, trimesh, manifold3d as mf, scipy.sparse as sp

HERE = os.path.dirname(os.path.abspath(__file__))
NOX = os.path.abspath(os.path.join(HERE, ".."))
ROOT = os.path.abspath(os.path.join(NOX, ".."))
STAND = os.path.join(HERE, "stand")
for p in (STAND, HERE, NOX):
    if p not in sys.path: sys.path.insert(0, p)
os.environ.setdefault("PYPARDISO_MKL_RT", "/usr/local/lib/libmkl_rt.so.3")
import stand_v6_parts as S, stand_v7_parts as V
from topo.part import voxelise
from topo.fem import hex_ke, LOCAL
from topo.material import MATERIALS, stiffness, allowables, SAFETY
OUT = os.path.join(ROOT, "runs", "box_system", "v1"); os.makedirs(OUT, exist_ok=True)
CACHE = os.path.join(OUT, "W"); os.makedirs(CACHE, exist_ok=True)
FLOOR = S.FLOOR; G = 9.81e-3                                                     # N per g (mass in g)

# ======================================================================= pieces (exact print files -> W)
# name: (part, version, script tag, print file, qty, build axis in W (print pose), material)
PIECES = {
    "mast": ("mast", "v6", "v6", "box_mast_v6.stl", 1, 2),
    "cradle_keel": ("cradle_keel", "v6", "v6", "box_cradle_keel_v6.stl", 1, 1),
    "frame_mid": ("frame_mid", "v6", "v6", "box_frame_mid_v6.stl", 5, 0),
    "kb": ("kb", "v7b", "v7b", "box_kb_v7b.stl", 1, 2),
    "toe": ("toe", "v7b", "v7b", "box_toe_v7b.stl", 1, 2),
    "frame_endR": ("frame_endR", "v7b", "v7b", "box_frame_endR_v7b.stl", 1, 0),
    "frame_endL": ("frame_endL", "v7b", "v7b", "box_frame_endL_v7b.stl", 1, 0),
    "post_seg": ("post_seg", "v7", "v7", "box_post_seg_v7.stl", 12, 0),
    "bracket_jB": ("bracket_jB", "v7", "v7", "box_bracket_jB_v7.stl", 2, 0),
    "bracket_e": ("bracket_e", "v7c", "v7c", "box_bracket_e_v7c.stl", 2, 0),
    "bar_half": ("bar_half", "v8", "topo", "box_bar_half_v8.stl", 2, 1),
    "bracket_jA": ("bracket_jA", "v8", "topo", "box_bracket_jA_v8.stl", 2, 0),
    "baymod": ("baymod", "v4i", "obj", "box_baymod_v4i.stl", 2, 1),
}


def sha12(p):
    return hashlib.sha256(open(p, "rb").read()).hexdigest()[:12]


def piece_meshes(log=print):
    """W meshes of every piece, regenerated from the part scripts (sha checked against print/) or mapped back from the
    print file (topology parts, topo.compare.old_in_w: drawn bodies fitted, 0 mm3 missing)."""
    out, proof = {}, {}
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([STAND, HERE, NOX]))
    for key, (n, ver, tag, pf, qty, ax) in PIECES.items():
        w = os.path.join(CACHE, f"box_{n}_{ver}_W.stl")
        pr = os.path.join(ROOT, "print", pf)
        if tag == "obj":
            t = trimesh.load(os.path.join(HERE, "models", "box_baymod_v4i_W.obj"), force="mesh")
            p = trimesh.load(pr, force="mesh")
            proof[key] = dict(source="models/box_baymod_v4i_W.obj", print_g=round(p.volume * 1.27e-3, 1),
                              W_g=round(t.volume * 1.27e-3, 1), same_volume=bool(abs(p.volume - t.volume) < 1.0))
            out[key] = t; continue
        if not os.path.exists(w):
            if tag == "topo":
                import noxopt  # noqa: F401  (points pypardiso at pip's MKL)
                from topo.compare import old_in_w
                from topo.demos import PARTS
                spec = PARTS[n](); part = spec["setup"](spec["h"])
                Wm, fit = old_in_w(n, part, pr, log=lambda s: None)
                Wm.export(w); json.dump(fit, open(w + ".fit.json", "w"))
            else:
                script = "stand_v6_parts.py" if tag == "v6" else "stand_v7_parts.py"
                subprocess.run([sys.executable, os.path.join(STAND, script), n], cwd=CACHE, env=dict(env, VTAG=tag),
                               check=True, stdout=subprocess.DEVNULL)
        t = trimesh.load(w, force="mesh")
        if tag == "topo":
            proof[key] = dict(source=f"print/{pf} mapped to W", fit=json.load(open(w + ".fit.json")), sha=sha12(pr))
        else:
            gen = os.path.join(CACHE, pf)
            proof[key] = dict(source=f"{'stand_v6_parts' if tag == 'v6' else 'stand_v7_parts'} VTAG={tag}", sha=sha12(pr),
                              generated_sha=sha12(gen), exact=bool(sha12(gen) == sha12(pr)))
        out[key] = t
    return out, proof


def man(t):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))


def tmesh(m):
    g = m.to_mesh(); return trimesh.Trimesh(np.asarray(g.vert_properties)[:, :3], np.asarray(g.tri_verts), process=True)


def rot_y(m, deg):
    return m.rotate([0, deg, 0])


# ======================================================================= the assembled box
BAYS = S.BAYS                                     # built bay modules at -90 / -60 (box-case.md)
Y_C = 175.0                                       # tab line of the built demo stack (v4i modules, arm v5 on the posts)
Y_FUN = 177.0                                     # funnel collar centre on the end frames' pads (box-stand.md)


def instances(M):
    """every structure piece placed, exactly as stand_v7_check.py places them. name -> (piece key, manifold)."""
    I = {}
    I["mast"] = ("mast", man(M["mast"])); I["cradle_keel"] = ("cradle_keel", man(M["cradle_keel"]))
    I["kb"] = ("kb", man(M["kb"])); I["toe"] = ("toe", man(M["toe"]))
    ba = man(M["bar_half"]); I["barA"] = ("bar_half", ba); I["barB"] = ("bar_half", ba.rotate([0, 0, 180]).translate([0, 2 * 54.0, 0]))
    fm = man(M["frame_mid"])
    for p in S.POS["frame_mid"]: I[f"frame_mid@{p}"] = ("frame_mid", S.place(fm, p))
    I["frame_endR@30"] = ("frame_endR", S.place(man(M["frame_endR"]), 30))
    I["frame_endL@-210"] = ("frame_endL", S.place(man(M["frame_endL"]), -210))
    ps = man(M["post_seg"])
    for c in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
        for k in range(3):
            I[f"post{'+' if c[0] > 0 else '-'}{'F' if c[1] > 0 else 'R'}{k}"] = ("post_seg", V.post_place(ps, c).translate([0, 0, FLOOR + k * V.SEG]))
    jA, jB, be = man(M["bracket_jA"]), man(M["bracket_jB"]), man(M["bracket_e"])
    I["bracket_jA@-15"] = ("bracket_jA", jA); I["bracket_jA@165"] = ("bracket_jA", V.bracket_place("bracket_jA", jA, "L"))
    I["bracket_jB@15"] = ("bracket_jB", jB); I["bracket_jB@195"] = ("bracket_jB", V.bracket_place("bracket_jB", jB, "L"))
    I["bracket_e@45"] = ("bracket_e", be); I["bracket_e@135"] = ("bracket_e", V.bracket_place("bracket_e", be, "L"))
    mod = M["baymod"]
    for pb in BAYS:
        t = mod.copy(); t.apply_transform(trimesh.transformations.rotation_matrix(math.radians(90 - pb), [0, 1, 0]))
        I[f"baymod@{pb}"] = ("baymod", man(t))
    return I


X_IN, YB, YF, ZT, TS = V.X_IN, V.Y_BACK, V.Y_FRONT, V.Z_TOP, V.T_SHEET
SHEETS = {"sheet+X": (X_IN, X_IN + TS, YB + 3, YF - 3, FLOOR, ZT), "sheet-X": (-X_IN - TS, -X_IN, YB + 3, YF - 3, FLOOR, ZT),
          "sheetF": (-X_IN + 3, X_IN - 3, YF, YF + TS, FLOOR, ZT), "sheetR": (-X_IN + 3, X_IN - 3, YB - TS, YB, FLOOR, ZT)}


def cyl_y(r, y0, y1, x=0.0, z=0.0, n=64):
    return mf.Manifold.cylinder(y1 - y0, r, r, n).rotate([-90, 0, 0]).translate([x, y0, z])


Z_EDGE, Z_RIM = 143.6, 219.5                     # collar inner face: bottom edge Z 143.6 on the pads' plane (C), 75.9 tall (G Funnel)


def collar(off0, off1):
    """funnel collar layer between off0 and off1 (horizontal, behind the inner face |u| = 84 + (Z - 143.6)), Z 143.6..
    219.5, centred X 0, Y 177. Skin 0..4.4 = the face plane the pads seat on (pad seat 37.4 + 0.3 + 4.4 = 42.1 = the
    face plane's offset from the 45 deg end plane, C); ribs 4.4..15.0 (panel v6: skin 1.6 + rib 9 normal, box-funnel.md)."""
    def pyr(off, z0=Z_EDGE, z1=Z_RIM):
        h0, h1 = 84 + off + (z0 - Z_EDGE), 84 + off + (z1 - Z_EDGE)
        pts = [[-h0, Y_FUN - h0, z0], [h0, Y_FUN - h0, z0], [h0, Y_FUN + h0, z0], [-h0, Y_FUN + h0, z0],
               [-h1, Y_FUN - h1, z1], [h1, Y_FUN - h1, z1], [h1, Y_FUN + h1, z1], [-h1, Y_FUN + h1, z1]]
        return mf.Manifold.hull_points(pts)
    return pyr(off1) - pyr(off0, Z_EDGE - 1, Z_RIM + 1)


def context():
    """name -> (owner, manifold, mass g, tag). Bought parts in pose and the other systems' parts as envelopes."""
    C = {}
    for n, b in SHEETS.items():
        m = S.box(*b); C[n] = ("box structure (bought, 3.0 acrylic DEFAULT)", m, m.volume() * 1.19e-3, "C")
    C["stub"] = ("swap arm (stub v3)", cyl_y(45, 0, 6) + cyl_y(25, 6, 42), 47.7, "C")
    C["arm_hub_sweep"] = ("swap arm (hub r 86, Y 7..74, turning)", cyl_y(86, 7, 74), 0.0, "C")
    C["arm_motor_sweep"] = ("swap arm (3 NEMA17 on the arm, r 106, Y 2..50, turning)", cyl_y(106.5, 2, 50) - cyl_y(86, 2, 50), 0.0, "C")
    C["arm_park_sweep"] = ("swap arm (heads + packs parked, r 98.5, Y 74..222, turning)", cyl_y(98.5, 74, 222) - cyl_y(86, 74, 74.5), 0.0, "C")
    C["collar_skin"] = ("box catch (funnel collar skin)", collar(0.0, 4.4), 475.0, "E")
    C["collar_ribs"] = ("box catch (funnel panel v6 ribs, 9 deep, where drawn)", collar(4.4, 15.0), 0.0, "E")
    C["psu"] = ("box electronics (LRS-350-24 on the rear sheet)", S.box(42, 157, -66, -36, -170, 45), 750.0, "E")
    return C


# ======================================================================= masses (E unless M / C)
LUMPS = {   # name: (g, point W, what) applied in every case at the case's acceleration
    "arm": (2900.0, (0.0, 60.0, 0.0), "arm v5 (hub 215 M, rails 100 M, heads 112 M, 3 NEMA17 1200, 2 x 6010 420, servos 120, 2 packs 400, rest 330, E) on the stub"),
    "funnel": (640.0, (0.0, Y_FUN, 180.0), "collar 475 + hook latch set 110 M + 4 MG90S 52 (E) on the 4 pads"),
    "electronics": (1250.0, (100.0, -50.0, -60.0), "charger, Pi, relays, boards (E) on the rear sheet zone (the PSU is its own body)"),
    "bay_packs": (440.0, (0.0, Y_C, -125.0), "2 loaded Scout cartridges, 220 each (E), in the -90 / -60 bays"),
}
DRONES = {"Scout": 520.0, "Overseer": 2330.0}     # g (E: Scout v3 ~500 g brief + cage; Overseer cage v0 2.33 kg)


def fit_check(I, C, log=print):
    names = list(I) + list(C)
    body = {k: I[k][1] for k in I}; body.update({k: C[k][1] for k in C})
    bb = {k: np.array(body[k].bounding_box()).reshape(2, 3) for k in names}
    sweeps = [k for k in C if k.startswith("arm_") or k.startswith("collar")]
    ov, sw = {}, {}
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            A, B = bb[a], bb[b]
            if not ((A[0] < B[1]).all() and (B[0] < A[1]).all()): continue
            if a in sweeps and b in sweeps: continue                                  # arm / collar envelopes vs each other: theirs
            v = (body[a] ^ body[b]).volume()
            if v <= 0.01: continue
            key = f"{a} ^ {b}"
            if a in sweeps or b in sweeps: sw[key] = round(v, 1)
            else: ov[key] = round(v, 2)
    return dict(piece_and_bought_overlaps_mm3=ov, keepout_overlaps_mm3=sw)


def mass_cg(I, C, M):
    rows, mt, mc = {}, 0.0, np.zeros(3)
    for k, (pk, m) in I.items():
        g = m.volume() * MATERIALS["PETG"].density * 1e-3
        cen = np.array(tmesh(m).center_mass); rows[k] = round(g, 1); mt += g; mc += g * cen
    for k, (own, m, g, tag) in C.items():
        if g <= 0 or k in ("collar_skin", "stub"): continue
        cen = np.array(tmesh(m).center_mass); rows[k] = round(g, 1); mt += g; mc += g * cen
    for k, (g, p, _) in LUMPS.items():
        rows[k] = g; mt += g; mc += g * np.array(p)
    cg = mc / mt
    out = dict(rows_g=rows, total_g=round(mt, 0), cg=np.round(cg, 1).tolist())
    for dn, g in DRONES.items():
        p = np.array([0.0, Y_FUN, 144.5 + 40.0]); c2 = (mc + g * p) / (mt + g)
        h = c2[2] - FLOOR
        out[f"with {dn}"] = dict(total_g=round(mt + g, 0), cg=np.round(c2, 1).tolist(),
                                 tip_deg={"+X": round(math.degrees(math.atan2(X_IN + TS - c2[0], h)), 1),
                                          "-X": round(math.degrees(math.atan2(X_IN + TS + c2[0], h)), 1),
                                          "+Y": round(math.degrees(math.atan2(YF + TS - c2[1], h)), 1),
                                          "-Y": round(math.degrees(math.atan2(c2[1] - (YB - TS), h)), 1)})
    return out


# ======================================================================= system FE (F)
ACRYLIC = dict(E=3200.0, nu=0.37, density=1.19, Xt=60.0)          # cast acrylic (E): sheets
GRID_O = np.array([-192.0, -72.0, FLOOR])                          # floor = a node plane
GRID_MAX = np.array([192.0, 362.0, 246.0])
RING = lambda k: k.startswith("frame_") or k.startswith("baymod")


def iso_C(E, nu):
    l = E * nu / ((1 + nu) * (1 - 2 * nu)); m = E / (2 * (1 + nu))
    C = np.zeros((6, 6)); C[:3, :3] = l; C[[0, 1, 2], [0, 1, 2]] += 2 * m; C[[3, 4, 5], [3, 4, 5]] = m
    return C


class Model:
    """piece-labelled voxel model: every instance has its own cells (volume fraction from 3 x 3 x 3 samples) and nodes;
    two instances share a node only where their surfaces come within one sample of each other (bolted / seated faces),
    ring neighbours only at r >= 150 (lug pairs r 165..179, dovetail r 157: the joint planes are 0.6 apart elsewhere)."""

    def __init__(self, bodies, h, log=print):
        self.h, self.hs, self.log = h, h / 3.0, log
        self.n = np.ceil((GRID_MAX - GRID_O) / h).astype(int)                 # cells per axis
        self.ns = self.n * 3                                                   # samples per axis
        self.names = list(bodies)
        self.mat = {}; self.axis = {}; self.cells = {}; self.frac = {}; self.samp = {}
        t0 = time.time()
        for k, (tmsh, mat, axis) in bodies.items():
            self._vox(k, tmsh); self.mat[k] = mat; self.axis[k] = axis
        log(f"voxels: {sum(len(c) for c in self.cells.values())} cells over {len(bodies)} bodies at h {h} ({time.time() - t0:.0f} s)")
        self._nodes(); self._joints(); log(f"nodes: {self.nP} piece nodes -> {self.nN} after joins, {len(self.pairs)} joints ({time.time() - t0:.0f} s)")

    def _vox(self, k, t):
        h, hs = self.h, self.hs
        c0 = np.maximum(np.floor((t.bounds[0] - GRID_O) / h).astype(int), 0)
        c1 = np.minimum(np.ceil((t.bounds[1] - GRID_O) / h).astype(int), self.n)
        ax = [GRID_O[d] + (np.arange(c0[d] * 3, c1[d] * 3) + 0.5) * hs for d in range(3)]
        A = voxelise(t, *ax)
        m = (c1 - c0)
        fr = A.reshape(m[0], 3, m[1], 3, m[2], 3).mean((1, 3, 5))
        idx = np.argwhere(fr > 0)
        self.cells[k] = idx + c0; self.frac[k] = fr[fr > 0]
        si = np.argwhere(A) + c0 * 3
        self.samp[k] = np.sort((si[:, 0] * self.ns[1] + si[:, 1]) * self.ns[2] + si[:, 2])

    def gnode(self, ijk):
        return (ijk[..., 0] * (self.n[1] + 1) + ijk[..., 1]) * (self.n[2] + 1) + ijk[..., 2]

    def gxyz(self, g):
        nz, ny = self.n[2] + 1, self.n[1] + 1
        k = g % nz; j = (g // nz) % ny; i = g // (nz * ny)
        return np.stack([i, j, k], -1) * self.h + GRID_O

    def _nodes(self):
        L = np.array(LOCAL)
        self.en = {}; self.pnodes = {}; self.poff = {}; off = 0
        for k in self.names:
            g = self.gnode(self.cells[k][:, None, :] + L[None])                # (nel, 8) global grid node ids
            u, inv = np.unique(g, return_inverse=True)
            self.pnodes[k] = u; self.poff[k] = off; self.en[k] = inv.reshape(g.shape) + off; off += len(u)
        self.nP = off

    def pid(self, k, g):
        """piece-node ids of instance k at grid nodes g (-1 where k has no node)."""
        u = self.pnodes[k]; i = np.searchsorted(u, g); i = np.minimum(i, len(u) - 1)
        return np.where(u[i] == g, i + self.poff[k], -1)

    def _joints(self):
        ns = self.ns; d = np.array([(a * ns[1] + b) * ns[2] + c for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1)])
        bb = {}
        for k in self.names:
            c = self.cells[k]; bb[k] = (c.min(0) - 1, c.max(0) + 2)
        L = np.array(LOCAL)
        ea, eb = [], []; self.pairs = {}
        for i, a in enumerate(self.names):
            for b in self.names[i + 1:]:
                if not ((bb[a][0] < bb[b][1]).all() and (bb[b][0] < bb[a][1]).all()): continue
                sa, sb = self.samp[a], self.samp[b]
                if len(sa) > len(sb): sa, sb = sb, sa
                dil = np.unique((sa[:, None] + d[None]).ravel())
                hit = sb[np.isin(sb, dil, assume_unique=False)]
                if len(hit) == 0: continue
                si = np.stack([hit // (ns[1] * ns[2]), (hit // ns[2]) % ns[1], hit % ns[2]], 1)
                if RING(a) and RING(b):
                    xyz = GRID_O + (si + 0.5) * self.hs
                    si = si[np.hypot(xyz[:, 0], xyz[:, 2]) >= 150.0]
                    if len(si) == 0: continue
                g = np.unique(self.gnode((si // 3)[:, None, :] + L[None]).ravel())
                pa, pb = self.pid(a, g), self.pid(b, g); ok = (pa >= 0) & (pb >= 0)
                if ok.sum() == 0: continue
                ea.append(pa[ok]); eb.append(pb[ok]); self.pairs[(a, b)] = g[ok]
        from scipy.sparse.csgraph import connected_components
        ea = np.concatenate(ea); eb = np.concatenate(eb)
        Gm = sp.coo_matrix((np.ones(len(ea)), (ea, eb)), shape=(self.nP, self.nP))
        self.nN, self.comp = connected_components(Gm, directed=False)
        self.pg = np.empty(self.nP, np.int64)                                  # piece node -> grid node
        for k in self.names: self.pg[self.poff[k]: self.poff[k] + len(self.pnodes[k])] = self.pnodes[k]

    def xyz_p(self, p):
        return self.gxyz(self.pg[p])

    # ---------------------------------------------------------------- stiffness
    def Cmat(self, k):
        m = self.mat[k]
        return iso_C(ACRYLIC["E"], ACRYLIC["nu"]) if m == "acrylic" else stiffness(MATERIALS["PETG"], self.axis[k])

    def assemble(self):
        t0 = time.time(); K = None
        self.KE = {}
        for k in self.names:
            C = self.Cmat(k); KE, B0 = hex_ke(self.h, C); self.KE[k] = (KE, B0, C)
            ed = self.comp[self.en[k]]                                         # (nel, 8) model nodes
            dof = (ed[:, :, None] * 3 + np.arange(3)[None, None]).reshape(len(ed), 24)
            v = (self.frac[k][:, None, None] * KE[None]).ravel()
            Kk = sp.coo_matrix((v, (np.repeat(dof, 24, 1).ravel(), np.tile(dof, (1, 24)).ravel())), shape=(3 * self.nN,) * 2).tocsr()
            K = Kk if K is None else K + Kk
        self.K = K; self.log(f"K: {K.shape[0]} dofs, {K.nnz / 1e6:.1f} M nonzeros ({time.time() - t0:.0f} s)")

    def masses(self, k):
        rho = (ACRYLIC["density"] if self.mat[k] == "acrylic" else MATERIALS["PETG"].density) * 1e-3   # g / mm3
        return self.frac[k] * self.h ** 3 * rho

    def solve(self, Fp, fixed_p):
        """Fp: (nP, 3) external forces on piece nodes; fixed_p: piece nodes held (xyz). Returns U on model nodes."""
        import pypardiso
        F = np.zeros((self.nN, 3)); np.add.at(F, self.comp, Fp)
        fixed = np.zeros(self.nN, bool); fixed[self.comp[fixed_p]] = True
        free = np.where(~np.repeat(fixed, 3))[0]
        key = fixed.tobytes().__hash__()
        if getattr(self, "_fkey", None) != key:
            if getattr(self, "_ps", None) is not None: self._ps.free_memory(everything=True)
            self._Kf = sp.triu(self.K[free][:, free], format="csr"); self._ps = pypardiso.PyPardisoSolver(); self._ps.set_matrix_type(2)
            self._fkey = key; self._free = free
        U = np.zeros(3 * self.nN); U[free] = self._ps.solve(self._Kf, F.ravel()[free])
        return U.reshape(-1, 3)

    def internal(self, U):
        """per piece node internal force K_piece u (nP, 3)."""
        f = np.zeros((self.nP, 3))
        for k in self.names:
            KE = self.KE[k][0]; ed = self.comp[self.en[k]]
            ue = U[ed].reshape(len(ed), 24)
            fe = (ue @ KE.T) * self.frac[k][:, None]
            np.add.at(f, self.en[k].ravel(), fe.reshape(-1, 3))
        return f

    def stress(self, k, U):
        KE, B0, C = self.KE[k]; ed = self.comp[self.en[k]]
        return (U[ed].reshape(len(ed), 24) @ B0.T) @ C.T                       # material stress (strain x full C)


def failure(s, mat, cls, axis):
    if mat == "acrylic":
        sxx, syy, szz, sxy, syz, szx = s.T
        vm = np.sqrt(0.5 * ((sxx - syy) ** 2 + (syy - szz) ** 2 + (szz - sxx) ** 2) + 3 * (sxy ** 2 + syz ** 2 + szx ** 2))
        return vm / (ACRYLIC["Xt"] * 0.85 / SAFETY[cls])
    Axy, Az, As = allowables(MATERIALS["PETG"], cls)
    sxx, syy, szz, sxy, syz, szx = s.T
    vm = np.sqrt(0.5 * ((sxx - syy) ** 2 + (syy - szz) ** 2 + (szz - sxx) ** 2) + 3 * (sxy ** 2 + syz ** 2 + szx ** 2))
    sn = s[:, axis]; pairs = {3: (0, 1), 4: (1, 2), 5: (2, 0)}
    tau = np.sqrt(sum(s[:, q] ** 2 for q, (i, j) in pairs.items() if axis in (i, j)))
    return np.maximum(vm / Axy, np.sqrt((np.maximum(sn, 0) / Az) ** 2 + (tau / As) ** 2))


def wrench(X, F, M=(0, 0, 0), P=None):
    """nodal forces (n, 3) on points X equivalent to force F at P plus moment M (rigid distribution)."""
    X = np.asarray(X, float); F = np.asarray(F, float); M = np.asarray(M, float)
    c = X.mean(0); P = c if P is None else np.asarray(P, float)
    Mc = M + np.cross(P - c, F); r = X - c
    J = (r * r).sum() * np.eye(3) - r.T @ r
    w = np.linalg.lstsq(J, Mc, rcond=None)[0]
    return F[None] / len(X) + np.cross(w[None], r)


# ======================================================================= system cases (E unless tagged; DEFAULT + log)
def E_catch(m_g, v=0.15, drop=0.030):
    """impact energy, J: descent v (G Capture: <= 0.15 m/s) plus the roll down the 45 deg face from first touch to the
    seat (offset 30 = the landing gate, G)."""
    m = m_g * 1e-3; return 0.5 * m * v * v + m * 9.81 * drop


STOP = 3.0                                    # mm: impact force = energy / 3 mm (box-stand.md weak-spot 4: Scout drop 64 N per pad)
P_LAND = (60.0, Y_FUN + 30.0, 175.0)          # the drone comes in 30 off on the diagonal (toward the +X, +Y pad), E
ARM_TIP = lambda ang, r=140.0: np.array([r * math.cos(math.radians(ang)), Y_C, r * math.sin(math.radians(ang))])


def cases():
    """name: dict(cls, acc (g, W), drone, loads [(region, F N, M N.mm, P)], support). Classes: sustained SF 10, flight
    (functional) SF 4, misuse SF 2.5 (topo.material)."""
    e = lambda m, v=0.15, d=0.030: E_catch(m, v, d) * 1000 / STOP                # N
    up = np.array([0, 0, 1.0])
    C = {}
    C["standing, Scout seated"] = dict(cls="sustained", acc=(0, 0, -1), drone="Scout", loads=[])
    C["standing, Overseer seated"] = dict(cls="sustained", acc=(0, 0, -1), drone="Overseer", loads=[])
    C["uneven floor (front-right corner off), Overseer seated"] = dict(cls="sustained", acc=(0, 0, -1), drone="Overseer", loads=[], support="uneven")
    for dn in ("Scout", "Overseer"):
        F = e(DRONES[dn])
        C[f"{dn} catch, 30 off at 0.15 m/s ({F:.0f} N)"] = dict(cls="flight", acc=(0, 0, -1), drone=dn,
                                                              loads=[("funnel", (-0.25 * F, -0.25 * F, -F), (0, 0, 0), P_LAND)])
    F = e(DRONES["Scout"], 0.0, 0.150)
    C[f"misuse: Scout free fall 0.15 m ({F:.0f} N)"] = dict(cls="misuse", acc=(0, 0, -1), drone="Scout",
                                                          loads=[("funnel", (-0.25 * F, -0.25 * F, -F), (0, 0, 0), P_LAND)])
    F = e(DRONES["Overseer"], 0.15, 0.040)
    C[f"misuse: Overseer motors cut at the gate ({F:.0f} N)"] = dict(cls="misuse", acc=(0, 0, -1), drone="Overseer",
                                                                    loads=[("funnel", (-0.25 * F, -0.25 * F, -F), (0, 0, 0), P_LAND)])
    C["XT60 insert at the drone 60 N"] = dict(cls="flight", acc=(0, 0, -1), drone="Scout",
                                              loads=[("funnel", 60 * up, (0, 0, 0), ARM_TIP(90, 144.5)), ("stub", -60 * up, (0, 0, 0), ARM_TIP(90))])
    C["XT60 pull at the drone 50 N"] = dict(cls="flight", acc=(0, 0, -1), drone="Scout",
                                            loads=[("funnel", -50 * up, (0, 0, 0), ARM_TIP(90, 144.5)), ("stub", 50 * up, (0, 0, 0), ARM_TIP(90))])
    for ang in (-90, -60):
        d = ARM_TIP(ang, 1.0) - np.array([0, Y_C, 0])
        C[f"XT60 insert at the {ang} bay 60 N"] = dict(cls="flight", acc=(0, 0, -1), drone="Scout",
                                                     loads=[(f"backplate@{ang}", 60 * d, (0, 0, 0), ARM_TIP(ang, 150)), ("stub", -60 * d, (0, 0, 0), ARM_TIP(ang))])
    d = ARM_TIP(-90, 1.0) - np.array([0, Y_C, 0])
    C["XT60 pull at the -90 bay 50 N"] = dict(cls="flight", acc=(0, 0, -1), drone="Scout",
                                              loads=[("receiver@-90", -50 * d, (0, 0, 0), ARM_TIP(-90, 120)), ("stub", 50 * d, (0, 0, 0), ARM_TIP(-90))])
    C["axle motor stall 1.8 N.m"] = dict(cls="flight", acc=(0, 0, -1), drone="Scout", loads=[("stub", (0, 0, 0), (0, 1800.0, 0), None)])
    C["misuse: 50 N sideways on the funnel rim"] = dict(cls="misuse", acc=(0, 0, -1), drone="Scout",
                                                       loads=[("funnel", (50.0, 0, 0), (0, 0, 0), (-160.0, Y_FUN, Z_RIM))])
    C["lifted by the side sheets, 2 g"] = dict(cls="flight", acc=(0, 0, -2), drone=None, loads=[], support="hands")
    C["misuse: set down hard, 5 g"] = dict(cls="misuse", acc=(0, 0, -5), drone=None, loads=[])
    C["misuse: knocked over onto the +X side, 5 g lateral"] = dict(cls="misuse", acc=(5, 0, 0), drone=None, loads=[], support="side")
    return C


def regions(Mo, I):
    """piece-node sets (instance, piece node ids) where the system loads act."""
    R = {}
    def sel(k, f):
        p = np.arange(Mo.poff[k], Mo.poff[k] + len(Mo.pnodes[k])); X = Mo.xyz_p(p); return p[f(X)]
    ym = Mo.xyz_p(np.arange(Mo.poff["mast"], Mo.poff["mast"] + len(Mo.pnodes["mast"])))[:, 1].max()
    R["stub"] = [("mast", sel("mast", lambda X: (X[:, 1] > ym - 0.1) & (np.hypot(X[:, 0], X[:, 2]) <= 45)))]
    pads = []
    for k, s in (("frame_endR@30", -1), ("frame_endL@-210", 1)):
        t = (lambda X, s=s: (X[:, 2] - X[:, 0]) / math.sqrt(2)) if s < 0 else (lambda X: (X[:, 2] + X[:, 0]) / math.sqrt(2))
        pads.append((k, sel(k, lambda X, t=t: t(X) > 33.0)))
    R["funnel"] = pads
    for ang in BAYS:
        k = f"baymod@{ang}"
        def ang_of(X): return np.degrees(np.arctan2(X[:, 2], X[:, 0]))
        R[f"backplate@{ang}"] = [(k, sel(k, lambda X: (np.hypot(X[:, 0], X[:, 2]) > 147) & (np.abs(((ang_of(X) - ang + 180) % 360) - 180) < 10) & (X[:, 1] > 150) & (X[:, 1] < 200)))]
        R[f"receiver@{ang}"] = [(k, sel(k, lambda X: (np.hypot(X[:, 0], X[:, 2]) > 100) & (np.hypot(X[:, 0], X[:, 2]) < 145) & (np.abs(((ang_of(X) - ang + 180) % 360) - 180) < 12)))]
    R["electronics"] = [("sheetR", sel("sheetR", lambda X: (X[:, 0] > 42) & (X[:, 0] < 157) & (X[:, 2] > -170) & (X[:, 2] < 45)))]
    R["bay_packs"] = [(f"baymod@{a}", sel(f"baymod@{a}", lambda X: (np.hypot(X[:, 0], X[:, 2]) > 100) & (np.hypot(X[:, 0], X[:, 2]) < 150))) for a in BAYS]
    for k, v in R.items():
        for inst, p in v:
            if len(p) == 0: raise RuntimeError(f"region {k} on {inst}: no nodes")
    return R


def supports(Mo, kind):
    g = Mo.pg; X = Mo.gxyz(g)
    floor = np.abs(X[:, 2] - FLOOR) < 1e-6
    if kind == "uneven":
        return np.where(floor & ~((X[:, 0] > 0) & (X[:, 1] > (YB + YF) / 2)))[0], np.array([0, 0, 1.0])
    p = np.arange(Mo.nP)
    if kind == "hands":
        sh = np.zeros(Mo.nP, bool)
        for k in ("sheet+X", "sheet-X"):
            q = np.arange(Mo.poff[k], Mo.poff[k] + len(Mo.pnodes[k])); Xq = Mo.xyz_p(q)
            sh[q[(Xq[:, 1] > 100) & (Xq[:, 1] < 250) & (Xq[:, 2] > 0) & (Xq[:, 2] < 150) & (np.abs(Xq[:, 0]) > X_IN + TS - 0.1 - Mo.h)]] = True
        return p[sh], None
    if kind == "side":
        q = np.arange(Mo.poff["sheet+X"], Mo.poff["sheet+X"] + len(Mo.pnodes["sheet+X"])); Xq = Mo.xyz_p(q)
        return q[Xq[:, 0] >= Xq[:, 0].max() - 1e-6], np.array([-1.0, 0, 0])
    return np.where(floor)[0], np.array([0, 0, 1.0])


def run_fe(I, h=4.0, log=print, seed=None, max_it=30):
    bodies = {}
    for k, (pk, m) in I.items():
        bodies[k] = (tmesh(m), "PETG", PIECES[pk][5])
    for n, b in SHEETS.items():
        bodies[n] = (tmesh(S.box(*b)), "acrylic", 0)
    Mo = Model(bodies, h, log=log); Mo.assemble()
    R = regions(Mo, I)
    def rnodes(name):
        p = np.concatenate([q for _, q in R[name]]); return p, Mo.xyz_p(p)
    CS = cases()
    pc = np.zeros(Mo.nP)                                                       # partner count per piece node (joint split)
    for (a, b), g in Mo.pairs.items():
        pc[Mo.pid(a, g)] += 1; pc[Mo.pid(b, g)] += 1
    res = dict(h=h, cells=int(sum(len(c) for c in Mo.cells.values())), dofs=int(3 * Mo.nN), cases={})
    joints = {}                                                                # (a, b) -> {case: (F on a, M on a about c)}
    jc = {ab: Mo.xyz_p(Mo.pid(ab[0], g)).mean(0) for ab, g in Mo.pairs.items()}
    pin = dict(h=h, cases={}, joint_nodes={ab: Mo.xyz_p(Mo.pid(ab[0], g)) for ab, g in Mo.pairs.items()}, jc=jc)
    for cname, c in CS.items():
        t0 = time.time()
        acc = np.array(c["acc"], float) * G
        Fp = np.zeros((Mo.nP, 3))
        for k in Mo.names:                                                     # self weight
            me = Mo.masses(k); np.add.at(Fp, Mo.en[k].ravel(), np.repeat(me[:, None] * acc[None] / 8.0, 8, 0))
        lump = {"stub": [(LUMPS["arm"][0] * acc, LUMPS["arm"][1])],
                "funnel": [(LUMPS["funnel"][0] * acc, LUMPS["funnel"][1])],
                "electronics": [((LUMPS["electronics"][0] + 750.0) * acc, LUMPS["electronics"][1])],
                "bay_packs": [(LUMPS["bay_packs"][0] * acc, (32.0, Y_C, -120.0))]}
        if c["drone"]: lump["funnel"].append((DRONES[c["drone"]] * acc, (0.0, Y_FUN, 184.5)))
        hot = np.zeros(Mo.nP, bool); ext = {}
        def addf(rn, p, f):
            Fp[p] += f
            for k, q in R[rn]:
                m_ = np.isin(p, q)
                if not m_.any(): continue
                X = Mo.xyz_p(p[m_]); cc = X.mean(0); ff = f[m_]
                o = ext.get((k, rn))
                F0, M0 = (np.zeros(3), np.zeros(3)) if o is None else (o[1], o[2] - np.cross(o[3] - cc, o[1]))
                ext[(k, rn)] = (X, F0 + ff.sum(0), M0 + np.cross(X - cc, ff).sum(0), cc)
        for rn, lst in lump.items():
            p, X = rnodes(rn)
            for F, P in lst: addf(rn, p, wrench(X, F, (0, 0, 0), P))
        for rn, F, Mm, P in c["loads"]:
            p, X = rnodes(rn); addf(rn, p, wrench(X, F, Mm, P)); hot[p] = True
        fx, nrm = supports(Mo, c.get("support", "floor"))
        if seed is not None and nrm is not None:                                # start from a coarser run's contact set
            from scipy.spatial import cKDTree
            keep = np.zeros(len(fx), bool); Xf = Mo.xyz_p(fx)
            for k, Xs in seed["cases"][cname]["sup"].items():
                q = (fx >= Mo.poff[k]) & (fx < Mo.poff[k] + len(Mo.pnodes[k]))
                if q.any(): keep[q] = cKDTree(Xs).query(Xf[q], distance_upper_bound=seed["h"] * 1.01)[0] < np.inf
            fx = fx[keep]
        for it in range(max_it):                                                    # unilateral floor: release nodes in tension
            U = Mo.solve(Fp, fx)
            fj = Mo.internal(U) - Fp
            if nrm is None: break
            Rm = np.zeros((Mo.nN, 3)); np.add.at(Rm, Mo.comp, fj)
            rn = Rm[Mo.comp[fx]] @ nrm                                          # support push along its normal (K u - F = reaction)
            cm = np.zeros(Mo.nN); np.add.at(cm, Mo.comp[fx], 1)
            pull = rn / cm[Mo.comp[fx]] < -0.01
            if not pull.any(): break
            fx = fx[~pull]
        hot[fx] = True
        rows = {}
        for k in Mo.names:
            s = Mo.stress(k, U); idx = failure(s, Mo.mat[k], c["cls"], Mo.axis[k])
            ok = (Mo.frac[k] >= 0.3) & ~hot[Mo.en[k]].any(1)
            if ok.sum() == 0: ok = Mo.frac[k] >= 0
            iw = np.where(ok, idx, 0); e = int(np.argmax(iw))
            rows[k] = dict(worst=round(float(iw[e]), 3), at=np.round(GRID_O + (Mo.cells[k][e] + 0.5) * h, 0).tolist(),
                           p99=round(float(np.quantile(idx[ok], 0.99)), 3))
        for ab, g in Mo.pairs.items():
            pa = Mo.pid(ab[0], g); X = Mo.xyz_p(pa); f = fj[pa] / np.maximum(pc[pa], 1)[:, None]
            Fa = f.sum(0); Ma = np.cross(X - jc[ab], f).sum(0)
            joints.setdefault(ab, {})[cname] = (Fa, Ma)
        reac, sup = {}, {}
        for k in Mo.names:
            q = np.arange(Mo.poff[k], Mo.poff[k] + len(Mo.pnodes[k])); q = q[np.isin(q, fx)]
            if len(q): reac[k] = np.round(fj[q].sum(0), 1).tolist(); sup[k] = Mo.xyz_p(q)
        pin["cases"][cname] = dict(cls=c["cls"], acc=acc, ext=ext, sup=sup, bilateral=nrm is None)
        dfl = {rn: np.round(U[Mo.comp[rnodes(rn)[0]]].mean(0), 3).tolist() for rn in ("stub", "funnel", "backplate@-90", "backplate@-60")}
        dfl["bay -90 minus stub"] = np.round(np.array(dfl["backplate@-90"]) - np.array(dfl["stub"]), 3).tolist()
        dfl["funnel minus stub"] = np.round(np.array(dfl["funnel"]) - np.array(dfl["stub"]), 3).tolist()
        res["cases"][cname] = dict(cls=c["cls"], rows=rows, support_reactions_N=reac, deflection_mm=dfl, contact_iterations=it + 1,
                                   umax_mm=round(float(np.abs(U).max()), 3))
        wk = max(rows, key=lambda k: rows[k]["worst"])
        log(f"{cname:58s} worst {rows[wk]['worst']:.2f} {wk}  stub {dfl['stub']}  ({time.time() - t0:.0f} s)")
    res["joints"] = {f"{a} | {b}": dict(centroid=np.round(jc[(a, b)], 1).tolist(), nodes=int(len(g)),
                                       cases={cn: dict(F=np.round(F, 1).tolist(), M_Nmm=np.round(M, 0).tolist()) for cn, (F, M) in joints[(a, b)].items()})
                     for (a, b), g in Mo.pairs.items()}
    pin["joints"] = joints
    import pickle
    pickle.dump(pin, open(os.path.join(OUT, f"fe_h{h:g}_piece_inputs.pkl"), "wb"))
    return res, Mo


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--h", type=float, default=4.0); ap.add_argument("--only", default="all")
    a = ap.parse_args()
    lg = open(os.path.join(OUT, "log.txt"), "a")
    def log(s): print(s, flush=True); lg.write(s + "\n"); lg.flush()
    log(f"== box_system {time.strftime('%Y-%m-%d %H:%M')} h {a.h}")
    M, proof = piece_meshes(log); I = instances(M); C = context()
    out = dict(pieces=proof)
    if a.only in ("all", "fit"):
        out["fit"] = fit_check(I, C, log); out["mass"] = mass_cg(I, C, M)
        json.dump(out, open(os.path.join(OUT, "fit_mass.json"), "w"), indent=1, default=float); log(json.dumps(out["fit"]))
    if a.only in ("all", "fe"):
        seed = None
        if a.h < 6.0:
            r6, _ = run_fe(I, 6.0, log); json.dump(r6, open(os.path.join(OUT, "fe_h6.json"), "w"), indent=1, default=float)
            import pickle; seed = pickle.load(open(os.path.join(OUT, "fe_h6_piece_inputs.pkl"), "rb"))
        res, Mo = run_fe(I, a.h, log, seed=seed, max_it=10)
        json.dump(res, open(os.path.join(OUT, f"fe_h{a.h:g}.json"), "w"), indent=1, default=float)


if __name__ == "__main__":
    main()
