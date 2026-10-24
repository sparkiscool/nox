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


def piece_meshes(log=print, swap=None):
    """W meshes of every piece, regenerated from the part scripts (sha checked against print/) or mapped back from the
    print file (topology parts, topo.compare.old_in_w: drawn bodies fitted, 0 mm3 missing). swap = {piece key: W stl}
    puts a new version in its place (the piece pass's assembled recheck: same placement as the piece it replaces)."""
    out, proof = {}, {}
    swap = swap or {}
    for key, path in swap.items():
        out[key] = trimesh.load(path, force="mesh"); proof[key] = dict(source=f"swap {os.path.relpath(path, ROOT)}", sha=sha12(path))
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([STAND, HERE, NOX]))
    for key, (n, ver, tag, pf, qty, ax) in PIECES.items():
        if key in swap: continue
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


Z_SEAT = 142.3                                    # catch seat frame z 0 in W: panel contact face 82.7 + z (box_catch.assembly.json) on the pads' plane 84 + (Z - 143.6)
SEAT_TO_W = np.array([[0, -1.0, 0, 0], [1.0, 0, 0, Y_FUN], [0, 0, 1.0, Z_SEAT], [0, 0, 0, 1.0]])   # seat y -> W -X (the seated faces), seat x -> W Y


def catch_parts():
    """the box catch system's own assembly (funnel panels v7, hook latch v2 out), posed in W."""
    import assembly
    a = assembly.load(os.path.join(HERE, "box_catch.assembly.json"))
    out = {}
    ms = [(p, t) for p, t in zip(a.parts, a.meshes()) if not p["name"].startswith("drone")]
    T = SEAT_TO_W
    if min(t.bounds[0][2] for _, t in ms) > 100.0:                        # the file is in W at the v2 seat (R 113, Y 140): back to the v1 demo's
        T = np.eye(4); T[1, 3] = Y_FUN - 140.0; T[2, 3] = Z_SEAT - 155.3
    for p, t in ms:
        t = t.copy(); t.apply_transform(T); out["catch: " + p["name"]] = man(t)
    return out


def context():
    """name -> (owner, manifold, mass g, tag). Bought parts in pose and the other systems' parts as envelopes."""
    C = {}
    for n, b in SHEETS.items():
        m = S.box(*b); C[n] = ("box structure (bought, 3.0 acrylic DEFAULT)", m, m.volume() * 1.19e-3, "C")
    C["stub"] = ("swap arm (stub v3)", cyl_y(45, 0, 6) + cyl_y(25, 6, 42), 47.7, "C")
    C["arm_hub_sweep"] = ("swap arm (hub r 86, Y 7..74, turning)", cyl_y(86, 7, 74), 0.0, "C")
    C["arm_motor_sweep"] = ("swap arm (3 NEMA17 on the arm, r 106, Y 2..50, turning)", cyl_y(106.5, 2, 50) - cyl_y(86, 2, 50), 0.0, "C")
    C["arm_park_sweep"] = ("swap arm (heads + packs parked, r 98.5, Y 74..222, turning)", cyl_y(98.5, 74, 222) - cyl_y(86, 74, 74.5), 0.0, "C")
    for k, m in catch_parts().items(): C[k] = ("box catch (assembly file)", m, 0.0, "M")
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
    sweeps = [k for k in C if k.startswith("arm_") or k.startswith("collar") or k == "stub"]
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
        if g <= 0 or k == "stub": continue
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
            self._Kf = sp.triu(self.K[free][:, free] + sp.identity(len(free)) * 1e-4, format="csr"); self._ps = pypardiso.PyPardisoSolver(); self._ps.set_matrix_type(2)
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
P_LAND_M = (-60.0, Y_FUN - 30.0, 175.0)       # and its mirror (toward the -X, -Y pad)
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
        C[f"{dn} catch, 30 off toward +X +Y at 0.15 m/s ({F:.0f} N)"] = dict(cls="flight", acc=(0, 0, -1), drone=dn,
                                                              loads=[("funnel", (-0.25 * F, -0.25 * F, -F), (0, 0, 0), P_LAND)])
    F = e(DRONES["Overseer"])
    C[f"Overseer catch, 30 off toward -X -Y ({F:.0f} N)"] = dict(cls="flight", acc=(0, 0, -1), drone="Overseer",
                                                               loads=[("funnel", (0.25 * F, 0.25 * F, -F), (0, 0, 0), P_LAND_M)])
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
            if not pull.any() or (~pull).sum() < 30: break
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
        def rigid(rn):
            p, X = rnodes(rn); u = U[Mo.comp[p]]; c0 = X.mean(0); r = X - c0
            A = np.zeros((3 * len(X), 6)); A[:, :3] = np.tile(np.eye(3), (len(X), 1))
            for i in range(3):
                e = np.zeros(3); e[i] = 1.0; A[:, 3 + i] = np.cross(e[None], r).ravel()
            x = np.linalg.lstsq(A, u.ravel(), rcond=None)[0]
            return lambda P: x[:3] + np.cross(x[3:], np.asarray(P) - c0), x[3:]
        arm, th = rigid("stub")                                               # the arm rides the stub face as a rigid body
        fun, _ = rigid("funnel"); bay, _ = rigid("backplate@-90")
        dfl["stub rotation mrad"] = np.round(th * 1000, 3).tolist()
        dfl["arm tip at the drone minus funnel"] = np.round(arm(ARM_TIP(90, 140)) - fun(ARM_TIP(90, 140)), 3).tolist()
        dfl["arm tip at -90 minus bay -90"] = np.round(arm(ARM_TIP(-90, 140)) - bay(ARM_TIP(-90, 140)), 3).tolist()
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


# ======================================================================= piece rechecks (F)
H_FINE = dict(mast=2.0, kb=2.0, cradle_keel=2.0, bar_half=1.5, toe=1.5, frame_mid=1.5, frame_endR=1.5, frame_endL=1.5,
              post_seg=1.0, bracket_jA=1.0, bracket_jB=1.0, bracket_e=1.0, baymod=1.0)     # the stand FE grids (box-stand.md)
PRIOR = dict(mast=0.08, kb=0.09, cradle_keel=0.18, bar_half=0.38, toe=0.19, frame_mid=0.59, frame_endR=0.86, frame_endL=0.90,
             post_seg=0.16, bracket_jA=0.05, bracket_jB=0.09, bracket_e=0.14, baymod=0.05)   # each piece's own FE worst (box-stand.md, box-case.md)


def piece_model(t, hf, axis):
    from scipy import ndimage
    b0 = np.floor(t.bounds[0] / hf) * hf - hf; n = np.ceil((t.bounds[1] - b0) / hf).astype(int) + 1
    A = voxelise(t, *[b0[d] + (np.arange(n[d]) + 0.5) * hf for d in range(3)])
    lab, nl = ndimage.label(A); sizes = np.bincount(lab.ravel())[1:]
    keep = lab == (1 + int(np.argmax(sizes)))
    cells = np.argwhere(keep); L = np.array(LOCAL)
    g = ((cells[:, None, :] + L[None]) * np.array([(n[1] + 1) * (n[2] + 1), n[2] + 1, 1])).sum(-1)
    u, inv = np.unique(g, return_inverse=True); en = inv.reshape(g.shape)
    ijk = np.stack([u // ((n[1] + 1) * (n[2] + 1)), (u // (n[2] + 1)) % (n[1] + 1), u % (n[2] + 1)], 1)
    X = ijk * hf + b0
    C = stiffness(MATERIALS["PETG"], axis); KE, B0 = hex_ke(hf, C)
    dof = (en[:, :, None] * 3 + np.arange(3)).reshape(len(en), 24)
    K = sp.coo_matrix((np.tile(KE.ravel(), len(en)), (np.repeat(dof, 24, 1).ravel(), np.tile(dof, (1, 24)).ravel())),
                      shape=(3 * len(u),) * 2).tocsr()
    return dict(X=X, en=en, dof=dof, K=K, B0=B0, C=C, cells=cells, b0=b0, h=hf, bodies=int(nl), lost_mm3=float((A.sum() - keep.sum()) * hf ** 3))


def piece_pass(I, pin, log=print, only=None):
    import pypardiso
    from scipy.spatial import cKDTree
    hc = pin["h"]; out = {}
    for k, (pk, m) in I.items():
        if only and pk not in only and k not in only: continue
        t0 = time.time(); axis = PIECES[pk][5]; hf = H_FINE[pk]
        P = piece_model(tmesh(m), hf, axis); X = P["X"]; tree = cKDTree(X); nn = len(X)
        me = np.full(len(P["en"]), hf ** 3 * MATERIALS["PETG"].density * 1e-3)
        surf = np.bincount(P["en"].ravel(), minlength=nn) < 8                 # loads and holds act on the piece's surface
        def near(Xc):
            r = np.unique(np.concatenate([np.array(q, int) for q in tree.query_ball_point(Xc, hc * 1.01)] + [np.zeros(0, int)]))
            return r[surf[r]]
        js = [(ab, ab[0] == k) for ab in pin["joints"] if k in ab]
        jreg = {ab: near(pin["joint_nodes"][ab]) for ab, _ in js}
        mag = {ab: np.mean([np.linalg.norm(v[0]) for v in pin["joints"][ab].values()]) for ab, _ in js}
        held = max(mag, key=mag.get) if mag else None
        rows = {}
        for cname, c in pin["cases"].items():
            F = np.zeros((nn, 3)); hot = np.zeros(nn, bool)
            np.add.at(F, P["en"].ravel(), np.repeat(me[:, None] * c["acc"][None] / 8.0, 8, 0))
            if k in c["sup"]: fixn = near(c["sup"][k]); hj = None
            else: fixn = jreg[held]; hj = held
            for ab, isa in js:
                if ab == hj: continue
                Fa, Ma = pin["joints"][ab][cname]; sg = 1.0 if isa else -1.0
                r = jreg[ab]
                if len(r) == 0: continue
                F[r] += wrench(X[r], sg * Fa, sg * Ma, pin["jc"][ab]); hot[r] = True
            for (kk, rn), (Xc, Fe, Me, cc) in c["ext"].items():
                if kk != k: continue
                r = near(Xc); F[r] += wrench(X[r], Fe, Me, cc); hot[r] = True
            hot[fixn] = True
            fixed = np.zeros(nn, bool); fixed[fixn] = True
            free = np.where(~np.repeat(fixed, 3))[0]
            Kf = sp.triu(P["K"][free][:, free], format="csr"); ps = pypardiso.PyPardisoSolver(); ps.set_matrix_type(2)
            U = np.zeros(3 * nn); U[free] = ps.solve(Kf, F.ravel()[free]); ps.free_memory(everything=True)
            s = (U[P["dof"]] @ P["B0"].T) @ P["C"].T
            idx = failure(s, "PETG", c["cls"], axis)
            ok = ~hot[P["en"]].any(1)
            iw = np.where(ok, idx, 0); e = int(np.argmax(iw))
            Rf = -(F.sum(0))                                                  # what the held region must give
            mis = None
            if hj is not None:
                Fh = pin["joints"][hj][cname][0] * (1.0 if hj[0] == k else -1.0)
                mis = round(float(np.linalg.norm(Rf - Fh) / max(np.linalg.norm(Fh), 1.0)), 3)
            rows[cname] = dict(worst=round(float(iw[e]), 3), at=np.round(P["b0"] + (P["cells"][e] + 0.5) * hf, 1).tolist(),
                               p99=round(float(np.quantile(idx[ok], 0.99)), 3) if ok.any() else None, masked=round(float(1 - ok.mean()), 2),
                               cls=c["cls"], held="floor" if hj is None else " | ".join(hj),
                               held_mismatch=mis, umax_mm=round(float(np.abs(U).max()), 3))
        wc = max(rows, key=lambda q: rows[q]["worst"])
        out[k] = dict(piece=pk, h=hf, bodies_in_grid=P["bodies"], lost_mm3=round(P["lost_mm3"], 1), worst=rows[wc]["worst"], worst_case=wc,
                      at=rows[wc]["at"], prior=PRIOR[pk], cases=rows)
        log(f"piece {k:18s} worst {rows[wc]['worst']:.2f} in {wc} at {rows[wc]['at']} (own FE {PRIOR[pk]}) ({time.time() - t0:.0f} s)")
    return out


def export_requirements(I, pin, pieces, out_dir):
    """one JSON per piece instance: its joints (partner, coarse contact points W, F and M on this piece about the joint
    centroid per case), its external loads, its floor contact per case, the case class and acceleration. These are the
    piece's topo parameters (docs/generative.md The rule 0): loads = the joint reactions, supports = the neighbours."""
    os.makedirs(out_dir, exist_ok=True)
    for k, (pk, m) in I.items():
        js = {}
        for ab in pin["joints"]:
            if k not in ab: continue
            sg = 1.0 if ab[0] == k else -1.0; other = ab[1] if ab[0] == k else ab[0]
            js[other] = dict(points=np.round(pin["joint_nodes"][ab], 1).tolist(), centroid=np.round(pin["jc"][ab], 2).tolist(),
                             cases={cn: dict(F_N=np.round(sg * v[0], 2).tolist(), M_Nmm=np.round(sg * v[1], 1).tolist())
                                    for cn, v in pin["joints"][ab].items()})
        cs = {}
        for cn, c in pin["cases"].items():
            ext = {rn: dict(points=np.round(v[0], 1).tolist(), F_N=np.round(v[1], 2).tolist(), M_Nmm=np.round(v[2], 1).tolist(),
                            centroid=np.round(v[3], 2).tolist()) for (kk, rn), v in c["ext"].items() if kk == k}
            cs[cn] = dict(cls=c["cls"], acc_N_per_g=np.round(c["acc"], 6).tolist(), external=ext,
                          floor_points=np.round(c["sup"][k], 1).tolist() if k in c["sup"] else None)
        r = dict(instance=k, piece=pk, version=PIECES[pk][1], file=f"print/{PIECES[pk][3]}", build_axis="XYZ"[PIECES[pk][5]],
                 system_h=pin["h"], note="W coordinates (box-arm.md); forces act ON this piece; F x M sign: right hand; "
                 "index target <= 1.0 at the case class (topo.material SAFETY)", assembled=pieces.get(k), joints=js, cases=cs)
        json.dump(r, open(os.path.join(out_dir, f"{k}.json"), "w"), indent=0, default=float)


def report(h=4.0):
    """runs/box_system/v1/summary.md from the json results."""
    fm = json.load(open(os.path.join(OUT, "fit_mass.json"))); fe = json.load(open(os.path.join(OUT, f"fe_h{h:g}.json")))
    pp = json.load(open(os.path.join(OUT, f"pieces_h{h:g}.json")))
    L = ["# Box structure system pass v1 (`nox/box/box_system.py`)", "",
         f"System FE h {h:g} (F): {fe['cells']} cells, {fe['dofs']} dofs; pieces at their own grids (box-stand.md FE grids). "
         "Index 1.0 = allowable at the case class (sustained SF 10, flight = functional SF 4, misuse SF 2.5).", "",
         "## Fit (C / M)", "", "| Pair | mm3 |", "|---|---|"]
    for k, v in {**fm["fit"]["piece_and_bought_overlaps_mm3"], **fm["fit"]["keepout_overlaps_mm3"]}.items(): L.append(f"| {k} | {v} |")
    m = fm["mass"]
    L += ["", f"## Mass and CG (M pieces, E bought / lumps)", "", f"Box {m['total_g'] / 1000:.2f} kg, CG {m['cg']}; "
          f"with the Scout {m['with Scout']['total_g'] / 1000:.2f} kg, CG {m['with Scout']['cg']}, tip {m['with Scout']['tip_deg']}; "
          f"with the Overseer {m['with Overseer']['total_g'] / 1000:.2f} kg, CG {m['with Overseer']['cg']}, tip {m['with Overseer']['tip_deg']} (deg).", "",
          "## Assembled worst index per piece (piece FE with the system's joint reactions)", "",
          "| Instance | Piece | Own FE worst | Assembled worst | Case | At (W) |", "|---|---|---|---|---|---|"]
    for k, v in sorted(pp.items(), key=lambda kv: -kv[1]["worst"]):
        L.append(f"| {k} | {v['piece']} | {v['prior']} | **{v['worst']:.2f}** | {v['worst_case']} | {v['at']} |")
    L += ["", "## Per case: worst piece, interface deflections (F)", "", "| Case | Class | Worst piece | Index | Stub face mm | Bay -90 minus stub mm | Funnel pads minus stub mm |", "|---|---|---|---|---|---|---|"]
    for cn, c in fe["cases"].items():
        wk = max(pp, key=lambda k: pp[k]["cases"][cn]["worst"])
        d = c["deflection_mm"]
        L.append(f"| {cn} | {c['cls']} | {wk} | {pp[wk]['cases'][cn]['worst']:.2f} | {d['stub']} | {d['bay -90 minus stub']} | {d['funnel minus stub']} |")
    L += ["", "## Concept check (docs/generative.md rule 0)", "",
          f"- System line (`python3 nox/assembly.py nox/box/box_structure.assembly.json --check`): {fm.get('system_check')}; "
          "crash probes skipped (DEFAULT: floor-standing box, the floor shields the open bottom; alt: probes on read -34.0 mm (E) at "
          "-Z on the PSU). Its 5080 g counts the structure, sheets and PSU only (the arm, catch and lumps are in the mass table above).",
          "- Concept review: the `review` agent on PR 73. Layout artifact: with the v2 layout (the v1 layout is the built demo).",
          "- Alternative A (DEFAULT): box structure v2 on the 9-bay ring at R 113 (arm v6 ask): Overseer bays fit the 30 deg sector, "
          "every ring and base piece is redrawn once from v2 requirements; costs +13 mm on every radius (sheets ~+26 wide, floor -13).",
          "- Alternative B: keep R 100, the Overseer cartridge lowered 13 (latch / Overseer sessions): keeps the stand envelopes, "
          "but box-case Problem 1 (Overseer cartridge at r ~107 in a 30 deg sector at r 100) is still open and arm v6 reads FAIL.",
          "- Index targets: 0.5 functional / 1.0 misuse (as System Box catch, DEFAULT) vs 1.0 everywhere: the strict set fails mast, "
          "cradle_keel, bracket_jA and the bay module in addition to kb and the frames."]
    open(os.path.join(OUT, "summary.md"), "w").write("\n".join(L) + "\n")


TARGET = 0.8                                   # assembled worst index a new piece must reach (DEFAULT: 1.0 rule less 20 % for the coarse system FE)


def requirements_md(h=4.0):
    """runs/box_system/v1/requirements.md: per piece type its envelope, joints with the largest reaction over the cases,
    the assembled worst index now and the target. Full per-case reactions: req/<instance>.json."""
    pp = json.load(open(os.path.join(OUT, f"pieces_h{h:g}.json")))
    L = ["| Piece | Instances | Envelope W (now, C) | Joints: partner, centroid W, max abs F N / M N.m (case) | Assembled worst (case) | Target |", "|---|---|---|---|---|---|"]
    by = {}
    for k, v in pp.items(): by.setdefault(v["piece"], []).append(k)
    for pk, ks in by.items():
        js, bbs = [], []
        for k in ks:
            r = json.load(open(os.path.join(OUT, "req", f"{k}.json")))
            for other, j in r["joints"].items():
                cF = max(j["cases"], key=lambda c: np.linalg.norm(j["cases"][c]["F_N"]))
                js.append((np.linalg.norm(j["cases"][cF]["F_N"]), f"{other} ({', '.join(f'{x:.0f}' for x in j['centroid'])}) "
                           f"{np.linalg.norm(j['cases'][cF]['F_N']):.0f} / {max(np.linalg.norm(c['M_Nmm']) for c in j['cases'].values()) / 1000:.1f} ({cF})", k))
        js.sort(key=lambda t: -t[0])
        wk = max(ks, key=lambda k: pp[k]["worst"])
        top = "; ".join(f"{k}: {t}" for _, t, k in js[:4])
        L.append(f"| {pk} {PIECES[pk][1]} | {', '.join(ks)} | see req/{ks[0]}.json | {top} | {pp[wk]['worst']:.2f} on {wk} ({pp[wk]['worst_case']}) | <= {TARGET} |")
    open(os.path.join(OUT, "requirements.md"), "w").write("\n".join(L) + "\n")


ASM = os.path.join(HERE, "box_structure.assembly.json")


def write_assembly(I, M, proof):
    """nox/box/box_structure.assembly.json (nox/ASSEMBLY.md): every structure piece as its W mesh + pose, sheets and
    PSU as bought boxes, the bay-swap access paths. The catch (box_catch.assembly.json) and the arm
    (box_swap_arm.assembly.json) keep their own files; box_system.py loads them as context."""
    import assembly
    rel = lambda p: os.path.relpath(p, ROOT)
    parts = []
    for k, (pk, m) in I.items():
        n, ver, tag, pf, qty, ax = PIECES[pk]
        src = os.path.join(HERE, "models", "box_baymod_v4i_W.obj") if tag == "obj" else os.path.join(CACHE, f"box_{n}_{ver}_W.stl")
        if k.startswith("baymod@"):
            T = trimesh.transformations.rotation_matrix(math.radians(90 - int(k.split("@")[1])), [0, 1, 0])
        else:
            a = np.asarray(man(M[pk]).to_mesh().vert_properties)[:, :3]; b = np.asarray(m.to_mesh().vert_properties)[:, :3]
            ca, cb = a.mean(0), b.mean(0); U, _, Vt = np.linalg.svd((a - ca).T @ (b - cb))
            R = Vt.T @ np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T
            T = np.eye(4); T[:3, :3] = R; T[:3, 3] = cb - R @ ca
        parts.append(dict(name=k, role="structure", mesh=rel(src), pose=np.round(T, 6).tolist(), material="PETG", tag="M",
                          owner="box structure", version=ver, source=f"nox/box/box_system.py ({proof[pk]['source']})",
                          note=f"print/{pf}"))
    for n_, b in SHEETS.items():
        parts.append(dict(name=n_, role="cage", box=list(b), material="acrylic", mass_g=round(S.box(*b).volume() * 1.19e-3, 1),
                          tag="C", owner="box structure", version="3.0 acrylic (DEFAULT)", source="stand_v7_parts.py X_IN / Y_BACK / Y_FRONT / Z_TOP"))
    parts.append(dict(name="PSU LRS-350-24", role="bought", protect=True, box=[42, 157, -66, -36, -170, 45], mass_g=750.0, tag="E",
                      owner="box electronics", version="v1", source="dimensions.md I Box electronics v1"))
    access = [dict(name=f"bay swap {a}: module out along +Y 100", part=f"baymod@{a}", dir=[0, 1, 0], length_mm=100, tool="part")
              for a in BAYS]
    access.append(dict(name="stub M5 key from the arm side", part="mast", **{"from": [10.0, 0.5, 30.0]}, dir=[0, 1, 0], length_mm=60,
                       tool={"type": "cylinder", "d": 8}, ignore=["sheetF"]))
    A = assembly.new("Box structure", "box", parts=parts, note=(
        "Box structure system v1 (24 Oct, box_system.py), the built demo: stand, 2 bay modules v4i (-90, -60), 5 mid + 2 end "
        "frames, enclosure. W frame (box-arm.md). Left out: the swap arm (box_swap_arm.assembly.json, its owner), the catch "
        "(box_catch.assembly.json, posed in W by box_system.SEAT_TO_W), bolts and nuts, the electronics boards (lumped in "
        "box_system.LUMPS). The 4 sheets are the cage (role cage) round the PSU; the top is the funnel mouth. Crash probes off: a "
        "floor-standing box (DEFAULT; the floor shields the open bottom)."), fit_tol_mm3=0.5,
        access=access, contact_pairs=[[f"post-{e}{k}", s_, 2.0] for e in "FR" for k in range(3) for s_ in ("sheetF" if e == "F" else "sheetR", "sheet-X")]
        + [[f"post+{e}{k}", s_, 2.0] for e in "FR" for k in range(3) for s_ in ("sheetF" if e == "F" else "sheetR", "sheet+X")]
        + [["cradle_keel", "baymod@-90", 0.5]], probes=False)
    A.save(ASM)
    return A


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--h", type=float, default=4.0); ap.add_argument("--only", default="all")
    ap.add_argument("--pieces", default="")
    ap.add_argument("--swap", default="", help="piece=W.stl,... : a new version in the assembled model")
    ap.add_argument("--out", default="", help="results folder (default runs/box_system/v1)")
    a = ap.parse_args()
    global OUT
    if a.out: OUT = os.path.abspath(a.out); os.makedirs(OUT, exist_ok=True)
    lg = open(os.path.join(OUT, "log.txt"), "a")
    def log(s): print(s, flush=True); lg.write(s + "\n"); lg.flush()
    log(f"== box_system {time.strftime('%Y-%m-%d %H:%M')} h {a.h}")
    M, proof = piece_meshes(log, swap=dict(kv.split("=") for kv in a.swap.split(",") if "=" in kv)); I = instances(M); C = context()
    out = dict(pieces=proof)
    if a.only in ("all", "fit"):
        out["fit"] = fit_check(I, C, log); out["mass"] = mass_cg(I, C, M)
        A = write_assembly(I, M, proof); chk = A.check(); out["system_check"] = chk.get("summary")
        json.dump(out, open(os.path.join(OUT, "fit_mass.json"), "w"), indent=1, default=float); log(json.dumps(out["fit"])); log(str(chk.get("summary")))
    if a.only in ("all", "fe"):
        seed = None
        if a.h < 6.0:
            r6, _ = run_fe(I, 6.0, log); json.dump(r6, open(os.path.join(OUT, "fe_h6.json"), "w"), indent=1, default=float)
            import pickle; seed = pickle.load(open(os.path.join(OUT, "fe_h6_piece_inputs.pkl"), "rb"))
        res, Mo = run_fe(I, a.h, log, seed=seed, max_it=10)
        json.dump(res, open(os.path.join(OUT, f"fe_h{a.h:g}.json"), "w"), indent=1, default=float)
    if a.only in ("all", "pieces"):
        import pickle
        pin = pickle.load(open(os.path.join(OUT, f"fe_h{a.h:g}_piece_inputs.pkl"), "rb"))
        pp = piece_pass(I, pin, log, only=set(a.pieces.split(",")) - {""} or None)
        fn = os.path.join(OUT, f"pieces_h{a.h:g}.json")
        old = json.load(open(fn)) if os.path.exists(fn) and a.pieces else {}
        old.update(pp); json.dump(old, open(fn, "w"), indent=1, default=float)
        export_requirements(I, pin, old, os.path.join(OUT, "req"))
    if a.only in ("all", "report"): report(a.h); requirements_md(a.h)


if __name__ == "__main__":
    main()
