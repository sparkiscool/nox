r"""reconstruct: P2 section 8 as one shared tool (P0 build item B7). Density + exact interface bodies -> one printable body.

    final = ((generated surface + exact preserve bodies, untrimmed) ^ envelope, trimmed ONCE) - exact obstacles

Steps (P2 s8, numbers in brackets):
  [8.1] upsample the physical density `up` x (trilinear), light Gaussian blur, and choose the iso value that keeps the
        optimised generated volume (sum of rho over the free cells), not a fixed 0.5.
  [8.2] carry the density one cell past every face that is trimmed exactly afterwards (cells outside the envelope or
        inside an exact obstacle), so the exact trims cut through material. Void cells that NO exact body trims are
        left empty and reported (they would otherwise grow material that nothing removes).
        Preserve cells are set just above the iso value, so the generated skin sits just inside the drawn faces.
  [8.6] root fillets: grey closing of the field with a ball of radius `fillet` (default 1.0 mm), applied only within
        2 x fillet of a preserve body and never in void cells (adds material in the concave corners where generated
        material meets a preserve body). Done on the field before the surface is made, so it is smooth.
  [8.3] marching cubes on the field (skimage), orient the whole surface once, Taubin smoothing with every vertex within
        `smooth_band` (0.5 mm) of a preserve or obstacle body put back where it was.
  [8.4] union with the exact preserve bodies, untrimmed (interfaces.py already pushes faces on the envelope 0.5 out),
        then ONE intersection with the envelope.
  [8.5] subtract the exact obstacles (interfaces.py obstacles already overlap 0.3 where cuts meet).
  [8.7] manifold3d booleans with merge tolerance 1e-3 mm; no simplification; closed voids filled and loose bits dropped
        (largest outward shell kept, both reported); a weld (0.8 mm cubes at non-manifold edges, inside the envelope,
        obstacles re-subtracted) only as the last resort and logged as WELD.
  [8.8] rotate into print orientation once (build direction -> +z, part on z = 0, min corner at the origin), export
        STL + 3MF, then RELOAD both files and check: watertight, winding consistent, one body, volume unchanged.

What a part must provide (everything else is generic):
  * the density: `Density(rho, shape, h, origin, solid, free)`. rho is the PHYSICAL density (PrintOpt.physical(x, beta)
    at the final beta, or `rho_f` in the older npz files); `solid` / `free` the Problem masks (free = ~solid & ~void).
    `Density.from_problem(op, x)` and `Density.from_npz(path)` build it.
  * `preserve`: exact preserve bodies (interfaces.preserve_of(features) + any part-specific drawn bodies), UNTRIMMED,
    faces on the envelope pushed 0.5 outside it.
  * `obstacle`: every exact later cut (interfaces.obstacle_of(features) + part cuts: cavities, slots, keep-outs). Every
    void region of the Problem inside the envelope must be covered by one, or it is reported.
  * `envelope`: the outer envelope solid (trimmed against once).
  * `build`: the printopt build direction ('+z', '-x', ...).
  Bodies may be CadQuery shapes / Workplanes, trimesh meshes, manifold3d Manifolds, STL paths or lists of these.
  The density grid must cover the envelope (two cells of margin are added automatically).

    res = reconstruct(Density.from_problem(op, x), preserve, obstacle, envelope, build="+z",
                      out_dir=r"%TEMP%\noxcheck\run", stem="scout_widget_v1")
    res.ok, res.stl, res.threemf, res.transform (part -> print frame, 4x4), res.log_lines

Outputs (out_dir): <stem>.stl and <stem>.3mf (print orientation), <stem>_assembled.stl (part frame, for fit suites),
<stem>_build.log (Rules stamp + every step), <stem>_transform.json (4x4 part -> print). Units mm.
No layer height is used (nothing is snapped to layers here; the slicer profile, 0.16 mm "Optimal", is Spark's).
Volume check in the log: final body vs exact target = (exact preserve inside the envelope - obstacles) + optimised
generated volume, plus the root fillets (test block: -0.9 %).
"""
import json
import os
import sys
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import trimesh
import manifold3d as mf
from scipy import ndimage

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))

MERGE_TOL = 1e-3          # P2 s8.7
NUDGE = np.array([0.0123, 0.0087, 0.0071])   # keep marching-cubes grid lines off whole / half mm (drawn faces)
PAD = 2                   # coarse cells added round the grid: 1 edge-carried, 1 empty, so the surface is closed


def _tmpdir():
    d = Path(os.environ.get("TEMP", tempfile.gettempdir())) / "noxcheck" / "recon_tmp"
    d.mkdir(parents=True, exist_ok=True)
    return d


# ----------------------------------------------------------------------------------------------- mesh conversion
def tm2mf(m):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(m.vertices, np.float32),
                               tri_verts=np.asarray(m.faces, np.uint32)))


def mf2tm(M):
    m = M.to_mesh()
    return trimesh.Trimesh(np.asarray(m.vert_properties)[:, :3].astype(float), np.asarray(m.tri_verts), process=False)


def as_trimesh(obj, tol=0.01):
    """CadQuery shape / Workplane, trimesh, Manifold, STL path, or a list of them -> one trimesh (None -> None)."""
    if obj is None:
        return None
    if isinstance(obj, (list, tuple)):
        parts = [as_trimesh(o, tol) for o in obj if o is not None]
        parts = [p for p in parts if p is not None and len(p.faces)]
        if not parts:
            return None
        M = tm2mf(parts[0])
        for p in parts[1:]:
            M = M + tm2mf(p)
        return mf2tm(M)
    if isinstance(obj, (str, Path)):
        return trimesh.load(str(obj), force="mesh")
    if isinstance(obj, trimesh.Trimesh):
        return obj.copy()
    if isinstance(obj, mf.Manifold):
        return mf2tm(obj)
    import cadquery as cq
    fn = _tmpdir() / f"b_{os.getpid()}_{id(obj)}_{time.time_ns()}.stl"
    shape = obj
    if isinstance(obj, cq.Workplane):
        vals = [v for v in obj.vals() if isinstance(v, cq.Shape)]
        shape = vals[0] if len(vals) == 1 else cq.Compound.makeCompound(vals)
    cq.exporters.export(shape, str(fn), tolerance=tol, angularTolerance=0.1)
    t = trimesh.load(str(fn), force="mesh")
    fn.unlink(missing_ok=True)
    return t


def as_manifold(obj):
    t = as_trimesh(obj)
    if t is None:
        return None
    M = tm2mf(t)
    if M.status() != mf.Error.NoError:
        raise ValueError(f"body is not a closed manifold mesh ({M.status()})")
    return M


# ------------------------------------------------------------------------------------------------ orientation
_ROT = {"+z": ([1, 0, 0], 0.0), "-z": ([1, 0, 0], np.pi), "+x": ([0, 1, 0], -np.pi / 2), "-x": ([0, 1, 0], np.pi / 2),
        "+y": ([1, 0, 0], np.pi / 2), "-y": ([1, 0, 0], -np.pi / 2)}


def build_vector(build):
    b = np.zeros(3)
    b["xyz".index(build[1])] = 1.0 if build[0] == "+" else -1.0
    return b


def print_rotation(build):
    """4x4 rotation taking the build direction of the part frame to +z."""
    ax, ang = _ROT[build]
    R = trimesh.transformations.rotation_matrix(ang, ax)
    R[np.abs(R) < 1e-12] = 0.0
    return R


def print_transform(mesh_or_bounds, build):
    """part frame -> print frame: rotate build -> +z, then move the minimum corner to the origin (part on z = 0)."""
    R = print_rotation(build)
    if isinstance(mesh_or_bounds, trimesh.Trimesh):
        pts = mesh_or_bounds.vertices
    else:
        b = np.asarray(mesh_or_bounds, float)
        pts = np.array([[x, y, z] for x in b[:, 0] for y in b[:, 1] for z in b[:, 2]])
    q = trimesh.transformations.transform_points(pts, R)
    T = np.eye(4)
    T[:3, 3] = -q.min(0)
    return T @ R


# ---------------------------------------------------------------------------------------------------- density
@dataclass
class Density:
    rho: np.ndarray            # physical density, flat (C order of shape), 0..1
    shape: tuple
    h: float
    origin: tuple
    solid: np.ndarray          # passive solid (preserve) cells
    free: np.ndarray           # design cells

    @classmethod
    def from_problem(cls, op, x, beta=None):
        """op: printopt.PrintOpt, x: the design vector run() returned. Physical density at the final beta (16)."""
        pb = op.pb
        rp = op.physical(np.asarray(x, float), 16.0 if beta is None else beta)
        return cls(np.asarray(rp, float), tuple(pb.shape), float(pb.h), tuple(pb.origin), pb.solid.copy(),
                   ~pb.solid & ~pb.void)

    @classmethod
    def from_npz(cls, path):
        """npz in the cart12 / catcher fork format: rho_f (physical), shape, h, origin, solid, free."""
        d = np.load(path)
        rho = d["rho_f"] if "rho_f" in d.files else d["rho"]
        return cls(np.asarray(rho, float), tuple(int(n) for n in d["shape"]), float(d["h"]),
                   tuple(float(o) for o in d["origin"]), d["solid"].astype(bool), d["free"].astype(bool))

    @property
    def target_volume(self):
        """optimised generated volume, mm3 (P2 s8.1 keeps this)."""
        return float(self.rho[self.free].sum() * self.h ** 3)


def _lin_axis(a, axis, n_out, up):
    """upsample one axis of a cell-centred array by `up`, linear, clamped at the ends."""
    u = (np.arange(n_out) + 0.5) / up - 0.5
    i0 = np.clip(np.floor(u).astype(int), 0, a.shape[axis] - 1)
    i1 = np.clip(i0 + 1, 0, a.shape[axis] - 1)
    w = np.clip(u - np.floor(u), 0, 1).astype(a.dtype)
    sh = [1, 1, 1]
    sh[axis] = -1
    w = w.reshape(sh)
    return np.take(a, i0, axis=axis) * (1 - w) + np.take(a, i1, axis=axis) * w


def _upsample(a, up):
    out = a.astype(np.float32)
    for ax in range(3):
        out = _lin_axis(out, ax, a.shape[ax] * up, up)
    return out


def _repeat(m, up):
    return m.repeat(up, 0).repeat(up, 1).repeat(up, 2)


def _ball(r):
    k = int(np.ceil(r))
    i = np.arange(-k, k + 1)
    I, J, K = np.meshgrid(i, i, i, indexing="ij")
    return I * I + J * J + K * K <= r * r + 1e-9


# ------------------------------------------------------------------------------------------------------ result
@dataclass
class Result:
    ok: bool = False
    stl: str = ""
    threemf: str = ""
    assembled: str = ""
    log_path: str = ""
    transform: np.ndarray = None
    mesh: trimesh.Trimesh = None            # print frame
    mesh_part: trimesh.Trimesh = None       # part frame
    iso: float = 0.0
    target_volume: float = 0.0              # optimised generated volume, mm3
    generated_volume: float = 0.0           # thresholded generated volume on the fine grid (free cells), mm3
    density_volume: float = 0.0             # sum of rho over solid + free cells x h3 (what the optimiser "saw"), mm3
    exact_target: float = 0.0               # exact preserve inside the envelope - obstacles + target_volume, mm3
    fillet_volume: float = 0.0
    volume: float = 0.0                     # final body, mm3
    dropped: list = field(default_factory=list)    # (kind, volume) of shells removed
    welds: int = 0
    nonmanifold_edges: int = 0
    uncovered_void_cells: int = 0
    reload: dict = field(default_factory=dict)
    log_lines: list = field(default_factory=list)

    def summary(self):
        return "\n".join(self.log_lines)


# ------------------------------------------------------------------------------------------------- the builder
def reconstruct(dens, preserve, obstacle, envelope, build="+z", up=3, sigma=None, fillet=1.0, smooth_band=0.5,
                taubin_iters=10, weld=True, out_dir=None, stem="part", title="", log=print, export=True):
    """P2 s8 from a density to one exported, reload-checked body. See the module docstring for the inputs.
    sigma: Gaussian blur in mm (default 0.4 h). fillet: root fillet radius in mm (0 = off). Returns Result."""
    t0 = time.time()
    R = Result()
    L = R.log_lines

    def say(s):
        L.append(s)
        if log:
            log(s)

    try:
        import rules_stamp
        say(rules_stamp.stamp())
    except Exception:
        say("Rules: (rules_stamp unavailable)")
    say(f"reconstruct (tools\\reconstruct.py, P2 s8) {title}  {time.strftime('%d %b %Y %H:%M')}")
    shape, h, origin = tuple(dens.shape), float(dens.h), np.asarray(dens.origin, float)
    rho = np.clip(np.asarray(dens.rho, float).reshape(shape), 0, 1)
    solid, free = dens.solid.reshape(shape), dens.free.reshape(shape)
    sigma = 0.4 * h if sigma is None else sigma
    p = h / up
    say(f"grid {shape} h {h} origin {origin.round(3).tolist()}; upsample x{up} -> pitch {p:.3f}; blur sigma {sigma:.2f} mm; "
        f"fillet r {fillet}; Taubin {taubin_iters} it outside a {smooth_band} mm interface band; build {build}")

    # ---- exact bodies
    import interfaces as I
    PRE, OBS, ENV = as_trimesh(preserve), as_trimesh(obstacle), as_trimesh(envelope)
    if ENV is None:
        raise ValueError("envelope required (P2 s8.4: trim once against it)")
    say(f"exact bodies: preserve {PRE.volume if PRE is not None else 0:.1f} mm3, obstacle "
        f"{OBS.volume if OBS is not None else 0:.1f} mm3, envelope {ENV.volume:.1f} mm3")
    if PRE is not None and OBS is not None:
        inter = (tm2mf(PRE) ^ tm2mf(OBS)).volume()
        say(f"preserve x obstacle (exact) = {inter:.4f} mm3" + (" (interfaces.py preserves are pre-cut: P2 s4 gate holds)"
            if inter < 1e-2 else " NOTE: the obstacles cut into the preserve bodies; fine for preserves drawn uncut on "
            "purpose (legacy specs: keep boxes minus cuts), a P2 s4 gate failure for interfaces.py features "
            "(run interfaces.gate())"))

    # ---- coarse masks of what is trimmed exactly (s8.2)
    pshape = tuple(n + 2 * PAD for n in shape)
    gpad = (pshape, h, tuple(origin - PAD * h))
    # a cell is trimmed exactly if ANY of it lies outside the envelope or inside an obstacle (half-cells on the
    # envelope faces included: a 0.5 test left the cartridge's z 39-39.5 band uncarried)
    env_c = I.fractions(ENV, gpad, sub=2).reshape(pshape) >= 1 - 1e-9
    obs_c = (I.fractions(OBS, gpad, sub=2).reshape(pshape) > 1e-9) if OBS is not None else np.zeros(pshape, bool)
    trimmed = ~env_c | obs_c
    sl = tuple(slice(PAD, PAD + n) for n in shape)
    dom = solid | free
    void_in = ~dom & ~trimmed[sl]
    R.uncovered_void_cells = int(void_in.sum())
    if R.uncovered_void_cells:
        say(f"WARNING: {R.uncovered_void_cells} void cells inside the envelope are not covered by an exact obstacle; "
            "density is not carried into them (give every keep-out as an exact obstacle)")
    R.target_volume = dens.target_volume
    R.density_volume = float(rho[dom].sum() * h ** 3)

    # ---- s8.2 carry the density one cell past trimmed faces
    rp = np.zeros(pshape)
    rp[sl] = np.where(dom, rho, 0.0)
    dp = np.zeros(pshape, bool)
    dp[sl] = dom
    # the envelope may reach the grid boundary: extend the domain one cell past the grid edge (mode edge)
    rp[1:-1, 1:-1, 1:-1] = np.pad(rp[sl], 1, mode="edge")
    dp_edge = np.zeros(pshape, bool)
    dp_edge[1:-1, 1:-1, 1:-1] = np.pad(dp[sl], 1, mode="edge")
    ext = ndimage.grey_dilation(np.where(dp_edge, rp, 0.0), size=(3, 3, 3))
    carry = ~dp_edge & trimmed
    rp = np.where(dp_edge, rp, np.where(carry, ext, 0.0))
    sol_p = np.zeros(pshape, bool)
    sol_p[sl] = solid
    free_p = np.zeros(pshape, bool)
    free_p[sl] = free
    void_p = ~dp_edge & ~carry                     # cells that stay empty

    # ---- s8.1 upsample, blur, volume-preserving iso (two passes: preserve cells just above the iso found)
    free_f = _repeat(free_p, up)
    sol_f = _repeat(sol_p, up)
    target_f = R.target_volume / p ** 3

    def field_with(pres_val):
        a = np.where(sol_p, pres_val, rp)
        return ndimage.gaussian_filter(_upsample(a, up), sigma=sigma / p)

    def find_iso(F):
        lo, hi = 0.02, 0.98
        vals = F[free_f]
        for _ in range(40):
            iso = 0.5 * (lo + hi)
            if (vals > iso).sum() > target_f:
                lo = iso
            else:
                hi = iso
        return 0.5 * (lo + hi)

    F = field_with(0.55)
    iso = find_iso(F)
    F = field_with(min(1.0, iso + 0.05))
    iso = find_iso(F)
    R.iso = iso
    R.generated_volume = float((F[free_f] > iso).sum() * p ** 3)
    say(f"s8.1 iso {iso:.3f} keeps the generated volume: target {R.target_volume/1000:.3f} cm3, thresholded "
        f"{R.generated_volume/1000:.3f} cm3 ({100*(R.generated_volume/max(R.target_volume,1e-9)-1):+.2f} %)")
    say(f"s8.2 density carried one cell into {int(carry.sum())} trimmed cells (outside the envelope / inside obstacles)")

    # ---- fine masks of the exact preserve / obstacle (interface band, fillets)
    gfine = (F.shape, p, tuple(origin - PAD * h))
    pres_f = (I.fractions(PRE, gfine, sub=1).reshape(F.shape) >= 0.5) if PRE is not None else np.zeros(F.shape, bool)
    obs_f = (I.fractions(OBS, gfine, sub=1).reshape(F.shape) >= 0.5) if OBS is not None else np.zeros(F.shape, bool)
    void_f = _repeat(void_p, up)

    # ---- s8.6 root fillets: grey closing near preserve bodies
    if fillet and fillet > 0 and pres_f.any():
        G = np.where(pres_f | sol_f, np.float32(1.0), F)
        d_pres = ndimage.distance_transform_edt(~pres_f) * p
        near = (d_pres <= 2 * fillet) & ~pres_f & ~void_f & ~obs_f
        if near.any():
            idx = np.argwhere(near)
            m = int(np.ceil(fillet / p)) + 1
            lo_i = np.maximum(idx.min(0) - m, 0)
            hi_i = np.minimum(idx.max(0) + m + 1, F.shape)
            box = tuple(slice(a, b) for a, b in zip(lo_i, hi_i))
            C = ndimage.grey_closing(G[box], footprint=_ball(fillet / p))
            Fb = F[box]
            newb = np.where(near[box], np.maximum(Fb, np.minimum(C, 1.0)), Fb)
            added = (newb > iso) & ~(Fb > iso)
            R.fillet_volume = float(added.sum() * p ** 3)
            F[box] = newb
        say(f"s8.6 root fillets r {fillet} mm: {R.fillet_volume:.1f} mm3 added in concave corners within {2*fillet} mm "
            "of preserve bodies")

    # ---- s8.3 surface
    from skimage import measure
    Fp = np.pad(F, 1, constant_values=0.0)
    v, f, _, _ = measure.marching_cubes(Fp, iso, spacing=(p, p, p))
    v = v + np.asarray(gfine[2]) + p / 2 - p + NUDGE
    gen = trimesh.Trimesh(v, f, process=True)
    if gen.volume < 0:                        # orient the whole surface once (never per shell)
        gen.invert()
    if taubin_iters:
        band_f = ndimage.distance_transform_edt(~(pres_f | obs_f)) * p <= smooth_band + p
        vi = np.clip(np.floor((gen.vertices - np.asarray(gfine[2])) / p).astype(int), 0, np.array(F.shape) - 1)
        keep = band_f[vi[:, 0], vi[:, 1], vi[:, 2]]
        v0 = gen.vertices.copy()
        trimesh.smoothing.filter_taubin(gen, lamb=0.5, nu=0.53, iterations=taubin_iters)
        vv = np.asarray(gen.vertices).copy()
        vv[keep] = v0[keep]
        gen.vertices = vv
        say(f"s8.3 surface: {len(gen.faces)} faces, Taubin on {int((~keep).sum())} vertices, {int(keep.sum())} held in the "
            f"{smooth_band} mm interface band")
    TO = tm2mf(gen)
    if TO.status() != mf.Error.NoError:
        raise RuntimeError(f"generated surface is not manifold: {TO.status()}")

    # ---- s8.4 / 8.5 booleans
    KEEP = tm2mf(PRE).set_tolerance(MERGE_TOL) if PRE is not None else None
    ENVm = tm2mf(ENV).set_tolerance(MERGE_TOL)
    CUT = tm2mf(OBS).set_tolerance(MERGE_TOL) if OBS is not None else None
    TO = TO.set_tolerance(MERGE_TOL)
    body = (TO + KEEP) if KEEP is not None else TO
    final = body ^ ENVm
    if CUT is not None:
        final = final - CUT
    say(f"s8.4-8.5 ((generated + preserve) ^ envelope) - obstacles, merge tolerance {MERGE_TOL} mm: "
        f"{final.volume()/1000:.3f} cm3")
    pres_in = ((KEEP ^ ENVm) - CUT if CUT is not None else KEEP ^ ENVm).volume() if KEEP is not None else 0.0
    R.exact_target = pres_in + R.target_volume
    say(f"volume target (exact preserve inside the envelope minus obstacles {pres_in/1000:.3f} cm3 + optimised generated "
        f"{R.target_volume/1000:.3f} cm3) = {R.exact_target/1000:.3f} cm3; root fillets add {R.fillet_volume/1000:.3f} cm3")

    # ---- s8.7 non-manifold edges (weld last resort), shells, voids
    for it in range(6):
        tm = mf2tm(final)
        chk = trimesh.Trimesh(tm.vertices, tm.faces, process=True)
        u_, c_ = np.unique(chk.edges_sorted, axis=0, return_counts=True)
        bad = u_[c_ != 2]
        R.nonmanifold_edges = len(bad)
        if len(bad) == 0 or not weld or len(bad) > 200:
            if len(bad):
                say(f"s8.7 {len(bad)} non-manifold edges after the booleans" + (" (too many to weld: modelling problem)"
                                                                               if len(bad) > 200 else " (weld off)"))
            break
        mids = np.unique(chk.vertices[bad].mean(axis=1).round(2), axis=0)
        for q in mids:
            final = final + (mf.Manifold.cube([0.8, 0.8, 0.8], center=True).translate(q.tolist()) ^ ENVm)
        if CUT is not None:
            final = final - CUT
        R.welds += len(mids)
        say(f"s8.7 WELD (last resort): {len(bad)} non-manifold edges, {R.welds} cubes so far")
    tm = mf2tm(final)
    shells = sorted(tm.split(only_watertight=False), key=lambda q: -abs(q.volume))
    bodies = [q for q in shells if q.volume > 0]
    if not bodies:
        raise RuntimeError("no body left after the booleans")
    part = bodies[0]
    for q in shells:
        if q is part:
            continue
        kind = "closed void (inward shell, filled)" if q.volume < 0 else "detached body (dropped)"
        R.dropped.append((kind, abs(float(q.volume))))
    for kind, vol in R.dropped[:10]:
        say(f"s8.7 {kind}: {vol:.2f} mm3")
    if len(R.dropped) > 10:
        say(f"s8.7 ... {len(R.dropped) - 10} more shells")
    say(f"s8.7 kept the largest body: {part.volume/1000:.3f} cm3; {len(R.dropped)} other shells "
        f"({sum(v for k, v in R.dropped if 'void' in k):.2f} mm3 voids filled, "
        f"{sum(v for k, v in R.dropped if 'detached' in k):.2f} mm3 loose bits dropped); no simplification")
    R.mesh_part = part
    R.volume = float(part.volume)

    # ---- s8.8 print orientation, export, reload
    T = print_transform(part, build)
    R.transform = T
    pm = part.copy()
    pm.apply_transform(T)
    R.mesh = pm
    say(f"s8.8 print orientation: build {build} -> +z, part on z = 0; print size {np.round(pm.extents, 2).tolist()} mm")
    if export:
        out = Path(out_dir or (Path(os.environ.get("TEMP", ".")) / "noxcheck" / stem))
        out.mkdir(parents=True, exist_ok=True)
        R.stl, R.threemf = str(out / f"{stem}.stl"), str(out / f"{stem}.3mf")
        R.assembled = str(out / f"{stem}_assembled.stl")
        pm.export(R.stl)
        pm.export(R.threemf)
        part.export(R.assembled)
        (out / f"{stem}_transform.json").write_text(json.dumps({"part_to_print": T.tolist(), "build": build}, indent=1))
        R.reload = {"stl": reload_check(R.stl, R.volume), "3mf": reload_check(R.threemf, R.volume)}
        for k, r in R.reload.items():
            say(f"reload {k}: watertight {r['watertight']}, winding {r['winding']}, bodies {r['bodies']}, "
                f"volume {r['volume']/1000:.3f} cm3 (diff {r['vol_diff_pct']:+.4f} %) -> {'OK' if r['ok'] else 'FAIL'}")
        R.ok = all(r["ok"] for r in R.reload.values()) and R.nonmanifold_edges == 0
    else:
        R.ok = R.nonmanifold_edges == 0 and part.is_watertight
    dv = R.volume / max(R.exact_target + R.fillet_volume, 1e-9) - 1
    say(f"mass {R.volume/1000*1.27:.1f} g solid PETG / {R.volume/1000*1.22:.1f} g TPU (M); final {R.volume/1000:.3f} cm3 vs "
        f"target + fillets {(R.exact_target + R.fillet_volume)/1000:.3f} cm3 ({100*dv:+.1f} %; voxel-model part "
        f"{R.density_volume/1000:.3f} cm3)")
    say(f"{'OK' if R.ok else 'FAIL'}  ({time.time()-t0:.0f} s)")
    if export:
        R.log_path = str(Path(R.stl).with_name(f"{stem}_build.log"))
        Path(R.log_path).write_text("\n".join(L) + "\n", encoding="utf-8")
    return R


# ------------------------------------------------------------------------------- P0 runner adapters (spec contract)
SPEC_CONTRACT = r"""Spec contract for the P0 adapters (orient S4, reconstruct S8, class_regions S9): the conductor writes
<run>\<stem>_spec.py (stem = <part>_v<N>); importing it must NOT run printopt (put the run under
if __name__ == '__main__': and save the result as <run>\<stem>_density.npz with key "x", the design vector run()
returned). Names read (attribute or zero-argument function):
  problem / PROBLEM            printopt Problem (required)
  coarse_problem               optional coarse Problem for S4 'run' mode when no density exists yet
  FEATURES                     interfaces.py Feature list (required; [] allowed)
  ENVELOPE / envelope          outer envelope solid (required for S8 / S9)
  PRESERVE_EXTRA, OBSTACLE_EXTRA   part-specific exact preserve / obstacle bodies (optional lists)
  DRAWN [(name, body, cls)], FLEXURES [(name, body)]   extra class regions (optional, S9)
  PRECISION [feature names], SUPPORTS (bool), OVERHANG_ALLOW_MM2, BRIDGE_MM, RUN_ITERS   S4 options (optional)
  RECONSTRUCT {up, sigma, fillet, smooth_band, taubin_iters}   S8 settings (optional; fix F1 changes these)
  MATERIAL ("PETG" | "TPU95A", else the intake's), DENSITY (path, else <stem>_density.npz)"""


def load_spec(run):
    """Import <run.dir>/<stem>_spec.py (never runs it as __main__). Raises FileNotFoundError with the contract."""
    import importlib.util
    p = Path(run.dir) / f"{run.stem()}_spec.py"
    if not p.exists():
        raise FileNotFoundError(f"no spec script {p.name} in the run folder (the conductor writes it before S4/S8).\n"
                                + SPEC_CONTRACT)
    sys.dont_write_bytecode = True
    if str(p.parent) not in sys.path:
        sys.path.insert(0, str(p.parent))
    sp = importlib.util.spec_from_file_location(f"nox_spec_{p.stem}", p)
    m = importlib.util.module_from_spec(sp)
    sp.loader.exec_module(m)
    return m


def spec_get(m, *names, default=None):
    """first of `names` defined in the spec module; zero-argument functions are called."""
    for n in names:
        if hasattr(m, n):
            v = getattr(m, n)
            return v() if callable(v) and not isinstance(v, type) else v
    return default


def density_file(run, m):
    p = spec_get(m, "DENSITY")
    return Path(p) if p else Path(run.dir) / f"{run.stem()}_density.npz"


def load_density(run, m, pb=None):
    """Density from <stem>_density.npz: key "x" (design vector, physical density through the spec's Problem) or the
    older rho_f / solid / free format. None if the file does not exist."""
    p = density_file(run, m)
    if not p.exists():
        return None
    d = np.load(p)
    if "x" in d.files:
        from printopt import PrintOpt
        pb = pb or spec_get(m, "problem", "PROBLEM")
        return Density.from_problem(PrintOpt(pb), d["x"])
    return Density.from_npz(p)


def material_of(run, m):
    mat = spec_get(m, "MATERIAL") or (getattr(run, "R", {}) or {}).get("material") or "PETG"
    return "TPU95A" if "TPU" in str(mat).upper() else "PETG"


def output_name(run):
    """P0 s7.1 file name: scout_<part>_v<N> in scout areas, <part>_v<N> elsewhere."""
    area = str(getattr(run, "area", "") or "").lower()
    st = run.stem()
    return st if st.startswith("scout") or "scout" not in area else f"scout_{st}"


def autorun_hook(ctx):
    """P0 S8 adapter (tools\\autorun.py HOOKS 'reconstruction', B7). ctx: run, stage, dry. Reads the spec (SPEC_CONTRACT)
    and <stem>_density.npz, runs reconstruct() in the Problem's build direction (the density was optimised in it), writes
    the STL / 3MF (print orientation), <name>_assembled.stl, <name>_transform.json in the run folder and the build log in
    checks\\. Returns {stl, threemf, assembled, transform, log, ok, volume_cm3, outputs}; dry: {status: 'dry', plan}."""
    run = ctx["run"]
    dry = ctx.get("dry", getattr(run, "dry", False))
    name = output_name(run)
    plan = [f"import {run.stem()}_spec.py (problem, FEATURES, ENVELOPE, extras)", f"load {run.stem()}_density.npz",
            "reconstruct(): P2 s8 steps 1-8, build = the Problem's build direction",
            f"write {name}.stl / .3mf (print orientation), {name}_assembled.stl, {name}_transform.json, "
            f"checks\\{name}_build.log; reload-check both exports"]
    if dry:
        return dict(status="dry", plan=plan, stl=None, threemf=None)
    import interfaces as I
    m = load_spec(run)
    pb = spec_get(m, "problem", "PROBLEM")
    dens = load_density(run, m, pb)
    if dens is None:
        raise FileNotFoundError(f"no density {density_file(run, m).name}: S7 (the spec's printopt run) saves it")
    feats = spec_get(m, "FEATURES", default=[]) or []
    pres = ([I.preserve_of(feats)] if feats else []) + list(spec_get(m, "PRESERVE_EXTRA", default=[]) or [])
    obs = ([I.obstacle_of(feats)] if feats else []) + list(spec_get(m, "OBSTACLE_EXTRA", default=[]) or [])
    env = spec_get(m, "ENVELOPE", "envelope")
    s4 = ((getattr(run, "R", {}) or {}).get("orientation") or {}).get("build")
    if s4 and s4 in _ROT and s4 != pb.build and hasattr(run, "add"):
        run.add("check", f"S8 built in {pb.build} (the direction the density was optimised in); S4 chose {s4}: "
                         "regenerate in the chosen direction (fix F4) before release")
    settings = dict(spec_get(m, "RECONSTRUCT", default={}) or {})
    logf = getattr(run, "log", None)
    res = reconstruct(dens, pres or None, obs or None, env, build=pb.build, out_dir=run.dir, stem=name,
                      title=f"{run.stem()} (P0 S8)", log=(lambda s: logf("S8 " + s)) if logf else None, **settings)
    chk = Path(run.dir) / "checks"
    chk.mkdir(exist_ok=True)
    log_p = chk / Path(res.log_path).name
    Path(res.log_path).replace(log_p)
    if hasattr(run, "R"):
        run.R["reconstruct"] = dict(ok=res.ok, volume_cm3=round(res.volume / 1000, 3), iso=round(res.iso, 3),
                                    fillet_mm3=round(res.fillet_volume, 1), dropped=len(res.dropped), welds=res.welds,
                                    transform=res.transform.tolist(), build=pb.build)
    if hasattr(run, "add"):
        if not res.ok:
            run.add("check", f"S8 export failed its reload check ({res.reload}); P4 G1 will fail: fix F1")
        if res.welds:
            run.add("check", f"S8 used the last-resort weld ({res.welds} cubes): inspect those spots")
        if res.uncovered_void_cells:
            run.add("check", f"S8: {res.uncovered_void_cells} void cells not covered by an exact obstacle (spec)")
    tr = str(Path(run.dir) / f"{name}_transform.json")
    return dict(stl=res.stl, threemf=res.threemf, assembled=res.assembled, transform=tr, log=str(log_p), ok=res.ok,
                volume_cm3=res.volume / 1000, outputs=[res.stl, res.threemf, res.assembled, tr, str(log_p)])


def reload_check(path, volume=None):
    """Reload an exported STL / 3MF the way a slicer would: merge identical vertices only, nothing repaired."""
    m = trimesh.load(path, force="mesh", process=False)
    m.merge_vertices()
    comps = m.split(only_watertight=False)
    bodies = [c for c in comps if c.volume > 1.0]
    vd = 100 * (m.volume / volume - 1) if volume else 0.0
    ok = bool(m.is_watertight and m.is_winding_consistent and len(comps) == 1 and abs(vd) < 0.01)
    return dict(watertight=bool(m.is_watertight), winding=bool(m.is_winding_consistent), bodies=len(comps),
                big_bodies=len(bodies), volume=float(m.volume), vol_diff_pct=float(vd), ok=ok)


def compare(a, b, n=20000, seed=0):
    """Surface distance between two meshes (same frame): symmetric sampled distances, volumes, bounds."""
    A = a if isinstance(a, trimesh.Trimesh) else trimesh.load(str(a), force="mesh")
    B = b if isinstance(b, trimesh.Trimesh) else trimesh.load(str(b), force="mesh")
    pa, _ = trimesh.sample.sample_surface(A, n, seed=seed)
    pb, _ = trimesh.sample.sample_surface(B, n, seed=seed + 1)
    da = trimesh.proximity.closest_point(B, pa)[1]
    db = trimesh.proximity.closest_point(A, pb)[1]
    MA, MB = tm2mf(A), tm2mf(B)
    return dict(vol_a=float(A.volume), vol_b=float(B.volume), a_minus_b=float((MA - MB).volume()),
                b_minus_a=float((MB - MA).volume()), bounds_a=np.round(A.bounds, 2).tolist(),
                bounds_b=np.round(B.bounds, 2).tolist(), a_to_b_p50=float(np.median(da)), a_to_b_p95=float(np.percentile(da, 95)),
                a_to_b_max=float(da.max()), b_to_a_p50=float(np.median(db)), b_to_a_p95=float(np.percentile(db, 95)),
                b_to_a_max=float(db.max()), watertight_a=bool(A.is_watertight), watertight_b=bool(B.is_watertight),
                bodies_a=len(A.split(only_watertight=False)), bodies_b=len(B.split(only_watertight=False)))
