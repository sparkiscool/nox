"""Funnel collar v5 on the bay ring: Overseer landing on one face, reacted only at the 4 seat pads. 16 Oct 2026.
Whole glued collar (8 panels, bonds taken as perfect), P2 tets (skfem), pardiso. Index = printopt layer-aware failure
index (in-layer von Mises / Axy; across-layer sqrt((sigma_n+/Az)^2 + (tau_n/As)^2), layer normal = each panel's face
normal, it prints contact face down). Elements touching a support or a load patch are left out (printopt rule).
Landing (E, box-stand.md envelope): Overseer 2.0 kg, motors off at first touch, free fall then 0.15 m/s drift out;
2 contacts 200 apart on one face, force normal to the face (they roll: no friction). Peak force from the face's own
stiffness at the contacts with a rigid drone: F = v_n sqrt(k m) + m g cos45 (no contact compliance: conservative).
  routine: 50 mm fall, class flight (SF 4);   worst: 100 mm fall, class misuse (SF 2.5).
Faces: +x (over 2 seats) and +y (no seat under it: the load goes round the valleys). Contacts high (top band) and
mid-slope, centred and +30 off along the face.
python funnel_seat_fe.py [W L F] [--save]   (pad size; default = funnel_panel.PAD)"""
import sys, os, json, math, time
import numpy as np, scipy.sparse as sp
os.environ.setdefault("PYPARDISO_MKL_RT", "/usr/local/lib/libmkl_rt.so.3")
import pypardiso, skfem, collar_mesh
from skfem.models.elasticity import linear_elasticity, lame_parameters
from skfem.helpers import dot
import funnel_panel as fp

E_, NU = 2780.0, 0.38                          # PETG in-layer (printopt card); across-layer softness ignored (E)
XT, ZT_, KD = 51.0, 25.5, 0.85
SF = dict(flight=4.0, misuse=2.5)
ALLOW = {c: (XT * KD / s, ZT_ * KD / s, ZT_ * KD / s) for c, s in SF.items()}
GLUE = 2.0                                     # MPa bond design value on PETG (research rule, E) until coupons
M, G = 2.0, 9.81
def v_normal(h):                               # m/s normal to a 45 deg face: fall h (m) + 0.15 drift outward
    return math.sqrt(2 * G * h) * math.cos(math.radians(45)) + 0.15 * math.cos(math.radians(45))
DROPS = dict(routine=(0.05, "flight"), worst=(0.10, "misuse"))
SEATS = [(sx * fp.SEAT_X, sy * fp.SEAT_Y) for sx in (1, -1) for sy in (1, -1)]
H_FINE = float(os.environ.get("HFINE", 1.6))   # mesh size at the pads, mm
TCD_R = 1.5                                    # critical distance, mm (E, printed PETG)
POST_R = float(os.environ.get("POST_R", 7.0))   # seat post top radius, mm (DEFAULT, E)
R_PATCH = 4.0                                  # contact patch radius, mm (wheel on the face, E)

def contacts(face, v, off):
    """two contact points on a face, v up the slope from the bottom edge, 200 apart, shifted `off` along the face."""
    s = v / math.sqrt(2)
    r, z = 84.0 + s, fp.ZB + s
    pts = [(r, off + 100.0, z), (r, off - 100.0, z)]
    if face == "+y": pts = [(-p[1], p[0], p[2]) for p in pts]           # rotate +90 deg about z
    n_in = np.array([-1, 0, 1]) / math.sqrt(2) if face == "+x" else np.array([0, -1, 1]) / math.sqrt(2)
    return np.array(pts), -n_in                                          # drone pushes outward-down

V_HI = fp.SLANT - 10.0                          # high on the face: just under the top band
CASES = [("x_top", "+x", V_HI, 0.0), ("x_top_off", "+x", V_HI, 30.0), ("x_mid", "+x", fp.SLANT / 2, 0.0),
         ("y_top", "+y", V_HI, 0.0), ("y_top_off", "+y", V_HI, 30.0), ("y_mid", "+y", fp.SLANT / 2, 0.0)]

def mesh_collar(W, L, F, h):
    full = fp.collar_world(W=W, L=L, F=F)              # keys left out (57 mm3, their 0.15 gaps would wreck the mesh)
    fine = [(fp.SEAT_X, fp.SEAT_Y, 0.0)]; fine += [(x, -y, z) for x, y, z in fine]
    fine = [(x * c - y * s, x * s + y * c, z) for x, y, z in fine for c, s in ((1, 0), (0, 1), (-1, 0), (0, -1))]
    nodes, elem, tv = collar_mesh.tet_mesh(full, h=h, h_fine=H_FINE, fine_pts=fine, r_fine=25.0)
    assert abs(tv / full.volume() - 1) < 0.003, f"mesh volume {tv:.0f} vs CAD {full.volume():.0f}"
    return skfem.MeshTet(nodes.T.copy(), elem.T.copy()), full.volume()

def run(W, L, F, maxvol=5.0, save=False, log=print):
    t0 = time.time()
    mesh, vol = mesh_collar(W, L, F, maxvol)
    e = skfem.ElementVector(skfem.ElementTetP2())
    ib = skfem.Basis(mesh, e, intorder=2)
    lam, mu = lame_parameters(E_, NU)
    K = skfem.asm(linear_elasticity(lam, mu), ib)
    p = mesh.p; bf = mesh.boundary_facets(); fc = p[:, mesh.facets[:, bf]].mean(1)
    # supports: the 4 used seat posts. Post top = disc of POST_R round the screw axis (DEFAULT, E): the pad is the spreader
    on_bot = np.abs(fc[2] - fp.ZP) < 1e-3
    near = np.zeros(bf.size, bool)
    for sx, sy in SEATS:
        near |= np.hypot(fc[0] - sx, fc[1] - sy) < POST_R
    sup_f = bf[on_bot & near]
    D = ib.get_dofs(facets=sup_f).all()
    # contact-face facets
    nrm = np.cross(p[:, mesh.facets[1, bf]] - p[:, mesh.facets[0, bf]], p[:, mesh.facets[2, bf]] - p[:, mesh.facets[0, bf]], axis=0)
    nrm /= np.linalg.norm(nrm, axis=0)
    B, info_c, load_verts = [], [], set()
    for name, face, v, off in CASES:
        pts, d = contacts(face, v, off)
        n_in = -d
        sel = np.zeros(bf.size, bool)
        for q in pts:
            sel |= (np.linalg.norm(fc - q[:, None], axis=0) < R_PATCH) & (np.abs(np.abs(nrm.T @ n_in) - 1) < 0.02)
        fb = skfem.FacetBasis(mesh, e, facets=bf[sel], intorder=4)
        area = skfem.Functional(lambda w: 1.0 + 0 * w.x[0]).assemble(skfem.FacetBasis(mesh, skfem.ElementTetP1(), facets=bf[sel]))
        dvec = d / area                                                   # 1 N total, uniform over the patches
        b = skfem.LinearForm(lambda vv, w: dvec[0] * vv[0] + dvec[1] * vv[1] + dvec[2] * vv[2]).assemble(fb)
        B.append(b); info_c.append((name, face, int(sel.sum()), area))
        load_verts.update(np.unique(mesh.facets[:, bf[sel]]).tolist())
    B = np.stack(B, 1)
    I = ib.complement_dofs(D)
    Kf = K[I][:, I].tocsr()
    U = np.zeros_like(B)
    ps = pypardiso.PyPardisoSolver(); ps.set_matrix_type(2); Kf = sp.triu(Kf, format="csr")
    U[I] = ps.solve(Kf, np.ascontiguousarray(B[I])).reshape(len(I), -1)
    ps.free_memory(everything=True)
    t_solve = time.time() - t0
    # element stresses at quadrature points, per unit load
    tc = p[:, mesh.t].mean(1)
    ax_x = np.abs(tc[0]) >= np.abs(tc[1])
    ln = np.zeros((3, mesh.t.shape[1]))
    ln[0] = np.where(ax_x, -np.sign(tc[0]), 0); ln[1] = np.where(ax_x, 0, -np.sign(tc[1])); ln[2] = 1
    ln /= np.linalg.norm(ln, axis=0)
    excl_v = np.zeros(p.shape[1], bool)
    excl_v[np.unique(mesh.facets[:, sup_f])] = True; excl_v[list(load_verts)] = True
    keep = ~excl_v[mesh.t].any(0)
    from scipy.spatial import cKDTree
    a_, b_, c_, d_ = (p[:, mesh.t[i]].T for i in range(4))
    evol = np.abs(np.einsum("ij,ij->i", np.cross(b_ - a_, c_ - a_), d_ - a_)) / 6
    pairs = cKDTree(tc.T).query_pairs(TCD_R, output_type="ndarray")
    ii = np.concatenate([pairs[:, 0], pairs[:, 1], np.arange(tc.shape[1])]); jj = np.concatenate([pairs[:, 1], pairs[:, 0], np.arange(tc.shape[1])])
    ball = sp.csr_matrix((evol[jj], (ii, jj)), shape=(tc.shape[1],) * 2)
    ball = sp.diags(1 / np.asarray(ball.sum(1)).ravel()) @ ball
    # region tags for reporting
    rr = np.maximum(np.abs(tc[0]), np.abs(tc[1]))
    pad_reg = tc[2] < fp.ZB + 0.01 + 0 * rr                               # below the bottom edge = pads
    for sx, sy in SEATS: pass
    near_seat = np.zeros(tc.shape[1], bool)
    for sx, sy in SEATS:
        near_seat |= np.hypot(tc[0] - sx, tc[1] - sy) < 30
    out = dict(pad=dict(W=W, L=L, F=F), tets=int(mesh.t.shape[1]), dofs=int(ib.N), solve_s=round(t_solve, 1),
               g_collar=round(vol / 1000 * 1.27, 1), cases=[])
    worst = None; fields = {}
    for j, (name, face, nf, area) in enumerate(info_c):
        u = U[:, j]; c1 = float(B[:, j] @ u)                              # mm per N
        k = 1.0 / c1 * 1000.0                                             # N/m
        w = ib.interpolate(u); gu = w.grad                                # (3,3,nel,nqp)
        eps = 0.5 * (gu + np.transpose(gu, (1, 0, 2, 3)))
        tr = eps[0, 0] + eps[1, 1] + eps[2, 2]
        sig = 2 * mu * eps;
        for a in range(3): sig[a, a] += lam * tr
        s = sig
        vm = np.sqrt(np.maximum(0.5 * ((s[0, 0] - s[1, 1]) ** 2 + (s[1, 1] - s[2, 2]) ** 2 + (s[2, 2] - s[0, 0]) ** 2)
                                + 3 * (s[0, 1] ** 2 + s[1, 2] ** 2 + s[2, 0] ** 2), 0))
        tn = np.einsum("ijeq,je->ieq", s, ln)                              # traction on the layer plane
        sn = np.einsum("ieq,ie->eq", tn, ln)
        tau = np.linalg.norm(tn - sn[None] * ln[:, :, None], axis=0)
        # critical distance (E): element-mean stress, then averaged over a TCD_R ball (sharp re-entrant edges are
        # singular; a point peak there is mesh noise, the 1.5 mm mean is what a printed PETG part feels)
        vm_e, sn_e, tau_e = (ball @ q.mean(1) for q in (vm, np.maximum(sn, 0), tau))
        # glue lines: centre seams (x = 0 / y = 0) and valleys (|x| = |y|), material within 1.6 of the plane
        gl = np.zeros(tc.shape[1]);
        for nvec, dist in (((1, 0, 0), np.abs(tc[0])), ((0, 1, 0), np.abs(tc[1])),
                           ((1, -1, 0), np.abs(tc[0] - tc[1]) / math.sqrt(2)), ((1, 1, 0), np.abs(tc[0] + tc[1]) / math.sqrt(2))):
            nv = np.array(nvec, float) / np.linalg.norm(nvec); m_ = dist < 1.6
            t = np.einsum("ijeq,j->ieq", s[:, :, m_], nv); tnn = np.einsum("ieq,i->eq", t, nv)
            mag = np.sqrt(np.maximum(tnn, 0) ** 2 + (np.linalg.norm(t, axis=0) ** 2 - tnn ** 2)).mean(1)
            gl[m_] = np.maximum(gl[m_], mag)
        gl = ball @ gl
        for drop, (h, cls) in DROPS.items():
            Fpk = v_normal(h) * math.sqrt(k * M) + M * G * math.cos(math.radians(45))
            Axy, Az, As = ALLOW[cls]
            f1 = vm_e * Fpk / Axy; f2 = np.sqrt((sn_e * Fpk / Az) ** 2 + (tau_e * Fpk / As) ** 2)
            f = np.where(keep, np.maximum(f1, f2), 0)
            i = int(np.argmax(f)); fg = np.where(keep, gl * Fpk / GLUE, 0); ig = int(np.argmax(fg))
            fpad = np.where(keep & (near_seat | pad_reg), np.maximum(f1, f2), 0); ip = int(np.argmax(fpad))
            r = dict(case=name, drop=drop, cls=cls, k_N_per_mm=round(k / 1000, 1), F_N=round(Fpk, 1),
                     defl_mm=round(Fpk * c1, 2), index=round(float(f[i]), 2), kind="in-layer" if f1[i] >= f2[i] else "across",
                     where=[round(float(x), 1) for x in tc[:, i]],
                     pad_index=round(float(fpad[ip]), 2), pad_where=[round(float(x), 1) for x in tc[:, ip]],
                     glue_index=round(float(fg[ig]), 2), glue_where=[round(float(x), 1) for x in tc[:, ig]])
            out["cases"].append(r)
            if worst is None or r["index"] > worst["index"]: worst = r
            if save and drop == "routine": fields[name] = np.maximum(f1, f2) * keep
    out["worst"] = worst
    out["worst_pad"] = max(out["cases"], key=lambda r: r["pad_index"])
    out["worst_glue"] = max(out["cases"], key=lambda r: r["glue_index"])
    if save:
        np.savez_compressed("out/fe_fields.npz", p=p, t=mesh.t, **fields)
    log(json.dumps(dict(pad=out["pad"], tets=out["tets"], dofs=out["dofs"], s=round(time.time() - t0)), separators=(",", ":")))
    return out

if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if len(args) >= 4: fp.PAD.update(W=float(args[0]), L=float(args[1]), F=float(args[2]), FY=float(args[3]))
    W, L, F = fp.PAD["W"], fp.PAD["L"], fp.PAD["F"]
    mv = float(os.environ.get("MAXVOL", 5.0))
    res = run(W, L, F, maxvol=mv, save="--save" in sys.argv)
    os.makedirs("out", exist_ok=True)
    res["pad"]["FY"] = fp.PAD["FY"]
    for r in res["cases"]: r["index_per_100N"] = round(r["index"] / r["F_N"] * 100, 3)
    json.dump(res, open(f"out/fe_{W:g}_{L:g}_{F:g}_{fp.PAD['FY']:g}.json", "w"), indent=1)
    for r in res["cases"]:
        print(f"{r['case']:10s} {r['drop']:7s} k {r['k_N_per_mm']:6.1f} N/mm  F {r['F_N']:6.1f} N  defl {r['defl_mm']:5.2f}  "
              f"index {r['index']:5.2f} {r['kind']:8s} at {r['where']}  pad {r['pad_index']:5.2f}  glue {r['glue_index']:5.2f}")
