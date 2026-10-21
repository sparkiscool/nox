"""Overseer bay modules 45 / 135 deg v1 (17 Oct 2026; box-case.md To add 2 + 3, Problem 1): bay ENVELOPES (the Overseer
receiver is added after D10) carrying the funnel collar's 2 posts per module and one spider-tie boss.
One design: the 45 deg module as generated, the 135 deg module = its mirror in X (every asymmetric keep-out and load is
mirrored into the run, so the mirror holds).   python arm_run_ovbay.py [h] [gen_cm3]   (NOX_SETUP_ONLY=1: report only)

Frames. W = wheel frame (arm doc). Local = the module turned to the top (radial = +Z), W = local.rotate([0, 45, 0]);
local +X faces the 0 deg Scout bay (the link side), local -X faces the drone sector.
Drawn (mounting only): back plate r 161.5..164.5 (Scout plate, sized to the Overseer cartridge), 2 link bosses on the
link side (Scout pattern r 125 / 146, 6 inside the 22.5 deg plane; the drone side has no neighbour), 2 post seats
(flat dia 16 disc + M3 x 5.7 insert boss, hole dia 4.0 x 8 along the screw axis) and the spider-tie boss (r 146 on the
45 deg diagonal, face on the near face Y 86).
Reserved (kept empty for the receiver after D10): the Overseer cartridge 55 x 60 x 157 (E) with the Scout receiver's
margins, X +-40.5, r 97..161.5, Y 94.2..259.8. Keep-outs: both heads (and their mirrors) swept to the seat (dilated 1),
the Overseer pack path, the bars' sweep, the link plate and its screw access, the collar (skin + ribs + flanges, 12.5
behind the faces, its lip, its inside) less the post room, the 168 opening down to W Z 140, r 9.5 round each screw
axis for 12 past the seat.
Loads: the Scout module's 11 cases on the back plate (receiver loads enter there), mirrored where one-sided;
Overseer landing 500 N per seat along the screw axis (1 kN per face, F); 13 N rim push in 8 directions shared by the 4
seats (rigid collar, least-norm); box weight 1 g (own mass, collar share on the seats, Overseer pack on the back plate).
Support: the 2 link bosses + the tie boss. Print: near face (Y 86) on the bed, overhang filter ON, no supports."""
import sys, os, math, json
import numpy as np, manifold3d as mf
import arm_layout as L, arm_cad as A
from arm_cad import box, cylY, hull, wedge, swept_head_env, mirror_x
import arm_topo
from arm_topo import PartProblem, run_part, log


def tm(m):
    """manifold -> trimesh welding the seam vertices manifold reports (watertight; arm_cad.tm's process=True is not
    always), patched into arm_cad and arm_topo for this run."""
    import trimesh
    me = m.to_mesh()
    idx = np.arange(len(me.vert_properties))
    if len(me.merge_from_vert): idx[np.asarray(me.merge_from_vert)] = np.asarray(me.merge_to_vert)
    t = trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], idx[np.asarray(me.tri_verts)], process=False)
    t.remove_unreferenced_vertices()
    return t
A.tm = tm; arm_topo.tm = tm

H = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
GEN = float(sys.argv[2]) if len(sys.argv) > 2 else 45.0
S2 = math.sqrt(2.0)
ANG = 45.0

# ---- Overseer cartridge (E: pack 148 x 48 x 45 P + the Scout cartridge's overheads +9 long, +7 wide, +15 tall)
OV_L, OV_W, OV_H = 157.0, 55.0, 60.0
CY0 = L.Y_C - L.Z_TABC                       # 98.5: closed end at the glide, tabs at Y_C 136 (latch block off-centre, DEFAULT)
CY1 = CY0 + OV_L                             # 255.5
R_TOP = L.R + OV_H                           # 160 cartridge top in the bay
R_PLATE = (R_TOP + 1.5, R_TOP + 4.5)         # TPU strip 1.5, back plate 3 (Scout pattern)
PX = OV_W / 2 + 4.3                          # back plate half width (Scout: rails + 4.3)
RES_X = OV_W / 2 + 13.0                      # receiver reserve half width (Scout receiver reaches 13 past the rails)
RES_Y = (CY0 - 4.3, CY1 + 4.3)               # 94.2 .. 259.8 (end tabs)
R_ENV = 170.0                                # module envelope: 5.5 over the back plate (DEFAULT)
Y_TIE = 86.0                                 # spider-tie face (DEFAULT, shared with the -45 / -135 ties)
Y0_ENV, Y1_ENV = Y_TIE - 1.0, RES_Y[1] + 0.5
OPEN_Z = 140.0                               # W Z: the 168 opening kept clear down to here (DEFAULT; below it the drone stays in its sector)

# ---- funnel collar v6 (v2/catch_v4/funnel_panel.py): seat frame -> W: X = x, Y = 138 + y, Z = 159.2 + z
ZB, ZT, ZP, K0 = 1.3, 77.2, -1.2, 84.0 - 1.3
COLLAR_Y, COLLAR_Z = 138.0, 159.2
BEHIND = 12.5                                # skin 1.6 + ribs 9 = 10.6, flanges 12; +0.5
N_AX = np.array([S2 / 2, 0.0, -S2 / 2])      # +X seats' screw axis, out and down (W)
SEATS_W = [np.array([sx * 98.14, y, 166.16]) for sx in (1, -1) for y in (102.0, 174.0)]   # C (box-case To add 2)
MY = [s for s in SEATS_W if s[0] > 0]


def to_loc(m): return m.rotate([0, -ANG, 0])
def to_W(m): return m.rotate([0, ANG, 0])

def _rot_matrix(deg):
    """the 3x3 matrix manifold applies for rotate([0, deg, 0]), read off the basis vectors."""
    M = np.zeros((3, 3))
    for k in range(3):
        e = np.zeros(3); e[k] = 1.0
        c = box(*(np.repeat(e * 10.0, 2) + np.array([0, 0.01, 0, 0.01, 0, 0.01]))).rotate([0, deg, 0])
        bb = np.array(c.bounding_box()); M[:, k] = (bb[:3] + bb[3:]) / 2 / 10.0
    return M
_c, _s = math.cos(math.radians(ANG)), math.sin(math.radians(ANG))
RL = np.array([[_c, 0, -_s], [0, 1, 0], [_s, 0, _c]])        # W -> local for vectors (manifold rotate([0, -45, 0]))
assert np.allclose(RL, _rot_matrix(-ANG), atol=3e-3)
vl = lambda v: RL @ np.asarray(v, float)

def cyl_axis(p0, n, length, r, nseg=64):
    """cylinder from p0 along unit n (n in the XZ plane), length, radius."""
    th = math.degrees(math.atan2(n[0], n[2]))                 # rotate([0, th, 0]) turns +Z toward +X by th
    return mf.Manifold.cylinder(length, r, r, nseg).rotate([0, th, 0]).translate([float(v) for v in p0])

def frustum(k_off, z0, z1):
    a0, a1 = K0 + k_off + z0, K0 + k_off + z1
    s0 = mf.Manifold.cube([2 * a0, 2 * a0, 0.001]).translate([-a0, -a0, z0])
    s1 = mf.Manifold.cube([2 * a1, 2 * a1, 0.001]).translate([-a1, -a1, z1 - 0.001])
    return mf.Manifold.batch_hull([s0, s1])

def seat_axis(s): return N_AX if s[0] > 0 else np.array([-N_AX[0], 0.0, N_AX[2]])


def parts():
    a = math.radians(22.5)
    # ---------------- drawn (local)
    back = box(-PX, PX, RES_Y[0], RES_Y[1], *R_PLATE)
    bosses = None; boss_c = None; link_keep = None
    for r in L.LINK_R:                                        # link side only (+X: the 0 deg Scout bay)
        x = r * math.sin(a) - 6.0 * math.cos(a); z = r * math.cos(a) + 6.0 * math.sin(a)
        bq = cylY(x, z, 98.5, 107.0, 4.5); cq = cylY(x, z, 97.0, 105.0, 2.0)
        acc = cylY(x, z, 40.0, 98.5, 5.0)                     # screw / driver access from -Y (heads face the arm side)
        bosses = bq if bosses is None else bosses + bq
        boss_c = cq if boss_c is None else boss_c + cq
        link_keep = acc if link_keep is None else link_keep + acc
    link_keep += hull([[r * math.sin(a) + dx, y, r * math.cos(a) + dz] for r in (L.LINK_R[0] - 7, L.LINK_R[1] + 7)
                       for y in (90.0, 98.5) for dx, dz in ((-9 * math.cos(a), 9 * math.sin(a)), (0, 0))])
    tie = cylY(0.0, 146.0, Y_TIE, Y_TIE + 8.0, 4.5)           # spider tie: M3 x 5.7 heat-set, hole 4.0 x 6.5 (as the link bosses)
    tie_c = cylY(0.0, 146.0, Y_TIE - 1.0, Y_TIE + 6.5, 2.0)
    seatsW = None; seat_cW = None
    for s in MY:
        n = seat_axis(s)
        d = cyl_axis(s, n, 2.0, 8.0) + cyl_axis(s, n, 10.0, 4.3)          # flat dia 16 seat + insert boss
        c = cyl_axis(s - 0.5 * n, n, 8.5, 2.0, 48)                         # dia 4.0 x 8 from the seat centre
        seatsW = d if seatsW is None else seatsW + d
        seat_cW = c if seat_cW is None else seat_cW + c
    seats = to_loc(seatsW); seat_c = to_loc(seat_cW)
    drawn = back + bosses + tie + seats
    # ---------------- keep-outs (local)
    reserve = box(-RES_X, RES_X, RES_Y[0], RES_Y[1], L.R - 3.0, R_PLATE[0])
    ppath = box(-OV_W / 2 - 0.15, OV_W / 2 + 0.15, CY0 - 0.15, CY1 + 0.15, L.Z0_A_HOME + L.CART_BOT_DZ - 1.0, R_PLATE[0] - 0.1)
    heads = None
    for v in ("A", "B"):
        hv = swept_head_env(v, L.Z0_A_HOME, L.Z0_BAY + 1.5)
        heads = hv if heads is None else heads + hv
    heads = heads + mirror_x(heads)                              # the 135 deg module sees the heads mirrored
    bars = cylY(0, 0, 60.0, 95.5, 136.5)
    collarW = frustum(BEHIND * S2, ZP - 0.5, 260.0).translate([0, COLLAR_Y, COLLAR_Z])   # skin, ribs, flanges, lip and the inside
    roomW = None; nearW = None
    for s in SEATS_W:
        n = seat_axis(s)
        rm = cyl_axis(s, n, 40.0, 9.5)                            # post room (no ribs within r 9.5 behind the landing)
        nz = cyl_axis(s, n, 12.0, 30.0) - cyl_axis(s - n, n, 14.0, 9.4)   # the post stays inside r 9.4 for 12 past the seat (9.5 rule; 9.4 so no surface coincides with the post room)
        roomW = rm if roomW is None else roomW + rm
        nearW = nz if nearW is None else nearW + nz
    collarW = collarW - roomW
    openingW = box(-85.0, 85.0, COLLAR_Y - 85.0, COLLAR_Y + 85.0, OPEN_Z, COLLAR_Z + ZP)   # the 168 opening (+1), 18 under the collar lip
    shaftW = None
    for s in SEATS_W:                                             # screw axis inside the funnel (driver), already inside the collar solid
        sh = cyl_axis(s - 60.0 * seat_axis(s), seat_axis(s), 60.0 - 6.0, 5.0)
        shaftW = sh if shaftW is None else shaftW + sh
    keepW = collarW + nearW + openingW + shaftW
    keep = reserve + ppath + heads + bars + link_keep + to_loc(keepW)
    cuts = boss_c + tie_c + seat_c + keep
    # ---------------- envelope (local)
    main = (cylY(0, 0, Y0_ENV, Y1_ENV, R_ENV, 180) - cylY(0, 0, Y0_ENV - 1, Y1_ENV + 1, 99.5, 180)) ^ wedge(22.5, L.JOINT_GAP, Y0_ENV - 1, Y1_ENV + 1)
    postz = (cylY(0, 0, Y0_ENV, 192.0, 206.0, 180) - cylY(0, 0, Y0_ENV - 1, 193.0, R_ENV - 0.5, 180)) ^ wedge(22.5, L.JOINT_GAP, Y0_ENV - 1, 193.0)
    env = (main + postz) - keep + drawn
    regions = dict(back=box(-PX, PX, CY0, CY1, *R_PLATE), links=bosses, tie=tie, support=bosses + tie,
                   seats=[to_loc(cyl_axis(s, seat_axis(s), 2.0, 8.0)) for s in MY])
    return dict(drawn=drawn, cuts=cuts, env=env, allowed=env - cuts, regions=regions, keepW=keepW)


def rim_cases(F=13.0):
    """13 N at the funnel rim in 8 horizontal directions, carried by the rigid collar to its 4 seats (least-norm split).
    Returns [(name, [force on MY[0], force on MY[1]] in W)]."""
    c = np.array([0.0, COLLAR_Y, 0.0]); P = np.array(SEATS_W) - c
    Aeq = np.zeros((6, 12))
    for i, p in enumerate(P):
        Aeq[0:3, 3 * i:3 * i + 3] = np.eye(3)
        Aeq[3:6, 3 * i:3 * i + 3] = np.array([[0, -p[2], p[1]], [p[2], 0, -p[0]], [-p[1], p[0], 0]])
    out = []
    for k in range(8):
        t = math.radians(45.0 * k); d = np.array([math.cos(t), math.sin(t), 0.0])
        m = max(abs(d[0]), abs(d[1])); pr = np.array([-160.0 * d[0] / m, -160.0 * d[1] / m, COLLAR_Z + ZT])
        Fv = F * d; b = np.concatenate([Fv, np.cross(pr, Fv)])
        f = np.linalg.lstsq(Aeq, b, rcond=None)[0].reshape(4, 3)
        mine = [f[i] for i, s in enumerate(SEATS_W) if s[0] > 0]
        out.append((f"rim push 13 N {int(45 * k)} deg", mine))
    return out


class OvProblem(PartProblem):
    """the ray voxeliser (trimesh rays) missed a hit on the keep-out union (odd hit count: a whole column of the post
    room came out 'cut'). Here: exact cross-sections of the manifold at every cell-centre height, cell centres tested
    against them even-odd (a micron off the grid so no centre sits on an edge)."""
    def vox(self, t):
        from matplotlib.path import Path
        from arm_cad import man
        m = man(t); xs, ys, zs = self.axes
        XX, YY = np.meshgrid(xs + 0.0013, ys + 0.0029, indexing="ij"); P = np.c_[XX.ravel(), YY.ravel()]
        A = np.zeros((len(xs), len(ys), len(zs)), bool)
        for k, z in enumerate(zs):
            ins = np.zeros(len(P), bool)
            for poly in m.slice(float(z)).to_polygons():
                if len(poly) >= 3: ins ^= Path(np.asarray(poly)).contains_points(P)
            A[:, :, k] = ins.reshape(len(xs), len(ys))
        return A

    def drop_islands(self, support_region):
        """design pockets with no face-connected path to the supports (slivers between keep-outs) make the stiffness
        singular: void them."""
        from scipy import ndimage
        live = (self.solid | self.design).reshape(self.shape)
        lab, n = ndimage.label(live)
        sup = self.vox(tm(support_region)) & live
        keep_ids = np.unique(lab[sup]); keep_ids = keep_ids[keep_ids > 0]
        dead = live & ~np.isin(lab, keep_ids)
        nd = int((dead & self.design.reshape(self.shape)).sum()); ns = int((dead & self.solid.reshape(self.shape)).sum())
        if ns: log(f"WARNING: {ns} drawn cells not connected to the supports")
        self.design &= ~dead.ravel(); self.void |= (dead.ravel() & ~self.solid)
        log(f"  islands: {n} components, {n - len(keep_ids)} dropped ({nd} design cells)")


def multi_load(pp, name, pairs, cls="misuse", accel=None):
    F = np.zeros(pp.ndof)
    for region, vec in pairs:
        el = pp.elems(region, pp.solid | pp.design)
        if len(el) == 0: raise ValueError(f"load {name}: empty region")
        nd = pp.nodes_of(el)
        for a_ in range(3): F[3 * nd + a_] += vec[a_] / len(nd)
    import printopt as PO
    pp.cases.append(PO.LoadCase(name, F, cls=cls, support="default", accel=None if accel is None else np.asarray(accel, float)))


if __name__ == "__main__":
    B = parts(); drawn, cuts, env, R = B["drawn"], B["cuts"], B["env"], B["regions"]
    pp = OvProblem("ovbay", H, drawn, cuts, env, build="+y", overhang=True)          # near face (Y 86) on the bed
    pp.drop_islands(R["support"])
    pp.support("default", R["support"])
    back = R["back"]
    # the Scout module's 11 cases, on the back plate (the receiver hangs from it after D10); one-sided ones mirrored
    pp.load("insert push 20 N", back, (0, 0, 20)); pp.load("extract pull 20 N", back, (0, 0, -20))
    pp.load("contact shoe +X 10 N", back, (10, 0, 0)); pp.load("contact shoe -X 10 N", back, (-10, 0, 0))
    for v, nm in (((20, 0, 0), "+x"), ((-20, 0, 0), "-x"), ((0, 20, 0), "+y"), ((0, -20, 0), "-y"), ((0, 0, -20), "-z")):
        pp.load(f"hand on the back plate 20 N {nm}", back, v)
    PACK_N = 0.70 * 9.81                                     # Overseer pack + cartridge 0.70 kg (E)
    for v, nm in (((3, 0, 0), "+x"), ((-3, 0, 0), "-x"), ((0, 3, 0), "+y"), ((0, 0, -3), "-z")):
        pp.load(f"Overseer pack 3 g {nm}", back, tuple(PACK_N * np.array(v)))
    # Overseer landing: 1 kN per face (F, box-stand.md), 500 N per seat along the screw axis
    multi_load(pp, "Overseer landing 1 kN per face", [(rg, vl(500.0 * seat_axis(s))) for rg, s in zip(R["seats"], MY)], cls="crash")
    for nm, fs in rim_cases():
        multi_load(pp, nm, [(rg, vl(f)) for rg, f in zip(R["seats"], fs)])
    COLLAR_N = 0.475 * 9.81 / 4                              # collar ~475 g (E) on 4 seats
    multi_load(pp, "box weight 1 g", [(rg, vl((0, 0, -COLLAR_N))) for rg in R["seats"]] + [(back, vl((0, 0, -PACK_N)))],
               cls="sustained", accel=vl((0, 0, -1.0)))
    log(f"cases {len(pp.cases)}; design {pp.design.sum() * H ** 3 / 1000:.0f} cm3; drawn {(drawn - cuts).volume() / 1000 * 1.27:.1f} g")
    for nm, f in rim_cases()[:2]: log(f"  {nm}: seat forces W {[np.round(x, 2).tolist() for x in f]}")
    if not os.environ.get("NOX_SETUP_ONLY"):
        os.makedirs("out", exist_ok=True)
        res, mesh = run_part(pp, drawn, cuts, env, GEN, "out/box_ovbay_v1_local.stl")
