"""Box stand v5e printed parts (18 Oct 2026): 4 funnel seat pads + 1 bay cradle, drawn in manifold3d from stand_v5_parts.json.
W frame (box-arm.md): X across, Y along the axle, Z up, axle X 0 Z 0, floor Z -201, rail tops Z -181. Tags: C CAD, M mesh, E estimate.

Seat pad (R-front shown; R-back = Y mirror about 177, L = X mirror; R-front == L-back as a print, R-back == L-front):
  20 x 20 pad on the collar seat face (X 98.14, Z 151.46, n (0.707, 0, -0.707)), 12 thick, M3 along n into the collar insert;
  a 6-chord truss bar (10 wide, flush with the pad's far-from-slot face) to a 6 plate on the post's inner face X 180;
  the plate runs 26 along Y to the post's slot (Y 167 for the Y 141 pad) and carries one M5 + T-nut.
  Print face: the pad's outer Y face (everything else stands on it; no overhang).
Cradle: Y 137..197, X +-35, floor Z -201, top = r 152.5 cylinder (the -90 module's plate), 48.5 tall at X 0; open to the front,
  cap 2.4 / walls 2.4 / floor 2.4 / back 3; two ribs under the cap carry the 4 M5 up-bolts (placed by the mesh probe,
  see probe_depth()); 2 x M5 through the back wall into rail 2's slot at (X +-10, Z -191); hex access holes in the floor.
  Print face: the back (bed on rail 2's -Y face); every pocket then opens upward, no overhang.
Usage: python stand_v5_parts.py   -> box_seat_pad_v5e_A.stl, _B.stl, box_bay_cradle_v5e.stl (print orientation), fit report."""
import json, numpy as np, trimesh
from manifold3d import Manifold, CrossSection
from shapely.geometry import Polygon
from shapely.geometry.polygon import orient

J = json.load(open("stand_v5_parts.json"))
X_POST = J["x_post"]                       # 180 (C)
FLOOR, RAIL_TOP = J["base"]["top"] - J["base"]["t"], J["base"]["top"]   # -201, -181 (C)
R_PLATE = 152.5                            # module back plate outer face (C / M)
SEAT = np.array([98.14, 151.46]); N = np.array([1, -1]) / np.sqrt(2); U = np.array([1, 1]) / np.sqrt(2)   # collar seat face (C)
PAD_Y = (141.0, 213.0); SLOT_Y = (167.0, 187.0)        # funnel seats and the side posts' inner-face slots (C)
WALL = 2.4; D = 1.27e-3                     # g/mm3 PETG solid (E)

def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])
def cyl(r, h, axis, at, seg=96):
    """cylinder of radius r, length h along axis ('x','y','z'), starting at point at (3-vector)."""
    c = Manifold.cylinder(h, r, r, seg)
    if axis == "x": c = c.rotate([0, 90, 0])
    if axis == "y": c = c.rotate([-90, 0, 0])
    return c.translate(list(at))
def ccw(poly):
    return [tuple(map(float, p)) for p in orient(Polygon(poly), 1.0).exterior.coords[:-1]]
def prism_xz_(poly, y0, y1):
    """polygon in the XZ plane extruded over Y y0..y1."""
    cs = CrossSection([ccw(poly)])
    m = cs.extrude(y1 - y0)                      # (a=x, b=z, c=extrude)
    return m.transform(np.array([[1, 0, 0, 0], [0, 0, 1, y0], [0, 1, 0, 0]], float))
def prism_yz(poly, x0, x1):
    cs = CrossSection([ccw(poly)])
    m = cs.extrude(x1 - x0)                      # (a=y, b=z, c=x)
    return m.transform(np.array([[0, 0, 1, x0], [1, 0, 0, 0], [0, 1, 0, 0]], float))
def mesh_of(m):
    g = m.to_mesh(); return trimesh.Trimesh(np.asarray(g.vert_properties)[:, :3], np.asarray(g.tri_verts), process=False)

# ---------------- seat pad (R-front)
def seat_pad():
    yp = PAD_Y[0]; ys = SLOT_Y[0]; y_bed = yp - 10          # bed face Y 131 (far from the slot); part grows toward +Y
    p0 = SEAT
    A = p0 - 10 * U + 0.3 * N; B = p0 + 10 * U + 0.3 * N; A2 = A + 12 * N; B2 = B + 12 * N
    pad = prism_xz_([A, B, B2, A2], y_bed, y_bed + 20)                                   # 20 x 20 x 12 (C)
    P = Polygon([A2, B2, (X_POST - 6, 165), (X_POST - 6, 140)])                          # bar profile to the plate (C)
    W_ = P.buffer(-6, join_style=2)                                                        # window: chords and ends 6 (C)
    bar = prism_xz_(list(P.exterior.coords)[:-1], y_bed, y_bed + 10) - prism_xz_(list(W_.exterior.coords)[:-1], y_bed - 1, y_bed + 11)
    # root: the bar's solid corner around the M3 hole, full pad width (the hole and its head stay in material)
    root = prism_xz_(list(P.exterior.coords)[:-1], y_bed, y_bed + 20) ^ box(90, 119.5, y_bed, y_bed + 20, 120, 170)
    root = root - prism_xz_(list(W_.exterior.coords)[:-1], y_bed - 1, y_bed + 21)
    # plate on the post face: link bar 6 x 12 from the beam to the slot, head 6 x 25 over Y ys-10..ys+6 (bears on the post)
    link = box(X_POST - 6, X_POST, y_bed, ys - 10, 146, 159)
    head = prism_yz([(ys - 10 - 3.5, 146), (ys - 10, 140), (ys + 6, 140), (ys + 6, 165), (ys - 10, 165), (ys - 10 - 3.5, 159)], X_POST - 6, X_POST)
    part = pad + bar + root + link + head
    # M3 along n through the pad centre: counterbore 6.5 from the far side so 15 of PETG stays under the head (C)
    c0 = (p0 + 0.3 * N); c3 = np.array([c0[0], yp, c0[1]])
    nvec = np.array([N[0], 0, N[1]])
    def along(r, h, start, direction):
        c = Manifold.cylinder(h, r, r, 48)
        d = np.asarray(direction, float); d /= np.linalg.norm(d)
        a = np.cross([0, 0, 1], d); s = np.linalg.norm(a); cth = d[2]
        if s < 1e-9: R = np.eye(3) if cth > 0 else np.diag([1, -1, -1])
        else:
            k = a / s; K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]]); R = np.eye(3) + s * K + (1 - cth) * K @ K
        T = np.c_[R, np.asarray(start, float)]
        return c.transform(T)
    part = part - along(1.7, 40, c3 - 5 * nvec, nvec)
    part = part - along(3.25, 20, c3 + 15.0 * nvec, nvec)                                  # counterbore floor at n 15.3 (C)
    part = part - cyl(2.75, 10, "x", [X_POST - 8, ys, 152.5])                              # M5 clearance, bolt along +X into the T-nut (C)
    return part, dict(y_bed=y_bed, m3_axis=c3.tolist(), m5=[X_POST, ys, 152.5])

# ---------------- cradle
def probe_depth():
    """first-solid depth under the -90 module's plate (ray up from below), min over a r 4 disc, both sides; M."""
    mm = trimesh.load("../models/box_baymod_v4i_W.obj", force="mesh")
    V = mm.vertices.copy(); V[:, 0] *= -1; V[:, 2] *= -1                                 # bay 0 (+Z) -> -90 (-Z)
    mm = trimesh.Trimesh(V, mm.faces, process=False); mm.fix_normals()
    ri = trimesh.ray.ray_triangle.RayMeshIntersector(mm)
    def dep(x, y):
        loc, _, _ = ri.intersects_location(np.array([[x, y, -300.0]]), np.array([[0, 0, 1.0]]))
        if len(loc) < 2: return 0.0, np.nan
        z = np.sort(loc[:, 2]); return z[1] - z[0], z[0]
    def disc_min(x, y, r=4.0):
        return min(dep(x + dx, y + dy)[0] for dx in range(-4, 5) for dy in range(-4, 5) if dx * dx + dy * dy <= r * r)
    return mm, dep, disc_min

Z_TOP = -152.9                             # the -90 module's back plate outer face is FLAT at Z -152.6..-152.8 across X +-22 (M, probe), not the r 152.5 cylinder; 0.1 clear
# module sides are radial planes at +-12 deg (X 31 at Z -153, 35 at Z -166, M), flush with the lugs; the lug nuts sit inside them
# at Y 141..149 / 205..213, Z below -161, |X| > 27 (C). The cradle body stays inside: walls X +-20.4, a 6 thick lip to X +-30 carries the up-bolts.
def cradle(bolts):
    XW, XL, Y0, Y1 = 20.4, 30.0, 140.0, 197.0           # Y0 140: the plate has a 0.3 rim at Y 137..139 (M)
    body = box(-XW, XW, Y0, Y1, FLOOR, Z_TOP) - box(-XW + WALL, XW - WALL, Y0 - 1, Y1 - 3.0, FLOOR + WALL, Z_TOP - WALL)   # cap / walls / floor 2.4, back 3, front open (C)
    for s in (1, -1):
        body += box(min(s * (XW - 1), s * XL), max(s * (XW - 1), s * XL), Y0, Y1, Z_TOP - 6.0, Z_TOP)   # lips: 6.0 of PETG under each bolt head, heads in open air (C)
    for x, y in bolts:
        body -= cyl(2.65, 30, "z", [x, y, -170])                                          # M5 clearance through the lip
    for s in (1, -1):
        body -= cyl(2.75, 10, "y", [s * 10, Y1 - 6, -191])                                # M5 through the back wall into rail 2's slot (C)
        w = [(190, -190), (172, -190), (172 - 14 * np.tan(np.radians(60)), -176), (172, -162), (190, -162)]   # chevron window, 60 deg sides, apex to the front (up in print)
        body -= prism_yz(w, (XW - WALL - 1) if s > 0 else -XW - 1, (XW + 1) if s > 0 else -(XW - WALL - 1))
    return body

if __name__ == "__main__":
    mm, dep, disc_min = probe_depth()
    cands = []
    for band in ((146, 162), (182, 192)):
        best = max(((min(disc_min(x, y), disc_min(-x, y)), x, y) for x in np.arange(24, 32, 1.0) for y in np.arange(*band, 1.0)))
        cands.append(best); print(f"bolt band Y {band}: best depth {best[0]:.1f} at |X| {best[1]:.0f} Y {best[2]:.0f} (M, min over a r4 disc, both sides)")
    # (+-26, 154): deepest mirrored front spot (12.6, strut face lumpy Z -152.6..-146, M); the lug nut is pocketed in the lug at Y 141..148 (M),
    # the head (Y 149.75..) clears it by 1.25. (+-26, 189): flat, 17.6 deep (M). One lip per side carries both.
    bolts = [(s * x, y) for s in (1, -1) for x, y in ((26.0, 154.0), (26.0, 189.0))]
    for x, y in bolts: print(f"  bolt ({x:+.0f}, {y:.0f}): solid depth {disc_min(x, y):.1f} over a r4 disc, strut face Z {dep(x, y)[1]:.1f} (M)")
    cr = cradle(bolts)
    pad, info = seat_pad()
    out = {}
    # print orientations: cradle on its back (Y 197 -> bed), pad on its Y 131 face
    T_cr = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 197.0]], float)                # (X, Z, 197 - Y)
    T_pad = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, -info["y_bed"]]], float)      # (X, -Z, Y - 131), proper rotation
    padB = pad.mirror([0, 1, 0]).translate([0, 2 * 177.0, 0])                             # R-back (Y mirror about 177)
    T_padB = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 2 * 177.0 - info["y_bed"]]], float)   # its bed face Y 223 -> z 0
    for name, m, T in (("box_bay_cradle_v5e", cr, T_cr), ("box_seat_pad_v5e_A", pad, T_pad), ("box_seat_pad_v5e_B", padB, T_padB)):
        tm = mesh_of(m.transform(T)); tm.fix_normals()
        tm.apply_translation([0, 0, -tm.bounds[0][2]])
        tm.export(name + ".stl"); mesh_of(m).export(name + "_W.stl")
        out[name] = dict(g=round(tm.volume * D, 1), watertight=tm.is_watertight, bbox=np.round(tm.extents, 1).tolist(), bodies=len(tm.split(only_watertight=False)))
        print(name, out[name])
    # fit: cradle top vs the module plate (M), lugs vs walls, pad vs funnel panel
    crW = mesh_of(cr)
    print("cradle top Z", round(crW.bounds[1][2], 2), "(C) vs module plate plane Z", round(float(np.nanmin([dep(x, y)[1] for x in range(-20, 21, 5) for y in range(150, 191, 10)])), 2), "..", round(float(np.nanmax([dep(x, y)[1] for x in range(-20, 21, 5) for y in range(150, 191, 10)])), 2), "(M); cradle height", round(crW.extents[2], 1))
    V = mm.vertices; inside = (np.abs(V[:, 0]) < 30.0) & (V[:, 1] > 140) & (V[:, 1] < 197) & (V[:, 2] < Z_TOP)
    print("module vertices inside the cradle envelope (|X| < 30, Y 140..197, below the top):", int(inside.sum()), "(M)")
    for x in (20.4, 26, 28, 30, 31):
        s_ = (np.abs(np.abs(V[:, 0]) - x) < 0.5) & (V[:, 1] > 140) & (V[:, 1] < 194); print(f"  module lowest Z at |X| {x}: {V[s_, 2].min():.1f} (M)")
    pw = mesh_of(pad).vertices; w = 84 + (pw[:, 2] - 143.6)
    print("seat pad A points inside the funnel panel outer surface:", int(((np.abs(pw[:, 0]) < w + 4.4 - 0.5) & (pw[:, 2] > 143.6) & (pw[:, 2] < 219.6)).sum()), "(C)")
    print("seat pad A extents W:", np.round(mesh_of(pad).bounds, 1).tolist())
    json.dump(dict(bolts=bolts, parts=out, pad=info), open("stand_v5_prints.json", "w"), indent=1)
