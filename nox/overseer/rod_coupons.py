"""Rod socket coupons v1 (27 Oct, Spark: "make coupons for the 6mm and the 3mm rods, make it so it lets resin make the best hold
on the rods and print 3 per with small difference in size so I can tell you what we want").

One coupon a stock, 3 sockets each, bores stepping 0.15 from the rule's OD + 0.3 up (an FDM hole prints ~0.1 to 0.2 under, so the
fit is chosen on the printed part): 6 mm tube 6.30 / 6.45 / 6.60, 15 deep; 3 mm rod 3.30 / 3.45 / 3.60, 10 deep (the node and
head-mount socket depths). For the epoxy:
  - two glue rings inside each socket (0.5 deep, 2 tall, 45 deg roof so they print without support): the cured epoxy keys into
    them, so the joint holds on shape, not only on adhesion (epoxy grips TPU poorly);
  - a vent through the floor (Ø1.6 / Ø1.2) so air and extra epoxy leave and the socket fills to the bottom;
  - a 1 x 45 deg entry chamfer: a small resin reservoir and an easy start for the rod;
  - walls 2.5 round every socket (as the nodes), on a 3 mm base plate.
Size marks: 1, 2, 3 dots on the base beside each socket, smallest bore first.
Print sockets up, base down (the largest flat face), in the node material (TPU; PETG too to test the head-mount sockets).

v2 (27 Oct, Spark: "the coupons are too big the smallest one is still loose print the smaller series, also are we making sure we are
accounting for ones bring printed sideways being the same"): bores 6.00 / 6.10 / 6.20 and 3.00 / 3.10 / 3.20 (the v1 6.30 / 3.30 were
loose in PETG), and each bore twice: a row standing up (axis = build z, a round hole) and a row lying sideways (axis along the bed:
layer steps and a sagging roof print it tighter top to bottom). The node and head sockets point every way, so the pick is made for
both and a socket between takes the bore by its angle to the bed.

python3 nox/overseer/rod_coupons.py [v1|v2]  -> print/rod_coupon_<stock>_<v>.stl
"""
import os, sys
import numpy as np
import manifold3d as mf
import trimesh

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
STOCKS = {"v1": {"6mm": dict(od=6.0, bores=(6.30, 6.45, 6.60), depth=15.0, vent=1.6, pitch=20.0),
                 "3mm": dict(od=3.0, bores=(3.30, 3.45, 3.60), depth=10.0, vent=1.2, pitch=14.0)},
          "v2": {"6mm": dict(od=6.0, bores=(6.00, 6.10, 6.20), depth=15.0, vent=1.6, pitch=20.0, side=True),
                 "3mm": dict(od=3.0, bores=(3.00, 3.10, 3.20), depth=10.0, vent=1.2, pitch=14.0, side=True)}}
WALL, BASE, GROOVE_D, GROOVE_H, CHAMFER = 2.5, 3.0, 0.5, 2.0, 1.0


def cyl(r0, r1, h, z0=0.0, n=96):
    return mf.Manifold.cylinder(h, r0, r1, n).translate([0, 0, z0])


def socket_cut(r, s):
    """one socket's cut along +z: floor at z 0, mouth (chamfer) at z depth, two glue rings, vent down through the floor"""
    D = s["depth"]
    c = cyl(r, r, D + 0.01, 0.0) + cyl(r, r + CHAMFER, CHAMFER + 0.01, D - CHAMFER)
    for zf in (D * 0.3, D * 0.65):
        c = c + cyl(r + GROOVE_D, r + GROOVE_D, GROOVE_H - GROOVE_D, zf) + cyl(r + GROOVE_D, r, GROOVE_D, zf + GROOVE_H - GROOVE_D)
    return c + cyl(s["vent"] / 2, s["vent"] / 2, BASE + 0.02, -BASE - 0.01)


def dots(n, x, y, z):
    """n dots (Ø1.8, 0.6 deep) centred on x, in the face at height z"""
    out = None
    for d in range(n):
        m = cyl(0.9, 0.9, 0.61, z - 0.6).translate([x + (d - (n - 1) / 2) * 3.0, y, 0]); out = m if out is None else out + m
    return out


def build(name, s, ver):
    n = len(s["bores"]); P = s["pitch"]; ro = max(s["bores"]) / 2 + WALL; H = BASE + s["depth"]
    y1 = ro + 4.0 if not s.get("side") else ro + 4.0 + BASE + s["depth"]          # the sideways row behind the standing one
    body = mf.Manifold.cube([P * (n - 1) + 2 * ro + 4.0, y1 + ro + 4.0, BASE]).translate([-ro - 2.0, -ro - 4.0, 0])
    cuts = []
    for k, b in enumerate(s["bores"]):
        x = k * P; r = b / 2
        body = body + cyl(ro, ro, H).translate([x, 0, 0])                          # standing: axis up
        cuts += [socket_cut(r, s).translate([x, 0, BASE]), dots(k + 1, x, -ro - 2.0, BASE)]
        if s.get("side"):                                                          # sideways: axis along +y, mouth on the back face
            Y0 = ro + 4.0 + BASE                                                   # its floor; the vent leaves forward into the gap
            body = body + mf.Manifold.cube([2 * ro, BASE + s["depth"], 2 * ro]).translate([x - ro, Y0 - BASE, 0])
            cuts += [socket_cut(r, s).rotate([-90, 0, 0]).translate([x, Y0, ro]), dots(k + 1, x, Y0 + s["depth"] * 0.5, 2 * ro)]
    cut = cuts[0]
    for c in cuts[1:]: cut = cut + c
    m = (body - cut).to_mesh()
    tm = trimesh.Trimesh(np.asarray(m.vert_properties)[:, :3], np.asarray(m.tri_verts), process=True)
    out = os.path.join(ROOT, "print", f"rod_coupon_{name}_{ver}.stl"); tm.export(out)
    print(out, "extents", np.round(tm.extents, 1), "volume", round(tm.volume / 1000, 2), "cm3", "watertight", tm.is_watertight)
    return out


if __name__ == "__main__":
    ver = sys.argv[1] if len(sys.argv) > 1 else "v2"
    for name, s in STOCKS[ver].items(): build(name, s, ver)
