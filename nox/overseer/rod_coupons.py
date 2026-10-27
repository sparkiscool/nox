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

python3 nox/overseer/rod_coupons.py  -> print/rod_coupon_6mm_v1.stl, print/rod_coupon_3mm_v1.stl
"""
import os
import numpy as np
import manifold3d as mf
import trimesh

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
STOCKS = {"6mm": dict(od=6.0, bores=(6.30, 6.45, 6.60), depth=15.0, vent=1.6, pitch=20.0),
          "3mm": dict(od=3.0, bores=(3.30, 3.45, 3.60), depth=10.0, vent=1.2, pitch=14.0)}
WALL, BASE, GROOVE_D, GROOVE_H, CHAMFER = 2.5, 3.0, 0.5, 2.0, 1.0


def cyl(r0, r1, h, z0=0.0, n=96):
    return mf.Manifold.cylinder(h, r0, r1, n).translate([0, 0, z0])


def build(name, s):
    n = len(s["bores"]); P = s["pitch"]; ro = max(s["bores"]) / 2 + WALL; H = BASE + s["depth"]
    body = mf.Manifold.cube([P * (n - 1) + 2 * ro + 4.0, 2 * ro + 8.0, BASE]).translate([-ro - 2.0, -ro - 4.0, 0])
    cuts = None
    add = lambda m: m if cuts is None else cuts + m
    for k, b in enumerate(s["bores"]):
        x = k * P; r = b / 2
        body = body + cyl(ro, ro, H).translate([x, 0, 0])
        c = cyl(r, r, s["depth"] + 0.01, BASE) + cyl(r, r + CHAMFER, CHAMFER + 0.01, H - CHAMFER)
        for zf in (BASE + s["depth"] * 0.3, BASE + s["depth"] * 0.65):
            c = c + cyl(r + GROOVE_D, r + GROOVE_D, GROOVE_H - GROOVE_D, zf) + cyl(r + GROOVE_D, r, GROOVE_D, zf + GROOVE_H - GROOVE_D)
        c = c + cyl(s["vent"] / 2, s["vent"] / 2, BASE + 0.02, -0.01)
        cuts = add(c.translate([x, 0, 0]))
        for d in range(k + 1):
            cuts = cuts + cyl(0.9, 0.9, 0.61, BASE - 0.6).translate([x + (d - k / 2) * 3.0, -ro - 2.0, 0])
    m = (body - cuts).to_mesh()
    tm = trimesh.Trimesh(np.asarray(m.vert_properties)[:, :3], np.asarray(m.tri_verts), process=True)
    out = os.path.join(ROOT, "print", f"rod_coupon_{name}_v1.stl"); tm.export(out)
    print(out, "extents", np.round(tm.extents, 1), "volume", round(tm.volume / 1000, 2), "cm3", "watertight", tm.is_watertight)
    return out


if __name__ == "__main__":
    for name, s in STOCKS.items(): build(name, s)
