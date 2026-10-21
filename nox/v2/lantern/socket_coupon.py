"""Socket v2 coupon (20 Oct 2026): 3 bore sizes (valley 3.5 / 3.6 / 3.7, ribs 0.3 under each), each standing (axis z, as the
column sockets print) and lying (axis x, worst case: the hole prints oval). Notches on the plate edge: 1 / 2 / 3 = 3.5 / 3.6 / 3.7.
Test: dry fit a 3.0 rod in each (should slide in by hand, no wobble), then epoxy one rod in the 3.6 pair, 24 h, pull by hand.
Print flat on the plate, PA12-CF (or PETG for the fit check only)."""
import copy, numpy as np, manifold3d as mf, node_topo2 as N
parts = mf.Manifold.cube([66, 30, 2.0]).translate([0, 0, 0])
cuts = mf.Manifold()
for i, v in enumerate((3.5, 3.6, 3.7)):
    S = dict(N.SOCK); S["valley"] = v; S["rib_d"] = v - 0.3
    b, c, z, pth = N.socket_parts("end", S=S)
    x = 11 + 22 * i
    parts += b.translate([x, 22, 2.0]); cuts += c.translate([x, 22, 2.0]); paths = pth.translate([x, 22, 2.0])                 # standing: bottom on the plate
    bl, cl = N.orient(b, [0, -1, 0], [0, 0, 0]), N.orient(c, [0, -1, 0], [0, 0, 0])
    parts += bl.translate([x, 13, 2.0 + 3.6]); cuts += cl.translate([x, 13, 2.0 + 3.6])     # lying, axis -y (mouth at the front edge), sits on the plate
    for k in range(i + 1):                                                                   # size notches on the front edge
        cuts += mf.Manifold.cube([1.2, 1.6, 3]).translate([x + 5 + 2.0 * k, -0.01, -0.5])
cuts += mf.Manifold.cube([200, 200, 50]).translate([-50, -50, -50])                          # nothing below the bed
part = parts - cuts
t = N.tm(part); t.export("socket_v2_coupon.stl")
print(f"coupon {t.volume/1000*1.19:.2f} g PA12-CF, bbox {np.round(t.extents,1)}, watertight {t.is_watertight}, bodies {len(t.split(only_watertight=False))}")
