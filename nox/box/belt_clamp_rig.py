import json, base64, math, numpy as np, manifold3d as mf
import belt_clamp as B

cb, pl = B.coupon()
uc = B.END_L + B.LAND_L / 2          # 20
# M3 heat-set insert (brass, E): ring Ø4 / Ø3 bore with two knurl bands, from the land face into the land
ins = B.cyl_p(3.0, uc, B.P_RIDGE, B.P_RIDGE + B.INSERT_L, B.INSERT_D / 2, 32)
ins -= B.cyl_p(3.0, uc, B.P_RIDGE - 1, B.P_RIDGE + B.INSERT_L + 1, 1.5, 32)
for a, b in ((0.9, 1.6), (3.4, 4.1)):
    ins -= (B.cyl_p(3.0, uc, B.P_RIDGE + a, B.P_RIDGE + b, 2.3, 32) - B.cyl_p(3.0, uc, B.P_RIDGE + a, B.P_RIDGE + b, 1.85, 32))
# M3 x 8 socket head: head Ø5.5 x 3 sunk in the counterbore, hex socket 2.5 AF
p_head0 = B.P_FL[0]
scr = B.cyl_p(3.0, uc, p_head0, p_head0 + 3.0, 2.75, 40)
scr += B.cyl_p(3.0, uc, p_head0 + 3.0, p_head0 + 3.0 + 8.0, 1.5, 24)
hexs = mf.Manifold.cylinder(2.0, 2.5 / math.sqrt(3), 2.5 / math.sqrt(3), 6).rotate([0, 90, 0]).translate([p_head0 - 0.5, 3.0, uc])
scr -= hexs
# belt (GT2 6 mm, spec E): one end seated in the 8 grooves, butted on the land, tail hanging down
e0, e1 = -46.0, B.END_L
belt = B.box(B.P_RIDGE - B.BACK, B.P_RIDGE, 0.0, B.BELT_W, e0, e1)
uu = 1.0 - 2.0 * 23
while uu < e1:
    if uu - B.TOOTH_W / 2 >= e0 and uu + B.TOOTH_W / 2 <= e1:
        r = B.TOOTH_W / 2
        t = B.box(B.P_RIDGE - 0.01, B.P_RIDGE + B.TOOTH_H - r, 0.0, B.BELT_W, uu - r, uu + r)
        t += mf.Manifold.cylinder(B.BELT_W, r, r, 20).rotate([-90, 0, 0]).translate([B.P_RIDGE + B.TOOTH_H - r, 0.0, uu])
        belt += t
    uu += 2.0
# hang pin: M6 bolt through the Ø6.5 tab hole (u 33), head on the back face
u_tab = B.END_L + B.LAND_L + 9.0
pin = B.cyl_p(3.0, u_tab, -6.0, B.P_BED1 + 0.01, 3.0, 40)
pin += mf.Manifold.cylinder(4.0, 5.77, 5.77, 6).rotate([0, 90, 0]).translate([B.P_BED1, 3.0, u_tab])
pin += mf.Manifold.cylinder(5.2, 5.0, 5.0, 6).rotate([0, 90, 0]).translate([-6.0 - 5.2, 3.0, u_tab])  # nut

parts = dict(coupon=cb, plate=pl, insert=ins, screw=scr, belt=belt, pin=pin)
out = {}
for k, m in parts.items():
    me = m.to_mesh()
    v = np.asarray(me.vert_properties)[:, :3].astype(np.float64)
    f = np.asarray(me.tri_verts).astype(np.uint32)
    # display frame: x = w - 3, y = u, z = -p (plate side toward the viewer)
    d = np.c_[v[:, 1] - 3.0, v[:, 2], -v[:, 0]].astype(np.float32)
    out[k] = dict(v=base64.b64encode(d.tobytes()).decode(), f=base64.b64encode(f.tobytes()).decode(),
                  nt=len(f), vol=round(m.volume(), 1))
    print(k, len(f), "tris", round(m.volume(), 1), "mm3", [round(x, 2) for x in m.bounding_box()])
# checks: interferences in the assembled state
for a, b in (("belt", "coupon"), ("belt", "plate"), ("plate", "coupon"), ("screw", "plate"), ("insert", "coupon")):
    print(a, "x", b, round((parts[a] ^ parts[b]).volume(), 3))
json.dump(out, open("asm.json", "w"))
import os; print(os.path.getsize("asm.json") / 1e3, "kB")
