"""Harness printed parts v1 (17 Oct 2026, wiring chat): wire_grommet, end_clip x2, slot_clip x6.
nox/box-wiring.md Parts section; numbers C unless tagged E/CHECK. Frames:
- grommet: own frame, bore along Z.
- end_clip: bar frame = arm frame for bar A (X tangential +-20, Y axial 74..94, Z along the bar,
  end face at Z 22, axle axis = the Y axis). Same part fits bar B (both frames put the belt on +X).
- slot_clip: slot frame, length along X, surface at Z 0, slot below.
The saddle/beam placement is CHECKED here against the fixed parts (all bodies of revolution about
the axle axis above Y 54, so one static distance check covers every arm angle):
  80T: flange r 27.5 Y 54..55.5, body r 25.8 Y 54..64 (E split), collar r 15 Y 64..72 (P)
  detent ring: r 23.5, Y 64..69.9 (C)
Rule: every rotating-to-fixed gap >= 1.0 (CLR_POST). Roller arm / tower / bands are arm-side
(rotate with the clip): static separation only, detent at -105 deg vs clip at 90/180 deg.
"""
import numpy as np, trimesh
from manifold3d import Manifold, set_circular_segments

set_circular_segments(96)

def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])

def cyl(r, h, axis="z"):
    m = Manifold.cylinder(h, r, r)
    if axis == "y": m = m.rotate([-90, 0, 0]).translate([0, 0, 0])
    if axis == "x": m = m.rotate([0, 90, 0])
    return m

def to_mesh(m):
    mm = m.to_mesh()
    return trimesh.Trimesh(np.asarray(mm.vert_properties)[:, :3], np.asarray(mm.tri_verts), process=False)

PETG = 1.27e-3  # g/mm3

# ---------------- wire_grommet ----------------
def grommet():
    body = cyl(23.8 / 2, 8).translate([0, 0, 3])          # press section, 3 crush ribs 0.2 proud
    for a in (0, 120, 240):
        rib = box(23.8 / 2 - 0.5, 23.8 / 2 + 0.2, -0.75, 0.75, 3, 11)
        body += rib.rotate([0, 0, a])
    flange = cyl(28 / 2, 3)
    g = body + flange
    g -= cyl(7.5 / 2, 20).translate([0, 0, -1])            # bore: bundle 7.3 lightly held
    g -= box(0, 15, -1.0, 1.0, -1, 12)                     # split slit 2.0: opens over the finished bundle
    for s in (1, -1):                                      # 2 zip slots through the flange (lash the bundle)
        g -= box(-1.75, 1.75, s * 9 - 1, s * 9 + 1, -1, 4)
    return g

# ---------------- end_clip (bar frame) ----------------
SADDLE_Y = (78.0, 86.0)
def end_clip():
    plate = box(-20, 20, 74, 94, 17, 22)
    beam = box(-5, 5, SADDLE_Y[0], SADDLE_Y[1], 4, 17.5)
    cradle = (cyl(6, SADDLE_Y[1] - SADDLE_Y[0], "y") - cyl(4, 30, "y").translate([0, -10, 0])
              ).translate([0, SADDLE_Y[0], 0])
    cradle -= box(0, 8, SADDLE_Y[0] - 1, SADDLE_Y[1] + 1, -3.2, 3.2)   # 90 deg opening toward +X
    c = plate + beam + cradle
    for yz in (79.7, 84.3):                                # 2 zip grooves round the cradle, 2.2 x 1.2
        ring = (cyl(6.01, 2.2, "y") - cyl(4.8, 4, "y").translate([0, -1, 0])).translate([0, yz - 1.1, 0])
        c -= ring
    for sx in (-10, 10):                                   # 2 x M5 self-tap into the 2040 core holes
        c -= cyl(5.5 / 2, 10, "z").translate([sx, 84, 14])
        c -= cyl(9.8 / 2, 4, "z").translate([sx, 84, 13.9])  # counterbore (button head)
    # ribbon guide groove 4.4 x 2 up the front face at X -10, cradle level to the top edge
    c -= box(-12.2, -7.8, 82, 95, 15.0, 17.0)
    # lash slot for the stepper tail near the +X edge
    c -= box(13.25, 16.75, 77, 79, 16, 23)
    return c

# ---------------- slot_clip (cover segment; slot profile E, CHECK by hand test) ----------------
def slot_clip():
    wings = box(-5, 5, -4.3, 4.3, -2.4, -1.8)              # 8.6 wide x 0.6, rides the lip undercut
    spine = box(-5, 5, -2.8, 2.8, -1.8, -0.3)              # 5.6 wide, top 0.3 below the bar surface
    c = wings + spine
    c -= box(3.2, 5.6, -5, 5, -3, 0).rotate([0, -30, 0]).translate([1.8, 0, 0])   # lead chamfer
    return c

# ---------------- clearance check (end_clip vs fixed, any arm angle) ----------------
def check(clip_mesh):
    fixed = [("80T flange", 27.5, (54.0, 55.5)), ("80T body", 25.8, (54.0, 64.0)),
             ("80T collar", 15.0, (64.0, 72.0)), ("detent ring", 23.5, (64.0, 69.9))]
    v, _ = trimesh.sample.sample_surface_even(clip_mesh, 20000)
    v = np.vstack([v, clip_mesh.vertices])
    out = {}
    for name, r, (y0, y1) in fixed:                        # distance point -> cylinder (axis = Y)
        rad = np.hypot(v[:, 0], v[:, 2])
        dr = np.maximum(rad - r, 0)
        dy = np.maximum(np.maximum(y0 - v[:, 1], v[:, 1] - y1), 0)
        inside = (rad <= r) & (dy == 0)
        d = np.where(inside, -1.0, np.where((rad <= r), dy, np.where(dy == 0, dr, np.hypot(dr, dy))))
        out[name] = round(float(d.min()), 2)
    return out


if __name__ == "__main__":
    parts = dict(wire_grommet=grommet(), end_clip=end_clip(), slot_clip=slot_clip())
    meshes = {}
    for n, m in parts.items():
        tm = to_mesh(m)
        tm.export(f"/home/claude/{n}.stl")
        meshes[n] = tm
        print(f"{n:14s} watertight={tm.is_watertight} vol={tm.volume:8.0f} mm3  "
              f"mass={tm.volume * PETG:5.1f} g  bbox={np.round(tm.extents, 1)}")
    print("end_clip clearances to fixed (any angle, mm):", check(meshes["end_clip"]))
