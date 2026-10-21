"""TPU foot pad v1 (x4: 2 on the stub spider, 2 on the bay ring). python pad_v1.py -> out/foot_pad_v1.stl
Prints flat, contact face on the bed, stem up, no supports (every overhang <= 45 deg). TPU 95A.
Seat it snaps into (same on the spider and the ring, DEFAULT): pocket 30.6 x 30.6 x 1 in the land face, 4-mm floor with a
Ø6.4 hole, Ø11 x 6 cavity behind it. Pad 30 x 30 x 4 (E): 1 in the pocket, 3 proud -> land face 3 above the floor.
Barb Ø8.6 squeezes through Ø6.4 (TPU), slotted 1.2 so it folds; neck Ø6 x 4 = the floor."""
import os
import numpy as np
import manifold3d as mf

PAD, T, R_CORNER, CH = 30.0, 4.0, 3.0, 0.6        # E size, DEFAULT corner radius and bed-edge chamfer
NECK_D, NECK_L = 6.0, 4.0                          # = seat floor
BARB_D, FLARE, TAPER, TIP_D, SLOT = 8.6, 1.3, 3.0, 5.0, 1.2


def rounded_square(s, r, z0, z1, n=48):
    c = mf.CrossSection.square([s - 2 * r, s - 2 * r], center=True).offset(r, mf.JoinType.Round, circular_segments=n)
    return c.extrude(z1 - z0).translate([0, 0, z0])


def cone(z0, z1, d0, d1, n=64):
    return mf.Manifold.cylinder(z1 - z0, d0 / 2, d1 / 2, n).translate([0, 0, z0])


def pad():
    # 45 deg chamfer on the bed edge: hull of the two squares
    body =mf.Manifold.batch_hull([rounded_square(PAD - 2 * CH, R_CORNER - CH, 0, 0.01),
                                   rounded_square(PAD, R_CORNER, CH, CH + 0.01)]) + rounded_square(PAD, R_CORNER, CH, T)
    z = T
    stem = cone(z - 0.01, z + NECK_L, NECK_D, NECK_D)
    flare = cone(z + NECK_L, z + NECK_L + FLARE, NECK_D, BARB_D)            # 45 deg (1.3 out over 1.3 up)
    taper = cone(z + NECK_L + FLARE, z + NECK_L + FLARE + TAPER, BARB_D, TIP_D)
    top = z + NECK_L + FLARE + TAPER
    slot = mf.Manifold.cube([BARB_D + 2, SLOT, top - (z + NECK_L - 1)]).translate([-(BARB_D + 2) / 2, -SLOT / 2, z + NECK_L - 1])
    return body + stem + flare + taper - slot


if __name__ == "__main__":
    import trimesh
    p = pad()
    me = p.to_mesh()
    t = trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts))
    os.makedirs("out", exist_ok=True); t.export("out/foot_pad_v1.stl")
    bb = p.bounding_box()
    print(f"foot pad v1: {p.volume() / 1000 * 1.22:.1f} g TPU 95A (M), bbox {[round(bb[i + 3] - bb[i], 1) for i in range(3)]}, "
          f"bodies {len(p.decompose())}, watertight {t.is_watertight}")
