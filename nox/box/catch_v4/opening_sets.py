"""Funnel v8 opening vs the real latch v1 sets (24 Oct, PR 81 review): the Overseer and Scout receiver + lid (on the drone)
and cartridge (swept down the swap path) from runs/box_system/v2/latch_ctx/ (M, battery latch v1 PR 69), each centred on
its own set centre along drone x (hub DEFAULT 24 Oct + arm PR 79: each drone's catch centre moves onto its latch set's
centre; E until the Overseer chat moves its wheel nodes), swept over yaw -5 .. +5 deg and the XT60 lift 0 .. 5.7 (C),
against the collar (panels v7 on +-y, v8 plain on +-x; runs/funnel_panel/v8_plain/collar_v8_seat_frame.stl).
Placement (box_layout_v2.place at the drone slot, C): cartridge coords c (x across, y up the install axis from the flat
bottom, z from the closed end) -> seat frame: seat x = -(z - z_centre) (drone x along the set), seat y = -x, seat z =
y + R - Z_SEAT = y - 42.3 (R 113, Z_SEAT 155.3, C).
python3 opening_sets.py -> runs/funnel_panel/v8_plain/opening_sets.json"""
import json, os
import numpy as np, trimesh, manifold3d as mf

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
CTX = os.path.join(ROOT, "runs", "box_system", "v2", "latch_ctx")
RUN = os.path.join(ROOT, "runs", "funnel_panel", "v8_plain")
R_RING, Z_SEAT = 113.0, 155.3                    # box structure v2 (C)
DZ0 = R_RING - Z_SEAT                            # cartridge flat bottom at seat z -42.3 (C)
LIFT = 5.7                                       # XT60 push lifts the drone up to 5.7 (C, dimensions G hook latch)
SWEEP = 120.0                                    # cartridge swept down the swap path, well below the collar (z -1.2)
YAWS = np.arange(-5.0, 5.01, 1.0)


def man(t):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))


def load(pk, k):
    return trimesh.load(os.path.join(CTX, f"{pk}_{k}_c.stl"), force="mesh")


def to_seat(t, zc):
    M = np.array([[0, 0, -1, zc], [-1, 0, 0, 0], [0, 1, 0, DZ0], [0, 0, 0, 1.0]])
    t = t.copy(); t.apply_transform(M); return t


def main():
    col = man(trimesh.load(os.path.join(RUN, "collar_v8_seat_frame.stl"), force="mesh"))
    sides = {}
    for nm, (a, b) in {"+x": (1, 0), "-x": (-1, 0), "+y": (0, 1), "-y": (0, -1)}.items():
        # the face's own quadrant, split on the two valley planes
        sides[nm] = col.trim_by_plane([a + b, b - a, 0.0], 0.0).trim_by_plane([a - b, b + a, 0.0], 0.0)
    out = {}
    for pk in ("overseer", "scout"):
        rec = load(pk, "receiver"); lid = load(pk, "receiver_lid"); cart = load(pk, "cartridge")
        rb = trimesh.util.concatenate([rec, lid]).bounds
        zc = float(rb[:, 2].mean())                       # set centre along its length (receiver + lid, M)
        R = man(to_seat(rec, zc)) + man(to_seat(lid, zc))
        C = man(to_seat(cart, zc))
        C = mf.Manifold.batch_hull([C, C.translate([0, 0, -SWEEP])])
        row = dict(set_centre_z_c=round(zc, 2), receiver_len=round(float(rb[1, 2] - rb[0, 2]), 1),
                   receiver_w=round(float(rb[1, 0] - rb[0, 0]), 1),
                   receiver_seat_z=[round(float(rb[0, 1] + DZ0), 1), round(float(rb[1, 1] + DZ0), 1)])
        worst = {s: 1e9 for s in sides}; ov = 0.0; ext = 0.0
        for yaw in YAWS:
            for dz in (0.0, LIFT):
                S = (R + C).rotate([0, 0, float(yaw)]).translate([0, 0, dz])
                bb = np.array(S.bounding_box()).reshape(2, 3); ext = max(ext, float(np.abs(bb[:, 0]).max()))
                for s, m in sides.items():
                    ov += (S ^ m).volume()
                    worst[s] = min(worst[s], S.min_gap(m, 40.0))
        row.update(yaws_deg=[float(YAWS[0]), float(YAWS[-1])], lift_mm=[0.0, LIFT], swept_down_mm=SWEEP,
                   overlap_mm3=round(ov, 3), min_gap_mm={s: round(v, 2) for s, v in worst.items()},
                   max_abs_seat_x=round(ext, 2), passes_3_a_side=bool(ov < 1e-6 and min(worst.values()) >= 3.0))
        out[pk] = row
    out["note"] = ("min_gap = least distance from the set (receiver + lid on the drone, cartridge swept down 120) to that "
                   "face's panels over the yaw and lift sweep (manifold3d min_gap, C on M meshes)")
    json.dump(out, open(os.path.join(RUN, "opening_sets.json"), "w"), indent=1)
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
