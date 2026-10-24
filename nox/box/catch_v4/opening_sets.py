"""Funnel v8 opening vs the real latch v1 sets (24 Oct, PR 81 review): the Overseer and Scout receiver + lid (on the drone)
and cartridge (swept down the swap path) from runs/box_system/v2/latch_ctx/ (M, battery latch v1 PR 69), each centred on
its own set centre along drone x (hub DEFAULT 24 Oct + arm PR 79: each drone's catch centre moves onto its latch set's
centre; E until the Overseer chat moves its wheel nodes), swept over yaw -5 .. +5 deg and the XT60 lift 0 .. 5.7 (C),
against the collar as the system's assembly file has it (nox/box/box_catch.assembly.json: panels v7 on +-y, v8 plain on
+-x, taken back to the seat frame by catch_system.SEAT_TO_W), each face's panels apart.
Placement (box_layout_v2.place at the drone slot, C): cartridge coords c (x across, y up the install axis from the flat
bottom, z from the closed end) -> seat frame: seat x = -(z - z_centre) (drone x along the set), seat y = -x, seat z =
y + R - Z_SEAT = y - 42.3 (R 113, Z_SEAT 155.3, C).
v9 plain (24 Oct): the sets placed as the box structure layout places them (box_layout_v2.place, PR 86: each latch's tab
line Z_T on the grip line Y_C, so the Scout receiver sits at W Y 110.4..211.7, its mid 3.95 off Y_C, the Overseer's at
75.4..254.7; not forced to the set centre), the +-x faces' panels with their keep-alive probe pins out at free (tip 8.0
off the face) and caps, from the assembly file.
python3 opening_sets.py [--centred] -> runs/funnel_panel/v9_plain/opening_sets.json (--centred: the v8 check, each set on
its own centre)"""
import json, os, sys
import numpy as np, trimesh, manifold3d as mf

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
CTX = os.path.join(ROOT, "runs", "box_system", "v2", "latch_ctx")
RUN = os.path.join(ROOT, "runs", "funnel_panel", "v9_plain")
R_RING, Z_SEAT = 113.0, 155.3                    # box structure v2 (C)
DZ0 = R_RING - Z_SEAT                            # cartridge flat bottom at seat z -42.3 (C)
LIFT = 5.7                                       # XT60 push lifts the drone up to 5.7 (C, dimensions G hook latch)
SWEEP = 120.0                                    # cartridge swept down the swap path, well below the collar (z -1.2)
YAWS = np.arange(-5.0, 5.01, 0.5)
LIFTS = (0.0, 1.9, 3.8, LIFT)


def man(t):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))


def load(pk, k):
    return trimesh.load(os.path.join(CTX, f"{pk}_{k}_c.stl"), force="mesh")


def to_seat(t, zc):
    M = np.array([[0, 0, -1, zc], [-1, 0, 0, 0], [0, 1, 0, DZ0], [0, 0, 0, 1.0]])
    t = t.copy(); t.apply_transform(M); return t


def main():
    sys.path[:0] = [os.path.dirname(HERE), os.path.join(ROOT, "nox")]
    import assembly as AS, catch_system as CS
    a = AS.load(os.path.join(os.path.dirname(HERE), "box_catch.assembly.json"))
    a = a.at(**{n: -6.0 for n in a.names if n.startswith("probe pin")})         # probe pins out at free (tip 8.0 off the face)
    Wi = np.linalg.inv(CS.SEAT_TO_W)
    meta = json.load(open(os.path.join(CTX, "meta.json")))
    centred = "--centred" in sys.argv
    sides = {}
    for face in ("+x", "-x", "+y", "-y"):
        ms = [m for p, m in zip(a.parts, a.meshes()) if (p["name"].startswith("funnel panel") or p["name"].startswith("probe"))
              and face in p["name"].split()]
        t = trimesh.util.concatenate(ms); t.apply_transform(Wi)
        sides[face] = man(t)
    out = {}
    for pk in ("overseer", "scout"):
        rec = load(pk, "receiver"); lid = load(pk, "receiver_lid"); cart = load(pk, "cartridge")
        rb = trimesh.util.concatenate([rec, lid]).bounds
        zc = float(rb[:, 2].mean()) if centred else float(meta[pk]["Z_T"])   # set centre (v8) or the tab line on Y_C (layout, v9)
        R = man(to_seat(rec, zc)) + man(to_seat(lid, zc))
        C = man(to_seat(cart, zc))
        C = mf.Manifold.batch_hull([C, C.translate([0, 0, -SWEEP])])
        rr = rec.bounds
        row = dict(placement="set centre on the catch centre" if centred else "tab line Z_T on Y_C (box_layout_v2.place, PR 86)",
                   z_c_on_catch_centre=round(zc, 2), receiver_W_Y=[round(165.0 - (float(rr[1, 2]) - zc), 1), round(165.0 - (float(rr[0, 2]) - zc), 1)],
                   receiver_mid_off_Y_C=round(-(float(rr[:, 2].mean()) - zc), 2), receiver_len=round(float(rb[1, 2] - rb[0, 2]), 1),
                   receiver_w=round(float(rb[1, 0] - rb[0, 0]), 1),
                   receiver_seat_z=[round(float(rb[0, 1] + DZ0), 1), round(float(rb[1, 1] + DZ0), 1)])
        worst = {s: 1e9 for s in sides}; ov = 0.0; ext = 0.0
        for yaw in YAWS:
            for dz in LIFTS:
                S = (R + C).rotate([0, 0, float(yaw)]).translate([0, 0, dz])
                bb = np.array(S.bounding_box()).reshape(2, 3); ext = max(ext, float(np.abs(bb[:, 0]).max()))
                for s, m in sides.items():
                    ov += (S ^ m).volume()
                    worst[s] = min(worst[s], S.min_gap(m, 40.0))
        row.update(yaws_deg=[float(YAWS[0]), float(YAWS[-1]), 0.5], lift_mm=list(LIFTS), swept_down_mm=SWEEP,
                   overlap_mm3=round(ov, 3), min_gap_mm={s: round(v, 2) for s, v in worst.items()},
                   max_abs_seat_x=round(ext, 2), passes_3_a_side=bool(ov < 1e-6 and min(worst.values()) >= 3.0))
        out[pk] = row
    out["note"] = ("min_gap = least distance from the set (receiver + lid on the drone, cartridge swept down 120) to that "
                   "face's panels (assembly file meshes) over the yaw (0.5 deg steps) and lift sweep (manifold3d min_gap, C on "
                   "M meshes); 40.0 = nothing within the 40 search")
    out["sides_include"] = "funnel panels per face (v7 on +-y, v9 plain on +-x) + on +-x the probe caps and pins at free"
    json.dump(out, open(os.path.join(RUN, "opening_sets_centred.json" if centred else "opening_sets.json"), "w"), indent=1)
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
