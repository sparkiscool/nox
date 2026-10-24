"""Heads A / B v7: the drawn printed parts of the grabber (24 Oct 2026, heads v7 part session), from head_v7_layout.py:
  - carrier +X and carrier -X (PETG): blade 3 x 10 with the 0.8 rib into the tab's grip pocket, boss 12 long on the two 3.0 carbon
    rods (bores 3.3), the inward stop face, the 623ZZ roller lug (3.0 carbon pin, epoxied, cantilevered 4.5), the spring arm and seat
    on the spring rod (bore 3.5);
  - cam (PETG): the two-fold profile (base 4.0, lobe 10.5), 6 thick, on a clamping metal 25T disc horn (radial clamp screw), 2 x M2.5 x 10
    at r 7 in the lobes, M2.5 (horn hole pattern E, CHECK on hand: drill the horn if its holes differ);
  - rack bar (PETG): the m1 rack (tips X 46.5, root 44.25), 5.0 x 14 x 93 (A) / 84 (B), printed on its Y 73.25 face (tooth profile in
    the layers), epoxied along its back face into the body's web (arm_v5_cad tooth phase).
Head B uses the same printed parts mirrored (its carriers are A's turned over: carrier +X of B = mirror of A's, printed as the mirror).
python3 head_v7_parts.py [A|B] -> runs/head/v7/parts/<name>.stl (head frame) and runs/head/v7/parts/print/<name>.stl (print pose),
solid_check and print_orient per file."""
import os, sys, math, json
import numpy as np, trimesh, manifold3d as mf
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
import head_v7_layout as H
from arm_cad import box, cylX, cylY, cylZ, hull, tm, mirror_x
OUT = os.path.join(H.ROOT, "runs", "head", "v7", "parts"); os.makedirs(os.path.join(OUT, "print"), exist_ok=True)


def carrier(sign):
    """printed carrier (A frame, squeezed pose): the layout's carrier with the roller pin bore and the blade tip chamfers"""
    c = H.carrier(sign, open_=False)
    # the roller pin: blind 3.0 bore in the lug, from the roller side, 6 deep (epoxied carbon pin)
    xr = sign * H.ROLLER["x_sq"]
    c -= cylY(xr, H.U_C, H.ROLLER["y"][1] - 0.01, H.ROLLER["y"][1] + 0.5 + 6.0, 1.5, 24)
    # blade tip chamfers (0.5 on the top edges)
    xi = H.BLADE["inner_sq"]; xo = xi + H.BLADE["t"]; ut = H.BLADE["tip_u"]
    for xe in (xi, xo):
        x = sign * xe; d = -sign if xe == xo else sign
        c -= hull([[x - d * 0.01, y, ut + 0.01] for y in (150, 180)] + [[x + d * 0.6, y, ut + 0.01] for y in (150, 180)] + [[x - d * 0.01, y, ut - 0.6] for y in (150, 180)])
    return c


def cam():
    """the cam on a clamping metal disc horn (radial clamp screw; no centre screw through the cam): 2 x M2.5 x 10 at r 7 in the lobes"""
    c = H.cam_solid() + cylY(0, H.U_C, *H.CAM["y"], 4.4, 48)                 # the layout's centre access hole filled: none needed
    for a in (45.0, 225.0):
        x, u = 7.0 * math.cos(math.radians(a)), H.U_C + 7.0 * math.sin(math.radians(a))
        c -= cylY(x, u, H.CAM["y"][0] - 1, H.CAM["y"][1] + 1, 1.35, 20)
    return c


def solid(path):
    import subprocess
    r = subprocess.run([sys.executable, os.path.join(H.ROOT, "nox", "solid_check.py"), path], capture_output=True, text=True)
    return r.stdout.strip().splitlines()[-1] if r.stdout else r.stderr[-300:]


def orient(t, force=None):
    """lay the part on its largest flat face (the face direction with the largest coplanar area at the extreme), or on `force` (ax, sign)"""
    best = None
    for ax in range(3):
        for sg in (-1, 1):
            if force and (ax, sg) != force: continue
            n = np.zeros(3); n[ax] = sg
            ext = (t.vertices @ n).max()
            fn = t.face_normals; ctr = t.triangles_center @ n
            sel = (fn @ n > 0.999) & (np.abs(ctr - ext) < 0.05)
            a = t.area_faces[sel].sum()
            if best is None or a > best[0]: best = (a, ax, sg)
    a, ax, sg = best
    v = np.zeros(3); v[ax] = sg
    R = trimesh.geometry.align_vectors(v, [0, 0, -1.0]); t2 = t.copy(); t2.apply_transform(R); t2.apply_translation(-t2.bounds[0])
    return t2, dict(bed_face=f"{'+' if sg > 0 else '-'}{'XYu'[ax]} down", bed_mm2=round(float(a), 1))


def main():
    head = sys.argv[1] if len(sys.argv) > 1 else "A"
    P = {"carrier_px": carrier(+1), "carrier_mx": carrier(-1), "cam": cam(), "rack_bar": H.rack_bar("MG995" if head == "B" else "DS3218")}
    if head == "B": P = {k: mirror_x(v) for k, v in P.items()}
    FORCE = {"rack_bar": (1, 1)}                 # the bar on its full-length Y 73.25 face: the tooth profile (X-u) in the layers, the tail step on top
    res = {}
    for nm, m in P.items():
        t = tm(m); name = f"box_head_v7_{head}_{nm}"
        p = os.path.join(OUT, name + ".stl"); t.export(p)
        tp, info = orient(t, FORCE.get(nm)); pp = os.path.join(OUT, "print", name + ".stl"); tp.export(pp)
        info.update(mass_g=round(t.volume * 1.27e-3, 2), bbox=[round(float(v), 1) for v in tp.extents], solid=solid(pp))
        res[name] = info
        print(name, info)
    json.dump(res, open(os.path.join(OUT, f"parts_{head}.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
