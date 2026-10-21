"""Print-orientation check for Nox parts (M numbers).
For each of the 6 axis-aligned 'down' directions: bed contact area, overhang area needing support
(faces more than 60 deg from vertical pointing down, i.e. steeper than the 30 deg support threshold),
and the worst free-standing island history: islands < 20 mm2 that persist > 10 mm from the bed.
Usage: python print_orient.py part.stl [--all]   (needs trimesh, shapely, scipy)
Pass: saved face has the largest bed contact (or the designer says why not); no first-layer island < 20 mm2 lasting > 10 mm; no free-standing post > 10 mm. Rule: nox/README.md Printing, Orientation and supports.
"""
import sys, numpy as np, trimesh

def rot_to_down(axis, sign):
    # rotation taking (sign*axis) to -z
    v = np.zeros(3); v[axis] = sign
    target = np.array([0, 0, -1.0])
    return trimesh.geometry.align_vectors(v, target)

def island_check(m, step=1.0):
    lo, hi = m.bounds[0][2], m.bounds[1][2]
    zs = np.arange(lo + 0.3, hi, step)
    first = None; worst = []
    for z in zs:
        s = m.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
        if s is None: continue
        p, _ = s.to_2D()
        ar = sorted(pp.area for pp in p.polygons_full)
        if first is None:
            first = (len(ar), [round(a, 1) for a in ar[:8]], round(sum(ar), 1))
        worst.append((z - lo, len(ar), ar[0] if ar else 0))
    # length of the run from the bed where any island < 20 mm2 exists
    run = 0
    for h, n, a in worst:
        if a < 20: run = h
        else: break
    return first, run

def analyse(m, label):
    n = m.face_normals; a = m.area_faces; zc = m.triangles_center[:, 2]
    zmin = m.bounds[0][2]
    bed = a[(n[:, 2] < -0.99) & (zc < zmin + 0.3)].sum()
    over = a[(n[:, 2] < -0.5) & ~((n[:, 2] < -0.99) & (zc < zmin + 0.3))].sum()
    first, run = island_check(m)
    print(f"{label:>8}: bed {bed:6.0f} mm2  overhang {over:6.0f} mm2  height {m.extents[2]:5.1f}  "
          f"first layer {first[0]} islands {first[1]} (total {first[2]})  thin-island run {run:.0f} mm")
    return bed, over, run

if __name__ == '__main__':
    m0 = trimesh.load(sys.argv[1])
    print(sys.argv[1], 'extents', np.round(m0.extents, 1), 'g PETG', round(m0.volume / 1000 * 1.27, 2), 'watertight', m0.is_watertight)
    analyse(m0, 'saved')
    if '--all' in sys.argv:
        for ax, name in [(0, 'x'), (1, 'y'), (2, 'z')]:
            for s in (1, -1):
                if ax == 2 and s == -1: continue
                m = m0.copy(); m.apply_transform(rot_to_down(ax, s))
                analyse(m, f"{'-' if s < 0 else '+'}{name} down")
