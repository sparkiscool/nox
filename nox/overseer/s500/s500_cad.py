"""Holybro S500 V2 frame CAD (S500-C1_ASM.step, docs.holybro.com, exported 2025-01-15):
the arm-to-plate interface, plate gap, motor centre and arm sections for the printed arms.

python3 nox/overseer/s500/s500_cad.py   (needs OCP + trimesh + shapely; ~1 min)

CAD frame: Y up, bottom plate top face at Y 0; arm 002 lies along +X +Z.
Printed in the arm frame: u along the arm from the frame centre, v across, y up.
"""
import math, os, sys, tempfile
import numpy as np, trimesh
from shapely.geometry import Polygon
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_LabelSequence, TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.TopoDS import TopoDS
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer

STEP = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'S500-C1_ASM.step')
S2 = 1 / math.sqrt(2)


def load(path):
    doc = TDocStd_Document(TCollection_ExtendedString('doc'))
    r = STEPCAFControl_Reader(); r.SetNameMode(True)
    assert r.ReadFile(path) == 1
    r.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    def name(l):
        a = TDataStd_Name()
        return a.Get().ToExtString() if l.FindAttribute(TDataStd_Name.GetID_s(), a) else '?'

    parts = {}

    def walk(label, loc, path):
        if st.IsReference_s(label):
            ref = TDF_Label(); st.GetReferredShape_s(label, ref)
            walk(ref, loc.Multiplied(st.GetLocation_s(label)), path + [name(label)])
        elif st.IsAssembly_s(label):
            kids = TDF_LabelSequence(); st.GetComponents_s(label, kids)
            for i in range(1, kids.Length() + 1):
                walk(kids.Value(i), loc, path + [name(label)])
        else:
            parts['/'.join(path + [name(label)])] = st.GetShape_s(label).Moved(loc)

    top = TDF_LabelSequence(); st.GetFreeShapes(top)
    for i in range(1, top.Length() + 1):
        walk(top.Value(i), TopLoc_Location(), [])
    return parts


def bbox(shp):
    bb = Bnd_Box(); BRepBndLib.Add_s(shp, bb)
    return bb.Get()


def vertical_holes(shp):
    out = set()
    ex = TopExp_Explorer(shp, TopAbs_FACE)
    while ex.More():
        s = BRepAdaptor_Surface(TopoDS.Face_s(ex.Current()))
        if s.GetType() == GeomAbs_Cylinder:
            c = s.Cylinder(); p = c.Axis().Location(); d = c.Axis().Direction()
            if abs(abs(d.Y()) - 1) < 1e-3 and c.Radius() < 4:
                out.add((round(2 * c.Radius(), 2), round(p.X(), 2), round(p.Z(), 2), round(p.Y(), 2)))
        ex.Next()
    return sorted(out)


def find(parts, key):
    return next(v for k, v in parts.items() if k.endswith(key))


if __name__ == '__main__':
    parts = load(STEP)
    top, bot, arm = find(parts, 'TOP-PLATE-S500'), find(parts, 'BOTTOM-PLATE-S500'), find(parts, 'SOLID002/SOLID')
    tb, bb, ab = bbox(top), bbox(bot), bbox(arm)
    print(f'top plate    X {tb[0]:.2f}..{tb[3]:.2f}  Z {tb[2]:.2f}..{tb[5]:.2f}  Y {tb[1]:.2f}..{tb[4]:.2f}')
    print(f'bottom plate X {bb[0]:.2f}..{bb[3]:.2f}  Z {bb[2]:.2f}..{bb[5]:.2f}  Y {bb[1]:.2f}..{bb[4]:.2f}')
    print(f'plate gap (bottom top face to top underside) {tb[1] - bb[4]:.2f}')

    print('\narm 002 holes (u along the arm, v across):')
    for d, x, z, y in vertical_holes(arm):
        print(f'  Ø{d:4.2f}  u {(x + z) * S2:7.2f}  v {(x - z) * S2:6.2f}  face y {y:5.2f}')
    print('plate holes on the arm 002 diagonal (u 25..80, |v| < 25):')
    for nm, shp in (('top', top), ('bottom', bot)):
        for d, x, z, y in vertical_holes(shp):
            u, v = (x + z) * S2, (x - z) * S2
            if 25 < u < 80 and abs(v) < 25:
                print(f'  {nm:6s} Ø{d:4.2f}  u {u:6.2f}  v {v:6.2f}   (X {x:6.2f}, Z {z:6.2f})')

    with tempfile.TemporaryDirectory() as tmp:
        BRepMesh_IncrementalMesh(arm, 0.05, False, 0.2, True)
        StlAPI_Writer().Write(arm, os.path.join(tmp, 'arm.stl'))
        m = trimesh.load(os.path.join(tmp, 'arm.stl'))
    R = np.array([[S2, 0, S2], [0, 1, 0], [S2, 0, -S2]])
    m.vertices = m.vertices @ R.T  # columns u, y, v
    print(f'\narm: u {m.bounds[0][0]:.1f}..{m.bounds[1][0]:.1f}, width {m.bounds[1][2] - m.bounds[0][2]:.1f}, '
          f'y {m.bounds[0][1]:.2f}..{m.bounds[1][1]:.2f}, volume {abs(m.volume):.0f} mm3')
    s = m.section(plane_origin=[0, 28, 0], plane_normal=[0, 1, 0])
    for d in s.discrete:
        q = d[:, [0, 2]]; c = q.mean(0); r = np.linalg.norm(q - c, axis=1)
        if r.std() < 0.05 and 5 < 2 * r.mean() < 12:
            print(f'motor centre bore Ø{2 * r.mean():.1f} at u {c[0]:.2f} (wheelbase {2 * c[0]:.1f})')
    print('sections (u: y range, v range, area, I about the horizontal centroid axis):')
    for u in (40, 50, 60, 70, 80, 100, 120, 150, 180, 200):
        sec = m.section(plane_origin=[u, 0, 0], plane_normal=[1, 0, 0])
        pts = np.vstack(sec.discrete)
        polys = [Polygon(d[:, [2, 1]]) for d in sec.discrete]
        A = Sy = Iy = 0.0
        for i, p in enumerate(polys):
            sign = -1 if sum(q.contains(p) for j, q in enumerate(polys) if j != i) % 2 else 1
            c = np.array(p.exterior.coords); x, y = c[:, 0], c[:, 1]
            cr = x[:-1] * y[1:] - x[1:] * y[:-1]
            A += sign * abs(cr.sum() / 2)
            Sy += sign * abs(((y[:-1] + y[1:]) * cr).sum() / 6)
            Iy += sign * abs(((y[:-1] ** 2 + y[:-1] * y[1:] + y[1:] ** 2) * cr).sum() / 12)
        print(f'  u {u:3d}: y {pts[:, 1].min():5.2f}..{pts[:, 1].max():5.2f}  v ±{pts[:, 2].max():5.2f}  '
              f'A {A:5.0f}  I {Iy - A * (Sy / A) ** 2:6.0f}')
