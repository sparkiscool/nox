"""Tet mesh of the glued collar for funnel_seat_fe.py: one-body CSG surface -> gmsh (classify, reparametrize, remesh)
-> tets. h: global size; h_fine within r_fine of each point in fine_pts (the seat pads)."""
import math, numpy as np, gmsh, trimesh

def tet_mesh(manifold, h=3.0, h_fine=1.5, fine_pts=(), r_fine=25.0, path="out/collar.stl"):
    me = manifold.to_mesh()
    trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=False).export(path)
    gmsh.initialize(); gmsh.option.setNumber("General.Terminal", 0)
    gmsh.merge(path)
    gmsh.model.mesh.classifySurfaces(30 * math.pi / 180, True, True, math.pi)
    gmsh.model.mesh.createGeometry()
    s = gmsh.model.getEntities(2)
    sl = gmsh.model.geo.addSurfaceLoop([e[1] for e in s]); gmsh.model.geo.addVolume([sl]); gmsh.model.geo.synchronize()
    if fine_pts:
        pts = [gmsh.model.geo.addPoint(*p) for p in fine_pts]; gmsh.model.geo.synchronize()
        f1 = gmsh.model.mesh.field.add("Distance"); gmsh.model.mesh.field.setNumbers(f1, "PointsList", pts)
        f2 = gmsh.model.mesh.field.add("Threshold"); gmsh.model.mesh.field.setNumber(f2, "InField", f1)
        for k, v in (("SizeMin", h_fine), ("SizeMax", h), ("DistMin", 0.6 * r_fine), ("DistMax", r_fine)):
            gmsh.model.mesh.field.setNumber(f2, k, v)
        gmsh.model.mesh.field.setAsBackgroundMesh(f2)
        gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 0); gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 0)
    gmsh.option.setNumber("Mesh.MeshSizeMax", h); gmsh.option.setNumber("Mesh.MeshSizeMin", 0.3)
    gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 0); gmsh.option.setNumber("Mesh.Algorithm", 6)
    gmsh.model.mesh.generate(3)
    gmsh.model.mesh.optimize("Netgen")
    nt, nc, _ = gmsh.model.mesh.getNodes()
    et, etags, en = gmsh.model.mesh.getElements(3)
    gmsh.finalize()
    P = nc.reshape(-1, 3); order = np.argsort(nt); lookup = np.searchsorted(nt[order], en[0].astype(np.int64))
    T = order[lookup].reshape(-1, 4)
    used = np.unique(T); remap = -np.ones(len(P), np.int64); remap[used] = np.arange(len(used))
    P, T = P[used], remap[T]
    a, b, c, d = (P[T[:, i]] for i in range(4))
    vol = np.einsum("ij,ij->i", np.cross(b - a, c - a), d - a) / 6
    T[vol < 0] = T[vol < 0][:, [0, 2, 1, 3]]
    return P, T, float(np.abs(vol).sum())
