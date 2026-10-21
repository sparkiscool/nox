"""usage: python trim_keep.py <tag>   exact cut of the prop keep-out on the built part (the 1 mm slice keep-out in
geom_top under-reaches by up to ~0.2 between slices): 0.25 mm slices at each slice's largest radius, subtracted from
runs/<tag>/s2_top_case.stl; then function / thin / bodies / keep-out clearance; writes s2_top_case_trim.stl."""
import sys, numpy as np, trimesh, manifold3d as mf
import geom_top as G, noxopt as NO
tag = sys.argv[1]; d = f"runs/{tag}"
t = trimesh.load(f"{d}/s2_top_case.stl")
P = mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))
z0, z1, K, dz = G.PROP_Z - G.SWEEP, G.PROP_Z + G.SWEEP, G.PROP_KEEP, 0.25
sl = []
for mx, my in G.MOTORS:
    sl.append(G.zcyl(mx, my, z0, z1, G.PROP_R + K + 0.02, 192))
    for k in range(int(K / dz)):
        r = G.PROP_R + np.sqrt(max(K ** 2 - (k * dz) ** 2, 0.0)) + 0.02
        sl.append(G.zcyl(mx, my, z1 + k * dz, z1 + (k + 1) * dz, r, 192))
        sl.append(G.zcyl(mx, my, z0 - (k + 1) * dz, z0 - k * dz, r, 192))
KO = G.U(sl)
geo = G.parts(); drawn = G.U(geo["drawn"]); cuts = G.U(geo["cuts"])
side_stack = G.U([G.zcyl(G.BOLT_SIDE[0], s * G.BOLT_SIDE[1], G.PLATE_TOP, G.pad_z0(s) + G.PAD_T + 0.01, G.PAD_R + 0.01) for s in (1, -1)])
Q = (P - (KO - side_stack))            # the side pads are the accepted bolt-stack exception (pad on the cage hub)
cut = (P.volume() - Q.volume())
tq = G.to_trimesh(Q); tq.export(f"{d}/s2_top_case_trim.stl")
print(f"cut {cut:.2f} mm3; mass {Q.volume()/1000*1.27:.2f} g; bodies {len(Q.decompose())}; watertight {tq.is_watertight}")
fc = NO.function_check(tq, drawn - cuts, cuts, geo["env"] - G.U(geo["keep"]) - cuts)
th = NO.thin_check(tq, G.to_trimesh(drawn - cuts), 2.0)
S, _ = trimesh.sample.sample_surface_even(tq, 120000); S = np.vstack([S, tq.vertices]); w = 1e9
far = (np.hypot(S[:, 0] - 3.9, np.abs(S[:, 1]) - 31) > 5.6) | (S[:, 2] > 30)
for hx, hy in G.MOTORS:
    dr = np.maximum(0, np.hypot(S[:, 0] - hx, S[:, 1] - hy) - G.PROP_R); dzz = np.maximum(0, np.maximum(z0 - S[:, 2], S[:, 2] - z1))
    w = min(w, np.hypot(dr, dzz)[far].min())
print(f"keep-out clearance outside the side pad stacks: {w:.2f}")
