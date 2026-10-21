import sys, json, trimesh, solid_check as SC, node_topo2 as N
import trimesh.transformations as T
name = sys.argv[1]
f = 'socket_v2_coupon.stl' if name == 'coupon' else f'v12/{name}.stl'
m = trimesh.load(f, force='mesh')
drawn = []
if name != 'coupon':
    nd = N.NODES[name]; g = N.geometry(nd); drawn = [N.tm(g['drawn'] - g['bores'])]
ok, info = SC.solid_check(m, drawn=drawn)
if not ok and name == 'hub':
    mm = m.copy(); mm.apply_transform(T.rotation_matrix(0.0021, [0, 0, 1])); d2 = [x.copy() for x in drawn]
    for x in d2: x.apply_transform(T.rotation_matrix(0.0021, [0, 0, 1]))
    ok2, info2 = SC.solid_check(mm, drawn=d2); print(name, 'turned 0.12 deg:', 'PASS' if ok2 else 'FAIL', {k: info2[k] for k in ('bodies_plain', 'bodies_eroded', 'necks', 'drawn_not_in_main', 'thin_walls')}, flush=True)
print(name, 'PASS' if ok else 'FAIL', {k: info[k] for k in ('bodies_plain', 'bodies_eroded', 'necks', 'drawn_not_in_main', 'thin_walls')}, flush=True)
