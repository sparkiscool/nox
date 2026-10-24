import sys, os, numpy as np, trimesh, subprocess
sys.path.insert(0, '/home/user/nox/nox/box'); sys.path.insert(0, '/home/user/nox/nox')
import head_v7_layout as H
from arm_cad import tm
from assembly import load
head = H.design_space() 
for nm, m in H.parts().items():
    if nm != "body (drawn interfaces)": head = head + m
t = tm(head); t.apply_translation([0, 0, 111.3])        # head A at the commanded drone insert (phi 90): W Z = z0 + u
print("head at drone: Z", t.bounds[:, 2].round(1))
res = []
for f in ("nox/box/box_catch.assembly.json",):
    a = load("/home/user/nox/" + f)
    for p, m in zip(a.parts, a.meshes()):
        if m is None or len(m.faces) == 0: continue
        d = trimesh.proximity.signed_distance(m, t.vertices[::3]) if False else None
        lo = np.maximum(m.bounds[0], t.bounds[0]); hi = np.minimum(m.bounds[1], t.bounds[1])
        if (lo < hi).all():
            from arm_cad import man
            try: v = (man(m) ^ man(t)).volume()
            except Exception as e: v = -1
            res.append((p.get("name"), round(v, 1), m.bounds.round(0).tolist()))
        else:
            res.append((p.get("name"), 0.0, "bbox apart"))
for r in res: print(r)
print("--- structure v2 at every bay (head A at bay_cmd 101.8) and the drone")
import arm_kinematics as K
from arm_cad import man
a = load("/home/user/nox/nox/box/box_structure_v2.assembly.json")
base = tm(head)
poses = [(90.0, 111.3)] + [(b, 101.8) for b in (30, 0, -30, -60, -90, -120, -150, -180, -210)]
worst = {}
for p, m in zip(a.parts, a.meshes()):
    nm = p.get("name")
    if m is None or len(m.faces) == 0 or nm.startswith("arm:") or "receiver" in nm or "cartridge" in nm: continue
    for phi, z0 in poses:
        T = K.rotY(90.0 - phi) @ K.trans(0, 0, z0)
        h = base.copy(); h.apply_transform(T)
        lo = np.maximum(m.bounds[0], h.bounds[0]); hi = np.minimum(m.bounds[1], h.bounds[1])
        if not (lo < hi).all(): continue
        try: v = (man(m) ^ man(h)).volume()
        except Exception: v = -1
        if v != 0: worst[(nm, phi)] = round(v, 1)
print(worst if worst else "structure v2: 0 mm3 at every bay and the drone")
