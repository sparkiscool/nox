import json, base64, math, numpy as np, trimesh, fast_simplification as fs
import cage_cad as cc
OUT = cc.OUT
def dec(n, tgt):
    m = trimesh.load(f"{OUT}/s2_cage_v2_{n}.stl"); r = max(0.0, 1 - tgt / len(m.faces))
    if r > 0:
        v, f = fs.simplify(m.vertices.astype(np.float32), m.faces.astype(np.int32), target_reduction=r)
    else:
        v, f = m.vertices, m.faces
    return dict(p=base64.b64encode(np.asarray(v, np.float32).tobytes()).decode(), i=base64.b64encode(np.asarray(f, np.uint32).tobytes()).decode())
meshes = {n: dec(n, t) for n, t in (("corner_node", 24000), ("corner_wheel_A", 400), ("corner_wheel_B", 400), ("tee_node", 8000), ("tee_node_tail", 8000), ("side_hub", 9000), ("tail_hub", 3000), ("foot_node", 4000))}
lay = json.load(open(f"{OUT}/cage_layout.json")); chk = json.load(open(f"{OUT}/cage_check.json"))
# collar faces in the drone frame (seated)
n0, d0 = cc.face_plane(0); s45, c45 = math.sin(cc.SLOPE), math.cos(cc.SLOPE)
z_of = lambda h: (h * s45 - d0) / c45
collar = dict(h_lo=84.0, h_hi=159.9, z_lo=round(z_of(84.0), 2), z_hi=round(z_of(159.9), 2))
foot = json.load(open('/home/claude/cage/foot_opt.json'))
data = dict(meshes=meshes, rods=lay["rods"], collar=collar, check=chk, foot=foot)
t = open("/home/claude/cage/cage_viewer_template.html").read()
open("/home/claude/cage/scout_cage_v2.html", "w").write(t.replace("__DATA__", json.dumps(data, separators=(",", ":")).replace("</", "<\\/")))
print("collar", collar, "KB", round(len(json.dumps(data)) / 1024))
