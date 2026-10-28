"""Hub deck setup picture: drawn bodies (orange) vs the design room (blue) at the front hub, from core_topo_v3.py's own setup
(python3 nox/overseer/hub_deck_setup_view.py <out.png>). Reports the drawn and room volumes ahead of x 55 (C)."""
import sys, numpy as np, trimesh, matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
OUTP = sys.argv[1]
sys.argv = ["x", "hub_deck", "1.25", "18", "--no-git"]
g = {"__file__": __file__.replace("hub_deck_setup_view.py", "core_topo_v3.py"), "__name__": "x"}
exec(compile(open(g["__file__"]).read(), "core_topo_v3", "exec"), g)
drawn, cuts, env, feet, ph = g["hub_deck_bodies"]()
front = g["box"](55, 130, -60, 60, -5, 60)
tm = lambda m: trimesh.Trimesh(np.asarray(m.to_mesh().vert_properties)[:, :3], np.asarray(m.to_mesh().tri_verts))
D = (drawn - cuts) ^ front; Rm = (env - cuts - drawn) ^ front
dv, rv = D.volume(), Rm.volume()
print(f"ahead of x 55: drawn {dv / 1000:.1f} cm3, design room {rv / 1000:.1f} cm3 (C)")
lt = np.array([0.3, -0.45, 0.85]); lt /= np.linalg.norm(lt)
def draw(ax, m, col, a):
    sh = 0.42 + 0.58 * np.clip(np.abs(m.face_normals @ lt), 0, 1); c = np.array(matplotlib.colors.to_rgb(col))[None] * sh[:, None]
    ax.add_collection3d(Poly3DCollection(m.vertices[m.faces], facecolors=np.c_[c, np.full(len(c), a)], linewidths=0))
Dm, Rmm = tm(D), tm(Rm)
fig = plt.figure(figsize=(16, 6), facecolor="white")
for i, (el, az, t) in enumerate([(15, -70, "from the front-left"), (0, -90, "side (nose right)"), (90, -90, "top"), (15, -130, "from behind")]):
    ax = fig.add_subplot(1, 4, i + 1, projection="3d"); draw(ax, Rmm, "#4f8fd8", 0.12); draw(ax, Dm, "#e8833a", 0.95)
    ax.set_xlim(55, 120); ax.set_ylim(-40, 40); ax.set_zlim(-2, 35); ax.set_box_aspect((65, 80, 37)); ax.view_init(el, az); ax.set_axis_off(); ax.set_title(t)
fig.suptitle(f"Hub deck setup, front hub: orange = drawn (sockets, feet, posts only) {dv / 1000:.1f} cm3; blue = room the topology may fill {rv / 1000:.1f} cm3 (C)", fontsize=12)
plt.tight_layout(); plt.savefig(OUTP, dpi=95)
