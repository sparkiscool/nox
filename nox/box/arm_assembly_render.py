"""Assembly picture of the swap arm v1 (W coords): hub (generated if out/box_hub_v1.stl exists, else its envelope),
stub, bar, motors, pulley, heads, fingers, carried cartridge, one bay + its cartridge, casing outline.
python arm_assembly_render.py [state] -> out/arm_assembly_<state>.png"""
import sys, os, math
import numpy as np, trimesh
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import arm_layout as L, arm_cad as A

state = sys.argv[1] if len(sys.argv) > 1 else "park"
S2 = math.sqrt(2)

def load_or(path, fallback):
    if os.path.exists(path):
        t = trimesh.load(path, force="mesh"); return t
    return A.tm(fallback)

parts = []   # (trimesh, color, alpha)
H = A.hub_parts(); C = A.head_parts(); F = A.fingers_and_racks(); B = A.bay_parts()
hub_t = load_or("out/box_hub_v1_W.stl", H["drawn"]); parts.append((hub_t, "#8899aa", 1.0))
parts.append((A.tm(A.stub()), "#556677", 1.0))
bar = A.box(-20, 20, L.Y_BAR[0], L.Y_BAR[1], -L.BAR_HALF, L.BAR_HALF); parts.append((A.tm(bar), "#777777", 1.0))
for nm, M in L.MOTORS.items():
    parts.append((A.tm(A.rsquare(M, L.NEMA / 2, 2.0, L.Y_PLATE0)), "#333333", 1.0))
    parts.append((A.tm(A.cylY(M["xc"], M["zc"], 62 if nm != "axle" else 54, 76 if nm != "axle" else 64, L.P20_FL / 2)), "#aa8833", 1.0))
parts.append((A.tm(A.cylY(0, 0, L.Y_P80[0], L.Y_P80[1], L.P80_FL / 2) + A.cylY(0, 0, L.Y_P80[1], 72, L.P80_BOSS / 2)), "#aa8833", 1.0))
head_t = load_or("out/box_head_v1_W.stl", C["drawn"])
z0A = {"park": L.Z0_A_HOME, "bay": L.Z0_BAY, "drone": L.Z0_DRONE}[state]
def place(t, z0, rot=0.0):
    u = t.copy(); u.apply_translation([0, 0, z0])
    if rot: u.apply_transform(trimesh.transformations.rotation_matrix(math.radians(rot), [0, 1, 0]))
    return u
parts.append((place(head_t, z0A), "#cc6644", 1.0)); parts.append((place(head_t, -L.Z0_B_HOME, 180), "#cc6644", 1.0))
fing = A.tm(F["A"] + F["B"])
parts.append((place(fing, z0A), "#dd9977", 1.0)); parts.append((place(fing, -L.Z0_B_HOME, 180), "#dd9977", 1.0))
yc = L.Y_C
cart = A.box(-L.Z_TABC, L.CART_L - L.Z_TABC, yc - L.CART_W / 2, yc + L.CART_W / 2, z0A + L.CART_BOT_DZ, z0A + L.CART_TOP_DZ)
parts.append((A.tm(cart), "#44aa66", 0.9))
bay_t = load_or("out/box_bay_v1_W.stl", B["drawn"])
for a in L.BAY_ANGLES:
    parts.append((place(bay_t, 0, 90 - a), "#6688cc", 1.0))
    bc = A.box(-L.Z_TABC, L.CART_L - L.Z_TABC, yc - L.CART_W / 2, yc + L.CART_W / 2, L.R, L.R_BAY_TOP)
    parts.append((place(A.tm(bc), 0, 90 - a), "#44aa66", 0.5))
# casing outline (wire)
case = A.box(-L.CASE_IN, L.CASE_IN, L.Y_BACK_IN, L.Y_FAR_WALL, -L.CASE_IN, L.CASE_IN)

def draw(ax, elev, azim, title):
    az, el = np.radians(azim), np.radians(elev)
    cam = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    for t, col, al in parts:
        n = t.face_normals; shade = 0.4 + 0.6 * np.clip(n @ cam, 0, 1)
        rgb = np.array(matplotlib.colors.to_rgb(col))
        cols = np.clip(rgb[None, :] * shade[:, None], 0, 1)
        ax.add_collection3d(Poly3DCollection(t.triangles, facecolors=cols, edgecolors="none", alpha=al))
    bb = np.array(case.bounding_box()); lo, hi = bb[:3], bb[3:]
    for s, e in [((lo[0], lo[1], lo[2]), (hi[0], lo[1], lo[2])), ((lo[0], lo[1], lo[2]), (lo[0], hi[1], lo[2])), ((lo[0], lo[1], lo[2]), (lo[0], lo[1], hi[2])),
                 ((hi[0], hi[1], hi[2]), (lo[0], hi[1], hi[2])), ((hi[0], hi[1], hi[2]), (hi[0], lo[1], hi[2])), ((hi[0], hi[1], hi[2]), (hi[0], hi[1], lo[2])),
                 ((hi[0], lo[1], lo[2]), (hi[0], hi[1], lo[2])), ((hi[0], lo[1], lo[2]), (hi[0], lo[1], hi[2])), ((lo[0], hi[1], lo[2]), (hi[0], hi[1], lo[2])),
                 ((lo[0], hi[1], lo[2]), (lo[0], hi[1], hi[2])), ((lo[0], lo[1], hi[2]), (hi[0], lo[1], hi[2])), ((lo[0], lo[1], hi[2]), (lo[0], hi[1], hi[2]))]:
        ax.plot([s[0], e[0]], [s[1], e[1]], [s[2], e[2]], color="#999999", lw=0.6)
    r = 190; ax.set_xlim(-r, r); ax.set_ylim(-60, 320); ax.set_zlim(-r, r)
    ax.set_box_aspect((1, 1, 1), zoom=1.5); ax.view_init(elev=elev, azim=azim); ax.set_title(title, fontsize=10); ax.set_axis_off()

fig = plt.figure(figsize=(14, 9), dpi=110)
draw(fig.add_subplot(1, 2, 1, projection="3d"), 20, -120, f"swap arm v1, state {state}: from the far-wall side")
draw(fig.add_subplot(1, 2, 2, projection="3d"), 0, -90, "along the axle (X right, Z up): bays at 45 deg, drone at the top")
fig.suptitle(f"Box swap arm v1 (W coords). R {L.R} ring (cartridge bottom face), pins r {L.R_PIN}, casing inner {2 * L.CASE_IN} x {2 * L.CASE_IN} x {L.Y_FAR_WALL - L.Y_BACK_IN}", fontsize=11)
fig.subplots_adjust(left=0, right=1, bottom=0, top=0.92, wspace=0)
out = f"out/arm_assembly_{state}.png"; fig.savefig(out); print(out)
