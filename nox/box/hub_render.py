"""Hub renders: ray-traced (embree) orthographic views of the parts and the assembly, cutaway on X > 0 for the
assembly. v5 (17 Oct): the detent tower is drawn into the hub; the ring, roller arm, roller and springs are shown.
python3 hub_render.py -> out/hub_v5_parts.png, out/hub_v5_assembly.png (v4 when arm_layout.HUB_DETENT is False)"""
import os, math, json, numpy as np, trimesh, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import ndimage as ndi
import arm_layout as L, arm_cad as A
from arm_cad import tm, cylY, rsquare

BG = np.array([0.985, 0.98, 0.968])

def view_dir(az, el):
    a, e = math.radians(az), math.radians(el)
    return -np.array([math.cos(e) * math.sin(a), math.sin(e), math.cos(e) * math.cos(a)])   # Y up in the render

def render(parts, az=35, el=25, W=1100, H=900, pad=1.08):
    """parts: [(trimesh, rgb)]. Mesh coords are W (Y = axle); the render puts the axle vertical."""
    meshes, cols = [], []
    for m, c in parts:
        t = m.copy(); t.vertices = t.vertices[:, [0, 1, 2]]
        meshes.append(t); cols.append(np.array(c))
    allv = np.vstack([m.vertices for m in meshes])
    d = view_dir(az, el); up0 = np.array([0, 1.0, 0])
    r = np.cross(d, up0); r /= np.linalg.norm(r); u = np.cross(r, d)
    c = (allv.max(0) + allv.min(0)) / 2
    pr, pu = (allv - c) @ r, (allv - c) @ u
    s = max(np.ptp(pr) / W, np.ptp(pu) / H) * pad
    xs = (np.arange(W) - W / 2) * s; ys = (H / 2 - np.arange(H)) * s
    X, Y = np.meshgrid(xs, ys)
    org = c + X.ravel()[:, None] * r + Y.ravel()[:, None] * u - d * 2000
    dirs = np.tile(d, (len(org), 1))
    big = trimesh.util.concatenate(meshes); pid = np.concatenate([np.full(len(m.faces), k) for k, m in enumerate(meshes)])
    ix, ir, loc = big.ray.intersects_id(org, dirs, multiple_hits=False, return_locations=True)
    img = np.tile(BG, (H * W, 1)); depth = np.full(H * W, np.inf); part = np.full(H * W, -1)
    n = big.face_normals[ix]; n = np.where((n @ d)[:, None] > 0, -n, n)
    key = d * -0.55 + u * 0.6 - r * 0.35; key /= np.linalg.norm(key)
    fill = -d
    lam = 0.28 + 0.55 * np.clip(n @ key, 0, 1) + 0.22 * np.clip(n @ fill, 0, 1)
    h = key - d; h /= np.linalg.norm(h); spec = 0.18 * np.clip(n @ h, 0, 1) ** 24
    col = np.array(cols)[pid[ix]]
    img[ir] = np.clip(col * lam[:, None] + spec[:, None], 0, 1)
    depth[ir] = ((loc - org[ir]) @ d); part[ir] = pid[ix]
    img = img.reshape(H, W, 3); depth = depth.reshape(H, W); part = part.reshape(H, W)
    # outlines: silhouettes, part boundaries and depth creases
    dz = np.where(np.isfinite(depth), depth, 1e4)
    edge = (ndi.generic_gradient_magnitude(dz, ndi.sobel) > 6 * s * 8) | (ndi.maximum_filter(part, 3) != ndi.minimum_filter(part, 3))
    img[edge] = img[edge] * 0.45
    return img

def colr(hexs): return np.array([int(hexs[i:i + 2], 16) / 255 for i in (1, 3, 5)])
C = dict(hub=colr("#e8914f"), post=colr("#3d5a80"), stub=colr("#aeb6c2"), brg=colr("#d9d2c3"), p80=colr("#d8b84a"),
         loop=colr("#2b2b2b"), motor=colr("#2f3640"), bar=colr("#c2c8cf"), p20=colr("#c99a12"),
         ring=colr("#5b7fa6"), arm=colr("#2a9d8f"), roller=colr("#e9e4d8"), spring=colr("#c0392b"))

if __name__ == "__main__":
    v5 = getattr(L, "HUB_DETENT", False); tag = "v5" if v5 else "v4"
    hub = trimesh.load(f"out/box_hub_{tag}.stl"); post = tm(A.sun_post()); stub = tm(A.stub())
    res = json.load(open(f"box_hub_{tag}_result.json")) if os.path.exists(f"box_hub_{tag}_result.json") else dict(mass_g=hub.volume / 1000 * 1.27, worst=float("nan"))
    S = A.axle_sun_bodies()
    brgs = trimesh.util.concatenate([tm(cylY(0, 0, y0, y0 + 16, 40) - cylY(0, 0, y0 - 1, y0 + 17, 25)) for y0 in L.Y_BRG])
    motors = trimesh.util.concatenate([tm(rsquare(M, L.NEMA / 2, 2.0, L.Y_PLATE0)) for M in L.MOTORS.values()])
    bars = trimesh.util.concatenate([tm(A.bar_2040(*L.BAR_A)), tm(A.bar_2040(*L.BAR_B).rotate([0, -90, 0]))])
    p20 = trimesh.util.concatenate([tm(cylY(M["xc"], M["zc"], 54, 64, 8)) for M in L.MOTORS.values()])
    det = []
    if v5:
        import detent as DT
        c = DT.CHOSEN; Dc = DT.design(c["th"], c["s"], c["a"], c["b"], c["phi_d"], c["psi"]); Sd = DT.arm_solids(Dc, np.array(c["Q"]))
        ring = tm(DT.ring(c["th"])); arm = tm(Sd["arm"] + Sd["spacer"] + Sd["hook"]); roller = tm(Sd["roller"] + Sd["nut"])
        springs = tm(Sd["springs"][0] + Sd["springs"][1]); anchor = tm(Sd["anchor"])
        det = [(ring, C["ring"]), (arm, C["arm"]), (roller, C["roller"]), (springs, C["spring"]), (anchor, C["roller"])]

    # ---- parts sheet
    sheet = [(hub, C["hub"], f"Hub {tag}  {res['mass_g']:.1f} g (M), worst {res['worst']:.2f}", 30, 38)]
    if v5:
        sheet += [(ring, C["ring"], f"Detent ring v1  {DT.ring(c['th']).volume() / 1000 * 1.27:.1f} g (C), 12 notches", 30, 55),
                  (arm, C["arm"], f"Roller arm v1  {(Sd['arm'] + Sd['spacer']).volume() / 1000 * 1.27:.1f} g (C) + 683ZZ", 30, 55)]
    else:
        sheet += [(post, C["post"], "Sun post v1  30.4 g (C)", 30, 32), (stub, C["stub"], "Stub axle v2  109.7 g (C)", 30, 32)]
    fig, ax = plt.subplots(1, 3, figsize=(16, 6.4), dpi=150); fig.patch.set_facecolor(BG)
    for a, (m, cc, t, az, el) in zip(ax, sheet):
        a.imshow(render([(m, cc)], az, el, 900, 820)); a.set_title(t, fontsize=12, color="#222", loc="left"); a.axis("off")
    fig.suptitle(f"Nox box arm: hub {tag} parts (axle vertical, + away from the stub)", x=0.02, ha="left", fontsize=14, color="#222")
    plt.tight_layout(); plt.savefig(f"out/hub_{tag}_parts.png", facecolor=BG); plt.close()

    # ---- assembly: full + cutaway (X > 0 removed) along the axle + the detent from above
    def cut(m):
        try: return m.slice_plane([0, 0, 0], [-1, 0, 0], cap=True)
        except Exception: return m.slice_plane([0, 0, 0], [-1, 0, 0])
    full = [(stub, C["stub"]), (brgs, C["brg"]), (post, C["post"]), (hub, C["hub"]), (tm(S["p80"]), C["p80"]), (tm(S["loop"]), C["loop"]),
            (motors, C["motor"]), (p20, C["p20"]), (bars, C["bar"])] + det
    n = 3 if v5 else 2
    fig, ax = plt.subplots(1, n, figsize=(8 * n, 7.6), dpi=150); fig.patch.set_facecolor(BG)
    ax[0].imshow(render(full, 215, 30, 1000, 900)); ax[0].set_title("Assembly from the bar side", fontsize=12, loc="left"); ax[0].axis("off")
    cutparts = [(cut(m), cc) for m, cc in full if m is not bars and m is not motors] + [(cut(motors), C["motor"])]
    ax[1].imshow(render(cutparts, 60, 18, 1000, 900)); ax[1].set_title("Cutaway X > 0: post through the hub, 80T on the post", fontsize=12, loc="left"); ax[1].axis("off")
    if v5:
        box_ = trimesh.creation.box(extents=[90, 60, 90]); box_.apply_translation([5, 75, -15])
        close = [(hub.slice_plane([0, 62, 0], [0, 1, 0]), C["hub"]), (tm(S["p80"]), C["p80"])] + det
        close = [(m.slice_plane([0, 54, 0], [0, 1, 0]), cc) for m, cc in close]
        ax[2].imshow(render(close, 20, 70, 1000, 900, pad=1.04)); ax[2].set_title("Detent: tower on the hub, roller in a notch", fontsize=12, loc="left"); ax[2].axis("off")
    from matplotlib.patches import Patch
    keys = ["stub", "brg", "post", "hub", "p80", "loop", "motor", "bar"] + (["ring", "arm", "spring"] if v5 else [])
    labels = ["stub (fixed)", "6010 x2", "sun post (fixed)", f"hub {tag} (turns)", "80T sun (fixed)", "122T loop", "NEMA17 x3", "2040 bars"] + \
        (["detent ring (fixed)", "roller arm", "springs"] if v5 else [])
    fig.legend([Patch(fc=C[k]) for k in keys], labels, loc="lower center", ncol=len(keys), frameon=False, fontsize=9)
    fig.suptitle(f"Nox box arm: hub {tag} on the fixed sun post" + (", auto-centering detent" if v5 else ""), x=0.02, ha="left", fontsize=14, color="#222")
    plt.tight_layout(rect=(0, 0.05, 1, 0.93)); plt.savefig(f"out/hub_{tag}_assembly.png", facecolor=BG); plt.close()
    print(f"out/hub_{tag}_parts.png out/hub_{tag}_assembly.png")
