"""Battery latch figures for the layout artifact, all from the script's geometry: tab-line sections (latched / gripped /
free, both packs), the assembled sets (latch_assembly, Scout end skids), receiver v1 vs v2 (compare) and their |u| under
the swap cases (fe_sections, from battery_latch_topo --swap-v1 / --swap-v2 fields), and the Scout set in the collar planes.
python3 nox/box/battery_latch_figure.py -> runs/battery_latch/v2/figs/*.png"""
import os, sys, math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.join(os.path.dirname(HERE))]
import battery_latch as BL

OUT = os.path.join(BL.OUT, "figs")
COL = dict(receiver="#5b6b7a", cartridge="#c9a227", slider="#c0392b", pack="#9aa5ad", finger="#2a7f62", lid="#7d8b98",
           strips="#3b3b3b", xt60="#e0a020", skid="#1f6fb2")
KEYCOL = dict(cartridge="cartridge", slider_px="slider", slider_nx="slider", pack="pack", cartridge_skid_closed="skid",
              cartridge_skid_lead="skid", xt60_plug="xt60", receiver="receiver", receiver_lid="lid", receiver_strips="strips",
              xt60e1m="xt60", xt60_carrier="xt60")
T_ISO = np.array([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1.0]])    # plot (z, x, y): pack along, across, up


def polys(m, z):
    cs = m.slice(z)
    return [np.asarray(p) for p in cs.to_polygons()]


def draw(ax, m, z, c, a=1.0):
    for p in polys(m, z):
        if len(p) > 2:
            ax.add_patch(Polygon(p, closed=True, fc=c, ec="none", alpha=a))


def section(pack, ax, state):
    B = BL.build(pack); g = B["g"]
    rt = BL.TRAVEL if state == "gripped" else 0.0
    zt = g["Z_T"]
    draw(ax, BL.pack_body(g), zt, COL["pack"])
    draw(ax, B["cart"], zt, COL["cartridge"])
    if state != "free":
        draw(ax, B["recv"], zt, COL["receiver"])
        draw(ax, B["strips"], zt, COL["strips"])
    for s in (1, -1):
        draw(ax, BL.slider(g, rt, s), zt, COL["slider"])
    if state != "free":
        draw(ax, BL.finger_model(g, open_=(state == "latched")), zt, COL["finger"], 0.85)
    ax.set_aspect("equal"); ax.set_xlim(-45, 45); ax.set_ylim(-14, B["r"]["Y_M"] + 4)
    ax.axhline(0, color="#888", lw=0.6, ls=":")
    ttl = dict(latched="latched, fingers open", gripped="gripped: tabs squeezed 4.0, bolts in",
               free="free: out of the receiver, tabs at rest")[state]
    ax.set_title(f"{pack}: {ttl}", fontsize=9)
    ax.set_xlabel("x (mm), cartridge coords at the tab line z_T", fontsize=7); ax.tick_params(labelsize=7)


def iso(ax, meshes, T=T_ISO, title="", el=22, az=-60):
    """meshes [(Manifold, colour)] in one Poly3DCollection so the faces depth-sort together."""
    light = np.array([0.4, -0.5, 0.75]); light /= np.linalg.norm(light)
    tris, cols = [], []
    for m, c in meshes:
        t = BL.tm(m, T)
        sh = 0.45 + 0.55 * np.clip(t.face_normals @ light, 0, 1)
        tris.append(t.triangles); cols.append(np.clip(np.array(matplotlib.colors.to_rgb(c)) * sh[:, None], 0, 1))
    tri = np.vstack(tris)
    ax.add_collection3d(Poly3DCollection(tri, facecolors=np.vstack(cols), linewidths=0))
    V = tri.reshape(-1, 3); lo, hi = V.min(0), V.max(0); cc = (lo + hi) / 2; r = (hi - lo).max() / 2
    ax.set_xlim(cc[0] - r, cc[0] + r); ax.set_ylim(cc[1] - r, cc[1] + r); ax.set_zlim(cc[2] - r, cc[2] + r)
    ax.set_box_aspect((1, 1, 1)); ax.view_init(elev=el, azim=az); ax.set_axis_off(); ax.set_title(title, fontsize=9)


def assembly(pack, state, drop=0.0, skip=()):
    A = BL.latch_assembly(pack, state)
    return [(m.translate([0, -drop, 0]) if k in ("cartridge", "slider_px", "slider_nx", "pack", "xt60_plug",
                                                  "cartridge_skid_closed", "cartridge_skid_lead") else m, COL[KEYCOL[k]])
            for k, m in A.items() if k not in skip and not m.is_empty()]


def side(ax, m, x, c, a=1.0):
    """section of m at plane x (draws z horizontal, y up)."""
    t = BL.tm(m)
    p = t.section(plane_origin=[x, 0, 0], plane_normal=[1, 0, 0])
    if p is None: return
    for poly in p.discrete:
        ax.add_patch(Polygon(poly[:, [2, 1]], closed=True, fc=c, ec="none", alpha=a))


def compare(out=None):
    """receiver v1 vs v2 (body + XT60 carrier + cap), both packs: tab-line section and the wall's side section."""
    fig, axs = plt.subplots(4, 2, figsize=(13, 15), dpi=120, gridspec_kw=dict(width_ratios=[1, 2.2]))
    for i, pack in enumerate(("scout", "overseer")):
        B = BL.build(pack); g = B["g"]; zt = g["Z_T"] + 6.0      # beside the tab-line bolt, through the pockets
        for j, (lab, R, lid_) in enumerate((("v1", B["recv_v1"], BL.lid(B["r1"])), ("v2", B["recv"], B["lid"]))):
            ax = axs[2 * i + j, 0]
            draw(ax, BL.pack_body(g), zt, COL["pack"]); draw(ax, B["cart"], zt, COL["cartridge"])
            draw(ax, R, zt, COL["receiver"])
            for sg in (1, -1): draw(ax, BL.slider(g, 0.0, sg), zt, COL["slider"])
            ax.set_aspect("equal"); ax.set_xlim(-42, 42); ax.set_ylim(-4, B["r"]["Y_M"] + 4)
            ax.set_title(f"{pack} receiver {lab}: section at the tab line + 6", fontsize=9); ax.tick_params(labelsize=7)
            ax = axs[2 * i + j, 1]
            xs = B["r"]["RF"] + 1.5
            side(ax, BL.pack_body(g), 0.0, COL["pack"], 0.35); side(ax, B["cart"], xs - 3.0, COL["cartridge"], 0.5)
            side(ax, R, xs, COL["receiver"]); side(ax, lid_, 0.0, COL["lid"])
            if lab == "v2": side(ax, B["carrier"], 0.0, COL["xt60"])
            ax.set_aspect("equal"); ax.set_xlim(-12, g["L_C"] + 12); ax.set_ylim(-4, B["r"]["Y_M"] + 4)
            ax.set_title(f"{pack} receiver {lab}: side section through the wall (x {xs:.1f}); carrier and cap at x 0",
                         fontsize=9)
            ax.set_xlabel("z (mm), closed end left, XT60 end right", fontsize=7); ax.tick_params(labelsize=7)
    fig.tight_layout(); fig.savefig(out or os.path.join(OUT, "compare_v1_v2.png")); plt.close(fig)


def _clip_image(ax, img, extent, polys, vmax, cmap="viridis"):
    from matplotlib.path import Path
    from matplotlib.patches import PathPatch
    verts, codes = [], []
    for p in polys:
        if len(p) < 3: continue
        verts += list(p) + [p[0]]; codes += [Path.MOVETO] + [Path.LINETO] * (len(p) - 1) + [Path.CLOSEPOLY]
    if not verts: return None
    patch = PathPatch(Path(verts, codes), fc="none", ec="#222", lw=0.4)
    ax.add_patch(patch)
    im = ax.imshow(img, origin="lower", extent=extent, cmap=cmap, vmin=0, vmax=vmax, interpolation="bilinear")
    im.set_clip_path(patch)
    return im


def fe_sections(case="swap insert 74 N + across 43.5 N", out=None):
    """|u| (mm) of receiver v1 and v2 under one swap case (battery_latch_topo --swap-v1 / --swap-v2 saved the node fields
    in runs/battery_latch/v2/fe/), on the tab-line section and the side section through the wall, same colour scale."""
    import trimesh as T_
    d = os.path.join(BL.OUT, "fe")
    key = case.replace(" ", "_")
    fig, axs = plt.subplots(4, 2, figsize=(13, 15), dpi=120, gridspec_kw=dict(width_ratios=[1, 2.2]))
    for i, pack in enumerate(("scout", "overseer")):
        if not all(os.path.exists(os.path.join(d, f"field_{pack}_v{v}.npz")) for v in (1, 2)):
            continue
        B = BL.build(pack); g = B["g"]
        F = {v: np.load(os.path.join(d, f"field_{pack}_v{v}.npz")) for v in (1, 2)}
        meshes = {1: B["recv_v1"] + BL.lid(B["r1"]), 2: BL.U(B["recv"], B["lid"])}
        sl = {}
        for v in (1, 2):
            o, h = F[v]["origin"], float(F[v]["h"]); u = F[v][key]
            zt = g["Z_T"] + 6.0; xs = B["r"]["RF"] + 1.5
            kz = int(round((zt - o[2]) / h)); kx = int(round((xs - o[0]) / h))
            n = u.shape
            sl[v] = dict(xy=(u[:, :, kz].T, [o[0], o[0] + (n[0] - 1) * h, o[1], o[1] + (n[1] - 1) * h]),
                         zy=(u[kx, :, :], [o[2], o[2] + (n[2] - 1) * h, o[1], o[1] + (n[1] - 1) * h]), zt=zt, xs=xs)
        # colour scale: the 95th percentile of |u| inside the material on both versions' sections
        vals = [np.percentile(sl[v][k][0][sl[v][k][0] > 0], 99) for v in (1, 2) for k in ("xy", "zy")]
        vmax = float(max(vals))
        for j, v in enumerate((1, 2)):
            t = BL.tm(meshes[v])
            ax = axs[2 * i + j, 0]
            sec = t.section(plane_origin=[0, 0, sl[v]["zt"]], plane_normal=[0, 0, 1])
            polys = [p[:, :2] for p in sec.discrete] if sec is not None else []
            im = _clip_image(ax, *sl[v]["xy"], polys, vmax)
            draw(ax, B["cart"], sl[v]["zt"], COL["cartridge"], 0.25)
            ax.set_aspect("equal"); ax.set_xlim(-42, 42); ax.set_ylim(-4, B["r"]["Y_M"] + 4); ax.tick_params(labelsize=7)
            ax.set_title(f"{pack} receiver v{v}: |u| at the tab line + 6", fontsize=9)
            ax = axs[2 * i + j, 1]
            sec = t.section(plane_origin=[sl[v]["xs"], 0, 0], plane_normal=[1, 0, 0])
            polys = [p[:, [2, 1]] for p in sec.discrete] if sec is not None else []
            im = _clip_image(ax, *sl[v]["zy"], polys, vmax)
            ax.set_aspect("equal"); ax.set_xlim(-12, g["L_C"] + 12); ax.set_ylim(-4, B["r"]["Y_M"] + 4); ax.tick_params(labelsize=7)
            ax.set_title(f"{pack} receiver v{v}: |u| through the wall (x {sl[v]['xs']:.1f}), {case}", fontsize=9)
            if im is not None:
                cb = fig.colorbar(im, ax=ax, fraction=0.025, pad=0.01); cb.set_label("|u| mm (E loads)", fontsize=7)
                cb.ax.tick_params(labelsize=6)
    fig.tight_layout(); fig.savefig(out or os.path.join(OUT, f"fe_{key}.png")); plt.close(fig)


def main():
    os.makedirs(OUT, exist_ok=True)
    fig, axs = plt.subplots(2, 3, figsize=(14, 8.4), dpi=130)
    for i, pack in enumerate(("scout", "overseer")):
        for j, st in enumerate(("latched", "gripped", "free")):
            section(pack, axs[i, j], st)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "sections.png")); plt.close(fig)

    # assembled sets from latch_assembly: latched (lid hidden to show the bolts) and the cartridge dropped out (free)
    fig = plt.figure(figsize=(12, 9), dpi=130)
    for i, pack in enumerate(("scout", "overseer")):
        iso(fig.add_subplot(2, 2, 2 * i + 1, projection="3d"), assembly(pack, "latched", skip=("receiver_lid",)),
            title=f"{pack} set latched (lid off)")
        A = BL.latch_assembly(pack, "latched")
        free = assembly(pack, "free", drop=40.0) + [(A[k], COL[KEYCOL[k]]) for k in ("receiver", "receiver_strips", "xt60e1m",
                                                                                        "xt60_carrier") if k in A]
        iso(fig.add_subplot(2, 2, 2 * i + 2, projection="3d"), free, title=f"{pack}: cartridge out 40 mm (free)")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "sets.png")); plt.close(fig)


    # Scout side view in the drone frame with the collar planes (inset 7.0 line) and the keel nodes
    B = BL.build("scout"); g = B["g"]; T = BL.T_dc(g)
    fig, axs = plt.subplots(1, 2, figsize=(10, 4.2), dpi=140)
    for k, (ax, i, lab) in enumerate(((axs[0], 0, "x (drone, nose +)"), (axs[1], 1, "y (drone, left +)"))):
        for m, c in ((B["recv"], COL["receiver"]), (B["cart"], COL["cartridge"]), (B["lid"], COL["lid"])):
            t = BL.tm(m, T)
            ax.scatter(t.vertices[:, i], t.vertices[:, 2], s=0.4, c=c)
        zz = np.linspace(-50, 20, 50)
        for sgn in (-1, 1):
            ax.plot(sgn * (BL.COLLAR_K + zz), zz, color="#333", lw=1.0)
            ax.plot(sgn * (BL.COLLAR_K + zz - BL.COLLAR_NEED), zz, color="#c0392b", lw=0.8, ls="--")
        for kx, ky in ((28, -28.5), (-28, -28.5), (28, 28.5)):
            ax.plot([(kx, ky)[i]], [-47.5], "o", color="#2a7f62", ms=5)
        ax.set_aspect("equal"); ax.set_xlim(-75, 75); ax.set_ylim(-60, 25); ax.set_xlabel(lab, fontsize=8)
        ax.tick_params(labelsize=7)
    axs[0].set_title("Scout: collar planes (black), 7.0 inside (red), keel nodes (green)", fontsize=9)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "scout_collar.png")); plt.close(fig)
    compare()
    for c in ("swap insert 74 N + across 43.5 N", "swap across 43.5 N +x"):
        if os.path.isdir(os.path.join(BL.OUT, "fe")): fe_sections(c)
    print("figures in", OUT)


if __name__ == "__main__":
    main()
