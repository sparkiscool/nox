"""Battery latch v1 figures for the layout artifact, all from the script's geometry: tab-line sections (latched / gripped /
free, both packs), the assembled sets (latch_assembly, Scout end skids), pitch guidance option B (Overseer receiver, GUIDE)
and the Scout set in the collar planes. python3 nox/box/battery_latch_figure.py -> runs/battery_latch/v1/figs/*.png"""
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
              xt60e1m="xt60")
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
        free = assembly(pack, "free", drop=40.0) + [(A[k], COL[KEYCOL[k]]) for k in ("receiver", "receiver_strips", "xt60e1m")]
        iso(fig.add_subplot(2, 2, 2 * i + 2, projection="3d"), free, title=f"{pack}: cartridge out 40 mm (free)")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "sets.png")); plt.close(fig)

    # pitch guidance option B (GUIDE, end gap 0.15, truss windows) next to the DEFAULT receiver, Overseer
    fig = plt.figure(figsize=(12, 5), dpi=130)
    keep = (BL.GUIDE, BL.END_GAP, BL.WINDOWS)
    for k, opt in enumerate((False, True)):
        BL.GUIDE, BL.END_GAP, BL.WINDOWS = (opt, 0.15 if opt else 0.3, "truss" if opt else "box"); BL._CACHE.clear()
        B = BL.build("overseer")
        iso(fig.add_subplot(1, 2, k + 1, projection="3d"), [(B["recv"], COL["receiver"]), (B["strips"], COL["strips"])],
            title="overseer receiver, " + ("option B: corner guide posts, truss windows" if opt else "DEFAULT: no guide posts"),
            el=18, az=-35)
    BL.GUIDE, BL.END_GAP, BL.WINDOWS = keep; BL._CACHE.clear()
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "pitch_option_b.png")); plt.close(fig)

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
    print("figures in", OUT)


if __name__ == "__main__":
    main()
