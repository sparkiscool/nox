"""Renders for xswap v1: assembly, clip section, swap sequence, print previews. python3 xswap_render.py"""
import os, math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.patches import Polygon
import xswap as X
import zrender as ZR

FIG = os.path.join(X.OUT, "figs"); os.makedirs(FIG, exist_ok=True)
YC, ZP = X.Y_C, X.Z_P
COL = dict(holder="#5b8def", cap="#3b6fd0", shell="#f2a541", keeper="#d98a2b", pack="#4a4a4a", female="#e8d44d", male="#e8d44d",
           jaw="#7fbf7f", finger="#2f9e5a", pinion="#1f6e3d", servo="#999999")
LIGHT = np.array([0.4, -0.6, 0.7]); LIGHT /= np.linalg.norm(LIGHT)

def draw(ax, m, color, alpha=1.0):
    t = X.tm(m)
    n = t.face_normals
    shade = 0.45 + 0.55 * np.clip(n @ LIGHT, 0, 1)
    rgb = np.array(matplotlib.colors.to_rgb(color))
    pc = Poly3DCollection(t.triangles, facecolors=np.clip(rgb * shade[:, None], 0, 1), edgecolor="none", alpha=alpha)
    ax.add_collection3d(pc)
    return t.bounds

def frame(ax, lo, hi, elev=18, azim=-60):
    c = (lo + hi) / 2; r = (hi - lo).max() / 2
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect([1, 1, 1]); ax.view_init(elev=elev, azim=azim); ax.set_axis_off()

def scene(items, path, title, elev=18, azim=-60, size=(8, 8), bounds=None):
    parts = [(X.tm(m), matplotlib.colors.to_rgb(c)) for m, c, a in items]
    img = ZR.render(parts, elev=elev, azim=azim, size=int(size[0] * 110), bounds=bounds)
    fig = plt.figure(figsize=size); ax = fig.add_axes([0, 0, 1, 0.94]); ax.imshow(img); ax.set_axis_off()
    fig.suptitle(title, fontsize=11); fig.savefig(path, dpi=110); plt.close(fig)

def pincer_items(p):
    return [(p["jaw"], COL["jaw"], 1), (p["finger_p"], COL["finger"], 1), (p["finger_m"], COL["finger"], 1),
            (p["pinion"], COL["pinion"], 1), (p["servo"], COL["servo"], 1)]

def sect(m, y):
    """section of m by the plane y = y, as polygons in (x, z)."""
    r = m.rotate([90, 0, 0])            # (x, y, z) -> (x, -z, y): plane y = y becomes z' = y
    cs = r.slice(y)
    return [np.array([(px, -py) for px, py in poly]) for poly in cs.to_polygons()]

def fill(ax, polys, color, alpha=1.0, z=1):
    for p in polys:
        if len(p) > 2:
            ax.add_patch(Polygon(p, closed=True, facecolor=color, edgecolor="k", lw=0.4, alpha=alpha, zorder=z))

if __name__ == "__main__":
    shell_r, shell_p = X.shell(YC), X.shell(YC, X.PINCH)
    H, cap = X.holder(YC), X.xt60_cap()
    pin_o, pin_s = X.pincer(YC, X.FACE_OPEN), X.pincer(YC, X.FACE_SHUT)
    base = [(H, COL["holder"], 1), (cap, COL["cap"], 1), (X.xt60_male(), COL["male"], 1)]

    # 1 assembly at the drone: shell seated, pincer shut
    scene(base + [(shell_p, COL["shell"], 1), (X.xt60_keeper(), COL["keeper"], 1)] + pincer_items(pin_s),
          os.path.join(FIG, "assembly.png"), "At the drone: holder (blue), shell (orange), pincer shut on the clips (green). View from below", elev=-22, azim=-50)
    scene([(H, COL["holder"], 1), (cap, COL["cap"], 1), (shell_r, COL["shell"], 1)],
          os.path.join(FIG, "holder_shell.png"), "Holder (blue) and shell (orange) latched, from below", elev=-30, azim=-35)
    scene([(shell_r, COL["shell"], 1), (X.xt60_keeper(), COL["keeper"], 1), (X.xt60_female(), COL["female"], 1)],
          os.path.join(FIG, "shell.png"), "Battery shell: clip beams at the CG, XT60 block at the lead end", elev=25, azim=-60)
    scene(pincer_items(pin_o), os.path.join(FIG, "pincer.png"), "Pincer, open: rack + pinion fingers, flip hubs both sides", elev=28, azim=-60)

    # 2 clip section at y = YC: relaxed + latched (left), pinched + gripped (right)
    fig, axs = plt.subplots(1, 2, figsize=(12, 6.5))
    for ax, (sh, p, ttl) in zip(axs, [(shell_r, pin_o, "Latched: nose rests on the holder's tooth, finger open"),
                                       (shell_p, pin_s, "Pinched: nose in the pad pocket, clip pulled in 1.5, free to drop")]):
        fill(ax, sect(X.pack(), YC), COL["pack"], 0.25, 0)
        fill(ax, sect(sh, YC), COL["shell"], 1, 2)
        fill(ax, sect(H, YC), COL["holder"], 1, 2)
        for k in ("finger_p", "finger_m"): fill(ax, sect(p[k], YC), COL["finger"], 1, 3)
        fill(ax, sect(p["jaw"], YC), COL["jaw"], 0.6, 1)
        ax.set_xlim(18, 50); ax.set_ylim(-48, 14); ax.set_aspect("equal"); ax.grid(alpha=0.3)
        ax.set_title(ttl, fontsize=10); ax.set_xlabel("x across the pack (mm)"); ax.set_ylabel("z up (mm)")
    for ax in axs:
        ax.annotate("tooth (holder)", (X.TOOTH_TIP + 0.5, X.TOOTH_Z1 - 1.5), (40, -18), arrowprops=dict(arrowstyle="->"), fontsize=8)
        ax.annotate("latch nose", (X.BX1 + 0.5, X.NOSE_Z0 + 1), (38, -6), arrowprops=dict(arrowstyle="->"), fontsize=8)
        ax.annotate("pad + finger pocket", (X.BX1 + 1.5, ZP), (40, -42), arrowprops=dict(arrowstyle="->"), fontsize=8)
        ax.annotate("clip beam (flexure)", (X.BX0 + 1, -12), (19, 6), arrowprops=dict(arrowstyle="->"), fontsize=8)
        ax.annotate("XT60 deck", (25, 5), (36, 10), fontsize=8)
    fig.suptitle("Section through the clip (y = %.1f, the swap CG): one squeeze unlatches and grips" % YC)
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "clip_section.png"), dpi=110); plt.close(fig)

    # 3 swap sequence (side view along x)
    D1, D2 = 80.0, 85.0
    bay = lambda m: m.translate([0, -YC, -ZP]).rotate([180, 0, 0]).translate([0, YC, ZP - D1 - D2])
    bayH = [(bay(H), COL["holder"], 1), (bay(cap), COL["cap"], 1)]
    def carry(t, ang, p, sh):
        ms = [(sh, COL["shell"], 1), (X.pack(), COL["pack"], 1)] + pincer_items(p)
        out = []
        for m, c, a in ms:
            m = m.translate([0, 0, -t])
            if ang: m = m.translate([0, -YC, -(ZP - t)]).rotate([ang, 0, 0]).translate([0, YC, ZP - t])
            out.append((m, c, a))
        return out
    steps = [("1 rise open under the pack", carry(60, 0, pin_o, shell_r) if False else
              [(shell_r, COL["shell"], 1), (X.pack(), COL["pack"], 1)] + [(m.translate([0, 0, -40]), c, a) for m, c, a in pincer_items(pin_o)]),
             ("2 pinch: unlatch + grip", carry(0, 0, pin_s, shell_p)),
             ("3 down 80: XT60 out, clear", carry(D1, 0, pin_s, shell_p)),
             ("4 flip 90", carry(D1, 90, pin_s, shell_p)),
             ("5 flip 180", carry(D1, 180, pin_s, shell_p)),
             ("6 down 85 into the bay, open", carry(D1 + D2, 180, pin_o, shell_r))]
    B = [[-80, -100, -262], [80, 100, 22]]
    B = [[x, y, z] for x in (B[0][0], B[1][0]) for y in (B[0][1], B[1][1]) for z in (B[0][2], B[1][2])]
    fig = plt.figure(figsize=(13, 15))
    for i, (ttl, items) in enumerate(steps):
        parts = [(X.tm(m), matplotlib.colors.to_rgb(c)) for m, c, a in base + bayH + items]
        img = ZR.render(parts, elev=8, azim=-12, size=640, pad=0.01, bounds=B)
        ax = fig.add_subplot(2, 3, i + 1); ax.imshow(img); ax.set_axis_off(); ax.set_title(ttl, fontsize=12)
    fig.suptitle("Swap sequence (rail = vertical). Drone holder on top, bay holder (same print, flipped) at the bottom")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "sequence.png"), dpi=100); plt.close(fig)

    # 4 print previews (print orientation; red = overhang > 45 deg, needs support)
    import glob, trimesh
    files = sorted(glob.glob(os.path.join(X.OUT, "stl", "*.stl")))
    fig = plt.figure(figsize=(18, 10))
    for i, f in enumerate(files):
        t = trimesh.load(f)
        n = t.face_normals
        down = n[:, 2] < -math.cos(math.radians(45)); bed = down & (t.triangles_center[:, 2] < 0.2)
        parts = []
        for mask, c in ((~down, (0.80, 0.80, 0.77)), (down & ~bed, (0.92, 0.32, 0.26)), (bed, (0.30, 0.45, 0.92))):
            if mask.any(): parts.append((t.submesh([np.where(mask)[0]], append=True), c))
        img = ZR.render(parts, elev=24, azim=-55, size=520, bounds=t.vertices)
        ax = fig.add_subplot(2, 4, i + 1); ax.imshow(img); ax.set_axis_off()
        nm = os.path.basename(f)[:-4]
        cnt = {"xswap_finger_v1": " (print 2)"}.get(nm, "")
        ax.set_title("%s%s\n%.0f x %.0f x %.0f mm, %.0f g solid" % (nm, cnt, *t.extents, t.volume * X.PETG), fontsize=9)
    fig.suptitle("Print files in print orientation, bed at the bottom (blue = on the bed, red = overhang: tree supports)")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "prints.png"), dpi=95); plt.close(fig)
    print("figs ->", FIG)
