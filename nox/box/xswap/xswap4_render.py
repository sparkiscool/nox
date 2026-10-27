"""v4 figures: plan section of the latch (latched / released), end section through tip + ledge, assembly, prints."""
import glob, math, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import trimesh
import xswap4 as X
import zrender as ZR

FIG = os.path.join(X.OUT, "figs"); os.makedirs(FIG, exist_ok=True)
COL = dict(holder="#3f6fb5", shell="#e0892b", bar="#b04a1a", finger="#2f9e5a", jaw="#7fbf7f", pack="#4a4a4a", plug="#d9b92e", servo="#999999")

def sect_z(m, z):
    return [np.array(p) for p in m.slice(z).to_polygons()]

def sect_y(m, y):
    r = m.rotate([90, 0, 0]); return [np.array([(px, -py) for px, py in p]) for p in r.slice(y).to_polygons()]

def sect_x(m, x):
    r = m.rotate([0, -90, 0]); return [np.array([(py, -px) for px, py in p]) for p in r.slice(x).to_polygons()]

def fill(ax, polys, c, a=1.0, z=1, lw=0.4):
    for p in polys:
        if len(p) > 2: ax.add_patch(Polygon(p, closed=True, fc=c, ec="k", lw=lw, alpha=a, zorder=z))

def tmesh(m, c): return (X.tm(m), matplotlib.colors.to_rgb(c))

if __name__ == "__main__":
    body, H = X.shell_body(), X.holder()
    th_r = X.theta_from_u(X.STROKE, 0.0)

    # 1 plan section through the tip level and the bar body: latched vs released (the +y end, bar A)
    fig, axs = plt.subplots(1, 2, figsize=(13, 8.2))
    for ax, (th, u, ttl) in zip(axs, [(0.0, 0.0, "Latched: tip in the leg pocket, on the ledge"),
                                       (th_r, X.STROKE, "Released: finger moved %.1f mm, tip swung %.1f mm" % (X.STROKE, X.L_TIP * math.sin(math.radians(th_r))))]):
        zc = -63.0
        fill(ax, sect_z(H, zc), COL["holder"], 1, 1)
        fill(ax, sect_z(body, zc), COL["shell"], 0.55, 1)
        fill(ax, sect_z(X.bar(th, "A"), zc), COL["bar"], 1, 3)
        fill(ax, sect_z(X.finger_A(u), zc), COL["finger"], 1, 4)
        ax.plot([0], [X.PIV_Y], "o", ms=5, color="k", zorder=5)
        # spring (bought): coil from the boss pocket to the bar's stub
        x0 = X.SPR_BOSS_X1 - X.SPR_POCKET
        pa = np.array([0.0 - X.BODY_W / 2, X.SPR_Y]); pa = np.array([pa[0] * math.cos(math.radians(th)) - (pa[1] - X.PIV_Y) * math.sin(math.radians(th)),
                                                                    X.PIV_Y + pa[0] * math.sin(math.radians(th)) + (pa[1] - X.PIV_Y) * math.cos(math.radians(th))])
        n = 9; xs_ = np.linspace(x0, pa[0], 2 * n + 1); ys_ = np.linspace(X.SPR_Y, pa[1], 2 * n + 1) + np.array([0] + [2.6 * (-1) ** k for k in range(2 * n - 1)] + [0])
        ax.plot(xs_, ys_, color="#555", lw=1.4, zorder=6)
        ax.set_xlim(-30, 32); ax.set_ylim(26, 100); ax.set_aspect("equal"); ax.grid(alpha=0.25)
        ax.set_title(ttl, fontsize=10.5); ax.set_xlabel("x across the pack (mm)"); ax.set_ylabel("y along the pack (mm)")
        ax.annotate("steel dowel (pivot)", (0, X.PIV_Y), (8, 30), arrowprops=dict(arrowstyle="->"), fontsize=8.5)
        ax.annotate("finger post in the bar's slot", (X.finger_A(u).bounding_box()[0] + 3.5, X.FIN_Y), (9, 42), arrowprops=dict(arrowstyle="->"), fontsize=8.5)
        ax.annotate("spring", (-14, X.SPR_Y), (-29, 63), arrowprops=dict(arrowstyle="->"), fontsize=8.5)
        ax.annotate("holder end leg", (20, 92.5), (10, 98.3), fontsize=8.5)
        ax.annotate("shell end wall (band above)", (14, 87), (9, 80), arrowprops=dict(arrowstyle="->"), fontsize=8.5)
        ax.annotate("release channel", (-14, 91.5), (-29, 97.5), arrowprops=dict(arrowstyle="->"), fontsize=8.5)
    fig.suptitle("Gate bar, plan section 3 mm above the tip's load face (the -y end is the same part turned 180 deg)", fontsize=12)
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "latch_plan.png"), dpi=110); plt.close(fig)

    # 2 side section through the tip, the leg, the band, the finger (x = 0 plane, latched)
    fig, ax = plt.subplots(figsize=(12, 5.6))
    fill(ax, sect_x(X.pack(), 0.0), COL["pack"], 0.25, 0)
    fill(ax, sect_x(H, 0.0), COL["holder"], 1, 1)
    fill(ax, sect_x(body, 0.0), COL["shell"], 1, 2)
    fill(ax, sect_x(X.bar(0.0, "A"), 0.0), COL["bar"], 1, 3)
    fill(ax, sect_x(X.finger_A(0.0), 0.0), COL["finger"], 1, 4)
    fill(ax, sect_x(X.jaw_body(), 0.0), COL["jaw"], 0.6, 1)
    ax.set_xlim(20, 100); ax.set_ylim(-90, -45); ax.set_aspect("equal"); ax.grid(alpha=0.25)
    ax.set_xlabel("y along the pack (mm)"); ax.set_ylabel("z up (mm)")
    ax.set_title("Section on the centre line (x 0): pull-out load goes tip -> ledge; the shell hangs on the tip through its end band", fontsize=10.5)
    for txt, xy, xyt in [("tip on the 12 mm ledge", (92, -67.6), (70, -84)), ("end band", (82, -58), (62, -49)),
                         ("pack floor", (55, -54), (40, -48)), ("finger 7.7 deep in the bar", (45.5, -60), (27, -62)),
                         ("T-slot lips hold the flange", (45.5, -68), (24, -76)), ("holder end leg", (94.5, -60), (88, -48))]:
        ax.annotate(txt, xy, xyt, arrowprops=dict(arrowstyle="->"), fontsize=8.5)
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "latch_side.png"), dpi=110); plt.close(fig)

    # 3 assembly from below (holder see-through) and the pincer
    parts = [tmesh(H, COL["holder"]), tmesh(body, COL["shell"]), tmesh(X.bar(0.0, "A"), COL["bar"]), tmesh(X.bar(0.0, "B"), COL["bar"])]
    p = X.pincer(X.STROKE)
    parts_p = [tmesh(p["jaw"], COL["jaw"]), tmesh(p["finger_a"], COL["finger"]), tmesh(p["finger_b"], COL["finger"]), tmesh(p["pinion"], "#1f6e3d")]
    for name, items, el, az, ttl in [("assembly", parts + parts_p, -28, -50, "Shell in the drone holder, pincer under it (narrower than the pack)"),
                                     ("pincer", parts_p + [tmesh(p["servo"], COL["servo"])], 28, -60, "Pincer: 44 mm wide, two round finger posts with flanges"),
                                     ("shell_under", [tmesh(body, COL["shell"]), tmesh(X.bar(0.0, "A"), COL["bar"]), tmesh(X.bar(0.0, "B"), COL["bar"])], -55, -60,
                                      "Shell from below: two keyholes for the fingers, bar tips at both ends")]:
        img = ZR.render(items, elev=el, azim=az, size=900)
        fig = plt.figure(figsize=(8, 8)); a = fig.add_axes([0, 0, 1, 0.94]); a.imshow(img); a.set_axis_off()
        fig.suptitle(ttl, fontsize=11); fig.savefig(os.path.join(FIG, name + ".png"), dpi=110); plt.close(fig)

    # 4 print previews
    files = sorted(glob.glob(os.path.join(X.OUT, "stl", "*.stl"))) + sorted(glob.glob(os.path.join(X.OUT, "coupons", "*.stl")))
    fig = plt.figure(figsize=(16, 13))
    for i, f in enumerate(files):
        t = trimesh.load(f); n = t.face_normals
        down = n[:, 2] < -math.cos(math.radians(45)); bed = down & (t.triangles_center[:, 2] < 0.2)
        items = [(t.submesh([np.where(mk)[0]], append=True), c) for mk, c in ((~down, (0.8, 0.8, 0.77)), (down & ~bed, (0.92, 0.32, 0.26)), (bed, (0.3, 0.45, 0.92))) if mk.any()]
        img = ZR.render(items, elev=24, azim=-55, size=480, bounds=t.vertices)
        a = fig.add_subplot(3, 4, i + 1); a.imshow(img); a.set_axis_off()
        a.set_title("%s\n%.0f x %.0f x %.0f mm, %.0f g solid" % (os.path.basename(f)[:-4], *t.extents, t.volume * X.PETG), fontsize=9)
    fig.suptitle("v4 print files and coupons, print orientation (blue = bed, red = overhang: tree supports)")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "prints.png"), dpi=95); plt.close(fig)
    print("figs ->", FIG)
