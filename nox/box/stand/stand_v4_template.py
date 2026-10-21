"""Box stand v4 (wood): 1:1 drill template, US letter tiles (nox/box-stand.md, 17 Oct 2026 22:xx).
    python stand_v4_template.py   (stand_v4.py beside it)  ->  box_stand_v4_drill_template_v1.pdf
Sheet A  upright, seen from its BACK face (X to the right, as stand_v4.py and the layout page; the face you drive the
         block screws from): axle centre, Ø26 lead hole, stub flange 4 x M5 at r 38 on the 45 deg diagonals, the end
         outlines of the 2 funnel blocks (45 deg) and the +15 deg bay block, 3 screw holes per block, base edge.
Sheet B  base, seen from above (X right, +Y up the page = toward the box): upright footprint and its 4 screws, the -45 deg
         wedge footprint and its 2 screws.
Patterns 1:1 strips for the faces that take the lugs and the collar seats (bay block, funnel block, wedge) and the
         wedge's cross-section (rip pattern, 45 deg cut line).
Every position from the axle (W, mm). Pages overlap 10 mm; orange crosses are shared by neighbouring pages."""
import json, math, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib import font_manager as fm
import stand_v4 as SV

for f in fm.findSystemFonts():
    if "Inter" in os.path.basename(f): fm.fontManager.addfont(f)
plt.rcParams.update({"font.family": "Inter", "pdf.fonttype": 3, "font.size": 7})

INK, MUTED, FAINT, ACCENT, WOOD, WOODL = "#1d1d1f", "#6e6e73", "#c7c7cc", "#e8590c", "#8a5a2b", "#f6ede3"
C = {"stub": "#1d1d1f", "block": "#0f766e", "base": "#4338ca", "lug": "#c2410c", "seat": "#7c3aed"}
LET_W, LET_H, MARG, FOOT = 215.9, 279.4, 12.0, 14.0
TILE_W, TILE_H, OVL = LET_W - 2 * MARG, LET_H - 2 * MARG - FOOT, 10.0
P = {k: np.array(v["v"])[:len(v["v"]) // 2] for k, v in json.load(open("stand_v4_parts.json")).items()}
c45 = math.sqrt(0.5)

# ------------------------------------------------------------------ holes (C from stand_v4 / box-stand.md; screw spots DEFAULT)
def block_screws(poly):
    """3 screws through the upright into the block's end grain, on the long centreline at -28 / 0 / +28 (DEFAULT)"""
    q = poly[:, [0, 2]]; c = q.mean(0)
    e = [q[1] - q[0], q[3] - q[0]]; L = max(e, key=np.linalg.norm); u = L / np.linalg.norm(L)
    return [c + k * 28.0 * u for k in (-1, 0, 1)]

def upright_holes():
    hs = [("AX", "stub", 0.0, 0.0, 26.0, "axle centre: lead bundle (stub bore 24)")]
    for i, a in enumerate((45, 135, 225, 315)):
        hs.append((f"S{i + 1}", "stub", 38 * math.cos(math.radians(a)), 38 * math.sin(math.radians(a)), 5.5, "stub flange M5, nut on this face"))
    for tag, k in (("FR", "funnel_block_R"), ("FL", "funnel_block_L"), ("BB", "bay_block_p15")):
        for i, (x, z) in enumerate(block_screws(P[k])):
            hs.append((f"{tag}{i + 1}", "block", float(x), float(z), 5.5, "screw 5 x 40 into the block end, countersink"))
    return hs

UP_SCREW_X = (-200.0, -70.0, 70.0, 200.0)          # base -> upright bottom edge (DEFAULT)
def base_holes():
    hs = [(f"U{i + 1}", "base", x, -9.0, 5.5, "screw 5 x 40 up into the upright's bottom edge") for i, x in enumerate(UP_SCREW_X)]
    w = P["bay_wedge_m45"]; xc = 0.5 * (w[0, 0] + w[1, 0])
    for i, y in enumerate((155.0, 200.0)):
        hs.append((f"W{i + 1}", "base", float(xc), y, 5.5, "screw 5 x 40 up into the wedge"))
    return hs

# face patterns: (across, along) in mm from the named edges; holes C
FACES = [
    dict(name="Bay block +15 deg", w=89.0, l=230.0, across="from the axle-side edge (r 155)", along="from the upright end (Y 0)",
         holes=[("L1", 172 - 155, 145.0), ("L2", 172 - 155, 209.0)], kind="lug",
         note="the 89 face on the +15 deg plane takes the 0 deg module's 2 free-side lugs. Drill Ø3 pilot for 5 x 50 wood screws, or Ø5.5 through the 38 for M5 x 60 + nut."),
    dict(name="Funnel block (make 2: L and R)", w=89.0, l=230.0, across="from the lower edge of the 45 deg face", along="from the upright end (Y 0)",
         holes=[("C1", 10.0, 141.0), ("C2", 10.0, 213.0)], kind="seat",
         note="the 89 face lies on the funnel side panel. Drill Ø3.4 square through the 38 for M3 x 50 into the collar inserts."),
    dict(name="Bay wedge -45 deg", w=42.0, l=85.0, across="from the axle-side edge (r 158)", along="from the end nearer the upright (Y 135)",
         holes=[("L3", 172 - 158, 145 - 135.0), ("L4", 172 - 158, 209 - 135.0)], kind="lug",
         note="the 45 deg face takes the -30 deg module's 2 free-side lugs: Ø3 pilot (5 x 50) or Ø5.5 (M5 x 60 + nut)."),
]


# ------------------------------------------------------------------ drawing helpers
def cross(ax, x, y, d, c, lw=0.35, tick=3.5):
    r = d / 2 + tick
    ax.plot([x - r, x + r], [y, y], color=c, lw=lw); ax.plot([x, x], [y - r, y + r], color=c, lw=lw)
    ax.add_patch(plt.Circle((x, y), d / 2, fill=False, lw=0.75, ec=c)); ax.plot([x], [y], "o", ms=0.7, color=c)

def label(ax, x, y, d, lab, c, sub):
    o = d / 2 + 2.2
    ax.text(x + o, y + o, lab, color=c, fontsize=6.8, fontweight="semibold", ha="left", va="bottom")
    ax.text(x + o, y + o - 0.6, sub, color=MUTED, fontsize=4.4, ha="left", va="top")

def reg(ax, xs, ys):
    for x in xs:
        for y in ys:
            ax.plot([x - 6, x + 6], [y, y], color=ACCENT, lw=0.45); ax.plot([x, x], [y - 6, y + 6], color=ACCENT, lw=0.45)
            ax.add_patch(plt.Circle((x, y), 3.0, fill=False, lw=0.45, ec=ACCENT))

def footer(fig, page, title, sub):
    fa = fig.add_axes([0, 0, 1, (MARG + FOOT - 2) / LET_H]); fa.set_xlim(0, LET_W); fa.set_ylim(0, MARG + FOOT - 2); fa.axis("off")
    fa.text(MARG, 15, f"{page}", fontsize=9, color=ACCENT, fontweight="bold")
    fa.text(MARG + 8, 15, title, fontsize=7, color=INK, fontweight="semibold")
    fa.text(MARG + 8, 10.5, sub, fontsize=5.8, color=MUTED)
    bx = LET_W - MARG - 100
    fa.plot([bx, bx + 100], [13, 13], color=INK, lw=0.8)
    for k in range(11): fa.plot([bx + 10 * k] * 2, [13, 14.6 if k % 5 else 15.6], color=INK, lw=0.4)
    fa.text(bx + 50, 8.6, "100 mm  ·  check before drilling", ha="center", fontsize=5.6, color=MUTED)

def tiles(xa, xb, ya, yb, top=None):
    """tile grid over the drawing; top = the first row's top edge (to keep the overlap bands off the holes)"""
    nx = max(1, math.ceil((xb - xa - OVL) / (TILE_W - OVL))); ny = max(1, math.ceil((yb - ya - OVL) / (TILE_H - OVL)))
    cx = (xa + xb) / 2 - (nx * (TILE_W - OVL) + OVL) / 2; cy = (ya + yb) / 2 + (ny * (TILE_H - OVL) + OVL) / 2
    if top is not None: cy = top
    out = []
    for r in range(ny):
        for c in range(nx):
            x0 = cx + c * (TILE_W - OVL); y1 = cy - r * (TILE_H - OVL)
            out.append((r, c, x0, x0 + TILE_W, y1 - TILE_H, y1))
    gx = sorted({t[2] + OVL / 2 for t in out if t[1] > 0}) or [out[0][2] + TILE_W / 2]
    gy = sorted({t[5] - OVL / 2 for t in out if t[0] > 0}) or [out[0][5] - TILE_H / 2]
    gx = [out[0][2] + 25] + gx + [out[-1][3] - 25]; gy = [out[-1][4] + 25] + gy + [out[0][5] - 25]
    return out, nx, ny, gx, gy

def poly(ax, q, c, fill=None, lw=0.8, ls="-", z=1):
    q = np.vstack([q, q[:1]])
    if fill: ax.fill(q[:, 0], q[:, 1], color=fill, lw=0, zorder=z - 1)
    ax.plot(q[:, 0], q[:, 1], color=c, lw=lw, ls=ls, zorder=z)


# ------------------------------------------------------------------ sheet A: upright, back-face view (X right, Z up)
def draw_upright(ax, hs):
    up = P["upright"][:, [0, 2]]; base = P["base"][:, [0, 2]]
    poly(ax, up, INK, lw=0.9)
    poly(ax, base, MUTED, fill="#f2f2f5", lw=0.5)
    ax.text(0, -161, "base (its back edge, below the upright)  ·  base top Z -152", ha="center", va="center", fontsize=6, color=MUTED)
    ax.plot([-262, 262], [0, 0], color=FAINT, lw=0.35, ls=(0, (8, 3, 1, 3))); ax.plot([0, 0], [-152, 250], color=FAINT, lw=0.35, ls=(0, (8, 3, 1, 3)))
    ax.add_patch(plt.Circle((0, 0), 45, fill=False, lw=0.45, ec=C["stub"], ls=(0, (2, 1.5))))
    ax.text(31, -42, "stub flange Ø90 (front face)", fontsize=5.2, color=MUTED)
    for k, nm in (("funnel_block_R", "funnel block R"), ("funnel_block_L", "funnel block L"), ("bay_block_p15", "bay block +15°")):
        q = P[k][:, [0, 2]]
        poly(ax, q, WOOD, fill=WOODL, lw=0.8)
        c = q.mean(0)
        ang = math.degrees(math.atan2(*(lambda e: (e[1], e[0]))(max([q[1] - q[0], q[3] - q[0]], key=np.linalg.norm))))
        if ang > 90: ang -= 180
        if ang < -90: ang += 180
        ax.text(c[0], c[1] + 9 * (1 if k != "bay_block_p15" else -1), nm, rotation=ang, ha="center", va="center", fontsize=5.6, color=WOOD, fontweight="semibold")
    # 45 deg face lines of the funnel blocks: the face that meets the funnel panel (the seat side)
    for k in ("funnel_block_R", "funnel_block_L"):
        q = P[k][:, [0, 2]]
        ax.plot(q[:2, 0], q[:2, 1], color=C["seat"], lw=1.4, solid_capstyle="butt")
    q = P["bay_block_p15"][:, [0, 2]]; ax.plot(q[:2, 0], q[:2, 1], color=C["lug"], lw=1.4, solid_capstyle="butt")
    ax.text(-240, 238, "BACK FACE of the upright", fontsize=9, fontweight="bold", color=INK)
    ax.text(-240, 230, "the box mounts on the other face  ·  X to the right, Z up", fontsize=6, color=MUTED)
    ax.annotate("", xy=(240, 238), xytext=(214, 238), arrowprops=dict(arrowstyle="-|>", lw=0.7, color=INK))
    ax.text(212, 238, "+X  bays side", fontsize=6.2, color=INK, ha="right", va="center")
    ax.text(0, 243, "TOP", fontsize=7, fontweight="bold", color=INK, ha="center")
    for lab, fam, x, z, d, note in hs:
        cross(ax, x, z, d, C[fam]); label(ax, x, z, d, lab, C[fam], f"X {x:+.1f}  Z {z:+.1f}")
    # piece-face legend lines
    ax.plot([-240, -228], [-130, -130], color=C["seat"], lw=1.4); ax.text(-226, -130, "face on the funnel panel (collar seats)", va="center", fontsize=5.4, color=MUTED)
    ax.plot([-240, -228], [-138, -138], color=C["lug"], lw=1.4); ax.text(-226, -138, "face on the +15° plane (bay lugs)", va="center", fontsize=5.4, color=MUTED)


# ------------------------------------------------------------------ sheet B: base, top view (X right, Y up)
def draw_base(ax, hs):
    ax.add_patch(plt.Rectangle((-250, -18), 500, 300, fill=False, lw=0.9, ec=INK))
    ax.add_patch(plt.Rectangle((-250, -18), 500, 18, color="#f2f2f5", lw=0))
    ax.plot([-250, 250], [0, 0], color=MUTED, lw=0.5)
    ax.text(0, -9, "upright sits here (Y -18..0, its bottom edge)", ha="center", va="center", fontsize=6, color=MUTED)
    ax.plot([0, 0], [-18, 282], color=FAINT, lw=0.35, ls=(0, (8, 3, 1, 3)))
    ax.text(2, 270, "axle line (X 0)", fontsize=5.4, color=MUTED)
    w = P["bay_wedge_m45"]; x0, x1 = w[0, 0], w[1, 0]
    poly(ax, np.array([[x0, 135], [x1, 135], [x1, 220], [x0, 220]]), WOOD, fill=WOODL)
    ax.text(0.5 * (x0 + x1), 228, "bay wedge -45°", ha="center", fontsize=5.8, color=WOOD, fontweight="semibold")
    ax.text(0.5 * (x0 + x1), 223.5, "45° face toward the axle (left)", ha="center", fontsize=4.8, color=MUTED)
    ax.plot([x0, x0], [135, 220], color=C["lug"], lw=1.4)
    ax.text(-240, 268, "BASE, seen from above", fontsize=9, fontweight="bold", color=INK)
    ax.text(-240, 260, "top face Z -152  ·  X to the right, +Y = toward the box (up the page)", fontsize=6, color=MUTED)
    ax.text(-240, 252, "drill through; drive the screws up from below", fontsize=6, color=MUTED)
    ax.annotate("", xy=(240, 268), xytext=(214, 268), arrowprops=dict(arrowstyle="-|>", lw=0.7, color=INK))
    ax.text(212, 268, "+X  bays side", fontsize=6.2, color=INK, ha="right", va="center")
    for lab, fam, x, y, d, note in hs:
        cross(ax, x, y, d, C[fam]); label(ax, x, y, d, lab, C[fam], f"X {x:+.1f}  Y {y:+.1f}")


def sheet(pdf, page0, draw, hs, ext, title, sub, top=None):
    tl, nx, ny, gx, gy = tiles(*ext, top=top)
    for i, (r, c, x0, x1, y0, y1) in enumerate(tl):
        fig = plt.figure(figsize=(LET_W / 25.4, LET_H / 25.4))
        ax = fig.add_axes([MARG / LET_W, (MARG + FOOT) / LET_H, TILE_W / LET_W, TILE_H / LET_H])
        ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.set_aspect("equal"); ax.axis("off")
        if c > 0: ax.add_patch(plt.Rectangle((x0, y0), OVL, y1 - y0, color="#fff1e6", lw=0, zorder=-9))
        if r > 0: ax.add_patch(plt.Rectangle((x0, y1 - OVL), x1 - x0, OVL, color="#fff1e6", lw=0, zorder=-9))
        draw(ax, hs)
        for t in ax.texts: t.set_clip_on(True)          # labels by a tile edge stay on their tile
        reg(ax, [g for g in gx if x0 - 1 < g < x1 + 1], [g for g in gy if y0 - 1 < g < y1 + 1])
        ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, lw=0.5, ec=MUTED))
        footer(fig, page0 + i, f"{title}  ·  row {r + 1}/{ny}, column {c + 1}/{nx}", sub)
        pdf.savefig(fig); plt.close(fig)
    return tl, nx, ny


# ------------------------------------------------------------------ pattern pages
def face_page(pdf, page, faces):
    fig = plt.figure(figsize=(LET_W / 25.4, LET_H / 25.4))
    ax = fig.add_axes([MARG / LET_W, (MARG + FOOT) / LET_H, TILE_W / LET_W, TILE_H / LET_H])
    ax.set_xlim(0, TILE_W); ax.set_ylim(0, TILE_H); ax.set_aspect("equal"); ax.axis("off")
    x = 6.0
    for F in faces:
        w, l = F["w"], F["l"]; y0 = 4.0
        ax.add_patch(plt.Rectangle((x, y0), w, l, fill=True, fc=WOODL, ec=WOOD, lw=0.9))
        ax.text(x, y0 + l + 3.5, F["name"], fontsize=7.2, fontweight="bold", color=INK)
        ax.text(x + w / 2, y0 - 2.2, "upright end" if "upright" in F["along"] else "end nearer the upright", ha="center", va="top", fontsize=5, color=MUTED)
        ax.text(x - 1.5, y0 + l / 2, F["across"].replace("from the ", "") + "  ->", rotation=90, ha="right", va="center", fontsize=4.8, color=MUTED)
        for lab, a, b in F["holes"]:
            c = C[F["kind"]]
            cross(ax, x + a, y0 + b, 3.4 if F["kind"] == "seat" else 3.0, c)
            ax.text(x + a + 4, y0 + b + 1.5, lab, fontsize=6.5, color=c, fontweight="semibold")
            ax.text(x + a + 4, y0 + b - 2.6, f"{a:.0f} across, {b:.0f} along", fontsize=4.6, color=MUTED)
            ax.plot([x, x + a], [y0 + b, y0 + b], color=c, lw=0.25, ls=(0, (1, 1)))
        x += w + 16
    fig.text(MARG / LET_W, (LET_H - MARG + 2) / LET_H, "Face patterns, 1:1: cut out, lay on the face named, punch the crosses", fontsize=8, fontweight="semibold", color=INK)
    y = 0.0
    for F in faces:
        fig.text(MARG / LET_W, (MARG + FOOT - 0.5 - y) / LET_H, "", fontsize=1)
    footer(fig, page, "Face patterns  ·  " + " / ".join(f["name"].split(" (")[0] for f in faces), "  ".join(f["note"] for f in faces)[:170])
    pdf.savefig(fig); plt.close(fig)

def wedge_page(pdf, page):
    fig = plt.figure(figsize=(LET_W / 25.4, LET_H / 25.4))
    ax = fig.add_axes([MARG / LET_W, (MARG + FOOT) / LET_H, TILE_W / LET_W, TILE_H / LET_H])
    w = P["bay_wedge_m45"][:, [0, 2]]; o = np.array([w[:, 0].min() - 105, w[:, 1].min() - 40])
    ax.set_xlim(o[0], o[0] + TILE_W); ax.set_ylim(o[1], o[1] + TILE_H); ax.set_aspect("equal"); ax.axis("off")
    poly(ax, w, WOOD, fill=WOODL, lw=1.0)
    ax.plot(w[:2, 0], w[:2, 1], color=C["lug"], lw=1.6, solid_capstyle="butt")
    ext = np.array([[w[0, 0] - 25 * c45, w[0, 1] + 25 * c45], [w[1, 0] + 25 * c45, w[1, 1] - 25 * c45]])
    ax.plot(ext[:, 0], ext[:, 1], color=ACCENT, lw=0.5, ls=(0, (4, 2)))
    ax.text(ext[0, 0] - 1, ext[0, 1] + 2, "45° rip line", color=ACCENT, fontsize=6.5, rotation=-45, ha="left", va="bottom")
    for p_, t in ((w[0], "r 158"), (w[1], "r 200")):
        ax.text(p_[0] - 2, p_[1] + 2, t, fontsize=5.2, color=MUTED, ha="right")
    ax.text(w[2, 0] + 3, w[2, 1] - 6, "sits on the base (Z -152)", fontsize=5.5, color=MUTED)
    ax.text(0.5 * (w[2, 0] + w[3, 0]), w[3, 1] - 3, f"{w[1, 0] - w[0, 0]:.1f}", fontsize=5.5, color=MUTED, ha="center", va="top")
    ax.text(w[0, 0] - 2, 0.5 * (w[0, 1] + w[3, 1]), f"{w[0, 1] - w[3, 1]:.1f}", fontsize=5.5, color=MUTED, ha="right", va="center")
    ax.text(w[1, 0] + 2, 0.5 * (w[1, 1] + w[2, 1]), f"{w[1, 1] - w[2, 1]:.1f}", fontsize=5.5, color=MUTED, ha="left", va="center")
    ax.text(o[0] + 6, o[1] + TILE_H - 14, "Bay wedge -45°: cross-section, 1:1 (seen from behind, as sheet A)", fontsize=9, fontweight="bold", color=INK)
    ax.text(o[0] + 6, o[1] + TILE_H - 22, "Rip an 85-long offcut (~45 x 45) to this outline. The orange face goes on the -45° plane and takes the lugs;", fontsize=6.4, color=MUTED)
    ax.text(o[0] + 6, o[1] + TILE_H - 28, "the bottom stands square on the base. Corners (W, mm): " +
            "  ".join(f"({p_[0]:.1f}, {p_[1]:.1f})" for p_ in w), fontsize=6.0, color=MUTED)
    F = FACES[2]; fx, fy = o[0] + 8, o[1] + 22
    ax.add_patch(plt.Rectangle((fx, fy), F["w"], F["l"], fc=WOODL, ec=WOOD, lw=0.9))
    ax.text(fx, fy + F["l"] + 4, "Bay wedge: 45° face pattern, 1:1", fontsize=7.2, fontweight="bold", color=INK)
    ax.text(fx + F["w"] / 2, fy - 2.2, "end nearer the upright (Y 135)", ha="center", va="top", fontsize=5, color=MUTED)
    ax.text(fx - 1.5, fy + F["l"] / 2, "axle-side edge (r 158)  ->", rotation=90, ha="right", va="center", fontsize=4.8, color=MUTED)
    for lab, a, b in F["holes"]:
        cross(ax, fx + a, fy + b, 3.0, C["lug"])
        ax.text(fx + a + 4, fy + b + 1.5, lab, fontsize=6.5, color=C["lug"], fontweight="semibold")
        ax.text(fx + a + 4, fy + b - 2.6, f"{a:.0f} across, {b:.0f} along", fontsize=4.6, color=MUTED)
    ax.text(fx, fy - 9, F["note"], fontsize=5.4, color=MUTED)
    footer(fig, page, "Bay wedge rip pattern + face pattern", "trace or glue the outline on the offcut's end, rip on the dashed 45° line, check against the base square")
    pdf.savefig(fig); plt.close(fig)


def cover(pdf, hsA, hsB, tlA, nA, tlB, nB, n_pat):
    fig = plt.figure(figsize=(LET_W / 25.4, LET_H / 25.4))
    ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(0, LET_W); ax.set_ylim(0, LET_H); ax.axis("off")
    y = LET_H - 22
    ax.text(MARG, y, "Nox box  ·  stand v4 drill template", fontsize=15, fontweight="bold", color=INK)
    ax.text(MARG, y - 7, "6 wood pieces, every hole from the axle  ·  v1  ·  17 Oct 2026", fontsize=7.5, color=MUTED)
    y -= 18
    pa, pb = 2, 2 + len(tlA)
    steps = [f"Print at 100 % (actual size). Every page has a 100 mm bar: measure it.",
             f"Sheet A, pages {pa}..{pb - 1}: the upright, seen from its BACK face. Sheet B, pages {pb}..{pb + len(tlB) - 1}: the base from above. "
             f"Then {n_pat} pattern pages.",
             "Trim each tile on its edge, overlap 10 mm (light band), match the orange crosses, tape.",
             "Upright: tape sheet A on the back face, bottom line on the bottom edge, centred left-right. Punch every cross.",
             "Drill square to the face: AX Ø26, everything else Ø5.5 unless a pattern says otherwise. Countersink the block screws.",
             "Blocks: set each 2x4 end inside its outline on the other face (prick the corners through), screw from the back, glue.",
             "Base: tape sheet B on the top face, the upright line on the back edge. Screws go up from below."]
    for i, s in enumerate(steps):
        ax.text(MARG, y, f"{i + 1}", fontsize=8, fontweight="bold", color=ACCENT)
        ax.text(MARG + 6, y, s, fontsize=6.9, color=INK); y -= 8.2
    y -= 4
    ax.text(MARG, y, "Holes  (W, mm from the axle)", fontsize=7.5, fontweight="semibold", color=INK); y -= 6
    cols = (MARG, MARG + 13, MARG + 31, MARG + 49, MARG + 63)
    for cx, h in zip(cols, ("hole", "X", "Z / Y", "Ø drill", "what")):
        ax.text(cx, y, h, fontsize=6.0, color=MUTED, fontweight="semibold")
    y -= 1.8; ax.plot([MARG, LET_W - MARG], [y, y], color=FAINT, lw=0.5); y -= 4.0
    for lab, fam, x, z, d, note in hsA + hsB:
        for k, (cx, v) in enumerate(zip(cols, (lab, f"{x:+.1f}", f"{z:+.1f}", f"{d:.1f}", note))):
            ax.text(cx, y, v, fontsize=6.0, color=C[fam] if k == 0 else INK, fontweight="semibold" if k == 0 else "normal")
        y -= 3.9
    y -= 2
    for F in FACES:
        hs = ", ".join(f"{lab} {a:.0f}/{b:.0f}" for lab, a, b in F["holes"])
        ax.text(MARG, y, f"{F['name']}:", fontsize=6.0, color=C[F["kind"]], fontweight="semibold")
        ax.text(MARG + 45, y, f"{hs}  (across / along, mm)", fontsize=6.0, color=INK); y -= 3.9
    y -= 3
    ax.text(MARG, y, "Pieces: base 500 x 300 · upright 500 x 402 · 2 funnel blocks 2x4 x 230 · bay block 2x4 x 230 · bay wedge 85 long", fontsize=6.0, color=MUTED)
    ax.text(MARG, y - 4, "Hardware: 4 x M5 flange · 9 + 6 screws 5 x 40 · 4 x 5 x 50 lug screws (or M5 x 60 + nuts) · 4 x M3 x 50 collar", fontsize=6.0, color=MUTED)
    footer(fig, 1, "Cover", "Nox box stand v4  ·  geometry: Nox Source nox/box/stand/stand_v4.py  ·  doc: nox/box-stand.md")
    pdf.savefig(fig); plt.close(fig)


def main(path="box_stand_v4_drill_template_v1.pdf"):
    hsA, hsB = upright_holes(), base_holes()
    from io import BytesIO
    with PdfPages(path) as pdf:
        # cover needs the page counts: lay out the tiles first
        TOP_A = 47.0 + TILE_H        # sheet A's row seam at Z 47..57: between the flange (r 45) and the bay block screws (Z 63)
        tA = tiles(-268, 268, -176, 254, top=TOP_A); tB = tiles(-262, 262, -26, 278)
        cover(pdf, hsA, hsB, tA[0], tA[1:3], tB[0], tB[1:3], 2)
        p = 2
        sheet(pdf, p, draw_upright, hsA, (-268, 268, -176, 254), "Sheet A  ·  upright, back face", "Nox box stand v4  ·  1:1  ·  X right, Z up, from the axle", top=TOP_A); p += len(tA[0])
        sheet(pdf, p, draw_base, hsB, (-262, 262, -26, 278), "Sheet B  ·  base, from above", "Nox box stand v4  ·  1:1  ·  X right, +Y up the page"); p += len(tB[0])
        face_page(pdf, p, FACES[:2]); p += 1
        wedge_page(pdf, p); p += 1
    print(path, "pages", p - 1, "holes", len(hsA) + len(hsB))
    return path


if __name__ == "__main__":
    main()
