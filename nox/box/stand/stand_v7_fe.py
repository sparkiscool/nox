"""Stand v7 structural check (19 Oct 2026): the changed / new parts as voxel FE (stand_v6_fe machinery, printopt PETG,
misuse class). Cases (E): bar_half 50 N on the tip (box on one tip); toe / kb 13 N push on the sheet plate (fore-aft) +
v6 cases; end frames v6 cases + a Scout drop 64 N on one pad; brackets fixed at the sheet flange, loaded at the ring
side (bracket_e: 2 x 64 N drop + 13 N side; bracket_j: 30 N each way at the lug pair); post segment fixed at its bottom,
13 N at the top (standing alone, worst case). python3 stand_v7_fe.py [part ...] -> stand_v7_fe.json / .png"""
import sys, os, json, math, numpy as np
import stand_v6_fe as F6
from stand_v6_fe import put, FLOOR
import stand_v6_parts as S, stand_v7_parts as V
F6.VER = os.environ.get("VTAG", "v7")

def bar_load(c, X, used, F):
    tip = (X[:, 0] > 172) & (X[:, 2] < FLOOR + 0.6) & used
    put(F, tip, [0, 0, 50.0]); return tip
def toe_fix(X): return X[:, 1] < 201.5
def toe_load(c, X, used, F):
    pl = (X[:, 1] > V.Y_FRONT - 0.6) & used
    if c == "front sheet push 13 N": put(F, pl, [0, -13, 0])
    elif c == "front sheet lift 13 N (corner lifted)": put(F, pl, [0, 0, 13])
    elif c == "misuse: 50 N on the plate": put(F, pl, [0, -50, 0])
    return pl
def kb_load(c, X, used, F):
    if c in ("arm lean (moment at the spigot top)", "yaw 20 N at the bay"): return F6.kb_load(c, X, used, F)
    pl = (X[:, 1] < V.Y_BACK + 0.6) & used
    put(F, pl, [0, 13, 0]); return pl
def end_load(c, X, used, F, side):
    if c != "Scout drop 64 N on one pad": return F6.end_load(c, X, used, F, side)
    pads = (np.hypot(X[:, 0], X[:, 2]) > 183) & used; a = pads & (X[:, 1] < 177)
    put(F, a, [0, 0, -64]); return a
def br_fix(X): return X[:, 0] > V.X_IN - 0.6
def bre_load(c, X, used, F):
    r = np.hypot(X[:, 0], X[:, 2]); ang = np.degrees(np.arctan2(X[:, 2], X[:, 0]))
    ch = (r < 179.5) & (np.abs(ang - 45) < 6) & used          # the cheeks on the station blocks
    if c == "drop 2 x 64 N + 13 N side": put(F, ch, [13, 0, -128])
    elif c == "rim pull 20 N outward": put(F, ch, [0, 0, 20]); put(F, ch, [-20, 0, 0])
    return ch
def brj_load(a):
    def f(c, X, used, F):
        r = np.hypot(X[:, 0], X[:, 2]); ang = np.degrees(np.arctan2(X[:, 2], X[:, 0]))
        ch = (r < 179.5) & (np.abs(ang - a) < 6) & used
        if c == "30 N down + 30 N side": put(F, ch, [30, 0, -30])
        elif c == "30 N up + 30 N fore-aft": put(F, ch, [0, 30, 30])
        return ch
    return f
def post_fix(X): return X[:, 2] < 0.6
def post_load(c, X, used, F):
    top = (X[:, 2] > V.SEG - 0.6) & used
    put(F, top, [-13, 0, 0]); return top

PARTS = {
    "bar_half": (1.5, 2, F6.bar_fix, bar_load, ["box weight on one tip (misuse)"]),
    "toe": (1.5, 2, toe_fix, toe_load, ["front sheet push 13 N", "front sheet lift 13 N (corner lifted)", "misuse: 50 N on the plate"]),
    "kb": (2.0, 2, F6.kb_fix, kb_load, ["arm lean (moment at the spigot top)", "yaw 20 N at the bay", "rear sheet push 13 N"]),
    "frame_endR": (1.5, 0, lambda X: F6.end_fix(X, +1), lambda c, X, u, F: end_load(c, X, u, F, +1), ["funnel + Scout 6 N + rim push 6.5 N", "misuse: 50 N on one pad", "Scout drop 64 N on one pad"]),
    "frame_endL": (1.5, 0, lambda X: F6.end_fix(X, -1), lambda c, X, u, F: end_load(c, X, u, F, -1), ["misuse: 50 N on one pad", "Scout drop 64 N on one pad"]),
    "bracket_e": (1.0, 0, br_fix, bre_load, ["drop 2 x 64 N + 13 N side", "rim pull 20 N outward"]),
    "bracket_jA": (1.0, 0, br_fix, brj_load(-15), ["30 N down + 30 N side", "30 N up + 30 N fore-aft"]),
    "bracket_jB": (1.0, 0, br_fix, brj_load(15), ["30 N down + 30 N side", "30 N up + 30 N fore-aft"]),
    "post_seg": (1.0, 0, post_fix, post_load, ["13 N at the top, standing alone"]),
}
if __name__ == "__main__":
    names = sys.argv[1:] or list(PARTS)
    out = {}; figs = {}
    for n in names:
        h, ax, fx, ld, cs = PARTS[n]; print(n)
        r, fd, org, hh = F6.run(n, h, ax, fx, ld, cs); out[n] = r; figs[n] = (fd, org, hh)
    json.dump(out, open(f"stand_{F6.VER}_fe.json", "w"), indent=1)
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, axs = plt.subplots(2, 5, figsize=(20, 9), dpi=120)
    for axp, n in zip(axs.ravel(), names):
        fd, org, hh = figs[n]; cells = fd[0][1]; idx = np.max(np.stack([f[2] for f in fd]), 0)
        pts = cells * hh + org + hh / 2; ext = pts.max(0) - pts.min(0); drop = int(np.argmin(ext)); keep = [i for i in range(3) if i != drop]
        o = np.argsort(idx); sc = axp.scatter(pts[o, keep[0]], pts[o, keep[1]], c=idx[o], s=2, cmap="inferno", vmin=0, vmax=max(1.0, idx.max()))
        axp.set_aspect("equal"); axp.set_title(f"{n}: worst {out[n]['worst']:.2f}", fontsize=10); axp.set_xticks([]); axp.set_yticks([]); plt.colorbar(sc, ax=axp, fraction=0.04)
    for axp in axs.ravel()[len(names):]: axp.axis("off")
    fig.suptitle("Stand v7 failure index (printopt PETG, misuse; max over cases; 1.0 = allowable; load spots masked)", fontsize=11)
    fig.tight_layout(); fig.savefig(f"stand_{F6.VER}_fe.png"); print("stand_v7_fe.png")
