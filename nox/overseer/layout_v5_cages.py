"""Overseer cage alternatives, side by side (27 Oct review: the concept's alternatives with a picture): cage v3 (main, 48 nodes),
cage v4 (fewest cut lengths, dropped by Spark) and cage v5 (half the tubes, close nodes merged, the frame model's splits; current).
Plan and side view of each, tube count, cut lengths, total length and shell mass (6/4 tube 24.3 g/m + 4.5 g a node, E).

python3 nox/overseer/layout_v5_cages.py -> runs/overseer_layout/v5/cages_alt.png
"""
import os, json
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
CAGES = [("v3 (main, superseded)", "cage_v3_shell.json"), ("v4 (fewest lengths, dropped)", "cage_v4_shell.json"), ("v5 (current)", "cage_v5_shell.json")]


def main():
    fig, axs = plt.subplots(2, 3, figsize=(16, 8.5))
    for col, (name, fn) in enumerate(CAGES):
        d = json.load(open(os.path.join(HERE, fn))); N = np.array(d["nodes"]); R = d["rods"]
        L = np.array([np.linalg.norm(N[a] - N[b]) for a, b in R]); ring = d.get("open", [{}])[0].get("ring", [])
        cuts = len({round(x) for x in L}) if "cut_lengths" not in d else len(d["cut_lengths"])
        g = L.sum() * 24.3e-3 + 4.5 * len(N); ext = N.max(0) - N.min(0)
        for row, (i, j, lab) in enumerate(((0, 1, "plan (x, y)"), (0, 2, "side (x, z)"))):
            ax = axs[row, col]
            for (a, b) in R: ax.plot([N[a, i], N[b, i]], [N[a, j], N[b, j]], color="#4a6fa5", lw=1.0)
            if ring:
                rr = N[ring + ring[:1]]; ax.plot(rr[:, i], rr[:, j], color="#d1495b", lw=1.8)
            ax.plot(N[:, i], N[:, j], "o", color="#222", ms=3)
            ax.set_aspect("equal"); ax.set_xlim(-380, 420); ax.set_ylim(-360, 360) if j == 1 else ax.set_ylim(-120, 240)
            ax.set_xlabel(lab, fontsize=8); ax.tick_params(labelsize=7)
        axs[0, col].set_title(f"cage {name}\n{len(N)} nodes, {len(R)} tubes, {cuts} cut lengths, {L.min():.0f}..{L.max():.0f}, {L.sum() / 1000:.1f} m\n"
                              f"{ext[0]:.0f} x {ext[1]:.0f} x {ext[2]:.0f}, shell ~{g:.0f} g (E)", fontsize=9)
    axs[1, 0].plot([], [], color="#d1495b", lw=1.8, label="base loop (swap opening)"); axs[1, 0].legend(fontsize=7, loc="lower left")
    fig.suptitle("Overseer cage alternatives (C geometry, E mass)"); fig.tight_layout()
    out = os.path.join(ROOT, "runs", "overseer_layout", "v5", "cages_alt.png"); fig.savefig(out, dpi=100); print(out)


if __name__ == "__main__":
    main()
