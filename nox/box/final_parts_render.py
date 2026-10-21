"""Final box parts: three views of every READY print file (print pose, as it sits on the bed) and one assembled arm view
(hub v5 set on the stub, bars, motors, detent; W frame). Renders with hub_render.render (embree ray tracing).
python3 final_parts_render.py <stl_dir> -> out/final_<name>.png, out/final_arm_assembly.png, out/final_parts.json"""
import os, sys, math, json
import numpy as np, trimesh, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import hub_render as HR
import arm_cad as A, arm_layout as L, detent as DT
from arm_cad import tm, cylY, rsquare

SRC = sys.argv[1] if len(sys.argv) > 1 else "/home/claude/printfiles/stl"
OUT = "out"; os.makedirs(OUT, exist_ok=True)
BG = HR.BG; D = 1.27
PARTS = [  # file, label, version, qty, state, colour
    ("box_hub_v5", "Hub v5", "v5 (regenerated 17 Oct 22:43)", 1, "READY", HR.C["hub"]),
    ("box_sun_post_v1", "Sun post", "v1", 1, "READY", HR.C["post"]),
    ("box_detent_ring_v1", "Detent ring", "v1 (12 notches)", 1, "READY", HR.C["ring"]),
    ("box_detent_arm_v1", "Detent roller arm + spacer", "v1", 1, "READY", HR.C["arm"]),
    ("box_idler_end_A_v3", "Idler end A", "v3", 1, "READY", HR.C["bar"]),
    ("box_idler_end_B_v3", "Idler end B", "v3", 1, "READY", HR.C["bar"]),
    ("box_pinion_v4", "Gripper pinion (12T m1)", "v4", 2, "READY", HR.C["p20"]),
    ("box_finger_A_for_head_A_v4", "Finger A, head A", "v4", 1, "READY", HR.C["arm"]),
    ("box_finger_B_for_head_A_v4", "Finger B, head A", "v4", 1, "READY", HR.C["arm"]),
    ("box_finger_A_for_head_B_v4", "Finger A, head B", "v4", 1, "READY", HR.C["arm"]),
    ("box_finger_B_for_head_B_v4", "Finger B, head B", "v4", 1, "READY", HR.C["arm"]),
    ("box_baymod_v4i", "Bay module (universal)", "v4i", 2, "READY", HR.C["p80"]),
    ("belt_clamp_coupon_set_v2", "Belt clamp coupon set", "v2 (slots 1.8 / 2.0 / 2.2)", 1, "READY", HR.C["brg"]),
]
VIEWS = [("iso", 35, 28), ("side", 0, 2), ("top", 0, 89)]


def load(name):
    m = trimesh.load(f"{SRC}/{name}.stl", force="mesh"); m.merge_vertices(); return m


def sheet(name, label, ver, qty, state, col, m):
    g = m.volume / 1000 * D
    fig, ax = plt.subplots(1, 3, figsize=(15, 5.4), dpi=130); fig.patch.set_facecolor(BG)
    for a, (vn, az, el) in zip(ax, VIEWS):
        # render puts Y up: map print Z (up) -> render Y
        t = m.copy(); t.vertices = t.vertices[:, [0, 2, 1]] * [1, 1, -1]
        a.imshow(HR.render([(t, col)], az, el, 820, 640)); a.set_title(vn, fontsize=11, loc="left", color="#333"); a.axis("off")
    e = m.extents
    fig.suptitle(f"{label}  {ver}  ·  {g:.1f} g (M)  ·  x{qty}  ·  {state}  ·  {e[0]:.0f} x {e[1]:.0f} x {e[2]:.0f} print pose, bed face down",
                 x=0.02, ha="left", fontsize=12.5, color="#222")
    plt.tight_layout(rect=(0, 0, 1, 0.94)); plt.savefig(f"{OUT}/final_{name}.png", facecolor=BG); plt.close()
    return dict(name=name, label=label, version=ver, qty=qty, state=state, g=round(g, 1), extents=[round(x, 1) for x in e],
                faces=len(m.faces), watertight=bool(m.is_watertight), png=f"final_{name}.png")


def hub_to_W(P):
    """print pose -> W: undo the +90 about X (post_hub_v5), then centre the Ø80.2 bore on the Y axis and put the sleeve end at Y 7."""
    W = P.copy(); W.apply_transform(trimesh.transformations.rotation_matrix(math.radians(-90), [1, 0, 0]))
    v = W.vertices; c = np.array([v[:, 0].mean(), v[:, 2].mean()])
    for _ in range(6):
        r = np.hypot(v[:, 0] - c[0], v[:, 2] - c[1]); ymid = v[:, 1] > v[:, 1].min() + 10
        sel = ymid & (v[:, 1] < v[:, 1].min() + 32) & (np.abs(r - 40.1) < 0.6)
        if sel.sum() < 50: sel = ymid & (np.abs(r - 40.1) < 1.5)
        pts = v[sel][:, [0, 2]]
        A_ = np.c_[2 * pts, np.ones(len(pts))]; b = (pts ** 2).sum(1)
        sol = np.linalg.lstsq(A_, b, rcond=None)[0]; c = sol[:2]
    W.apply_translation([-c[0], 7.0 - v[:, 1].min(), -c[1]])
    v = W.vertices; r = np.hypot(v[:, 0], v[:, 2]); sel = (v[:, 1] > 10) & (v[:, 1] < 38) & (np.abs(r - 40.1) < 0.6)
    return W, dict(bore_fit_pts=int(sel.sum()), bore_d=round(2 * float(r[sel].mean()), 2) if sel.any() else None)


if __name__ == "__main__":
    info = {}
    for name, label, ver, qty, state, col in PARTS:
        m = load(name)
        if len(m.faces) > 90000:
            import fast_simplification as FS
            vv, ff = FS.simplify(m.vertices.astype(np.float32), m.faces.astype(np.int64), target_count=80000)
            m_r = trimesh.Trimesh(vv, ff, process=True)
        else: m_r = m
        row = sheet(name, label, ver, qty, state, col, m_r); row["g"] = round(m.volume / 1000 * D, 1); info[name] = row
        print(name, row["g"], "g", row["extents"], flush=True)
    # ---- assembled arm (W): hub v5 (print STL re-posed), drawn stub / post / bars / motors / pulleys / detent
    hub, fit = hub_to_W(load("box_hub_v5")); print("hub W fit", fit)
    post = tm(A.sun_post()); stub = tm(A.stub()); S = A.axle_sun_bodies()
    brgs = trimesh.util.concatenate([tm(cylY(0, 0, y0, y0 + 16, 40) - cylY(0, 0, y0 - 1, y0 + 17, 25)) for y0 in L.Y_BRG])
    motors = trimesh.util.concatenate([tm(rsquare(M, L.NEMA / 2, 2.0, L.Y_PLATE0)) for M in L.MOTORS.values()])
    bars = trimesh.util.concatenate([tm(A.bar_2040(*L.BAR_A)), tm(A.bar_2040(*L.BAR_B).rotate([0, -90, 0]))])
    p20 = trimesh.util.concatenate([tm(cylY(M["xc"], M["zc"], 54, 64, 8)) for M in L.MOTORS.values()])
    idlers = trimesh.util.concatenate([tm(A.idler_end("A")), tm(A.idler_end("B").rotate([0, -90, 0]))]) if hasattr(A, "idler_end") else None
    c = DT.CHOSEN; Dc = DT.design(c["th"], c["s"], c["a"], c["b"], c["phi_d"], c["psi"]); Sd = DT.arm_solids(Dc, np.array(c["Q"]))
    ring = tm(DT.ring(c["th"])); darm = tm(Sd["arm"] + Sd["spacer"] + Sd["hook"]); roller = tm(Sd["roller"] + Sd["nut"]); springs = tm(Sd["springs"][0] + Sd["springs"][1])
    full = [(stub, HR.C["stub"]), (brgs, HR.C["brg"]), (post, HR.C["post"]), (hub, HR.C["hub"]), (tm(S["p80"]), HR.C["p80"]), (tm(S["loop"]), HR.C["loop"]),
            (motors, HR.C["motor"]), (p20, HR.C["p20"]), (bars, HR.C["bar"]), (ring, HR.C["ring"]), (darm, HR.C["arm"]), (roller, HR.C["roller"]), (springs, HR.C["spring"])]
    if idlers is not None: full.append((idlers, HR.C["bar"]))
    fig, ax = plt.subplots(1, 2, figsize=(16, 7.6), dpi=130); fig.patch.set_facecolor(BG)
    ax[0].imshow(HR.render(full, 215, 30, 1000, 900)); ax[0].set_title("Arm from the bar side (heads not shown: BUILT, not saved)", fontsize=11, loc="left"); ax[0].axis("off")
    ax[1].imshow(HR.render(full, 35, 25, 1000, 900)); ax[1].set_title("Arm from the stub side", fontsize=11, loc="left"); ax[1].axis("off")
    from matplotlib.patches import Patch
    keys = ["stub", "brg", "post", "hub", "p80", "loop", "motor", "bar", "ring", "arm", "spring"]
    labels = ["stub v2 (fixed; v3 pending)", "6010 x2", "sun post (fixed)", "hub v5 (turns)", "80T sun (fixed)", "122T loop", "NEMA17 x3", "2040 bars + idler ends", "detent ring", "roller arm", "bands"]
    fig.legend([Patch(fc=HR.C[k]) for k in keys], labels, loc="lower center", ncol=6, frameon=False, fontsize=9)
    fig.suptitle("Nox box swap arm: hub v5 set as saved (W frame, axle vertical)", x=0.02, ha="left", fontsize=13, color="#222")
    plt.tight_layout(rect=(0, 0.07, 1, 0.93)); plt.savefig(f"{OUT}/final_arm_assembly.png", facecolor=BG); plt.close()
    info["_assembly"] = dict(png="final_arm_assembly.png", hub_fit=fit)
    json.dump(info, open(f"{OUT}/final_parts.json", "w"), indent=1)
    print("done")
