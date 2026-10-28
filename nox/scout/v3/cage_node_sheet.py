"""Scout cage nodes v1: the visual (one page): the cage with every saved node placed (3D, real meshes), then per design its print
picture, a section through its tightest socket pair, its sockets and loads. Reads runs/scout_nodes/v1 (result.json, placed.json,
the print files) and writes runs/scout_nodes/v1/nodes.html + section_<i>.png.

python3 nox/scout/v3/cage_node_sheet.py [run]
"""
import os, sys, json, base64, io, math
import numpy as np, trimesh
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
RUN = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "runs", "scout_nodes", "v1")
LAY = os.path.join(ROOT, "runs", "scout_layout", "v12")


def b64(a, dt):
    return base64.b64encode(np.ascontiguousarray(a, dt).tobytes()).decode()


def section_png(i, rec, out):
    """a cut through the node centre in the plane of its two closest socket axes (node frame, mm)"""
    loc = trimesh.load(os.path.join(ROOT, rec["placed_mesh"]), force="mesh"); loc.apply_translation(-np.array(rec["centre"]))
    S = rec["sockets"]; best = None
    for p in range(len(S)):
        for q in range(p + 1, len(S)):
            c = float(np.dot(S[p]["dir"], S[q]["dir"]))
            if best is None or c > best[0]: best = (c, p, q)
    _, p, q = best; a, b = np.array(S[p]["dir"]), np.array(S[q]["dir"])
    # the plane holds bore p's axis and bore q's mid point (eccentric lines need not be coplanar): both bores show
    Pp, Pq = np.array(S[p]["line_origin"]), np.array(S[q]["line_origin"])
    mp, mq = Pp + a * (S[p]["setback"] + 5.0), Pq + b * (S[q]["setback"] + 5.0)
    n = np.cross(a, mq - Pp); n = np.cross(a, b) if np.linalg.norm(n) < 1e-6 else n; n /= np.linalg.norm(n)
    o = Pp
    sec = loc.section(plane_origin=o, plane_normal=n)
    fig, ax = plt.subplots(figsize=(4.2, 4.2), dpi=110)
    if sec is not None:
        e1 = a.copy(); e2 = np.cross(n, e1)
        T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = e1, e2, n, o
        p2, _ = sec.to_2D(to_2D=np.linalg.inv(T))
        for poly in p2.polygons_full:
            x, y = poly.exterior.xy; ax.fill(x, y, color="#c9a227", alpha=0.85, lw=0)
            for h in poly.interiors:
                x, y = h.xy; ax.fill(x, y, color="white", lw=0)
            x, y = poly.exterior.xy; ax.plot(x, y, color="#333", lw=0.8)
        for s_ in (S[p], S[q]):
            d = np.array(s_["dir"]); P = np.array(s_["line_origin"])
            f0 = (P + d * s_["setback"]) - o; m0 = (P + d * s_["mouth_r"]) - o
            ax.plot([f0 @ e1, m0 @ e1 + 8 * (d @ e1)], [f0 @ e2, m0 @ e2 + 8 * (d @ e2)], color="#1f5fa8", lw=1.0, ls="--")
            ax.annotate(f"{s_['name']}\nwall {s_['wall']}, floor at {s_['setback']}", (m0 @ e1, m0 @ e2), fontsize=7, color="#1f5fa8",
                        xytext=(4, 4), textcoords="offset points")
    ax.plot([0], [0], "k+", ms=8)
    ax.set_aspect("equal"); ax.grid(alpha=0.25); ax.set_xlabel("mm"); ax.tick_params(labelsize=7)
    ax.set_title(f"node {i}: section through {S[p]['name']} / {S[q]['name']}", fontsize=8)
    fig.tight_layout(); fig.savefig(out); plt.close(fig)


def main():
    R = json.load(open(os.path.join(RUN, "result.json"))); PL = json.load(open(os.path.join(RUN, "placed.json")))
    CG = json.load(open(os.path.join(LAY, "cage_shell.json")))
    meshes = []
    for p in PL:
        t = trimesh.load(os.path.join(ROOT, p["mesh"]), force="mesh")
        if len(t.faces) > 6000: t = t.simplify_quadric_decimation(face_count=6000)
        meshes.append(dict(name=p["name"], node=p["node"], design=p.get("design"), tpu=p.get("material") == "TPU",
                           v=b64(t.vertices, "<f4"), f=b64(t.faces, "<u4")))
    rods = [dict(name=r["rod"], a=r["floor_a"], b=r["floor_b"], ecc=r.get("ecc_mm", 0.0)) for r in R["rods"]]
    cards = []
    for k, rec in R["nodes"].items():
        i = int(k)
        rec["placed_mesh"] = next(p["mesh"] for p in PL if p["node"] == i and p["name"].startswith("cage node"))
        sp = os.path.join(RUN, f"section_{i}.png"); section_png(i, rec, sp)
        pv = os.path.join(RUN, f"preview_scout_node_{i}_v1.png")
        img = lambda f: ("data:image/png;base64," + base64.b64encode(open(f, "rb").read()).decode()) if os.path.exists(f) else ""
        fe = rec["fe"][-1] if rec["fe"] else None
        cards.append(dict(i=i, mirror=rec["mirror"], g=rec["mass_g"], env=rec["envelope"], solid=rec["solid_check"]["ok"], prev=img(pv), sec=img(sp),
                          ext=rec["extents_print"], bed=rec["bed_below_centre"], hub=rec["hub_r"],
                          fe=None if fe is None else dict(worst=fe["max_cell"]["value"], wcase=fe["max_cell"]["case"], wname=fe["max_cell"]["socket"], p999=fe["worst"]["p999"], case=fe["worst"]["case"], name=fe["worst"]["name"], iters=len(rec["fe"])),
                          socks=[dict(n=s["name"], r=s["setback"], w=s["wall"], e=s["ecc"], el=s["elev_deg"], push=s["loads"]["push"], pull=s["loads"]["pull"],
                                      side=s["loads"]["side"]) for s in rec["sockets"]]))
    cards.sort(key=lambda c: c["i"])
    data = dict(meshes=meshes, rods=rods, cards=cards, total=R["total_nodes_g"], cap=R["cap"], ecc=R.get("eccentric", {}), option=R["option"])
    tpl = open(os.path.join(HERE, "cage_node_sheet.html")).read()
    html = tpl.replace("/*DATA*/null", json.dumps(data))
    open(os.path.join(RUN, "nodes.html"), "w").write(html)
    print(os.path.join(RUN, "nodes.html"), f"{len(html) / 1e6:.2f} MB")


if __name__ == "__main__":
    main()
