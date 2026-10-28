"""Scout camera mount v2, the system pass (concept check): where the camera can hang so it stays >= 100 Hz on the v12 cage with
every rod 3.0 solid carbon (Spark 27 Oct). The v12 cage frame model (nox/scout/v3/cage_frame.py: rigid PETG nodes, saddle
ties fixed at the saddle, carbon E 100 GPa, E) with the camera + mount (30 g, E) as one lumped body, clipped to rods (rigid
links) and / or tied to the core by a PETG bar (E 2000 MPa, E) from the plate edge, clamped there.

Weighed (v13, the saved nodes' real masses, frame v4): clip placements on the cage only (v1's mid-span points, next to the nose
nodes, 4 and 6 clips), a core tie + 2 gripping clips next to nodes 14 / 15, and the picked one: the core tie with the 2 clips floating
(Ø5.0 pocket on the 3.0 rod, 1.0 all round): in flight they never touch, so the camera is out of the cage's 38 Hz nose modes. Modes with the camera's share of the kinetic energy,
and the clips' share of a 250 N frontal hit (the topology run's clip-bearing case).

python3 nox/scout/v3/camera_mount_v2_concept.py [out.json]   (default runs/scout_camera_mount/v2/concept.json)
"""
import os, sys, json
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox")); sys.path.insert(0, HERE)
import camera_mount_v2 as cm                                       # sets the layout run (v13) for cage_frame, and the clip position
import cage_frame as cf
from frame_model import Frame

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "runs", "scout_camera_mount", "v2", "concept.json")
N = cf.N
CAM = (138.0, 0.0, 42.0)                      # camera + mount body, C (camera box centre, mount a little behind)
PLATE_EDGE = (52.0, 0.0, 21.6)                # the tie's clamp at the plate's front edge (the foot is clamped on the plate behind it), C


def rect(b, t, E=2000.0):
    I1, I2 = b * t ** 3 / 12, t * b ** 3 / 12
    return dict(E=E, G=E / 2.6, A=b * t, I=min(I1, I2), J=0.4 * b * t * (b * b + t * t) / 12, c=max(b, t) / 2)


def model(clips, ties=(), cam_g=30.0):
    """clips: [(a, b, d)] a clip on rod a-b at d mm from node a; ties: [(point, section)] bars from the camera to a fixed point"""
    f = Frame(); nid = [f.node(p) for p in N]; rods = [tuple(r) for r in cf.CG["rods"]]
    split, cpts = {}, []
    for a, b, d in clips:
        key = (a, b) if (a, b) in rods else (b, a)
        p = N[a] + (N[b] - N[a]) / np.linalg.norm(N[b] - N[a]) * d
        split.setdefault(key, []).append(p); cpts.append(p)
    for a, b in rods:
        pts = [N[a]] + sorted(split.get((a, b), []), key=lambda c: np.linalg.norm(c - N[a])) + [N[b]]
        for p_, q_ in zip(pts[:-1], pts[1:]): f.member(f.node(p_), f.node(q_), cf.ROD3, f"rod {a}-{b}")
    H = f.node(CAM)
    for c in cpts: f.member(H, f.node(c), cf.RIGID, "camera clip")
    for k, (p, q) in cf.RES["links"].items():
        p, q = np.array(p), np.array(q); c_ = f.node(p); f.member(c_, f.node(q), cf.ROD3, k, k=cf.K_TIE); f.fix(c_)
    for p, sec in ties:
        t_ = f.node(p); f.member(H, t_, sec, "tie"); f.fix(t_)
    m = cf.masses(f, nid, H); m[H] = m.get(H, 0) - cf.CAM_G + cam_g
    hz, U = f.modes(m, 6)
    share = [float(m[H] * np.sum(U[6 * H:6 * H + 3, k] ** 2) / sum(m[j] * np.sum(U[6 * j:6 * j + 3, k] ** 2) for j in m)) for k in range(len(hz))]
    hd = [float(np.linalg.norm(U[6 * H:6 * H + 3, k]) / max(1e-12, np.abs(U[:, k]).max())) for k in range(len(hz))]   # cage_frame.py's "the camera moves" (> 0.3)
    return f, H, [round(float(h), 1) for h in hz], [round(s, 2) for s in share], [round(x, 2) for x in hd]


def main():
    L = lambda a, b: float(np.linalg.norm(N[a] - N[b]))
    bar = [(PLATE_EDGE, rect(15, 14))]
    yc = cm.CLIP_Y; d14 = float(cm.N[14][1]) - yc                           # the picked clip points on rod 14-15 (2.5 inside node 14's saved socket tip)
    rows = {
        "v1 clips mid-span on 7-8 / 14-15 (4)": ([(7, 8, L(7, 8) - 77.8), (8, 7, L(7, 8) - 77.8), (14, 15, L(14, 15) - 44.2), (15, 14, L(14, 15) - 44.2)], ()),
        "B: 4 clips next to nodes (17 on 14-17 / 15-17, 14 / 15 on 14-15)": ([(17, 14, 8), (17, 15, 8), (14, 15, 8), (15, 14, 8)], ()),
        "B: 6 clips next to nodes 17, 14 / 15, 18": ([(17, 14, 8), (17, 15, 8), (14, 15, 8), (15, 14, 8), (18, 14, 8), (18, 15, 8)], ()),
        "B: 6 clips next to nodes 17, 14 / 15, 7 / 8": ([(17, 14, 8), (17, 15, 8), (14, 15, 8), (15, 14, 8), (8, 17, 8), (7, 17, 8)], ()),
        "A (picked): core tie, the 2 clips float in flight (bar 15 x 14 PETG)": ([], bar),
        f"C: core tie + 2 gripping clips on 14-15 at y +-{cm.CLIP_Y}": ([(14, 15, d14), (15, 14, d14)], bar),
        "C with a 15 x 10 bar": ([(14, 15, d14), (15, 14, d14)], [(PLATE_EDGE, rect(15, 10))]),
        "C with 2 more gripping clips next to node 17 (nose top)": ([(14, 15, d14), (15, 14, d14), (17, 14, 14), (17, 15, 14)], bar),
        "bound: camera rigid to the core, no clips": ([], [(PLATE_EDGE, cf.RIGID)]),
        f"bound: rigid tie + 2 clips at y +-{cm.CLIP_Y}": ([(14, 15, d14), (15, 14, d14)], [(PLATE_EDGE, cf.RIGID)]),
    }
    out = {}
    for k, (cl, ti) in rows.items():
        f, H, hz, sh, hd = model(cl, ti)
        cam_modes = [h for h, x in zip(hz, hd) if x > 0.3]
        out[k] = dict(modes_hz=hz, camera_share=sh, camera_disp_ratio=hd, first_mode_hz=hz[0], first_camera_mode_hz=cam_modes[0] if cam_modes else None)
        print(f"{k:62s} cam {str(out[k]['first_camera_mode_hz']):>6s} | " + "  ".join(f"{h:6.1f} ({s:.0%} {x:.2f})" for h, s, x in zip(hz, sh, hd)))
    f, H, hz, sh, hd = model([(14, 15, d14), (15, 14, d14)], bar)                 # the clips' share of a 250 N frontal hit on the picked concept
    u = f.solve({H: np.array([-250.0, 0, 0])}); share = []
    for m_ in f.M:
        if m_["name"] == "camera clip":
            kg, T, kl, Lm = f._k(m_); ue = np.concatenate([u[6 * m_["a"]:6 * m_["a"] + 6], u[6 * m_["b"]:6 * m_["b"] + 6]])
            share.append(np.round((kg @ ue)[6:9], 1).tolist())
    out["frontal 250 N on C (gap closed): force from each clip on its rod (N)"] = share
    print("frontal 250 N, clip on rod:", share)
    os.makedirs(os.path.dirname(OUT), exist_ok=True); json.dump(out, open(OUT, "w"), indent=1)


if __name__ == "__main__":
    main()
