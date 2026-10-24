"""Motion artifact for funnel v8 plain's opening (PR 81 review round 2, CLAUDE.md "Every idea comes with a visual"):
the box catch assembly in box W (nox/box/box_catch.assembly.json: panels v7 on +-y, v8 plain on +-x, hook latch v2 with the
plungers out) and the Overseer latch v1 set (receiver + lid on the drone, cartridge; runs/box_system/v2/latch_ctx, M)
centred on the catch centre, stepping yaw -5 .. +5 deg at lift 0 and 5.7, then the cartridge down the swap path. The
series strip reads the gap from the set to the +x / -x panels per frame (manifold3d min_gap, C on M meshes; the swept
check in opening_sets.py gives 3.42 / 3.42 over the whole sweep with the latch_ctx meshes on main after PR 86; 3.15 / 3.27 with the earlier ones).
python3 opening_motion.py [--no-video] -> runs/funnel_panel/v8_plain/motion.html (+ .mp4 / .gif / .png)"""
import os, sys
import numpy as np, trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
sys.path[:0] = [HERE, os.path.dirname(HERE), os.path.join(ROOT, "nox")]
import opening_sets as OS
import catch_system as CS
import assembly as AS
from motion import Scene

RUN = os.path.join(ROOT, "runs", "funnel_panel", "v8_plain")
FPS, DUR = 10, 14.0
DOWN = 90.0                                   # cartridge travel shown (the check sweeps 120)


def rz_about(deg, c):
    t = np.radians(deg); T = np.eye(4)
    T[:2, :2] = [[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]]
    T[:3, 3] = c - T[:3, :3] @ c
    return T


def state(t):
    """yaw (deg), lift (mm), cartridge down (mm) at time t."""
    s = lambda a, b: min(max((t - a) / (b - a), 0.0), 1.0)
    if t < 3.0: return -5 + 10 * s(0, 3), 0.0, 0.0
    if t < 4.0: return 5.0, 5.7 * s(3, 4), 0.0
    if t < 7.0: return 5 - 10 * s(4, 7), 5.7, 0.0
    if t < 8.0: return -5 + 5 * s(7, 8), 5.7 * (1 - s(7, 8)), 0.0
    if t < 11.0: return 0.0, 0.0, DOWN * s(8, 11)
    return 0.0, 0.0, DOWN * (1 - s(11, 14))


def main(video=True):
    a = AS.load(os.path.join(os.path.dirname(HERE), "box_catch.assembly.json"))
    a = a.at(**{n: 1.0 for n in a.names if n.startswith("latch plunger")})
    W = CS.SEAT_TO_W; c = W[:3, 3].copy()                       # the catch centre axis (seat x = y = 0) in W
    up = W[:3, :3] @ np.array([0, 0, 1.0])
    pk = "overseer"
    rec = trimesh.util.concatenate([OS.load(pk, "receiver"), OS.load(pk, "receiver_lid")])
    zc = float(rec.bounds[:, 2].mean())
    R = OS.to_seat(rec, zc); R.apply_transform(W)
    C = OS.to_seat(OS.load(pk, "cartridge"), zc); C.apply_transform(W)
    sc = Scene("Funnel v8 plain: opening 191 x 168 vs the Overseer latch set",
               "box W. Overseer latch v1 receiver + cartridge centred on the catch centre, yaw -5..+5 deg at lift 0 and 5.7, "
               "cartridge down the swap path. Min gap over the sweep (opening_sets.py, 0.5 deg steps): 3.15 mm to the +x panels, 3.27 mm to the -x panels, 0 mm3 (C); the strip reads it per frame")
    panels = {}
    for p, m in zip(a.parts, a.meshes()):
        nm = p["name"]
        if nm.startswith("drone"): continue
        plain = "v8 plain" in str(p.get("version", ""))
        grp = "panels v8 plain (+-x)" if plain else ("panels v7 (+-y)" if "panel" in nm else "hook latch v2")
        col = "#b9c2cc" if plain else ("#7fa7d9" if "panel" in nm else "#cc6644")
        sc.part(nm, m, group=grp, color=col, printed=True, faces=20000)
        if "panel" in nm and ("+x" in nm or "-x" in nm): panels[nm] = m
    sc.part("Overseer receiver + lid (latch v1)", R, group="Overseer set", color="#3a6fd8", printed=False, faces=20000)
    sc.part("Overseer cartridge (latch v1)", C, group="Overseer set", color="#e8a33c", printed=False, faces=20000)
    mp = {s: OS.man(trimesh.util.concatenate([m for n, m in panels.items() if n.endswith(s)])) for s in ("+x", "-x")}
    mR, mC = OS.man(R), OS.man(C)
    n = int(DUR * FPS); frames, gp, gm, yaw_s, lift_s, down_s = [], [], [], [], [], []
    for i in range(n):
        yaw, lift, down = state(i / FPS)
        T = np.eye(4); T[:3, 3] = up * lift
        T = T @ rz_about(yaw, c)
        Tc = np.eye(4); Tc[:3, 3] = -up * down
        Tc = T @ Tc
        frames.append({"Overseer receiver + lid (latch v1)": T, "Overseer cartridge (latch v1)": Tc})
        S = mR.transform(T[:3, :4].tolist()) + mC.transform(Tc[:3, :4].tolist())
        gp.append(S.min_gap(mp["+x"], 30.0)); gm.append(S.min_gap(mp["-x"], 30.0))
        yaw_s.append(yaw); lift_s.append(lift); down_s.append(down)
    sc.timeline(frames, duration_s=DUR)
    sc.series("gap to the +x panels", gp, unit="mm", bad_below=3.0)
    sc.series("gap to the -x panels", gm, unit="mm", bad_below=3.0)
    sc.series("yaw", yaw_s, unit="deg"); sc.series("lift", lift_s, unit="mm"); sc.series("cartridge down", down_s, unit="mm")
    sc.view("side: both +-x edges in profile", eye=(c[0] + 560, c[1], c[2] + 20), target=(c[0], c[1], c[2] - 10))
    sc.view("+x bottom edge from inside", eye=(c[0] - 60, c[1] - 40, c[2] + 120), target=(c[0], c[1] + 95, c[2] + 5))
    sc.view("from below, through the opening", eye=(c[0] + 40, c[1] - 30, c[2] - 430), target=(c[0], c[1], c[2] + 5))
    print("min gap over the frames +x / -x:", round(min(gp), 2), round(min(gm), 2))
    out = sc.write(os.path.join(RUN, "motion.html"))
    if video: sc.video(os.path.join(RUN, "motion.mp4"), view="from below, through the opening")
    return out


if __name__ == "__main__":
    main(video="--no-video" not in sys.argv)
