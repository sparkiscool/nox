"""Heads v7 in the arm v7 sweep (24 Oct 2026, heads v7 part session): arm_kinematics' v7 cycle with the v7 head meshes in place of the
v6 carriage proxy and the v4 gripper, without editing the arm's files (build_parts is wrapped here). Per head: head<h> = the body
(the topology body when runs/head/v7/<A|B>/box_head_v7_<A|B>.stl exists, A frame, else the layout's design space: an envelope check, every body inside it
passes), fingA_<h> / fingB_<h> = the -X / +X carriers with their 623ZZ rollers (moved by the cycle's finger squeeze, inner faces at
10 - d), servo_<h> = servo + metal horn, grip<h> = the cam's swept disc, the three carbon rods and the spring; the bonded rack bar
rides with the body.
Head B = head A mirrored in X with the MG995 (its own frame, as the arm's B parts).
python3 head_v7_sweep.py [dt] [v7:<pack>:<pick>:<ret>] [--space]  -> runs/head/v7/sweep/ (log, json)."""
import os, sys, math, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(ROOT, "runs", "head", "v7", "sweep"); os.makedirs(OUT, exist_ok=True)
import arm_kinematics as K
import head_v7_layout as H
from arm_cad import tm, mirror_x, man
import trimesh

SERVO = dict(A="DS3218", B="MG995")
_ORIG_BUILD = K.build_parts


def body_mesh(h, use_space=False):
    p = os.path.join(ROOT, "runs", "head", "v7", h, f"box_head_v7_{h}.stl")          # the topology body, A frame (B mirrored below)
    if not use_space and os.path.exists(p):
        return man(trimesh.load(p, force="mesh"))
    return H.design_space(SERVO[h])


def head_parts(h, use_space=False):
    """the v7 head h in its own frame (A frame for A; mirrored for B), as manifolds"""
    sv = SERVO[h]
    P0 = H.parts(sv, open_=False)
    d0 = H.LATCH["tab_latched"][1] - H.BLADE["inner_sq"]                       # 4.0: the cycle's d = 0 has the inner faces at 10
    fingA = (P0["carrier-"] + P0["roller-"]).translate([-d0, 0, 0])
    fingB = (P0["carrier+"] + P0["roller+"]).translate([d0, 0, 0])
    from arm_cad import cylY
    grip = cylY(0, H.U_C, *H.CAM["y"], H.CAM["lobe"], 64) + P0["rods"] + P0["spring rod"] + P0["spring"]
    servo = P0["servo"] + P0["horn"]
    body = body_mesh(h, use_space) + H.rack_bar(sv)                          # the bonded rack bar rides with the body
    out = {f"head{h}": body, f"fingA_{h}": fingA, f"fingB_{h}": fingB, f"grip{h}": grip, f"servo_{h}": servo}
    if h == "B": out = {k: mirror_x(v) for k, v in out.items()}
    return out


def build(mode, use_space=False):
    P = _ORIG_BUILD(mode)
    for h in ("A", "B"):
        for k in (f"grip{h}", f"fingA_{h}", f"fingB_{h}", f"pinion_{h}", f"servo_{h}", f"head{h}"): P.pop(k, None)
        hp = head_parts(h, use_space)
        names = {f"head{h}": f"head {h} v7 body", f"fingA_{h}": f"carrier -X {h} + roller", f"fingB_{h}": f"carrier +X {h} + roller",
                 f"grip{h}": f"cam disc, rods, spring {h}", f"servo_{h}": f"{SERVO[h]} + horn {h}"}
        for k, m in hp.items():
            P[k] = K.Part(names[k], tm(m), f"head{h}", "#d35400", printed=k.startswith(("head", "fing")))
        K.ALLOWED.update({(f"grip{h}", f"head{h}"), (f"servo_{h}", f"head{h}"), (f"servo_{h}", f"grip{h}"), (f"fingA_{h}", f"grip{h}"),
                          (f"fingB_{h}", f"grip{h}"), (f"wheels{h}", f"head{h}"), (f"axles{h}", f"head{h}"), (f"bush{h}", f"head{h}")})
    K.PARTS_CACHE.clear(); K.PARTS_CACHE.update(P)
    return P


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    dt = float(args[0]) if args else 1.0
    mode = args[1] if len(args) > 1 else "v7:scout:-210:30"
    use_space = "--space" in sys.argv
    import arm_v7_layout as S7; S7.use()
    _, pk, pi, re_ = mode.split(":")
    K.ZP = K.L.Z0_A_HOME; K.KEYS = K.keys_v7(float(pi), float(re_)); K.T_END = 22.0
    K.build_parts = lambda m: build(m, use_space)
    K.OUT = OUT
    tag = mode.replace(":", "_") + ("_space" if use_space else "")
    logf = open(os.path.join(OUT, f"sweep_log_{tag}.txt"), "w")
    def log(s): logf.write(s + "\n"); logf.flush()
    rows = K.sweep(dt, log=log, mode=mode)
    worst = [r for r in rows if r["overlaps"]]
    tally = {}
    for r in rows:
        for a, b, v in r["overlaps"]:
            k = tuple(sorted((a, b))); tally[k] = max(tally.get(k, 0.0), v)
    res = dict(mode=mode, dt=dt, body="design space (envelope)" if use_space else "topology body", states=len(rows), states_with_overlaps=len(worst),
               min_clearance=min(r["clearance"] for r in rows), overlaps={f"{a} / {b}": v for (a, b), v in sorted(tally.items(), key=lambda kv: -kv[1])},
               closest=[(a, b, d, t) for (a, b), (d, t) in sorted(K.PAIR_MIN.items(), key=lambda kv: kv[1][0])[:25]
                        if a.startswith(("head", "fing", "grip", "servo")) or b.startswith(("head", "fing", "grip", "servo"))])
    json.dump(res, open(os.path.join(OUT, f"sweep_{tag}.json"), "w"), indent=1)
    print(json.dumps({k: v for k, v in res.items() if k != "closest"}, indent=1))
    for c in res["closest"][:12]: print("  closest", c)


if __name__ == "__main__":
    main()
