"""Heads v7 access sweep (24 Oct 2026, heads v7 part session; concept review r3, docs/generative.md: access paths swept against every
part). Each service path is a solid moved along its path in the head's frame, checked at every state of a swap cycle against every part
that is not the head's own (arm, other head, receivers, bays; the cartridges left out: service is done with the packs out):
  - servo + horn + cam out along -u (the body keeps the window under the servo's footprint), 0..32 in 2 mm steps;
  - the 3 mm L-key to the near axle (down Y from the plate top, its short leg along +X), the pin spanner on the eccentric flange;
  - the two 3.0 grip rods and the spring rod pushed in from +X (a 64 straight run past the +X wall);
  - a ball driver (shaft 3, 58 long) on each of the two ear screws from -Y, up through the plate's Ø7 holes.
A path is open at a state when no position along it touches a part. The result names the states (t, z0, phi) where each path is open.
python3 head_v7_access.py [v7:<pack>:<pick>:<ret>]  -> runs/head/v7/access/access_<mode>.json"""
import os, sys, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_kinematics as K
import head_v7_sweep as W
import head_v7_layout as H
from arm_cad import box, cylX, cylY, tm, mirror_x
import trimesh.collision as tc

OUT = os.path.join(H.ROOT, "runs", "head", "v7", "access"); os.makedirs(OUT, exist_ok=True)


def tools(h):
    sv = W.SERVO[h]; P0 = H.parts(sv, open_=False)
    servo = P0["servo"] + P0["horn"] + H.cam_solid()
    T = {f"servo + horn + cam out -u ({sv})": [servo.translate([0, 0, -d]) for d in np.arange(0.0, 32.1, 2.0)]}
    x, u = H.WHEELS[0]; yk = H.PLATE["y"][1] + 16.0
    T["L-key 3 mm, near axle"] = [cylY(x, u, H.PLATE["y"][1] + 0.5, yk, 1.5, 16) + cylX(yk, u, x, x + 50.0, 1.5, 16)]
    x, u = H.WHEELS[2]
    T["pin spanner, eccentric flange"] = [cylY(x, u, H.PLATE["y"][1] + 0.5, 100.0, 7.0, 32)]
    for yr in H.RODS["y"]:
        T[f"grip rod in from +X (Y {yr:g})"] = [cylX(yr, H.RODS["u"], H.RODS["wall_out"], H.RODS["wall_out"] + 64.0, H.ROD_D / 2, 16)]
    T["spring rod in from +X"] = [cylX(H.SPRING_ROD["y"], H.SPRING_ROD["u"], H.SPRING_ROD["wall_out"], H.SPRING_ROD["wall_out"] + 64.0, H.ROD_D / 2, 16)]
    s = H.SERVOS[sv]; ye = H.Y_SERVO_TOP - s["H"] + s["ear_y"]
    T["ball driver on the two ear screws (-Y, through the plate)"] = [sum((cylY(xh, uh, ye - 70.0, ye - 12.0, 1.5, 16) for (xh, uh) in H.ear_screws(sv)[1:]),
                                                                           cylY(*H.ear_screws(sv)[0], ye - 70.0, ye - 12.0, 1.5, 16))]
    if h == "B": T = {k: [mirror_x(m) for m in v] for k, v in T.items()}
    return {k: [tm(m) for m in v] for k, v in T.items()}


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "v7:scout:-210:30"
    import arm_v7_layout as S7; S7.use()
    _, pk, pi, re_ = mode.split(":")
    K.ZP = K.L.Z0_A_HOME; K.KEYS = K.keys_v7(float(pi), float(re_)); K.T_END = 22.0
    P = W.build(mode, True); K.PARTS_CACHE.clear(); K.PARTS_CACHE.update(P)
    res = dict(mode=mode, note="cartridges left out (service with the packs out); body = the design space", paths={})
    for h in ("A", "B"):
        own = {k for k in P if k.endswith(h) and k.startswith(("head", "fing", "grip", "servo"))}
        mgr = tc.CollisionManager()
        for nm, p in P.items():
            if nm not in own: mgr.add_object(nm, p.mesh, np.eye(4))
        TL = tools(h)
        for name, steps in TL.items():
            open_at = []; blocked = {}
            for t in np.arange(0.0, K.T_END + 0.01, 0.5):
                Tr, carts, s = K.transforms(t)
                for nm in P:
                    if nm not in own: mgr.set_transform(nm, Tr[nm])
                hit = set()
                for m in steps:
                    ok, names = mgr.in_collision_single(m, transform=Tr[f"head{h}"], return_names=True)
                    if ok: hit |= names
                if hit: blocked[round(float(t), 1)] = sorted(hit)
                else: open_at.append(dict(t=round(float(t), 1), z0=round(s["zA" if h == "A" else "zB"], 1), phi=round(s["phi"], 1)))
            res["paths"][f"{h}: {name}"] = dict(open_states=len(open_at), of=len(np.arange(0.0, K.T_END + 0.01, 0.5)), open=open_at[:40],
                                                 blocked_by=sorted({n for v in blocked.values() for n in v}))
            print(h, name, "open at", len(open_at), "states; blocked by", res["paths"][f"{h}: {name}"]["blocked_by"][:8])
    json.dump(res, open(os.path.join(OUT, f"access_{mode.replace(':', '_')}.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
