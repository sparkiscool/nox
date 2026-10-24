"""Heads v7 access sweep (24 Oct 2026, heads v7 part session; concept review r3, docs/generative.md: access paths swept against every
part). Each service path is a solid moved along its path in the head's frame, checked at every state of a swap cycle against every part
that is not the head's own (arm, other head, receivers, bays; the cartridges left out: service is done with the packs out):
  - servo + horn + cam out along -u, commanded OPEN first (the carriers held apart by the open dwell; also checked against the head's
    own carriers and rollers in that pose), 0..40 in 2 mm steps;
  - the 3 mm L-key to the near axle (down Y from the plate top, its short leg along +X), the pin spanner on the eccentric flange;
  - the two 3.0 grip rods and the spring rod pushed in from +X (a 64 straight run past the +X wall);
  - a driver (Ø3 bit 100 long + Ø25 handle) on the retainer's two M3 along +u from -u, and the retainer out along -u.
A path is open at a state when no position along it touches a part. The result names the states (t, z0, phi) where each path is open.
python3 head_v7_access.py [v7:<pack>:<pick>:<ret>]  -> runs/head/v7/access/access_<mode>.json"""
import os, sys, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_kinematics as K
import head_v7_sweep as W
import head_v7_layout as H
from arm_cad import box, cylX, cylY, cylZ, tm, mirror_x
import trimesh.collision as tc

OUT = os.path.join(H.ROOT, "runs", "head", "v7", "access"); os.makedirs(OUT, exist_ok=True)


def tools(h):
    """per path: a list of solids along it (head frame), and the head's own parts it must also clear"""
    sv = W.SERVO[h]; P1 = H.parts(sv, open_=True)
    cam_open = H.cam_solid().translate([0, 0, -H.U_C]).rotate([0, -120.0, 0]).translate([0, 0, H.U_C])   # the servo commanded OPEN
    servo = P1["servo"] + P1["horn"] + cam_open
    own_open = P1["carrier+"] + P1["carrier-"] + P1["roller+"] + P1["roller-"]
    T = {f"servo + horn + cam out -u, commanded OPEN first ({sv})": ([servo.translate([0, 0, -d]) for d in np.arange(0.0, 40.1, 2.0)], own_open)}
    x, u = H.WHEELS[0]; yk = H.PLATE["y"][1] + 16.0
    T["L-key 3 mm, near axle"] = ([cylY(x, u, H.PLATE["y"][1] + 0.5, yk, 1.5, 16) + cylX(yk, u, x, x + 50.0, 1.5, 16)], None)
    x, u = H.WHEELS[2]
    T["pin spanner, eccentric flange"] = ([cylY(x, u, H.PLATE["y"][1] + 0.5, 100.0, 7.0, 32)], None)
    for yr in H.RODS["y"]:
        T[f"grip rod in from +X (Y {yr:g})"] = ([cylX(yr, H.RODS["u"], H.RODS["wall_out"], H.RODS["wall_out"] + 64.0, H.ROD_D / 2, 16)], None)
    T["spring rod in from +X"] = ([cylX(H.SPRING_ROD["y"], H.SPRING_ROD["u"], H.SPRING_ROD["wall_out"], H.SPRING_ROD["wall_out"] + 64.0, H.ROD_D / 2, 16)], None)
    u0 = H.RETAINER["u"][0]
    drv = None
    for (xe, ye) in H.RETAINER["ends"]:                                       # a 2.0 hex bit (Ø3 shaft) 100 long + a Ø25 handle 90 long, along +u
        d = cylZ(xe, ye, u0 - 100.0, u0 - 0.2, 1.5, 16) + cylZ(xe, ye, u0 - 190.0, u0 - 100.0, 12.5, 32)
        drv = d if drv is None else drv + d
    T["driver on the two retainer screws (along +u, from -u)"] = ([drv], None)
    T["retainer out -u"] = ([H.retainer().translate([0, 0, -d]) for d in np.arange(0.0, 30.1, 2.0)], None)
    if h == "B": T = {k: ([mirror_x(m) for m in v], None if o is None else mirror_x(o)) for k, (v, o) in T.items()}
    return {k: ([tm(m) for m in v], None if o is None else tm(o)) for k, (v, o) in T.items()}


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
        for name, (steps, own_m) in TL.items():
            open_at = []; blocked = {}
            if own_m is not None:                                             # the head's own moving parts in the service pose
                om = tc.CollisionManager(); om.add_object("own", own_m)
                own_hit = [i for i, m in enumerate(steps) if om.in_collision_single(m)]
            else:
                own_hit = []
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
            if own_hit: blocked = {**blocked, "own": ["own carriers / rollers"]}; open_at = []
            res["paths"][f"{h}: {name}"] = dict(open_states=len(open_at), own_parts_hit_at_steps=own_hit, of=len(np.arange(0.0, K.T_END + 0.01, 0.5)), open=open_at[:40],
                                                 blocked_by=sorted({n for v in blocked.values() for n in v}))
            print(h, name, "open at", len(open_at), "states; blocked by", res["paths"][f"{h}: {name}"]["blocked_by"][:8])
    json.dump(res, open(os.path.join(OUT, f"access_{mode.replace(':', '_')}.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
