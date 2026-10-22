"""Collar contact check with the floor feet, skids and legs, the pack + receiver envelope and the flow sensor on the drone,
then a flat-floor drop on the feet. Full-physics model (catch_v4/s2_catch_real.py), wheels 2.5 proud as drawn.

17 Oct 03:27 (receiver v3 bottom layout, Spark): pack nose-tail x -45.8..41.8, |y| <= 24.3, bottom z -44.5 (E); flow
sensor board x 47..69, |y| <= 8, z -5..3; feet + x-running skids from foot_opt.json; legs to the side hub roots.
Seat at the cage v2 lower ring z 33.2 (was 32.5 in catch_v4); mass build-up with the pack lowered (mass_patch, E).
Run from a folder holding s2_collar_settle.py, s2_node_wheels.py, s2_catch_real.py (catch_v4/):
  python3 s2_feet_sim.py sweep 40     collar drops (offsets 0 / 10 / 20 / 40 mm, yaw +-5, tilt 0 / 2, drift in / out)
  python3 s2_feet_sim.py floor        flat-floor drops
"""
import sys, os, math, itertools, json, re, numpy as np, mujoco
import s2_collar_settle as s
s.ZS = 33.2e-3                                   # cage v2 lower ring (prop plane 41.2 - 8, C)
import s2_catch_real as r, s2_node_wheels as w
mm = 1e-3
w.PROUD = 2.5 * mm

def mass_patch():
    """props at the raised prop plane, pack + cartridge centred at drone x -2, z -20.5, long side along x (E)"""
    P = [p for p in r.PARTS if not p[0].startswith(("cartridge", "props", "lower ring", "upper ring", "8 PETG"))]
    P += [("lower ring 2.5 carbon", 6.6, "ring", 33.2), ("upper ring 2.0 carbon", 4.3, "ring", 49.2),
          ("8 PETG corner nodes + wheels", 9.6, "corners", 33.2), ("props x4", 16.0, "motors", 41.2),
          ("cartridge + 4S 1500", 191.4, (-2.0, 0, -20.5), (79, 40, 48))]
    r.PARTS = P; r.M, r.CG, r.INERTIA = r.mass_props()
mass_patch()

F = json.load(open(os.environ.get("FOOT_JSON", "foot_opt.json")))
xf, yf, zc = F["foot_centre"]; pr = F["pad_r"]; zs = F["skid"]["z"]
E = np.array(json.loads(os.environ.get("LEG_END", "[0.0, 50.0, 20.75]")), float)   # side hub root (strut_route.json)
FR = 'friction="0.4 0.005 0.0001"'
def box(name, lo, hi):
    lo, hi = np.array(lo) * mm, np.array(hi) * mm; c, h = (lo + hi) / 2, (hi - lo) / 2
    return f'<geom name="{name}" type="box" size="{h[0]} {h[1]} {h[2]}" pos="{c[0]} {c[1]} {c[2]}" {FR}/>'
EX = []
for sx, sy in itertools.product((1, -1), (1, -1)):
    EX.append(f'<geom name="x_foot{sx}{sy}" type="sphere" size="{pr*mm}" pos="{sx*xf*mm} {sy*yf*mm} {zc*mm}" {FR}/>')
    Fc = np.array([xf, yf, zc]); v = E - Fc; u = v / np.linalg.norm(v)
    A = (Fc + 4.0 * u) * [sx, sy, 1] * mm; B = (E - 2.5 * u) * [sx, sy, 1] * mm
    EX.append(f'<geom name="x_leg{sx}{sy}" type="capsule" size="{1.5*mm}" fromto="{A[0]} {A[1]} {A[2]} {B[0]} {B[1]} {B[2]}" {FR}/>')
for sy in (1, -1):
    EX.append(f'<geom name="x_skid{sy}" type="capsule" size="{1.25*mm}" fromto="{xf*mm} {sy*yf*mm} {zs*mm} {-xf*mm} {sy*yf*mm} {zs*mm}" {FR}/>')
if os.environ.get("OLD_PACK"):                           # 16 Oct layout for comparison: long side along y, bottom -41.5
    EX.append(box("x_pack", (-20.0, -39.5, -41.5), (20.0, 39.5, 6.5)))
    EX.append(box("x_recv", (-24.3, -40.0, -39.5), (24.3, 40.0, 8.0)))
elif os.environ.get("ONLY") != "feet":                     # ONLY=feet: feet, skids and legs alone (no pack / receiver / flow)
    # receiver v3 bottom (C, receiver_v3.py / s2_cartridge.py): cartridge 79 long, closed end at x 37.5, 38 wide, bottom
    # z -44.5; lead-in feet and locating posts |y| <= 24.3 from the mouth (z -42.5) between the posts (x -38.5..34.5);
    # end tabs and deck x -45.8..41.8 from the rail tops (z -6.5) up; plate underside z 18.5
    dx = float(os.environ.get("PACK_DX", "0"))            # test shift of the whole pack + receiver along x
    EX.append(box("x_pack", (-41.5 + dx, -19.0, -44.5), (37.5 + dx, 19.0, 3.5)))
    EX.append(box("x_recv", (-38.5 + dx, -24.3, -42.5), (34.5 + dx, 24.3, -6.5)))
    EX.append(box("x_rtop", (-45.8 + dx, -24.3, -6.5), (41.8 + dx, 24.3, 18.5)))
    EX.append(box("x_flow", (47.0, -8.0, -5.0), (69.0, 8.0, 3.0)))
r.EXTRA = EX

def sweep(gate):
    m = r.model("wheelD", 0.4)
    watch = {i for i in range(m.ngeom) if m.geom(i).name.startswith("x_")}
    ref = r.fly(m, 0, 0, 0, 0, [0, 0, -0.10])[0]
    offs = [(0, 0)] + [(d * mm * math.cos(a), d * mm * math.sin(a)) for d in (10, 20, gate) for a in [k * math.pi / 4 for k in range(8)]]
    ok, hit_runs, names, hits = [], 0, {}, []
    for (x, y), yaw, tilt, dr in itertools.product(offs, (-5, 0, 5), (0, 2), (1, -1)):
        a = math.atan2(y, x) if (x or y) else 0.0
        r.LAST_HITS = 0; r.LAST_NAMES.clear()
        p, yf_, tf, _ = r.fly(m, x, y, yaw, tilt, [dr * .15 * math.cos(a), dr * .15 * math.sin(a), -.1], watch=watch)
        e = (p - ref) / mm; ok.append(math.hypot(e[0], e[1]) < 0.5 and abs(yf_) < 0.3 and tf < 0.3)
        hit_runs += r.LAST_HITS > 0
        for n in r.LAST_NAMES: names[n] = names.get(n, 0) + 1
        if r.LAST_HITS: hits.append([round(math.hypot(x, y) / mm), round(math.degrees(a)), yaw, tilt, dr, sorted(r.LAST_NAMES)])
    return dict(hits=hits if os.environ.get("LIST") else len(hits), gate=gate, runs=len(ok), centred_pct=round(100 * sum(ok) / len(ok), 1),
                runs_with_added_part_contact=hit_runs, contacts_by_part=names,
                mass_g=round(r.M * 1000, 1), cg_mm=[round(c / mm, 2) for c in r.CG])

def floor_drop(v=0.5, tilt=8.0, yaw=0.0, mu=0.6):
    """flat floor, motors off, drop at v with a tilt: first touch, final tilt, what it rests on"""
    m0 = r.model("wheelD", 0.4)
    p_ = f"/tmp/_floor_{os.getpid()}.xml"; mujoco.mj_saveLastXML(p_, m0); x = open(p_).read()
    x = re.sub(r'<geom[^>]*type="box"[^>]*rgba="\.8 \.3 \.3 \.4"[^>]*/>', '', x)       # collar faces out
    x = x.replace("<worldbody>", f'<worldbody><geom name="floor" type="plane" size="1 1 0.01" friction="{mu} 0.005 0.0001"/>', 1)
    m = mujoco.MjModel.from_xml_string(x); d = mujoco.MjData(m)
    q = np.zeros(4); mujoco.mju_euler2Quat(q, np.array([math.radians(tilt), math.radians(tilt / 2), math.radians(yaw)]), "xyz")
    d.qpos[:3] = [0, 0, 0.12]; d.qpos[3:7] = q; d.qvel[2] = -v
    first = None
    for _ in range(int(1.5 / m.opt.timestep)):
        mujoco.mj_step(m, d)
        if first is None:
            for ci in range(d.ncon):
                c = d.contact[ci]; n1, n2 = m.geom(c.geom1).name, m.geom(c.geom2).name
                if "floor" in (n1, n2): first = (n2 if n1 == "floor" else n1) or "ring/wheel"; break
    R = d.xmat[m.body("drone").id].reshape(3, 3); tilt_f = math.degrees(math.acos(min(1, R[2, 2])))
    touching = sorted({(m.geom(d.contact[i].geom1).name or "ring/wheel") if m.geom(d.contact[i].geom2).name == "floor"
                       else (m.geom(d.contact[i].geom2).name or "ring/wheel") for i in range(d.ncon)})
    return dict(v=v, tilt0=tilt, first_touch=first, final_tilt=round(tilt_f, 1), resting_on=touching)

if __name__ == "__main__":
    if len(sys.argv) > 3:
        w.set_slope(45, edge_half=float(sys.argv[3]) * mm)
    if sys.argv[1] == "sweep":
        o = sweep(float(sys.argv[2])); o["opening"] = 2 * float(sys.argv[3]) if len(sys.argv) > 3 else 168
        print(json.dumps(o), flush=True)
    else:
        for v, t in ((0.5, 0), (0.5, 8), (1.0, 15), (1.0, 25), (1.0, 30)):
            print(json.dumps(floor_drop(v, t)), flush=True)
