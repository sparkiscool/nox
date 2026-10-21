"""Arm detent kinematics + interference (17 Oct 2026, hub v5). detent.py places the roller arm seated in a notch; the
earlier sweeps held it there. Here the roller follows the ring between notches: for every arm angle phi the roller arm
swings about its pivot until the 683ZZ just touches the notched rim (2D profile, exact), the hook moves and the springs
stretch. Outputs: psi(phi), lift, spring length and force, the restoring torque on the arm from the spring energy, and
the clearance of every moving detent part to every other part with that motion (manifold3d min_gap, C).
python3 detent_kin.py -> out/detent_kin.json"""
import math, json, time
import numpy as np
import shapely.geometry as sg, shapely.affinity as sa
import manifold3d as mf, trimesh
import arm_layout as L, arm_cad as A, detent as DT
from arm_cad import cylY, box

C0 = DT.CHOSEN
D0 = DT.design(C0["th"], C0["s"], C0["a"], C0["b"], C0["phi_d"], C0["psi"])
P, CR0, H0, AN = D0["P"], D0["C"], D0["H"], D0["A"]
Q = np.array(C0["Q"])
LO, HI = -278.0, 143.0                          # firmware soft limits (C)


def ring2d():
    """the notched rim (Y 67.8..69.9) in W's XZ plane as a polygon: the roller rides this."""
    disc = sg.Point(0, 0).buffer(DT.RING_R_OUT, 720)
    k = math.tan(math.radians(DT.NOTCH_HALF)); far = DT.RING_R_OUT + 5.0
    for a in DT.notch_angles(C0["th"]):
        t = math.radians(a); u = np.array([math.cos(t), math.sin(t)]); n = np.array([-u[1], u[0]])
        tri = sg.Polygon([DT.R_APEX * u, far * u + (far - DT.R_APEX) * k * n, far * u - (far - DT.R_APEX) * k * n])
        disc = disc.difference(tri)
    return disc

RING = ring2d()


def rot2(p, c, ang):
    s, co = math.sin(ang), math.cos(ang); d = p - c
    return c + np.array([co * d[0] - s * d[1], s * d[0] + co * d[1]])


def roller_w(psi, phi):
    """roller centre in W (XZ) for roller-arm swing psi (rad, about P in the arm frame) at arm angle phi."""
    c = rot2(CR0, P, psi)
    return rot2(c, np.zeros(2), math.radians(phi - 90.0))


def gap(psi, phi):
    c = roller_w(psi, phi); pt = sg.Point(c)
    d = RING.exterior.distance(pt)
    return (-d if RING.contains(pt) else d) - DT.ROLLER_R


OUT_SIGN = 1.0 if np.linalg.norm(rot2(CR0, P, 0.05)) > np.linalg.norm(CR0) else -1.0   # psi sign that lifts the roller


def solve(phi):
    lo, hi = -0.10 * OUT_SIGN, 0.60 * OUT_SIGN
    if gap(lo, phi) >= 0: return lo                                                   # never: the notch is a V
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        if gap(mid, phi) >= 0: hi = mid
        else: lo = mid
    return hi


def spring_len(psi):
    return float(np.linalg.norm(rot2(H0, P, psi) - AN))


def energy(Ls):                                     # one spring, N.mm, from its free length (F = F0 + k (L - L0))
    x = Ls - DT.SPRING_FREE
    return DT.SPRING_F0 * x + 0.5 * DT.SPRING_K * x * x


# ---------------- moving detent solids at a swing psi (arm frame, then to W) ----------------
def moving(psi):
    D = dict(D0); D["C"] = rot2(CR0, P, psi); D["H"] = rot2(H0, P, psi)
    S = DT.arm_solids(D)
    return dict(arm=S["arm"] + S["spacer"] + S["nut"] + S["hook"], roller=S["roller"], springs=S["springs"][0] + S["springs"][1])


def to_w(m, phi):
    return m.rotate([0, 90.0 - phi, 0])             # arm frame (A up) -> W at arm angle phi (rotY(90 - phi), as arm_kinematics)


if __name__ == "__main__":
    t0 = time.time()
    phis = np.round(np.arange(LO, HI + 1e-9, 0.25), 3)
    psis = np.array([solve(p) for p in phis])
    lift = np.array([np.linalg.norm(rot2(CR0, P, s)) - DT.R_SEAT for s in psis])
    Ls = np.array([spring_len(s) for s in psis])
    U = 2 * np.array([energy(l) for l in Ls])                                        # two springs (DEFAULT fit: both hooked)
    torque = -np.gradient(U, np.radians(phis)) * 1e-3                                # N.m on the arm, + = toward +phi
    print(f"kinematics {time.time() - t0:.0f} s: lift 0..{lift.max():.2f}, swing {math.degrees(abs(psis).max()):.2f} deg, "
          f"spring {Ls.min():.2f}..{Ls.max():.2f} (limit {DT.SPRING_LMAX}), torque peak {abs(torque).max():.3f} N.m (2 springs)")
    # ---- interference with the solved motion
    import arm_kinematics as K
    PARTS = K.build_parts("axle")
    man = lambda t: A.man(t)
    armfix = {}                                                                      # arm-frame parts (move with the arm)
    for nm, p in PARTS.items():
        if nm.startswith("detent_") or nm.startswith(("fing", "pinion", "servo")): continue
        if p.group == "arm": armfix[nm] = man(p.mesh)
        elif p.group == "armB": armfix[nm] = man(p.mesh).rotate([0, -90, 0])
    armfix["detent_tower"] = man(PARTS["detent_tower"].mesh)
    for h in ("A", "B"):                                                            # heads over their whole stroke
        hm = man(PARTS[f"head{h}"].mesh)
        for z0 in (L.Z0_A_HOME, L.Z0_B_WAIT, L.Z0_DRONE):
            m = hm.translate([0, 0, z0])
            armfix[f"head{h}@{z0:.0f}"] = m if h == "A" else m.rotate([0, -90, 0])
    fixed = {nm: man(PARTS[nm].mesh) for nm in ("detent_ring", "p80", "post", "stub", "brg0", "brg1")}
    allowed = {("roller", "detent_ring"), ("arm", "detent_tower"), ("springs", "detent_tower"), ("arm", "springs")}
    # (a) relative to the arm: depends on psi only
    rel = {}
    for s in np.linspace(psis.min(), psis.max(), 13):
        mv = moving(s)
        for a_, ma in mv.items():
            for b_, mb in armfix.items():
                if (a_, b_) in allowed: continue
                g = ma.min_gap(mb, 4.0)
                if g < rel.get((a_, b_), (9e9,))[0]: rel[(a_, b_)] = (round(g, 2), round(math.degrees(s), 2))
    # (b) relative to the fixed parts: every 0.25 deg of the soft range, with the solved swing
    fixg = {}; worst_roller = 9e9; hubring = 9e9
    hub_w = armfix["hub"]
    for p_, s in zip(phis[::2], psis[::2]):
        mv = moving(s)
        for a_, ma in mv.items():
            if a_ == "springs": continue                                            # Y 75..86, above every fixed part (C)
            mw = to_w(ma, p_)
            for b_, mb in fixed.items():
                if a_ == "roller" and b_ == "detent_ring":
                    worst_roller = min(worst_roller, mw.min_gap(mb, 1.0)); continue
                g = mw.min_gap(mb, 3.0)
                if g < fixg.get((a_, b_), (9e9,))[0]: fixg[(a_, b_)] = (round(g, 2), float(p_))
        for b_ in ("detent_ring", "p80", "post"):
            hubring = min(hubring, to_w(armfix["detent_tower"], p_).min_gap(fixed[b_], 3.0))
    ov = (to_w(moving(psis[0])["roller"], phis[0]) ^ fixed["detent_ring"]).volume()
    rows = sorted([(v[0], a, b, v[1]) for (a, b), v in rel.items()] + [(v[0], a, b, v[1]) for (a, b), v in fixg.items()])
    print("closest pairs (gap, moving, other, at swing deg / phi):")
    for r in rows[:10]: print("  ", r)
    print(f"tower vs ring / 80T / post over the range: {hubring:.2f}; roller to ring min gap {worst_roller:.3f} (rolling contact)")
    out = dict(phi=phis.tolist(), psi_deg=np.degrees(psis).round(3).tolist(), lift=lift.round(3).tolist(), spring=Ls.round(3).tolist(),
               torque=torque.round(4).tolist(), notches=DT.notch_angles(C0["th"]), stops=DT.stops(),
               closest=[dict(gap=r[0], moving=r[1], other=r[2], at=r[3]) for r in rows[:12]], tower_fixed=round(hubring, 2),
               roller_ring=round(worst_roller, 3), P=P.tolist(), C=CR0.tolist(), H=H0.tolist(), A=AN.tolist(), Q=Q.tolist(),
               out_sign=OUT_SIGN, time_s=round(time.time() - t0))
    json.dump(out, open("out/detent_kin.json", "w"))
    print(f"done {time.time() - t0:.0f} s")
