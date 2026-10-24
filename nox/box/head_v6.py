"""Heads A / B v6 (24 Oct 2026, hub + heads worker): the v5 head with the glide and the belt clamp replaced by the arm v5
carriage (plate + wing + eccentric boss, 3 V-wheel axle bores, the hanging rack web with its m1 teeth). Gripper v4d unchanged
(arm_cad.gripper_v4: guide block, servo plate, servo / pinion / rack / blade / cartridge keep-outs). Head B = head A mirrored in X
(rack, eccentric and gripper on B's -X), so one generated run serves both.
Every carriage number comes from arm_v5_layout (the arm-mechanism worker's file); the getattr fallbacks are that worker's 24 Oct
values (rail top 93, rail bolts at X 0, ribbon channel at X 0, rack web u -47, Z0S + 95.6 drone insert, printed eccentric bush)
until its PR is on main. Frame: head A (X tangential, Y axial, u along the rail from the carriage's inner end; arm Z = u + z0).
python3 head_v6.py -> drawn checks + every 3D gap to the v5 parts over the stroke (C)."""
import math, os
import numpy as np, manifold3d as mf
import arm_layout as L, arm_cad as A, arm_v5_layout as V
import hub_v6 as HV                                           # first: syncs arm_v5_layout to the 24 Oct mechanism numbers until they are on main
from arm_cad import box, cylY, cylZ, hull, mirror_x

g = lambda n, d: getattr(V, n, d)
PLATE_X, PLATE_Y, PLATE_U = V.PLATE_X, V.PLATE_Y, V.PLATE_U
WING_X, WING_U = V.WING_X, V.WING_U
WEB_X, WEB_Y = V.WEB_X, V.WEB_Y
WEB_U = (min(V.WEB_U[0], -47.0), V.WEB_U[1])                # u -47 (24 Oct, +6 drone insert)
RACK_PITCH_X, RACK_TEETH_Y = V.RACK_PITCH_X, V.RACK_TEETH_Y
WHEELS = V.WHEELS
ECC = g("ECC", dict(x=-22.0, u=24.0, shank_r=5.5, bore_in_plate_r=5.6, plate_boss_r=7.6, flange_af=13.0, flange_y=(98.0, 100.5), throw=0.8, bolt_r=2.65))
AXLE_KEEP = g("AXLE_KEEP", dict(r=5.0, y=(98.0, 106.0)))
ECC_KEEP = g("ECC_KEEP", dict(r=13.0, y=(98.0, 101.0)))
Z0S = tuple(sorted(set(V.Z0S) | {L.Z0_BAY + 5.0 + g("DRONE_PUSH", 6.0)}))     # park 32, wait 81, bay 86.1, drone 89.6, drone insert 95.6
CHAN = g("RAIL_CHAN", dict(x=(-2.0, 2.0), y=(89.0, 93.0), mouth=3.2))
WIRE_X = 0.5 * (CHAN["x"][0] + CHAN["x"][1])                  # head wiring entry over the rail's ribbon channel (X 0)
WIRE_DU, WIRE_D = 8.0, 4.5                                    # as v5: Ø4.5 at u 8
AXLE_R = 2.75                                                 # M5 clearance Ø5.5
GUSSET = dict(x=(35.0, WEB_X[0]), y=(74.5, PLATE_Y[0]))       # generated stiffener room on the web's inner face: wheels end X 34 (+1),
                                                              # the detent arm tops out at Y 74.1 (B side, mirrored)
def carriage():
    """drawn carriage of head A (plate + eccentric boss + wing + web with teeth) and its cuts (axle bores, bush bore, keep-outs)"""
    y0, y1 = PLATE_Y
    import arm_v5_cad as V5C                                     # the arm-mechanism worker's carriage (one source for the head and its sweep)
    K = V5C.carriage("A", rack_teeth=True)
    drawn = K["printed"]                                         # plate + eccentric boss + wing + rack web with m1 teeth, axle / bush bores
    cuts = None
    for x, u in WHEELS:
        if abs(x - ECC["x"]) < 1e-6 and abs(u - ECC["u"]) < 1e-6:
            c = cylY(x, u, y0 - 1, y1 + 0.01, ECC["bore_in_plate_r"], 64) + cylY(x, u, *ECC_KEEP["y"], ECC_KEEP["r"], 64) + \
                cylY(x, u, ECC["flange_y"][1] - 0.01, AXLE_KEEP["y"][1] + ECC["flange_y"][1] - y1, AXLE_KEEP["r"], 48)   # bush hex (13 AF wrench) + the bolt head on it
        else:
            c = cylY(x, u, y0 - 1, y1 + 0.01, AXLE_R, 32) + cylY(x, u, *AXLE_KEEP["y"], AXLE_KEEP["r"], 48)
        cuts = c if cuts is None else cuts + c
    wire = cylY(WIRE_X, WIRE_DU, CHAN["y"][1] - 0.5, y1 + 3.0, WIRE_D / 2, 32)
    cuts += wire
    zl, zw = A.ZIP_SLOT                                         # zip fin beside the entry (as v5): 2 thick, 5 tall, 8 long, slot through it along X
    fx = WIRE_X + WIRE_D / 2 + 0.75
    fin = box(fx, fx + 2.0, y1 - 0.01, y1 + 5.0, WIRE_DU - 4.0, WIRE_DU + 4.0)
    fin -= box(fx - 0.5, fx + 2.5, y1 + 1.0, y1 + 1.0 + zw, WIRE_DU - zl / 2, WIRE_DU + zl / 2)
    return dict(printed=drawn, drawn=drawn + fin, fin=fin, cuts=cuts, wire=wire)

def head_env(y0, y1):
    """v4 / v5 envelope rule (arm_cad.head_env_region) from the plate top up: park bisector, r 98.5 at the drone outside the own mouth,
    u <= 14 under the pack, u >= -26"""
    return A.head_env_region(y0, y1)

def head_v6_parts(variant="A"):
    C = carriage()
    G = A.gripper_v4()
    genv = head_env(PLATE_Y[1] - 0.01, L.Y_C + 47.0)
    gus = box(GUSSET["x"][0], GUSSET["x"][1] + 0.01, GUSSET["y"][0], GUSSET["y"][1] + 0.01, *WEB_U)
    drawn = C["drawn"] + G["drawn"]
    cuts = C["cuts"] + G["cuts"]
    env = genv + gus + drawn
    parts = dict(drawn=drawn, cuts=cuts, env=env, guide=G["guide"], splate=G["plate"], carriage=C, gusset=gus)
    if variant == "B":
        parts = {k: (mirror_x(v) if isinstance(v, mf.Manifold) else v) for k, v in parts.items()}
        parts["carriage"] = {k: mirror_x(v) for k, v in C.items()}
    parts["allowed"] = parts["env"] - parts["cuts"]
    return parts

# ------------------------------------------------------------------ checks against the v5 parts (arm frame)
def head_to_arm(m, h, z0):
    """head frame (u from the carriage inner end) -> arm frame at stroke z0. A: arm Z = u + z0. B (mirrored head): arm X = -(u + z0),
    arm Z = X_B (arm_v5_layout.to_arm)."""
    m = m.translate([0, 0, z0])
    if h == "A": return m
    return m.transform([[0, 0, -1, 0], [0, 1, 0, 0], [1, 0, 0, 0]])     # (X_B, Y, Z) -> (-Z, Y, X_B)

def checks(mesh_A=None, verbose=True):
    """3D gaps of head A / B (the part, or the drawn bodies) to the hub v6 posts / tower, the rails, the pinions, the axle drive, the
    detent and each other, over every pair of stroke states (C). The wheels, nuts and bush are bought / mechanism parts: their gaps
    are the layout's. Returns {pair: gap}."""
    P = head_v6_parts("A")
    hA = mesh_A if mesh_A is not None else (P["drawn"] - P["cuts"])
    hB = mirror_x(hA)
    st = HV.v5_static()
    H = HV.hub_v6_parts()
    D, S = H["detent"]
    st["hub"] = (H["drawn"] - H["cuts"]) ^ box(-200, 200, 50.0, 200, -200, 200)
    st["detent"] = S["arm"] + S["spacer"] + S["hook"] + S["springs"][0] + S["springs"][1] + S["anchor"]
    touch = {("A", "pinionA"), ("B", "pinionB")}                    # rack on pinion (designed mesh, 0.2 backlash)
    gaps = {}
    def rec(k, gg): gaps[k] = min(gaps.get(k, 1e9), gg)
    for zA in Z0S:
        a = head_to_arm(hA, "A", zA)
        for nm, m in st.items():
            if ("A", nm) not in touch: rec(("A", nm), a.min_gap(m, 6.0))
        for zB in Z0S:
            if zA > 40 and zB > 40 and zA != 81.0 and zB != 81.0: continue    # both out at once never happens except B waiting
            b = head_to_arm(hB, "B", zB)
            rec(("A", "B"), a.min_gap(b, 6.0))
    for zB in Z0S:
        b = head_to_arm(hB, "B", zB)
        for nm, m in st.items():
            if ("B", nm) not in touch: rec(("B", nm), b.min_gap(m, 6.0))
    if verbose:
        for k, gg in sorted(gaps.items(), key=lambda kv: kv[1])[:14]:
            print(f"  {gg:6.2f}  {k[0]} / {k[1]}")
    return gaps

if __name__ == "__main__":
    P = head_v6_parts("A")
    d = P["drawn"] - P["cuts"]
    print(f"Z0S {Z0S}; web u {WEB_U}; wire entry X {WIRE_X}")
    print(f"drawn {d.volume() * 1.27e-3:.1f} g, bodies {len(d.decompose())}, env {P['env'].volume() / 1000:.0f} cm3, design {(P['allowed'] - d).volume() / 1000:.0f} cm3")
    print("bbox", [round(v, 1) for v in P["env"].bounding_box()])
    print("gaps (C, drawn bodies):")
    checks()
