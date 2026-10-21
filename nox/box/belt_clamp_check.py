"""Belt clamp v1 clearance check in the arm frame (C): both heads' clamps (bed + plate + M3 head) at park, bay and
drone stroke against the hub (drawn), posts, motors + their 20T pulleys, the 80T sun, both bars, both idler ends, the
other head's allowed body and its belt runs. python belt_clamp_check.py -> min gaps (mm) and overlaps (mm3)."""
import numpy as np, manifold3d as mf
import arm_layout as L, arm_cad as A, belt_clamp as BC
box, cylY = A.box, A.cylY
TB = [[0, 0, -1, 0], [0, 1, 0, 0], [1, 0, 0, 0]]          # head B frame -> arm: X = -u, Z = x


def head_to_arm(m, h, z0):
    m = m.translate([0, 0, z0])
    return m if h == "A" else m.transform(TB)


def clamp_solid(h):
    c = BC.head_clamp(h)
    H = BC.HEADS[h]; uc = H["cz"][0] + BC.END_L + BC.LAND_L / 2
    head = BC.to_head(BC.cyl_p(BC.BELT_W / 2, uc, BC.P_FL[0] - 0.01, BC.P_FL[0] + BC.HEAD_H, 2.75), h)
    return c["drawn"] + c["plate"] + head


fixed = {}
Hb = A.hub_parts()
fixed["hub drawn"] = Hb["drawn"]
fixed["motors"] = None
for nm, M in L.MOTORS.items():
    m = A.rsquare(M, L.NEMA / 2, 2.0, L.Y_PLATE0)
    fixed["motors"] = m if fixed["motors"] is None else fixed["motors"] + m
fixed["20T A"] = cylY(L.MOTORS["A"]["xc"], L.MOTORS["A"]["zc"], 64.0, 74.0, L.P20_FL / 2)
fixed["20T B"] = cylY(L.MOTORS["B"]["xc"], L.MOTORS["B"]["zc"], 54.0, 64.0, L.P20_FL / 2)
fixed["80T sun"] = cylY(0, 0, L.Y_P80[0], L.Y_P80[1], L.P80_FL / 2) + cylY(0, 0, L.Y_P80[1], 72.0, L.P80_BOSS / 2)
barA = box(-20, 20, L.Y_BAR[0], L.Y_BAR[1], L.BAR_A[0], L.BAR_A[1])
fixed["bar A"] = barA
fixed["bar B"] = barA.transform(TB)
fixed["idler end A"] = A.idler_end("A") + cylY(L.X_HBELT_C, L.Z_IDLER, 64.0, 74.0, L.IDLER_OD / 2)
fixed["idler end B"] = (A.idler_end("B") + cylY(L.X_HBELT_C_B, L.Z_IDLER_B, 54.0, 64.0, L.IDLER_OD / 2)).transform(TB)
# belt runs of each head (pitch line +- 0.7 thick), full length motor..idler
def runs(h):
    xc = L.X_HBELT_C if h == "A" else L.X_HBELT_C_B
    yb = L.Y_HBELT if h == "A" else L.Y_HBELT_B
    zm, zi = (L.Z_MOTOR_A, L.Z_IDLER) if h == "A" else (L.Z_MOTOR_B, L.Z_IDLER_B)
    r = L.HEAD_BELT_PD / 2
    b = None
    for x in (xc - r, xc + r):
        s = box(x - 0.7, x + 0.7, yb[0], yb[1], zm, zi)
        b = s if b is None else b + s
    return b if h == "A" else b.transform(TB)
fixed["belt runs A (outer only for A)"] = box(L.X_HBELT_C + L.HEAD_BELT_PD / 2 - 0.7, L.X_HBELT_C + L.HEAD_BELT_PD / 2 + 0.7, 66, 72, L.Z_MOTOR_A, L.Z_IDLER)
fixed["belt runs B"] = runs("B")

rows = []
other = {"A": "B", "B": "A"}
for h in ("A", "B"):
    cs = clamp_solid(h)
    own_skip = {"A": ("bar A",), "B": ("bar B",)}[h]
    for z0, st in ((L.Z0_A_HOME, "park"), (L.Z0_BAY, "bay"), (L.Z0_DRONE, "drone")):
        c = head_to_arm(cs, h, z0)
        for k, f in fixed.items():
            if h == "A" and k.startswith("belt runs A"): continue
            if h == "B" and k == "belt runs B": f = fixed["belt runs A (outer only for A)"] if False else box(L.X_HBELT_C_B + L.HEAD_BELT_PD / 2 - 0.7, L.X_HBELT_C_B + L.HEAD_BELT_PD / 2 + 0.7, 56, 62, L.Z_MOTOR_B, L.Z_IDLER_B).transform(TB)
            ov = (c ^ f).volume()
            gap = c.min_gap(f, 15.0)
            rows.append((h, st, k, ov, gap))
        # the other head parked (its allowed body + drawn) and the other head's belt runs
        o = other[h]
        P = A.head_parts(o)
        ob = head_to_arm(P["allowed"] + P["drawn"], o, L.Z0_A_HOME)
        rows.append((h, st, f"head {o} parked (allowed+drawn)", (c ^ ob).volume(), c.min_gap(ob, 15.0)))
        if h == "A":
            rows.append((h, st, "belt runs B", (c ^ runs("B")).volume(), c.min_gap(runs("B"), 15.0)))
        else:
            rA = box(L.X_HBELT_C - L.HEAD_BELT_PD / 2 - 0.7, L.X_HBELT_C + L.HEAD_BELT_PD / 2 + 0.7, 66, 72, L.Z_MOTOR_A, L.Z_IDLER)
            rows.append((h, st, "belt runs A", (c ^ rA).volume(), c.min_gap(rA, 15.0)))
worst = {}
for h, st, k, ov, gap in rows:
    key = (h, k)
    if key not in worst or gap < worst[key][1] or ov > worst[key][0]:
        worst[key] = (max(ov, worst.get(key, (0, 99))[0]), min(gap, worst.get(key, (0, 99))[1]), st)
print(f"{'head':4s} {'against':38s} {'overlap mm3':>11s} {'min gap':>8s}")
for (h, k), (ov, gap, st) in sorted(worst.items()):
    print(f"{h:4s} {k:38s} {ov:11.2f} {gap:8.2f}" + ("   <-- OVERLAP" if ov > 0.05 else ""))
