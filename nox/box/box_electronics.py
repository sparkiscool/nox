#!/usr/bin/env python3
"""Box electronics v1 (24 Oct 2026): the box's electrical system end to end.

Board placement on the rear enclosure sheet (search), nets with gauges, voltage drops, power budget,
cut list and connector counts, the swap state machine, and the diagram (layout, schematic, states).
Doc: nox/box/box-electronics.md. Numbers C unless tagged; board sizes spec / E as in BOARDS.
W frame (nox/dimensions.md I): X tangential, Y axial (+ away from the stub wall), Z up; the
rear sheet's inner face is Y -66 (box-stand.md Enclosure, C).

Run: python3 nox/box/box_electronics.py
 -> runs/box_electronics/v1/{result.json, layout.svg, schematic.svg, brake.svg, states.svg, diagram.html}
"""
import json, math, pathlib, random

ROOT = pathlib.Path(__file__).resolve().parents[2]
OUT = ROOT / "runs" / "box_electronics" / "v1"

# ------------------------------------------------------------------ sheet, keep-outs (C unless tagged)
SHEET_Y = -66.0            # rear sheet inner face (box-stand.md, C)
STANDOFF = 10.0            # nylon M3 F-F standoffs, board plane at Y -56 (DEFAULT)
DEPTH_MAX = -2.0           # nothing of the panel past Y -2: motor bodies start Y 2 (dimensions I), stub face Y 0
USABLE = (-160.0, 160.0, -195.0, 232.0)   # X, Z: post legs reach X +-164.3 (box-stand.md), floor -201, top 240 (4 margin, E)
KEEPOUT = [("mast + kb", (-40.0, 40.0, -201.0, 50.0))]   # mast X +-30, head top Z 40 (stand_v6 MAST) + 10 (C)
GAP = 6.0                  # board to board (wire and finger room, DEFAULT)

# fixed bodies on the sheet: name -> (x0, x1, z0, z1, depth, why)
FIXED = {
    "PSU": (42.0, 157.0, -170.0, 45.0, 30.0,
            "LRS-350-24 215 x 115 x 30 (spec) on its bottom face against the sheet, 4 x M4 x 6 through the sheet "
            "(thread engagement 3.0 = the spec max), terminal end up, air out the bottom end 31 over the floor"),
    "CHARGER": (-162.0, -44.0, -200.0, -29.0, 59.0,
                "dual-channel DC charger 171 x 118 x 59 (listing, iCharger 456DUO class, E) standing on the floor on its "
                "118 x 59 end, strapped to the sheet; Y -66..-7 (5 to the stub face)"),
    "IEC": (78.0, 128.0, 56.0, 86.0, 30.0,
            "IEC C14 fused switched inlet, cut-out 48 x 28 (E, CHECK) in the sheet over the PSU terminal end"),
    "ESTOP": (112.0, 152.0, 188.0, 228.0, 45.0,
              "22 mm mushroom E-stop, NC, hole 22.5 (E) in the sheet, reached from behind the box"),
}

# movable boards: name -> (w along X, h along Z, depth Y, tag, what)
BOARDS = {
    "SKR":     (103.75, 70.25, 25, "spec", "BTT SKR Mini E3 V3: 3 x TMC2209 (heads A, B, axle), Klipper MCU"),
    "PI":      (85.0, 56.0, 35, "spec", "Raspberry Pi 4 B 2 GB + GPIO screw-terminal HAT (stack 35 E)"),
    "PCA":     (62.5, 25.4, 15, "spec", "PCA9685 16-ch PWM (Adafruit 815 class), I2C 0x60 (A5 bridged)"),
    "RELAY":   (75.0, 55.0, 20, "E", "4-ch 5 V opto relay board, SRD-05VDC-SL-C 10 A 30 VDC"),
    "INA_KA":  (32.0, 22.0, 10, "E", "INA226 module, shunt R010 (2512), I2C 0x41: keep-alive"),
    "INA_S":   (32.0, 22.0, 10, "E", "INA226 module, shunt R010, I2C 0x44: Scout bay -90"),
    "INA_O":   (32.0, 22.0, 10, "E", "INA226 module, shunt R010, I2C 0x45: Overseer bay 180"),
    "BUCK_KA": (66.0, 47.0, 30, "E", "XL4016-class CC/CV buck 8 A: keep-alive 13.0 V, CC 3.0 A"),
    "BUCK_SV": (66.0, 47.0, 30, "E", "XL4016-class CC/CV buck 8 A: servo rail 6.0 V"),
    "BUCK_5V": (41.0, 26.0, 10, "E", "24 -> 5.1 V 5 A buck (Pololu D36V50F5 class): Pi + logic"),
    "FUSE":    (105.0, 60.0, 35, "E", "6-way blade fuse box with negative bus (12/24 V)"),
    "BUS":     (70.0, 30.0, 20, "E", "3 x Wago 221-415 on tape: 6.0 V servo bus, 5 V logic bus, GND"),
    "INA_GR":  (32.0, 22.0, 10, "E", "INA226 module, shunt R010, I2C 0x40: gripper servo pair behind K4 (stall cut, heads v7)"),
    "ADC":     (28.0, 18.0, 8, "E", "ADS1115 16-bit 4-ch ADC module at 3.3 V, I2C 0x48: one grip-hall line per head (2 DRV5055A4 at 3.3 V, 3:1 sum)"),
}

# off-sheet endpoints: name -> ((x, y, z) W, detour E, route)
ENDPOINTS = {
    "ARM":   ((-27.0, 3.0, 0.0), 20.0, "arm bundle at the stub v3 lead channel exit, flange X -27 (box-arm.md Lead exit), drops beside the mast"),
    "L1":    ((-122.0, 75.0, 182.0), 60.0, "hook latch unit, -X seated face, Y_C - 100: up the corner post, over the side sheet top, down to the housing"),
    "L2":    ((-122.0, 275.0, 182.0), 60.0, "hook latch unit, -X face, Y_C + 100"),
    "R1":    ((122.0, 75.0, 182.0), 60.0, "hook latch unit, +X face, Y_C - 100"),
    "R2":    ((122.0, 275.0, 182.0), 60.0, "hook latch unit, +X face, Y_C + 100"),
    "PANEL_P": ((0.0, 275.0, 160.0), 60.0, "2 spring probes in the +x plain funnel panel (B+), Y_C + 100: up the corner post, over the sheet top, down the panel (hub DEFAULT 24 Oct, was the pogo post)"),
    "PANEL_M": ((0.0, 75.0, 160.0), 60.0, "2 spring probes in the -x plain funnel panel (B-), Y_C - 100: same route"),
    "BAY_S": ((0.0, 175.0, -150.0), 120.0, "Scout live bay -90 (bottom), XT60E1-M on its float carrier: side corridor, under the ring's -Y edge"),
    "BAY_O": ((-150.0, 175.0, 0.0), 60.0, "Overseer live bay 180 (reserved, wired when built): left side corridor"),
}
# latch unit positions (E): seated faces are the W +-X faces (drone +-y); wheels at drone x +-100 = W Y_C +-100,
# plunger axis 54.1 up the 45 deg face from the 168 opening's edge: |X| 84 + 38.3 = 122, Z 144.5 - 0.8 + 38.3 = 182.

# ------------------------------------------------------------------ wire data (E: copper 20 C, silicone stranded)
OHM_PER_M = {28: 0.2129, 26: 0.1339, 24: 0.0842, 22: 0.0530, 20: 0.0333, 18: 0.0210, 16: 0.0132, 14: 0.0083}
AMPS_MAX = {28: 0.8, 26: 2.2, 24: 3.5, 22: 5.0, 20: 7.5, 18: 10.0, 16: 13.0, 14: 18.0}   # E, bundled, derated

# nets: (name, a, b, conductors, awg, I_cont, I_peak, rail V, kind)
NETS = [
    ("mains L/N/PE", "IEC", "PSU", 3, 18, 6.8, 6.8, 115, "mains"),   # LRS-350 input 6.8 A at 115 VAC (spec)
    ("24 V main", "PSU", "FUSE", 2, 14, 7.4, 11.9, 24, "power"),
    ("F1 charger in", "FUSE", "CHARGER", 2, 16, 6.5, 6.5, 24, "power"),
    ("F2 motion in", "FUSE", "ESTOP", 2, 18, 3.2, 5.2, 24, "power"),
    ("SKR VIN", "ESTOP", "SKR", 2, 18, 1.0, 3.0, 24, "power"),
    ("servo buck in", "ESTOP", "BUCK_SV", 2, 18, 0.3, 2.2, 24, "power"),
    ("F3 keep-alive in", "FUSE", "BUCK_KA", 2, 18, 0.9, 1.8, 24, "power"),
    ("F4 logic in", "FUSE", "BUCK_5V", 2, 20, 0.5, 1.0, 24, "power"),
    ("KA + to K1", "BUCK_KA", "RELAY", 1, 18, 1.0, 3.0, 13, "power"),
    ("KA + K1 to INA", "RELAY", "INA_KA", 1, 18, 1.0, 3.0, 13, "power"),
    ("KA + probes", "INA_KA", "PANEL_P", 1, 18, 1.0, 3.0, 13, "power"),
    ("KA - probes", "BUCK_KA", "PANEL_M", 1, 18, 1.0, 3.0, 13, "power"),
    ("CH1 + to K2", "CHARGER", "RELAY", 1, 16, 3.0, 6.5, 16.8, "power"),
    ("CH1 + K2 to INA", "RELAY", "INA_S", 1, 16, 3.0, 6.5, 16.8, "power"),
    ("bay S +", "INA_S", "BAY_S", 1, 16, 3.0, 6.5, 16.8, "power"),
    ("bay S -", "CHARGER", "BAY_S", 1, 16, 3.0, 6.5, 16.8, "power"),
    ("CH2 + to K3", "CHARGER", "RELAY", 1, 16, 6.0, 6.5, 16.8, "power"),
    ("CH2 + K3 to INA", "RELAY", "INA_O", 1, 16, 6.0, 6.5, 16.8, "power"),
    ("bay O +", "INA_O", "BAY_O", 1, 16, 6.0, 6.5, 16.8, "power"),
    ("bay O -", "CHARGER", "BAY_O", 1, 16, 6.0, 6.5, 16.8, "power"),
    ("6.0 V to bus", "BUCK_SV", "BUS", 2, 18, 1.0, 8.0, 6.0, "power"),
    ("6.0 V to PCA V+", "BUS", "PCA", 2, 18, 0.3, 3.2, 6.0, "power"),
    ("6.0 V gripper to K4", "BUS", "RELAY", 1, 22, 0.5, 5.0, 6.0, "servo"),
    ("6.0 V gripper K4 to INA", "RELAY", "INA_GR", 1, 22, 0.5, 5.0, 6.0, "servo"),
    ("6.0 V gripper pair", "INA_GR", "ARM", 2, 22, 0.5, 5.0, 6.0, "servo"),
    ("latch servo L1", "PCA", "L1", 3, 22, 0.1, 0.8, 6.0, "servo"),
    ("latch servo L2", "PCA", "L2", 3, 22, 0.1, 0.8, 6.0, "servo"),
    ("latch servo R1", "PCA", "R1", 3, 22, 0.1, 0.8, 6.0, "servo"),
    ("latch servo R2", "PCA", "R2", 3, 22, 0.1, 0.8, 6.0, "servo"),
    ("latch hall L1", "PI", "L1", 3, 28, 0.01, 0.01, 5.0, "signal"),
    ("latch hall L2", "PI", "L2", 3, 28, 0.01, 0.01, 5.0, "signal"),
    ("latch hall R1", "PI", "R1", 3, 28, 0.01, 0.01, 5.0, "signal"),
    ("latch hall R2", "PI", "R2", 3, 28, 0.01, 0.01, 5.0, "signal"),
    ("seat sense", "PI", "INA_KA", 1, 28, 0.001, 0.001, 3.3, "signal"),   # GPIO 24 -> 1k -> BAT54 onto the B+ line at the INA226 out
    ("5.1 V to Pi", "BUCK_5V", "PI", 2, 20, 1.5, 3.5, 5.1, "power"),
    ("5.1 V to bus", "BUCK_5V", "BUS", 2, 20, 0.2, 0.5, 5.1, "power"),
    ("relay JD-VCC", "BUS", "RELAY", 2, 22, 0.15, 0.3, 5.1, "power"),
    ("relay IN + VCC", "PI", "RELAY", 5, 26, 0.01, 0.01, 3.3, "signal"),
    ("I2C Pi-PCA", "PI", "PCA", 4, 26, 0.01, 0.01, 3.3, "signal"),
    ("I2C PCA-INA_KA", "PCA", "INA_KA", 4, 26, 0.01, 0.01, 3.3, "signal"),
    ("I2C INA_KA-INA_S", "INA_KA", "INA_S", 4, 26, 0.01, 0.01, 3.3, "signal"),
    ("I2C INA_S-INA_O", "INA_S", "INA_O", 4, 26, 0.01, 0.01, 3.3, "signal"),
    ("I2C INA_O-INA_GR", "INA_O", "INA_GR", 4, 26, 0.01, 0.01, 3.3, "signal"),
    ("I2C INA_GR-ADC", "INA_GR", "ADC", 4, 26, 0.01, 0.01, 3.3, "signal"),
    ("USB Pi-SKR", "PI", "SKR", 1, 0, 0.1, 0.1, 5.0, "cable"),
    ("steppers A, B, axle", "SKR", "ARM", 12, 26, 1.4, 2.0, 24, "stepper"),
    ("axle hall", "SKR", "ARM", 3, 28, 0.01, 0.01, 5.0, "signal"),
    ("gripper PWM", "PCA", "ARM", 2, 28, 0.01, 0.01, 6.0, "signal"),
    ("grip halls", "ADC", "ARM", 3, 28, 0.01, 0.01, 3.3, "signal"),   # heads v7: H_A, H_B (each head's 2 halls summed 3:1 on the head) + their own 3.3 V, upstream of K4
    ("GND bus to star", "BUS", "FUSE", 1, 18, 1.2, 8.0, 0, "power"),
]
# arm-side lengths of the bundle conductors past the stub exit: box-wiring.md cut list / 1.2 less its 250 E fixed tail
# (arm_wiring.cut_list); the fixed tail is now this placement's route from the board to the stub lead-channel exit
ARM_SIDE = {"steppers A, B, axle": 646 / 1.2 - 250, "axle hall": 595 / 1.2 - 250, "6.0 V gripper pair": 435 / 1.2 - 250,
            "gripper PWM": 435 / 1.2 - 250, "grip halls": 435 / 1.2 - 250}


def pos(name, place):
    """3D terminal of a board / endpoint (board centre on the board plane, E)."""
    if name in ENDPOINTS:
        return ENDPOINTS[name][0]
    if name in FIXED:
        x0, x1, z0, z1, d, _ = FIXED[name]
        return ((x0 + x1) / 2, SHEET_Y + d / 2, (z0 + z1) / 2)
    x, z, rot = place[name]
    return (x, SHEET_Y + STANDOFF, z)


def route_len(a, b, place):
    pa, pb = pos(a, place), pos(b, place)
    L = sum(abs(p - q) for p, q in zip(pa, pb))
    for n in (a, b):
        if n in ENDPOINTS:
            L += ENDPOINTS[n][1]
    return L


def rect(name, place):
    w, h = BOARDS[name][0], BOARDS[name][1]
    x, z, rot = place[name]
    if rot:
        w, h = h, w
    return (x - w / 2, x + w / 2, z - h / 2, z + h / 2)


def overlap(r1, r2, g):
    return not (r1[1] + g <= r2[0] or r2[1] + g <= r1[0] or r1[3] + g <= r2[2] or r2[3] + g <= r1[2])


def violations(place):
    v = 0.0
    names = list(place)
    rs = {n: rect(n, place) for n in names}
    for n in names:
        r = rs[n]
        X0, X1, Z0, Z1 = USABLE
        v += max(0, X0 - r[0]) + max(0, r[1] - X1) + max(0, Z0 - r[2]) + max(0, r[3] - Z1)
        for _, k in KEEPOUT:
            if overlap(r, k, GAP):
                v += 50
        for f in FIXED.values():
            if overlap(r, f[:4], GAP):
                v += 50
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if overlap(rs[a], rs[b], GAP):
                v += 50
    return v


def cost(place):
    c = 0.0
    for name, a, b, n, awg, ic, ip, V, kind in NETS:
        w = n * (0.25 + ic)          # conductor count + current weighted (DEFAULT objective)
        c += w * route_len(a, b, place)
    return c


def legal_one(n, place):
    """board n legal against the sheet, keep-outs, fixed bodies and every other placed board."""
    r = rect(n, place)
    X0, X1, Z0, Z1 = USABLE
    if r[0] < X0 or r[1] > X1 or r[2] < Z0 or r[3] > Z1:
        return False
    if any(overlap(r, k, GAP) for _, k in KEEPOUT) or any(overlap(r, f[:4], GAP) for f in FIXED.values()):
        return False
    return all(not overlap(r, rect(m, place), GAP) for m in place if m != n)


def greedy(rng):
    """legal start: boards in random order, each at the first free grid spot scanning from a random corner."""
    X0, X1, Z0, Z1 = USABLE
    order = sorted(BOARDS, key=lambda n: -BOARDS[n][0] * BOARDS[n][1] * (0.7 + 0.6 * rng.random()))
    place = {}
    xs = [X0 + 5 * i for i in range(int((X1 - X0) / 5) + 1)]
    zs = [Z1 - 5 * i for i in range(int((Z1 - Z0) / 5) + 1)]
    if rng.random() < 0.5:
        xs = xs[::-1]
    for n in order:
        done = False
        for z in zs:
            for x in xs:
                for rot in (False, True):
                    place[n] = (x, z, rot)
                    if legal_one(n, place):
                        done = True
                        break
                if done:
                    break
            if done:
                break
        if not done:
            raise RuntimeError(f"no room for {n}")
    return place


def search(seed, iters=150000):
    """simulated annealing over board centres (2.5 grid) and 90 deg turns, legal moves only."""
    rng = random.Random(seed)
    place = greedy(rng)
    cur = cost(place)
    best, bestc = dict(place), cur
    T0 = 3e3
    names = list(BOARDS)
    for i in range(iters):
        T = T0 * (1 - i / iters) + 1.0
        n = rng.choice(names)
        old = dict(place)
        u = rng.random()
        if u < 0.1:
            x, z, r = place[n]
            place[n] = (x, z, not r)
            moved = [n]
        elif u < 0.25:
            m = rng.choice(names)
            (xn, zn, rn), (xm, zm, rm) = place[n], place[m]
            place[n], place[m] = (xm, zm, rn), (xn, zn, rm)
            moved = [n, m]
        else:
            x, z, r = place[n]
            s = 60 * (1 - i / iters) + 2.5
            place[n] = (round((x + rng.gauss(0, s)) / 2.5) * 2.5, round((z + rng.gauss(0, s)) / 2.5) * 2.5, r)
            moved = [n]
        if not all(legal_one(k, place) for k in moved):
            place = old
            continue
        c2 = cost(place)
        if c2 < cur or rng.random() < math.exp((cur - c2) / T):
            cur = c2
            if cur < bestc:
                best, bestc = dict(place), cur
        else:
            place = old
    return best, bestc


def ceil5(v):
    return int(5 * math.ceil(v / 5.0))


def nets_table(place):
    rows = []
    for name, a, b, n, awg, ic, ip, V, kind in NETS:
        L = route_len(a, b, place)
        if name in ARM_SIDE:
            L += ARM_SIDE[name]
        cut = ceil5(L * 1.2 + 40)                  # +20 % slack (box-wiring rule) + 2 x 20 strip / terminal (E)
        drop = None
        if awg and kind in ("power", "servo", "stepper") and V:
            loop = 2 * L / 1000 if n >= 2 or kind != "power" else L / 1000
            drop = dict(cont_V=round(OHM_PER_M[awg] * loop * ic, 3), peak_V=round(OHM_PER_M[awg] * loop * ip, 3),
                        cont_pct=round(100 * OHM_PER_M[awg] * loop * ic / V, 2),
                        peak_pct=round(100 * OHM_PER_M[awg] * loop * ip / V, 2))
        amp_ok = (not awg) or ip <= AMPS_MAX[awg]
        rows.append(dict(net=name, a=a, b=b, conductors=n, awg=awg, I_cont=ic, I_peak=ip, rail_V=V, kind=kind,
                         route_mm=round(L), cut_mm=cut, ampacity_ok=amp_ok, drop=drop,
                         fixed_tail_mm=round(L - ARM_SIDE[name]) if name in ARM_SIDE else None))
    return rows


# ------------------------------------------------------------------ power budget (E)
LOADS = [  # (load, rail, W_cont, W_peak, efficiency of its converter)
    ("Pi 4 + HAT + later USB camera", "5.1 V", 6.0, 9.0, 0.90),
    ("relay coils (2 on typical, 4 peak)", "5.1 V", 0.72, 1.44, 0.90),
    ("hall switches x5, PCA9685, INA226 x3 logic", "5.1 V", 0.15, 0.15, 0.90),
    ("SKR + 3 NEMA17 (2 A; detent holds with drivers off)", "24 V", 1.5, 30.0, 1.0),
    ("2 gripper servos (DS3218, MG995)", "6.0 V", 1.5, 30.0, 0.90),
    ("4 latch servos MG90S (not with the grippers in the sequence, budgeted together)", "6.0 V", 0.3, 19.2, 0.90),
    ("keep-alive: Scout ~8.5 W, Overseer ~24 W, CC limit 3.0 A at 13.0 V", "13.0 V", 8.5, 39.0, 0.90),
    ("charger, input limit 6.5 A (Scout 2C ~50 W + Overseer ~1C ~96 W out)", "24 V", 156.0, 156.0, 1.0),
]
PSU_W = 350.0  # LRS-350-24, 14.6 A (spec)


def budget():
    rows, cont, peak = [], 0.0, 0.0
    for name, rail, wc, wp, eff in LOADS:
        ic, ip = wc / eff, wp / eff
        cont += ic
        peak += ip
        rows.append(dict(load=name, rail=rail, W_cont=wc, W_peak=wp, W_at_24V_cont=round(ic, 1), W_at_24V_peak=round(ip, 1)))
    return dict(rows=rows, total_cont_W=round(cont, 1), total_peak_W=round(peak, 1), psu_W=PSU_W,
                peak_pct=round(100 * peak / PSU_W, 1), peak_A=round(peak / 24, 2))


# ------------------------------------------------------------------ swap state machine (DEFAULT; timings E)
STATES = [
    # (state, action, pass condition, time limit s, on fail)
    ("IDLE", "drone away; live bays charge; heads parked; K1 open; latch in", "swap request from the mission script", None, None),
    ("PRESTAGE", "head A picks the fullest charged pack (bay relay open and bay I < 50 mA first); head B waits open at 81 under the seat", "both heads at pose, no stall", 6.0, "A5 stall"),
    ("WAIT_SEAT", "drone lands under its own landing mode; kill at first touch", "seat sense low for 100 ms", 30.0, "no seat: mission retries the landing"),
    ("LATCH_OUT", "PCA ch 0..3 to OUT (servo pulse per unit from calibration), seat sense ignored from here", "all 4 halls low within 0.5 s", 0.5, "A1 latch not out"),
    ("KA_ON", "close K1 while the old pack is higher than 13.0 V (0 A through the pads)", "INA_KA bus 12.8..13.2 V and I < 0.5 A for 0.2 s", 0.5, "A3 keep-alive fault"),
    ("PULL", "head B rises to 89.6, closes, retracts with the dead pack: the drone XT60 unmates under the pull", "stroke done, no stall, INA_KA I > 100 mA at the end of the stroke (pads carry the drone)", 2.0, "A4 pull / keep-alive lost"),
    ("TURN", "axle 90 deg: head A to the top; <= 100 full steps/s through each notch; step target + the bay's gravity offset", "no stall; hall state matches phi", 1.1, "A5 stall"),
    ("INSERT", "head A extends, last 12 at <= 20 mm/s, cartridge to the bolt seat, 1 over-travel into the TPU strip", "full stroke, no stall", 2.0, "A5 stall"),
    ("MATE_CHECK", "hold the stroke; read INA_KA", "I > 100 mA seen during INSERT, then I < 50 mA within 1.0 s (the new pack carries the drone)", 1.0, "A6 mate fail"),
    ("RELEASE", "gripper opens (bolts into the drone pockets), head A retracts to park", "park, no stall", 1.0, "A5 stall"),
    ("KA_OFF", "open K1 (0 A: pack higher)", "INA_KA I < 50 mA before and bus < 1 V after", 0.3, "A3 keep-alive fault"),
    ("LATCH_IN", "PCA ch 0..3 to IN, only with a pack mated (pack-out hold)", "all 4 halls high within 0.5 s", 0.5, "A11 latch stuck out"),
    ("READY", "SWAP_DONE to the mission; the mission arms the drone; take off", "seat sense high (drone gone)", 60.0, "drone stays: mission decides"),
    ("STORE", "axle to the free bay (the live bay when free); head B extends with the dead pack, bay relay open", "full stroke, no stall", 4.0, "A5 stall"),
    ("BAY_MATE", "relay open: read the bay INA226 bus voltage", "13.0..17.0 V within 1.0 s", 1.0, "A7 bay mate fail"),
    ("CHARGE", "close the bay relay at 0 A, start the charger channel", "charger done, or I < C/10 and rest voltage >= 4.15 V/cell", None, "A8 charge fault"),
    ("CHARGE_END", "stop the channel; wait I < 50 mA; open the bay relay", "I < 50 mA then relay open", 5.0, "A8 charge fault"),
]
ABORTS = [
    ("A1 latch not out", "a hall still high 0.5 s after OUT", "all 4 IN, retry OUT once; second fail: latch IN, SWAP_FAIL, the drone keeps its pack (no motion yet)"),
    ("A2 latch lost mid-swap", "any hall goes high between LATCH_OUT and LATCH_IN", "stop all motion where it is, keep K1 as it is, alarm; the plungers self-lock under load, so a drone cannot be released by this"),
    ("A3 keep-alive fault", "bus outside 12.8..13.2 V, or I > 2.5 A for 0.2 s with the old pack in, or K1 will not open at 0 A", "open K1; before PULL: SWAP_FAIL with the old pack in; after: alarm, K1 and the latch stay as they are (pack-out hold)"),
    ("A4 pull / keep-alive lost", "INA_KA I < 100 mA at the end of the pull stroke", "either the pack did not come out or the pads did not make: head B re-inserts what it holds; always the pack-out hold (latch OUT, K1 closed), SWAP_FAIL, alarm (the current cannot prove a mate here)"),
    ("A5 stall", "TMC2209 DIAG (StallGuard) on X / Y during a fast move, a Klipper shutdown, the E-stop, or the axle hall out of its window", "stop all motion, latch stays OUT, K1 stays as it is (the drone stays powered), alarm; Spark clears by hand; heads re-home before the next cycle"),
    ("A6 mate fail", "INA_KA I >= 50 mA 1.0 s after the full stroke", "back off 5, push again once at 10 mm/s; still >= 50 mA: pull the pack back into head A, drone stays on keep-alive, alarm (no time limit: the box powers it)"),
    ("A7 bay mate fail", "bay bus voltage not 13.0..17.0 V with the relay open", "retry the insert once; then leave the pack, no charge, alarm (not on the flight path)"),
    ("A8 charge fault", "charger error, bay I > 7 A, pack over 45 C (charger probe, CHECK), or no current 5 s after start", "stop the channel, wait I < 50 mA, open the relay, mark the pack"),
    ("A9 bay interlock", "a head move into a live bay with its relay closed or I >= 50 mA", "refused in software before the move; stop the charger first"),
    ("A10 seat lost before latch", "seat sense high before LATCH_OUT", "back to WAIT_SEAT"),
    ("A11 latch stuck out", "a hall still low 0.5 s after IN", "OUT then IN once more; then alarm: no SWAP_DONE (the drone cannot leave), it sits on its new pack disarmed"),
    ("A12 keep-alive at 0 A, pack out", "after PULL, no pack mated, keep-alive I < 50 mA for 2 s", "open K1 and read seat sense: high (drone lifted off) -> latch IN, SWAP_FAIL, alarm; low (drone still seated: pins or K1 / buck / F3 lost) -> latch stays OUT, alarm (catch PR 93)"),
]


# ------------------------------------------------------------------ SVG drawing
STYLE = """
.bg{fill:var(--bg,#ffffff)} .sheet{fill:var(--sheet,#eef3f6);stroke:var(--line,#8a97a3);stroke-width:1}
.ko{fill:var(--ko,#e2e2e2);stroke:var(--line,#8a97a3);stroke-dasharray:4 3}
.fix{fill:var(--fix,#dfe8f0);stroke:var(--ink,#1d2730);stroke-width:1}
.brd{fill:var(--brd,#fff4d8);stroke:var(--ink,#1d2730);stroke-width:1}
.pwr{fill:var(--pwr,#ffe1dc);stroke:var(--ink,#1d2730);stroke-width:1}
.sig{fill:var(--sig,#e1f0e4);stroke:var(--ink,#1d2730);stroke-width:1}
.off{fill:var(--off,#ececf6);stroke:var(--ink,#1d2730);stroke-width:1;stroke-dasharray:5 3}
.t{font:11px system-ui,sans-serif;fill:var(--ink,#1d2730)} .ts{font:9px system-ui,sans-serif;fill:var(--muted,#55626e)}
.tb{font:bold 12px system-ui,sans-serif;fill:var(--ink,#1d2730)}
.w24{stroke:var(--c24,#c0392b);stroke-width:2.2;fill:none} .w13{stroke:var(--c13,#d35400);stroke-width:1.8;fill:none}
.w6{stroke:var(--c6,#8e44ad);stroke-width:1.6;fill:none} .w5{stroke:var(--c5,#2471a3);stroke-width:1.4;fill:none}
.wch{stroke:var(--cch,#b7950b);stroke-width:2;fill:none} .wsg{stroke:var(--csg,#1e8449);stroke-width:1.1;fill:none;stroke-dasharray:3 2}
.wst{stroke:var(--cst,#34495e);stroke-width:1.6;fill:none} .wm{stroke:var(--cm,#7b241c);stroke-width:2.6;fill:none}
.arr{stroke:var(--ink,#1d2730);stroke-width:1.2;fill:none;marker-end:url(#ah)} .ab{stroke:var(--c24,#c0392b);stroke-width:1;fill:none;stroke-dasharray:4 3}
"""


def svg_wrap(body, w, h, standalone):
    defs = ('<defs><marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">'
            '<path d="M0,0 L10,5 L0,10 z" fill="currentColor"/></marker></defs>')
    style = f"<style>{STYLE}</style>" if standalone else ""
    head = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" width="100%" '
            f'style="max-width:{w}px;height:auto" role="img">')
    bg = f'<rect class="bg" x="0" y="0" width="{w}" height="{h}"/>' if standalone else ""
    return head + style + defs + bg + body + "</svg>"


def layout_svg(place, standalone=True):
    s = 1.6
    X0, X1, Z0, Z1 = -186.0, 186.0, -201.0, 240.0
    W, H = (X1 - X0) * s + 40, (Z1 - Z0) * s + 70

    def P(x, z):
        return 20 + (x - X0) * s, 40 + (Z1 - z) * s

    b = []
    x, y = P(-183, 240)
    b.append(f'<rect class="sheet" x="{x:.1f}" y="{y:.1f}" width="{366*s:.1f}" height="{441*s:.1f}"/>')
    b.append(f'<text class="tb" x="20" y="22">Rear sheet, inner face Y -66, seen from behind the box (+X right); W mm</text>')
    for nm, (a0, a1, c0, c1) in KEEPOUT:
        x, y = P(a0, c1)
        b.append(f'<rect class="ko" x="{x:.1f}" y="{y:.1f}" width="{(a1-a0)*s:.1f}" height="{(c1-c0)*s:.1f}"/>')
        b.append(f'<text class="ts" x="{x+6:.1f}" y="{y+14:.1f}">{nm} (keep-out)</text>')
    for nm, (a0, a1, c0, c1, d, why) in FIXED.items():
        x, y = P(a0, c1)
        b.append(f'<rect class="fix" x="{x:.1f}" y="{y:.1f}" width="{(a1-a0)*s:.1f}" height="{(c1-c0)*s:.1f}" rx="3"/>')
        b.append(f'<text class="t" x="{x+5:.1f}" y="{y+15:.1f}">{nm}</text>')
    for nm in BOARDS:
        r = rect(nm, place)
        x, y = P(r[0], r[3])
        cls = "pwr" if nm.startswith(("BUCK", "FUSE", "RELAY", "INA", "BUS")) else "brd"
        b.append(f'<rect class="{cls}" x="{x:.1f}" y="{y:.1f}" width="{(r[1]-r[0])*s:.1f}" height="{(r[3]-r[2])*s:.1f}" rx="3"/>')
        b.append(f'<text class="t" x="{x+4:.1f}" y="{y+13:.1f}">{nm}</text>')
    # harness exits
    for nm, (xa, za, lab) in {"ARM": (-40, 0, "arm bundle in"), "TOP": (0, 232, "latch x4, panel probes out"),
                              "BAYS": (-160, 60, "bay leads out (side corridors)")}.items():
        x, y = P(xa, za)
        b.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="5" fill="none" stroke="currentColor" stroke-width="1.5"/>')
        b.append(f'<text class="ts" x="{x+8:.1f}" y="{y+4:.1f}">{lab}</text>')
    x, y = P(-183, -201)
    b.append(f'<text class="ts" x="{x:.1f}" y="{y+16:.1f}">floor Z -201; sheet X +-183, Z -201..240; stub face Y 0 is 66 in front of the sheet; boards on 10 nylon standoffs</text>')
    return svg_wrap("".join(b), int(W), int(H), standalone)


def schematic_svg(standalone=True):
    W, H = 1180, 820
    B = {  # name: (x, y, w, h, class, label, sub)
        "MAINS": (20, 40, 120, 50, "fix", "Mains", "IEC C14, switch + fuse"),
        "PSU": (190, 30, 150, 70, "fix", "LRS-350-24", "24 V 14.6 A"),
        "FUSE": (390, 30, 150, 70, "pwr", "Fuse box", "F1 10 A, F2 5 A, F3 3 A, F4 2 A"),
        "CHG": (620, 20, 170, 60, "fix", "Charger 2-ch", "in limit 6.5 A"),
        "ESTOP": (390, 160, 150, 40, "pwr", "E-stop (NC)", "motion only"),
        "SKR": (620, 150, 170, 60, "brd", "SKR Mini E3 V3", "Klipper, TMC2209 x3"),
        "BSV": (390, 260, 150, 50, "pwr", "Servo buck", "6.0 V, 8 A"),
        "BKA": (190, 260, 150, 50, "pwr", "Keep-alive buck", "13.0 V, CC 3.0 A"),
        "B5": (20, 260, 140, 50, "pwr", "Logic buck", "5.1 V 5 A"),
        "PI": (20, 420, 160, 70, "brd", "Raspberry Pi 4", "Klipper host + boxd"),
        "PCA": (390, 420, 150, 50, "sig", "PCA9685", "I2C 0x60, OE GPIO4"),
        "RELAY": (190, 560, 150, 60, "pwr", "Relay 4-ch", "K1 KA, K2 bay S, K3 bay O, K4 grippers"),
        "INAKA": (390, 560, 110, 40, "sig", "INA226 0x41", "keep-alive"),
        "INAS": (520, 560, 110, 40, "sig", "INA226 0x44", "bay S"),
        "INAO": (650, 560, 110, 40, "sig", "INA226 0x45", "bay O"),
        "POST": (850, 640, 160, 60, "off", "Face pins", "2 per +-x A panel (v9 plain)"),
        "BAYS": (850, 540, 160, 50, "off", "Bay -90 (Scout)", "XT60E1-M, float carrier"),
        "BAYO": (850, 470, 160, 50, "off", "Bay 180 (Overseer)", "reserved, wired later"),
        "ARM": (850, 150, 160, 120, "off", "Arm (twist bundle)", "3 NEMA17, 2 gripper servos, axle hall"),
        "LATCH": (850, 330, 160, 90, "off", "Hook latch x4", "MG90S + A3144 each"),
        "DRONE": (1040, 600, 120, 120, "off", "Drone", "ideal diode + caps"),
        "INAGR": (390, 720, 110, 40, "sig", "INA226 0x40", "gripper pair"),
        "ADC": (560, 720, 110, 40, "sig", "ADS1115 0x48", "grip halls, 3.3 V"),
    }
    b = []

    def box(k):
        x, y, w, h, c, lab, sub = B[k]
        b.append(f'<rect class="{c}" x="{x}" y="{y}" width="{w}" height="{h}" rx="5"/>')
        b.append(f'<text class="tb" x="{x+8}" y="{y+18}">{lab}</text><text class="ts" x="{x+8}" y="{y+33}">{sub}</text>')

    def wire(cls, pts, lab=None, at=0):
        d = "M" + " L".join(f"{x},{y}" for x, y in pts)
        b.append(f'<path class="{cls}" d="{d}"/>')
        if lab:
            x, y = pts[at]
            b.append(f'<text class="ts" x="{x+4}" y="{y-4}">{lab}</text>')

    wire("wm", [(140, 65), (190, 65)], "18 AWG", 0)
    wire("w24", [(340, 65), (390, 65)], "14 AWG", 0)
    wire("w24", [(540, 50), (620, 50)], "F1 16 AWG", 0)
    wire("w24", [(465, 100), (465, 160)], "F2 18", 0)
    wire("w24", [(540, 180), (620, 180)], "18 AWG", 0)
    wire("w24", [(465, 200), (465, 260)], "18", 0)
    wire("w24", [(420, 100), (420, 130), (265, 130), (265, 260)], "F3 18", 2)
    wire("w24", [(410, 100), (410, 120), (90, 120), (90, 260)], "F4 20", 2)
    wire("w5", [(90, 310), (90, 420)], "5.1 V 20 AWG", 0)
    wire("w5", [(130, 310), (130, 380), (230, 380), (230, 560)], "5 V JD-VCC", 1)
    wire("w6", [(540, 285), (580, 285), (580, 445), (540, 445)], "6.0 V 18 AWG", 0)
    wire("w13", [(265, 310), (265, 560)], "13.0 V 18 AWG", 0)
    wire("w13", [(340, 580), (390, 580)], None)
    wire("w13", [(445, 600), (445, 670), (850, 670)], "pad + 18 AWG, pad - to buck -", 1)
    wire("wch", [(790, 40), (820, 40), (820, 530), (360, 530), (360, 600), (340, 600)], "CH1 + / CH2 + 16 AWG", 3)
    wire("wch", [(340, 610), (370, 610), (370, 625), (575, 625), (575, 600)], None)
    wire("wch", [(340, 615), (380, 615), (380, 632), (705, 632), (705, 600)], None)
    wire("wch", [(630, 570), (850, 570)], "bay S + 16 AWG", 1)
    wire("wch", [(760, 575), (780, 575), (780, 495), (850, 495)], "bay O +", 2)
    wire("wch", [(800, 80), (800, 500), (850, 510)], "CH - direct to the bays (never box GND)", 0)
    wire("wst", [(790, 190), (850, 190)], "12 x 26 AWG", 0)
    wire("wsg", [(180, 455), (390, 455)], "I2C", 0)
    wire("wsg", [(540, 460), (560, 460), (560, 520), (445, 520), (445, 560)], None)
    wire("wsg", [(500, 580), (520, 580)], None)
    wire("wsg", [(630, 585), (650, 585)], None)
    wire("wsg", [(100, 490), (100, 590), (190, 590)], "GPIO 5..8 (IN, active low)", 1)
    wire("wsg", [(540, 430), (760, 430), (760, 360), (850, 360)], "PWM ch 0..3 + 22 AWG servo leads", 1)
    wire("wsg", [(540, 440), (720, 440), (720, 230), (850, 230)], "PWM ch 4, 5", 2)
    wire("wsg", [(180, 470), (300, 470), (300, 500), (830, 500), (830, 400), (850, 400)], "halls GPIO 17, 27, 22, 23", 2)
    wire("wsg", [(150, 490), (150, 700), (850, 700)], "seat sense GPIO 24 (1k, BAT54)", 1)
    wire("wsg", [(180, 430), (300, 430), (300, 220), (620, 220), (620, 205)], "USB", 1)
    wire("wsg", [(790, 200), (850, 205)], None)
    wire("wch", [(1010, 670), (1040, 670)], None)
    wire("w6", [(300, 620), (300, 740), (390, 740)], "K4: 6.0 V gripper pair (twist), stall cut", 1)
    wire("w6", [(500, 740), (520, 740), (520, 712), (820, 712), (820, 262), (850, 262)], None)
    wire("wsg", [(670, 740), (835, 740), (835, 268), (850, 268)], "grip halls: H per head + 3.3 V (DRV5055 x2 per head)", 0)
    wire("wsg", [(180, 480), (200, 480), (200, 765), (615, 765), (615, 760)], "I2C to 0x40, 0x48", 1)
    for k in B:
        box(k)
    b.append('<text class="tb" x="20" y="22">Box electronics v1: power and signal (C / E, DEFAULT)</text>')
    leg = [("wm", "mains"), ("w24", "24 V"), ("w13", "13.0 V keep-alive"), ("w6", "6.0 V servo"), ("w5", "5.1 V logic"),
           ("wch", "charge"), ("wst", "stepper"), ("wsg", "signal")]
    for i, (c, t) in enumerate(leg):
        x = 20 + i * 140
        b.append(f'<path class="{c}" d="M{x},800 L{x+30},800"/><text class="ts" x="{x+36}" y="804">{t}</text>')
    return svg_wrap("".join(b), W, H, standalone)


def states_svg(standalone=True):
    W, rowh = 1000, 46
    H = 60 + rowh * len(STATES) + 20
    b = ['<text class="tb" x="20" y="24">Swap state machine (boxd on the Pi; DEFAULT, times E)</text>']
    for i, (st, act, ok, tmax, fail) in enumerate(STATES):
        y = 44 + i * rowh
        cls = "sig" if st not in ("STORE", "BAY_MATE", "CHARGE", "CHARGE_END") else "brd"
        b.append(f'<rect class="{cls}" x="20" y="{y}" width="150" height="34" rx="5"/>')
        b.append(f'<text class="tb" x="30" y="{y+21}">{st}</text>')
        a = act if len(act) < 96 else act[:93] + "..."
        b.append(f'<text class="ts" x="182" y="{y+13}">{a}</text>')
        o = ("pass: " + ok) if len(ok) < 92 else "pass: " + ok[:89] + "..."
        b.append(f'<text class="ts" x="182" y="{y+27}">{o}</text>')
        if fail:
            b.append(f'<path class="ab" d="M770,{y+17} L800,{y+17}"/><text class="ts" x="806" y="{y+21}">{fail}{"" if tmax is None else f" ({tmax:g} s)"}</text>')
        if i < len(STATES) - 1:
            b.append(f'<path class="arr" d="M95,{y+34} L95,{y+rowh-1}"/>')
    return svg_wrap("".join(b), W, H, standalone)


def brake_svg(standalone=True):
    """head / axle brake (one of 3 identical DPDT channels) and the keep-alive + seat-sense circuit, as wired."""
    W, H = 1180, 820
    b = ['<text class="tb" x="20" y="24">Brake: one DPDT relay per motor (KB1 head A, KB2 head B, KB3 axle), drawn de-energised = braked</text>']

    def box(x, y, w, h, c, lab, sub=""):
        b.append(f'<rect class="{c}" x="{x}" y="{y}" width="{w}" height="{h}" rx="5"/>')
        b.append(f'<text class="tb" x="{x+8}" y="{y+18}">{lab}</text>')
        if sub:
            b.append(f'<text class="ts" x="{x+8}" y="{y+33}">{sub}</text>')

    def wire(cls, pts, lab=None, at=0, dx=4, dy=-4):
        b.append(f'<path class="{cls}" d="M' + " L".join(f"{x},{y}" for x, y in pts) + '"/>')
        if lab:
            x, y = pts[at]
            b.append(f'<text class="ts" x="{x+dx}" y="{y+dy}">{lab}</text>')

    def dot(x, y):
        b.append(f'<circle cx="{x}" cy="{y}" r="3.2" fill="currentColor"/>')

    def contact(x, y, lab):
        # common at (x, y); NO up-right at (x+60, y-22), NC down-right at (x+60, y+22); blade rests on NC
        for cy, t in ((y - 22, "NO"), (y + 22, "NC")):
            b.append(f'<circle cx="{x+60}" cy="{cy}" r="3" fill="none" stroke="currentColor" stroke-width="1.3"/>')
            b.append(f'<text class="ts" x="{x+66}" y="{cy+4}">{t}</text>')
        b.append(f'<circle cx="{x}" cy="{y}" r="3" fill="none" stroke="currentColor" stroke-width="1.3"/>')
        b.append(f'<path class="wst" d="M{x+3},{y+1} L{x+57},{y+20}"/>')
        b.append(f'<text class="ts" x="{x-16}" y="{y-10}">{lab}</text>')

    # driver, relay poles, motor
    box(30, 60, 170, 250, "brd", "TMC2209 (SKR)", "head A shown; B, axle same")
    for i, (nm, y) in enumerate((("A1 (out 1)", 110), ("A2 (out 2)", 160), ("B1 (out 1)", 220), ("B2 (out 2)", 270))):
        b.append(f'<text class="ts" x="120" y="{y+4}">{nm}</text>')
        dot(200, y)
    box(330, 60, 260, 250, "pwr", "KB1: G2RL-2 DC24 (DPDT, 8 A)", "pole 1 = coil A, pole 2 = coil B")
    contact(380, 140, "pole 1 COM")
    contact(380, 250, "pole 2 COM")
    box(760, 60, 190, 250, "off", "NEMA17 (head A)", "own pigtail, JST-XH inline")
    b.append('<path class="wst" d="M850,120 c 30,0 30,20 0,20 c 30,0 30,20 0,20 c 30,0 30,20 0,20"/>')
    b.append('<text class="ts" x="890" y="155">coil A</text>')
    b.append('<path class="wst" d="M850,230 c 30,0 30,20 0,20 c 30,0 30,20 0,20 c 30,0 30,20 0,20"/>')
    b.append('<text class="ts" x="890" y="265">coil B</text>')
    # coil A: out1 -> NO, COM -> coil wire 1, coil wire 2 -> out2 and -> NC
    wire("wst", [(200, 110), (300, 110), (300, 118), (440, 118)], "driver out 1 to NO", 0, 8, -6)
    wire("wst", [(380, 140), (350, 140), (350, 96), (700, 96), (700, 120), (850, 120)], "COM to coil wire 1", 3, -140, -6)
    wire("wst", [(440, 162), (620, 162), (620, 180), (850, 180)], "NC to coil wire 2", 0, 30, 18)
    wire("wst", [(200, 160), (260, 160), (260, 196), (660, 196), (660, 180)], "driver out 2 to coil wire 2 (always)", 1, 8, 14)
    dot(620, 180); dot(660, 180)
    wire("wst", [(200, 220), (300, 220), (300, 228), (440, 228)], None)
    wire("wst", [(380, 250), (350, 250), (350, 206), (700, 206), (700, 230), (850, 230)], None)
    wire("wst", [(440, 272), (620, 272), (620, 290), (850, 290)], None)
    wire("wst", [(200, 270), (260, 270), (260, 300), (660, 300), (660, 290)], None)
    dot(620, 290); dot(660, 290)
    # relay coil drive
    box(330, 360, 260, 80, "sig", "ULN2003A (bare DIP)", "COM pin 9 to 24 V: the flyback diodes")
    box(30, 360, 170, 80, "brd", "SKR EXP1 pin", "[output_pin head_a_brake] (CHECK pins)")
    wire("wsg", [(200, 400), (330, 400)], "high = released", 0, 8, -6)
    wire("w24", [(700, 380), (700, 340), (470, 340), (470, 310)], "24 V to the relay coil + and ULN COM", 0, 6, 16)
    wire("w24", [(700, 380), (590, 380)])
    wire("wsg", [(590, 420), (620, 420), (620, 330), (540, 330), (540, 310)], "OUT1 sinks the relay coil -", 0, 6, 16)
    b.append('<text class="ts" x="30" y="476">De-energised (pin low, power off, E-stop, Klipper shutdown): out 1 is open on NO and each coil is shorted on itself through NC, so the motor brakes and the driver is never shorted.</text>')
    b.append('<text class="ts" x="30" y="494">Start: brake pin high, dwell 30 ms, MANUAL_STEPPER ENABLE=1, move. Stop: ENABLE=0, dwell 30 ms, pin low; re-enable only after a further 50 ms (NC closes ~35 ms, E).</text>')

    # keep-alive + seat sense
    y0 = 540
    b.append(f'<text class="tb" x="20" y="{y0}">Keep-alive and seat sense (panel v9 plain pins on the drone\'s lower-ring pads; runs/keepalive_contacts/concept)</text>')
    box(30, y0 + 20, 140, 60, "pwr", "Keep-alive buck", "13.0 V CV, 3.0 A CC")
    box(210, y0 + 20, 90, 60, "pwr", "K1", "relay ch 1")
    box(340, y0 + 20, 110, 60, "sig", "INA226 0x41", "R010 shunt")
    box(700, y0 + 10, 150, 80, "off", "+x face: 2 pins", "B+, panel v9 plain")
    box(700, y0 + 150, 150, 80, "off", "-x face: 2 pins", "B- = box GND")
    box(960, y0 + 10, 200, 220, "off", "Drone", "")
    b.append(f'<text class="ts" x="968" y="{y0+50}">+ pad (lower ring, 22 x 12)</text>')
    b.append(f'<text class="ts" x="968" y="{y0+80}">ideal diode (LM74700) to bus</text>')
    b.append(f'<text class="ts" x="968" y="{y0+110}">10 kOhm across the pads</text>')
    b.append(f'<text class="ts" x="968" y="{y0+200}">- pad</text>')
    wire("wst", [(1060, y0 + 50), (1060, y0 + 120)])
    wire("wst", [(1060, y0 + 180), (1060, y0 + 190)])
    wire("w13", [(170, y0 + 50), (210, y0 + 50)])
    wire("w13", [(300, y0 + 50), (340, y0 + 50)])
    wire("w13", [(450, y0 + 50), (700, y0 + 50)], "B+ 18 AWG, bullet at the sheet", 0, 50, -6)
    wire("w13", [(850, y0 + 50), (960, y0 + 50)], "spring pins", 0, 8, -6)
    wire("w13", [(850, y0 + 190), (960, y0 + 190)], None)
    wire("w13", [(700, y0 + 190), (100, y0 + 190), (100, y0 + 80)], "B- 18 AWG to the buck -", 1, 8, -6)
    b.append(f'<path class="wst" d="M1060,{y0+120} L1060,{y0+135} l -6,4 l 12,6 l -12,6 l 12,6 l -12,6 l 6,4 L1060,{y0+180}"/>')
    # seat-sense branch
    dot(560, y0 + 50)
    wire("wsg", [(560, y0 + 50), (560, y0 + 100)], "BAT54 (cathode up)", 1, 8, 0)
    b.append(f'<path d="M552,{y0+100} L568,{y0+100} L560,{y0+88} z" fill="currentColor"/>')
    wire("wsg", [(560, y0 + 100), (560, y0 + 130), (480, y0 + 130)], "1 kOhm", 2, 0, -6)
    wire("wsg", [(480, y0 + 130), (380, y0 + 130)], None)
    dot(520, y0 + 130)
    wire("wsg", [(520, y0 + 130), (520, y0 + 160)], "5 V TVS, &#8804; 1 uA at 3.3 V", 1, 8, 4)
    box(220, y0 + 110, 160, 60, "brd", "Pi GPIO 24", "100 kOhm to 3.3 V")
    dot(620, y0 + 50)
    wire("w13", [(620, y0 + 50), (620, y0 + 110)], "SMBJ15A to GND", 1, 6, 4)
    b.append(f'<text class="ts" x="30" y="{y0+262}">K1 open, drone seated: 3.3 V, 100 kOhm, 1 kOhm, BAT54 into the drone\'s 10 kOhm: GPIO 24 ~0.52 V = low (E, CHECK). No drone: high. K1 closed: the line is at 13.0 V, the BAT54 blocks, GPIO reads high and boxd ignores it.</text>')
    return svg_wrap("".join(b), W, H, standalone)


HTML = """<title>Box Electronics</title>
<meta name="claude:viewport-fit-sides" content="cover">
<meta name="description" content="Nox box electronics v1: power, I/O, rear-sheet layout and the swap state machine.">
<style>
:root{--bg:#ffffff;--ink:#1d2730;--muted:#55626e;--line:#8a97a3;--sheet:#eef3f6;--ko:#e2e2e2;--fix:#dfe8f0;--brd:#fff4d8;
--pwr:#ffe1dc;--sig:#e1f0e4;--off:#ececf6;--c24:#c0392b;--c13:#d35400;--c6:#8e44ad;--c5:#2471a3;--cch:#b7950b;--csg:#1e8449;
--cst:#34495e;--cm:#7b241c;color-scheme:light dark}
@media (prefers-color-scheme: dark){:root:not([data-theme="light"]){--bg:#12171c;--ink:#e6edf3;--muted:#9aa7b3;--line:#5b6875;
--sheet:#1b2630;--ko:#2a2f35;--fix:#1f2d3a;--brd:#3a3220;--pwr:#3d2522;--sig:#1f3324;--off:#262638;--c24:#ff6b5b;--c13:#ff9f43;
--c6:#c39bd3;--c5:#5dade2;--cch:#f4d03f;--csg:#58d68d;--cst:#aab7c4;--cm:#e57373}}
:root[data-theme="dark"]{--bg:#12171c;--ink:#e6edf3;--muted:#9aa7b3;--line:#5b6875;--sheet:#1b2630;--ko:#2a2f35;--fix:#1f2d3a;
--brd:#3a3220;--pwr:#3d2522;--sig:#1f3324;--off:#262638;--c24:#ff6b5b;--c13:#ff9f43;--c6:#c39bd3;--c5:#5dade2;--cch:#f4d03f;
--csg:#58d68d;--cst:#aab7c4;--cm:#e57373}
body{background:var(--bg);color:var(--ink);font:15px/1.45 system-ui,sans-serif;margin:0;padding-block:16px;padding-left:max(16px, env(safe-area-inset-left, 0px));padding-right:max(16px, env(safe-area-inset-right, 0px))}
main{max-width:1180px;margin:0 auto} h1{font-size:22px;margin:4px 0 8px} h2{font-size:17px;margin:26px 0 8px}
p,li{color:var(--ink)} .muted{color:var(--muted)} .fig{overflow-x:auto;border:1px solid var(--line);border-radius:8px;padding:6px}
table{border-collapse:collapse;width:100%;font-size:13px} td,th{border-bottom:1px solid var(--line);padding:4px 6px;text-align:left;vertical-align:top}
.wrap{overflow-x:auto} svg{color:var(--ink)}
""" + STYLE + """
</style><main>
<h1>Box electronics v1</h1>
<p class="muted">24 Oct 2026. Doc <code>nox/box/box-electronics.md</code>, numbers <code>nox/box/box_electronics.py</code>. C unless tagged; E = estimate; DEFAULT = chosen without Spark.</p>
<h2>Power and signal</h2><div class="fig">__SCHEM__</div>
<h2>Head and axle brake, keep-alive and seat sense (wiring)</h2><div class="fig">__BRAKE__</div>
<h2>Rear sheet layout (placement search)</h2><div class="fig">__LAYOUT__</div>
<h2>Swap state machine</h2><div class="fig">__STATES__</div>
<h2>Aborts</h2><div class="wrap"><table><tr><th>Abort</th><th>Trigger</th><th>Action</th></tr>__ABORTS__</table></div>
<h2>Power budget (E)</h2><div class="wrap"><table><tr><th>Load</th><th>Rail</th><th>W cont</th><th>W peak</th><th>at 24 V peak</th></tr>__BUDGET__</table></div>
<h2>Nets and cut list</h2><div class="wrap"><table><tr><th>Net</th><th>From</th><th>To</th><th>n</th><th>AWG</th><th>I peak A</th><th>route mm</th><th>cut mm</th><th>drop peak</th></tr>__NETS__</table></div>
</main>
"""


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    runs = []
    for seed in range(12):
        pl, c = search(seed)
        runs.append((c, seed, pl))
    runs.sort(key=lambda t: t[0])
    c, seed, place = runs[0]
    assert violations(place) == 0, "no legal placement"
    nets = nets_table(place)
    bud = budget()
    # checks (reported)
    depth_ok = all(SHEET_Y + STANDOFF + BOARDS[n][2] <= DEPTH_MAX for n in BOARDS) and \
        all(SHEET_Y + f[4] <= DEPTH_MAX for f in FIXED.values())
    amp_fail = [r["net"] for r in nets if not r["ampacity_ok"]]
    drop_flags = [r["net"] for r in nets if r["drop"] and (r["drop"]["cont_pct"] > 2.0 or r["drop"]["peak_pct"] > 10.0)]
    # min gap between any two bodies on the sheet
    allr = {n: rect(n, place) for n in BOARDS}
    allr.update({k: v[:4] for k, v in FIXED.items()})
    allr.update({k: v for k, v in KEEPOUT})

    def gap(r1, r2):
        dx = max(r2[0] - r1[1], r1[0] - r2[1], 0)
        dz = max(r2[2] - r1[3], r1[2] - r2[3], 0)
        return math.hypot(dx, dz) if dx and dz else max(dx, dz)

    names = list(allr)
    gmin = min((gap(allr[a], allr[b]), a, b) for i, a in enumerate(names) for b in names[i + 1:]
               if a in BOARDS or b in BOARDS)   # board to anything; fixed vs keep-out is drawn, not searched
    totals = {}
    for r in nets:
        if r["awg"]:
            totals[r["awg"]] = totals.get(r["awg"], 0) + r["cut_mm"] * r["conductors"]
    # arm-side conductors are already counted in box-wiring.md's cut list: report the fixed-tail extra separately
    result = dict(
        date="2026-10-24 18:00", status="DONE", part="box_electronics", version="v1",
        note="design + docs (no printed part); placement search on the rear sheet, nets, budget, state machine",
        solid_ok=None, mass_g=None,
        placement={n: dict(x=round(place[n][0], 1), z=round(place[n][1], 1), turned=bool(place[n][2]),
                           rect=[round(v, 1) for v in rect(n, place)]) for n in BOARDS},
        search=dict(seeds=12, best_seed=seed, best_cost=round(c), costs=[round(r[0]) for r in runs],
                    objective="sum over nets of conductors x (0.25 + I_cont) x route length (DEFAULT)"),
        fixed={k: dict(rect=list(v[:4]), depth=v[4], why=v[5]) for k, v in FIXED.items()},
        endpoints={k: dict(pos=list(v[0]), detour_E=v[1], route=v[2]) for k, v in ENDPOINTS.items()},
        checks=dict(depth_ok=depth_ok, min_gap_mm=round(gmin[0], 1), min_gap_pair=[gmin[1], gmin[2]],
                    ampacity_fail=amp_fail, drop_flags=drop_flags),
        nets=nets, wire_totals_mm_by_awg=totals, budget=bud, states=STATES, aborts=ABORTS,
    )
    (OUT / "result.json").write_text(json.dumps(result, indent=1))
    lay, sch, sta = layout_svg(place), schematic_svg(), states_svg()
    (OUT / "layout.svg").write_text(lay)
    (OUT / "schematic.svg").write_text(sch)
    (OUT / "brake.svg").write_text(brake_svg())
    (OUT / "states.svg").write_text(sta)
    ab = "".join(f"<tr><td>{a}</td><td>{t}</td><td>{x}</td></tr>" for a, t, x in ABORTS)
    bu = "".join(f"<tr><td>{r['load']}</td><td>{r['rail']}</td><td>{r['W_cont']}</td><td>{r['W_peak']}</td><td>{r['W_at_24V_peak']}</td></tr>"
                 for r in bud["rows"])
    bu += f"<tr><th>Total</th><th></th><th>{bud['total_cont_W']}</th><th></th><th>{bud['total_peak_W']} = {bud['peak_pct']} % of 350 W</th></tr>"
    nt = "".join(f"<tr><td>{r['net']}</td><td>{r['a']}</td><td>{r['b']}</td><td>{r['conductors']}</td><td>{r['awg'] or 'cable'}</td>"
                 f"<td>{r['I_peak']}</td><td>{r['route_mm']}</td><td>{r['cut_mm']}</td>"
                 f"<td>{'' if not r['drop'] else str(r['drop']['peak_V']) + ' V (' + str(r['drop']['peak_pct']) + ' %)'}</td></tr>" for r in nets)
    html = (HTML.replace("__SCHEM__", schematic_svg(False)).replace("__LAYOUT__", layout_svg(place, False))
            .replace("__STATES__", states_svg(False)).replace("__BRAKE__", brake_svg(False)).replace("__ABORTS__", ab).replace("__BUDGET__", bu).replace("__NETS__", nt))
    (OUT / "diagram.html").write_text(html)
    print(f"best seed {seed} cost {c:.0f}; costs {[round(r[0]) for r in runs]}")
    for n in BOARDS:
        print(f"  {n:8s} x {place[n][0]:7.1f} z {place[n][1]:7.1f} {'turned' if place[n][2] else ''}")
    print(f"depth ok {depth_ok}; min gap {gmin[0]:.1f} ({gmin[1]}-{gmin[2]}); ampacity fail {amp_fail}; drop flags {drop_flags}")
    print(f"budget cont {bud['total_cont_W']} W, peak {bud['total_peak_W']} W ({bud['peak_pct']} %, {bud['peak_A']} A)")
    for r in nets:
        d = r["drop"]
        print(f"  {r['net']:22s} {r['conductors']:2d} x {r['awg'] or '--':>3} route {r['route_mm']:5d} cut {r['cut_mm']:5d}"
              + (f"  drop {d['cont_pct']:.2f} / {d['peak_pct']:.2f} %" if d else "")
              + (f"  fixed tail {r['fixed_tail_mm']}" if r["fixed_tail_mm"] else ""))
    print("wire totals mm by AWG:", totals)


if __name__ == "__main__":
    main()
