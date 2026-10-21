"""Arm harness v1 (17 Oct 2026, wiring chat). Everything on the arm is fed through ONE bundle that
twists inside the fixed sun post's bore (box-arm.md Problem 2: bounded travel, no slip ring).
Fixed side: electronics bay (Y -60) -> stub grommet (stub bore Ø24, clamp at Y ~ -10) -> post bore
Ø8.5 -> spigot / 80T bore Ø12 -> exits on axis at Y 72. Rotating side: central saddle of the end
clip (ON AXIS, Y ~82: the twist is pure torsion, no wrap round the exit lip) -> fans out.
All numbers C from arm_layout.py geometry unless tagged E.

Loads on the arm (the old 15-wire count missed the head servos):
  3x NEMA17 (A, B, axle)      12 x 26 AWG silicone        (2 A/phase set, low duty: detent holds
                                                            with drivers OFF; derate note below)
  2x head servo (DS3218/MG995) 1 shared 6.8 V pair 22 AWG + 2 signal 28 AWG  (servos never both
                                                            stall in the cycle, E)
  1x hall A3144 (homing)       3 x 28 AWG
  => 19 wires through the twist.

Servo branch per head: 3-wire ribbon (V/G 26 AWG + S 28 AWG) recessed in the bar's TOP-face
T-slot at X -10 (away from the belt; the bottom slots are taken by the bar posts' M5 slot nuts).
Moving slack = the ribbon folds back on itself INSIDE the slot (printer-carriage style): anchor at
the end clip, fold slides in the slot, exits up through the glide (entry channel = head v5 feature).
The whole run lives inside the glide's own swept tube, so it can never meet anything the sweep
hasn't already cleared the head against.
"""
import math

# ---- wire set (OD mm: thin-wall silicone, E) ----
OD = {"26": 1.30, "22": 1.60, "28": 1.13}
BUNDLE = [("stepper", "26", 12), ("servo_pwr", "22", 2), ("servo_sig", "28", 2), ("hall", "28", 3)]
FILL = 0.60                     # round-wire loose-bundle fill (E)
BORE_D = 8.5                    # post bore (arm_layout POST_BORE_R*2)
TRAVEL = 421.0                  # soft limits -278..+143 (box-arm.md)
L_TWIST = 90.0                  # grommet Y -10 .. saddle Y ~80

# ---- slot fold (per head) ----
SLOT_D, SLOT_W = 6.0, 6.0       # 2040 T-slot depth/opening (E)
RIB_T, RIB_W = 1.3, 3.9         # 3-wire ribbon thickness / width (26+26+28)
Z0 = (32.0, 89.6)               # glide inner end: park .. drone
E_ENTRY = 8.0                   # glide entry channel above the glide's inner end (head v5 feature)
R_ANCHOR = 23.0                 # ribbon anchor on the end clip
IDLER_CLAMP_R0 = 124.0          # idler-end M5 slot nuts start (E): slot must be free below this


def bundle_od():
    area = sum(n * math.pi * (OD[g] / 2) ** 2 for _, g, n in BUNDLE)
    d = 2 * math.sqrt(area / FILL / math.pi)
    return dict(n=sum(n for *_, n in BUNDLE), area=round(area, 1), od=round(d, 2),
                clear=round(BORE_D - d, 2))


def twist_strain(od):
    a = od / 2 - OD["26"] / 2                      # outer wire centre radius
    th = math.radians(TRAVEL / 2)                  # tie at mid-range
    return (math.hypot(L_TWIST, a * th) - L_TWIST) / L_TWIST


def slot_fold():
    x0, x1 = Z0[0] + E_ENTRY, Z0[1] + E_ENTRY      # entry hole range 40 .. 97.6
    Lw = (x1 - R_ANCHOR) + 6.0                      # in-slot ribbon length (+ fold allowance)
    f0 = (Lw + R_ANCHOR + x0) / 2                   # fold at park
    f1 = (Lw + R_ANCHOR + x1) / 2                   # fold at drone (max reach)
    return dict(Lw=round(Lw, 1), fold_park=round(f0, 1), fold_drone=round(f1, 1),
                fold_margin_to_idler=round(IDLER_CLAMP_R0 - f1, 1),
                stack=2 * RIB_T + 0.8, under_mouth=round(SLOT_D - (2 * RIB_T + 0.8), 1))


def cut_list():
    """per-conductor cut lengths, mm (E, +20% slack included where noted)."""
    twist = L_TWIST + 20                            # grommet .. saddle, service
    fixed = 250                                     # SKR / buck .. grommet (E, bay layout open)
    drop = 35                                       # saddle Y86 .. plate top Y50 at r 32, 135 deg lane
    hub_arc = {"A": 46 * math.radians(165), "B": 46 * math.radians(93), "ax": 46 * math.radians(105)}
    down = 45                                       # plate .. motor lead exit (Y ~8)
    out = {}
    for m, arc in hub_arc.items():
        out[f"stepper_{m} x4 26AWG"] = round((fixed + twist + drop + arc + down) * 1.2)
    out["hall x3 28AWG"] = round((fixed + twist + drop + 46 * math.radians(165) * 0.6 + 20) * 1.2)
    rib = (R_ANCHOR - 14) + slot_fold()["Lw"] + 110   # saddle..anchor + slot + glide..servo plate
    out["servo ribbon x2 (V26/G26/S28)"] = round((rib) * 1.2)
    out["servo pwr pair 22AWG (bay..saddle)"] = round((fixed + twist) * 1.2)
    out["servo sig x2 28AWG (bay..saddle)"] = round((fixed + twist) * 1.2)
    return out


if __name__ == "__main__":
    b = bundle_od()
    print("bundle:", b, " twist strain %:", round(100 * twist_strain(b["od"]), 2))
    print("slot fold:", slot_fold())
    for k, v in cut_list().items():
        print(f"  {k:38s} {v} mm")
