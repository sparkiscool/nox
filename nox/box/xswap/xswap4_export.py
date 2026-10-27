"""Export xswap v4 print files and strength-test coupons (print orientation, z min = bed). python3 xswap4_export.py"""
import os
import numpy as np
import xswap4 as X

STL = os.path.join(X.OUT, "stl"); os.makedirs(STL, exist_ok=True)
CPN = os.path.join(X.OUT, "coupons"); os.makedirs(CPN, exist_ok=True)

def to_bed(m, rot):
    m = m.rotate(rot) if rot else m
    b = m.bounding_box()
    return m.translate([-(b[0] + b[3]) / 2, -(b[1] + b[4]) / 2, -b[2]])

PRINTS = {
    "xswap_shell_v4": (X.shell_body(), [90, 0, 0], "1 per pack: tail end down, tree supports"),
    "xswap_bar_v4": (X.bar_local(), None, "2 per pack (same part both ends): flat, slot axis vertical"),
    "xswap_xt60_keeper_v4": (X.xt60_keeper(), None, "1 per pack: flat"),
    "xswap_holder_v4": (X.holder(), [180, 0, 0], "1 per drone + 1 per bay: mount face down, legs up"),
    "xswap_xt60_cap_v4": (X.xt60_cap(), [180, 0, 0], "1 per holder: plate down"),
    "xswap_finger_v4": (X.finger_A(0.0), None, "2 (same part both sides): rack bottom down, post up"),
    "xswap_pinion_v4": (X.pinion(), [180, 0, 0], "1: tooth face down"),
    "xswap_jaw_v4": (X.jaw_body(), None, "1: bottom down, tree supports"),
}

# coupons: the +y latch end of the shell (tray + band + slot + spring boss + pivot), the matching holder leg with a slab
# of deck, plus a real bar: pull-test the tip on the ledge, cycle the bar, without printing the whole shell / holder.
def coupons():
    sh = X.shell_body() ^ X.box(-40, 40, 26.0, 95.0, X.SZ0 - 1, X.Z_FL0 + 0.01)        # floor plate kept as the coupon top
    lg = X.holder() ^ X.box(-40, 40, 70.0, 100.0, X.LEG_Z0 - 1, X.DK_T + 1)
    return {"coupon_shell_end_v4": (sh, [180, 0, 0], "floor plate down"),
            "coupon_holder_leg_v4": (lg, [180, 0, 0], "deck slab down, leg up")}

if __name__ == "__main__":
    for name, (m, rot, note) in PRINTS.items():
        t = X.tm(to_bed(m, rot)); t.export(os.path.join(STL, name + ".stl"))
        print(name, np.round(t.extents, 1), "%.1f g solid" % (t.volume * X.PETG), "|", note)
    for name, (m, rot, note) in coupons().items():
        t = X.tm(to_bed(m, rot)); t.export(os.path.join(CPN, name + ".stl"))
        print(name, np.round(t.extents, 1), "%.1f g solid" % (t.volume * X.PETG), "|", note)
