"""Export xswap v2 print files (print orientation, z min = bed) and the assembled-pose meshes. python3 xswap_export.py"""
import os
import numpy as np
import xswap as X

STL = os.path.join(X.OUT, "stl"); os.makedirs(STL, exist_ok=True)
POSE = os.path.join(X.OUT, "pose"); os.makedirs(POSE, exist_ok=True)
YC = X.Y_C

def to_bed(m, rot):
    m = m.rotate(rot) if rot else m
    b = m.bounding_box()
    return m.translate([-(b[0] + b[3]) / 2, -(b[1] + b[4]) / 2, -b[2]])

PRINTS = {   # name: (part, rotation to print pose, count, note)
    "xswap_shell_v2": (X.shell_body(YC), [90, 0, 0], 1, "tail end down, tree supports (ears, sockets, head roof)"),
    "xswap_rocker_v2": (X.rocker_local(YC), [90, 0, 0], 2, "on its side (y face down); print 2, same part both sides"),
    "xswap_xt60_keeper_v2": (X.xt60_keeper(), None, 1, "flat"),
    "xswap_holder_v2": (X.holder(YC), [180, 0, 0], 1, "mount face down (rib tops), skirt up; print 1 per drone + 1 per bay"),
    "xswap_xt60_cap_v2": (X.xt60_cap(), [180, 0, 0], 1, "plate down; 1 per holder"),
    "xswap_finger_v2": (X.pincer(YC, X.FACE_OPEN)["finger_p"], None, 2, "rack bottom down; print 2 (same part both sides)"),
    "xswap_pinion_v2": (X.pinion(YC), [180, 0, 0], 1, "tooth face down, horn recess up"),
    "xswap_jaw_v2": (X.jaw_body(YC), None, 1, "bottom down, tree supports (hubs, tunnels)"),
}

if __name__ == "__main__":
    for name, (m, rot, n, note) in PRINTS.items():
        t = X.tm(to_bed(m, rot))
        t.export(os.path.join(STL, name + ".stl"))
        print(name, "x%d" % n, np.round(t.extents, 1), "%.1f g solid" % (t.volume * X.PETG))
    # assembled meshes in the drone frame for renders (shell seated, pincer shut)
    pose = dict(holder=X.holder(YC), cap=X.xt60_cap(), xt60_male=X.xt60_male(), shell=X.shell(YC, X.TH_OPEN),
                keeper=X.xt60_keeper(), xt60_female=X.xt60_female(), pack=X.pack(), **X.pincer(YC, X.FACE_SHUT))
    for k, m in pose.items():
        X.tm(m).export(os.path.join(POSE, k + ".stl"))
