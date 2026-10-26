"""Synthetic cases for system_check.py: cd nox && python3 test_system_check.py"""
import json, os, tempfile, numpy as np, trimesh
from system_check import run, main


def shell(a, t):
    """hollow cube cage: outer half-size a, wall t, centred on the origin."""
    o = trimesh.creation.box([2 * a] * 3); i = trimesh.creation.box([2 * (a - t)] * 3)
    return trimesh.boolean.difference([o, i])


CAGE = {"name": "cage", "trimesh": shell(50, 4), "role": "cage", "material": "PETG"}


def sysm(*parts, **kw):
    return dict(name="t", parts=[CAGE, *parts], n_dirs=1000, **kw)


def test_box_in_cage():
    r = run(sysm({"name": "pack", "box": [-40, 40, -40, 40, -40, 40], "role": "protected", "mass_g": 100}))
    cf = r["cage_first"]
    assert r["pass"], r["summary"]
    assert abs(cf["margin_mm"] - 10.0) < 1e-6, cf                          # gap = 50 - 40 on every face
    assert r["fit"]["pass_"] and r["fit"]["worst"] is None


def test_sphere_pokes_out():
    sp = {"name": "cam", "sphere": {"r": 10, "center": [45, 0, 0]}, "role": "protected", "mass_g": 5}
    r = run(sysm(sp))
    cf = r["cage_first"]
    assert not r["pass"] and not cf["pass_"]
    assert cf["worst_part"] == "cam" and cf["worst_dir_label"] == "+X", cf
    assert abs(cf["margin_mm"] - (-5.0)) < 0.1, cf
    assert not r["fit"]["pass_"]                                            # it also cuts through the +X wall
    # an opening only lets a part sit recessed (margin >= 0): poking out still fails
    r2 = run(sysm(sp, openings=[{"part": "cam", "axis": [1, 0, 0], "half_angle_deg": 30}]))
    assert not r2["cage_first"]["pass_"]


def test_opening_exempts_recess():
    sp = {"name": "cart", "sphere": {"r": 10, "center": [38.5, 0, 0]}, "role": "protected", "mass_g": 5}   # h(+X) 48.5: 1.5 behind, under 3
    r = run(sysm(sp))
    assert not r["cage_first"]["pass_"] and r["cage_first"]["worst_dir_label"] == "+X"
    assert abs(r["cage_first"]["margin_mm"] - 1.5) < 0.05
    r2 = run(sysm(sp, openings=[{"part": "cart", "axis": [1, 0, 0], "half_angle_deg": 30}]))
    assert r2["cage_first"]["pass_"], r2["summary"]
    pp = r2["cage_first"]["per_part"]["cart"]
    assert pp["min_margin_mm"] < 3 and pp["min_margin_outside_openings_mm"] >= 3
    r3 = run(sysm(sp, openings=[{"part": "other", "axis": [1, 0, 0], "half_angle_deg": 30}]))
    assert not r3["cage_first"]["pass_"]                                    # an opening for another part does not apply


def test_overlap_fit():
    a = {"name": "a", "box": [0, 20, 0, 20, 0, 20], "role": "structure"}
    b = {"name": "b", "box": [15, 35, 0, 20, 0, 20], "role": "structure"}
    r = run(dict(name="fit", parts=[a, b]))
    w = r["fit"]["worst"]
    assert not r["pass"] and not r["fit"]["pass_"]
    assert abs(w["overlap_mm3"] - 2000) / 2000 < 0.05, w
    assert r["cage_first"]["applies"] is False
    r2 = run(dict(name="fit", parts=[a, b], contact_pairs=[["b", "a", 2100]]))
    assert r2["pass"] and r2["fit"]["pairs"][0]["contact"], r2["fit"]
    r3 = run(dict(name="fit", parts=[a, b], contact_pairs=[["a", "b", 100]]))
    assert not r3["pass"]                                                   # press fit allowance smaller than the overlap
    c = {"name": "c", "box": [25, 30, 0, 5, 0, 5], "role": "structure"}
    r4 = run(dict(name="cl", parts=[a, c], clearance_pairs=[["a", "c", 3]]))
    cl = r4["fit"]["clearance"][0]
    assert abs(cl["clearance_mm"] - 5) < 0.05 and cl["ok"], cl


def test_mass_cg():
    a = {"name": "a", "box": [-5, 5, -5, 5, -5, 5], "material": "PETG"}     # 1000 mm^3 -> 1.27 g at 0
    b = {"name": "b", "box": [15, 25, -5, 5, -5, 5], "mass_g": 5.0, "role": "bought"}   # 5 g at x = 20
    r = run(dict(name="m", parts=[a, b], cg_ref=[10, 0, 0]))
    mc = r["mass_cg"]
    assert abs(mc["mass_g"] - 6.27) < 0.05, mc
    x = 5 * 20 / 6.27
    assert abs(mc["cg"][0] - x) < 0.02 and abs(mc["cg"][1]) < 1e-6, mc
    assert abs(mc["cg_offset"][0] - (x - 10)) < 0.02
    tpu = run(dict(name="m", parts=[dict(a, material="TPU")]))["mass_cg"]["mass_g"]
    assert abs(tpu - 1.21) < 0.01
    try:
        run(dict(name="m", parts=[{"name": "esc", "box": [0, 1, 0, 1, 0, 1], "role": "bought"}])); assert False
    except ValueError:
        pass
    e = run(dict(name="m", parts=[dict(a, tag="E"), b]))
    assert "(E)" in e["summary"] and "mass 6.3 g (E)" in e["summary"], e["summary"]


def test_balance():
    frame = {"name": "frame", "box": [-50, 50, -50, 50, -10, 10], "mass_g": 100}          # CG at origin
    off = {"name": "gps", "box": [36, 44, -4, 4, 20, 28], "mass_g": 10}                   # 10 g at x = 40 -> CG x = 3.64
    r = run(dict(name="b", parts=[frame, off], cg_ref=[0, 0, 0]))
    b = r["balance"]
    assert not b["pass_"] and not r["pass"] and abs(b["worst_horizontal_mm"] - 400 / 110) < 0.01, b
    assert abs(b["cases"]["all"]["axial_mm"] - 240 / 110) < 0.01                          # z offset is axial, not counted
    assert "FAIL (tol 3)" in r["summary"] and "off thrust axis" in r["summary"]
    r = run(dict(name="b", parts=[frame, off], cg_ref=[0, 0, 0], cg_tol_mm=4.0))
    assert r["balance"]["pass_"] and r["pass"]
    r = run(dict(name="b", parts=[frame, off], cg_ref=[0, 0, 0], thrust_axis=[1, 0, 0]))  # along x it is axial now; z 2.18 < 3
    assert r["balance"]["pass_"], r["balance"]
    # swap parts: a 50 g pack on the axis keeps both cases in; an off-axis pack passes with it but fails without it
    pack = {"name": "pack", "box": [-5, 5, -5, 5, -20, -10], "mass_g": 50, "swap": True}
    r = run(dict(name="b", parts=[frame, pack], cg_ref=[0, 0, 0]))
    assert r["balance"]["pass_"] and set(r["balance"]["cases"]) == {"all", "no_swap"} and "without swap parts" in r["summary"]
    ctr = {"name": "ballast", "box": [-24, -16, -4, 4, 10, 18], "mass_g": 25}           # 25 g at x = -20 balances the pack
    pk2 = dict(pack, box=[5, 15, -5, 5, -20, -10])                                        # 50 g at x = +10
    r = run(dict(name="b", parts=[frame, ctr, pk2], cg_ref=[0, 0, 0]))
    c = r["balance"]["cases"]
    assert c["all"]["pass_"] and c["all"]["horizontal_mm"] < 1e-6 and not c["no_swap"]["pass_"], c
    assert abs(c["no_swap"]["horizontal_mm"] - 500 / 125) < 0.01                          # 4 mm without the pack
    r = run(dict(name="b", parts=[frame, ctr, pk2], cg_ref=[0, 0, 0]))
    assert not r["pass"] and r["balance"]["worst_case"] == "no_swap"
    assert run(dict(name="b", parts=[frame, off]))["balance"]["applies"] is False


def test_pose_and_cli():
    a = {"name": "a", "box": [-5, 5, -5, 5, -5, 5], "pose": {"t": [100, 0, 0], "rpy_deg": [0, 0, 45]}, "material": "PETG"}
    r = run(dict(name="p", parts=[a]))
    assert abs(r["compact"]["extents_mm"][0] - 10 * np.sqrt(2)) < 0.1 and abs(r["compact"]["bbox_min"][0] - (100 - 5 * np.sqrt(2))) < 0.1
    d = tempfile.mkdtemp()
    trimesh.creation.box([10, 10, 10]).export(os.path.join(d, "a.stl"))
    man = dict(name="cli", parts=[{"name": "a", "mesh": "a.stl"}, {"name": "b", "mesh": "a.stl", "pose": {"t": [5, 0, 0]}}])
    json.dump(man, open(os.path.join(d, "m.json"), "w"))
    out = os.path.join(d, "o", "sc.json")
    assert main([os.path.join(d, "m.json"), "--out", out]) == 1             # 500 mm^3 overlap
    assert main([os.path.join(d, "m.json"), "--report"]) == 0
    res = json.load(open(out))
    assert abs(res["fit"]["worst"]["overlap_mm3"] - 500) < 25 and "(M)" in res["summary"]


RODS = [{"name": f"rod{i}", "cylinder": {"r": 2, "p0": [x, y, -60], "p1": [x, y, 60]}, "role": "cage", "material": "carbon"}
        for i, (x, y) in enumerate([(40, 40), (-40, 40), (-40, -40), (40, -40)])]
BOX = {"name": "pack", "box": [-30, 30, -30, 30, -30, 30], "role": "protected", "mass_g": 100}


def test_probes_open_cage():
    """4 vertical rods round a box: every flat wall meets a rod 12 mm ahead, but a post passes between the rods."""
    r = run(dict(name="rods", parts=RODS + [BOX]))
    cf, pr = r["cage_first"], r["probes"]
    assert cf["pass_"] and abs(cf["margin_mm"] - 12.0) < 0.05, cf                 # the flat-wall check alone passes
    assert pr["applies"] and not pr["pass_"] and not r["pass"], r["summary"]
    post = next(x for x in pr["per_probe_part"] if x["probe"] == "post r10")
    assert not post["pass_"] and post["unshielded"] and post["part"] == "pack"
    assert post["dir_label"] in ("+X", "-X", "+Y", "-Y"), post                     # straight through a side gap
    d = np.abs(post["dir"]); k = int(np.argmax(d))
    assert abs(post["contact"][k]) > 29.0 and abs(post["margin_mm"] + 12.0) < 1.0, post   # meets the box face, 12 mm inside the rods
    assert "probes -12" in r["summary"] and "unshielded) FAIL" in r["summary"], r["summary"]
    # close the cage with a frame at top and bottom: an infinite line or post can no longer reach the box first
    ring = []
    for z in (-60, 60):
        c = [[-40, -40, z], [40, -40, z], [40, 40, z], [-40, 40, z]]
        ring += [{"name": f"ring{z}_{k}", "cylinder": {"r": 2, "p0": c[k], "p1": c[(k + 1) % 4]}, "role": "cage", "material": "carbon"} for k in range(4)]
    cage = RODS + ring
    cp = [[a["name"], b["name"], 1e9] for i, a in enumerate(cage) for b in cage[i + 1:]]
    r2 = run(dict(name="frame", parts=cage + [BOX], contact_pairs=cp), probes={"n_dirs": 200})
    assert r2["probes"]["pass_"] and r2["pass"], r2["summary"]
    assert abs(r2["probes"]["worst"]["margin_mm"] - 12.0) < 1.0, r2["probes"]["worst"]   # 12 mm, to the 1 mm raster
    r3 = run(dict(name="rods", parts=RODS + [BOX], probes=False))
    assert r3["pass"] and not r3["probes"]["applies"]                               # probes off: only flat walls


def test_probe_margin():
    """the shell cage stops every probe; probe_margin_mm above the real gap fails it."""
    r = run(sysm({"name": "pack", "box": [-40, 40, -40, 40, -40, 40], "role": "protected", "mass_g": 100}), probes={"n_dirs": 100})
    assert r["probes"]["pass_"] and abs(r["probes"]["worst"]["margin_mm"] - 10.0) < 1.0, r["probes"]["worst"]
    r2 = run(sysm({"name": "pack", "box": [-40, 40, -40, 40, -40, 40], "role": "protected", "mass_g": 100}, probe_margin_mm=12.0), probes={"n_dirs": 100})
    assert not r2["probes"]["pass_"] and not r2["probes"]["worst"]["unshielded"]


def test_probes_shadow():
    """a probe stops at the first solid: a part inside the pack is never reached, even when the pack is exempt."""
    fc = {"name": "fc", "box": [-10, 10, -10, 10, -10, 10], "role": "protected", "mass_g": 10}
    r = run(dict(name="rods", parts=RODS + [BOX, fc]), probes={"n_dirs": 100})
    rows = r["probes"]["per_probe_part"]
    assert not r["probes"]["pass_"] and {x["part"] for x in rows} == {"pack"}, rows    # only the pack is ever met
    op = [{"part": "pack", "axis": [0, 0, 1], "half_angle_deg": 180.0, "skip_probes": True}]
    r2 = run(dict(name="rods", parts=RODS + [BOX, fc], openings=op), probes={"n_dirs": 100})
    assert r2["probes"]["pass_"] and not r2["probes"]["per_probe_part"], r2["probes"]   # exempt pack still shields the fc


TRAY = {"name": "tray", "box": [-50, 50, -30, 30, -5, 0], "role": "structure", "material": "PETG"}
PACK = {"name": "pack", "box": [-40, 40, -15, 15, 0, 25], "role": "bought", "mass_g": 180, "swap": True}


def test_access_pack_swap():
    """the pack slides out along +x; a strut across its path blocks it until it is moved up."""
    strut = {"name": "strut", "cylinder": {"r": 2, "p0": [60, -40, 12], "p1": [60, 40, 12]}, "role": "structure", "material": "carbon"}
    path = {"name": "pack swap", "part": "pack", "dir": [1, 0, 0], "length_mm": 100, "tool": "part", "ignore": ["tray"]}
    r = run(dict(name="acc", parts=[TRAY, PACK, strut], access=[path]))
    ac = r["access"]; p = ac["paths"][0]
    assert not ac["pass_"] and not r["pass"] and p["blocked_by"][0]["part"] == "strut", ac
    assert abs(p["overlap_mm3"] - np.pi * 4 * 30) / (np.pi * 4 * 30) < 0.1, p          # the strut's 30 mm inside the pack's width
    assert "pack swap blocked by strut" in r["summary"] and "access 0/1 clear" in r["summary"]
    up = dict(strut, cylinder={"r": 2, "p0": [60, -40, 30], "p1": [60, 40, 30]})       # above the pack (top 25): clear
    r2 = run(dict(name="acc", parts=[TRAY, PACK, up], access=[path]))
    assert r2["access"]["pass_"] and r2["pass"] and "access 1/1 clear" in r2["summary"], r2["summary"]
    r3 = run(dict(name="acc", parts=[TRAY, PACK, up], access=[dict(path, ignore=[])]))  # pack rests on the tray: rest contact is not counted
    assert r3["access"]["pass_"], r3["access"]


def test_access_key():
    """M3 hex key (cylinder d 4) straight up from a bolt head; a cover plate above blocks it, a hole-free offset clears it."""
    plate = {"name": "plate", "box": [-20, 20, -20, 20, 0, 3], "role": "structure", "material": "PETG"}
    cover = {"name": "cover", "box": [-10, 10, -10, 10, 30, 33], "role": "structure", "material": "PETG"}
    key = {"name": "M3 key", "part": "plate", "from": [0, 0, 3], "dir": [0, 0, 1], "length_mm": 60, "tool": {"type": "cylinder", "d": 4}}
    r = run(dict(name="key", parts=[plate, cover], access=[key]))
    p = r["access"]["paths"][0]
    assert not p["pass_"] and p["blocked_by"][0]["part"] == "cover" and abs(p["overlap_mm3"] - np.pi * 4 * 3) < 3, p
    r2 = run(dict(name="key", parts=[plate, cover], access=[dict(key, **{"from": [15, 15, 3]})]))
    assert r2["access"]["pass_"], r2["access"]
    usb = {"name": "USB-C", "part": "plate", "from": [0, 0, 3], "dir": [0, 0, 1], "length_mm": 40, "tool": {"type": "box", "w": 12, "h": 7}}
    r3 = run(dict(name="usb", parts=[plate, cover], access=[usb]))
    assert abs(r3["access"]["paths"][0]["overlap_mm3"] - 12 * 7 * 3) < 3              # the 12 x 7 plug's section through the 3 mm cover


def test_mass_budget():
    """soft limit (Spark 24 Oct): reported and warned, never a FAIL or exit 1."""
    from system_check import report
    a = {"name": "frame", "box": [-5, 5, -5, 5, -5, 5], "mass_g": 60, "budget_g": 70}
    b = {"name": "lidar", "box": [10, 20, -5, 5, -5, 5], "mass_g": 50, "budget_g": 45, "role": "bought"}
    r = run(dict(name="mb", parts=[a, b], mass_budget_g=120))
    mb = r["mass_budget"]
    assert r["pass"] and mb["pass_"] and mb["over"] and mb["over_parts"][0]["name"] == "lidar" and mb["total_ok"], mb
    assert "mass 110 of 120 g (92 %)" in r["summary"] and "over budget (lidar 50.0 of 45 g)" in r["summary"] and "FAIL" not in r["summary"], r["summary"]
    assert "WARN  over budget: lidar 50.0 of 45.0 g" in report(r)
    r2 = run(dict(name="mb", parts=[a, dict(b, budget_g=55)], mass_budget_g=100))
    assert r2["pass"] and not r2["mass_budget"]["total_ok"] and "mass 110 of 100 g (110 %) (C) over budget (total)" in r2["summary"], r2["summary"]
    assert "WARN  over budget: total 110.0 of 100.0 g (110.0 %)" in report(r2)
    d = tempfile.mkdtemp(); json.dump(dict(name="mb", parts=[a, b], mass_budget_g=100), open(os.path.join(d, "m.json"), "w"))
    assert main([os.path.join(d, "m.json")]) == 0                          # over budget: exit code unchanged
    r3 = run(dict(name="mb", parts=[a, dict(b, budget_g=55)], mass_budget_g=120))
    assert r3["pass"] and not r3["mass_budget"]["over"] and "over budget" not in r3["summary"] and "WARN" not in report(r3)
    assert run(dict(name="mb", parts=[dict(a, budget_g=None)]))["mass_budget"]["applies"] is False




def _geo():
    m = trimesh.creation.icosphere(1, 100.0)
    rods = sorted({tuple(sorted(e)) for e in m.edges.tolist()})
    return m.vertices.tolist(), [list(r) for r in rods]


def test_shell_closed_and_open():
    N, R = _geo()
    base = dict(name="d", machine="overseer", parts=[{"name": "c", "sphere": {"r": 100, "center": [0, 0, 0]}, "role": "cage", "mass_g": 1}], n_dirs=200,
                probes=False)
    r = run(dict(base, cage_shell=dict(nodes=N, rods=R)))
    assert r["shell"]["pass_"] and r["shell"]["closed"], r["shell"]
    # drop the rods round the bottom vertex: open there, a FAIL unless the opening is declared
    low = int(np.argmin(np.asarray(N)[:, 2])); R2 = [e for e in R if low not in e]
    r2 = run(dict(base, cage_shell=dict(nodes=N, rods=R2)))
    assert not r2["shell"]["pass_"] and not r2["pass"]
    r3 = run(dict(base, cage_shell=dict(nodes=N, rods=R2, open=[dict(axis=[0, 0, -1], half_angle_deg=45)])))
    assert r3["shell"]["pass_"], r3["shell"]
    ring = sorted({i for e in R if low in e for i in e} - {low})       # the same hole declared as its ring of nodes
    r4 = run(dict(base, cage_shell=dict(nodes=N, rods=R2, open=[dict(ring=ring)])))
    assert r4["shell"]["pass_"], r4["shell"]
    # a flat lens is fine (Spark 26 Oct: formed to the drone, often a disk) unless a manifest asks for a ratio
    Nf = (np.asarray(N) * [1, 1, 0.4]).tolist()
    assert run(dict(base, cage_shell=dict(nodes=Nf, rods=R)))["shell"]["pass_"]
    assert not run(dict(base, cage_shell=dict(nodes=Nf, rods=R, ratio_min=0.85)))["shell"]["pass_"]


def test_shell_missing_on_drone():
    r = run(dict(name="d", machine="scout", parts=[{"name": "c", "sphere": {"r": 100, "center": [0, 0, 0]}, "role": "cage", "mass_g": 1}],
                 n_dirs=200, probes=False))
    assert not r["shell"]["pass_"] and "no cage_shell" in r["summary"]
    r2 = run(dict(name="d", parts=[{"name": "c", "sphere": {"r": 100, "center": [0, 0, 0]}, "role": "cage", "mass_g": 1}], n_dirs=200, probes=False))
    assert not r2["shell"]["applies"]




def test_balance_no_swap_reported():
    parts = [{"name": "body", "box": [-10, 10, -10, 10, -10, 10], "role": "structure", "mass_g": 100},
             {"name": "pack", "box": [-40, -20, -5, 5, -30, -20], "role": "bought", "mass_g": 100, "swap": True},
             {"name": "nose", "box": [25, 35, -5, 5, -5, 5], "role": "bought", "mass_g": 100}]
    m = dict(name="b", parts=parts, cg_ref=[0, 0, 0], n_dirs=200, probes=False)
    assert not run(m)["balance"]["pass_"]                                # all 0 off, no_swap 15 off: gated by default
    r = run(dict(m, balance_no_swap_gate=False))
    assert r["balance"]["pass_"] and "reported" in r["summary"], r["summary"]


if __name__ == "__main__":
    for k, f in list(globals().items()):
        if k.startswith("test_"):
            f(); print("ok", k)
    print("system_check tests: all PASS")
