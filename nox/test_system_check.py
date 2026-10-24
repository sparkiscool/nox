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


if __name__ == "__main__":
    for k, f in list(globals().items()):
        if k.startswith("test_"):
            f(); print("ok", k)
    print("system_check tests: all PASS")
