"""Tests for assembly.py: cd nox && python3 test_assembly.py (or python3 -m pytest test_assembly.py)"""
import json, os, tempfile, numpy as np, trimesh
from assembly import load, new, main, Assembly

HERE = os.path.dirname(os.path.abspath(__file__))
SEEDED = ["overseer/overseer.assembly.json", "box/box_catch.assembly.json", "scout/v3/scout.assembly.json"]


ACCESS = [{"part": "pack", "name": "pack out", "dir": [1, 0, 0], "length_mm": 10, "tool": "part", "ignore": ["motor"]}]
PROBES = {"n_dirs": 100}                                       # system_check keys, kept as given


def tiny(d):
    """cage shell mesh + a protected box + a bought cylinder with a moves function, saved in d."""
    o = trimesh.creation.box([100] * 3); i = trimesh.creation.box([92] * 3)
    trimesh.boolean.difference([o, i]).export(os.path.join(d, "cage.stl"))
    open(os.path.join(d, "kin.py"), "w").write(
        "import numpy as np\n"
        "def lift(s, part):\n"
        "    T = np.eye(4); T[2, 3] = 10.0 * s\n"
        "    return T\n")
    a = new("tiny", "test", [
        {"name": "cage", "role": "cage", "mesh": "cage.stl", "material": "PETG", "owner": "test", "version": "v1", "source": "test_assembly.py"},
        {"name": "pack", "role": "protected", "box": [-30, 30, -30, 30, -30, 30], "mass_g": 100, "swap": True, "tag": "E"},
        {"name": "motor", "role": "bought", "protect": True, "cylinder": {"r": 5, "h": 10, "center": [0, 0, 0], "axis": "z"}, "mass_g": 20,
         "moves": os.path.join(d, "kin.py") + ":lift"},
    ], cg_ref=[0, 0, 0], contact_pairs=[["pack", "motor"]], access=ACCESS, probes=PROBES, custom_key="kept")
    return a


def test_load_save_roundtrip():
    d = tempfile.mkdtemp(); a = tiny(d); p = a.save(os.path.join(d, "tiny.assembly.json"))
    b = load(p)
    assert b.names == ["cage", "pack", "motor"] and b.validate() == []
    assert b.data["access"] == ACCESS and b.data["probes"] == PROBES and b.data["custom_key"] == "kept"
    assert b.part("cage")["owner"] == "test" and b.part("motor")["moves"].endswith(":lift")
    txt = open(p).read()
    assert txt.count('{"name": ') == 3                              # one part per line
    b.save(); assert open(p).read() == txt                           # stable on resave
    assert json.load(open(p)) == json.loads(a.dumps())


def test_pose_and_meshes():
    d = tempfile.mkdtemp(); a = tiny(d); a.save(os.path.join(d, "t.json"))
    m0 = a.mesh("pack"); assert np.allclose(m0.bounds, [[-30] * 3, [30] * 3])
    a.set_pose("pack", {"t": [5, 0, 0], "rpy_deg": [0, 0, 90]})
    assert np.allclose(a.mesh("pack").bounds[:, 0], [-25, 35])
    T = np.eye(4); T[:3, 3] = [0, 0, -7]
    a.set_pose("pack", T)
    assert np.allclose(a.pose("pack"), T) and np.allclose(a.mesh("pack").bounds[:, 2], [-37, 23])
    assert len(a.meshes()) == 3 and all(isinstance(m, trimesh.Trimesh) for m in a.meshes())
    try:
        a.set_pose("pack", [1, 2, 3]); assert False
    except Exception:
        pass
    b = load(a.save()); assert np.allclose(b.pose("pack"), T)       # pose survives the file


def test_check_and_moves():
    d = tempfile.mkdtemp(); a = tiny(d); a.save(os.path.join(d, "t.json"))
    r = a.check(n_dirs=500)
    assert r["pass"], r["summary"]
    assert abs(r["cage_first"]["margin_mm"] - 20.0) < 1e-6           # 50 - 30 on every face
    assert r["summary"].startswith("System: tiny, cage first 20.0 mm")
    assert abs(r["mass_cg"]["mass_g"] - (100 + 20 + (100 ** 3 - 92 ** 3) / 1000 * 1.27)) < 0.5
    up = a.at(motor=1.0)
    assert np.allclose(up.mesh("motor").bounds[:, 2], [5, 15]) and np.allclose(a.mesh("motor").bounds[:, 2], [-5, 5])
    a.set_pose("pack", {"t": [0, 0, 25]})                            # pack top at 55: pokes through the 50 cage
    r2 = a.check(n_dirs=500)
    assert not r2["pass"] and not r2["cage_first"]["pass_"] and r2["cage_first"]["worst_part"] == "pack"
    bad = Assembly(json.loads(a.dumps()), a.path); bad.parts[1]["role"] = "bogus"
    assert bad.validate() and "role" in bad.validate()[0]
    try:
        bad.check(); assert False
    except ValueError:
        pass


def test_scene_and_exports():
    d = tempfile.mkdtemp(); a = tiny(d); a.save(os.path.join(d, "t.json"))
    s = a.scene()
    assert [p["name"] for p in s.parts] == a.names
    assert {p["name"]: p["printed"] for p in s.parts} == {"cage": True, "pack": True, "motor": False}
    dat = s.data(); assert dat["n_frames"] > 2 and len(dat["parts"]) == 3          # the motor's moves animate
    assert len(a.scene(move_s=False).data()["parts"]) == 3
    g = a.export_glb(os.path.join(d, "x.glb")); st = a.export_stl(os.path.join(d, "x.stl"))
    assert os.path.getsize(g) > 1000 and len(trimesh.load(st).faces) > 12
    assert main([a.path, "--list"]) == 0
    assert main([a.path, "--check", "--dirs", "300", "--out", os.path.join(d, "o", "r.json")]) == 0
    assert json.load(open(os.path.join(d, "o", "r.json")))["pass"] is True


def test_seeded_files_load():
    for f in SEEDED:
        a = load(os.path.join(HERE, f))
        assert a.validate() == [], (f, a.validate())
        assert a.data.get("machine") in ("overseer", "box", "scout") and a.data.get("note")
        assert len(a.meshes()) == len(a.parts)
    sc = load(os.path.join(HERE, "scout/v3/scout.assembly.json")).check(n_dirs=300)
    assert sc["summary"].startswith("System: Scout v3")
    bc = load(os.path.join(HERE, "box/box_catch.assembly.json"))
    pl = [p["name"] for p in bc.parts if p.get("moves")]
    assert len(pl) == 4
    m0, m1 = bc.mesh(pl[0]), bc.at(**{pl[0]: 1.0}).mesh(pl[0])
    assert abs(np.linalg.norm(m1.centroid - m0.centroid) - 12.0) < 1e-4           # hook latch RISE


if __name__ == "__main__":
    for k, f in list(globals().items()):
        if k.startswith("test_"):
            f(); print("ok", k)
    print("assembly tests: all PASS")
