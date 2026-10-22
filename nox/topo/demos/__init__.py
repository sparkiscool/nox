"""Demo parts for nox/topo. Each entry returns dict(setup=f(h) -> Part, h=default cell size, gen_cm3=generated
volume target, baseline=numbers of the part it would replace, for the comparison table).

  holder      the old engine's benchmark holder (noxopt_bench.holder / noxopt_demo.py), 28 load cases, 3 supports
  bracket_e   box stand ring-to-wall end bracket: the part whose v8 bolt pads hung on 2..3 mm necks
  bracket_jA  box stand ring-to-wall joint bracket (old v8 shipped)
  bar_half    box stand cross bar half (old v8 shipped)
  motor_mount generic drone motor mount on a carbon plate (dimensions are assumptions, E), thrust / torque / crash
"""
import os
import sys
import json

HERE = os.path.dirname(os.path.abspath(__file__))
NOX = os.path.abspath(os.path.join(HERE, "..", ".."))


def _old(name):
    p = os.path.join(NOX, "box", "stand", name)
    try:
        with open(p) as f:
            r = json.load(f)
        return dict(engine="noxopt v8", mass_g=round(r.get("mass_g", 0), 1), worst=r.get("worst"),
                    solid_ok_then=r.get("solid_ok"), necks_checked="necks" in r.get("solid", {}), gen_cm3=r.get("gen_cm3"))
    except OSError:
        return None


def _holder():
    from .holder import setup
    return dict(setup=setup, h=1.0, gen_cm3=6.0,
                baseline=dict(engine="noxopt v0 demo (h 4/3, vf 0.12)", mass_g=10.4, worst=0.32,
                              note="noxopt_demo.py smoke test, 16 Oct (T)"))


def _stand(part, gen, h=1.0, baseline_file=None, note=None):
    def f():
        from .stand import setup_for
        b = _old(baseline_file) if baseline_file else None
        if b is not None and note:
            b["note"] = note
        return dict(setup=setup_for(part), h=h, gen_cm3=gen, baseline=b)
    return f


def _motor():
    from .motor_mount import setup
    return dict(setup=setup, h=0.8, gen_cm3=7.0, baseline=None)


PARTS = {
    "holder": _holder,
    "bracket_e": _stand("bracket_e", 60.0, 1.0, "bracket_e_result.json",
                        "v8 run 3: passed the 0.9 mm erosion check, pulled 20 Oct 14:30 for bolt pads on 2..3 mm necks"),
    "bracket_jA": _stand("bracket_jA", 12.0, 1.0, "bracket_jA_result.json", "v8 shipped 20 Oct 17:55"),
    "bar_half": _stand("bar_half", 45.0, 1.0, "bar_half_v8_result.json", "v8 shipped 20 Oct 18:25 (after a failed run 1)"),
    "motor_mount": _motor,
}
