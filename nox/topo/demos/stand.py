"""Box stand parts (nox/box/stand/stand_v7_topo.py setups) on the new engine, unchanged geometry and loads.

The old setups build an `arm_topo.PartProblem`; topo.Part has the same call pattern, so a stub `arm_topo` module is
installed before the import and the setups run as written. Nothing from the old engine is imported.
"""
import os
import sys
import types

from ..part import Part

HERE = os.path.dirname(os.path.abspath(__file__))
NOX = os.path.abspath(os.path.join(HERE, "..", ".."))


def _module():
    for p in (os.path.join(NOX, "box"), os.path.join(NOX, "box", "stand")):
        if p not in sys.path:
            sys.path.insert(0, p)
    if "arm_topo" not in sys.modules or not getattr(sys.modules["arm_topo"], "_topo_stub", False):
        stub = types.ModuleType("arm_topo")
        stub.PartProblem = Part
        stub.run_part = None
        stub.log = lambda s: print(str(s)[:400], flush=True)
        stub._topo_stub = True
        sys.modules["arm_topo"] = stub
    import stand_v7_topo as T
    T.PartProblem = Part
    return T


def setup_for(name):
    def setup(h):
        T = _module()
        pp, drawn, cuts, env, gen = T.SETUP[name](h)
        return pp
    return setup
