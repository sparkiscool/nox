"""Overseer arms: build and check again from a finished layout (density.npz), with a version wrapper's settings, e.g.
after a drawn change that leaves the design mask as it was (v8c: the socket tapers on v8b's layouts).

python3 nox/overseer/arm_rebuild.py <arm_topo_vN.py> front|rear <gen_cm3> <src dir with density.npz> <out dir>
"""
import builtins, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
script, which, gen, src, out = sys.argv[1:6]
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(ROOT, "nox"))


class _Chain:                                    # run the wrapper chain for its env only, stop at arm_topo_v1
    @staticmethod
    def run_path(path, run_name=None):
        if os.path.basename(path) != "arm_topo_v1.py":
            exec(open(path).read(), {"__file__": path, "__name__": "__main__", "runpy": _Chain})


_imp = builtins.__import__
builtins.__import__ = lambda n, *a, **k: _Chain if n == "runpy" else _imp(n, *a, **k)
_argv = sys.argv; sys.argv = [script, which]
_Chain.run_path(os.path.abspath(script))
sys.argv = _argv
builtins.__import__ = _imp
import arm_topo_v1 as A
from topo import Settings
from topo.run import rebuild
rebuild(A.setup_for(which), h=2.0 if which == "front" else 1.5, gen_cm3=float(gen), src_dir=src, out_dir=out,
        name=f"overseer_arm_{which}", settings=Settings())
