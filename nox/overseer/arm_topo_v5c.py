"""Overseer printed arms v5c, front and rear (27 Oct): v5b (6/4 carbon tube chords, cage arm_loads.json, no brace sockets)
plus two drawn fixes from rear v5b (solid FAIL: 1.1 / 1.2 walls where webs met the bare tube mid-span; cage crash 11 at
1.47 where the shoulder socket hung off the root flange corner): a full-length 2.5 sleeve on every tube, and the top /
bottom flanges spanning the root block to both shoulder sockets. Same arguments as arm_topo_v1.py.
"""
import os, runpy
HERE = os.path.dirname(os.path.abspath(__file__))
for k, v in (("NOX_ARM_VERSION", "v5c"), ("NOX_ARM_SLEEVE", "1"), ("NOX_ARM_SHFLANGE", "1")):
    os.environ.setdefault(k, v)
runpy.run_path(os.path.join(HERE, "arm_topo_v5b.py"), run_name="__main__")
