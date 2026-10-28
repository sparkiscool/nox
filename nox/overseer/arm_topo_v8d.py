"""Overseer printed arms v8d (28 Oct): v8b with a full-length drawn 2.5 sleeve on every tube instead of the cleared ring
(v8b: four sub-voxel flakes at the flat socket faces; v8c's 45° tapers ran out to 1.0 walls). The sleeve sits where v8b
kept the ring clear, so v8b's layouts rebuild with it (same design mask, `arm_rebuild.py`). Same arguments as
arm_topo_v1.py.
"""
import os, runpy
HERE = os.path.dirname(os.path.abspath(__file__))
for k, v in (("NOX_ARM_VERSION", "v8d"), ("NOX_ARM_SLEEVE", "1")):
    os.environ.setdefault(k, v)
runpy.run_path(os.path.join(HERE, "arm_topo_v8b.py"), run_name="__main__")
