"""Overseer printed arms v5d (27 Oct): v5c plus a 2.5 wall closing each tube's blind tip socket (rear v5c solid FAIL:
1.58 walls at the tip ends). Rear runs with --gen 50 (v5c at 40: crash 1.01 in the arm). Same arguments as arm_topo_v1.py.
"""
import os, runpy
HERE = os.path.dirname(os.path.abspath(__file__))
for k, v in (("NOX_ARM_VERSION", "v5d"), ("NOX_ARM_CAP", "1")):
    os.environ.setdefault(k, v)
runpy.run_path(os.path.join(HERE, "arm_topo_v5c.py"), run_name="__main__")
