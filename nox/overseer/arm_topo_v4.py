"""Overseer printed arms v4 (rear, 27 Oct): v3 plus drawn root flanges, 6 thick, top and bottom, from the root out to
x 40 along the arm (rear v3 read cage crash 1.22 on the arm's top face just past the top plate edge, the plate edge the
pivot; flight 0.09, handling 0.26, modes 268 / 274 Hz passed). Third rear version: a fail goes back to the system pass.
"""
import os, runpy
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
os.environ.setdefault("NOX_ARM_VERSION", "v4")
os.environ.setdefault("NOX_ARM_LINKS", "1")
os.environ.setdefault("NOX_ARM_ROOT_U1", "84")
os.environ.setdefault("NOX_ARM_FLANGE_X", "40")
os.environ.setdefault("NOX_ARM_LAYOUT", os.path.join(ROOT, "runs", "overseer_layout", "v5"))
os.environ.setdefault("NOX_ARM_CORE", os.path.join(ROOT, "runs", "overseer_core", "v1", "parts"))
runpy.run_path(os.path.join(HERE, "arm_topo_v1.py"), run_name="__main__")
