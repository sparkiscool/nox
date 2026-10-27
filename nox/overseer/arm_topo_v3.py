"""Overseer printed arms v3 (rear first, 27 Oct): v2 (cage nodes, frame-model loads, core keep-outs, bore 3.2 / 6.2) with
the root block and its plate contact pads run out to the plate corners (u to 84, clipped by each plate outline: rear v2
read crash 2.44 at the top-plate face edge of the stock 17 mm block) and more material (rear v2: modes 158 / 184 Hz,
handling 0.58). Same arguments as arm_topo_v1.py; runs go to runs/overseer_arm_<front|rear>/v3.
"""
import os, runpy
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
os.environ.setdefault("NOX_ARM_VERSION", "v3")
os.environ.setdefault("NOX_ARM_LINKS", "1")
os.environ.setdefault("NOX_ARM_ROOT_U1", "84")
os.environ.setdefault("NOX_ARM_LAYOUT", os.path.join(ROOT, "runs", "overseer_layout", "v5"))
os.environ.setdefault("NOX_ARM_CORE", os.path.join(ROOT, "runs", "overseer_core", "v1", "parts"))
runpy.run_path(os.path.join(HERE, "arm_topo_v1.py"), run_name="__main__")
