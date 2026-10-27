"""Overseer printed arms v2: arms v1's method on the current layout (27 Oct: cage v5 moved the motor square 5.6
forward, front r ~249 / rear r ~157) with the core session's real parts as keep-outs and the 3.2 rod bore (Spark's coupon
pick). Same arguments as arm_topo_v1.py; runs go to runs/overseer_arm_<front|rear>/v2. Cage nodes still left off (Spark).
"""
import os, runpy
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
os.environ.setdefault("NOX_ARM_VERSION", "v2")
os.environ.setdefault("NOX_ARM_LAYOUT", os.path.join(ROOT, "runs", "overseer_layout", "v5"))
os.environ.setdefault("NOX_ARM_CORE", os.path.join(ROOT, "runs", "overseer_core", "v1", "parts"))
runpy.run_path(os.path.join(HERE, "arm_topo_v1.py"), run_name="__main__")
