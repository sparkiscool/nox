"""Overseer printed arms v3, FRONT (27 Oct): v2 (cage nodes, carbon chords, frame-model loads) with the root block solid
between the plates (front v2 read 1.6 flight / 5.5 handling / 9.1 cage crash in its open middle), drawn root flanges 6
thick to x 50, the bottom rods raised to z 5.5 (a 1.2 wall under the bore at z 4 failed the gate) and the root pads to
the plate corners. Same arguments as arm_topo_v1.py; runs go to runs/overseer_arm_front/v3.
"""
import os, runpy
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
os.environ.setdefault("NOX_ARM_VERSION", "v3")
os.environ.setdefault("NOX_ARM_LINKS", "1")
os.environ.setdefault("NOX_ARM_ROOT_U1", "84")
os.environ.setdefault("NOX_ARM_ROOT_SOLID", "1")
os.environ.setdefault("NOX_ARM_FLANGE_X", "50")
os.environ.setdefault("NOX_ARM_ROD_Z", "39.5,5.5")
os.environ.setdefault("NOX_ARM_LAYOUT", os.path.join(ROOT, "runs", "overseer_layout", "v5"))
os.environ.setdefault("NOX_ARM_CORE", os.path.join(ROOT, "runs", "overseer_core", "v1", "parts"))
runpy.run_path(os.path.join(HERE, "arm_topo_v1.py"), run_name="__main__")
