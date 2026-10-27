"""Overseer printed arms v5, front and rear (27 Oct, the system pass after three strikes on each pair): the chords are
6/4 carbon tube (the cage's stock, bore 6.2, 20 mm sockets, Part.insert at the tube's section) instead of 3.0 rod, so the
cage's crash loads (front tip ~1.1 kN, braces ~0.9 kN, rear tip ~1.8 kN; frame model) go into carbon, not PETG; tube
centres z 6.5 / 38.5 (3 walls in the 45 depth); rear gets shoulders and tubes too; solid root block, root flanges,
pads to the plate corners kept. Same arguments as arm_topo_v1.py; runs go to runs/overseer_arm_<front|rear>/v5.
"""
import os, runpy
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
for k, v in (("NOX_ARM_VERSION", "v5"), ("NOX_ARM_LINKS", "1"), ("NOX_ARM_ROOT_U1", "84"), ("NOX_ARM_ROOT_SOLID", "1"),
             ("NOX_ARM_FLANGE_X", "40"), ("NOX_ARM_CHORD", "tube6"), ("NOX_ARM_ROD_Z", "38.5,6.5"),
             ("NOX_ARM_LAYOUT", os.path.join(ROOT, "runs", "overseer_layout", "v5")),
             ("NOX_ARM_CORE", os.path.join(ROOT, "runs", "overseer_core", "v1", "parts"))):
    os.environ.setdefault(k, v)
runpy.run_path(os.path.join(HERE, "arm_topo_v1.py"), run_name="__main__")
