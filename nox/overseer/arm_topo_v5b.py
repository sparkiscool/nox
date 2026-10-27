"""Overseer printed arms v5b, front and rear (27 Oct): v5 (6/4 carbon tube chords, solid root, flanges) with the cage's
newest loads and no L2 brace sockets: the braces root on a top-plate socket block (cage session, layout v5 BRACE_ROOT),
so the front arm carries only its spoke node. Loads: the cage frame's arm_loads.json (front spoke node 1013 N, rear
2354 N, F on E), snapshot in runs/overseer_arms/v5/arm_loads.json. Same arguments as arm_topo_v1.py.
"""
import os, runpy
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
for k, v in (("NOX_ARM_VERSION", "v5b"), ("NOX_ARM_BRACES", "0"),
             ("NOX_ARM_JOINT", os.path.join(ROOT, "runs", "overseer_arms", "v5", "arm_loads.json"))):
    os.environ.setdefault(k, v)
runpy.run_path(os.path.join(HERE, "arm_topo_v5.py"), run_name="__main__")
