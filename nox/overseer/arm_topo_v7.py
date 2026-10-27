"""Overseer printed arms v7 (27 Oct): both arms on the cage's rebalanced layout (main @ 0dd9fc97: motor square, spoke
nodes and spoke ends from runs/overseer_layout/v5, snapshot runs/overseer_arms/v7/inputs; loads arm_loads.json rear
1886 N / front 1063 N, snapshot runs/overseer_arms/v7/arm_loads.json; core v3 parts as keep-outs). Rear: v5d (tube chords,
sleeves, caps, flanges), gen 50. Front: v6 as built (12 x 3.0 diagonals, nodes, solid shoulder wedge;
v6 read 182 Hz built, accepted against a 1.15 x 1P full target, DEFAULT pending Spark; 6/4 diagonals failed the install
geometry at offset 9). Same arguments as arm_topo_v1.py.
"""
import os, runpy, sys
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
V7 = (("NOX_ARM_VERSION", "v7"),
      ("NOX_ARM_LAYOUT", os.path.join(ROOT, "runs", "overseer_arms", "v7", "inputs")),
      ("NOX_ARM_JOINT", os.path.join(ROOT, "runs", "overseer_arms", "v7", "arm_loads.json")),
      ("NOX_ARM_CORE", os.path.join(ROOT, "runs", "overseer_core", "v3", "parts")))
for k, v in V7:
    os.environ.setdefault(k, v)
if __name__ == "__main__":
    if "front" in sys.argv[1:2]:
        runpy.run_path(os.path.join(HERE, "arm_topo_v6.py"), run_name="__main__")
    else:
        runpy.run_path(os.path.join(HERE, "arm_topo_v5d.py"), run_name="__main__")
