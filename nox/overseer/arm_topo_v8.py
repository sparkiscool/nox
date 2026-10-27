"""Overseer printed arms v8 (27 Oct, Spark: "too bulky ... isnt the whole point to have natural structures"; targets
cage crash index <= 1.0, front first mode >= 175 Hz): drawn = attachments only (root contact pads + insert bosses, motor
pad + pocket wall, the cage spoke node sockets, the 4 tube end sockets with tip caps, drawn collar rings on the tube free
spans with the rest of the ring round each bore kept clear); no solid root block, flanges, sleeves, diagonals, node
blocks or shoulder wedge. Topology grows everything else. On the v7 inputs (rebalanced layout, 1886 / 1063 N).
Same arguments as arm_topo_v1.py.
"""
import os, runpy
HERE = os.path.dirname(os.path.abspath(__file__))
for k, v in (("NOX_ARM_VERSION", "v8"), ("NOX_ARM_SLEEVE", "0"), ("NOX_ARM_SHFLANGE", "0"), ("NOX_ARM_FLANGE_X", ""),
             ("NOX_ARM_ROOT_SOLID", "0"), ("NOX_ARM_DIAG", "0"), ("NOX_ARM_SHWEB", ""), ("NOX_ARM_COLLARS", "3")):
    os.environ.setdefault(k, v)
import sys
if "rear" in sys.argv[1:2]:
    os.environ["NOX_ARM_COLLARS"] = os.environ.get("NOX_ARM_COLLARS_REAR", "1")   # the rear free span is 23 mm
runpy.run_path(os.path.join(HERE, "arm_topo_v7.py"), run_name="__main__")
