"""Overseer arm rear v5c: build and check again from the finished layout (the run OOM'd in the solid check, 27 Oct; fixed
in nox/topo voxelise). python3 nox/overseer/arm_rebuild_v5c.py <src dir with density.npz>"""
import os, sys
ROOT = os.getcwd()
for k, v in (("NOX_ARM_VERSION","v5c"),("NOX_ARM_SLEEVE","1"),("NOX_ARM_SHFLANGE","1"),("NOX_ARM_BRACES","0"),
             ("NOX_ARM_JOINT",os.path.join(ROOT,"runs/overseer_arms/v5/arm_loads.json")),("NOX_ARM_LINKS","1"),("NOX_ARM_ROOT_U1","84"),
             ("NOX_ARM_ROOT_SOLID","1"),("NOX_ARM_FLANGE_X","40"),("NOX_ARM_CHORD","tube6"),("NOX_ARM_ROD_Z","38.5,6.5"),
             ("NOX_ARM_LAYOUT",os.path.join(ROOT,"runs/overseer_layout/v5")),("NOX_ARM_CORE",os.path.join(ROOT,"runs/overseer_core/v1/parts"))):
    os.environ.setdefault(k, v)
sys.path.insert(0, os.path.join(ROOT, "nox/overseer")); sys.path.insert(0, os.path.join(ROOT, "nox"))
import arm_topo_v1 as A
from topo import Settings
from topo.run import rebuild
rebuild(A.setup_for("rear"), h=1.5, gen_cm3=40.0, src_dir=sys.argv[1], out_dir=os.path.join(ROOT, "runs/overseer_arm_rear/v5c"),
        name="overseer_arm_rear", settings=Settings())
