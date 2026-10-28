"""Overseer printed arms v8c (28 Oct): v8b plus a drawn 45° taper closing each tube socket onto the bore (rear v8b's only
solid FAIL: four 0.016 mm³ build specks where the flat socket faces met the cleared ring). The design mask is v8b's, so
v8b's layouts rebuild with it (`arm_rebuild.py`). Same arguments as arm_topo_v1.py.
"""
import os, runpy
HERE = os.path.dirname(os.path.abspath(__file__))
for k, v in (("NOX_ARM_VERSION", "v8c"), ("NOX_ARM_TAPER", "1")):
    os.environ.setdefault(k, v)
runpy.run_path(os.path.join(HERE, "arm_topo_v8b.py"), run_name="__main__")
