"""Overseer printed arms v8b (28 Oct): v8 (attachments only, organic) after rear v8 (96 g, cage crash 1.77, solid FAIL:
fragments and 1.1..1.6 walls at the collar rings and the bottom tongue): no collar rings (the tube free spans stay bare,
the 2.5 ring round each bore kept clear), top third screw only (the bottom one has no driver path, cage session), more
generated volume (rear 55, front set per run). Same arguments as arm_topo_v1.py.
"""
import os, runpy
HERE = os.path.dirname(os.path.abspath(__file__))
for k, v in (("NOX_ARM_VERSION", "v8b"), ("NOX_ARM_COLLARS", "0"), ("NOX_ARM_COLLARS_REAR", "0"), ("NOX_ARM_THIRD_BOTTOM", "0")):
    os.environ.setdefault(k, v)
runpy.run_path(os.path.join(HERE, "arm_topo_v8.py"), run_name="__main__")
