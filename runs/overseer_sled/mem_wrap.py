"""run wrapper: tells the topo engine this worker's share is 8 GB (budget set to 4: the direct estimate runs ~2x low) (not the box's 16), so it picks multigrid; no engine edit."""
import sys, runpy
sys.path.insert(0, "nox")
import topo.fem as f
f._machine_mem_gb = lambda: 4.0 / 0.75
sys.argv = sys.argv[1:]
runpy.run_path(sys.argv[0], run_name="__main__")
