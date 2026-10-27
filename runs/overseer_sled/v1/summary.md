# overseer_sled v1: FAILED (no mesh; out of memory in the fine stage)

No STL was built, so solid_check, print_orient, fit and key-line checks were not run. Requirements: `nox/dimensions.md` "System Overseer core" (Sled rows). Script `nox/overseer/core_topo_v1.py sled`, wrapper `runs/overseer_sled/mem_wrap.py` (no engine edit).

Attempts (all killed by the cgroup OOM killer at the start of the fine stage; box shared with the hub deck run):
- DEFAULT h 1.25, direct: killed, peak 9.5 GB (M).
- DEFAULT h 1.5 (the fallback in the brief): fails in the coarse stage, "zero compliance ... load sits on fixed nodes" (feet too thin on the 64 x 48 x 32 coarse grid), peak 1.6 GB.
- DEFAULT h 1.25, multigrid via wrapper (budget 8 GB): killed, peak 8.4 GB.
- DEFAULT h 1.4, direct (estimate 4.3 GB, real > 9.5 GB): killed, peak 9.5 GB.
- DEFAULT h 1.4, multigrid (budget 6 GB): killed, peak 8.6 GB (coarse stage 17 min).
- DEFAULT h 1.4, multigrid (budget 4 GB), resumed from the coarse checkpoint: killed after 0.05 h, peak 13.3 GB.

Next: rerun alone on the box after the hub deck finishes, h 1.4, or reduce the 26 load cases per solve (engine memory scales with cases x dofs). Overhang crash index (35 g cases past the bolt rows): not obtained.
