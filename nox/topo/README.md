# nox/topo: the Nox topology engine

Written 22 Oct 2026 to replace `noxopt.py`, `noxopt_local.py` and `printopt.py` for generated parts. The old files stay in the repo until the demos below have been compared; nothing in `nox/topo` imports them.

## What it does differently

| Problem with the old engine | What nox/topo does |
|---|---|
| Pads hung on 2 to 3 mm necks (bracket_e v8). The 0.9 mm erosion check passed them, and the neck test found them only after the build. | **Anchored-core erosion.** In the eroded analysis only the drawn bodies' cores stay solid. Their skins erode like design material, and loads on them move to the core. A narrow root leaves a gap in the eroded design and every load through it blows up, so the optimiser widens the root during the run. Unit test `t_neck` measures this. |
| The masked filter treated keep-outs as "outside", so material flush against a cut was never eroded. Thin sheets and necks along cuts survived. | Constant-normalised cone filter. Every void and the grid edge count as empty, so the length scale holds against keep-outs too. |
| The length scale was set by hand per part (R in cells, floor_mm, prune_thin passes). | **Minimum member in mm** (default 3.2 mm rods, about 3.0 mm plates). The filter radius comes from a 3D calibration of the robust thresholds (`layout.calibration`). On top of that, Zhou et al. 2015 geometric constraints act on solid and void from beta 16. There is no prune pass. |
| Unreachable setups (an access cut wider than its pad, an uncovered boss) ran for hours before failing. | **Reachability check** before the first solve. Every drawn body must be joinable by a member of the minimum size. If not, the run raises with the body's position. |
| Ties were translations only, at weight 0.25. | Ties use 3 forces plus 3 moments per unheld body, weight 0.5, on the eroded design. |
| Pardiso OOM above about 60k design cells. The separate gmg file had its own Layout. | One `Solver`. Pardiso runs on the active dofs only, and a Galerkin geometric multigrid CG takes over beyond the RAM budget. Coarse levels are built element by element and exactly. It warm-starts and solves all load cases of a support group together. |
| Polish flow, prune loops and reinforcement balls at build. | The surface is the eta_n iso-surface of the filtered field. It is smooth by construction and blends into drawn bodies with a fillet of the filter radius. It is held 0.5 mm off keep-outs and cut exactly with manifold3d. |
| Limit-cycling volume control at gain 1.0. | Damped volume-limit update (gain 0.3). |

The gate is unchanged: `nox/solid_check.py`, run on the exact built mesh.

## Formulation

- **Fields.** The design variable x lives on the design cells. x_hat is x there, 1 on drawn cells and 0 on voids. Filtering gives x_tilde, and three Heaviside projections give the designs: eroded (eta 0.75), nominal or blueprint (0.5) and dilated (0.25) (Wang, Lazarov & Sigmund 2011).
- **Objective.** The p-mean (P = 6) of c_i(eroded) / c_i(full solid) over the real load cases and the tie cases. SIMP p = 3, Emin 1e-6. Body forces (`accel`) are density-weighted and carry their sensitivities.
- **Constraints.** Dilated volume <= V*, with V* adapted so the blueprint meets the mass target. The Zhou 2015 solid and void indicators are capped at eps (c = R^4).
- **Optimiser.** MMA (Svanberg 2007 asymptotes, dual solved by L-BFGS-B). Beta runs 1, 2, 4, 8, 16, 32. A stage advances when it has settled. The run stops at <= 2 % grey.
- **Material.** PETG, PLA or TPU, transversely isotropic about the build axis. The as-built strength report uses in-layer von Mises / A_xy and across-layer sqrt((sigma_n+/A_z)^2 + (tau_n/A_s)^2), with the same allowables as the old engine so indices compare.

## Compute plan (4 vCPU / 16 GB cloud box, 6 h cap)

- **Stage 1 (2h grid).** Beta 1 to 4 run on a grid twice as coarse, with 8x fewer cells. Most iterations happen here.
- **Stage 2 (h grid).** Warm-started from stage 1. It runs from beta 4 to the end.
- **Solver choice.** Pardiso is used while the estimated factor fits 60 % of 75 % of RAM, about 0.5 M active dofs. Above that the run uses multigrid CG: Jacobi(0.6) x2, 2:1 levels down to <= 40k dofs, Pardiso on the coarsest level, tolerance 1e-5, warm starts.
- **Time limit.** Past 75 % of `time_limit_s` the beta steps are taken as soon as allowed. Checkpoints are written every 10 iterations, and `--git` commits and pushes each one (CLAUDE.md, Heavy runs).
- **Grid size.** h defaults to about the min member / 3, the nTop rule for element size. The filter radius is then about 2.4 to 3 cells.

## Use

```bash
cd nox
python3 -m topo.test_topo            # self-test, ~3 min
python3 -m topo.run holder --git     # a demo part end to end -> runs/holder/topo-v1/
```

From a part script:

```python
from topo import Part, Settings, optimise
def setup(h):
    p = Part("my_part", h, drawn, cuts, env, build="+z")
    p.support("default", region); p.load("case", region, (fx, fy, fz), cls="misuse")
    return p
optimise(setup, h=1.0, gen_cm3=12, out_dir="runs/my_part/v1", name="my_part", settings=Settings(min_member_mm=3.2), git=True)
```

The old `arm_topo.PartProblem` setups run unchanged. See `demos/stand.py`.

## Demos

| Name | What |
|---|---|
| `holder` | The old engine's benchmark holder (28 cases, 3 supports). |
| `bracket_e` | The bracket whose v8 bolt pads hung on necks. |
| `bracket_jA`, `bar_half` | Box stand parts the old v8 shipped. |
| `motor_mount` | A generic drone motor mount (assumed dimensions, E). |

## Sources

- Wang, Lazarov, Sigmund 2011, SMO 43:767: the robust formulation.
- Trillet, Duysinx, Fernández 2021, arXiv 2101.08605: analytical length scales.
- Zhou, Lazarov, Wang, Sigmund 2015, CMAME 293:266: geometric constraints.
- Clausen & Andreassen 2017, SMO: filter boundary conditions and eroded designs detaching at boundaries.
- Cool, Aage, Sigmund 2025, arXiv 2501.09402: connectivity review.
- Amir, Aage, Lazarov 2014 and Ferrari & Sigmund 2020: multigrid CG for TO.
- Svanberg 1987 / 2007: MMA.
- Research notes behind the choices: `claude/research/topo-engine-2026-10-22.md`.
