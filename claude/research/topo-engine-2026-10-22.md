# Topology engine research, 22 Oct 2026

Three literature sweeps were made before `nox/topo` was written: connectivity and length scale, solvers and open codes, and FDM / drone practice. This note keeps what decided the design, with the sources. Numbers marked E are estimates.

## 1. Why pads hung on necks

The robust eroded / nominal / dilated formulation only guarantees a length scale when all three designs share one topology. That holds per Zhou et al. 2015 and the Lazarov, Wang & Sigmund 2016 review.

Near non-design regions the eroded design detaches from frozen boundaries (Clausen & Andreassen 2017, SMO, doi 10.1007/s00158-017-1709-1). A pad frozen at 1 is such a boundary. Its full density props up any neck that touches it in the filter, so the neck survives erosion.

Tie loads in a p-mean are cheap to violate when the neck still conducts. Static compliance only ensures load-to-support connection when it is a target (Cool, Aage & Sigmund 2025, arXiv 2501.09402).

The old masked filter also treated keep-outs as outside the domain, so members flush with a cut never eroded.

**Chosen fix:** anchored-core erosion. Drawn skins erode in the eroded design, only cores and support anchors stay frozen, and loads move to the cores. A plain constant-normalised filter makes every void count as empty.

## 2. Length scale

| Item | Value and source |
|---|---|
| Robust thresholds | 0.75 / 0.5 / 0.25. 1D cone filter: a solid survives erosion if its width is >= 2 r (1 - sqrt(1 - eta_e)). Blueprint minimum = 2 r sqrt(eta_e - eta_n) = r. Trillet, Duysinx & Fernández 2021, arXiv 2101.08605. |
| 3D | Rods need a larger r than plates. `layout.calibration()` integrates the 3D cone kernel over a disc (rods) and a slab (plates) and finds both. |
| Geometric constraints | Zhou, Lazarov, Wang & Sigmund 2015, CMAME 293:266. Indicators exp(-c abs(grad x_tilde)^2) on the blueprint, c = r^4, eps 1e-6 to 1e-7, switched on once a topology exists. They cost about one filter transpose each. |
| Analytic parameters | Arrieta, Romano & Johnson 2025, arXiv 2507.16108 (not adopted yet). |
| Element size | nTop: element <= feature / 3, >= 250 iterations. Altair: min thickness default 3x element. |
| Printed wall | 3 perimeters (1.2 mm) is functional and 2 to 3 mm is load-bearing. PLA strength rises 51 % from 1 to 3 perimeters (Polymers 2025, PMC12251818). Nox rule: nothing under 2.0 mm, necks >= 3 mm. Hence the default min member of 3.2 mm. |

## 3. Connectivity options considered

| Method | Guarantee | Cost | Verdict |
|---|---|---|---|
| Virtual temperature, sinks on pads (Liu et al. 2015; Li et al. 2016; Hård et al. 2024) | Conductive path to every pad, even a 1-voxel one | 1 scalar solve + 1 adjoint | Needs a length scale on top. Kept as a future option. |
| Tie-load compliance on the eroded field | Load to support only | 0 extra solves | Used, with moments and anchored cores. |
| Spectral (Donoso et al. 2022/2023) | One component | Eigen-solve | Not needed. |
| MMC / geometry projection | Member thickness by bound, no connectivity | New engine | Poor fit. |
| Level set + thickness | Thickness | Full rewrite | Poor fit. |
| Stress constraints (da Silva, Beck & Sigmund 2019) | Thickens loaded necks only | +1 adjoint per case | Phase 2, <= 300k cells. |

## 4. Solvers on 4 vCPU / 16 GB

| Size | Pardiso direct | Matrix-free GMG-PCG | pyamg SA |
|---|---|---|---|
| 50k elements | 1-2 GB, 3-6 s (E) | 2-3 s per solve (E) | ~12 s (scaled from 64k = 15 s) |
| 200k | 5-8 GB, 15-40 s | 10-20 s | ~50 s |
| 500k | 15-25 GB, too much | 25-45 s | minutes |
| 1M | no | 45-90 s | marginal |

- **Recipe.** Galerkin coarse operators, damped Jacobi omega 0.6, 4 levels, direct coarse solve and warm start (Amir, Aage & Lazarov 2014). 128k elements ran 18 s per cycle in serial MATLAB. TopOpt_in_PETSc uses the same recipe at scale.
- **FP32.** Safe for the preconditioner only (arXiv 2604.26441).
- **Pardiso memory.** Fill is ~n^(4/3). openCFS measured 2.2M unknowns at 580 s on 8 cores.
- **Repos.** github.com/topopt/TopOpt_in_PETSc, github.com/odedamir/topopt-mgcg-matlab, github.com/PSLer/TOP3D_XL.

## 5. FDM and drone practice

- **Anisotropy.** PETG Z strength is about 40-50 % of in-plane, Z modulus about 90 %, and Z failure strain about 1/4 (Ultimaker PETG TDS: XY 38.5 / Z 19.0 MPa). PLA upright is 23-55 % weaker (von Windheim et al.).
- **Strength criteria.** Tsai-Wu or Hashin with a separate interlayer check (Mirzendehdel, Rankouhi & Suresh 2018; Kundu & Zhang 2023). The engine keeps the old in-layer / across-layer indices so results compare.
- **Bolt interfaces.** Pad OD >= 3 D (M3: 9-10 mm), edge web >= 2 mm, e/D >= 3, loads over the washer area, clamped face on the bed (NASA CR 19960029069; Ambrozkiewicz & Kriegesmann 2021, arXiv 2005.03398).
- **Overhangs.** Self-supporting costs only 0-7 % compliance at 45 deg (Zhao et al. 2017; Ansys docs). Strength drops on stair-stepped overhangs (Additive Manufacturing Frontiers 2025). Spark's rule allows tree supports, so no overhang filter in v1.
- **Drone TO literature.** Mostly static and isotropic with little physical testing. Keep motor-arm modes away from ~150 Hz (IMAV 2018). A frequency constraint is a candidate for v2.
- **Output stage.** SDF / implicit extraction with volume preservation (rho2sdf, arXiv 2512.06976). Re-check the build by morphological opening, and re-run voxel FE on the as-built part (DECODE / VOLCO).

## 6. Next candidates (not in v1)

1. Aggregated Tsai-Wu or interlayer stress constraint on the crash cases, for parts <= 300k cells.
2. Virtual-temperature connectivity with sinks on the pads, if a tie case is ever satisfied through a soft path.
3. An eigenfrequency constraint for motor arms and mounts.
4. An overhang filter (Langelaar 2017) with enumerated bed faces, for parts printed without supports.
5. An FP32 multigrid preconditioner and a numba stencil kernel, for grids above 1M cells.
