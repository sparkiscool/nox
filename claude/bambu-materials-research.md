# Bambu materials for Nox parts (8 Oct 2026)

Research for Spark: which Bambu filament for which drone part. Printer: A1 mini, open frame, stainless nozzle (hardened nozzle not fitted yet). All values from Bambu technical data sheets (100 % infill test bars, annealed/dried), tagged E for our use: real parts print weaker and the as-built recheck stays the judge.

## What the A1 mini can run
- Bambu's own list for the A1 mini, stainless nozzle: PLA, PETG HF, TPU 95A HF, PVA. Hardened nozzle adds TPU 90A, PLA-CF, PETG-CF, PLA Glow. Not on the AMS lite: PETG-CF, TPU 95A HF, PLA Glow.
- Open frame, 80 C bed: ABS, ASA, PC, nylons and the -CF engineering grades (PAHT-CF, PET-CF, PPA) are out. Already ruled out in `v2/README.md`.
- PETG Basic and TPU for AMS are not on Bambu's A1 mini page but print fine here (P: every v1 part and the cage v3 nodes).

## Datasheet numbers (E)
| Material | Density g/cm3 | E XY / Z MPa | Tensile XY / Z MPa | Elong. XY / Z % | Impact XY / Z kJ/m2 | HDT 0.45 MPa C |
|---|---|---|---|---|---|---|
| PETG Basic | 1.25 | 1460 / 1120 | 48 / 39 | 11.2 / 5.7 | 52.7 / 13.6 | 69 |
| PETG HF | 1.28 | 1810 / 1540 | 34 / 23 | n/a | 31.5 / 10.6 | 69 |
| PETG-CF | 1.25 | 2460 / 1340 | 35 / 29 | 10.4 / 4.7 | 41.2 / 10.7 | 74 |
| PLA Basic | 1.24 | 2680 / 2160 | 39 / 35 | n/a | n/a / 13.8 | (PLA family ~55) |
| PLA Tough | 1.26 | 2330 / 2110 | 44 / 39 | n/a | n/a / 12.3 | ~55 |
| PLA Tough+ | n/a | n/a | 34.9 / 20.9 | n/a | 80.6 / 25.9 | ~55 |
| PLA-CF | 1.22 | 2790 / 2160 | 38 / 26 | n/a | n/a | ~55 |
| PLA Aero | 1.21 filament, prints 50-80 % | 2150 / 1270 | 26 / 14 | 2.2 / 0.9 | 24.5 / 2.5 | 54 |
| TPU for AMS (68D) | 1.26 | 1190 / 600 (looks high, CHECK) | 22.4 / 11.2 | >650 / 31 | 124 / 9.6 | n/a |
| TPU 95A HF | n/a | 9.8 / 7.4 | 27.3 / 22.3 | n/a | 123 / 86.3 | n/a |
| TPU 90A / 85A | n/a | n/a | 12.5 / 12.0 | >650 / >700 | n/a | n/a |

## Per gram (E)
| Material | Strength/density XY / Z | Stiffness/density XY / Z | Mass vs PETG Basic, strength-sized beam (Z) | Mass vs PETG Basic, stiffness-sized beam (XY) |
|---|---|---|---|---|
| PETG Basic | 38.4 / 31.2 | 1168 / 896 | 1.00 | 1.00 |
| PETG HF | 26.6 / 18.0 | 1414 / 1203 | ~1.45 | ~0.92 |
| PETG-CF | 28.0 / 23.2 | 1968 / 1072 | ~1.22 | ~0.77 |
| PLA Tough | 34.9 / 31.0 | 1849 / 1675 | ~1.01 | ~0.80 |
| PLA-CF | 31.1 / 21.3 | 2287 / 1770 | ~1.26 | ~0.71 |

Beam scaling: strength-sized mass ~ rho / sigma^(2/3), stiffness-sized mass ~ rho / E^(1/2).

## Findings
1. **Our generated parts are sized by failure index (strength, layer-aware), so PETG Basic is the lightest rigid choice.** It has the best Z strength per gram of anything the A1 mini prints, plus the best Z ductility and impact among the rigid options.
2. **PETG-CF only pays on stiffness-sized parts.** It is ~1.7x stiffer per gram in XY but its tensile is 27 % lower (35 vs 48) and Z 26 % lower. On a strength-sized part it comes out ~20 % heavier; on a part sized by deflection or a vibration mode it comes out ~20 % lighter (XY loads). Needs the hardened nozzle; not on the AMS lite.
3. **PETG HF is a speed filament, not a structural one.** Z tensile 23 vs 39: ~45 % heavier for the same Z strength. Fine for box jigs and test fits, not for flying parts.
4. **PLA family stays out of the drones.** PLA Tough matches PETG Basic per gram on paper, but HDT ~55 C (warm packs after flight, Radxa heat, a car in summer) and brittle crash failure. PLA Aero is the UAV filament on Bambu's page, but Z elongation 0.9 % and Z impact 2.5 kJ/m2: it cracks between layers in a hit, and it is for skins of RC planes, not load paths. Nox has no skins.
5. **TPU for AMS (68D) is a semi-rigid material, not a bumper.** Datasheet E 1190 XY is near PETG (CHECK: looks high for 68D; the 1 kg push test on the v8c cage settles it). Its Z is the weak point: tensile half of XY, Z elongation 31 %, Z impact 9.6 vs 86 for 95A HF. Cage nodes should carry socket loads in XY; a socket pulled along Z is where it splits.
6. **TPU 95A HF is the energy absorber.** Z impact 86 kJ/m2, near-isotropic strength. Prints on the stainless nozzle, but not through the AMS lite. TPU 90A / 85A are softer still but need the hardened nozzle on the A1 mini.
7. **PETG Basic card check:** the v2 card (1460 / 1120, 48 / 39) matches the current datasheet V2.0 exactly.

## Split (DEFAULT)
| Part | Material | Why |
|---|---|---|
| Scout v2 holder, pack frame, latch block | PETG Basic | strength-sized, battery retention never breaks first, contact preload is a sustained load near a warm pack (HDT 69) |
| Scout v2 latch spring | steel feeler blade (as V6) | |
| Scout v2 camera bracket | PETG Basic now; **PETG-CF trial once the hardened nozzle is fitted** | vibration mode (80-350 Hz keep-out) is stiffness-sized: the one part where PETG-CF saves mass |
| Scout v2 Radxa mount, underside | PETG Basic | strength-sized (pokes, press-on), Radxa heat |
| Scout v2 cage nodes | TPU for AMS | cage v10 layout is sized on its stiffness; keep socket axes in XY |
| Scout v2 top bumpers, feet, any crush pad | TPU 95A HF | energy absorber, strong across layers; load by hand, not the AMS lite |
| Overseer catch frame nodes | PETG Basic | strength-sized catch loads |
| Overseer skid pads | TPU 95A HF | |
| Box catcher forks, long arms | PETG-CF candidate | v2 fork already at its stiffness limit; mass is not a box target, stiffness is |
| Box everything else, gripper flexure | PETG Basic | |
| Box jigs, fit tests | PETG HF | 2x print speed, strength not needed |

## Sources
- Bambu A1 mini filament compatibility: https://bambulab.com/en/filament/printers/a1-mini
- Bambu A1 mini store page (no enclosure materials): https://eu.store.bambulab.com/products/a1-mini
- PETG Basic TDS V2.0: https://www.additive-x.com/shop/mpattachments/file/viewonline/id/580/product_id/2330
- PETG HF TDS V1.0: https://botland.com.pl/img/art/inne/25540_karta_produktu.pdf
- PETG-CF TDS V3.0: https://polyalkemi.no/wp-content/uploads/2023/08/Bambu_PETG-CF_Technical_Data_Sheet_V2.pdf
- PLA Basic TDS: https://www.additive-x.com/shop/mpattachments/file/viewonline/id/494/product_id/2236
- PLA Tough TDS: https://polyalkemi.no/wp-content/uploads/2023/06/Bambu_PLA_Tough-Technical_Data_Sheet.pdf
- PLA Tough+ TDS: https://cdn.kspelectronics.in/wp-content/uploads/2026/07/19233822/Bambu-PLA-Tough-Technical-Data-Sheet.pdf
- PLA-CF TDS: https://www.additive-x.com/shop/mpattachments/file/viewonline/id/817/product_id/2578
- PLA Aero TDS V2.0: https://www.additive-x.com/shop/mpattachments/file/viewonline/id/632/product_id/2439
- TPU for AMS TDS: https://cdn.shopify.com/s/files/1/1339/4265/files/Bambu_TPU_for_AMS_Technical_Data_Sheet.pdf
- TPU 95A HF TDS: https://www.additive-x.com/shop/mpattachments/file/viewonline/id/823/product_id/2584
- TPU 85A / 90A: https://eu.store.bambulab.com/en-hr/products/tpu-85a-tpu-90a
