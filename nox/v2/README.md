# Scout v2: scope and parameters (8 Oct 2026, trimmed 13 Oct, cleanup 14 Oct, cage all PETG 17 Oct)

Read `nox/README.md` first for the shared rules. For a Scout v2 part, where the two differ, this page wins.

## What v2 is (Spark, 8 Oct; battery under the drone since 14 Oct)
- The lightest Scout possible; every printed part and the guard designed new.
- Battery under the drone in a cartridge, installed from below by the box swap arm, held by the S2 bottom latch; connects through contact pads, not the XT60.
- Motors and props on top of the arms (14 Oct).
- One crash: the battery, FC, camera, Radxa and motors come out safe. Nothing has to survive a second crash.
- Frame: the cut-down Bee35 arm plate stays (printed frames dropped, `claude/printed-vs-carbon-frame-research.md`). Over ~22 g (E) bare, quote a thinner cut plate.

## House rules for v2
1. v2 docs and scripts live under `nox/v2/` in the repo, the only source (`nox/README.md` Cloud sessions); no PC paths. Meshes are rebuilt per chat from the part script. Versions are never reused: if your version's name exists, another chat made it, take the next number.
2. Names start with `s2_` (`s2_underside_v2.stl`); in talk and logs "S2 underside v2".
3. No old geometry seeded into a v2 run. An old mesh is only a mating part in a fit check.
4. The engine (`noxopt.py`, `printopt.py`, `render_preview.py`) isn't edited for v2. Settings go in the part's own spec script. An engine change is an option whose default keeps old behaviour, plus `test_noxopt.py`, plus a line in `v2/log.md`.

## Priorities
1. It works: flies, swaps its battery, sends video, props stay enclosed.
2. Lightest.
3. One crash: battery, FC, camera, Radxa and motors safe. Guard and printed parts may bend or break.

## Loads
| Case | Load | Sized for it? |
|---|---|---|
| Flight | 4 g vertical + 2 g lateral | every part |
| Landing and catch | 6 g | every part |
| Vibration | ±2 g, first mode outside 80 to 350 Hz | parts carrying a sensor, camera or board |
| Crash inside the guard | 25 g, only on battery retention and on any part that could reach the battery, FC, camera or Radxa if it broke | those; every other part reported only |
| Guard hit | 1.0 m/s once: rods and nodes may yield or break; inside ≤ 25 g; props not reached (DEFAULT) | the guard |
| Poke on a free feature | 5 N (DEFAULT) | yes |
| Hand pinch on a part handled in a swap | 20 N (DEFAULT) | yes |
| Battery retention | pack 165 g (P) at 25 g = 40.5 N across all retention features, plus contact preload | yes, never breaks first |
| Contact preload | ≥ 12 N on the pack, always on (E); check creep | yes |

Pass line: failure index ≤ 1.0 on every sized case, no extra factor. Poke, pinch and the 25 g inside limit are the first to lower if a part is still heavy; that's Spark's call.

## Thickness and materials
- **Thickness (changed 16 Oct 17:20, Spark: no hard limits if they leave a generated part with disconnected walls; the layout decides).** No 2.0 class. The only floor is what the printer can make: **1.2 mm (3 perimeters at 0.42 on the A1 mini, DEFAULT)**; flexures excepted. Above that every member is sized by the part's load cases (index ≤ 1.0). The 2.0 rule had forced the tray's wall band to 2 cells at h 1.0, where the optimiser can only make 1-cell sheets, and the post-process then tried to enforce 2.0 on a layout that never had room for it (`v2/s2-cartridge.md`, runs 1 and 2).
- **Connectivity is the hard rule.** A generated part is ready only when it is one body voxelised at 0.25 mm AND still one body with everything under the 1.2 floor removed (erosion test, `tray_to.py` `print_check`). `trimesh.split` alone does not count: run 2 read one body there and 20 at print resolution.
- **Function first (16 Oct 17:51 / 17:54, Spark: too hard on weight, it lost function).** Every functional surface is drawn and never optimised: guides and lead-ins, locating datums, contact bands and their backing, latch features, bay rail paths, snap-lip edges, pads that take a mating load. Topology only fills what is left, and every place a part is touched in use gets its own load case (insertion and capture side loads, contact forces as sustained, every support it sits in: drone, gripper, bay). A part that saves grams by losing one of these is a fail, whatever its index.
- **Thin-walled parts are plates:** a wall's design domain is a plate of fixed thickness extruded through its thickness (one variable per in-plane cell, through `pb.sym`), so topology picks the hole pattern only and every member is the full plate thick. Pattern: `v2/tray_to.py` run 4.
- **Room for the layout:** a shell's design band is at least 3 cells wide (band / h ≥ 3), or the band is widened; with the plain (unmasked) filter so a member flush against a keep-out is eroded and must sit inside the band at full width. Polish floor and thin check at the printability floor; clip margin 0 on shells.
- Rigid: PETG Basic (DEFAULT), best Z strength per gram on the A1 mini. PETG-CF only on a stiffness-sized part once a hardened nozzle is fitted; trial part = S2 camera mount (DEFAULT). On strength-sized parts it's ~20 % heavier (E). PETG HF not on flying parts (Z tensile 23 vs 39). PLA family, nylon-CF, PET-CF out. Detail: `claude/bambu-materials-research.md`.
- **Cage: every printed part PETG Basic (Spark 17 Oct 10:39): corner nodes, tees, hubs, feet if any.** No TPU on the cage. Tee and foot sockets get the PETG socket spec (bore rod + 0.3, 0.6 lead-in, epoxied); the hit energy goes into rod bending and the 1.0 m/s "may break" allowance, not into a soft node. Rods: carbon, epoxied.
- Bumpers, crush pads, latch preload strip: Bambu TPU 95A HF, fed by hand (DEFAULT).
- Material card: each spec carries its own from the Bambu datasheet (PETG Basic: modulus 1460, Z 1120; tensile 48, Z 39 MPa). The PETG card inside `printopt.py` reads ~1.9x too stiff (CHECK); don't use it for deflection or frequency.

## Optimizer settings (`noxopt.py`)
| Setting | Value |
|---|---|
| Cell size h | 1.0 or finer; small parts 0.5; members at least 2 cells wide and a shell's design band at least 3 cells (16 Oct) |
| Filter radius | 2.0 cells and at least 2.0 mm |
| Robust thresholds | 0.7 / 0.5 / 0.3 |
| Case weighting | p-mean P 4, tie cases x 0.25 |
| Load cases in the layout | all sized cases; "reported only" cases run in the recheck |
| Mass target | generated volume in cm³ from the part's budget, then the search below |
| Design space | open from the start: only real keep-outs (props + 12, battery path, camera view, mating parts); 2.0 empty skin at grid edges |
| Overhang filter | on for flat no-support parts, off on tree supports |
| Stop | ≤ 2 % grey and settled; 6 h cap |
| Smooth build | sigma ~0.6 x h, volume kept, clip 1.0 inside keep-outs (0 on shells); polish floor = printability floor 1.2 (16 Oct); through-holes ≤ 40 mm² filled and reported |
| Reinforcement | hot cells, at most 3 passes |
| Pass line | 1.0, and one body at print resolution with everything under 1.2 removed (16 Oct) |

## Search for the lightest (every generated part)
1. Start volume = the part's budget in `v2/current-parts.md` minus its drawn features.
2. Run, build, recheck as built.
3. Every sized case ≤ 1.0: cut generated volume 15 %, run again.
4. A case over 1.0 after 3 reinforcement passes: stop; the run before is the answer.
5. At most 4 runs per chat, then show Spark one table: run, grams, worst index, where.
6. If drawn features are over half the mass, the interfaces are the limit: list the three costliest and what changing each would save, and ask Spark.
7. Print the lightest passing run. Too flimsy in Spark's hand: one step back, and that part's hand load goes up in its doc.

## Mass budget (E unless tagged)
| Group | Budget g |
|---|---|
| Battery 4S 1500 | 165 (P) |
| Battery holder: cartridge + latch hooks + preload strip + camera mount | 21 (rewrite after the bottom layout, `v2/current-parts.md` Problem 5) |
| Contacts: spring pins + pad strips | 6 |
| Guard | 55 |
| Underside (Radxa carrier, flow sensor seat) | 8 |
| Frame plate, motors, props, FC, camera, Radxa, BEC, wiring, belly plate | unchanged from v1 + belly plate (E) |
| **All-up** | **~400** (v1 ~520) |

A part under budget gives its grams back to the total, not to another part. Per-part budgets, weights and next steps: `v2/current-parts.md`. One chat per part.
