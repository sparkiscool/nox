bracket_jA organic finish: final run organic-v3 (demo run, not saved to print/)

Engine nox/topo, branch topo/new-engine, `Settings.finish = "organic"` (default). Setup geometry and loads unchanged.

## Result (organic-v3)

| Item | Value |
|---|---|
| Mass | 36.82 g (M); old v8 38.37 g (-1.55 g); topo-v3 (drawn finish) 36.04 g |
| Worst index, same grid (topo.compare) | 0.108 in 'ring up + fore 30 N' (old v8 0.133, topo-v3 0.125) |
| Solid check | PASS (nox/solid_check.py on the exact STL): 1 body, 1 eroded body, 0 necks, 0 thin walls |
| Function check | ok: 0 mm3 skin missing, 0 in cuts, 0 outside the envelope |
| Surface | generated material is one smooth beam blended into the cheeks and the flange with 2.5 mm fillets, no voxel steps (a section shows a smooth profile). Geometric only on cut faces: bolt holes, access bores, lug faces, wall face, and the ring keep-out faces that wrap the cheeks |

## What changed and why

1. **organic-v1** (finish as committed in dcaceae). Solid and function pass, but it fails the rule: terraces on every
   generated face (distances taken from binary voxel masks), sharp skin rims, a creased flat face where the field was
   clipped off the envelope, and 43.5 g. The mass target `gen` had been raised to 20 cm3, which filled 85 % of the
   design space, so the part was a solid box beam.
2. **Engine change A** (commit a707063): all build distances come from fast marching (scikit-fmm) on continuous fields
   (the cubic-upsampled filtered density, blurred masks), so planes and curves have no steps. The keep-out clip is a
   smooth max, not a crease. The skins' free rims are rounded (`skin_round_mm` 1.0, an opening). The function check
   holds the skins minus that rounded rim. `gen_cm3` now means the same in both finishes (volume added to the drawn
   bodies as given; the layout puts back the drawn volume it opened), so bracket_jA is back to its own 12 cm3.
   The preview uses smooth shading.
3. **organic-v2**: smooth and organic, 34.6 g, but same-grid worst 0.143. The build came out 6 % under the layout's
   own design volume (the smooth-max pull at keep-outs plus the rim rounding), so the members were thinner than what
   the optimiser had analysed.
4. **Engine change B** (commit 375fb80): the build keeps the layout's volume. One level offset of the generated field
   (+0.42 mm here), found by bisection and applied before the keep-out clip so the clip margin holds. The keep-out
   blend is 2 x the clip margin.
5. **organic-v3**: 36.8 g, worst 0.108. Final.

## Runs

| Run | gen cm3 | Mass g (M) | Worst, same grid (old 0.133) | Solid check | Function check | Flat free surface | Wall time |
|---|---|---|---|---|---|---|---|
| organic-v1 | 20 | 43.45 | 0.115 | PASS | ok | 445 mm2 (8.4 %) + terraces | 633 s |
| organic-v2 | 12 | 34.58 | 0.143 | PASS | ok | 58 mm2 (1.3 %) | 614 s |
| **organic-v3** | 12 | **36.82** | **0.108** | PASS | ok | 101 mm2 (2.2 %) | 610 s |

Peak memory 2.39 GB for each run. "Flat free surface" counts surface that is not on a cut or envelope face and is
planar within 3 deg over a 4 mm patch.

## One remaining flat spot, and why the engine cannot remove it

v3 has two flat patches of about 50 mm2 each on the underside at z -63.6, where the beam roots meet the envelope's
bottom face (z = zc - 20 = -64.5). In both spots the optimised density is 1.0 in the first design cell next to the
envelope (rho_n at z -64.0 is 1.00). The optimiser wants material past the envelope, so the surface there is the
envelope limit held 0.5 mm off. The finish can only round its edges. Removing the patch needs a deeper envelope (a
part setup change) or less material at the most loaded root.

## Notes
- Self-test: 14 / 15. The end-to-end holder smoke test (h 2.0, 60 iterations) builds 2 bodies because one corner
  bolt boss is never joined. It fails the same way with `finish=drawn`, so the organic finish did not cause it.
- trimesh reads the STL as not watertight because of 4 edges shared by 4 faces at (167, 205.7, -66.5), inside a boss below the
  envelope (topo-v3 has the same kind). manifold3d reports 1 body and solid_check passes.
