holder organic-v11: final run, all four targets met (demo, not saved to print/)

| Item | Value |
|---|---|
| Engine | final: organic finish, held ties, MC zero fix, gate member 6.25 mm, case_norm solid, held bodies whole |
| Settings | `--gen 5.4`, min member 6.25 mm |
| Setup | `demos/holder.py` at 611f4e8: 11.5 mm bosses centred 0.75 mm toward the part, keep-out under the pack from z 29, battery channel top long edges rounded r 3 mm (approved by Spark 23 Oct) |
| Mass | 11.73 g (M): +14 % on topo-v1's 10.26 g (target within about 15 %) |
| Worst index (as built) | 0.493 in 'misuse 10' / 'misuse 12' at (28.5, 7.5 / 32.5, 21.5) (target <= topo-v1's 0.498); every other case <= 0.386 |
| Solid check | **PASS** (`nox/solid_check.py` on the exact STL): 1 body, 1 eroded body, 0 necks, 0 thin walls, 0 acute edges |
| Function check | ok: skins complete, 0 in cuts, 0 outside the envelope |
| Wall time | 2677 s fine stage here, 121 iterations, peak 3.52 GB. The coarse stage (71 iterations) is the run-only session's, pushed to this folder; this session resumed it (log.txt holds both headers) |

Look: thin round members with open space between them: four legs from the bolt bosses up to the pad, a ring round
the pad, rails along the battery channel to both hooks, a fork to the camera pad; every member blends into the bosses,
pad, hooks and camera pad through smooth fillets. Geometric only: bolt holes and counterbores, the pad, hook and camera
contact faces, and the boss faces the envelope cuts. No slabs, no voxel steps.

Margin note: the worst index clears its target by 0.005 and still sits at the bend over the battery channel's top
edge (now rounded r 3 mm); runs of the same setup spread by about 0.03. organic-v12 (gen 6.0, run-only session)
reads 0.481 at 12.5 g.
