holder organic-v2: demo run, not saved to print/

| Item | Value |
|---|---|
| Settings | organic finish, `--gen 5.3`, min member 5.0 mm (gate_member), held ties |
| Setup | bosses 10 mm round; keep-out under the pack from z 27 (5 mm layer under the pad) |
| Mass | 10.93 g (M) |
| Worst index (as built) | 0.529 in 'misuse 6' at (22.5, 7.5, 21.5), the battery channel's top edge (0.517 after the marching-cubes fix, same layout) |
| Solid check | FAIL: at the per-body 2.0 mm erosion the part falls into 30 pieces; plus 5 voxel bodies (slivers on the channel top) |
| Function check | ok |
| Wall time | 3333 s, 92 iterations |

Look: right. Thin round struts and branches, an arch over the battery channel, open space, smooth fillets into the
round bosses, hooks and camera pad. Small spikes under the pad (marching-cubes artefacts, below).

What is wrong, with the engine fix for each:
1. Members away from the drawn bodies build at 4.0 mm median against the 5.0 mm blueprint minimum, so the gate's
   2.0 mm erosion removes them. Fix: `gate_member_mm` divides by the 0.8 built / blueprint ratio (6.25 mm members).
2. Exact-zero samples of the build field gave 52 degenerate marching-cubes faces (27 shells); Taubin pulled spikes
   off them through the keep-out margin and the exact cut left slivers on the channel top. Fix: samples within 1e-4
   of the level are nudged outside. Rebuild of this layout: 1 shell, 1 voxel body, 0 acute edges, worst 0.517.
