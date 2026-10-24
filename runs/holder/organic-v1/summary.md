holder organic-v1: demo run, not saved to print/

| Item | Value |
|---|---|
| Settings | organic finish (default), `--gen 4.7`, held ties (new), min member 4.44 mm (gate wants 5.0; the design space routed at most 4.44 to the pad) |
| Setup | bosses 10 mm round (was 6 x 6 square, 1.4 mm walls) |
| Mass | 10.15 g (M); topo-v1 10.26 g |
| Worst index (as built) | 0.65 in 'misuse 6' at (22.5, 6.5, 20.5), the battery channel's top edge (topo-v1 0.498, same spot class) |
| Solid check | FAIL: 1 eroded body at 0.9 mm, but at the per-body 2.0 mm erosion the 4.44 mm members vanish and 4 boss / hook cores sit outside the main core |
| Function check | ok (skins complete, 0 in cuts, 0 outside the envelope) |
| Wall time | 3502 s (coarse 185 s), peak 3.76 GB |

Look: right. Distinct round struts and branches with open space between them, an arch over the battery channel,
smooth fillets into the round bosses, hooks and camera pad; no slabs, no voxel steps. Still geometric: the pad block
(keep-out faces), the flat hook and camera plates.

What is wrong: the pad sits on a 4 mm layer between the keep-out under the pack (z 26) and the battery channel
(top z 22). Every member out of the pad is a plate at most 4 mm thick there, which the gate's 2.0 mm erosion removes,
and the worst index sits in that layer. Fix (setup, DEFAULT): keep-out under the pack starts at z 27 (5 mm layer).
