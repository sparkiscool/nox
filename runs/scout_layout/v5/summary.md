# Scout v3 system pass v5: review 2 fixes, refit to the latch's receiver v3 (24 Oct)

`python3 nox/scout/v3/layout_v2.py` (OBJ mass, ESO_SEED 2, latch meshes `runs/scout_layout/v5/latch_v3/` = part/battery-latch-v6 @ 4f76d3a, merged to main as PR 109; ~75 min), then `python3 nox/scout/v3/write_assembly.py && python3 nox/system_check.py nox/scout/v3/scout.assembly.json`. Doc `nox/scout/v3/layout.md` v5; numbers `nox/dimensions.md` "System Scout v3", "Scout fit". Status: DESIGN (WIP); nothing saved or printed.

```
System: see system_line.txt: cage first 4.6 mm, fit 0.0, CG 1.5 mm, access 8 / 8, mass 1007 g (E, budget 500 soft); probes FAIL 2 rows
Cage: 212 members: 142 x 3.0 carbon rods (straight runs; shortest 33.7 mm) + 40 printed (6 x 6 struts / 3 x 12 webs) in 54 printed parts; keel y +-29.6; ~474 g (E)
Strength: every case <= 1.00 (F); handling 50 N on any node (strength); crash 1.5 m/s through the TPU suspension
```

Open probe rows: receiver v3 tail-bottom corner (-29.0, -24.3, -44.3), edge from 41 deg off straight down (the swap opening is 40), -3.1; camera lower corner (92, 19, 22), edge from 45 deg below the front, -1.4 (its corner sits under the lower ring: a rod there runs into the camera).
Candidates weighed (all on the final meshes): cold mass seed 2 474 g (kept), seed 1 479 g; with keel tail x -29.5 + camera underside nodes 493 / 501 g, same 2 rows; earlier on receiver v2 / stand-ins: warm from v4 352 g but the rods 1 mm short in the swap cone; fewest-pieces cold 560 g.
Grams: the full cage is ~474 g against the 500 g whole-drone budget (bought + latch set + wires 492 g already, E): the extra buys cage first everywhere, every probe but 2, 50 N on any node and the keel that takes a bottom hit through the suspension.
- `checks.txt`, `result.json`, `system_line.txt`, `compare.html` / `.png` (v3 / v5, https://claude.ai/artifact/Dm8HQfFbyDoDSPbs6wmzNe), `cold/`, `warm/`, `cold_f9/` (candidates), `latch/`, `latch_v3/` (the latch meshes used).
