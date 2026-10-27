# Overseer: L2 on the cage, concept (27 Oct 2026, Spark: "lets try to mount the lidar on the cage of the drone so we dont need the rods coming from the body")

Script `nox/overseer/head_cage_concept.py` (frame model F on E properties; `nox/frame_model.py`). Picture `concept.png` (side view of each option, struts to the core red, head-to-cage struts green). Data `result.json`.

Cases, core fixed: nose hit (the drone's 3178 g at 35 g into the head where its guard leads), the head's own 360 g at 35 g in 14 directions at its CG, crash 35 g through the first-touching cage node, flight 4 g. Head first mode: the head as a rigid body (L2 230 g at its CG with its own inertia, cameras 2 x 15, mount 100, E) on the frame's 6 x 6 stiffness at the head. Two joint bounds: rigid nodes, and TPU nodes (each tube end 6 short of the node centre on a TPU 95A stub Ø11.3, E 26 MPa, E).

| Option | Rods from the body | Head mode rigid / TPU (Hz, F on E) | L2 move, nose hit, TPU (mm) | Head carbon (g, E) |
|---|---|---|---|---|
| A as v5: 2 keels + 4 L2 braces | 6 | 200 / 119 | 0.5 | 47 |
| B cage only (front node + 2 wing struts) | 0 | 85 / 37 | 11.7 | 7 |
| C cage only + 3 top struts (195, 372, 372) | 0 | 204 / 50 | 6.0 | 30 |
| D cage only + centre top strut (195) | 0 | 158 / 49 | 6.3 | 12 |
| F cage only + 4 struts (top, 2 top sides, loop front) | 0 | 206 / 51 | 5.9 | 26 |
| G the 2 keels only | 2 | 102 / 44 | 7.2 | 21 |
| **H the 4 L2 braces only (DEFAULT)** | **4** | **187 / 101** | **0.6** | **37** |

Target: sensor mounts first mode ≥ 100 Hz (L, DEFAULT).

**Result.** With TPU nodes, every cage-only option stays near 50 Hz, however many cage nodes the head holds. The cage itself sits soft on its TPU joints, so more struts to it do not help. A cage-only head is half the target stiffness and moves 6 mm in a nose hit. The body rods make the lidar stiff: the 4 L2 braces alone reach 101 Hz, while the 2 keels alone reach 44.

**Spark 27 Oct:** "just do it with the least amount of rods possible" (Z below: the front node only, 32 Hz TPU, 15 mm nose hit), then "nevermind just keep the design we have right now i need the lidar to have max performance": **A stays** (2 keels + 4 L2 braces, the stiffest). Z kept as `V5_HEAD_ON=cage`.

| Z front node only, no struts | 0 | 65 / 32 | 15.3 | 0 |

Earlier DEFAULT (superseded): H. The 2 keels go and the 4 L2 braces stay: 4 rods from the body instead of 6, at 101 Hz, which only just meets the target. Alt: cage-only D (1 strut, ~49 Hz, E) if Spark waives the 100 Hz for the L2. Alt: keep A (119 Hz, 6 rods).

**Frame model fix found here.** `Frame.solve` regularised every free dof with 1e-9 x the mean diagonal. The stiff "rigid" head links (E 1e8) pushed that mean up, so every node sat on ~100 N/mm to ground and frame model v1 read low. It now uses 1e-10 x the median diagonal. Converged: rigid-link E 1e5..1e8 gives the same worst member (2.32) and head mode (200 Hz) on A. `cage_frame.py` rerun (`runs/overseer_frame/v1/`): 9 more links to 6/4 tube, then 6 members still buckle in the 35 g single-node crash: core ties left.1 / right.1 2.17 (L 276), left.2 / right.2 1.32 / 1.18 (L 337), shell tubes 11-18 / 12-19 1.15 (L 417). All are already 6/4 tube. Next: split the 2 tubes and shorten the 4 core ties, then rerun until none (the system pass).

Also found: the flat top of the cage (node 20 and its ring) has a ~26 Hz mode on TPU joints (F on E). It holds no sensor, so it is reported, not gated.
