# Overseer cage nodes v5 (28 Oct 2026): drawn first, 2.0 walls

Spark 27 Oct: "are you telling me that overnight you only made two nodes" and "why were they doing [topology] after checks passed". The node topology runs (v1 to v3) took about an hour each and added 0.3 g to a 14.9 g drawn node (node 7, v3, M). The nodes are now saved as drawn (CLAUDE.md Drawn first): `python3 nox/overseer/cage_node.py <n> --drawn` puts each design's drawn part (sockets, Ø7 arms, Ø12 hub, minus the bores) through `solid_check` and the as-built strength check on the frame model's socket loads (`runs/overseer_frame/v1/node_loads.json`, every member found, F). Then `python3 nox/overseer/cage_nodes_save.py` orients, checks and saves each one, with its mirror. Picture: `sheet.png` (every print file).

**Spec (C, DEFAULT):**
- sockets: 6/4 tube bore 6.2, 15 deep; 3.0 rod bore 3.2, 10 deep (Spark 27 Oct, coupons v2);
- a 3.0 solid floor behind each bore;
- 2.0 boss walls (was 2.5; the rule floor);
- PETG.

v4 (2.5 walls, `runs/cage_nodes/v4/`) passed too at 345.1 g (M); 2.0 walls save 55.7 g (M) with the worst index still 0.252. Shorter 12-deep 6 mm sockets would save about 30 g more (C), but they would raise the worst socket's epoxy shear from about 5.0 to 6.2 MPa (E): kept at 15.

**Result:** 24 print files, 289.4 g PETG (M) in all; every file solid_check PASS. Worst index 0.252 (node 17, head rod c side). Each prints with a socket mouth on the bed (a 51 mm² annulus, M): no thin-island run over 0.3 mm, tree supports under the side sockets, 5 mm brim (bed contact under 400 mm²).

| Node | Design | File | g (M) | solid | worst index (F loads) | bed | overhang mm² | extents mm |
|---|---|---|---|---|---|---|---|---|
| 0 | 4 | `print/overseer_cage_node_0_v5.stl` | 8.99 | PASS | 0.157 (tube to 11 pull) | the mouth of socket 0 (tube to 3, mirrored), 51 mm² | 1308 | 32.9 x 48.6 x 50.9 |
| 1 | 3 | `print/overseer_cage_node_1_v5.stl` | 12.7 | PASS | 0.049 (tube to 11 pull) | the mouth of socket 3 (tube to 18, mirrored), 51 mm² | 1456 | 36.3 x 62.1 x 62.5 |
| 2 | 2 | `print/overseer_cage_node_2_v5.stl` | 9.7 | PASS | 0.036 (tube to 18 pull) | the mouth of socket 2 (tube to 18), 51 mm² | 1209 | 25.4 x 55.4 x 55.0 |
| 3 | 3 | `print/overseer_cage_node_3_v5.stl` | 12.7 | PASS | 0.049 (tube to 11 pull) | the mouth of socket 3 (tube to 18), 51 mm² | 1456 | 36.3 x 62.1 x 62.5 |
| 4 | 4 | `print/overseer_cage_node_4_v5.stl` | 8.99 | PASS | 0.157 (tube to 11 pull) | the mouth of socket 0 (tube to 3), 51 mm² | 1308 | 32.9 x 48.6 x 50.9 |
| 5 | 5 | `print/overseer_cage_node_5_v5.stl` | 10.89 | PASS | 0.065 (tube to 13 pull) | the mouth of socket 1 (tube to 4), 51 mm² | 1403 | 32.3 x 58.0 x 56.4 |
| 6 | 6 | `print/overseer_cage_node_6_v5.stl` | 10.58 | PASS | 0.02 (tube to 15 pull) | the mouth of socket 1 (tube to 10), 51 mm² | 1201 | 29.5 x 36.7 x 59.4 |
| 7 | 7 | `print/overseer_cage_node_7_v5.stl` | 13.01 | PASS | 0.01 (tube to 15 pull) | the mouth of socket 0 (tube to 6), 51 mm² | 1475 | 32.2 x 51.3 x 57.7 |
| 8 | 6 | `print/overseer_cage_node_8_v5.stl` | 10.58 | PASS | 0.02 (tube to 15 pull) | the mouth of socket 1 (tube to 10, mirrored), 51 mm² | 1201 | 29.5 x 36.7 x 59.4 |
| 9 | 10 | `print/overseer_cage_node_9_v5.stl` | 12.29 | PASS | 0.054 (tube to 6 pull) | the mouth of socket 4 (tube to 21, mirrored), 51 mm² | 1527 | 42.9 x 69.5 x 42.6 |
| 10 | 10 | `print/overseer_cage_node_10_v5.stl` | 12.29 | PASS | 0.054 (tube to 6 pull) | the mouth of socket 4 (tube to 21), 51 mm² | 1527 | 42.9 x 69.5 x 42.6 |
| 11 | 11 | `print/overseer_cage_node_11_v5.stl` | 20.91 | PASS | 0.012 (tube to 10 pull) | the mouth of socket 8 (core tie left.1), 51 mm² | 2410 | 66.3 x 84.6 x 51.0 |
| 12 | 11 | `print/overseer_cage_node_12_v5.stl` | 20.91 | PASS | 0.012 (tube to 10 pull) | the mouth of socket 8 (core tie left.1, mirrored), 51 mm² | 2410 | 66.3 x 84.6 x 51.0 |
| 13 | 13 | `print/overseer_cage_node_13_v5.stl` | 15.41 | PASS | 0.018 (tube to 5 pull) | the mouth of socket 5 (core tie rear.1), 51 mm² | 1706 | 45.9 x 77.2 x 47.2 |
| 14 | 13 | `print/overseer_cage_node_14_v5.stl` | 15.41 | PASS | 0.018 (tube to 5 pull) | the mouth of socket 5 (core tie rear.1, mirrored), 51 mm² | 1706 | 45.9 x 77.2 x 47.2 |
| 15 | 15 | `print/overseer_cage_node_15_v5.stl` | 12.13 | PASS | 0.129 (tube to 6 pull) | the mouth of socket 2 (tube to 16), 51 mm² | 1229 | 47.8 x 50.6 x 59.2 |
| 16 | 15 | `print/overseer_cage_node_16_v5.stl` | 12.13 | PASS | 0.129 (tube to 6 pull) | the mouth of socket 2 (tube to 16, mirrored), 51 mm² | 1229 | 47.8 x 50.6 x 59.2 |
| 17 | 17 | `print/overseer_cage_node_17_v5.stl` | 9.48 | PASS | 0.252 (head rod c side b) | the mouth of socket 0 (tube to 15), 51 mm² | 1197 | 46.8 x 35.9 x 53.0 |
| 18 | 18 | `print/overseer_cage_node_18_v5.stl` | 15.57 | PASS | 0.121 (tube to 15 pull) | the mouth of socket 1 (tube to 3), 51 mm² | 1896 | 42.9 x 61.9 x 50.9 |
| 19 | 18 | `print/overseer_cage_node_19_v5.stl` | 15.57 | PASS | 0.121 (tube to 15 pull) | the mouth of socket 1 (tube to 3, mirrored), 51 mm² | 1896 | 42.9 x 61.9 x 50.9 |
| 20 | 20 | `print/overseer_cage_node_20_v5.stl` | 7.36 | PASS | 0.036 (tube to 13 pull) | the mouth of socket 0 (tube to 9), 51 mm² | 822 | 15.3 x 46.1 x 46.2 |
| 21 | 21 | `print/overseer_cage_node_21_v5.stl` | 9.61 | PASS | 0.013 (tube to 20 pull) | the mouth of socket 1 (tube to 9), 51 mm² | 1103 | 16.2 x 54.4 x 54.0 |
| 22 | 23 | `print/overseer_cage_node_22_v5.stl` | 6.07 | PASS | 0.04 (tube to 7 pull) | the mouth of socket 2 (tube to 10, mirrored), 51 mm² | 634 | 13.1 x 46.2 x 45.5 |
| 23 | 23 | `print/overseer_cage_node_23_v5.stl` | 6.07 | PASS | 0.04 (tube to 7 pull) | the mouth of socket 2 (tube to 10), 51 mm² | 634 | 13.1 x 46.2 x 45.5 |

The 9 mirror nodes are their own files (a mirrored part); node 17 is a cage node again (the 3-rod head holds none). In the assembly (`overseer.assembly.json`, cage shell nodes) as their meshes: system_check PASS (cage first 5.2, fit, balance, probes, access).

Carbon: none in this PR (the nodes take the cage's 6/4 tubes and the links, `runs/overseer_layout/v5/summary.md` Carbon). Visual: https://claude.ai/artifact/VhDfnp75uoVsj8SnyrW2Qg (the layout with every saved node).
