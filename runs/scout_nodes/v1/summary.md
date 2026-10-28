# Scout cage nodes v1 (28 Oct 2026, drawn; `nox/scout/v3/cage_node.py`)

Piece of **System Scout v3** (`dimensions.md` "System Scout v3": Rods, Nodes, Bottom, Landing, Funnel fit, Camera, Tie saddle; system model `nox/scout/v3/scout.assembly.json`, cage `runs/scout_layout/v12/cage_shell.json` + the 8 saddle ties of its `result.json` "links", frame model `runs/scout_frame/v3/node_loads.json`).

The brief:
- Spark 27 Oct: every Scout rod 3.0 solid carbon, socket bore 3.2 epoxied (the coupon pick); nodes PETG; perfect left-right mirror symmetry.
- Spark 27 Oct: the cage fails the 35 g crash on 3.0 rods, accepted (rods are the fuse); each node carries what its rods can deliver.
- Box catch F (PR 155): TPU foot caps Ø16 x 3 under the 4 ring nodes; the side ring rods keep a 6 mm sleeve zone over x ±32.
- Node 16: sockets toward +x within 5 of its centre (camera mount clips nearby; camera mount v2 running in parallel).

Visuals:
- Every node on the cage in 3D, each design's print picture, its section through the tightest socket pair, sockets and FE, the two options and the breaking order: https://claude.ai/artifact/WzJaJRF2Jwc721Gd9dYKsa. Source `nodes.html`, built by `nox/scout/v3/cage_node_sheet.py` (+ `cage_node_sheet.html`); sections `section_<i>.png`.

Previews (one per print file; a mirror print's picture is its design's, mirrored):
- `preview_scout_node_0_v1.png`: print/scout_node_0_v1.stl and print/scout_node_0_v1_mirror.stl
- `preview_scout_node_1_v1.png`: print/scout_node_1_v1.stl and print/scout_node_1_v1_mirror.stl
- `preview_scout_node_4_v1.png`: print/scout_node_4_v1.stl
- `preview_scout_node_5_v1.png`: print/scout_node_5_v1.stl and print/scout_node_5_v1_mirror.stl
- `preview_scout_node_8_v1.png`: print/scout_node_8_v1.stl and print/scout_node_8_v1_mirror.stl
- `preview_scout_node_10_v1.png`: print/scout_node_10_v1.stl and print/scout_node_10_v1_mirror.stl
- `preview_scout_node_11_v1.png`: print/scout_node_11_v1.stl
- `preview_scout_node_12_v1.png`: print/scout_node_12_v1.stl and print/scout_node_12_v1_mirror.stl
- `preview_scout_node_14_v1.png`: print/scout_node_14_v1.stl and print/scout_node_14_v1_mirror.stl
- `preview_scout_node_16_v1.png`: print/scout_node_16_v1.stl
- `preview_scout_node_17_v1.png`: print/scout_node_17_v1.stl
- `preview_scout_foot_cap_v1.png`: print/scout_foot_cap_v1.stl

## What it is
**18 PETG nodes in 11 designs** (one per mirror pair, the -y node is the `_mirror` print; one per centre-plane node 4, 11, 16, 17) **+ 4 TPU foot caps**: 119.9 g of nodes (M) + 3.4 g of caps (M) = 123.3 g, in place of the layout's 39 g (E) spheres.

Everything is drawn (the rule's drawn set: sockets with their bosses, the hub, the cap seat, the bed face); nothing generated:
- **Socket:** bore 3.2 x 10 deep (C), boss wall 3.0 round it (5.0 on tie 4.1 at node 14, 3.5 on tie 2.1 at node 12, where the FE asked), solid from its line's origin to the mouth so the floor behind it is ≥ 3.0 (C).
- **Set-back:** each socket starts only as far out as it must: every two bores keep the thicker of their two walls of PETG between them, and every rod leaves its mouth 0.5 clear of the other bosses (per-socket set-backs, COBYLA per node); 3.0 to 13.7 (C).
- **Hub** Ø12 (the layout's node); **every junction filleted** (smooth union, r about 2.5, C): the sharp crotches read 1.8 across the layers in the first FE pass, 1.19 filleted.
- **Bed face:** each node prints on its own outside, on a flat Ø8 foot under the hub (C), "up" chosen so every boss rises from the bed (lowest member elevation −0.7°, node 5, C).
- **Ring nodes 0..3** (the shared Overseer loop): a Ø16 column to a flat seat 10.15 under the rod centres with a Ø8.3 x 2.2 recess (C). The **TPU foot cap** is a Ø16 x 3 disc with a Ø8 x 2 plug pressed and bonded into the recess; contact 13.15 under the ring (C, the Overseer's caps).
- **Eccentric tight pairs (option B, DEFAULT):** where two rods meet a node under 25° (12.7 to 24.6°, C), one or both run up to 3.5 off the node centre at that node only (the rod's line tilts ≤ 3.5 / L there, its far end stays on its node's centre). 10 rod ends a side: node 0 rods 0-1 (1.19) and 0-17 (2.94), node 1 rods 1-5 and 1-12 (3.5), node 8 rods 4-8 (0.55) and 8-16 (3.47), node 10 rods 4-10 and 10-12 (3.5), node 12 rod 5-12 (3.5), node 14 rod 14-16 (1.67), and their mirrors (C; `result.json` eccentric, `offsets.json`). No rod crossing the mirror plane is eccentric; the centre-plane nodes stay concurrent.

| Node (+ mirror) | g (M) | sockets | envelope r | largest set-back | FE 99.9 % / single cell | solid | bed mm² (M) |
|---|---|---|---|---|---|---|---|
| 0 (+3), ring | 9.34 | 7 | 24.8 | 13.7 (0-17) | 0.71 / 1.37 | PASS | 147 |
| 1 (+2), ring | 7.86 | 6 | 19.6 | 9.1 (0-1) | 0.71 / 1.19 | PASS | 147 |
| 4, centre | 5.12 | 5 | 19.8 | 9.3 (4-7, 4-8) | 0.45 / 0.87 | PASS | 76 |
| 5 (+6), waist | 5.11 | 5 | 20.5 | 10.0 (tie 1.1) | 0.77 / 1.33 | PASS | 78 |
| 8 (+7) | 7.17 | 7 | 24.1 | 13.7 (8-10) | 0.54 / 1.07 | PASS | 67 |
| 10 (+9) | 5.76 | 5 | 23.1 | 12.7 (8-10) | 0.67 / 1.27 | PASS | 56 |
| 11, tail point | 6.11 | 5 | 22.6 | 12.2 (9-11, 10-11) | 0.68 / 0.98 | PASS | 64 |
| 12 (+13) | 6.51 | 6 | 22.9 | 12.5 (1-12) | 0.91 / 1.58 | PASS | 71 |
| 14 (+15) | 7.46 | 6 | 23.2 | 12.8 (14-15) | 0.88 / 1.31 | PASS | 72 |
| 16, nose top | 4.76 | 4 | 21.6 | 11.2 (7-16, 8-16) | 0.50 / 0.80 | PASS | 83 |
| 17, nose low | 5.45 | 4 | 19.4 | 8.9 (all) | 0.52 / 0.82 | PASS | 81 |
| foot cap (TPU, x4) | 0.86 | | Ø16 x 5 | | | PASS | 201 |

Envelopes (node frame, mm, every node and mirror): `envelopes.json`. **Node 16:** x −6.1 .. +6.4 about its centre (the fillets add 0.4 over the 6.0 limit drawn on the bosses), y ±21.6, z ±9.0; its 14-16 / 15-16 socket axes reach x +1.2 at the mouths; the camera mount v1 stays 5.1 clear (C; it was 5.45 off the layout's Ø12 sphere).

## Concept: two options weighed
- **A, every socket axis through the node centre:** the cage exactly as drawn and as the frame model assumes; 123.9 g at wall 3.0 (C), nodes to r 38.5: a tight pair needs both sockets ≥ 6.2 / sin θ out (12.7° at node 1: 28), so node 1 forks 38 long and half of rod 1-5 turns into PETG.
- **B, eccentric tight pairs (picked, DEFAULT):** 108.7 g at wall 3.0 (C; 119.9 g as built with fillets and FE walls), every node r ≤ 24.8; costs F x e moments in the node (in its FE: the loads ride the eccentric rods) and moves 10 rod ends a side by ≤ 3.5 (the cage session redraws those tubes from `result.json` rods).
- Review: the `review` agent could not be spawned from this worker (no Agent tool); the concept and the PR are left for the Scout v3 session's review.

## Strength and breaking order
Loads per socket (crash class, the engine's PETG allowables, transversely isotropic; as-built voxel FE at h 0.5 in the print pose; the other rods' stubs held; the rod a bonded carbon insert at its real EA):
- **push** min(frame axial F, Pcr at k 0.7 on the free length E, 600 MPa x A), **pull** the frame axial (F, `node_loads.json`, up to 1264 N), **bending** the rod's whole moment at its 600 MPa allowable (1590 N mm, E) as a couple on the stub plus the frame side force, 4 directions: 6 cases a socket, 102 sockets.
- **Gate (DEFAULT 28 Oct): each socket's 99.9th percentile index ≤ 1.0**; the single worst cell is reported. Cells on the bond line (sharing a node with the rod insert) and within 1.0 of a bore rim are left out (a voxel staircase round a 36x stiffer rod, a bi-material corner). Gating on the single cell sent node 1 round in circles (each thicker wall moved the socket out and the lip peak followed: 1.19 → 1.24, r 20 → 40, 14.6 g).
- Result: every node 99.9 % ≤ 0.91 (C); single cells up to 1.58 (node 12, tie 2.1 in bending), all on socket lips under the full rod moment.
- **Breaking order (decided):** (1) a rod buckles, elastic, it springs back (25 of 55 in the 35 g crash, F); (2) a rod snaps, or its epoxy bond lets go: pull-out 6 MPa x π x 3.2 x 10 = 603 N (E), under the frame's largest pulls (up to 1264 N), so a hard pull draws a rod out of its socket: re-glue; (3) the node last: it holds every rod's push and pull and, outside single lip cells, its full 600 MPa moment. A socket lip can whiten at the rod's 600 MPa moment (single cells 1.0 to 1.6), about half a pultruded rod's bending strength (E): under a pure bending overload a lip may crack before its rod snaps; the node is then the cheap part (5 to 9 g), reprinted without taking the drone apart past its rods. Alt: wall 4.0 on every socket (about +25 g, E) puts every cell under 1.
- The frame model is unchanged (rods 3.0, F): the eccentric ends are not in `cage_frame.py` yet (the cage session).

## Checks
- **Solid:** `python3 nox/solid_check.py print/<file>`: **solid_check PASS** on all 19 print files: scout_node_0_v1.stl, scout_node_0_v1_mirror.stl, scout_node_1_v1.stl, scout_node_1_v1_mirror.stl, scout_node_4_v1.stl, scout_node_5_v1.stl, scout_node_5_v1_mirror.stl, scout_node_8_v1.stl, scout_node_8_v1_mirror.stl, scout_node_10_v1.stl, scout_node_10_v1_mirror.stl, scout_node_11_v1.stl, scout_node_12_v1.stl, scout_node_12_v1_mirror.stl, scout_node_14_v1.stl, scout_node_14_v1_mirror.stl, scout_node_16_v1.stl, scout_node_17_v1.stl, scout_foot_cap_v1.stl (`print_checks.json`).
- **Print:** `print_orient.py`: the saved face is the designed flat (the nodes are round; no other flat face); bed contact 56 to 147 mm² (nodes), 201 mm² (cap); no first-layer island under 20 mm², no thin-island run (M). Brim 5 mm (under 400 mm²); tree supports under the rising bosses.
- **System** (the assembly with the saved nodes, `scout_assembly_updates.py` nodes()): `System: Scout v3, cage first 6.2 mm (E) at +Z (camera) (field cones open: camera 55 deg, out 0 mm), shell 47 rods closed, h/w 0.31, fit 0.0 mm³ (E) (cartridge v6 (latch)/cartridge skid lead end (latch, TPU)), bbox 14.1 L (E), extents 390 x 278 x 130 mm (E), mass 830 of 500 g (166 %) (E) over budget (total), CG 2.2 mm off thrust axis (E) [+2.2, -0.1, -37.7], 1.3 mm without swap parts, probes 7.5 mm (E) edge at (-0.47, +0.02, -0.88) (UBEC), access 25/25 clear` (was CG 0.2 / 2.2, 746 g: the nodes add 84 g, more of it forward; the CG stays inside 3).
- **Funnel / catch:** the ring nodes reach x 68.6 (node 0, −22.2) and −44.7 (node 1, +19.2): clear of the side sleeve zone x ±32 (C); the caps' contact 13.15 under the ring rod centres (the Overseer's, C).
- **What the grams buy:** 84 g over the layout's 39 g E: about 1 g a socket end (102 ends, wall 3.0 and the 3.0 floor) is the floor; fillets about +0.4 g a node; the FE walls +2.3 g (nodes 12, 14 and mirrors).

## Carbon
Carbon rods list: 55 rods, **all 3.0 solid; no non-3.0 rod, no tube**. Cut list floor to floor less 0.5 in `result.json` rods (C): 47 shell rods 47.7 .. 270.2, 5.45 m; 8 ties, node floor to the saddle end point (less the saddle's socket depth) 76.1 .. 110.5. Shortest 47.7 (rods 7-15 / 8-14), over the 33 break-even. No printed struts.

## Into the system
`nox/scout/v3/scout_assembly_updates.py` nodes(): the 18 placed node meshes (`placed/node_<i>.stl`, the mirror nodes placed too) and the 4 caps (`placed/foot_cap_<i>.stl`) replace "cage shell nodes"; each node is a designed joint with the shell tubes and the ties (drawn centre to centre, they cross each hub). `placed.json` lists name, mesh, node index, mass (M), tag. The Scout artifact is republished by the Scout v3 session.

## Requirements for others
- **Scout v3 cage session:** redraw the tubes from the socket floors in `result.json` rods (10 eccentric ends a side, ≤ 3.5) and add the eccentricity to `cage_frame.py` if wanted; the cut list is there.
- **Camera mount v2:** node 16 x ≤ +6.4, nodes 14 / 15 x ≤ +6.0, node 17 x ≤ +8.7 about their centres (`envelopes.json`); the clips stay outside these.
- **Tie saddle:** the 8 ties' node ends are fixed here; their saddle ends are unchanged.

## Runs
Set B (1, 5, 10, 12, 16) on this box; set A (0, 4, 8, 11, 14, 17) in a sibling cloud session (`run/scout-nodes-v1a`, merged) after both sets on one box hit its 14.3 GB memory limit together with the camera mount v2 run (peaks 3.9 / 5.2 GB, M). Logs `build_a.log`, `build_b.log`; per-node records `nodes/<i>.json`. Drawn parts with an FE check, no topo run (no `estimate.json`).

## Open
- Bond strength (6 MPa, E) and the fit of a 3.0 rod in a 3.2 bore 10 deep on this geometry: print node 17 (5.45 g) and pull a glued rod before the set.
- TPU cap bond (CA on PETG and TPU, E): try it on one ring node.
