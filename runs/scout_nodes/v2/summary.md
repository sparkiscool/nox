# Scout cage nodes v2 (28 Oct 2026, drawn; `nox/scout/v3/cage_node.py`)

Piece of **System Scout v3** (`dimensions.md` "System Scout v3": Rods, Nodes, Bottom, Landing, Funnel fit, Camera, Tie saddle; system model `nox/scout/v3/scout.assembly.json`, cage `runs/scout_layout/v12/cage_shell.json` + the 8 saddle ties of its `result.json` "links", frame model `runs/scout_frame/v3/node_loads.json` and `members.json`). Supersedes `runs/scout_nodes/v1/` (PR 166 review FAIL, see RETIRED.md there).

The brief:
- Spark 27 Oct: every Scout rod 3.0 solid carbon, socket bore 3.2 epoxied (the coupon pick); nodes PETG; perfect left-right mirror symmetry.
- Spark 27 Oct: the cage fails the 35 g crash on 3.0 rods, accepted (rods are the fuse); each node carries what its rods can deliver.
- Box catch F (PR 155): TPU foot caps Ø16 x 3 under the 4 ring nodes; the side ring rods keep a 6 mm sleeve zone over x ±32.
- Node 16: sockets toward +x within 5 of its centre (camera mount clips nearby).
- Review of PR 166 (28 Oct): the worst-cell gate (dimensions.md L index target), lip peaks fixed by shape or pose, eccentric moments within what the rods carry, an honest breaking order.

What changed from v1:
- **Gate:** each socket's worst cell ≤ 1.0 (v1 gated the 99.9th percentile, which dropped real peaks).
- **Print pose:** the bosses laid as flat as they go (smallest steepest elevation, none below −1°); v1 stood bosses up to 64..76° and loaded their bending across the layers.
- **Bell mouth + mouth collar** on every socket: the bore opens Ø3.2 → Ø5.2 over its last 2.0 (lip fillet) and the boss is 1.0 thicker over its last 3.0, so the wall stays ≥ 3 round the bell. With sharp mouths the worst cells sat 0.8 inside the mouth, the rod prying on the edge, and wall growth did not move them.
- **Saddle-tie sockets 14 deep** (rods 10): the ties carry the frame's largest side forces (to 100 N, F).
- **Bend case:** the moment at the mouth is the rod's allowable 1590 N mm with the frame side force as its shear (v1 added side force x lever on top: 2265 N mm on tie 4.1, more than the rod can carry).
- **Eccentric offsets capped:** offset ≤ 3.5 and frame axial x offset ≤ 1590 N mm (rod 10-12 / 9-13: 3.5 → 1.51 for its 1052 N, F; v1 carried 3680 N mm there).
- Fillets grow with the sizing passes (r 2.5 → 5.5).

Visuals:
- Every node on the cage in 3D, each design's print picture, its section through the tightest socket pair, sockets, FE and outlast, the two options and the breaking order: https://claude.ai/artifact/WzJaJRF2Jwc721Gd9dYKsa (version 2). Source `nodes.html`, built by `nox/scout/v3/cage_node_sheet.py` (+ `cage_node_sheet.html`); sections `section_<i>.png`.

Previews (one per print file; a mirror print's picture is its design's, mirrored):
- `preview_scout_node_0_v2.png`: print/scout_node_0_v2.stl and print/scout_node_0_v2_mirror.stl
- `preview_scout_node_1_v2.png`: print/scout_node_1_v2.stl and print/scout_node_1_v2_mirror.stl
- `preview_scout_node_4_v2.png`: print/scout_node_4_v2.stl
- `preview_scout_node_5_v2.png`: print/scout_node_5_v2.stl and print/scout_node_5_v2_mirror.stl
- `preview_scout_node_8_v2.png`: print/scout_node_8_v2.stl and print/scout_node_8_v2_mirror.stl
- `preview_scout_node_10_v2.png`: print/scout_node_10_v2.stl and print/scout_node_10_v2_mirror.stl
- `preview_scout_node_11_v2.png`: print/scout_node_11_v2.stl
- `preview_scout_node_12_v2.png`: print/scout_node_12_v2.stl and print/scout_node_12_v2_mirror.stl
- `preview_scout_node_14_v2.png`: print/scout_node_14_v2.stl and print/scout_node_14_v2_mirror.stl
- `preview_scout_node_16_v2.png`: print/scout_node_16_v2.stl
- `preview_scout_node_17_v2.png`: print/scout_node_17_v2.stl
- `preview_scout_foot_cap_v1.png`: print/scout_foot_cap_v1.stl (the cap is unchanged from v1)

## What it is
**18 PETG nodes in 11 designs** (one per mirror pair, the -y node is the `_mirror` print; one per centre-plane node 4, 11, 16, 17) **+ 4 TPU foot caps**: 188.8 g of nodes (M) + 3.4 g of caps (M) = 192.2 g (M), in place of the layout's 39 g (E) spheres.

Everything is drawn (the rule's drawn set: sockets with their bosses, the hub, the cap seat, the bed face); nothing generated:
- **Socket:** bore 3.2, 10 deep (saddle ties 14), bell mouth over the last 2.0 to Ø5.2, boss wall ≥ 3.0 with a +1.0 collar over the last 3.0 (C). FE walls: tie 4.1 and tie 2.1 at 4.5, rods 0-3, 0-5, 1-5, 0-14, 8-10, 8-16 and ties 1.1, 3.1 at 3.5 (C). Floor behind each socket ≥ 3.0 (C).
- **Set-back:** each socket starts only as far out as it must: every two bores keep the thicker of their walls of PETG between them, bells included, and every rod leaves its mouth 0.5 clear of the other bosses and collars (per-socket set-backs, COBYLA per node); 3.0 to 27.4 (C).
- **Hub** Ø12 (the layout's node), every junction filleted (smooth union, r 2.5 to 5.5 by node, C).
- **Bed face:** each node prints on its own outside, on a flat foot under the hub (C), the bosses as flat as they go (C).
- **Ring nodes 0..3** (the shared Overseer loop): a Ø16 column to a flat seat 10.15 under the rod centres with a Ø8.3 x 2.2 recess (C). The **TPU foot cap** is a Ø16 x 3 disc with a Ø8 x 2 plug pressed and bonded into the recess; contact 13.15 under the ring (C, the Overseer's caps).
- **Eccentric tight pairs (option B, DEFAULT):** where two rods meet a node under 25° (12.7 to 24.6°, C), one or both run off the node centre at that node only (the rod's line tilts ≤ e / L there, its far end stays on its node's centre), e ≤ 3.5 and frame axial x e ≤ 1590 N mm. 10 ends a side (C): node 0 rods 0-1 (2.68) and 0-17 (3.48), node 1 rods 1-5 and 1-12 (3.5), node 8 rods 4-8 (0.57) and 8-16 (3.5), node 10 rods 4-10 (3.5) and 10-12 (1.51), node 12 rod 5-12 (3.5), node 14 rod 14-16 (1.97), and their mirrors (`result.json` eccentric, `offsets.json`). Eccentric moments (frame axial F x e, C): 1588 (10-12), 1246 (0-1), 1215 (8-16), 1190 (0-17), 1068 (4-10), 984 / 980 (1-12 / 1-5), 697 (14-16), 483 (5-12), 173 (4-8) N mm: each within one rod's allowable 1590, and inside the node's FE (its push and pull ride the eccentric line).

| Node (+ mirror) | g (M) | sockets | envelope r (C) | largest set-back (C) | worst cell (C) | 99.9 % (C) | outlast (C on E) | passes | fillet (C) | bed mm² (M) |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 (+3), ring | 12.97 | 7 | 29.8 | 17.9 (0-17) | 0.94 (rod 0-8) | 0.49 | 1.45 | 2 | 3.5 | 148 |
| 1 (+2), ring | 9.99 | 6 | 25.2 | 13.4 (1-5) | 0.77 (rod 1-12) | 0.39 | 1.19 | 1 | 2.5 | 147 |
| 4, centre | 6.68 | 5 | 21.9 | 10.8 (4-7) | 0.76 (rod 4-7) | 0.38 | 1.18 | 1 | 2.5 | 88 |
| 5 (+6), waist | 9.45 | 5 | 27.8 | 12.4 (tie 1.1) | 0.90 (tie 1.1) | 0.47 | 1.39 | 2 | 3.5 | 189 |
| 8 (+7) | 11.75 | 7 | 31.5 | 19.4 (8-16) | 0.77 (rod 8-16) | 0.47 | 1.19 | 2 | 3.5 | 85 |
| 10 (+9) | 9.94 | 5 | 38.8 | 27.4 (10-12) | 0.87 (rod 4-10) | 0.36 | 1.35 | 2 | 3.5 | 90 |
| 11, tail point | 7.47 | 5 | 25.2 | 14.1 (9-11) | 0.86 (rod 2-11) | 0.48 | 1.33 | 1 | 2.5 | 102 |
| 12 (+13) | 13.55 | 6 | 28.4 | 16.6 (1-12) | 0.80 (tie 2.1) | 0.40 | 1.23 | 4 | 5.5 | 456 |
| 14 (+15) | 13.30 | 6 | 26.7 | 14.8 (14-15) | 0.92 (tie 4.1) | 0.56 | 1.42 | 4 | 5.5 | 375 |
| 16, nose top | 5.93 | 4 | 24.2 | 13.1 (7-16) | 0.70 (rod 8-16) | 0.41 | 1.08 | 1 | 2.5 | 175 |
| 17, nose low | 6.81 | 4 | 21.5 | 10.3 (0-17) | 0.90 (rod 0-17) | 0.47 | 1.39 | 1 | 2.5 | 185 |
| foot cap (TPU, x4) | 0.86 | | Ø16 x 5 | | | | | | | 201 |

Envelopes (node frame, mm, every node and mirror): `envelopes.json` (C). **Node 16:** x −7.0 .. +6.7 about its centre (the 6.0 limit is drawn on the bosses; the collars and fillets add 0.7), y ±24.1, z −10.3 .. +10.7; the camera mount v1 stays 4.7 clear (C; v1 5.1, the layout's sphere 5.45). Nodes 14 / 15 reach x +7.8, node 17 x +10.2.

## Concept: two options weighed
The concept review was not run: this worker cannot spawn the `review` agent (no Agent tool). The two options are weighed here and on the page; the Scout v3 session's review covers the PR.
- **A, every socket axis through the node centre:** the cage exactly as drawn and as the frame model assumes; 168.9 g at the drawn minimum (C, v2 sockets), nodes to r 43.7: a tight pair needs both sockets ≥ 6.2 / sin θ out (12.7° at node 1: 28), so node 1 forks over 40 long and half of rod 1-5 turns into PETG.
- **B, eccentric tight pairs (picked, DEFAULT):** 152.3 g at the drawn minimum (C; 188.8 g as built with the FE walls and fillets, M), every node r ≤ 38.8; costs the eccentric moments above (each ≤ one rod's allowable moment, in the node FE) and moves 10 rod ends a side by ≤ 3.5 (the cage session redraws those tubes from `result.json` rods). The frame model is not rerun with the offsets (`cage_frame.py` has no offsets option); the cap keeps every eccentric moment within what one rod carries alone.

## Strength and breaking order
Loads per socket (crash class, the engine's PETG allowables, transversely isotropic; as-built voxel FE at h 0.5 in the print pose; the other rods' stubs held; the rod a bonded carbon insert at its real EA):
- **push** min(frame axial F, Pcr at k 0.7 on the free length E, 600 MPa x A), **pull** the frame axial (F, up to 1264 N), **bending** the rod's allowable moment at the mouth (600 MPa x Z = 1590 N mm, E) with the frame side force as its shear (F, to 100 N), 4 directions: 6 cases a socket, 102 sockets.
- **Gate (dimensions.md L index target): each socket's worst cell ≤ 1.0**, with the bond line (PETG cells sharing a node with the rod insert) and 1.0 round each bore rim (floor, bell start, mouth) left out. Result: every node ≤ 0.94 (C); 99.9 % ≤ 0.56 (C).
- **Breaking order (stated plainly):**
  1. In the crash cases a rod buckles first, elastic, it springs back (25 of 55 in the 35 g crash, F): the rods are the fuse, as Spark accepted. Every node holds each rod's push and pull (frame F) at the crash allowable.
  2. A hard pull draws a rod out of its socket when the bond lets go: 6 MPa (E) x π x 3.2 x 8 (the bell's 2.0 carries only an epoxy fillet) = 482 N for a rod, 724 N for a tie (12 of contact), under the frame's largest pulls (up to 1264 N, F): re-glue.
  3. **In a pure bending overload the node goes before the rod.** The node holds the rod's 600 MPa allowable moment (worst cell ≤ 1.0), but a pultruded 3.0 rod breaks near 1200 MPa (E). At that moment each node's worst bending cell reads 1.08 (node 16) to 1.45 (node 0) of its PETG strength (outlast, C on E; no safety factor), so a socket cracks before its rod snaps. The node is then the cheap part, reprinted (6 to 14 g) and re-glued without taking the drone apart past its rods. Making every node outlast a rod's real strength would take about 1.5x the bending capacity: bosses near Ø13, about +60 g (E); not done (weight, and a reprintable node is the better fuse than a carbon rod in two nodes).

## Checks
- **Solid:** `python3 nox/solid_check.py print/<file>`: **solid_check PASS** on all 19 print files: scout_node_0_v2.stl, scout_node_0_v2_mirror.stl, scout_node_1_v2.stl, scout_node_1_v2_mirror.stl, scout_node_4_v2.stl, scout_node_5_v2.stl, scout_node_5_v2_mirror.stl, scout_node_8_v2.stl, scout_node_8_v2_mirror.stl, scout_node_10_v2.stl, scout_node_10_v2_mirror.stl, scout_node_11_v2.stl, scout_node_12_v2.stl, scout_node_12_v2_mirror.stl, scout_node_14_v2.stl, scout_node_14_v2_mirror.stl, scout_node_16_v2.stl, scout_node_17_v2.stl, scout_foot_cap_v1.stl (`print_checks.json`).
- **Print:** `print_orient.py`: the saved face is the designed flat (the nodes are round; no other flat face); bed contact 85 to 456 mm² (nodes, M), 201 mm² (cap, M); no first-layer island under 20 mm² past 10 mm (node 4: 2 mm, M). Brim 5 mm under 400 mm²; tree supports under the rising bosses and inside nothing (bores face up or sideways).
- **System** (the assembly with the saved nodes, `scout_assembly_updates.py` nodes()): `System: Scout v3, cage first 6.3 mm (E) at +Z (camera) (field cones open: camera 55 deg, out 0 mm), shell 47 rods closed, h/w 0.31, fit 0.0 mm³ (E) (cartridge v6 (latch)/cartridge skid lead end (latch, TPU)), bbox 14.3 L (E), extents 391 x 278 x 132 mm (E), mass 899 of 500 g (180 %) (E) over budget (total), CG 4.7 mm off thrust axis (E) [+4.7, -0.1, -36.6], 4.6 mm without swap parts FAIL (tol 3), probes 7.3 mm (E) edge at (-0.47, +0.02, -0.88) (UBEC), access 25/25 clear`. **CG FAIL:** the nodes add 153 g over the layout's 39 g (E), more of it forward (ring, nose and front nodes); the layout's catch trim (the Scout v3 session) moves it back. Was CG 0.2 / 2.2, 746 g.
- **Funnel / catch:** the ring nodes stay clear of the side sleeve zone x ±32 (C); the caps' contact 13.15 under the ring rod centres (the Overseer's, C).
- **What the grams buy:** 192.2 g (M) against 39 g (E): about 1 g a socket end at wall 3.0 is the floor (102 ends); the bell mouths and collars about +35 g (C, 117.7 → 152.3 at the drawn minimum); the FE walls and fillets +36.5 g (C → M) on nodes 0, 5, 8, 10, 12, 14 and mirrors.

## Carbon
Carbon rods list: 55 rods, **all 3.0 solid; no non-3.0 rod, no tube**. Cut list floor to floor less 0.5 in `result.json` rods (C): 47 shell rods 44.8 .. 267.9, 5.26 m; 8 ties, node floor to the saddle end point (less the saddle's socket depth) 74.7 .. 108.1. Shortest 44.8 (rods 7-15 / 8-14), over the 33 break-even. No printed struts.

## Into the system
`nox/scout/v3/scout_assembly_updates.py` nodes(): the 18 placed node meshes (`placed/node_<i>.stl`, the mirror nodes placed too) and the 4 caps (`placed/foot_cap_<i>.stl`) replace "cage shell nodes"; each node is a designed joint with the shell tubes and the ties (drawn centre to centre, they cross each hub). `placed.json` lists name, mesh, node index, mass (M), tag. The Scout v3 session redraws the tubes to the socket floors and republishes the Scout artifact.

## Requirements for others
- **Scout v3 session:** tubes to the socket floors in `result.json` rods (10 eccentric ends a side); re-trim the balance (CG 4.7 / 4.6 with these nodes, gate 3).
- **Camera mount v2:** node 16 x ≤ +6.7, nodes 14 / 15 x ≤ +7.8, node 17 x ≤ +10.2 about their centres (`envelopes.json`).
- **Tie saddle:** the 8 ties' node ends are fixed here (14 deep sockets); their saddle ends are unchanged.

## Runs
Nodes 0, 4, 8, 11, 14, 16, 17 on this box (two processes, peaks 3.3 / 3.9 GB, M); nodes 1, 5, 10, 12 in a sibling cloud session (`run/scout-nodes-v2d`, merged, archived). Logs `build_a.log`, `build_b.log`, `build_c.log`; per-node records `nodes/<i>.json` (every sizing pass: walls, fillet, worst cell, outlast). Drawn parts with an FE check, no topo run (no `estimate.json`).

## Open
- Bond strength (6 MPa, E), the fit of a 3.0 rod in the 3.2 bore and the bell's epoxy fillet: print node 17 (6.8 g) and pull a glued rod before the set.
- TPU cap bond (CA on PETG and TPU, E): try it on one ring node.
- Frame model with the eccentric offsets (an offsets option in `cage_frame.py`): the cage session, if it wants more than the per-rod cap.
