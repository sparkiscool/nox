# Scout cage frame model v1 (F on E properties: carbon E 100 GPa, allowables 1000 / 600 MPa; buckling k 1.0 shell rods, 0.7 saddle ties)

Cage `runs/scout_layout/v11/cage_shell.json`: 17 nodes, 44 shell rods + 6 saddle ties, every rod 3.0 solid; the ties' saddle ends fixed (the core held); the camera + cradle (27 g E) rigid on its 4 clips. Drone 680 g (E): crash 233 N through the most extreme node, 14 directions; flight 4 g; handling 50 N at mid-span (closed form).

Lowest modes (rigid joints, lumped masses): 25 Hz (the camera moves), 56 Hz (the camera moves), 78 Hz (the camera moves)

## Buckling: 31 members at or over 1.0 x their Euler load
- rod 0-3: 19.49 x Pcr (segment L 232; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 0-11: 4.49 x Pcr (segment L 210; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 3-10: 4.49 x Pcr (segment L 210; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 0-8: 4.45 x Pcr (segment L 114; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 3-7: 4.45 x Pcr (segment L 114; crash 4 [0.0, 0.0, 1.0] via node 4)
- saddle tie 2.1: 3.90 x Pcr (segment L 143; crash 4 [0.0, 0.0, 1.0] via node 4)
- saddle tie 2.1 m: 3.90 x Pcr (segment L 143; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 4-11: 3.09 x Pcr (segment L 131; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 4-10: 3.09 x Pcr (segment L 131; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 7-8: 2.67 x Pcr (segment L 92; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 12-13: 2.56 x Pcr (segment L 130; crash 1 [-1.0, 0.0, 0.0] via node 9)
- rod 4-9: 2.10 x Pcr (segment L 92; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 8-11: 2.05 x Pcr (segment L 205; crash 1 [-1.0, 0.0, 0.0] via node 9)
- rod 7-10: 2.05 x Pcr (segment L 205; crash 1 [-1.0, 0.0, 0.0] via node 9)
- rod 6-10: 1.70 x Pcr (segment L 117; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 5-11: 1.70 x Pcr (segment L 117; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 2-3: 1.62 x Pcr (segment L 155; crash 13 [-0.58, -0.58, -0.58] via node 2)
- rod 0-1: 1.62 x Pcr (segment L 155; crash 11 [-0.58, 0.58, -0.58] via node 1)
- rod 4-14: 1.61 x Pcr (segment L 119; crash 4 [0.0, 0.0, 1.0] via node 4)
- saddle tie 4.1: 1.46 x Pcr (segment L 86; crash 4 [0.0, 0.0, 1.0] via node 4)
- saddle tie 4.1 m: 1.46 x Pcr (segment L 86; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 1-5: 1.42 x Pcr (segment L 80; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 2-6: 1.42 x Pcr (segment L 80; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 8-14: 1.33 x Pcr (segment L 172; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 7-14: 1.33 x Pcr (segment L 172; crash 4 [0.0, 0.0, 1.0] via node 4)

## Crash stress over 600 MPa: 14 members
- saddle tie 4.1: 2808 MPa (crash 5 [0.0, 0.0, -1.0] via node 0)
- saddle tie 2.1: 2195 MPa (crash 5 [0.0, 0.0, -1.0] via node 0)
- saddle tie 2.1 m: 2078 MPa (crash 4 [0.0, 0.0, 1.0] via node 4)
- saddle tie 1.1 m: 1949 MPa (crash 4 [0.0, 0.0, 1.0] via node 4)
- saddle tie 1.1: 1949 MPa (crash 4 [0.0, 0.0, 1.0] via node 4)
- saddle tie 4.1 m: 1549 MPa (crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 1-5: 1284 MPa (crash 5 [0.0, 0.0, -1.0] via node 0)
- rod 2-6: 1063 MPa (crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 0-5: 1035 MPa (crash 5 [0.0, 0.0, -1.0] via node 0)
- rod 3-6: 909 MPa (crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 6-10: 719 MPa (crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 5-11: 719 MPa (crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 0-12: 632 MPa (crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 3-13: 632 MPa (crash 4 [0.0, 0.0, 1.0] via node 4)

## Handling 50 N at mid-span: 20 rods over 600 MPa, 13 bending over 10 mm
- rod 1-2: 1129 MPa, 36 mm (L 239)
- rod 0-3: 1095 MPa, 33 mm (L 232)
- rod 7-8: 1039 MPa, 28 mm (L 220)
- rod 2-9: 1034 MPa, 28 mm (L 219)
- rod 1-9: 1034 MPa, 28 mm (L 219)
- rod 3-10: 990 MPa, 24 mm (L 210)
- rod 0-11: 990 MPa, 24 mm (L 210)
- rod 8-11: 969 MPa, 23 mm (L 205)
- rod 7-10: 969 MPa, 23 mm (L 205)
- rod 9-11: 869 MPa, 16 mm (L 184)
- rod 9-10: 869 MPa, 16 mm (L 184)
- rod 8-14: 812 MPa, 13 mm (L 172)
- rod 7-14: 812 MPa, 13 mm (L 172)
- rod 2-3: 730 MPa, 10 mm (L 155)
- rod 0-1: 730 MPa, 10 mm (L 155)
- rod 11-14: 672 MPa, 8 mm (L 142)
- rod 10-14: 672 MPa, 8 mm (L 142)
- rod 4-11: 618 MPa, 6 mm (L 131)
- rod 4-10: 618 MPa, 6 mm (L 131)
- rod 12-13: 611 MPa, 6 mm (L 130)

Worst shell rod buckling 19.49, worst tie 3.90; node socket loads (largest axial / side per member and node over all cases) in node_loads.json.

## Variants (the same cases; `alt_*/`)
| Variant | Rods 6/4 tube | Added mass (E) | Buckling ≥ 1 | Crash stress > 600 | Handling > 600 | First mode |
|---|---|---|---|---|---|---|
| **v11 as drawn: every rod 3.0 (Spark rule)** | 0 | 0 | 31 | 14 | 20 | 25 Hz (camera) |
| `alt_ties6`: the 6 saddle ties 6/4 | 6 | ~10 g | 15 | 6 | 20 | 58 Hz |
| `alt_sized`: 6/4 where the frame model needs it (sizing loop, `up6.json`) | 38 of 50 (5.5 of 6.6 m) | +73 g | 0 | 0 | 0 | 79 Hz (camera) |
| `alt_all6`: every rod 6/4 | 50 | +88 g | 0 | 0 | 0 | 80 Hz (camera) |

Reading:
- **The 3.0 cage does not survive the system cases.** A 35 g crash (233 N) reaches the core only through the 6 saddle ties, and the long 3.0 rods (up to 239) buckle at a few tens of newtons. Handling 50 N mid-span overstresses 20 rods.
- **What passes:** the sized cage, with 38 members at 6/4 tube and 12 short rods kept 3.0 (+73 g, ~753 g all-up), or every rod 6/4 (+88 g).
- **This conflicts with the Scout clause** (every rod 3 mm, Spark 27 Oct). The general rod rule allows thicker only where the frame model needs it, so this is Spark's call (current-parts Spark question).
- **Camera mount:** 25 Hz on the 3.0 cage; 79 to 80 Hz on 6/4 (gate 100). Next: move the top clips toward their rod's end nodes, or onto a node (layout).
- Picture: https://claude.ai/artifact/JRR3DumWuT78VsWTiJpTDR (`frame.html`, `members.png`).
- **Nodes:** the node topology runs (`nox/overseer/cage_node.py` adapted) wait on this: the socket bores are 3.2 or 6.2 by member.
