# Scout cage frame model v1 (F on E properties: carbon E 100 GPa, allowables 1000 / 600 MPa; buckling k 1.0 shell rods, 0.7 saddle ties)

Cage `runs/scout_layout/v11/cage_shell.json`: 17 nodes, 44 shell rods + 6 saddle ties, every rod 3.0 solid; the ties' saddle ends fixed (the core held); the camera + cradle (27 g E) rigid on its 4 clips. Drone 680 g (E): crash 233 N through the most extreme node, 14 directions; flight 4 g; handling 50 N at mid-span (closed form).

Lowest modes (rigid joints, lumped masses): 58 Hz (the camera moves), 105 Hz, 149 Hz

## Buckling: 15 members at or over 1.0 x their Euler load
- rod 0-3: 6.16 x Pcr (segment L 232; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 4-10: 2.94 x Pcr (segment L 131; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 4-11: 2.94 x Pcr (segment L 131; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 4-9: 2.10 x Pcr (segment L 92; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 0-11: 1.91 x Pcr (segment L 210; crash 5 [0.0, 0.0, -1.0] via node 0)
- rod 4-14: 1.70 x Pcr (segment L 119; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 0-8: 1.61 x Pcr (segment L 114; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 3-7: 1.61 x Pcr (segment L 114; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 3-10: 1.54 x Pcr (segment L 210; crash 12 [-0.58, -0.58, 0.58] via node 10)
- rod 8-14: 1.54 x Pcr (segment L 172; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 7-14: 1.54 x Pcr (segment L 172; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 9-10: 1.05 x Pcr (segment L 184; crash 1 [-1.0, 0.0, 0.0] via node 9)
- rod 9-11: 1.05 x Pcr (segment L 184; crash 1 [-1.0, 0.0, 0.0] via node 9)
- rod 2-9: 1.04 x Pcr (segment L 219; crash 2 [0.0, 1.0, 0.0] via node 11)
- rod 1-9: 1.04 x Pcr (segment L 219; crash 3 [0.0, -1.0, 0.0] via node 10)

## Crash stress over 600 MPa: 6 members
- rod 2-6: 886 MPa (crash 13 [-0.58, -0.58, -0.58] via node 2)
- rod 1-5: 886 MPa (crash 11 [-0.58, 0.58, -0.58] via node 1)
- saddle tie 2.1: 658 MPa (crash 4 [0.0, 0.0, 1.0] via node 4)
- saddle tie 2.1 m: 658 MPa (crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 0-5: 640 MPa (crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 3-6: 640 MPa (crash 4 [0.0, 0.0, 1.0] via node 4)

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

Worst shell rod buckling 6.16, worst tie 0.10; node socket loads (largest axial / side per member and node over all cases) in node_loads.json.
