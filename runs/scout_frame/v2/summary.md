# Scout cage frame model v1 (F on E properties: carbon E 100 GPa, allowables 1000 / 600 MPa; buckling k 1.0 shell rods, 0.7 saddle ties)

Cage `runs/scout_layout/v12/cage_shell.json`: 18 nodes, 47 shell rods + 8 saddle ties, every rod 3.0 solid; the ties' saddle ends fixed (the core held); the camera + cradle (27 g E) rigid on its 4 clips. Drone 730 g (E): crash 250 N through the most extreme node, 14 directions; flight 4 g; handling 50 N at mid-span (closed form).

Lowest modes (rigid joints, lumped masses): 48 Hz (the camera moves), 67 Hz (the camera moves), 147 Hz

## Buckling: 28 members at or over 1.0 x their Euler load
- rod 8-10: 6.78 x Pcr (segment L 212; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 7-9: 6.78 x Pcr (segment L 212; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 4-8: 6.25 x Pcr (segment L 286; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 4-7: 6.25 x Pcr (segment L 286; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 14-15: 3.14 x Pcr (segment L 124; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 0-3: 3.01 x Pcr (segment L 232; crash 1 [-1.0, 0.0, 0.0] via node 11)
- rod 0-1: 2.81 x Pcr (segment L 155; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 2-3: 2.81 x Pcr (segment L 155; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 0-14: 2.40 x Pcr (segment L 88; crash 5 [0.0, 0.0, -1.0] via node 0)
- rod 9-13: 2.32 x Pcr (segment L 147; crash 11 [-0.58, 0.58, -0.58] via node 1)
- rod 10-12: 2.32 x Pcr (segment L 147; crash 13 [-0.58, -0.58, -0.58] via node 2)
- rod 3-15: 1.84 x Pcr (segment L 88; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 1-11: 1.45 x Pcr (segment L 225; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 2-11: 1.45 x Pcr (segment L 225; crash 4 [0.0, 0.0, 1.0] via node 4)
- saddle tie 2.1: 1.45 x Pcr (segment L 118; crash 4 [0.0, 0.0, 1.0] via node 4)
- saddle tie 2.1 m: 1.45 x Pcr (segment L 118; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 4-9: 1.28 x Pcr (segment L 129; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 4-10: 1.28 x Pcr (segment L 129; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 4-11: 1.26 x Pcr (segment L 75; crash 4 [0.0, 0.0, 1.0] via node 4)
- rod 1-2: 1.26 x Pcr (segment L 239; crash 11 [-0.58, 0.58, -0.58] via node 1)
- rod 3-17: 1.24 x Pcr (segment L 121; crash 5 [0.0, 0.0, -1.0] via node 0)
- rod 0-17: 1.22 x Pcr (segment L 121; crash 13 [-0.58, -0.58, -0.58] via node 2)
- rod 3-7: 1.20 x Pcr (segment L 114; crash 8 [0.58, -0.58, 0.58] via node 7)
- rod 0-8: 1.20 x Pcr (segment L 114; crash 6 [0.58, 0.58, 0.58] via node 8)
- rod 7-16: 1.10 x Pcr (segment L 100; crash 5 [0.0, 0.0, -1.0] via node 0)

## Crash stress over 600 MPa: 10 members
- saddle tie 4.1: 2136 MPa (crash 5 [0.0, 0.0, -1.0] via node 0)
- saddle tie 4.1 m: 1939 MPa (crash 4 [0.0, 0.0, 1.0] via node 4)
- saddle tie 3.1: 1418 MPa (crash 5 [0.0, 0.0, -1.0] via node 0)
- saddle tie 2.1: 978 MPa (crash 5 [0.0, 0.0, -1.0] via node 0)
- saddle tie 1.1: 847 MPa (crash 5 [0.0, 0.0, -1.0] via node 0)
- saddle tie 3.1 m: 753 MPa (crash 9 [0.58, -0.58, -0.58] via node 3)
- rod 8-14: 738 MPa (crash 5 [0.0, 0.0, -1.0] via node 0)
- saddle tie 1.1 m: 660 MPa (crash 5 [0.0, 0.0, -1.0] via node 0)
- rod 7-15: 655 MPa (crash 4 [0.0, 0.0, 1.0] via node 4)
- saddle tie 2.1 m: 654 MPa (crash 4 [0.0, 0.0, 1.0] via node 4)

## Handling 50 N at mid-span: 19 rods over 600 MPa, 11 bending over 10 mm
- rod 4-8: 1350 MPa, 61 mm (L 286)
- rod 4-7: 1350 MPa, 61 mm (L 286)
- rod 1-2: 1129 MPa, 36 mm (L 239)
- rod 0-3: 1095 MPa, 33 mm (L 232)
- rod 2-11: 1062 MPa, 30 mm (L 225)
- rod 1-11: 1062 MPa, 30 mm (L 225)
- rod 8-10: 998 MPa, 25 mm (L 212)
- rod 7-9: 998 MPa, 25 mm (L 212)
- rod 7-8: 909 MPa, 19 mm (L 193)
- rod 9-11: 826 MPa, 14 mm (L 175)
- rod 10-11: 826 MPa, 14 mm (L 175)
- rod 2-3: 730 MPa, 10 mm (L 155)
- rod 0-1: 730 MPa, 10 mm (L 155)
- rod 9-13: 694 MPa, 8 mm (L 147)
- rod 10-12: 694 MPa, 8 mm (L 147)
- rod 2-13: 680 MPa, 8 mm (L 144)
- rod 1-12: 680 MPa, 8 mm (L 144)
- rod 4-9: 607 MPa, 6 mm (L 129)
- rod 4-10: 607 MPa, 6 mm (L 129)

Worst shell rod buckling 6.78, worst tie 1.45; node socket loads (largest axial / side per member and node over all cases) in node_loads.json.

## Sized (`alt_sized/`, the same loop as v1)
39 of 55 members to 6/4 tube (+74 g E): 0 buckle, 0 over 600 MPa in crash or handling, first mode 138 Hz (the camera; gate 100). This is the frame model on `runs/scout_layout/v12/`; v1 was on v11 (38 members, +73 g, 79 Hz).
