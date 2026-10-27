# Overseer sensor head on 3 rods (27 Oct 2026, Spark: "i want to reduce the number of rods going to the lidar to a max of 3")

Script `nox/overseer/head_rods.py` (F on E, `nox/frame_model.py`). Picture `options.png`. Data `result.json` (every set, the rejected rods and why).

**Before:** 12 rods ended on the head: 4 L2 braces to the core, 4 struts to the side cage nodes, and the 4 cage tubes of node 17, which was printed in the head. The struts and node 17's tubes went to the same 4 nodes (15, 16, 18, 19).

**Method.** The head is a rigid body: the L2 230 g (P) at its CG with its inertia, both cameras 15 g each (E), the mount 143.7 g (E, v4's M). It holds no cage node, so node 17 becomes an ordinary cage node and its tubes are not head rods. The cage shell (PETG node stubs) and its links on the core are condensed onto the front cage nodes once.

Every mirror-symmetric set of 3 rods is tried: one rod in the centre plane plus a mirrored pair. Each rod runs from a point on the head to the core (the hub deck tower) or a front cage node. There are 16 head points, 6 behind the L2 base plane at (u along the L2 up axis, y).

Each rod is 6/4 carbon, rigid-ended (PETG stubs for the leaders). A rod is kept only if it is:
- at least 33 long (carbon break-even);
- behind the L2 base plane;
- out of both camera views;
- 12 off the prop disks;
- clear of the cage tubes it does not end on.

The sets are ranked by:
- the head's lowest mode (target >= 100 Hz);
- the L2 deflection and the largest rod force at 35 g in 14 directions;
- cage first: the L2 and both cameras >= 3 behind the cage outside their field cones. This is a support-function estimate (C); on the 8-rod head it read within 2 mm of system_check.

110 rods passed and 2241 sets were tried; 378 of them guard both cameras.

| Set | Head mode rigid / PETG joints (Hz, F on E) | L2 move at 35 g (mm) | Cameras / L2 guard (mm, C) | Carbon (mm) |
|---|---|---|---|---|
| **centre (u -40) to node 17 + pair (u +100, y ±70) to nodes 15 / 16 (chosen, DEFAULT)** | **151.8 / 128.4** | **0.24** | **10.4 / 9.8, L2 52.6** | **360** |
| centre to node 17 + pair (u +110, y ±85) to 15 / 16 | 169.0 / 141.0 | 0.22 | 17.7 / 16.5 | 337 |
| centre to node 17 + pair (u +10, y ±70) to 15 / 16 | 143.6 / 125.9 | 0.42 | -32.9 / -34.1 (cameras unguarded) | 373 |
| centre to the core tower front + pair to 15 / 16 | 89.6 / 78.4 | 0.37 | (not guarded) | 563 |

- **Chosen:** the stiffest guarded set that fits the A1 mini. The y ±85 pair is stiffer, but it puts the head 182 wide, over the bed's 180.
- **Rods:** all 3 run to the cage; none run to the body. The centre rod is 35 long (C), just over the 33 break-even.
- **Checks:** system_check on the assembled layout gives cage first 9.61 on camera left (was 7.7 with 8 rods). The L2 field is 0.0 % crossed and both camera views are clear (C).
- **Frame model (`runs/overseer_frame/v1`):** the head mode is 167 Hz with lumped masses. The rods carry at most 126 N in the head's own 35 g, and 25 N in a crash through the cage (F on E).
- **Carbon:** 3 rods, 360 mm of 6/4 tube in all (C). It replaces 4 braces (about 290 each), 2 keels and 4 struts, about 2.3 m.
- **Cost of 3 rods:** with only 3 rods the head is held partly by the rods' bending at their epoxied ends. The PETG-joint number (128.4 Hz) is the one that counts; it is still over the 100 Hz target.
