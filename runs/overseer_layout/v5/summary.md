# Overseer layout v5 / cage v3: sensor head outside the disk cage (26 Oct 2026, concept)

Spark 26 Oct: "The lidar needs to be outside the cage and not be blocked at all, try to give it enough support and guard is okay as long as it's outside the view of the lidar, also keep the two camera layout, also needs to not have its view obstructed, also why are there three orientations, I just need the one with perfect cg". Supersedes layout v4 / cage v2 (L2 inside the disk; the page showed the chosen layout with two alternatives beside it, which read as three orientations: this page shows one drone).
Script `nox/overseer/layout_v5.py` (~10 min: balance x start / dome / prop-snap scan), artifact `layout_v5_view.py`. Rule: CLAUDE.md "Cages are geodesic shells formed to the drone" (field sensors outside), `docs/generative.md` rule 0 Geodesic cage, check `system_check` shell + exempt openings.

Visual: https://claude.ai/artifact/VhDfnp75uoVsj8SnyrW2Qg (source `runs/overseer_layout/v5/motion.html`, still `preview.png`; one drone, with the L2 field (blue half-sphere outline) and the front camera view (green frustum) drawn)

System: Overseer, cage first 10.7 mm (E) at -Z (down camera) (Unitree L2, front camera outside the cage, exempt), shell 139 rods closed, h/w 0.28, fit 0.1 mm³ (E) (spokes, ties, keels, L2 guard ring/cartridge), bbox 113 L (E), extents 742 x 698 x 217 mm (E), mass 3375 of 2200 g (153 %) (E) over budget (total), CG 0.6 mm off thrust axis (E) [-0.0, +0.6, -63.9], 29.1 mm without swap parts (reported, L exception), probes 5.7 mm (E) edge at (-0.70, -0.06, -0.71) (down camera), access 19/19 clear

## Layout (C on E sizes; `checks.txt`)
- As v4 (the v2 drone: L2 at the nose axis 15° below +x, motors forward, catch 30 aft, pack top z -7, pods 35 ahead), but the L2, the front camera and their mount are left out of the shell.
- **L2 field clear:** every shell node centre 25.1 behind the L2 base plane (its 90° field); 0.0 % of the field crossed; the most forward cage member surface 3.0 behind the plane (guard ring nodes) (C).
- **Front camera view clear:** shell nodes 13.2 under its lower view plane (+2°) and behind the head plane (the head's own back, tilted 20° from the L2 axis, the tilt of 20 / 35 / 50 / 65 that keeps the L2 furthest back); nothing of the drone in its 73 x 89° view (C). **Down camera and flow:** nothing in their views through the bottom opening (C).
- **L2 support and guard (outside its view):** v2 seat, back block, side lugs, 2 keels to the front hub on the bottom plate, 4 ties into the shell behind its plane; guard ring r 62 round its axis, 8 behind its base plane, 8 rods + 16 struts into the shell (C). The L2 and the camera take a straight frontal hit themselves (outside the cage by Spark's ask); system_check reports them as exempt openings.
- The L2 sits 84 further forward than in v4 (base x 386.6) so the shell clears its plane with room for the guard ring; the balance moves the motor square 63.3 ahead (front arms r 246, rear r 159).
- Shell: frequency 2 class-I geodesic (equator at the prop plane), pushed out to the target, projected behind the three view limits, a 3 x 5 node grid on the flat front face, tubes over 300 split (DEFAULT); scan start 0.7 / 0.85 / 1.0 x dome 40 / 60 / 80 x prop snap; the most compact valid shell sets the structure bar, the least prop shadow within it wins (start 0.85, dome 40, snap). 50 nodes, 139 tubes 47 .. 296, 636 x 678 x 190 (h/w 0.28), 49.2 L (v4 64.7 L); largest cell Ø149; tubes 14.9 off the swept disks.
- Props: shadow 8.0 % above, 13.3 % below (E; v4 7.4 / 14.6).
- Opening, swap column, access as v4: swap column 0 mm³; access 19 / 19 (FC and Pi USB-C, every arm M3: two top screws by an L-key through a cell, the rear bottom ones through Ø9 driver holes in the belly sled, a requirement).

## One layout ("the one with perfect CG")
CG 0.6 mm off the thrust centre with the pack in (system_check, E). With the pack + cartridge out it is 29.1 mm off (reported, the L Balance exception, DEFAULT); balancing that too needs the pack under the thrust centre and front arms ~r 340+ (1P band), the alternative of v4 (`runs/overseer_layout/v4/alt_pack_axis/`), not drawn here.

## Carbon
- Shell: 139 tubes 6 x 4 (Spark 26 Oct; 6/4 DEFAULT wall), 23.0 m, 47 .. 296; socket bore 6.3.
- Links: 56 rods 3.0 solid, 5.75 m; printed struts (< 33): 2 guard ring struts of 29 (printed into the ring nodes).

## Mass (E)
3375 g (v4 3288): shell 785, links + guard ring 63, guard nodes 12 (E). T/W 1.96 at 1650 gf a motor (C on E). The extra 87 g over v4 buys the clear L2 and camera fields: the L2 84 further out needs longer front arms and a little more shell.

## Next
Frame model on the shell and the sensor head (crash 35 g in 14 directions incl. a nose hit on the L2 through its mount, floor landing on the feet, handling), then nodes, arms, belly sled; new funnel round the shell (box catch session).
