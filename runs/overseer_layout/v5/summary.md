# Overseer layout v5 / cage v3: sensor head outside the disk cage, mirror-exact (26-27 Oct 2026, concept)

Spark 26 Oct: "The lidar needs to be outside the cage and not be blocked at all, try to give it enough support and guard is okay as long as it's outside the view of the lidar, also keep the two camera layout, also needs to not have its view obstructed, also why are there three orientations, I just need the one with perfect cg". Supersedes layout v4 / cage v2 (L2 inside the disk; the page showed the chosen layout with two alternatives beside it, which read as three orientations: this page shows one drone).
Spark 27 Oct (sic): "There is no limit for the tube length do t worry about that delete it / There needs to be perfect left and right symmetry the nose and tail can be different / Don't worry about tube length just create the structure don't worry about materials just stay to using 6mm and 3mm tubes". The concept review (FAIL) asked for: a camera guard, the L2 exemption cut to its field cone, the L2 mount braced into the core, a fuse in the mount, the rule text, the pack-on-axis numbers. All done below.
Script `nox/overseer/layout_v5.py` (~10 min: balance x start / dome / prop-snap scan), artifact `layout_v5_view.py`. Rule: CLAUDE.md "Cages are geodesic shells formed to the drone" (field sensors outside), `docs/generative.md` rule 0 Geodesic cage, check `system_check` shell (closed + mirror) and field openings.

Visual: https://claude.ai/artifact/VhDfnp75uoVsj8SnyrW2Qg (source `runs/overseer_layout/v5/motion.html`, still `preview.png`; one drone, with the L2 field (blue half-sphere outline) and the front camera view (green frustum) drawn)

System: Overseer, cage first 7.5 mm (E) at (+0.50, -0.02, +0.87) (front camera) (field cones open: Unitree L2 75 deg, out 66.63 mm, front camera 55 deg, out 0 mm), shell 131 rods closed, h/w 0.31, fit 0.2 mm³ (E) (spokes, ties, keels, L2 guard ring/cartridge), bbox 118 L (E), extents 743 x 694 x 230 mm (E), mass 3382 of 2200 g (154 %) (E) over budget (total), CG 0.5 mm off thrust axis (E) [+0.1, +0.5, -62.4], 28.8 mm without swap parts (reported, L exception), probes 5.7 mm (E) edge at (-0.70, -0.06, -0.71) (down camera), access 19/19 clear

## Layout (C on E sizes; `checks.txt`)
- As v4 (the v2 drone: L2 at the nose axis 15° below +x, motors forward, catch 30 aft, pack top z -7, pods 35 ahead), but the L2, the front camera and their mount are left out of the shell.
- **L2 field clear:** every shell node centre 25.1 behind the L2 base plane (its 90° field); 0.0 % of the field crossed; the most forward cage member surface 3.0 behind the plane (guard ring nodes) (C).
- **Front camera view clear:** shell nodes 13.2 under its lower view plane (+2°) and behind the head plane (the head's own back, tilted 20° from the L2 axis, the tilt of 20 / 35 / 50 / 65 that keeps the L2 furthest back); nothing of the drone in its 73 x 89° view (C). **Down camera and flow:** nothing in their views through the bottom opening (C).
- **L2 support and guard (outside its view):** v2 seat, back block, side lugs, 2 keels to the front hub on the bottom plate, 4 ties into the shell behind its plane, and (review 27 Oct) 4 braces from the lugs (low and high) to the front arm beams 30 out of the root: the seat is a braced pyramid on the core, not a stalk on the keels. Guard ring r 62 round its axis, 8 behind its base plane, 8 rods + 16 struts into the shell (C). **Break order:** the apex lugs are the printed fuse, sized to let go before the L2 base (requirement for the frame model case "nose hit on the L2 through its mount", E).
- **Front camera guard (review 27 Oct):** the camera stood 33 above the cage; a bar of 2 printed nodes now runs over its top, behind its lens (out of its view) and behind the L2 base plane, 4 struts into the shell. Cage first over the camera 7.5 (worst direction up and forward, outside its view).
- **Field cones, not exemptions:** in `system_check` the L2 is a field opening 75° on its axis (its field is 90°; 75 keeps the ring's lateral cover gated: 37.5 outside the cone) and the front camera 55° on +x (its diagonal half-angle 51 + 4); cage first is gated everywhere else; probes skip the two (a face outside the cage can always be reached) and their protrusion is reported: the L2 stands 66.6 out of the cage along its axis, the camera 0.
- The L2 base sits at x 385.6 so the shell clears its plane with room for the guard ring; the balance puts the motor square 62.4 ahead (front arms r 245, rear r 159).
- **Shell, mirror-exact (Spark 27 Oct):** frequency 2 class-I geodesic (equator at the prop plane), every node moved with its mirror partner while it is pushed out to the target and projected behind the three view limits, a 3 x 5 node grid on the flat front face; the hull is merged into flat facets, a facet and its mirror share one triangulation, a centre-plane facet is fanned from its on-plane vertex or a centre node; links without a mirror get one. No tube-length limit (the 300 split and the 350 cap are gone). Scan start 0.7 / 0.85 / 1.0 x dome 40 / 60 / 80 x prop snap (18 shells, all valid); the most compact valid shell sets the bar (volume, mass), the least prop shadow within it wins: start 0.85, dome 60, snap. 48 nodes, 131 tubes 47 .. 437, 637 x 673 x 209 (h/w 0.31), 51.7 L; largest cell Ø141; tubes 13.3 off the swept disks. system_check: 0 tubes without a mirror.
- **Balance:** the shell pattern flips with the motor square (a node more or less), so the closer pattern is kept and the motor square trimmed onto the CG with it: CG x 0.1 off, 0.5 sideways (the hub electronics), E.
- Props: shadow 8.0 % above, 14.4 % below (E).
- Opening, swap column, access as v4: swap column 0 mm³; access 19 / 19 (FC and Pi USB-C, every arm M3; links kept off the screw key lines; the rear bottom ones through Ø9 driver holes in the belly sled, a requirement).

## One layout ("the one with perfect CG")
CG 0.5 mm off the thrust centre with the pack in (system_check, E). With the pack + cartridge out it is 28.8 mm off (reported, the L Balance exception, DEFAULT). The alternative, run 27 Oct (`V5_PACK=axis`, not drawn): the pack on the thrust axis balances pack out too (0.6 mm) but needs front arms r 373, 3645 g (+263) and T/W 1.81 (vs 1.95). Spark's call.

## Tubes
- Shell: 131 tubes 6 mm, 22.5 m, 47 .. 437 (no length limit, Spark 27 Oct).
- Links: 68 rods 3 mm, 8.2 m (spokes, core ties, L2 ties and braces, keels, bay ties, guard ring and camera guard), shortest 42.

## Mass (E)
3382 g: shell 763 (tubes 547 + nodes 216), links 90, guard nodes. T/W 1.95 at 1650 gf a motor (C on E). Over v5 of 26 Oct (3375): +7 g for the camera guard and the L2 braces.

## Next
Frame model on the shell and the sensor head (crash 35 g in 14 directions incl. a nose hit on the L2 through its mount, floor landing on the feet, handling), then nodes, arms, belly sled; new funnel round the shell (box catch session).
