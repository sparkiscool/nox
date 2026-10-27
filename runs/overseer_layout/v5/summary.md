# Overseer layout v5 / cage v5: halved geodesic disk, camera pair beside the L2, printed sensor head (26-27 Oct 2026, concept)

Spark's asks, in order (26-27 Oct):
- the L2 outside the cage, not blocked at all; guards only outside its view; one layout with perfect CG; no tube-length limit; perfect left-right symmetry; 6 and 3 mm tubes only;
- "get rid of the down camera ... put the two cameras on either side the lidar pointing so their field of view is maximized";
- "move the flow pod 15 forward, as long as with battery the drone is centered we are okay";
- "for the mounts on the lidar and the camera, replace the bunch of little rods with a topology printed mount";
- "make sure we are not changing any cage structure for this";
- then the cage itself: "as many of the same length rods as possible", "you can actually change the base as well"; "divide the number of rods by half"; finally "get rid of the number of same length rule just do the rods number reduction".

Script `nox/overseer/layout_v5.py` (builds the drone round a fixed cage, `nox/overseer/cage_v5_shell.json`); the cage from `nox/overseer/cage_equal.py`; artifact `layout_v5_view.py`; camera alternatives `layout_v5_cams.py`.

Visual: https://claude.ai/artifact/VhDfnp75uoVsj8SnyrW2Qg (source `runs/overseer_layout/v5/motion.html`; stills `preview.png` iso, `preview_top.png` top at frame 0 (the mirror), `cameras_alt.png` the camera orientations over the L2 field)

System: Overseer, cage first 7.4 mm (E) at (+0.90, +0.01, +0.43) (camera left) (field cones open: Unitree L2 75 deg, out 64.05 mm, camera left 55 deg, out 0 mm, camera right 55 deg, out 0.68 mm), shell 64 rods closed, h/w 0.28, fit 0.2 mm³ (E) (spokes, ties, keels, L2 guard ring/cartridge), bbox 108 L (E), extents 769 x 678 x 207 mm (E), mass 3120 of 2200 g (142 %) (E) over budget (total), CG 0.2 mm off thrust axis (E) [-0.1, -0.2, -65.7], 31.8 mm without swap parts (reported, L exception), probes 8.5 mm (E) edge at (-0.56, +0.43, -0.71) (cartridge), access 21/21 clear

## Cage v5 (27 Oct): half the tubes
- **From cage v3 (main), its 131 tubes halved (Spark):** the free nodes thinned to 16 (mirror pairs and centre nodes, farthest first), the 8 base-loop nodes kept, re-triangulated mirror-symmetric (flat facets in mirror pairs). Then the nodes moved only as far as needed (least squares, staying near the start): every part with its margin inside every face, props 12 off, the L2 base plane, both camera views, the base loop flat with its chords clear of the swap zone, free nodes above it.
- **25 nodes, 64 tubes** (was 48 / 131), 40 triangles, mirror-exact (0 tubes without a mirror, system_check gate). 663 x 658 x 187, 49.7 L (v3 51.7 L). Tubes 52 .. 653 (mean 245), 15.7 m; largest cell larger. The base loop is as in v3 (352 x 209 round the receiver + 30).
- **Gains:** shell 495 g (was 763), mass 3120 g (-260), T/W 2.12 (was 1.95), prop shadow 5.3 % above, 12.8 % below (was 8.6 / 14.4).
- **Cost:** long spans (to 653) and larger cells; their buckling and the cells' guarding go to the frame model (Next).
- **Weighed and dropped (Spark):** cage v4, the same 131 tubes moved to 11 cut lengths (every tube within ±1.4), but taller and bulkier (277 high, 63.8 L): `nox/overseer/cage_v4_shell.json`, kept for reference. Fewer lengths (3 to 6, also with spans cut in 2 to 5 pieces) did not converge on this shape.

## Camera pair (Spark 27 Oct)
- **Down camera removed** with its pod; the flow sensor stays in the belly, its pod 15 ahead of v2's (Spark), so every rear arm screw comes through a sled hole.
- **Method:** every orientation of the mirrored pair (portrait / landscape, yaw 0..70 out, pitch -30..+30). Ranked by:
  1. the union of the two fields (within 2 % is a tie: every non-overlapping pair gets the same, within sampling);
  2. the blind wedge straight ahead on the horizon (the fields meet at the nose);
  3. the share of the L2 field.
- **Mount:** the most compact one beside the L2. The body sits 2 behind the L2 base plane, clear of the guard ring and 3 off the cage. Nothing of the drone is in view.
- **Chosen:** landscape (89 across x 73 up), yawed 45° out, pitched 10° down. Lenses (385, ±90, 103), 13 behind the L2 base plane (the most compact of 15 clear mounts). Results:
  - combined 3.42 sr, 27 % of the sphere, no overlap;
  - nose gap 2°;
  - 54 % of the L2 field (C);
  - nothing of the drone in either view (C).
- **Alternatives** (`cameras_alt.png`):
  - portrait, 50° out, 15° down: 3.45 sr, 55 % of the L2 field, but a 29° blind wedge at the nose;
  - portrait, 37.5° out, level: 3.45 sr, gap 3°.
- **Guards:** a bar of 2 printed nodes over each camera, behind its lens. Cage first is 7.4 outside the 55° field cones.

## Sensor head mount (Spark 27 Oct: a topology printed mount instead of the little rods)
- **One printed PETG part.** It carries:
  - the drawn seat, back block and lugs;
  - the two camera pads;
  - the guard ring (r 62, 8 behind the L2 base plane);
  - the camera guard bars.

  It replaces the L2 ties, the guard ring rods and its 16 struts, the camera brackets' struts and the camera guards' struts. It holds 7 cage nodes as they are.
- **Carbon kept:** the 2 keels (289) and the 4 L2 braces (~310); they earn their length.
- **Mass budget:** 73 g, what it replaces, so the balance stays (topology material 38 g of it, E).
- **Requirements** for its topology run (`head_system.json`): drawn bodies, held nodes, kept rods, keep-outs (the L2 and its field, both cameras and views, the cage tubes 3 off, props), loads (dimensions.md L: flight, crash 35 g in 14 directions at each payload's CG, sensor mounts ≥ 100 Hz, the lugs the fuse before the L2 base), print limits.
- **Print cut (DEFAULT):** a centre piece |y| ≤ 75 and two mirrored wings, bolted M3 on drawn pads. The topology run is the next step (a heavy run, its own PR).

## Layout (C on E sizes; `checks.txt`)
- **Drone:** the v2 drone (L2 at the nose axis 15° below +x, base (384.0, 0, 51); motors forward; catch 30 aft; pack top z -7). The motor square sits 62.0 ahead (front arms r 245, rear r 160) with the CG 0.2 off (E). The L2, the cameras and the head mount are outside the shell.
- **L2:** its field 0.0 % crossed.
- **Props and swap:** shell tubes 14.9 off the props; swap zone clear; access 21 / 21.

## One layout ("the one with perfect CG")
- **Pack in:** CG 0.2 mm off the thrust centre (system_check, E).
- **Pack + cartridge out:** 31.8 mm off. Accepted by Spark 27 Oct ("as long as with battery the drone is centered"), the L Balance exception.

## Carbon rods (6 mm tube, 3 mm rod)
- Shell: 64 carbon tubes 6 mm, 15.7 m, 52 .. 653.
- Links: carbon rods 3 mm (spokes, core ties, keels, L2 braces, bay ties). The short ones at the head are printed now (the head mount).

## Mass (E)
3120 g: shell 495 (tubes 382 + nodes 112), links 70, head mount budget 73. T/W 2.12 at 1650 gf a motor (C on E).

## Next
1. Topology run of the sensor head mount on `head_system.json`.
2. Frame model on the halved shell: the long spans' buckling, cell guarding, the crash cases.
3. The nodes, arms and belly sled.
4. The new funnel round the shell (box catch session).
