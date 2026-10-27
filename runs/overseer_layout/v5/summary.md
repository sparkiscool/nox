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

System: Overseer, cage first 12.7 mm (E) at -Z (flow MTF-02P) (field cones open: Unitree L2 75 deg, out 64.05 mm, camera left 55 deg, out 0.42 mm, camera right 55 deg, out 0.91 mm), shell 63 rods closed, h/w 0.28, fit 0.0 mm³ (E), bbox 109 L (E), extents 771 x 675 x 209 mm (E), mass 3178 of 2200 g (144 %) (E) over budget (total), CG 0.2 mm off thrust axis (E) [-0.1, -0.2, -64.1], 31.4 mm without swap parts (reported, L exception), probes 7.1 mm (E) edge at (-0.68, -0.19, -0.71) (flow MTF-02P), access 21/21 clear

## Cage v5 (27 Oct): half the tubes, close nodes merged
- **From cage v3 (main), its 131 tubes halved (Spark).** The free nodes were thinned to 16 (mirror pairs and centre nodes, farthest first) and the shell re-triangulated mirror-symmetric (flat facets in mirror pairs).
- **Close nodes merged.** Spark: "anytime there's a small triangle or short rods like that ... I don't want nodes so close together when they can just become one". Every tube under 95 (DEFAULT) collapses into one node. This merged the front and rear corner triangles of the base loop, which went from 8 to 6 nodes: nose and tail vertices on the centre plane, a hexagon 404 x 239 round the same swap zone. Two nodes left inside the shell were dropped.
- **No socket clashes.** No two tubes meet at a node under 20° (DEFAULT).
- **Repair.** The nodes then moved only as far as needed: every part with its margin inside every face, props 12 off, the L2 base plane, both camera views, the loop flat with its chords clear of the swap zone (`nox/overseer/cage_equal.py`, CE_MODE=reduce, CE_MERGE_MM=95, CE_MIN_ANGLE=20).
- **Result after merging:** 21 nodes, 54 tubes. **After the frame model's splits:** 24 nodes, 63 tubes (was 48 / 131), mirror-exact, 670 x 655 x 188, tubes 116 .. 446, 15.9 m. Mass 3178 g, T/W 2.08, prop shadow 7.4 % above and 13.9 % below.
- **Cost:** long spans (to 655) and large cells, left for the frame model (Next).
- **The base loop is shared** with the Scout cage (PR #124) and the funnel: their sessions were told (27 Oct).
- **Weighed and dropped (Spark):** cage v4, 11 cut lengths but taller (277) and bulkier (63.8 L): `nox/overseer/cage_v4_shell.json`.
- **Nodes:** TPU topology parts, one per design (12 designs; mirror pairs share one), `nox/overseer/cage_node.py`. Each node's sockets are only for the members that meet there: 6 mm tube (bore 6.3, 15 deep) or 3 mm rod (bore 3.3, 10 deep), 2.5 wall. The topology joins only those sockets, each member loaded in turn with the others holding (E loads until the frame model). Runs on run/cage_nodes-v1.

## Frame model v1 (27 Oct, F on E carbon properties; `runs/overseer_frame/v1/summary.md`)
- `nox/frame_model.py`, a 3D beam solver (checked against a cantilever), and `nox/overseer/cage_frame.py`, the cage + links on the core with the sensor head rigid on its keels and braces. Cases: crash 35 g through the first-touching member in 14 directions, a nose hit through the head guard, flight 4 g, handling 50 N at mid-span.
- **Found and fixed:**
  - 19 links (core ties, keels, L2 braces, five spokes) buckled as 3.0 rods (up to 7.9 x Euler). They are 6/4 tube now, the only other stock (`nox/overseer/link_sizes.json`).
  - The 655 top cross tube (2.6 x) and the two 413 front-top tubes (1.25 x) buckled; each is split at mid-span by a node (`nox/overseer/cage_split.py`).
  - The cage is 24 nodes, 63 tubes, worst tube 0.87 x.
- **Marginal:** three core ties, already 6/4 tube, sit at 1.01 to 1.15 x on pinned ends. Their core ends are bolted, so k ~0.7 gives ~0.6 x (E).
- **No member over stress.**
- **Handling:** the longest spans (438, 446) bend 17 to 18 mm under 50 N.
- **Node loads:** the per-socket forces feed the node and head runs (`node_loads.json`).

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
- **Printed PETG, a centre piece and two mirrored wings (the bed is 180).** It carries:
  - the drawn seat, back block and lugs;
  - the two camera pads;
  - the guard ring (r 62, 8 behind the L2 base plane);
  - the camera guard bars.

  It replaces the L2 ties, the guard ring rods and its 16 struts, the camera brackets' struts and the camera guards' struts. It holds the cage's front centre node (its tube sockets are part of the centre piece); 3.0 carbon struts run from each wing to the side node by the head where they meet its tubes at 20°+.
- **Carbon kept:** the 2 keels (289) and the 4 L2 braces (~310); they earn their length.
- **Mass:** 100 g in the system model (E: seat and lugs, camera plates, guards, joining material held to what the rods weighed); `head_system.json` carries the same number. The saved pieces' M mass replaces it.
- **Requirements** for its topology run (`head_system.json`): drawn bodies, held nodes, kept rods, keep-outs (the L2 and its field, both cameras and views, the cage tubes 3 off, props), loads (dimensions.md L: flight, crash 35 g in 14 directions at each payload's CG, sensor mounts ≥ 100 Hz, the lugs the fuse before the L2 base), print limits.
- **Print cut (DEFAULT):** a centre piece |y| ≤ 75 and two mirrored wings, bolted M3 on drawn pads. Topology runs: `runs/head_centre/`, `runs/head_wing/` (run branches).

## Layout (C on E sizes; `checks.txt`)
- **Drone:** the v2 drone (L2 at the nose axis 15° below +x, base (384.0, 0, 51); motors forward; catch 30 aft; pack top z -7). The motor square sits 62.0 ahead (front arms r 245, rear r 160) with the CG 0.2 off (E). The L2, the cameras and the head mount are outside the shell.
- **L2:** its field 0.0 % crossed.
- **Props and swap:** shell tubes 20.2 off the props; swap zone clear; access 21 / 21.

## One layout ("the one with perfect CG")
- **Pack in:** CG 0.2 mm off the thrust centre (system_check, E).
- **Pack + cartridge out:** 31.6 mm off. Accepted by Spark 27 Oct ("as long as with battery the drone is centered"), the L Balance exception.

## Carbon rods (6 mm tube, 3 mm rod)
- Shell: 63 carbon tubes 6/4, 15.9 m, 116 .. 446 (after the frame model's splits).
- Links: 32, 6.30 m: 19 sized up to 6/4 tube by the frame model (`link_sizes.json`), the rest 3.0 solid (spokes, core ties, keels, L2 braces, bay ties). The short ones at the head are printed now (the head mount).

## Mass (E)
3178 g: shell 495 (tubes 387 + nodes 108), links 127, sensor head mount 100. T/W 2.08 at 1650 gf a motor (C on E).

## Next
1. Save the head centre and wing topology runs (running).
2. Node runs v2 (`run/cage_nodes-v2`); the coupon bore pick (Spark) sets the socket bores.
3. The arms and belly sled; the deeper belly from core PR 123 lowers the loop plane.
4. The new funnel round the shell (box catch session).
