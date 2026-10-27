# Overseer layout v5 / cage v5: halved geodesic disk, camera pair beside the L2, one-piece printed sensor head (26-27 Oct 2026)

Spark's asks, in order (26-27 Oct):
- the L2 outside the cage, not blocked at all; guards only outside its view; one layout with perfect CG; no tube-length limit; perfect left-right symmetry; 6 and 3 mm tubes only;
- "get rid of the down camera ... put the two cameras on either side the lidar pointing so their field of view is maximized";
- "move the flow pod 15 forward, as long as with battery the drone is centered we are okay";
- "for the mounts on the lidar and the camera, replace the bunch of little rods with a topology printed mount";
- "make sure we are not changing any cage structure for this";
- then the cage itself: "as many of the same length rods as possible", "you can actually change the base as well"; "divide the number of rods by half" ("ignore the tests"); finally "get rid of the number of same length rule just do the rods number reduction";
- "lets try to mount the lidar on the cage ... so we dont need the rods coming from the body", then "keep the design we have right now i need the lidar to have max performance";
- "make the camera mounted on the same print as the lidar mount".

Script `nox/overseer/layout_v5.py` (builds the drone round a fixed cage, `nox/overseer/cage_v5_shell.json`); the cage from `nox/overseer/cage_equal.py`; artifact `layout_v5_view.py`; camera alternatives `layout_v5_cams.py`; cage alternatives `layout_v5_cages.py`.

Visual: https://claude.ai/artifact/VhDfnp75uoVsj8SnyrW2Qg (source `runs/overseer_layout/v5/motion.html`; stills `preview.png` iso, `preview_top.png` top at frame 0, `preview_under.png` from below, `cameras_alt.png` the camera orientations over the L2 field, `cages_alt.png` cage v3 / v4 / v5 side by side). Head support options: https://claude.ai/artifact/4Z5Uv7N96VBaNhBLQyBE9e (`runs/overseer_head_cage/concept/`).

System: Overseer, cage first 7.7 mm (E) at (+0.90, +0.01, +0.43) (camera left) (field cones open: Unitree L2 75 deg, out 67.48 mm, camera left 55 deg, out 0.27 mm, camera right 55 deg, out 0.01 mm), shell 63 rods closed, h/w 0.28, fit 0.0 mm³ (E), bbox 116 L (E), extents 771 x 675 x 223 mm (E), mass 3215 of 2200 g (146 %) (E) over budget (total), CG 0.2 mm off thrust axis (E) [-0.1, -0.2, -63.2], 32.0 mm without swap parts (reported, L exception), probes 7.1 mm (E) edge at (-0.68, -0.19, -0.71) (flow MTF-02P), access 21/21 clear (`system_check.txt`)

## Cage v5 (27 Oct): half the tubes, close nodes merged
- **From cage v3 (main), its 131 tubes halved (Spark).** The free nodes were thinned to 16 (mirror pairs and centre nodes, farthest first) and the shell re-triangulated mirror-symmetric (flat facets in mirror pairs).
- **Close nodes merged.** Spark: "anytime there's a small triangle or short rods like that ... I don't want nodes so close together when they can just become one". Every tube under 95 (DEFAULT) collapses into one node. The base loop went from 8 to 6 nodes: a hexagon 404 x 239 (C) round the same swap zone.
- **No socket clashes.** No two tubes meet at a node under 20° (DEFAULT).
- **Repair.** The nodes moved only as far as needed: every part with its margin inside every face, props 12 off, the L2 base plane, both camera views, the loop flat with its chords clear of the swap zone (`cage_equal.py`, CE_MODE=reduce, CE_MERGE_MM=95, CE_MIN_ANGLE=20).
- **Result:** 24 nodes, 63 tubes (was 48 / 131), mirror-exact, 670 x 655 x 188 (C), tubes 116 .. 446 (C), 15.9 m (C), shell 495 g (E). Prop shadow 7.0 % above and 15.4 % below (E).
- **Alternatives (`cages_alt.png`):** cage v3 (main): 48 nodes, 131 tubes, 57 cut lengths, 22.5 m, ~762 g (E); cage v4 (fewest cut lengths, dropped by Spark): 11 lengths (C) but 277 tall (C) and ~773 g (E); cage v5 halves the tubes and saves ~270 g of shell (E). Its cost: long spans and large cells (largest cell Ø149, C), which the frame model checks below.
- **The base loop is shared** with the Scout cage (PR #124) and the funnel: their sessions were told and confirmed (27 Oct).
- **Nodes:** PETG topology parts (Spark 27 Oct, was TPU), 14 designs (C; mirror pairs share one), `nox/overseer/cage_node.py`. Each node's sockets are only for the members that meet there: 6 mm tube (bore 6.1 DEFAULT, 15 deep, C) or 3 mm rod (bore 3.1 DEFAULT, 10 deep, C), 2.5 wall (C). The bores wait on Spark's pick from the coupons (`runs/rod_coupons/v2/`). Loads: the frame model's socket forces (F). Runs: `run/cage_nodes-v3` (a child session).

## Frame model v1 (27 Oct, F on E carbon properties; `runs/overseer_frame/v1/summary.md`)
- `nox/frame_model.py` (3D beam solver, checked against a cantilever, static and modal) and `nox/overseer/cage_frame.py` (the cage + links on the core, the head rigid on its keels, braces and struts). Cases: crash 35 g through the first-touching member in 14 directions, a nose hit through the head guard, flight 4 g, handling 50 N at mid-span.
- **Solver fix (27 Oct).** The regularisation scaled by the mean diagonal let the stiff "rigid" head links put ~100 N/mm to ground on every node (F), so the first pass read low. It now uses the median; results are converged over rigid-link stiffness 1e5..1e8.
- **Sized:** 30 of the 36 links are 6/4 tube now (`link_sizes.json`), the rest 3.0 rod; every link ≤ 0.63 x its buckling load (F, k 0.7 with a core end, 0.5 keels / braces, E).
- **Reported over (not gated; the halved cage, Spark "ignore the tests"):** tubes 1-19 / 3-18 1.59 x on a top hit through the camera guards, tubes 11-18 / 12-19 1.29 x on a side-corner hit (F). Splitting moved the buckling to the next long tube each time, so the splits stop at the frame v1 set.
- **Modes:** lowest 144 Hz with rigid joints, the head in the 214 Hz mode (F on E); with PETG nodes the cage's lowest mode is 126 Hz, with TPU it was 26 Hz (F on E, `runs/overseer_head_cage/concept/`). No member over stress.
- **Handling:** the longest spans (438, 446) bend 17 to 18 mm under 50 N (F).

## Camera pair (Spark 27 Oct)
- **Down camera removed** with its pod; the flow sensor stays in the belly, its pod 15 ahead of v2's (Spark).
- **Method:** every orientation of the mirrored pair (portrait / landscape, yaw 0..70 out, pitch -30..+30). Ranked by:
  1. the union of the two fields (within 2 % is a tie);
  2. the blind wedge straight ahead on the horizon;
  3. the share of the L2 field.
- **Mount:** the most compact one beside the L2 whose head (seat, guard ring, both cameras and plates) fits one print: oriented box ≤ 175, the A1 mini's 180 cube (Spark: the cameras on the same print as the L2 mount).
- **Chosen:** landscape (89 across x 73 up), yawed 45° out, pitched 10° down. Lenses (390, ±75, 122) (C), 13 behind the L2 base plane (the only clear mount that fits). Results: combined 3.42 sr, 27 % of the sphere, no overlap (C); nose gap 2° (C); 54 % of the L2 field (C); nothing of the drone in either view (C).
- **Alternatives** (`cameras_alt.png`): portrait 50° out, 15° down: 3.45 sr, but a 29° blind wedge at the nose; portrait 37.5° out, level: 3.45 sr, gap 3° (C).
- **Guards:** a bar of 2 printed nodes over each camera, behind its lens. Cage first is 7.7 outside the 55° field cones (E).

## Sensor head mount (one print with both cameras)
- **Printed PETG, one piece** (`nox/overseer/head_mount.py head`): the drawn seat, back block and lugs, both camera plates (M2.5 inserts), the guard ring (r 62, 8 behind the L2 base plane), both camera guard bars, the cage's front centre node with its tube sockets, sockets for the 2 keels, 4 L2 braces and 4 head struts. Drawn 56 x 177 x 163 (C).
- **Supports (Spark: "max performance"):** the keels and braces to the core stay; 4 struts 6/4 run to the side cage nodes 15 / 16 / 18 / 19 (sockets searched inside the print box, behind the L2 base plane, out of both camera views, 20°+ from the nodes' tubes). Head mode 206 Hz on PETG cage nodes (223 with rigid joints, 120 with TPU; F on E), over the 100 Hz target. Options weighed: `runs/overseer_head_cage/concept/summary.md` (the cage-only head reads 32..50 Hz on TPU nodes, F on E).
- **Loads (topology):** the L2 at its CG (28 ahead of its base) through its 4 bolts with its moment, both cameras at their plates, crash 35 g in 14 directions, flight, guard hits (dimensions.md L).
- **Mass:** 102 g in the system model (E); the saved part's M mass replaces it. Topology run: `runs/head_mount/v1` (branch `run/head_mount-v1`).

## Layout (C on E sizes; `checks.txt`)
- **Drone:** the v2 drone (L2 at the nose, axis 15° below +x, base (384.0, 0, 51); motors forward; catch 30 aft; pack top z -7). The motor square sits 66.4 ahead (front arms r 249, rear r 157) with the CG 0.2 off (C on E).
- **L2:** its field 0.0 % crossed (C).
- **Props and swap:** shell tubes 20.3 off the props, links 13.7 (C); swap zone clear (M); access 21 / 21 (C).

## One layout ("the one with perfect CG")
- **Pack in:** CG 0.2 mm off the thrust centre (system_check, E).
- **Pack + cartridge out:** 32.0 mm off (system_check, E). Accepted by Spark 27 Oct ("as long as with battery the drone is centered"), the L Balance exception.

## Carbon (6 mm tube, 3 mm rod)
- Shell: 63 tubes 6/4, 15.9 m, 116 .. 446 (C).
- Links: 36, 6.86 m (C), shortest 71 (bay tie -1+1.1, C): 30 tube 6/4 (the frame model's sizing), 6 rod 3.0 (bay ties). The spokes, ties, keels, braces and head struts each run longer than the 33 break-even.

## Mass (E)
3215 g: shell 495 (tubes 387 + nodes 108), links 159, sensor head mount 102 (E). T/W 2.05 at 1650 gf a motor (C on E). Over the 2200 g slot by 1015 g (146 %): v3's cage mass slot predates the L2 (230 P), the 650 g pack, the 338.5 g latch set (M) and the full cage; against cage v3 the halved shell saves ~270 g (E). The extra grams over v3's link set (~60 g, E) buy the 6/4 links that hold the 35 g crash.

## Next
1. Save the one-piece head mount (`run/head_mount-v1`, running).
2. Node runs v3 (child session); Spark's bore pick from the v2 coupons sets the socket bores.
3. The arms and belly sled; the deeper belly from core PR 123 lowers the loop plane.
4. The new funnel round the shell (design center session).
