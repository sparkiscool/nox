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

Visual: https://claude.ai/artifact/VhDfnp75uoVsj8SnyrW2Qg (the layout, republished with each saved part; source `runs/overseer_layout/v5/motion.html`; stills `preview.png`, `preview_top.png`, `preview_under.png`, `cameras_alt.png`, `cages_alt.png`). Head on 3 rods: `runs/overseer_head_rods/v1/options.png`. Head support options before: https://claude.ai/artifact/4Z5Uv7N96VBaNhBLQyBE9e.

System: Overseer, cage first 5.2 mm (E) at (+0.90, +0.01, +0.43) (camera left) (field cones open: Unitree L2 75 deg, out 72.88 mm, camera left 55 deg, out 3.47 mm, camera right 55 deg, out 5.9 mm), shell 63 rods closed, h/w 0.32, fit 0.0 mm³ (E), bbox 126 L (E), extents 767 x 667 x 247 mm (E), mass 3447 of 2200 g (157 %) (E) over budget (total, hub deck 68.0 of 55 g), CG 0.2 mm off thrust axis (E) [+0.2, -0.1, -65.7], 25.4 mm without swap parts (reported, L exception), probes 8.5 mm (E) edge at (-0.68, -0.19, -0.71) (cartridge), access 37/37 clear (`system_check.txt`; the arms' third screw checked when arm v8 and core v4 merge: V5_THIRD_SCREW=1)

## Cage v5 (27 Oct): half the tubes, close nodes merged
- **From cage v3 (main), its 131 tubes halved (Spark).** The free nodes were thinned to 16 (mirror pairs and centre nodes, farthest first) and the shell re-triangulated mirror-symmetric (flat facets in mirror pairs).
- **Close nodes merged.** Spark: "anytime there's a small triangle or short rods like that ... I don't want nodes so close together when they can just become one". Every tube under 95 (DEFAULT) collapses into one node. The base loop went from 8 to 6 nodes: a hexagon 404 x 239 (C) round the same swap zone.
- **No socket clashes.** No two tubes meet at a node under 20° (DEFAULT).
- **Repair.** The nodes moved only as far as needed: every part with its margin inside every face, props 12 off, the L2 base plane, both camera views, the loop flat with its chords clear of the swap zone (`cage_equal.py`, CE_MODE=reduce, CE_MERGE_MM=95, CE_MIN_ANGLE=20).
- **Result:** 24 nodes, 63 tubes (was 48 / 131), mirror-exact, 670 x 655 x 188 (C; 211 tall with the loop at z -79.9, below), tubes 116 .. 446 (C), 15.9 m (C), shell 495 g (E). Prop shadow 7.0 % above and 15.4 % below (E).
- **Alternatives (`cages_alt.png`):** cage v3 (main): 48 nodes, 131 tubes, 57 cut lengths, 22.5 m, ~762 g (E); cage v4 (fewest cut lengths, dropped by Spark): 11 lengths (C) but 277 tall (C) and ~773 g (E); cage v5 halves the tubes and saves ~270 g of shell (E). Its cost: long spans and large cells (largest cell Ø149, C), which the frame model checks below.
- **The base loop is shared** with the Scout cage (PR #124) and the funnel: their sessions were told and confirmed (27 Oct).
- **Nodes:** PETG topology parts (Spark 27 Oct, was TPU), 14 designs (C; mirror pairs share one), `nox/overseer/cage_node.py`. Each node's sockets are only for the members that meet there: 6 mm tube (bore 6.2, 15 deep, C) or 3 mm rod (bore 3.2, 10 deep, C), 2.5 wall (C), a solid floor 3.0 behind each socket (C; 1.0 left node 2's arm-to-boss joint a neck under 1.8 after the loop drop, 27 Oct); +19.6 g of drawn PETG over the 23 cage nodes, ~0.85 g a node (C), the price of every socket joining its arm through solid material. Alternative weighed: arms flared to the boss (a gusset), but the neighbouring bores cut slivers off the flares (2 specks of 5 mm³ on node 2, C). Section: https://claude.ai/artifact/7dr1zrbULDDE5FnK137DZ9 (`runs/cage_nodes/floor/floor_section.png`). The bores are Spark's pick from the coupons (`runs/rod_coupons/v2/`: the largest fits, P). Loads: the frame model's socket forces (F). Runs: `run/cage_nodes-v3` (a child session).

## Frame model v1 (27 Oct, F on E carbon properties; `runs/overseer_frame/v1/summary.md`)
- `nox/frame_model.py` (3D beam solver, checked against a cantilever, static and modal) and `nox/overseer/cage_frame.py` (the cage + links on the core, the head rigid on its keels, braces and struts). Cases: crash 35 g through the first-touching member in 14 directions, a nose hit through the head guard, flight 4 g, handling 50 N at mid-span.
- **Solver fix (27 Oct).** The regularisation scaled by the mean diagonal let the stiff "rigid" head links put ~100 N/mm to ground on every node (F), so the first pass read low. It now uses the median; results are converged over rigid-link stiffness 1e5..1e8.
- **Sized:** 25 of the 31 links are 6/4 tube, the rest 3.0 rod; every link ≤ 0.70 x its buckling load (F; k 0.7 with a core end, E).
- **Reported over (not gated; the halved cage, Spark "ignore the tests"):** tubes 11-18 / 12-19 1.49 x on a side-corner hit, 15-16 1.15 x (F). The camera-guard top hit is gone with the ring.
- **Modes:** lowest 148 Hz with rigid joints, the head in the 168 Hz mode (F on E, lumped); the head on its 3 rods 151.8 / 128.4 Hz rigid / PETG joints (`runs/overseer_head_rods/v1`).
- **Handling:** the longest spans (438, 446) bend 17 to 18 mm under 50 N (F).

## Arm nodes and core v2 (27 Oct)
- **Arm session (Spark "add the arm nodes"):** the 3 spokes of each arm leave from a printed node beside the motor (`layout_v5.py` ARM_NODE: front (178.9, ±146.7, 8), rear (-53.3, ±119.7, 25), C), ranked from that node (the same 3 cage nodes as before, C).
- **Core v2:** the keels end 5 deeper in the hub deck (KEEL_INSET, C); the front hub block is the core's, off the head part and its mass (`apex_head`).
- **Load into the arms** (`runs/overseer_frame/v1/arm_loads.json`, F on E, 35 g crash, core rigid): rear spoke node 1932 N, front 1090 N (3-rod head). Spark 27 Oct: the arm root joint is taken as good.
- **L2 braces and keels:** removed 27 Oct (Spark: max 3 rods to the lidar; the keels already duplicated the low braces: 186 vs 187 Hz, F on E). The head hangs on 3 rods to the cage (below).
- **Socket loads:** largest axial 1405 N at node 0 (spoke 3.1), side ≤ 113 N (F).

## Camera pair (Spark 27 Oct)
- **Down camera removed** with its pod; the flow sensor stays in the belly, its pod 15 ahead of v2's (Spark).
- **Method:** every orientation of the mirrored pair (portrait / landscape, yaw 0..70 out, pitch -30..+30). Ranked by:
  1. the union of the two fields (within 2 % is a tie);
  2. the blind wedge straight ahead on the horizon;
  3. the share of the L2 field.
- **Mount:** the most compact one beside the L2 whose head (seat, guard ring, both cameras and plates) fits one print: oriented box ≤ 175 (C), the A1 mini's 180 cube (Spark: the cameras on the same print as the L2 mount).
- **Chosen:** landscape (89 across x 73 up, E), yawed 45° out, pitched 10° down (C). Lenses (390, ±75, 122) (C), 13 behind (C) the L2 base plane (the only clear mount that fits). Results: combined 3.42 sr, 27 % of the sphere, no overlap (C); nose gap 2° (C); 54 % of the L2 field (C); nothing of the drone in either view (C).
- **Alternatives** (`cameras_alt.png`): portrait 50° out, 15° down: 3.45 sr, but a 29° blind wedge at the nose; portrait 37.5° out, level: 3.45 sr, gap 3° (C).
- **Guards:** a bar of 2 printed nodes over each camera, behind its lens. Cage first is 7.7 outside the 55° field cones (E).

## Sensor head mount (one print with both cameras)
- **Printed PETG, one piece** (`nox/overseer/head_mount.py head`, v5): only what bolts or plugs in is drawn: 4 L2 bolt bosses (Ø16, a 5 pad, a 3.5 wall round the Ø9 driver), 4 insert bosses per camera (Ø12), 3 rod sockets 6.2 x 15 with a 3.0 floor; 21.6 g drawn (C). No seat disc, lugs, camera plates, guard ring or camera bars (Spark 27 Oct: "that ring around the lidar does nothing"; the cage guards the L2 and cameras outside their fields).
- **Supports (Spark 27 Oct: max 3 rods):** 3 carbon 6/4 rods to the cage, none to the body: a centre rod from above the L2's centre (u +40, 12 behind its base plane) to node 17, a pair from above the cameras (u +100, y ±80) to nodes 15 / 16. Every socket boss (12 deep + a 3.0 floor) is checked on the head's exact keep-outs; the L2 is rolled 180 about its axis (cable down, DEFAULT 28 Oct) so the centre rod clears its cable exit. 119.4 / 94.2 Hz rigid / PETG joints (F on E, `runs/overseer_head_rods/v1/summary.md`); cameras 5.2 behind the cage, L2 51 (system_check).
- **Loads (topology):** the system's own (`runs/overseer_frame/v1/head_loads.json`): every frame case's forces on the head at the 2 side rod sockets, the centre rod socket held; the head's 35 g in 14 directions and flight: the L2 at its CG through its 4 bolts (force and moment), each camera on its bosses, the mount's own mass. v4 had a guard-ring case at 124 N (E) where the system put 853 N (F) into the head; the 3-rod head takes no cage crash load (≤ 25 N, F).
- **Mass:** 143.7 g in the system model (E: v4's M) until v5 saves. v3 / v4 runs (`run/head_mount-v1`, `-v4`): solid FAIL (socket rim and floor specks; fixed with the 3.0 floor).

## Layout (C on E sizes; `checks.txt`)
- **Drone:** the v2 drone (L2 at the nose, axis 15° below +x, base (384.0, 0, 51) (C); motors forward; catch 30 aft; pack top z -7). The motor square sits 66.4 ahead (front arms r 249, rear r 157) with the CG 0.2 off (C on E).
- **L2:** its field 0.0 % crossed (C).
- **Props and swap:** shell tubes 20.3 off the props, links 16.8 (C); swap zone clear (M); access PASS with the core's 37 tool paths (system_check, C).
- **Core parts in (27 Oct, cage sync rule):** the core session's saved parts (hub deck v9, flow clip, FC stack, Pi 5, UBEC, buck, RX, wires, pack, cartridge, receiver, XT60; decimated in `saved/core/`, from `part/overseer-core-pieces` @ 26bf1475) replace the layout's placeholders. v5 is updated in place (the rows say v5b); the state before is commit b9f6137c.
  - Their cartridge bottom is z -76.9 (M), so the base loop drops 22.9 (C) to z -79.9 (C): opening plane = cartridge bottom - 3.
  - Front core ties root on the hub deck tower front face at (114, 0, 20) (C) and aim forward to nodes 7 / 17 (C). Aimed back, one crossed the FC nut driver lines (access FAIL).
  - The rear core ties (2, new) root at (-95.5, 0, -1) (C) and run to nodes 13 / 14: with the real cartridge they clear it. That is the 36 -> 38 links.
  - TPU feet are Ø16 x 3 caps (C) under the loop nodes, contact z -93.1 (C).
  - The 8 bay ties root at x 30 / -90 (C), y ±56 (C), z -5.1 (C), the sled's side faces in dimensions.md (were the old sled corners at y ±72, C); the sled is not drawn yet (Next).
  - Frame model: worst tube buckling 1.92 (F), link 0.72 (F), lowest mode 148 Hz (F on E); every tie root is taken as rigid core, the 8 bay tie roots on the undrawn sled included, so the arm and node loads assume a belly root that does not exist yet (E).
  - **Compactness cost:** the cage grows 26 taller (223 -> 249, E) and the bbox 116 -> 130 L (+12 %, E). Alternative weighed: keep the loop at z -57 and dip only the swap opening round the cartridge (a recessed pocket, not modelled): it adds nodes and tubes under the 95 merge length round a 23 deep pocket, and the loop is the funnel interface the Scout shares, so a pocket changes both. Dropping the loop keeps the shell as it is (DEFAULT); the other way is a shallower belly, which the latch set's depth fixes (core session).

## One layout ("the one with perfect CG")
- **Pack in:** CG 0.2 mm off the thrust centre (system_check, E), with the layout's E arm masses. With the saved v5d arm meshes and masses the arm session reads 5.8 mm (balance FAIL); DEFAULT: rebalance once, when the front arm is final (the motor square moves the arm and spoke ends).
- **Pack + cartridge out:** 27.4 mm off (system_check, E). Accepted by Spark 27 Oct ("as long as with battery the drone is centered"), the L Balance exception.

## Carbon (6 mm tube, 3 mm rod)
- Shell: 63 tubes 6/4, 15.9 m, 116 .. 446 (C).
- Links: 31, 5.54 m (C), shortest 35 (head rod c, C): 25 tube 6/4 (the frame model's sizing and the 3 head rods), 6 rod 3.0. Each runs longer than the 33 break-even.

## Mass (E)
3405 g (layout; system check 3447): shell 679 (tubes 389 E + nodes 289.4 M, the saved v5 nodes), links 125, sensor head 143.7 (E). T/W 1.94 at 1650 gf a motor (C on E). Over the 2200 g slot by 1205 g (155 %; system check 1247 g, 157 %): v3's cage mass slot predates the L2 (230 P), the 650 g pack, the 338.5 g latch set (M) and the full cage. The nodes weigh 289 g against the 108 g model (24 x 4.5, E): each 6 mm socket is about 1.4 g of PETG at 2.0 walls (C). The 3-rod head saves about 49 g of links against the 8-rod head.

## Next
1. Save the one-piece head mount (`run/head_mount-v1`, running).
2. Node runs v3 (child session); Spark's bore pick from the v2 coupons sets the socket bores.
3. The arms (new loads above). The body tie roots (front, rear, side, bay) are body nodes of the cage session (DEFAULT; the core's scope leaves them out, Spark 27 Oct). The sled is not drawn (Spark: an empty slot for now), so the bay ties have no root part yet: a belly ring round the slot, or drop them (open).
4. The new funnel round the shell (design center session).
