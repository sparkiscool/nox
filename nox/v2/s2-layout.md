# Scout v2.1 layout (19 to 20 Oct 2026): the lantern cage, electronics under the plate

Spark 19 Oct 19:12: v2 proved worse than v1. Kept from v2: the bottom-mounted battery (cartridge v3, bottom latch, contacts, receiver interface) and the way it fits the box funnel. Everything else reverts to a full cage. Props stay on top (prop plane z 41.2). Electronics no longer need to be on top. **19:45: more supports, structured, plank-proof, cannot land upside down, no reverse spin, fits the funnel. 20:20: too many nodes. 20:31: no material where there is nothing to protect, dome shape, supports toward the bottom. 20:49 / 20:50: no wheels, no moving parts, slides into the funnel on geometry; nothing disconnected; the bottom is not the battery; glove fit. 21:01: structure first, then the drone inside it. 21:13: deep research on cages and collision designs, compare. 22:00: apply it. 23:25: run the nodes through topology, show the assembled CAD. 20 Oct 02:42: sockets wider than before (holes were too tight), soak resin, slide in easier. 02:43: nodes that touch merge; a rod too short to exist goes. 20 Oct: thicker printed structure around the prop guard (the topology parts), rods stay 3.0 (cage v13). 20:23: flat feet on the bottom that still fit the funnel, so the drone lands flat on the ground. 22:16: smooth every node, one body, no sharp edges. 23:41: much smoother.** Drone coords: x nose, y left, z up, mm. Tags: P measured, M mesh, C CAD, E estimate, DEFAULT = decided here, change if Spark objects.

Artifact "Scout v2.1 Layout" (page version 13 = cage v13: drawings side / front / top with every keep-out and the feet, socket v2 section, topology table, the assembled CAD as a glTF with close-ups per part incl. Feet, cut list, self-righting chart, hits, problems). Research report: `claude/research/2026-10-19-cage-collision-designs.md`. Scripts: **Nox Source `nox/v2/lantern/`**: `s21_lantern12.py` (structure + checks), `s21_cage6.py` (rest finder, roll paths), `s21_layout.py` (electronics sweep), `views13.py` (drawings), `node_topo3.py` (v13: skeleton, sleeves, feet, ground plane; built on `node_topo2.py` = socket v2, part problems, keep-outs, run), `run_v13.py` (mass steps), `smooth14.py` (finish pass v2) + `sharp14.py` (edge audit) + `prep14.py` (records, viewer meshes), `assemble14.py` (instanced viewer meshes), `socket_coupon.py`, `assemble13.py` (CAD assembly + clearance check on the meshes, v13 parts with v12 fallback), `viewer.js`, `patch_html13.py`; the v12 scripts stay beside them.

## The structure: a lantern (cage v13)
One rod size (3.0 solid carbon), rings joined by printed parts. Read from the bottom:
1. **Sled**, z -47: two rails y ±32, x -52..37.5, and a tail cross at x -52, 2.5 under the battery. What the drone lands on, **on 4 flat feet** (section Flat feet). Open at the nose: the box's swap head rises through y -26..14. Tail cross at -52 (v11 -50) so its sockets clear the swap head body (x ≥ -47).
2. **Glove**: 8 rods from the skid ring down the funnel's 45° faces to the sled. Side columns straight to the rail sleeves. Tail corners through knees at (-66, ±52, -0.8): swept inward from v11's (-75, ±64) until the knee's sockets + 1.5 wall clear the 168 opening at 5° yaw, including a 6 mm band above the rim. Nose corners onto the **printed nose legs** at (37.5, ±32, -7..-47) (v11 rod legs were 40 long with 12 bare; x 40 → 37.5 because the leg sockets sat 1.2 inside the box arm sweep x ≥ 42 below z -9.5).
3. **Drum**: two clipped-square rings (flats ±112, chamfers (±112, ±100)..(±100, ±112)) at z 33.2 and z 60, held by **8 printed columns** (4 corner, 2 side, nose, tail). The v11 posts were 26.8 long between two sockets that need 28: they were inside their nodes. Both rings TPU-sleeved (4 x 3 tubing, OD 4). The skid ring carries the 8 skid strips.
4. **Crown**: 8 spokes from the column tops to one printed hub (drawn octagon ring **r 36, 6 x 6**, behind the socket floors, + 8 sockets) at z 85, the only thing above the props. Upside down the drone sits on the hub flat and one forward blip on two motors rolls it off.

Merge rule (DEFAULT, Spark 02:43): a rod with under 20 mm bare between its two sockets is printed as part of the parts it joins. Applied: 8 posts → columns, 2 nose legs → nose legs, 2 camera rails → the nose column (it carries the camera on 4 M2 pads). **The v11 nose post at (112, 0) stood in front of the camera lens**: the nose column splits its skid sleeve to y ±14 (8..20) and leaves the lens window open.

**17 printed parts** (PA12-CF): corner column nose x2 / tail x2, side column x2, nose column, tail column, hub, nose leg x2, tail knee x2, sled tail x2, rail sleeve x2. Plus the 2 v2 side hubs on the plate bolts (struts) on TPU grommets.

| Rod | Cut (axis length, C) |
|---|---|
| Skid ring 4 x 200; shoulder ring 4 x 200 | drum |
| Spokes: corners 4 x 112.7, tees 4 x 76.2 | crown |
| Glove: nose 2 x 108.6; tail 2 x 75.3 + low 2 x 52.3; sides 2 x 110.4 (v13: the rail sleeve socket starts 3 further out) | glove |
| Sled: rails 2 x 89.5, cross 1 x 64 | sled |
| Struts 2 x 62.7 | plate |

29 rods, 3.42 m axis (3.30 m cut, ending at the socket floors), 36 g. Printed parts **100.0 g M** (finished v13, finish v2; v12 73.1); side hubs 3.7 M; sleeves ~5, strips ~3, grommets 2, epoxy 2 → **cage ~152 g E** (v12 ~125, v11 ~116 with the narrow sockets).

## Rod layout v14 (22 Oct, research chat; Spark 13:41 "the arrangement of the rods ... make it better"; DEFAULT)
Frame model `lantern/rods14.py` (every rod + printed member as beams, core rigid at the 6 mounts, 1500 MPa E) over ~1,800 upper-half layouts in five passes (`rods14_search.py`; lower half unchanged). Gates vs v13: rods 12 off the swept disks, plank (38 mm) paths, top hits, edge spans, **rod bow under a 0.5 J edge hit stays off the disks**, and **a 90° plank edge between the drum rings stops short of the prop tips** (tip r = 112 − (gap/2 − 2.1), prop tip r 98.55). The wedge check holds the shoulder ring at **z 60** (v13 sat on that limit; z 68 lets the edge reach r 96.7). Picked: **4 corner spokes to a square hub (r 40, z 85) with a crown ring at 40% of each spoke (z 75), columns at the 4 corners + nose only** (tail skid and both shoulder side/tail flats become 200 through-rods; side skid tees keep a small node for glove side + strut).
| | v13 | v14 |
|---|---|---|
| Rods + printed (E) | 137.5 g | 134.0 g |
| Top-hit energy (F) | 0.15 J | 0.29 J |
| Weakest edge span (F) | 0.529 J (glove tail low) | 0.491 J (spoke) |
| Plank (38 mm) paths to a prop (F) | 4323 | 842 |
| Bow to prop, 0.5 J edge hit (F) | 6.6 | 5.0 (tail skid flat) |
| 90° edge tip in drum (E) | r 100.7 | r 100.7 |
| Recovery climb (E) | 43.0 mm | 43.0 mm |
| Printed parts | 17 | 20 (square hub, 4 crown nodes, 2 side nodes; tail + 2 side columns gone) |
Crown z 74 scores 0.35 J top hits, 11 shorter, but a 54 mm climb (0.28 J): hold until D20. 8 spokes + an 8-rod crown ring blocks more (395 paths) at +24 g: not picked. Research: `claude/research/2026-10-22-rod-arrangement.md`. Next: Spark OK, topology run for the hub, crown nodes and side nodes, `solid_check.py`. Page: artifact "Scout Cage Rod Layout".

## Socket v2 (Spark 02:42; `node_topo2.socket_parts`, coupon `s2_socket_v2_coupon_v2` READY)
| Feature | Value | Why |
|---|---|---|
| Bore | 3.6 for the 3.0 rod: 0.3 epoxy gap a side | v1 was 3.15 (0.075 a side) and printed holes come out 0.1..0.2 under: too tight, no room for resin |
| Centring ribs | 4, 0.8 wide, on a 3.3 circle (0.15 a side), keyed 0.5 into the wall, stop 1.0 short of the lead-in | centre the rod for an even glue line; the only thing that can bind, and thin enough to shave |
| Lock rings | 2 x Ø3.9, 1.2 wide, 3.5 and 7.0 from the mouth | mechanical key: PA12 bonds poorly to epoxy by adhesion alone |
| Bleed slot | 1.0 wide to Ø3.9, bottom to mouth | trapped air and surplus resin come back out instead of pushing the rod back |
| Lead-in | 45°, Ø4.8 → 3.6 | the rod finds the hole |
| Floor | closed, 2.0 thick | rod stop; seals the epoxy; also removed sealed slivers between the generated body and an open socket bottom |
| Wall | OD 7.9: 2.0 at the lock rings and bleed | `solid_check.py` legal minimum (OD 7.6 left 1.6..1.8 and split under the 0.9 erosion). **OD 9.9 tried for v13: thick walls plus generated material made isolated cores at the mouths (share 0.982); every socket stays 7.9** |

Each socket 0.57 g. The rod's clearance path out of a mouth now cuts only free space, never another socket (it reamed the nose column's split sleeves and the coupon's in-line sockets before). Coupon v2 (7.4 g, 66 x 30 x 12, solid check PASS): bores 3.5 / 3.6 / 3.7 (1 / 2 / 3 notches), each standing and lying (lying mouths at the front edge). Test: a 3.0 rod slides in by hand without wobble; epoxy a rod into the standing 3.6 (the plate seals its bottom as the floor does), pull after 24 h. **v1 coupon had the reaming bug and was removed.**

## Topology run v13 (20 Oct, Spark: thicker structure around the prop guard, rods stay 3.0; `node_topo3.py`, `run_v13.py`)
Same protocol, loads and keep-outs as v12 (below), plus:
- **Drawn skeleton** (decision 33's fix, applied as DEFAULT to the parts rerun here): socket-size cores (r 3.95) at the node points, a root from every end-socket floor to its nearest core, spines through columns. Nose column: spines at y ±14 up to a **crossbar at z 20.3** (under the shoulder rod, over the lens window top 15.8) and a short spine to the shoulder socket; no cores on the shoulder rod line (the skeleton draft put cores at (0, ±14, 26.8), which would have blocked the shoulder ring rod) and no socket roots (the spoke socket floor sits on the shoulder bore); the 4 camera pads get socket-size roots into the spines.
- **Thicker members**: drum parts and hub with a 3.0 minimum member (side and nose column 4.2) and higher gen targets (corner columns 1.8, side 1.3, nose 1.5, tail 0.8, hub 4.0 cm³).
- **Sleeves**: no generated material within 2.5 of a socket wall away from its node (end sockets from 4 mm above the floor, through sockets beyond ±4.5 of the node). Generated lumps hugging a 2.0 wall hung on thinner members and failed the neck test's share part (0.97..0.99); with the sleeves the material meets a socket only at its node.
- Hub ring r 36 x 6 x 6 with rounded corners: at r 40 the bores cut its core at every socket and the miter notches split it.
- Nose legs and sled tails carry the flat feet and a landing case.

| Part (n) | gen cm³ | each finished, g M | index | v12 g / index |
|---|---|---|---|---|
| Corner column nose (2) | 1.8 | 8.07 | 0.22 | 5.61 / 0.50 |
| Corner column tail (2) | 1.8 | 7.79 | 0.29 | 5.78 / 0.37 |
| Side column (2) | 1.3 | 6.06 | 0.13 | 4.69 / 0.43 |
| Nose column (1) | 1.5 | 10.07 | 0.11 | 5.02 / 0.43 |
| Tail column (1) | 0.8 | 4.33 | 0.16 | 3.30 / 0.59 |
| Hub (1) | 4.0 | 17.87 | 0.35 | 11.10 / 0.64 |
| Nose leg + foot (2) | 1.2 | 5.24 | 0.44 | 4.08 / 0.96 |
| Sled tail + foot (2) | 0.45 | 3.03 | 0.19 | 2.78 / 0.16 |
| Tail knee (2) | 0.4 | 1.83 | 0.20 | 2.06 / 0.13 |
| Rail sleeve (2) | 0.3 | 1.85 | 0.20 | 1.86 / 0.14 |

All 17 parts after finish v2: one body (saved STL), watertight, **`solid_check.py` current version (neck test included) PASS**, index ≤ 0.44. Tail knee and rail sleeve reran with the skeleton and sleeves; **the rail sleeve's glove socket starts 7 from the node point (TIP + 3)** so a socket-size core fits 5 off the rail axis between the rail tube and the glove floor (with TIP 4 the floor sat on the rail bore and no core could pass the neck test); that glove side rod is 3 shorter. Assembled (`assemble14.py` on the full meshes): collar face 0.70, props 12.05, box arm sweep 0.55, camera lens 0.32, opening + rim at 5° yaw 3.95, swap head lane 0.64.

## Finish pass v2: smooth, one body, no sharp edges (Spark 20 Oct 22:16, 23:41 "much smoother"; `smooth14.py`, `sharp14.py`)
Per part, on a signed distance field at 0.1 mm (hub 0.12, computed in 25 tiles with seams halfway between its sockets and assembled into one field before one marching cubes; whole-part 0.15 ran out of memory, 0.2 was too coarse, separate tile meshes creased at the seams):
1. **Closing r 2.0** (a fillet in every inside corner; tail column 1.0, the radius that passes the neck test there), then **opening r 0.9** (every outside edge rounded; the largest round the 2.0 walls allow). Distances half-voxel corrected.
2. **Gaussian low-pass 0.8** on the field: removes the generator's stepped contour marks. Then a **0.4 re-opening** (the blur feathers tapering edges into 0.1 fins) and a 0.3 blur.
3. **Exact sockets and foot pads blended back** with a smooth union (k 0.6), their own ends rounded 0.9 and lightly blurred (no voxel bands): the blur alone thinned the 2.0 socket walls below the solid check.
4. **Rounded intersection, 0.5 off every non-function keep-out** (funnel faces, yawed opening, props, arm sweep, head lane, receiver, plate, FC) instead of a slice.
5. Sealed bubbles filled and crumbs dropped (whole parts only), marching cubes, **no mesh fairing** (Taubin folded the surface in tight spots).
6. Function surfaces cut back exactly: socket bores (ribs, lock rings, bleed, 45° lead-in), camera pad M2 holes with a 45° lead-in at the pad face, rod paths through free space, the camera board / lens window, the ground plane (each foot pad sits 0.3 below the ground in the field, so the cut leaves one flat face meeting the round at ~40°).

**Edge audit:** every edge over 60° lies on a function surface (bore, pad hole, foot plane), except 4 spots of ~0.1 mm on the hub where its tiles meet and 1 on the nose column. Mass −0.3..+0.3 g a part; indices unchanged or lower except the nose leg 0.40 → 0.44. Print files stay full resolution (0.3..1.5 M faces); the viewer uses 55 k / 100 k (hub) face copies, one per part type, instanced.

## Flat feet (Spark 20 Oct 20:23; DEFAULT shape)
4 flat pads, drawn into the nose legs and sled tails, rounded corners R 4, 5.0 thick (so the pad's own core is continuous for the neck test), **bottoms all at z -52** (1.0 under the v12 sled nodes, 5 under the rail axis). Plan (C): nose x 29.5..41 (1.0 clear of the box arm sweep x ≥ 42), tail x -59..-45, y ±27..±39 (1.0 clear of the swap head lane y -26..14). All inside the 168 opening at 5° yaw (outer corner 62 of 83); they never touch the funnel. Flat face per foot 124 (nose) / 154 (tail) mm² (M). A ground plane at z -52 is a keep-out for every part, so nothing generated reaches below the feet; the rail sleeves bottom at -51.24 (0.76 up), the cartridge at -44.5 (7.5 up). Stance 100 x 78; tip-over at the outer pad edges 35° side, 36° nose, 47° tail (E, CG (0.57, 0, 2.79), 54.8 above the ground). Landing case in the feet parts: 27 N up on one foot (E: 6 g on ~0.45 kg) and 13.5 up + 13.5 sideways in ±x ±y.

## Topology run v12 (20 Oct, noxopt; `node_topo2.py`, `run_v12.py`)
Five steps of `nox-generative.md` per part at h 1.25 (hub 2.0); PA12-CF E 4500 / Xt 90 / Zt 36 MPa (E, added in the script). Drawn set = the sockets (+ hub ring 4 x 3 trimmed out of the bores, + nose column camera pads). Cuts = bores, rod paths through free space, and **every keep-out** (none in the v11 run): funnel faces 1.0 clear above z -0.8, the 168 opening at ±5° yaw below it and in a 6 mm rim band, props swept disks + 6, box arm sweep x 42.. below z -9.5, swap head body, receiver / cartridge envelope, plate, FC, camera board and lens window; mirrored about y for the parts used both sides (the head lane is off-centre). Loads: each socket not held 80 N (E) in ±x ±y ±z, class crash; columns hang on both rings and add a **drum shear** case (shoulder sockets 40 N sideways, skid ring held; E: the rest of a hit goes round the rings and down the spoke); sled parts hang on the rail. Columns and nose legs print on their side (layers along the bending). Pass = one body, exact function check 0 / 0 / 0, thin 0, index ≤ 1.0, **`solid_check.py` PASS**; a failing part steps to 1.5x / 2.25x material.

| Part (n) | gen cm³ | each, g M | of it socket | index | steps |
|---|---|---|---|---|---|
| Corner column nose (2) | 1.8 | 5.61 | 3.42 | 0.50 | pass |
| Corner column tail (2) | 1.8 | 5.78 | 3.42 | 0.37 | pass |
| Side column (2) | 1.3 | 4.69 | 3.15 | 0.43 | pass |
| Nose column (1) | 1.5 | 5.02 | 3.19 | 0.43 | 1.0: a camera pad unattached ✗ → 1.5 |
| Tail column (1) | 0.8 | 3.30 | 2.31 | 0.59 | pass |
| Hub (1) | 4.0 | 11.10 | 7.27 | 0.64 | pass (02:30 solid check, turned 0.12°) |
| Nose leg (2) | 2.4 | 4.08 | 1.14 | **0.96** | pass |
| Tail knee (2) | 0.8 | 2.06 | 1.14 | 0.13 | pass |
| Sled tail (2) | 0.9 | 2.78 | 1.69 | 0.16 | pass |
| Rail sleeve (2) | 0.4 | 1.86 | 1.40 | 0.14 | pass |

17 parts 73.1 g M, about half of it socket. Measured on the assembled meshes (`assemble12.py`): collar face 0.39 (side / corner column socket edge at the skid ring), props 12.05 (TPU sleeve), box arm sweep 0.36 (nose leg), camera lens 0.33 (skid ring rod passing under it, lens E), opening + rim at 5° yaw 3.96, swap head lane 0.40 (nose leg). Wrinkle p95 / p99 0.18..0.28 / 0.29..0.42. **Superseded by v13 except the tail knees and rail sleeves.**

**Not the print gate yet:** layouts at h 1.25 (hub 2.0), 80 N per socket until D21 measures the rod, no 25 g inertial case; before printing: the finer pass (h 1.0), the coupon result (bore size), `print_orient.py` per part.

## Applied from the research (v11, report `claude/research/2026-10-19-cage-collision-designs.md`)
| Change | State in v12 / v13 |
|---|---|
| Printed parts in PA12-CF (PC if the spool cannot be kept dry) | kept; buy one spool, hardened nozzle |
| TPU sleeves on both drum rings | kept: 4 x 3 tubing (OD 4), stopping 11 off every socket |
| Line-contact skids: 8 UHMW-taped strips on the skid ring flats | kept, **76 long at ±50** (v11's 90 at ±55 ran into the corner sockets); 2.5 proud |
| Soft mounts cage ↔ core: TPU 95A grommets at the 2 side hubs and the receiver's 4 rail sleeves (~1.5 travel) | kept; rail sleeves at **x 15 / -25** (bare rod between sockets) |
| Clip-in tees as the sacrificial joint | **dropped**: tees are columns now and every joint is epoxied in socket v2; the rod is the fuse |
| Turtle mode as fallback (D22) | kept |

Main warning stays: the 0.5 J per 100 span rod number assumes ~130 GPa pultrusion; hobby rod measures 85..113 GPa. **D21 bend test before any hit number is trusted.**

## Electronics (placement sweep, 24 combinations, objective = wire length + CG offset + height)
| Item | Placement | Tag |
|---|---|---|
| FC MicoAir743 AIO | Plate top on the frame's FC holes, diamond, 4 standoffs: z 26..34, 14.0 off the swept disks in plan | C |
| Radxa Zero 3W | Under the plate on the axis, flat, component side up, SD to the tail, M2.5 standoffs on the receiver v4 ceiling: PCB bottom z 10.4, USB-C tops 15.2, ports to +y | C |
| UBEC 5A + RP3 receiver | Right wing of receiver v4: y -23..-41, x ±16, z 8..18 (sizes D17) | E |
| Camera DECXIN 2633V2 | Board back face x 99, centre (99, 0, 41), lens front ~x 114 (D18); **on the nose column's 4 pads (M2) at (103, ±14, 27 / 55)**; looks out between the rings, nose skid sleeves split to y ±14 | C / E |
| Flow MTF-02P | x 63..85, z 2..10, lens (79, -2, 2) down, on receiver v4's nose | C |
| Battery, cartridge, pins | Unchanged: cartridge v3 flat bottom z -44.5, pins at the receiver sides z -3, x -21..-37 | shared |
| Wiring | Pack leads up the receiver sides, through the plate's tail XT60 cut-out to the FC; camera USB-C down the nose column to the Radxa's +y ports | E, DEFAULT |

Hard keep-outs (C): 12 off the swept disks for rods (printed parts 6); plate; cartridge + receiver ceiling (z to 7.0); box arm sweep x 42..84 below z -9.5; swap head body y -26..14, x -47..39 below z -0.8; collar faces 1.0 clear above z -0.8; 168 opening at ±5° yaw below it and in the 6 mm rim band; **ground plane z -52 (v13, feet)**.

## Catch: sliding, not rolling (shared catch interface, dimensions.md G)
Same 4 face planes, same seat height; 8 line contacts (the strips) instead of wheels, nothing on the drone moves. UHMW tape on the strips, μ ~0.15 on PETG (E), funnel bare. Gates: D14 strip coupon; `s2_collar_settle.py` rerun with fixed line contacts at the measured μ and the 1.5 mm mount travel. The glove rods and the feet never touch the funnel.

## Self-righting (convex hull, CG (0.57, 0, 2.79) E)
Upside down on the hub flat (CG 82, 36 mm, 0.16 N·m); the CG climbs 34 mm over a side or 38 over the nose, then downhill to the sled. Recovery = one forward blip on two motors (E 0.38 N·m, ~0.5 J vs 0.15..0.17 J of climb); no reverse spin (gate D20; turtle mode fallback D22). Other rests: sides 45° 12.6 mm, nose 45° 11.3, tail 45° 8.3; upright 32 mm (v12 hull; the feet lower the upright rest 5: rerun `s21_cage6.py` with the v13 meshes and masses).

## Hits (3.0 rod: 1500 MPa, 130 GPa, E; **D21 replaces these**)
0.5 J at a 100 span, 1.0 at 200; ring flats held at their columns (two 100 halves), 6.4 bow inside the 13.5 to the prop tips; drum window 27 x 100. A swung plank breaks a rod; the lattice keeps the rest whole. Frame model to run: 0.5 J mid-span on a ring half, a spoke, a glove rod; 15 N on two strips; 6 g landing on the feet; 2 J on the hub.

## Problems
1. **Skid-ring sockets are 0.39 off the funnel face** (axis 4.24 from the 45° face; OD 7.9 socket's lower outer edge). The strips touch first; a tilted seat could rub. Fix if wanted: strips 3.0 proud instead of 2.5 (seat 0.7 higher, back to ~1.0): **a shared catch number, Spark decides.** DEFAULT: keep 2.5.
2. **Cage mass ~152 g E (v12 ~125)**: +28 g from the thicker drum, hub and feet. Check all-up weight and hover throttle against the motor test (D20).
3. **Camera lens 0.32 over the skid ring rod** (lens size E, D18). Measure before printing the nose column.
4. **Hub vs `solid_check.py`**: checked turned 0.12° about z (the checker's z-rays run along facet edges on the x = y bores). The part is one body. Checker owner: consider jittering the grid.
5. **PA12 and epoxy**: lock rings key it; sand the rod ends; the coupon pull test decides the bore size before any part prints.
6. Not yet done for print: finer pass h 1.0, 25 g case, `print_orient.py` per part, D21 load.
7. **Neck test: closed.** All 17 parts pass with the skeleton (DEFAULT, decision 33's best option; Spark can still overrule).
8. **The box under the feet**: the feet are the drone's lowest point, z -52 (1.0 under the v12 sled nodes). Outside the swap head lane nothing in the box may rise above z -52 under a seated drone. Box chat check.

## Retired by this layout
S2 top case v1 / v2, crown + 6 spokes, cage C3 / v3.2 / v4 node set, cages v5..v11 and the v11 node set (27 nodes, socket v1), receiver v3.3 / v3.4 (interfaces carry into v4), camera mount v2, underside v2; every v12 printed part (v13).

## Receiver v4 (to design, own chat)
Drawn-plate PETG part on the 3 plate bolts: cartridge v3 interface, ceiling plate z 5..7 with the Radxa M2.5 standoffs (57.85 x 22.85), right wing for UBEC + RX, flow land at the nose (x 63..85, z 2..10), **4 outrigger sleeves on the sled rails at (15, ±32, -47) and (-25, ±32, -47), TPU 95A grommet around the 3.0 rod, 10 long**; no feet, no skirt sockets. Keep out of the head body, the arm sweep, the nose legs and their feet (x 29.5..41.4 at y ±27..39); nothing below z -52.

## To measure / buy (adds to dimensions.md D)
14. Skid strip coupon (μ ≤ 0.2). 17. UBEC + RP3 outline and mass. **18. Camera lens front to PCB back and lens OD (lens sits 0.32 over the skid ring).** 19. FC standoffs on hand. 20. Static thrust of one motor (righting blip). **21. Rod bend test (sets every hit number and the 80 N).** 22. ESC firmware on the AIO. **23. Socket coupon v2: which bore slides in by hand, and the 24 h pull.** Buy: one spool PA12-CF (or PC), 1 m of 4 x 3 PU tubing, UHMW tape; TPU 95A on hand.
