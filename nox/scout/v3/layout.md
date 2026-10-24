# Scout v3 layout (24 Oct 2026): the whole drone before the topology runs

**v2 = the system pass (Spark 24 Oct: "this design seems very flimsy ... structurally secure while light as possible ... the cage should be touching first").** `layout_v2.py`, `runs/scout_layout/v2/` (`checks.txt` has every case), numbers in `dimensions.md` "System Scout v3". The v1 stick cage below is superseded: the v2 cage comes from a ground structure of 3.0 rods trimmed by greedy mass removal on a frame model over 28 system cases, with PR 44's bottom (receiver interface, cartridge v4, pogo / KW11, Radxa, flow) as DEFAULT. Pass 1: 116 rods, 67 printed nodes, ~268 g, worst index 0.98, deflections ≤ 1.96 mm (F / E). Open: the receiver must sit ≥ 7.0 inside the funnel planes for the cage to touch first at the tail-bottom (control session); fewer joints; CG (−1.8, −1.8).

## Keep-alive pads (24 Oct, box catch DEFAULT, G (8))
The catch dropped the belly post: spring probes in the two plain ±x faces touch pads on the lower ring (B+ nose side y +6..+28, B− tail side y −28..−6, face 2.0 off the face plane). In G (8)'s band (within 9.0 of each ±x face, from the face bottom to the pad top) only the pad, its carrier and rods may sit, so each lower x-flat keeps two nodes: the pad carrier at y ±17 and one at y ∓50 outside the band. Result: 141 rods, ~319 g (E), every case ≤ 1.00 (F); band clear; System line cage first 4.5, fit 0, CG 1.3 / 1.9 PASS, edge probes FAIL.

## Belly post (24 Oct, box catch PR 76 keeps it; `pad_search.py`)
The catch chat kept the belly post (KW11 + 2 pogo pads; a latch contact cannot stay made through the pull). Searched every spot (both sides, x −40..50, |y| 26..45, z −45..−20) with its post model (pins free 8, r ≥ 101, cones): 9,772 fail the funnel planes, 2,014 the r ≥ 101, 7,973 the cones vs the latch receiver / cartridge; the one survivor puts the keel 4.2 inside the pin cones (C). No belly spot works with a cage that touches first: asked the catch chat for its alternative, hook latch v3 with leaf contacts, and the pad seat on the latched ±y wheel nodes.

## Refit to the latch standard (24 Oct, WIP; branch part/battery-latch @ b59b2dc, meshes in `runs/scout_layout/v2/latch/`)
The control session's receiver v1 + cartridge v5 (XT60 at the nose, 4 x M3 bosses at z 9.6) replace PR 44's; the Scout's own mount takes z 9.6..18.5 round the FC, with legs outside the receiver plates for the Radxa (-y), the flow seat and pogo carrier (+y) and the suspension joints. Findings (C / F): (1) PR 44's pads (±12, 38.5) break the belly post's landing cones against the new receiver feet (y 24.3); a grid 39.5..42 x −36..−38 leaves one band, (±12, 40.6) face −37.6, funnel 1.21 / cones 0.03, with the carrier chamfer starting at the pad edge. (2) There the 4th keel node (−28, 28.5, −47.5) that the receiver needs for cage first cannot get a rod past the pin cones: cage first −5.9 vs the receiver, and the optimiser ends heavier (178 rods, ~394 g E, all cases ≤ 1.0). (3) The belly post zone is the root: proposal to the box catch, keep-alive through the latch instead (a sprung contact in two plunger faces onto metal pads on the latched wheel nodes), which frees the whole belly. Settled (hub DEFAULT, same day): no belly post; keep-alive through two latch plungers onto pads on the latched ±y wheel nodes. Refit result: 4th keel node in, keel rails along both sides mandatory, flow moved to mid-length on the +y mount, UBEC into the mount's tail (balance); 130 rods, ~294 g (E), every case ≤ 0.98 (F); System line: cage first 4.5 PASS, fit 0, CG 1.3 / 2.0 PASS, **edge probes FAIL** (side windows between the keel and the rings let a diagonal edge reach the cartridge). Next: a window / probe constraint in the optimiser, then fewer joints.

## Pass 3 (24 Oct, Spark: "only 3mm rods ... definitely use tpu ... more sections fine, v1 was very sturdy")
Research (`claude/research/2026-10-19-cage-collision-designs.md` + a new web pass: Elios TPE suspensions, EPFL Gimball and Euler springs, Mintchev fuse joints, CogniFly TPU joints, Zha tensegrity): thin-rod cages survive by compliance, the joint or mount fails first, spans ≤ 80..100, a plateau (preloaded) give halves the peak force of a linear spring. Applied: crash as an energy case (1.5 m/s DEFAULT), rods epoxied in PETG sockets, a preloaded TPU suspension between cage and core (rigid below 120 N so the box sees a stiff drone, gives above it), stroke per direction from the contact lead, more sections (50 mm ring nodes, shoulders over the prop tips, mid glove nodes). Result: 124 rods, ~290 g (E), every case ≤ 0.97 (F). Tested and dropped: TPU sockets everywhere (the wheels sank 12 mm under the 50 N swap pull); a plain linear suspension (index 1.27..1.68 at 2 m/s). Numbers: `dimensions.md` "System Scout v3".

## Concept check, pass 2 (24 Oct, after main's "Every drone design" list and `dimensions.md` L)
Script defaults are now the L cases and a coarser node set (12 nodes a ring); pass 1 (my E cases, 80 N hits) is commit 1d9279a, `runs/scout_layout/v2/`.
| Alternative | Cases | Cage | Result (F) |
|---|---|---|---|
| A (pass 1) | 80 N point hits at each ring node, 80 N crown, landing 6 g, swap, latch, 20 N squeeze | 116 rods, 67 nodes, ~268 g (E) | every case ≤ 0.99; contact-first blocked only by the receiver's tail-bottom (needs ≥ 7.0 inside the funnel planes) |
| B (L standard) | crash 35 g = 172 N through the touching-first node, 14 directions; flight / landing ≤ 0.5; handling 50 N | every feasible 3.0 rod (259) | **fails: index up to 2.25** (side and lower corners); every member doubled (NROD=2): still 1.28 |
**No 3.0-rod cage meets L's crash case at index ≤ 1.0, at any mass.** Ways out (Spark): (1) the cage is sacrificial (L "breaking order": the node goes first): crash index on cage nodes / rods up to ~2.3, frame and mounts held ≤ 1; (2) a softer crash load through compliance (TPU node mounts, E: halves the peak g); (3) a larger rod (CLAUDE.md rod rule); (4) printed cage members. Mass: **L's 500 g Scout budget is not reachable with this architecture**: bought parts ~350 g (E) + PR 44's receiver and cartridge 122 g (M) leave ~28 g for the cage. DEFAULT until Spark answers: alternative A's cases, budget "none" (Spark 23 Oct: no weight target), lightest cage that passes.

## v1 (superseded by v2 where they differ)

Spark 24 Oct: "design it to the latest updates and then show me the design as a whole before the topology runs". This is v3 part 1 (bottom layout) laid out with the rest of the drone around it, so every box and funnel keep-out is checked on one model. Rough shapes: the receiver is the design space for its topology run, the cage is a stick model for part 2. Nothing here is saved or printed.

Script `nox/scout/v3/layout_v1.py` (~3 min). Run `runs/scout_layout/v1/`: `checks.txt` (search table + every check), `result.json`, `motion.html` (artifact: land, latch, swap the pack, take off), `motion.mp4` / `.gif` / `.png`, `receiver_envelope.stl`, `cartridge_v4_envelope.stl`. Numbers: `nox/dimensions.md` V. Drone frame as `dimensions.md` (x nose, y left, z up, plate top 22); seat frame z_s = z − 0.34 (C).

## Latest updates it follows
- **XT60, no contact pads** (Spark 23 Oct). The pack keeps its own XT60, fixed in the cartridge facing up; the drone's XT60E1-M sits on a ±1.0 float carrier in the receiver (`dimensions.md` I "XT60 mate").
- **Hook latch, no magnet, no steel belly plate** (Spark 23 / 24 Oct, `nox/box/box-hook-latch.md`). The drone carries only the R 5 wheels at the catch points and keeps the latch zone clear (G (7)).
- **Props on top, full cage, every rod 3.0, PETG / TPU only** (README Keep).
- **Box at the drone:** arm v5 rails (worst case C2, Y_C 118: carriage 2.5 past the cartridge's closed end), head under the pack, cartridge flat bottom at ring R 100.

## What it is (DEFAULT unless tagged; numbers in `dimensions.md` V)
- **Pack and cartridge v4 (Scout):** pack long side nose to tail on the axis, closed end at the nose (toward the swap arm), XT60 at the lead end (tail). Cartridge v3's section (38 x 48) and latch kept; **13 longer (92, C)** for the XT60 bay: 8 lead loop + the 8.1 housing. The pack's XT60 nose sits flush with the cartridge top, so the plug never stands proud; the drone's male shroud enters a recess in the cartridge top. Tab line unchanged (37.5 from the closed end). **Pack position from a CG solve:** closed end x 37.0, tab line x −0.5, CG x 0.04 (E).
- **XT60 mate:** pack XT60 at x −48.0, mate face = cartridge top (z 3.84, z_s 3.5). Drone XT60E1-M on the float carrier: shroud into the cartridge, flange 1.5 above the cartridge top under a carrier plate that takes the insert push, rear body and cups up through the plate's tail XT60 cut-out to z 19.3 (C). Leads to the FC battery pads under the plate.
- **Receiver v4 (envelope for its topology run):** the bay interface as before (30° lead-ins, locating faces 0.3, both latch pockets), ceiling z 5.34..7.34 over a 1.5 TPU preload strip, Ø12 columns to the two side plate bolts and a Ø15 one to the nose bolt, the float carrier at the tail, a flow land on a tongue at the nose, the pogo outrigger on −y, the 4 feet. Side walls stop 1.0 short of the arm carriage; the nose end stop is a top-only tab above the carriage corner (z_s ≥ −5).
- **Pogo pads (shared, G (6)):** 2 pads 10 x 10 face down at (±10, −63) in the catch frame, face z_s −6.0; a 12 x 12 post column under each is free of the drone; the post rises 66° from the axle (inside the 45..135° opening wedge, outside the head's reach). −y and x ±10 keep them off the Overseer v1 down-camera pod (+y) and flow pod (|x| ≤ 15, y ≥ −55).
- **Electronics (placement search, 24 cases: Radxa top / +y wing / −y wing x UBEC on the Radxa / plate-top tail / +y wing / −y wing x pogo side; objective wire length + 10 x |CG y|):** FC under the plate as built; **Radxa on the plate top** (component side up, M2.5 standoffs 2.0, 8.4 under the swept props) with the **UBEC stacked over it** (9.8 under the props); XR2 at the plate-top tail corner; camera on the nose; flow MTF-02P under the nose. Best case 167 mm of wire; the next feasible (UBEC under the +y wing) 199.
- **Camera (part 3, rough mount):** board centre (95, 0, 41.2) level between the rings, lens front x 108 = 4 behind the ring line, so both ring rods sit 63° off the lens axis (outside the 70° vertical field); nose cage columns at y ±22 rise to a crown cross bar, out of the 86° horizontal field. 0 drone points in view (C, lens size E Q3).
- **Flow:** board on the receiver's nose tongue, lens at (62.3, −2.0, −0.86), z_s −1.2 (1.0 above the arm rim sweep); its 23° cone is clear of every drone part by 7.6° (C). PX4 offsets in `dimensions.md` V.
- **Feet:** 4 TPU pads R 5 at (29, ±55) and (−47, ±55), bottoms 3 under the cartridge (z −47.16); each on a 3.0 post to z_s −36 and a V of two rods into the receiver wall (outside the head |y| ≤ 47 and the arm sweep). Stance 76 x 110; tip-over 41° side, 27° nose / tail (E).
- **Cage (part 2, stick model):** rings z 33.2 / 49.2 on the clipped-square outline, 8 R 5 wheels at the catch points, columns only at the x-face wheel nodes (±112, ±100), the sides (0, ±112), the tail and the nose pair (112, ±22) (none straight up from a ±y-face wheel: G (7)); crown of 4 spokes along the axes to a hub r 20 at z 62 (the |x|, |y| ≤ 9.6 strips are never under a disk); 2 side struts from the side hubs, a tail strut from the plate top. 49 rods, 2.91 m (C).
- **Mass ~517 g (E)** (Spark: about 500), CG (0.04, −0.53, 9.7).

## Checks (C on E sizes; `checks.txt`)
| Check | Rule | Result |
|---|---|---|
| Rods to the swept props | ≥ 12 | 12.6 |
| Printed / bought parts to the swept props | ≥ 6 | 8.4 (Radxa) |
| Funnel faces, seated, everything but the wheels | ≥ 1.0 (underside ≥ 3.0) | cage rods 2.6, nodes 2.3; underside 10.7 |
| Below the bottom edge, ±5° yaw | ≤ 83 | 70.1 |
| Hook latch plungers out (G (7)) | ≥ 1.0 | 1.9 (rods) |
| Arm carriage / rail sweep (C2) | ≥ 1.0 | 1.0 (receiver nose stop and walls, designed) |
| Head under the pack (|y| ≤ 47, E) | ≥ 1.0 | 2.0 |
| Cartridge path below the receiver mouth | ≥ 1.0 | 2.0 |
| Flow cone 23°, camera field 86 x 70° | nothing in view | 0 / 0 points |
| Body overlaps (receiver, FC, cartridge, electronics, cage, plate, XT60) | 0 | none |
| Pogo post columns | free | free |
| CG on the thrust centre | x 0 | 0.04 |

## Problems (each with its fix)
1. **Cartridge 79 → 92 (shared):** the bay modules take a 13 longer cartridge at the lead end (+Y, away from the arm). Box chat: the next bay module version grows 13 at its far end; the v4i modules cannot hold a v4 cartridge. Alternative that keeps 79: the XT60 on top of the pack, which the FC (lowest z 9.5) leaves no room for.
2. **FC lowest point 9.5 is CHECK** (H): the receiver ceiling top is 7.34, 2.16 under it. Measure before the receiver run; a lower FC moves the ceiling, not the seat.
3. **Plate tail edge (x −50) and its XT60 cut-out (10 x 18 here) are E:** the drone XT60's rear passes through it. Measure the plate (D1 when it is apart).
4. **Box numbers taken as E:** seat W Z 144.5 (cartridge bottom z_s −44.5), head half width 47 under the pack, arm carriage from Y_C 118 (C2). With Y_C 175 (the demo build) the carriage is 39.5 further away, so C2 is the tight case.
5. **Tail hub:** the tail strut roots on the plate top at x −38; the plate has no bolt there. Cage part 2: a clamp on the plate or a hole (CHECK the v1 pocket holes).
6. **Overseer belly:** its v1 sled (y ±72) covers the shared pogo positions; the Overseer layout opens two windows at (±10, −63).
7. **Nose / tail tip-over 27° (E)** with the feet inside the head and arm keep-outs; same as v2's 26.6.

## Next (after Spark looks at the layout)
Topology runs with `nox/topo`, one per session (`run-part`): 1 receiver v4 (this envelope, the drawn interface, the XT60 carrier, the feet, the pogo outrigger), 2 cartridge v4 tray (92 long, XT60 seat), 3 cage parts (nodes with wheels, crown hub, struts), 4 camera mount.
