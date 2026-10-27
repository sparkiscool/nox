# Overseer: scope (development started 22 Oct 2026)

The Overseer (2.0 kg S500, lidar). Hardware: `nox/dimensions.md` section O. Tune backup: `claude/overseer_params_2026-09-26.params` (never delete). It lands in the box collar funnel (`nox/box/box-funnel.md`).

## What the Overseer is (filled 23 Oct from the record; Spark edits or reverts)
Lines tagged with a Spark date are his decisions; **DEFAULT** lines are carried over until he says otherwise. Numbers: `nox/dimensions.md` O, G, M, S.

### Job
- The fleet's overhead view: maps the facility with the Unitree L2 and watches the Scout (Spark, `nox/scout/scout.md` What it is for).
- Lidar cloud coloured by the front camera as the drone yaws; no camera ring (Spark 23 Oct). No down camera (Spark 27 Oct: "we don't need it").
- Watches the Scout and can refuse its moves: refusal lives in the box (traffic manager + virtual obstacle sectors PX4 enforces on the Scout), not a drone-to-drone link (DEFAULT, E, `claude/research/2026-10-11-multi-drone-coordination.md`).
- Box = map origin: both drones boot squared in the funnel, compass off indoors (DEFAULT, same research).
- Lives in the box: lands in the same collar funnel as the Scout (Spark 12 Oct 22:37), pack swapped by the swap wheel; 3 of the 7 bays are Overseer bays (Spark 15 Oct; `nox/box/box.md`).
- The scan stick carries the same L2, camera, Pi 5 and pack on a pole to scan on foot the way the drone will (Spark 23 Oct; `scan_stick/scan_stick.md`): the mapping pipeline is proven there before it flies.

### Keep (on hand, decided)
- S500 centre plates (2015 edition, Spark 23 Oct, CHECK), SunnySky X2216 II, 1045 props, MicoAir743-Lite, Matek FCHUB-12S, 4S 6000 packs (3), MTF-02P, Unitree L2, Pi 5 companion (`dimensions.md` O, `nox/parts-list.md`).
- **L2 at the nose, axis 15° below forward** (Spark 24 Oct), outside the cage with nothing in its field, guarded only from behind (Spark 26 Oct; `dimensions.md` O Lidar, Cage v3).
- 2 x DECXIN 2633V2 on either side of the L2, pointed so their field is maximized (Spark 27 Oct): portrait, 50° out, 15° down, 3.45 sr with no overlap (`dimensions.md` O Camera); rigid to the L2 for the camera-lidar calibration. Pointing in `dimensions.md` O Camera.
- PX4 1.18.0rc, airframe 4001, DShot, CRSF; flies perfectly in Position mode (Spark 23 Oct); tune backup `claude/overseer_params_2026-09-26.params`, never delete. Switches matched to the Scout (mode ch 5, kill ch 8).
- Printed mounts generated with the `nox/topo` engine, not `noxopt` (Spark 23 Oct, lidar mount row).

### Priorities (DEFAULT)
1. It works: flies on its tune, maps, lands in the funnel, swaps its pack.
2. Sensors rigid: L2 and both cameras on one stiff mount (the calibration depends on it).
3. Nothing in the L2's field of view.
4. Smaller: printed arms pull the motors in (Spark 23 Oct). Weight is not a concern for now (Spark 23 Oct).

### Done when (DEFAULT)
- Flies perfectly in Position mode (Spark 23 Oct, done on the stock arms); same again on the printed arms.
- A coloured lidar map of one room recorded in flight, matching a scan-stick map of the same room.
- Lands in the funnel and the box swaps its pack.

### Parts to draw (order DEFAULT)
Cage first, now a geodesic shell formed to the drone (Spark 26 Oct: the v2 layout kept, its cage replaced by triangles, compact like a disk, larger cells, 6 mm tubes, battery close to the body, a new funnel follows the cage): `layout_v5.py`, `dimensions.md` O Cage v3 (the L2 and front camera outside the cage, Spark 26 Oct), method `docs/generative.md` rule 0 Geodesic cage. v0..v4 layouts kept.
0. **Cage**: frame model on the shell (crash 35 g in 14 directions through the node that touches first, floor landing on the TPU feet, handling), then the nodes: 39 shell nodes, 8 opening ring nodes, the spoke sockets at the arm tips, the L2 apex ties, the L2 guard ring (8 nodes, 16 struts), the belly ties.
1. **Printed arms**, front and rear pairs differ: tall root between the plates (O Arms and hub), plate interface from the S500 STEP (`s500/`, M S500 row), motors and spoke and strut sockets per the cage.
2. Pi deck between the plates (4 posts over the FC stack).
3. Belly sled: cartridge hanger centred on the catch, flow pod beside it (no down camera, Spark 27 Oct), 4 TPU feet; Ø9 driver holes at 3 rear arm bottom screws, 1 more reached with the flow board unplugged (access, Cage v3); pogo pads with the new funnel.
4. Overseer cartridge + latch block for the bays; its bottom face seat height must match the Scout's (swap wheel R 100); it grows the box along the axle unless its latch block sits off-centre (`nox/box/box-arm.md` Problem 15).

### Open for Spark (DEFAULT in brackets, work goes on)
- Centre plate arm bolt pattern and motor centres, measured (P) [generic S500].
- ESC and RC receiver models [not recorded].
- Tip gap for the printed arms [25 mm, E].
