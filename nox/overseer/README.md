# Overseer: scope (development started 22 Oct 2026)

The Overseer (2.0 kg S500, lidar). Hardware: `nox/dimensions.md` section O. Tune backup: `claude/overseer_params_2026-09-26.params` (never delete). It lands in the box collar funnel (`nox/box/box-funnel.md`).

## What the Overseer is (filled 23 Oct from the record; Spark edits or reverts)
Lines tagged with a Spark date are his decisions; **DEFAULT** lines are carried over until he says otherwise. Numbers: `nox/dimensions.md` O, G, M, S.

### Job
- The fleet's overhead view: maps the facility with the Unitree L2 and watches the Scout (Spark, `nox/scout/scout.md` What it is for).
- Lidar cloud coloured by the front camera as the drone yaws; no camera ring (Spark 23 Oct). Underside camera for inspection and landing (Spark 23 Oct).
- Watches the Scout and can refuse its moves: refusal lives in the box (traffic manager + virtual obstacle sectors PX4 enforces on the Scout), not a drone-to-drone link (DEFAULT, E, `claude/research/2026-10-11-multi-drone-coordination.md`).
- Box = map origin: both drones boot squared in the funnel, compass off indoors (DEFAULT, same research).
- Lives in the box: lands in the same collar funnel as the Scout (Spark 12 Oct 22:37), pack swapped by the swap wheel; 3 of the 7 bays are Overseer bays (Spark 15 Oct; `nox/box/box.md`).
- The scan stick carries the same L2, camera, Pi 5 and pack on a pole to scan on foot the way the drone will (Spark 23 Oct; `scan_stick/scan_stick.md`): the mapping pipeline is proven there before it flies.

### Keep (on hand, decided)
- S500 centre plates (2015 edition, Spark 23 Oct, CHECK), SunnySky X2216 II, 1045 props, MicoAir743-Lite, Matek FCHUB-12S, 4S 6000 packs (3), MTF-02P, Unitree L2, Pi 5 companion (`dimensions.md` O, `nox/parts-list.md`).
- **L2 at the nose, axis 15° below forward** (Spark 24 Oct; was top-mounted), above the collar faces (`dimensions.md` O Lidar, Layout v1, G).
- 2 x DECXIN 2633V2: front portrait at the horizon behind the L2 (DEFAULT 24 Oct with the forward-down L2; alt 30° up, the 23 Oct choice), rigid to the L2 for the camera-lidar calibration; underside straight down (Spark 23 Oct). Pointing in `dimensions.md` O Camera.
- PX4 1.18.0rc, airframe 4001, DShot, CRSF; flies perfectly in Position mode (Spark 23 Oct); tune backup `claude/overseer_params_2026-09-26.params`, never delete. Switches matched to the Scout (mode ch 5, kill ch 8).
- Printed mounts generated with the `nox/topo` engine, not `noxopt` (Spark 23 Oct, lidar mount row).

### Priorities (DEFAULT)
1. It works: flies on its tune, maps, lands in the funnel, swaps its pack.
2. Sensors rigid: L2 and front camera on one stiff mount (the calibration depends on it).
3. Nothing in the L2's field of view.
4. Smaller: printed arms pull the motors in (Spark 23 Oct). Weight is not a concern for now (Spark 23 Oct).

### Done when (DEFAULT)
- Flies perfectly in Position mode (Spark 23 Oct, done on the stock arms); same again on the printed arms.
- A coloured lidar map of one room recorded in flight, matching a scan-stick map of the same room.
- Lands in the funnel and the box swaps its pack.

### Parts to draw (order DEFAULT)
Cage first (Spark 24 Oct: "lets make the cage first it must fit in the funnel and then we will make the actual designs"): `layout_v2.py`, `dimensions.md` O Cage v0; 3 mm rods wherever they do the job (Spark 24 Oct). v0 / v1 layouts kept.
0. **Cage**: frame model on the rod layout (hits, floor landing, 30 / 100 N on the latched wheels), then the nodes: 8 ring nodes, 4 corner nodes with the wheels, the L2 apex node with its lugs and the camera plate, the front hub.
1. **Printed arms**, front and rear pairs differ: tall root between the plates (O Arms and hub), plate interface from the S500 STEP (`s500/`, M S500 row), motors and spoke and strut sockets per the cage.
2. Pi deck between the plates (4 posts over the FC stack).
3. Belly sled: cartridge hanger centred on the catch, down camera pod and flow pod beside it, 2 pogo pads (G (6)), 4 TPU feet.
4. Overseer cartridge + latch block for the bays; its bottom face seat height must match the Scout's (swap wheel R 100); it grows the box along the axle unless its latch block sits off-centre (`nox/box/box-arm.md` Problem 15).

### Open for Spark (DEFAULT in brackets, work goes on)
- Centre plate arm bolt pattern and motor centres, measured (P) [generic S500].
- ESC and RC receiver models [not recorded].
- Tip gap for the printed arms [25 mm, E].
