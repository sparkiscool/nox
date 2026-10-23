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
- S500 airframe, SunnySky X2216 II, 1045 props, MicoAir743-Lite, Matek FCHUB-12S, 4S 6000 packs (3), MTF-02P, Unitree L2, Pi 5 companion (`dimensions.md` O, `nox/parts-list.md`).
- **L2 top-mounted**, above the collar faces (log 23 Oct; `dimensions.md` O, G).
- 2 x DECXIN 2633V2: front portrait tilted up [tilt DEFAULT in `dimensions.md` O], rigid to the L2 for the camera-lidar calibration; underside straight down (Spark 23 Oct).
- PX4 1.18.0rc, airframe 4001, DShot, CRSF; tune backup `claude/overseer_params_2026-09-26.params`, never delete. Switches matched to the Scout (mode ch 5, kill ch 8).
- Printed mounts generated with the `nox/topo` engine, not `noxopt` (Spark 23 Oct, lidar mount row).

### Priorities (DEFAULT)
1. It works: flies a stable hover on its tune, maps, lands in the funnel, swaps its pack.
2. Sensors rigid: L2 and front camera on one stiff mount (the calibration depends on it).
3. Nothing in the L2's field of view.

### Done when (DEFAULT)
- Hovers and holds position indoors with the L2 + flow, compass off.
- A coloured lidar map of one room recorded in flight, matching a scan-stick map of the same room.
- Lands in the funnel and the box swaps its pack.

### Parts to draw (order DEFAULT)
1. Lidar + front camera mount (DESIGN row in `nox/current-parts.md`), reusing the scan stick head's L2 pads and camera block where they fit.
2. Pi 5 + UBEC / 12 V buck tray.
3. Underside camera mount.
4. Belly frame for the catch: 8 PETG wheels at the Scout's points, steel belly plate, pogo pads (`dimensions.md` G Catch interface).
5. Overseer cartridge + latch block for the bays; it grows the box along the axle unless its latch block sits off-centre (`nox/box/box-arm.md` Problem 15).

### Open for Spark (DEFAULT in brackets, work goes on)
- S500 variant (500 vs Holybro 480), ESC and RC receiver models, pack mass [`dimensions.md` O, M Q7].
- Front camera tilt [30° up, `dimensions.md` O].
- Has it flown on the 26 Sep tune, and in which modes [not recorded].
