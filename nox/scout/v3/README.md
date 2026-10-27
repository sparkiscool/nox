# Scout v3: scope (started 22 Oct 2026)

Scout v3 replaces Scout v2 (retired to `archive/scout-v2/`). Carried over: the numbers in `nox/dimensions.md` and the parts list in `nox/current-parts.md`. Aircraft hardware and bring-up: `nox/scout/scout.md`.

## What v3 is (filled 23 Oct from the record; Spark edits or reverts)
Lines tagged with a Spark date are his decisions; **DEFAULT** lines are carried over until he says otherwise. Numbers: `nox/dimensions.md` A, C, E, G, J.

### Job
- Close-up inspection where the Overseer's overhead view can't see (under tables, behind racks), live video to the operator through the box (Spark, `scout.md` What it is for).
- Flies in the fleet, can be taken over by hand, no onboard autonomy. Operator gives rough directional commands; the Overseer / box can refuse a move. ELRS stays manual override and kill (Spark).
- AprilTag stickers on the airframe, read from outside (Spark).
- Lives in the box: lands in the shared collar funnel, gets its pack swapped by the swap wheel, takes off again (Spark 12 / 14 Oct, `nox/box/box.md`).

### Keep (decided, the box is built around them)
- Airframe and electronics as built: cut-down Bee35 carbon plate, EMAX ECO II 2004, Gemfan 3520, MicoAir743 AIO, XR2, MTF-02P, Radxa Zero 3W, DECXIN camera (`scout.md` As built, `dimensions.md` A).
- **Props on top** (tractor, Spark 14 Oct); prop plane in `dimensions.md` C.
- **XT60, not contact pads** (Spark 23 Oct: the touch contacts won't work). Battery under the drone in a cartridge on the bottom latch stays (DEFAULT, Spark 14 Oct); the box swap now has to mate and unmate the XT60 along the drone axis (box chat; alternative: hand swap).
- **Cartridge long side along the box axle** (Spark 16 Oct 17:34).
- **Shared collar funnel catch**: electromagnet on a steel belly plate, 2 pogo keep-alive pads (Spark 12 / 14 Oct; `dimensions.md` G).
- **Full prop cage**, every printed cage part PETG, every rod 3.0 carbon epoxied (Spark 17 Oct 10:39 / 21:45; 19 Oct 19:12: "v2 worse than v1, keep only the bottom battery + funnel fit, full cage back").
- PX4 version and switches matched to the Overseer (mode ch 5, kill ch 8); no resoldering without asking (`scout.md` Flight setup brief).

### Priorities (DEFAULT; v2 was retired for being way too flimsy, Spark 23 Oct)
1. It works: flies, swaps its pack in the box, sends video, props enclosed.
2. Stiff and robust: nothing flexes in the hand or on a landing; parts that held up on v1 beat lighter ones.
3. One crash: pack, FC, camera, Radxa and motors come out safe; cage and printed parts may bend or break.
4. No weight target (Spark 23 Oct); all-up now in `dimensions.md` C.

### Done when (DEFAULT)
- Flies perfectly in Position mode (Spark 23 Oct, done); kill and failsafe proven props off; parameter file saved.
- Lands in the funnel, the box swaps its pack and it flies out, with the drone powered through the swap.
- Live video from the Radxa reaches the box (B7).

### Cage (Spark 27 Oct)
The latest layout (PR 94) with a geodesic shell cage (the Overseer cage v3 idea, CLAUDE.md "Cages are geodesic shells"): **bottom ring exactly the Overseer's** (same 8 nodes and tubes, same datum: catch centre, 3 under the cartridge flat bottom) so one funnel takes both drones; **one camera** at the nose outside the cage, nothing in its view; **the Bee35 carbon base stays**. Layout v3 / cage v1: `layout_v3.py`, `runs/scout_layout/v6/summary.md`, numbers `dimensions.md` System Scout v3. Supersedes PR 94's ground-structure cage.

### Electronics mount (Spark 27 Oct)
Confirmed list for the mount: MicoAir743v2-AIO-35A (FC + ESC) on 4 grommets, RadioMaster XR2, MicoAir MTF-02P, Radxa Zero 3W (+ microSD), 5 V 5 A UBEC, DECXIN 2633V2 camera, and their wiring (SH1.0 4-pin pigtail, 90° USB-C). **No lights.** **Battery side left as it is** (pack, XT60E1-M, 220 µF cap, keep-alive pads, 10 kΩ, ideal diode): not this mount's work, it stays with the battery latch / box catch.

### v3 parts to draw (order DEFAULT)
1. Bottom layout: pack, receiver with latch pockets, contacts, belly plate, pogo pads, underside stack (FC, Radxa, flow) in one sweep (v2 drew it in `archive/scout-v2/s2-receiver.md`; history only).
2. Cage: layout v3 / cage v1 geodesic (27 Oct); next the frame model on the shell, then shell nodes, spoke clamps, the camera boom + guard.
3. Camera mount on the nose, outside the cage keep-outs.
4. Cartridge (Scout size) matching the bay modules (`nox/box/box-case.md`).

### Open (DEFAULT in brackets, work goes on)
- How the box mates the XT60 [straight push on the drone axis; box chat sizes the push, the hold and the plug's float].
- PX4 multi-vehicle ids: none set yet (Spark 23 Oct) [Scout `MAV_SYS_ID` 2, Overseer 1 at the next bench session; `claude/research/2026-10-11-multi-drone-coordination.md`].
