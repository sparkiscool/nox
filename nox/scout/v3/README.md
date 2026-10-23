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
- **Battery under the drone** in a cartridge, pushed straight up the drone axis, held by the bottom latch (side bolts the box fingers squeeze out), contact pads not the XT60 (Spark 8 / 12 / 14 Oct; `dimensions.md` C, E, J).
- **Cartridge long side along the box axle** (Spark 16 Oct 17:34).
- **Shared collar funnel catch**: electromagnet on a steel belly plate, 2 pogo keep-alive pads (Spark 12 / 14 Oct; `dimensions.md` G).
- **Full prop cage**, every printed cage part PETG, every rod 3.0 carbon epoxied (Spark 17 Oct 10:39 / 21:45; 19 Oct 19:12: "v2 worse than v1, keep only the bottom battery + funnel fit, full cage back").
- PX4 version and switches matched to the Overseer (mode ch 5, kill ch 8); no resoldering without asking (`scout.md` Flight setup brief).

### Priorities (DEFAULT, from v2)
1. It works: flies, swaps its pack in the box, sends video, props enclosed.
2. Light: all-up target in `dimensions.md` C (E); a part under budget gives its grams back to the total.
3. One crash: pack, FC, camera, Radxa and motors come out safe; cage and printed parts may bend or break.

### Done when (DEFAULT)
- Holds position over a textured floor to the `scout.md` Flight setup brief line; kill and failsafe proven props off; parameter file saved.
- Lands in the funnel, the box swaps its pack and it flies out, with the drone powered through the swap.
- Live video from the Radxa reaches the box (B7).

### v3 parts to draw (order DEFAULT)
1. Bottom layout: pack, receiver with latch pockets, contacts, belly plate, pogo pads, underside stack (FC, Radxa, flow) in one sweep (v2 drew it in `archive/scout-v2/s2-receiver.md`; history only).
2. Cage around the raised prop plane that rolls into the funnel (catch interface `dimensions.md` G).
3. Camera mount on the nose, outside the cage keep-outs.
4. Cartridge (Scout size) matching the bay modules (`nox/box/box-case.md`).

### Open for Spark (DEFAULT in brackets, work goes on)
- Why v2 was retired, i.e. what v3 must do differently [v2.1's verdict: full cage, bottom battery, funnel fit].
- All-up target [`dimensions.md` C].
- PX4 multi-vehicle ids: Scout `MAV_SYS_ID` 2, Overseer 1 [E, `claude/research/2026-10-11-multi-drone-coordination.md`].
