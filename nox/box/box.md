# Box design (Spark 14 Oct 20:29: this is the box; heads became grippers 21:56; arm, bay and casing drawn 15 Oct 12:40; arm v3 16 Oct; arm v4 + bolted bay modules 16 Oct 23:45; stand fully printed 19 Oct)

**Master view (Spark 27 Oct): the Nox Ground Unit artifact https://claude.ai/artifact/RX2CexkfhB5UoQHnGTVgCC (source `nox/box/ground_unit_page.html`, the data block `D` at the top of its script), kept by the box master session (session_01EtHMZPwaEwEqxCkwFCWf7X): every merged box PR lands there as one change-log line, and it assigns one session per box system (swap arm, battery latch, structure v2, electronics; the `current-parts.md` System recheck table names them). The top of the box is no longer a funnel: the slot above the drone seat is reserved for the landing mechanism the design center session designs (PR 128 concept, `nox/box/box-landing.md`); the funnel panels, funnel hub and hook latch are superseded. Part sessions message the master (one line per merged PR), never Spark.**

The ground unit. One design, both drones. **23 Oct (Spark): XT60 back, contact pads retired.** Every swap mates and unmates the pack's XT60 along the install axis: ~60 N push in, ~50 N pull out (E) where the pads needed ~15 N; the hold (now the hook latch, `nox/box/box-hook-latch.md`), the heads, the gripper spring and the bay modules are re-sized for it (`nox/dimensions.md` I "XT60 mate"). Numbers: `nox/dimensions.md` G (funnel, hold) and I (wheel, arm, bays); latch interface J. Part docs: `nox/box/box-funnel.md` (funnel), `archive/scout-v2/s2-bottom-latch.md` (latch), **`nox/box/box-arm.md` (the arm: stub axle, hub, two bars, heads with the grippers, motors, belts), `nox/box/box-case.md` (the bolted bay modules and the printed ring frames that fill the empty positions), `nox/box/box-stand.md` (the all-printed stand: mast, keel, cross bar, ring end adapters)**; scripts `nox/box/`. Artifacts: "Nox Swap System", "Nox Swap Arm" (moving). Every earlier box design was deleted 14 Oct; never bring one back unless Spark asks fresh.

## The system (Spark 14 Oct 18:12 whiteboard, 18:18, 19:26, 21:56; drawn 15 Oct)
1. **Static funnel on top.** Printed funnel (`nox/box/box-funnel.md`). The drone lands into it under its own control and is centred; KW11 seated switch. No Z stage (a future option only).
2. **Hold: hook latch** (Spark 23 Oct: a hooked latch, no steel belly, no magnets; `nox/box/box-hook-latch.md`): 4 plungers slide out of the two seated faces just uphill of the catch wheels; the XT60 push (~60 N E) lifts the drone 1.4 onto them and they hold it with the servos off. Nothing on the drone. Pulling the dead pack down seats the drone harder.
3. **Keep-alive: 2 pogo pads** on the post under the belly (DEFAULT); the drone stays powered through the swap.
4. **Swap wheel under the funnel, vertical like a ferris wheel.** One horizontal axle (a fixed Ø50 printed stub with the 12-wire slip ring inside; the hub turns on the two 6010s). Around it **bolted bay modules (Spark 16 Oct 19:28: any one swaps out), one per 30° position (9 positions, 90° opening at the top); with printed frames in the empty positions they close into a ring that stands on the cradle and carries the funnel (Spark 16 Oct 23:39 ring = structure; casing retired 21:09; all printed 19 Oct, `nox/box/box-stand.md` v6)**. The top position is the drone. **Both cartridges sit long side along the axle (Spark 16 Oct 17:34), slide axis tangential.** **Arm v4: two 2040 bars (112 each, from the 230+ on hand) on the hub at 90°, so the heads sit one position apart, one head per bar**, each a **two-finger gripper** (rack and pinion on one standard servo standing radially, spring-closed) on a **belt-driven carriage** (NEMA17 + GT2 per head, all three motors standing beside the hub). The fingers squeeze the cartridge's two central tabs: one squeeze grips the pack and retracts its side bolts (`archive/scout-v2/s2-bottom-latch.md`). Ring radius 100 (`nox/box/box-arm.md`).
5. **Packs invert as the wheel turns:** in the head, the gripper holds by geometry (tapered pocket in each tab, 10 N any direction, E) and stays closed on power loss. Every bay is a copy of the drone receiver, so the cartridge's own latch holds it in any orientation.
6. **Charging in the bays** through each bay's XT60 (23 Oct; was the copper strips and shoes), on the same float carrier as the drone receiver's; main lead only (balance dropped 17 Oct 01:18). A bay mates and unmates only with its charger relay open. Charger: 2 bays live (one per drone size), the rest bench-charged until Spark decides on channels (after DubHacks).
7. **Batteries = bays**, counting the one flying, so one bay is always free (Spark). 7 bays: 3 Overseer + 4 Scout (Spark 15 Oct). Scout bays now at 0, −45, −90, −135°; Overseer bays reserved at 45, 135, 180°.
8. **Orientation (16 Oct):** every cartridge long side along the axle, slide axis tangential, so one head squeezes both cartridge types. B+ / B- sit on the long sides; a wheel turn rotates a pack and never mirrors it, so B+ is on the same side at the top every time.

## Swap sequence (arm v4, E, drone seated to leaving ~3.7 s with the slow XT60 insert (was 3.1); `nox/box/box-arm.md` timing)
1. Before the drone arrives: head A extends open into a charged bay, closes on the tabs (bolts leave the bay's latch pockets), retracts with the pack; head B waits open under the drone seat.
2. Drone lands, seated switch, hook latch out, pogo pads make.
3. Head B rises the last 8.6, closes: tabs go 4 mm to the stop, bolts leave the drone pockets (4 N per finger), head retracts with the dead pack and pulls the XT60 apart (~45 N E; the pogo pads hold the bus).
4. Wheel turns 90°: head A at the top.
5. Head A extends with the pack (bolts retracted), the last 12 slow (≤ 20 mm/s E): the lead-ins centre it, the XT60 mates (~45 N E), the cartridge meets the preload, 1 mm over-travel, opens: the spring pushes the bolts into the pockets, preload wedges them home; box keep-alive current under 50 mA = the XT60 is made, the drone sees pack voltage; head retracts.
6. Hook latch in, spin-up, out the funnel mouth.
7. Wheel turns 180° to put head B at the free bay, head extends, opens (bolts into the bay's pockets), retracts; charging starts.

## Drives and buy list (`nox/box/box-buy-list.md`)
- Axle: NEMA17 + GT2 20T:80T (closed 244 belt), unlimited turns through the slip ring; 2 x 6010-2RS on the fixed stub.
- Heads: 2 x (NEMA17 + GT2 open belt + kit idler), DS3218 + MG995 for the grippers, gripper springs (8 N per finger at the tab, E).
- Hook latch: 4 x MG90S (D18) + M2 / M3 screws (`nox/box/box-hook-latch.md`); electromagnets and the steel belly plate retired 23 Oct. Pogo pins, KW11. XT60E1-M panel mounts: 1 drone receiver + 1 per bay (3 now: drone + 2 live bays). Copper-clad sheet and AA spring contacts out (23 Oct).
- Funnel: `nox/box/box-funnel.md` Parts.

## Why this beats a flat ring (14 Oct 19:45)
Box footprint about the drone's, not a ring's; install is a pure push up the drone axis; one axle and bearing pair; the arm is nearly balanced (heads 90° apart, motors placed against them: ~0.08 N·m loaded, E), so the axle motor barely works. Cost: the box is ~2x the ring radius tall plus the funnel.

## Open (numbers in dimensions I)
1. ~~Ring layout~~ done (v4, 16 Oct 23:45, `nox/box/box-arm.md`): Scout bays and heads drawn; Overseer bays reserved, built after D10 data (pack 148 x 48 x 45 P; mass and lead exit E). Problem: the Overseer cartridge grows the box ~75 along the axle (E) unless its latch block sits off-centre (`nox/box/box-arm.md` Problem 15).
2. ~~Head stroke~~ done: glide inner end park 32, bay 84.6, drone 89.6.
3. Index time per bay pitch and per 180°: 0.6 s per 180° (E), to be timed.
4. Swap coupon: one bay module + a cartridge + a head: latch hold in every orientation, 50 cycles, release force, capture float, XT60 mate and pull (item 11) (`nox/box/box-arm.md` Problem 17).
5. Pack path through the hub and stand (Problem 30): opening X ±23, Y 96..180 for the Scout; the Overseer cartridge sets the final slot.
6. ~~Magnet size~~ hook latch v1 (23 Oct, `nox/box/box-hook-latch.md`): its funnel panel v7 (windows, bosses) and the first fit with the real drone; pogo positions with the bottom layout (Problem 31).
7. Funnel size: see Problem 36.
8. ~~Gripper mechanism~~ drawn (v4): rack and pinion, blades, spring as a band (DEFAULT); finger length from the first print.
9. Arm hardware sizes to measure (D13) before the hub and head prints.
11. **XT60 swap (23 Oct):** mate / pull force (D17) and the float carrier on the swap coupon (Problem 4): 50 cycles, every one mates with the head at 2 A; the hook latch holds the drone through the push (drone rise ≤ 3, E).
10. Charger channels for all 7 bays (more channels or a relay mux, a purchase): Spark, after DubHacks.
