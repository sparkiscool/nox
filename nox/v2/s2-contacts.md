# S2 contacts (sprung copper shoes on copper strips), Scout v2

Replaces the XT60. **16 Oct 20:08, Spark: no new parts; design only with what is on hand** (copper-clad board + AA battery spring contacts, both from the 15 Oct Amazon order). The Mill-Max pin / hard-gold strip design is retired. Numbers: `v2/dimensions.md` C (Contacts) and E (Shoes, Strips). CAD: `v2/s2_cartridge.py` (`shoe_module`, `shoe_carrier`, `shoe`, strip recess in `tray`), receiver run `v2/receiver_to.py`. Rules: `v2/README.md`.

## Parts (on hand only)
| Part | Qty | Use |
|---|---|---|
| Copper-clad board: MCIGICM FR-4, **single sided**, 70 x 100 x **1.6**, 10 pcs (Amazon B01MCVLDDZ, matched to Spark's order photo; copper weight not listed, 1 oz E) | 2 strips per cartridge + 2 shoes per drone + 2 shoes per bay | Strips 16.0 x 3.9 (cartridge), shoes 15.1 x 3.6 (drone, bays), cut with a hacksaw or score and snap, edges filed. One board makes ~50 |
| AA negative **leaf** contacts, 50 pcs (Amazon B0GTVNBSB5, matched to the order photo; listing drawing = MPD BK-209): 0.3 (0.012") nickel-plated spring steel, **7.92 wide x 11.40 tall, 6.35 deep free**, mounting leg 1.57 over its lanced tab. Force not listed | 1 per shoe | Pushes the shoe's carrier only. **Carries no current** |
| 16 AWG silicone lead (Spark has it) | 1 per shoe, 1 per strip | Through a Ø1.4 hole in the board, soldered on the copper face, filed flat |
| Epoxy (Spark has it) | | Strips into the cartridge recesses, shoes onto their carriers, a dab on each spring plate in its slot |
Not on hand, so not in the design: Mill-Max pins, gold strips, the 1000 µF bus cap, the INA226s (Problems 3 and 4).

## How it works
- **Cartridge:** one copper strip per side in a 1.5 deep recess in the guide rail, y 39 to 43, z 49.65 to 65.65 (cartridge coords), face **0.2 proud of the rail** (x ±20.2) so the flat shoe lands on copper, never on PETG. A ramp above the strip runs the rail chamfer up to the strip face: the shoe never meets a step. B+ on the drone's left, B- on the right, as before.
- **Drone (receiver):** one shoe module per side between the latch pocket block and the end post (z 47.3 to 68). A printed carrier prints in place in a chamber behind a lip (mouth 15.7 x 5.1); the copper shoe is glued on the carrier's boss with its copper face out. One leaf contact, turned so its 11.4 runs along the pack, sits with its mounting leg in a 1.8 slot in the back wall and its leaf crest on the carrier: it pushes the shoe 1.5 into the cartridge's path (free face x ±18.7). The cartridge's rail chamfer and ramp push it back to the strip face. The leg drops into its slot from above the deck before the receiver goes on the drone.
- **Current path:** pack lead → strip (copper) → shoe (copper) → shoe lead → bus. The spring only pushes. Contact is flat tinned copper on tinned copper, 15 x 3.6 per pole.
- **Force:** leaf at 3.85 deep seated (2.5 deflection from its free 6.35), 5.35 with no cartridge (1.0 preload): ~2 N per shoe (E, force not listed). Tinned contacts need ≥ 1 N. The two sides push against each other, so the latch carries none of it.
- **Error correction:** the receiver locates the cartridge (±0.3) before the shoe touches; the strip covers the shoe with 0.45 to spare in z and 0.1 to 0.2 in y; the leaf takes ±0.3 in x (stroke 1.5) and lets the carrier settle flat onto the strip.

## Assembly
1. Measure first (section below), then cut 4 strips and 2 shoes (more for the bays). File a small 45° bevel on each shoe's bottom edge and each strip's top edge.
2. Tin the copper faces thin and flat (iron, flux, wipe; sand flat with 400 grit on glass if lumpy).
3. Drill Ø1.4 in each piece: strips at z +5 from their centre (over the wall's lead hole), shoes 6.3 from the centre toward the lead notch. Lead through from the back, solder on the copper face, file flush.
4. Storage-charge the pack; cut and solder one pack lead at a time, insulate before the next. Leads run out the strip's lead hole in the wall.
5. Epoxy the strips into the recesses, copper 0.2 proud of the rail (a 0.2 feeler or tape shim on the rail as a stop). PETG softens ~80 °C: solder before gluing.
6. Receiver: free the printed carriers (push them in and out), thread each shoe lead through its carrier notch and the back-wall hole, epoxy the shoe on the boss. Drop the leaf contact's mounting leg into its slot, leaf crest toward the carrier; epoxy dab on the leg top. The lead leaves the carrier notch and goes out the back-wall hole beside the leg.
7. Before flying: 10 A through the seated pair, read the mV drop (gate below).

## Gate (DEFAULT)
1. Spring force: stand one leaf contact on its back on a kitchen scale and press the crest down to 3.85 above the scale (2.5 deflection): **≥ 100 g** (1 N). Under that, the slot moves 0.5 forward (Problem 1).
2. Bench: 20 inserts with the box head or by hand, 10 A through the contacts each time (a pack into a known load), read the drop: **≤ 50 mV for the pair (5 mΩ round trip, E)**; then a shake test with the meter on.
3. Log hover and punch-out current on the FC once flying (still a guess: 6 to 10 A hover, 30 to 40 A bursts, E).

## Problems
1. **Leaf force and its elastic range are not listed.** 2.5 deflection is a guess from AA holder use; the scale check (gate 1) settles it. Sizes are from the two listings (P listing), not measured; the CAD is parametric (`s2_cartridge.py`, top block).
2. **Tinned copper wears and oxidises.** Flat tin on tin at ~1.8 N is fine for a prototype but frets; re-tin the shoes when the drop climbs. Clean with IPA.
3. **No bus cap:** the drone loses power for the length of a swap unless the box's keep-alive pads carry it (`nox/box.md`). Without them the FC and Radxa reboot every swap.
4. **No INA226:** no after-swap resistance check in the box. The latched check is the pack voltage on the drone bus.
5. **Contact before latch:** the shoes reach the strips 3.8 before the bolts seat (shoe bottom 39.2, strip top 43). A half-inserted pack is live. The bolts still hold only when seated; the box must finish the stroke.
6. **Bays:** the charge bays (`nox/box-case.md`) still have the old 8 pin bores; they need the same shoe module (box chat).
7. Every mate sparks a little at the shoe; inspect and re-tin.
