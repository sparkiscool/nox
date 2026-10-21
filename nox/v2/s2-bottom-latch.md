# S2 bottom latch (battery under the drone), Scout v2

**Design locked by Spark 15 Oct 00:06** ("this is perfect", on the "S2 Latch Cycle" artifact): squeeze-release sliders on the cartridge, passive pockets on the drone, one two-finger gripper on the box head does grip and unlatch in one squeeze and holds the pack in any orientation. Replaces the drone-side flexure hooks and the head blades (concept 14 Oct 21:50 / 21:56). Artifact: "S2 Latch Cycle" (animated cross-section of the cycle, coupon geometry). **Coupon v1 drawn 22:30 (standalone rig, not fitted to the pack, Spark 22:13): `v2/s2_latch_coupon.py`.** Numbers: `v2/dimensions.md` J (E until printed). Rules and loads: `v2/README.md`. Box it works with: `nox/box.md` (swap wheel). Research: `claude/research/2026-10-14-bottom-cartridge-latch.md` (hook era; loads and contact rows still valid). Files: `Documents\Nox\scout_v2\bottomlatch\v<N>\`, names `s2_latch_coupon_<part>_v<N>`.

## Brief (Spark)
- Battery bottom mounted; the box pulls it out and pushes another in from below.
- Contacts with no mating force, holding force designed into the latch.
- Must survive all crashes and landings, easily unlatched by the box.
- Unlatch = two tabs pressed in from the middle, linked to latches on the sides, so the box arm is two small fingers that close (21:50).
- Those two fingers alone must hold the cartridge upside down and sideways (bays on the wheel), secure and tight once unlatched (21:56).
- Work the mechanism on its own first, no pack fit (22:13).

## Why bottom mount is hard
Hanging underneath, hover, climb, every funnel landing and an upright drop all load the latch in its release direction. Pack 165 (P) + cartridge ~9 (E) + sliders ~5.4 (E) = ~180 g: 1.8 N hover, 7 N at 4 g, 10.6 N at 6 g, 62 N at 35 g (E).

## How it works (locked; coupon numbers C unless tagged)
Coupon coords: x = slide axis (0 at centre), y = up toward the drone, z = part thickness = print Z (14).
- **Cartridge side: two mirrored one-piece PETG sliders**, each = side bolt + bar + central tab, guided in a channel in the frame's bottom member (0.35 clearance in y, print-in-place). Pressing the tab inward retracts the bolt inward (same direction, no linkage). Travel 4.0 to one central stop block on the frame (the block runs up to the top member as a spine); outboard stop = the frame wall. Return: one PETG leaf per slider, 0.8 thick x 13 wide x 26 free, from the frame's top member to the bar, zero pre-bend (no creep at rest); fixed-guided, 2.5 N at full stroke, strain 1.4 % per swap (E). Slider 3.9 g in the coupon (M; thinner in the real part).
- **Bolt:** 13 wide x 4 thick, 3.0 overlap into the drone pocket, 25 to 28 from the centre; **bearing face = its underside**, undercut 10° (drops 0.53 at the tip) so a pull-out (cartridge pulled down) drives it outward into the pocket; 1.0 lead-in chamfer on the tip's top edge; the wall hole is 0.7 deeper than the channel so the wedge retracts into the wall. The bolt is rigid; the leaf is only the return (side-release-buckle lesson: a prong that is both spring and latch flexes open under load).
- **Tab:** 6 thick x 14 wide, 20 below the frame bottom, faces at 8 and 14 from the centre at rest (4 and 10 squeezed). Outer face carries the **grip pocket**: 8 (z) x 4 (y) at the mouth, 10° included taper on all four walls, 3.0 deep, centred 11 below the frame.
- **Drone side (passive, nothing moves):** one pocket per side in a rail, mouth 4.6 tall at y 1.7 to 6.3, 3.3 deep, floor dropping 10° going in, flat ceiling, 1.0 chamfer on the mouth's top edge; bearing wall 3.0 under the pocket. Bolt bottom sits 0.25 above the floor at rest, so the latch bears after a 0.25 drop. Positions from the sweep with the bottom layout.
- **Preload:** TPU 95A strip between cartridge top and drone, 12 N, ≥ the 6 g landing load so the pads never lift. It pushes the cartridge down onto the pocket floors, so through the 10° undercut it wedges each bolt outward (~1 N) and adds ~1.8 N of friction: the bolts are locked by the preload itself, not by the leaf.
- **Half-latch:** pads reach the pins only after the bolts seat, so a half-in pack can't arm. Latched check after an insert: the drone reads pack voltage on its bus through the keep-alive link (replaces the tug test).
- **Box side (one gripper per swap-wheel head, `nox/box.md`):** two parallel fingers (rack and pinion, 1 MG90S), **normally closed by a spring at 8 N per finger**, servo only opens (12 stroke each: 6 clearance + 4 travel + 2). Each finger ends in a frustum rib matching the tab pocket (0.15 clearance per side). Squeeze = rib seats (zero play from the taper), then drives the tab to the stop: bolts out. Hold in any orientation goes through the pocket walls, not friction: 10 N design any direction (3 g wheel index x 2 margin). Power loss keeps the grip.

## Coupon v1 (`v2/s2_latch_coupon.py`, run 14 Oct 22:30; all C / M)
Three PETG prints, all 14 tall, flat on the plate, no supports: **cartridge** (frame 50 x 35 + both sliders in place, 21.8 g M, 56 x 55 x 14), **drone** (U with the two pockets and a string window on top, 12.5 g M, 64 x 46 x 14), **finger** (print 2, same part both sides: 20 x 30 block, rib, Ø4 hole for a band or spring, 10.5 g M). Checks (C): sliders clear the frame through the whole 4.0 travel; retracted cartridge passes the drone rails; latched bolts bear on the pocket floors after a 0.25 drop; finger rib seats in the pocket with the finger face on the tab. Bridges: 0.5 skin over each channel (4.7 span), pocket top walls 3 span. Settings DEFAULT: 0.2 layers, 3 walls, 40 % gyroid, PETG.
Tests, in order: (1) free the sliders (twist), measure tab travel and bolt stand-out; (2) press the cartridge up into the drone coupon with the tabs squeezed, release: bolts click out, measure the drop to bearing; (3) hang weight from the stop-block hole: 2 kg (~20 N, 2x hover + preload) then to failure, note where; (4) fingers on the tabs with a band at ~8 N each: release force by pull gauge, then hold the cartridge by the fingers alone in 6 orientations with 1 kg on it; (5) 50 squeeze cycles, re-measure travel and return.

## Sequences (swap-wheel head, drone held by the magnet)
- **Release:** fingers open; head extends until the ribs sit beside the pockets; servo off, spring closes: ribs seat, tabs go 4.0 to the stop, bolts clear (4 N per finger while the bolt wedges out under preload, 2 N after); head retracts with the pack. Lift first not needed (8 N > 4 N); 1 mm over-travel before the squeeze is the fallback if the coupon says otherwise.
- **Insert:** head extends with the pack held (bolts retracted) up the drone axis until the cartridge top meets the preload strip (~12 N through the pocket upper walls), 1 mm over-travel, servo opens: leaves push the bolts out into the pockets, preload wedges them home, pads make, bus sees the pack; head retracts.
- **Bay in / out:** same grip; the bay's TPU lips (~5 N) are beaten by the extender through the pocket walls. Squeezing a pack in a bay retracts its bolts too, harmless; the bay cradle clears the 3.0 bolt stand-out.

## Sizing (hand calc, E; FE recheck when drawn)
| Case | Per bolt | Result |
|---|---|---|
| Hover 1 g + preload | 7 N | |
| Landing 6 g + preload | 11.3 N | pads don't lift (preload 12 N > 10.6 N) |
| Crash 35 g pull-out + preload | 37 N | bolt bending ~1.3 MPa (13 wide), index ≤ 0.1; pocket wall 3.0, index ≤ 0.1; outward wedge into the frame stop 6.5 N |
| Side hit 35 g along the slide | slider inertia 1.3 N (3.9 g coupon) vs preload wedge 1.0 + friction 1.8 + leaf 0 | index ~0.5 in the coupon, ~0.3 with the slimmed slider; bolt stays |
| Worst overlap left | 2.35 of 3.0 | after ±0.2 per part and 0.25 lateral play |
| Leaf strain per swap | 1.4 % (fixed-guided, 0.8 x 26) | PETG yield ~3 to 4 %; zero pre-bend |
| Release | 4 N per finger in the undercut, 2 N after | spring 8 N per finger |
| Grip hold, any direction | 10 N through 8 x 4 pocket walls | ~0.3 MPa in the rib |
| Install push | 12 N preload + 2.5 N leaves through the pocket upper walls | 10° taper splays the fingers ~2.5 N, under the 8 N spring |

## Why not the alternatives
Flexing prong as the latch (plain side-release buckle): parts on its own under rising load. Hooks on the drone + blades: two more moving parts on the drone, blade servo and cup lips on every head, slots in the ledges; the gripper does all of it in one squeeze. Friction-only grip: lets go upside down above the spring force; the tapered pocket holds by geometry. Servo-held grip: drops the pack on power loss; spring-closed does not. Coil spring + pivot per bolt: more parts than one printed leaf. Bayonet: box must rotate the pack. Push-push: a crash push-in then pull-out releases it. Magnets: heavy, weak in shear.

## Patents (lay, E)
Latch on the pack, passive drone, two press-in tabs: the power-tool pack family (single-button Makita / DeWalt style, two-button Milwaukee) is old; the pressed-from-the-middle tabs and the one-squeeze grip-and-release are the part to read. No blades on the box, so the Coretronic resemblance from the hook era is gone. Pads-make-after-seat is geometry, not a latch switch (Milwaukee US11179841): attorney (current-parts Waiting 7).

## Problems
1. **Underside stack sits where the pack would go** (current-parts Problem 31). Best: lay out the bottom pack and the stack together before fitting; flow sensor stays lowest.
2. **Leaf fatigue unknown.** 50-cycle test on the coupon; fallback = feeler-gauge steel leaf glued in.
3. **Slider play vs overlap:** 0.35 channel clearance eats overlap and lets the bolt tilt; coupon measures real overlap and the drop to bearing. Fix if short: overlap 3.5, or a tighter channel printed as a separate slider.
4. **Pack path through the funnel** (current-parts Problem 30): tabs add 20 below the cartridge, fingers open to ±19 inside the ~40 width.
5. **Gripper spring force is a guess (8 N):** must beat 4 N release and the 2.5 N install splay, and the MG90S must open it (fine, ~20 N at a 10 mm pinion). Set with the coupon test 4.
6. **Print-in-place sliders may fuse** (0.35 side gaps, 0.5 under the skins). Fix: break free with a twist; if they won't, v2 prints the sliders separately and the leaf slots into the top member.
7. **Nothing retains the slider in z** in the coupon except the leaf and the 0.5 skin. Real part: the cartridge frame closes the channel in z; fine for the rig.

## Next
Spark prints coupon v1 (cartridge, drone, 2 fingers) and runs tests 1 to 5 → fix what fails in v2 → then the bottom layout (Problem 31), pocket and tab positions from a sweep, and the fitted cartridge + drone pockets with noxopt.

## Status
15 Oct 00:06: design locked (Spark, on the artifact). Coupon v1 drawn and checked (C), previews sent 14 Oct 22:30. Not printed.
