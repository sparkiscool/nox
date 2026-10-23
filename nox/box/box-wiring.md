# Arm harness v1: one 19-wire bundle through the sun post, servo ribbons folded in the bar slots (17 Oct 2026)

Closes box-arm.md Problem 2's open item (the bundle's arm-side tie and route). Numbers: `nox/box/arm_wiring.py` (reads nothing; standalone calcs, C unless tagged). Parts + clearance check: `nox/box/wire_parts.py`. Geometry context: `nox/box/arm_layout.py` WIRING block.

## What crosses the twist (the old 15-wire count missed the servos)
| Load | Wires | Gauge |
|---|---|---|
| 3x NEMA17 (A, B, axle) | 12 | 26 AWG silicone |
| 2x head servos, shared 6.8 V rail | 2 | 22 AWG (buck in the bay, E DEFAULT) |
| 2x servo signals | 2 | 28 AWG |
| hall A3144 (homing) | 3 | 28 AWG |
**19 wires, bundle Ø7.3 (C, fill 0.6 E) in the Ø8.5 post bore: 1.2 clear.** 24 AWG steppers would make Ø8.3 and bind, so 26 AWG, justified by duty: the detent holds with drivers OFF (box-arm.md detent firmware), so the steppers idle most of the cycle; 26 AWG at 2 A dissipates ~1.3 W total in the 90 bore section only at full continuous current (E). If the bore section runs warm in the swap test: set drivers to 1.2 A (arm torque 2.4 -> ~1.4 N·m, still 2.5x the detent's 0.55; head stall pull 93 -> ~56 N, still 3.7x the 15 N insert; the 10 kg clamp coupon test stays conservative).

## Path (fixed -> rotating)
1. **Fixed tail:** SKR + servo buck in the bay (Y -60) -> **stub grommet**: printed Ø24 split plug pressed into the stub bore's back end, 2 zip-tie slots clamp the bundle at Y ~ -10 (separate part, the stub print is untouched).
2. **Twist, Y -10..80 (L 90):** loose bundle up the stub Ø24 bore, the post Ø8.5 bore, the spigot / 80T bore (drilled Ø12, Problem 4), exits ON AXIS at Y 72. Bounded travel ±210.5° (soft limits -278..+143) -> outer-wire strain **0.74 % (C)**, fine for silicone stranded. No spiral wrap (would bind); talc the bundle (DEFAULT).
3. **End clip saddle (rotating anchor):** the clip's saddle clamps the bundle **on the axle axis at Y ~82** (2 zip slots). On-axis = pure torsion, no wrap round the Ø12 exit lip (an off-axis tie would flex the last 17 mm every cycle). Saddle bottom Y 78: 6 over the fixed 80T collar top (Y 72).
4. **Fan-out at Y ~86:** stepper 12 + hall 3 drop in the free sector between the bars (**135°, r ~32**: clears the ring r 23.5, roller arm at -105°, axle-loop sector at -120°, head B belt Z 48.65+, all belts Y >= 56) down to the plate top (Y 50), then lace along the hub's struts at r ~46 to each motor's lead exit and the tower's hall pocket. Micro zip ties round the generated struts, ~3 stations per branch (DEFAULT; **no hub regen**, v5 print file stays).
5. **Servo ribbons:** one 3-wire ribbon per head (V 26 / G 26 / S 28, ~3.9 x 1.3) from the saddle into each bar's **top-face T-slot at X -10** (belt is on +X; the bottom slots are taken by the bar posts' M5 slot nuts). The clip guides it up the bar's end face into the slot.

## Moving slack: fold-in-slot (per head, C)
The ribbon lies in the 6 x 6 slot, anchored at r 23 (clip), **folds 180° back on itself inside the slot** and runs to an entry hole in the glide (printer-carriage style). Stack 2 x 1.3 + fold = 3.4, **2.6 below the slot mouth**: the glide (0.3 off the bar) passes over it. In-slot length 80.6; entry at glide inner end + 8 (r 40 parked .. 97.6 at the drone); fold slides r 71.8..100.6, **23.4 short of the idler-end slot nuts (r >= 124 E)**. 3 snap-in slot clips per bar with a 3.5 clear window keep the ribbon seated when the bar is inverted; the fold passes under them. The whole run lives inside the glide's own swept tube, so the sweep's existing head clearances cover it.
- **Head v5 feature (for the head chats):** entry channel Ø4.5 from the top slot at X -10 (B mirrored), 8 above the glide's inner end, up through the glide top skin to the body surface, + a ribbon anchor rib beside it (zip slot); then a surface groove along the generated body to the servo plate (ribbon zip-tied at the plate).

## Parts (`wire_parts.py`, built + checked 17 Oct 16:45; STLs in the wiring chat)
- `wire_grommet` **5.9 g (M)**: Ø23.8 split press plug (3 crush ribs 0.2 proud -> Ø24.2), Ø28 x 3 flange, Ø7.5 bore, 2.0 slit (opens over the finished bundle: no threading), 2 zip slots in the flange. Print flange down.
- `end_clip` x2 **6.1 g each (M)**, identical parts (both bar frames put the belt on +X): plate 40 x 20 x 5 on the bar's inner end face, **2x M5 self-tapped into the 2040's Ø4.2 core holes** (counterbored, button heads); on-axis C-cradle ID 8 at Y 78..86 opening sideways (bundle drops in, 2 zip grooves; used on bar B, A's stays empty); ribbon groove 4.4 x 2 up the front face at X -10 over the top edge into the slot; lash slot at the +X corner for the stepper tail. Print lying on its Y 74 face.
- `slot_clip` x6 **0.2 g (M)**: 10 long twist-in slot cover segment, wings 8.6 x 0.6 in the lip undercut, spine top 0.3 below the bar surface, **3.6 clear beneath** (fold stack 3.4 + wire passes; slot profile E, CHECK: the hand test validates).
- **Clearance (any arm angle, `wire_parts.check`, 20k surface samples):** 80T flange 18.5, body 10.0, collar **2.86**, detent ring **4.1** (all C, rule 1.0). The fixed set above Y 54 is axisymmetric, so one static check covers every angle; the detent roller/tower/bands rotate with the clip (static separation, -105° vs 90/180°).

## Continuity and connectors
Continuous conductors from the bay to each load: **no connector inside the bore or on the arm** (DEFAULT). Steppers keep their own pigtails + JST-XH inline right at each motor (service). Servos keep their stock plugs; mate at the servo plate. Hall soldered + heat-shrink at the tower.

## Cut list (E, +20 % slack)
stepper A 4 x 690 · stepper B 4 x 620 · axle 4 x 630 · hall 3 x 595 · servo ribbons 2 x 240 · servo power pair 2 x 435 (22 AWG) · servo signals 2 x 435. Totals: 26 AWG ~7.8 m, 28 AWG ~2.8 m, 22 AWG ~0.9 m.

## Problems
1. **Wire count was wrong in box-arm.md (15 -> 19):** the head servos were never in the twist count. Fixed here; Problem 3 there updated.
2. **Bundle heat in the bore (E):** 26 AWG at full 2 A continuous warms a closed PETG tube. Low duty makes it moot (E); fallback = drivers to 1.2 A (margins above). Check by feel after the first full-cycle soak test. **XT60 (23 Oct): the 1.2 A fallback no longer pushes the XT60 home (insert 60 N E, `box-arm.md` XT60 swap); keep the head drivers at 2 A for the insert (short, so little heat) and drop only the hold current.**
3. **Fold-in-slot is untested:** ribbon stiffness, fold friction, lift toward the mouth. Fix: dry-run one ribbon in a loose 2040 offcut over 100 strokes by hand before the heads print; add a 4th slot clip if it lifts. **The slot clip's wing/undercut sizes ride on the E slot profile: caliper the slot (opening, depth, undercut width) before trusting the fit.**
4. **The 135° drop lane and the saddle are not yet in the sweep.** The v5-heads full sweep chat adds three wire keep-outs (saddle Ø10 at Y 78..86 on axis, drop lane Ø8 at 135° r 32 Y 50..86, hub lacing Ø6 at r 46 Y 8..50) as arm bodies vs the fixed parts.
5. **Fixed-tail length (250 E) is a guess:** the bay layout (PSU, SKR, buck) is undesigned. Cut the fixed tails last.
6. **Servo rail (6.8 V buck, output >= 6 A peak E) is not on the buy list.**
7. **M5 self-tap into the 2040 core holes (end clip):** Ø4.2 is the standard M5 tap size; thread-forming screw or run an M5 tap first (Spark has taps). If a core hole measures over 4.4, switch that clip to a drop-in T-nut in the end of the top slot.

## Status
Step list (one at a time, Spark 17 Oct 16:24): **1 done** -> 2 Spark prints the 3 parts + hand-tests the fold in a 2040 offcut (Problem 3) and calipers the slot profile -> 3 wire + buck stock check against the cut list -> 4 head chats take the entry channel -> 5 sweep chat takes the keep-outs -> 6 harness build at assembly.
17 Oct 16:45: **parts built and checked** (`wire_parts.py`): grommet 5.9 g, end clip 6.1 g x2 (identical both bars), slot clip 0.2 g x6; worst fixed-part clearance 2.86 (cradle to the 80T collar), rule 1.0; STLs delivered in the wiring chat.
17 Oct 16:10: harness v1 designed; bundle fits (Ø7.3 in Ø8.5, 1.2 clear), strain 0.74 %, fold fits the slot with 2.6 under the mouth and 23.4 to the idler clamps. Open then: parts (now done), head v5 entry feature (head chats), sweep keep-outs (sweep chat).
