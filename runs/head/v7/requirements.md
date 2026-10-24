# Heads A / B v7: piece requirements and system asks (24 Oct 2026, heads v7 part session)

Source: `nox/box/head_v7_layout.py` (layout, checks), `runs/head/v7/layout/layout.json`, the arm system pass
(`nox/dimensions.md` "System box swap: arm", `runs/arm/v7/system.json`) and the latch interface ("System box swap: latch",
latch PR 69; receiver v2 keeps the cartridge v5 and sliders v5, branch `part/battery-latch` cceae30). The session owns only
the heads, so the dimension rows below go to the arm system through the PR description and the hub.

## Proposed replacement for the row "Heads v7 grabber concept" (C / E)

| Item | Value | Tag |
|---|---|---|
| Heads v7 grabber (layout, concept review r5) | two carriers (PETG) on two 3.0 carbon rods along X (Y 161 / 169, u 6.75), each a blade 3 x 10 (Y 160..170) with a 0.8 rib into the tab's grip pocket at u 19.4, inner faces 6.0 squeezed / 11.0 open (stroke 5.0), tip u 21.4 under the slider bar (u 21.9); the boss on the two rods plus the seat's bore on the spring rod guide the carrier over 41 (cam force 1.54 x the spring's, mu 0.3) | C |
| Spring | one music-wire compression spring crossed between the carriers on a third 3.0 rod (Y 181.25, u 4.5): 0.7 x OD 5.5, L0 70, 37 coils, k 0.58 N/mm: 15.1 N squeezed (L 44), 20.9 N open (L 34), tau 905 MPa (0.86 of 0.45 UTS); the spring force stays inside the carriers | E |
| Stops (no sync gear) | each carrier stops inward on a body block (face X +-9.8, Y 171..174.5, u 2..7.5, blade inner 4.8): the cam opens both, the spring closes both onto the tabs; with no cartridge a carrier moves at most 1.2 past squeezed; lateral loads on the carried cartridge go through a blade to its stop and the body, never to the cam or the spline | C |
| Cam and servo | servo shaft along Y at (X 0, u -2.5), case top Y 138, body X -10..30 (DS3218) / -10.2..30.7 (MG995) (long side to +X), u -12.5..7.5; clamping metal 25T disc horn; PETG cam (6 thick, Y 148.5..154.5) base 4.0 / lobe 10.5, modified sine over 100 deg + dwells (turn 120), two 623ZZ roller followers (gap 1.5 at rest: the servo is unloaded, the grip holds unpowered; the open dwell holds unpowered too, the roller force through the shaft); peak 3.7 kg.cm with the carriers' guide friction (x1.54, mu 0.3, guides 41 apart, conservative): DS3218 x5.4, MG995 x2.4; pressure angle <= 29.0 deg; the roller forces cancel on the spline | C / E |
| Roller pin | 3.0 carbon, 8.8 long: flush with the roller (623ZZ inner race epoxied on it), 3.8 into the lug through a Ø7 collar past the cam's end (Y 154.5..155.6) | C |
| Clamp (cartridge v6) | the latch's 2.0 centre rib stops the tabs at +-1.0: blade inner 7.0 at the clamp (DEFAULT, latch 24 Oct), the crossed spring at L 42: 16.2 N per blade; the cam's rest gap 2.5 | C / E |
| Hold and release | hold while the XT60 unmates: 28.6 N per tab opens with 4.9 N vs 16.2 N at the clamp (x3.3; 15.1 N at the v5 datum, x3.1); release x5.4 (Scout) / x3.6 (Overseer) | E |
| Servo (bought) | dimensions.md M (spec): DS3218 40 x 20 x 40.4, ear underside 27.7, 54.5 over the ears, holes 49.5 x 10; MG995 40.9 x 20, 37 to the case top, ear underside 26.8, 54 over the ears (holes open, Q12); the case top at Y 138 for both, the ears land at Y 125.3 / 127.8; shaft offset and lead exit E: CHECK the DS3218 on hand before print (P) | spec / E |
| Servo mount | a U cradle round the case's upper 12 (open toward -u: the servo slides in and out along u through a window under its footprint, commanded OPEN first so the carriers stand apart), Ø12 pads under the four ears, the ears in slots 0.3 over their thickness; a printed retainer bar (3.3 g) under the servo, 2 M3 countersunk along +u into heat-set inserts in drawn Ø12 bosses, driven through the window (access swept: service pose head A t 11.5, phi 90, A out at z0 108.6; head B t 21.5, phi -135, both parked) | C |
| Rack tail | the pinion's separating force (22.1 N at the 60 N insert, tangential x tan 20, system.json) bends the tail: a drawn backing rib behind the web, per head from its free space (space map, >= 1.0): head A X 29.5..39.5 over the belt (Y 63..68.75, u -62..-11.5), up to the plate top for u > -37..-40.5 and down to Y 59.25 at the root (u > -31), 6.6 g; head B X 37.5 / 35.5 / 32.5 beside the hub and the detent at park (Y 63..68.75) and the root block X 32.5 (Y 59.25..63), 1.7 g; hand check root index 0.13 (A) / 0.29 (B) vs ~1.5 without (E). Alternative: a backing roller on the arm opposite the pinion (an arm part; not taken) | C / E |
| Rack bar | the m1 rack as its own PETG bar, 5.0 x 14 x 93 (A, u -63..30) / 84 (B, u -54..30), teeth tips X 46.5, root 44.25 (arm_v5_cad phase), printed on its Y 73.25 face (tooth profile in the layers), epoxied along its back face (X 41.5) onto the body web (X 39.5..41.5, 13.5 x 93) | C |
| Body | one PETG print from `nox/topo`, built +x on a 2 mm land at X -34 (the column's flat -X flank); plate 56 x 5.6 (Y 68.75..74.35) with a 12.5 x 10 fillet into the wing; column bottom u -16.3 (head lane r >= 95 at the drone insert); M5 x 20 axle bolts (countersunk, tails Y 54.35) | C |
| Push | deck pads X +-16.0..18.2, Y 155..175 on the latch frames (Scout 15.5..19, Overseer ..26.5): 1.09 MPa at the 74 N stall with a 1.0 head offset (1.7 of the worse pad on the frame, E); at that offset the pad edge meets the Scout v6 TPU strip's edge (both bear on the cartridge, harmless) | E |
| Halls | DRV5055A4 at X +-7.5, Y 157, face u 14.4 (1.0 under the flat bottom) on 4.2 x 3.2 boards (package long side along Y) at Y 155.5..158.9 under the deck, past the cam, 1.0 off the carriers and the blade (the pocket and the package's hole open into the blade slot); the package opens into the blade slot, 1.5 clear of the blade; 3.3 V, summed 3:1 onto one line (box electronics PR 103) | C |
| Wiring | the 5-wire head ribbon (V, G, S, H, 3.3 V; 6.1 x 1.3 E) down an 8.2 x 3.3 slot at X 0, u 7..10.3 through the plate into the rail's 5.0 mouth (rail v5); the hall boards' leads (3 x 30 AWG each, E) by a riser and a crossing to one closed run a side above the servo (|X| 8.8..12.3, u 9.9..12.2: >= 2.2 to the servo pocket and the horn, >= 1.9 to the cam, 2.7 roof; every section >= 2.0 x 2.0; threaded with a pull wire), merged at Y 92..96 into the ribbon slot; the servo lead from its grommet (far end face near the base, E, CHECK) down the end face, over the retainer's tab to the +X end of the mated JR pair laid along X under the case (X 9..38 for the longest pair, 29 E, + 4.0 bend room each end), the socket's wires up from its -X end to the ribbon slot; the retainer's tab closes the pair and the lead from below; with the retainer off they drop out along -u (swept); the boards glued in their pockets, the ribbon tied at the plate | C / E |
| Mass (E) | printed grabber parts 22.2 g (M: carriers 5.7 + 5.7, cam 1.0, rack bar 5.6 / 5.2, retainer 4.2), body from its topology run (~75 g, E), bought ~77 g (DS3218 60): head ~175 g (frame: 120 head + 60 carriage) | E / M |

## System asks (accepted by the arm system worker, 24 Oct: 1 and 2 as DEFAULTs in the PR description; 3 used; 4 kept in the latch loop; 5 sent to box structure)

1. **Carriage plate top Y 72.75 -> 74.35 under the cartridge** (1.0 under the Overseer receiver at Y_C 165; the system's own
   1.0 clearance rule; the Y_C >= 163.4 limit moves to 165.0). Upper-bound FE (`head_v7_topo.py bound`, the whole design space
   filled, h 1.5): the 4.0 plate between the column (u <= 14.9) and the eccentric / far wheels reads 1.03 (build +y) / 0.73
   (build +z) in the swap pull and insert vs the 0.5 target; with the plate top at 74.35: see `runs/head/v7/bound_*.json`.
   The axle bolts become M5 x 20 (tails Y 54.35, 1.35 over the hub's wheel lanes).
2. **Insert-lane keep-out at the drone, lower bound Z 111.5 -> 95.0** (head and grabber; the horn's bottom at u -15 is Z 96.3 at
   the commanded insert) and the carriage lane's Z 100 -> 89 (the plate grows to u -18 where the arm leaves room). Checked: the
   catch (`box_catch.assembly.json`) is bbox-apart (Z >= 154); box structure v2: only the coarse bay body envelopes (r >= 112)
   touch a head at its own bay, as they do the v6 head; the arm sweep with every receiver reads 0 overlaps (4 cycles).
3. **The frame's carriage-to-head moments assume a rigid pitch grip** (swap 1.8..2.3 N.m). The latch grip (rib in a 5 deg
   pocket + pads 20 long) transmits at most about F x 10 mm in pitch, so the receiver carries the cartridge's pitch couple and the
   head root sees the pull at the tab line: 57.3 N x 90.65 = 5.2 N.m (pull) / 6.7 N.m (stall) (C, layout.json; the concept had 5.3 / 6.8 at 92.25). The wheel
   axial loads follow (~190 N rigid plate, ~380 N near + eccentric only, pull). The heads' topology run applies the interface loads themselves (the moments follow from them).
4. **Lateral location of the carried cartridge (to the latch):** gripped, the two tabs meet each other, so the cartridge body can
   slide on the slider bars about +-4 in x relative to the head (only friction holds it); at the next receiver the lead-ins
   capture +-3. Proposal: a fixed centre rib in the cartridge between the tabs (tabs stop on it, stroke 3.5, blades squeezed at
   +-6.5): the squeezed tabs then clamp the cartridge to the head. The head follows any squeezed position (a parameter).
5. **Servo stall (to box electronics):** a DS3218 stalled on a jammed blade puts 163 N on a roller (76 MPa Hertz on the PETG
   cam): cut the servo after 0.5 s at stall current; the cam is a 1 g spare.
