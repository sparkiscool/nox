# Heads A / B v7: piece requirements and system asks (24 Oct 2026, heads v7 part session)

Source: `nox/box/head_v7_layout.py` (layout, checks), `runs/head/v7/layout/layout.json`, the arm system pass
(`nox/dimensions.md` "System box swap: arm", `runs/arm/v7/system.json`) and the latch interface ("System box swap: latch",
latch PR 69; receiver v2 keeps the cartridge v5 and sliders v5, branch `part/battery-latch` cceae30). The session owns only
the heads, so the dimension rows below go to the arm system through the PR description and the hub.

## Proposed replacement for the row "Heads v7 grabber concept" (C / E)

| Item | Value | Tag |
|---|---|---|
| Heads v7 grabber (layout, concept review r2) | two carriers (PETG) on two 3.0 carbon rods along X (Y 161 / 169, u 6.75), each a blade 3 x 10 (Y 160..170) with a 0.8 rib into the tab's grip pocket at u 19.4, inner faces 6.0 squeezed / 11.0 open (stroke 5.0), tip u 21.4 under the slider bar (u 21.9); boss 12 long (anti-rack needs 7.2) | C |
| Spring | one music-wire compression spring crossed between the carriers on a third 3.0 rod (Y 181.25, u 4.5): 0.7 x OD 5.5, L0 70, 37 coils, k 0.58 N/mm: 15.1 N squeezed (L 44), 20.9 N open (L 34), tau 905 MPa (0.86 of 0.45 UTS); the spring force stays inside the carriers | E |
| Sync | m1 12T pinion (PETG) on a fixed 3.0 pin at (X 2.5, Y 167), racks on both carriers: equal and opposite travel; lateral loads on the carried cartridge go to the body, never to the cam or the spline | C |
| Cam and servo | servo shaft along Y at (X 0, u -2.5), case top Y 138, body X -10..30.5 (long side to +X), u -12.5..7.5; clamping metal 25T disc horn; PETG cam (6 thick, Y 148.5..154.5) base 4.5 / lobe 10.5, modified sine over 100 deg (turn 120), two 623ZZ roller followers (gap 1.0 at rest: the servo is unloaded, the grip holds unpowered); peak 2.3 kg.cm: DS3218 x8.7, MG995 x3.9; pressure angle <= 26.5 deg; the roller forces cancel on the spline | C / E |
| Hold and release | hold while the XT60 unmates: 28.6 N per tab opens with 4.9 N vs 15.1 N (x3.08); release x5.0 (Scout) / x3.4 (Overseer) | E |
| Servo mount | a U cradle round the case's upper 12 (open toward -u: the servo slides in from the axle side), Ø12 pads under the four ear holes (>= 2.6 web to the case pocket), M3 x 8 from -Y through the pads, nylocs on the ears, driver channels kept in the body | C |
| Push | deck pads X +-16.5..20, Y 155..175 on the latch frames (both packs): 1.23 MPa at the 74 N stall, worst offset | E |
| Halls | DRV5055A4 at X +-7.5, Y 157, face u 14.4 (1.0 under the flat bottom) on 5 x 6.5 boards under the deck; the package opens into the blade slot, 1.5 clear of the blade | C |
| Mass (E) | printed grabber parts ~12 g, body from its topology run (~75 g, E), bought ~77 g (DS3218 60): head ~165 g (frame: 120 head + 60 carriage) | E / M |

## System asks (to the hub)

1. **Carriage plate top Y 72.75 -> 74.35 under the cartridge** (1.0 under the Overseer receiver at Y_C 165; the system's own
   1.0 clearance rule; the Y_C >= 163.4 limit moves to 165.0). Upper-bound FE (`head_v7_topo.py bound`, the whole design space
   filled, h 1.5): the 4.0 plate between the column (u <= 14.9) and the eccentric / far wheels reads 1.03 (build +y) / 0.73
   (build +z) in the swap pull and insert vs the 0.5 target; with the plate top at 74.35: see `runs/head/v7/bound_*.json`.
   The axle bolts become M5 x 20 (tails Y 54.35, 1.35 over the hub's wheel lanes).
2. **Insert-lane keep-out at the drone, lower bound Z 111.5 -> 95.0** (head and grabber; the horn's bottom at u -15 is Z 96.3 at
   the commanded insert) and the carriage lane's Z 100 -> 89 (the plate grows to u -22 where the arm leaves room). Checked: the
   catch (`box_catch.assembly.json`) is bbox-apart (Z >= 154); box structure v2: only the coarse bay body envelopes (r >= 112)
   touch a head at its own bay, as they do the v6 head; the arm sweep with every receiver reads 0 overlaps (4 cycles).
3. **The frame's carriage-to-head moments assume a rigid pitch grip** (swap 1.8..2.3 N.m). The latch grip (rib in a 5 deg
   pocket + pads 20 long) transmits at most about F x 10 mm in pitch, so the receiver carries the cartridge's pitch couple and the
   head root sees the pull at the tab line: 57.3 N x 92.25 = 5.3 N.m (pull) / 6.8 N.m (stall), the concept's numbers. The wheel
   axial loads follow (~190 N rigid plate, ~380 N near + eccentric only, pull). The heads' topology run uses 5.3 / 6.8.
4. **Lateral location of the carried cartridge (to the latch):** gripped, the two tabs meet each other, so the cartridge body can
   slide on the slider bars about +-4 in x relative to the head (only friction holds it); at the next receiver the lead-ins
   capture +-3. Proposal: a fixed centre rib in the cartridge between the tabs (tabs stop on it, stroke 3.5, blades squeezed at
   +-6.5): the squeezed tabs then clamp the cartridge to the head. The head follows any squeezed position (a parameter).
5. **Servo stall (to box electronics):** a DS3218 stalled on a jammed blade puts 163 N on a roller (76 MPa Hertz on the PETG
   cam): cut the servo after 0.5 s at stall current; the cam is a 1 g spare.
