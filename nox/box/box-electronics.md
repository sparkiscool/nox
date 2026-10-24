# Box electronics v1 (24 Oct 2026)

The box's electrical system end to end: mains to the 24 V supply, four rails, the box computer and its I/O, the stepper board, the keep-alive to the pogo pads, the two live bays and their charger, the swap state machine with every abort, wire gauges, connectors and the cut list. Numbers: `nox/box/box_electronics.py` (placement search, nets, drops, budget, cut list, diagrams); run `runs/box_electronics/v1/` (`result.json`, `diagram.html` with `schematic.svg`, `layout.svg`, `states.svg`). Interface numbers: `nox/dimensions.md` I "Box electronics v1". Arm harness (the 19-wire twist bundle): `nox/box/box-wiring.md`. Buy rows: `nox/parts-list.md` Box. C unless tagged; board sizes spec or E; **DEFAULT** = chosen without Spark, alternative in `nox/log.md`.

No printed part: every board sits on nylon standoffs on the rear clear sheet, so nothing here goes through `solid_check`.

## What it is
| Block | Choice | Tag |
|---|---|---|
| Supply | Mean Well LRS-350-24 (on hand), 24 V 14.6 A, on the rear sheet (moved from the floor spot, see Layout); IEC C14 fused switched inlet; **set the PSU's 115 / 230 V selector to 115 V** | spec / DEFAULT |
| 24 V distribution | 6-way blade fuse box with negative bus: F1 10 A charger, F2 5 A motion (through the E-stop), F3 3 A keep-alive buck, F4 2 A logic buck, 2 spare | DEFAULT |
| E-stop | 22 mm mushroom, NC, in F2: cuts the SKR (steppers) and the servo buck (all 6 servos). Pi, keep-alive and charger stay on: the drone stays powered and the latch self-locks | DEFAULT |
| Keep-alive | XL4016-class CC/CV buck, **13.0 V CV, 3.0 A CC** (one setting for both drones: Scout ~8.5 W, Overseer ~24 W, E) → relay K1 → INA226 (0x41) → pogo + pad; pogo − pad = box GND | E / DEFAULT |
| Servo rail | XL4016-class buck, **6.0 V** for all 6 servos (4 hook-latch MG90S, DS3218, MG995) | DEFAULT |
| Logic rail | 24 → 5.1 V 5 A buck (Pololu D36V50F5 class): Pi on its 5 V pins, relay JD-VCC, the 5 hall switches | DEFAULT |
| Box computer | **Raspberry Pi 4 B 2 GB** + GPIO screw-terminal HAT: Klipper host + Moonraker + `boxd` (the swap state machine, Python) | DEFAULT |
| Steppers | BTT SKR Mini E3 V3 (on hand), Klipper MCU over USB: X = head A, Y = head B, Z = axle, E spare | DEFAULT |
| Servos | PCA9685 (I2C 0x40) on the Pi: ch 0..3 latch L1, L2, R1, R2; ch 4 gripper A (DS3218), ch 5 gripper B (MG995); 50 Hz | DEFAULT |
| Current sense | 3 x INA226 modules, shunt replaced by **R010 2512** (full scale 8.19 A, LSB 0.25 mA): keep-alive 0x41, bay S 0x44, bay O 0x45 | DEFAULT |
| Relays | one 4-ch 5 V opto board (SRD-05VDC-SL-C, 10 A 30 VDC): K1 keep-alive, K2 bay S, K3 bay O, K4 spare | DEFAULT |
| **Head brake (arm v6 / v7 requirement, 24 Oct)** | **2 DPDT 24 V relays KB1 / KB2 (Omron G2RL-2 DC24 class, 8 A contacts, spec), one per head motor, one pole per coil: de-energised (NC) each coil is shorted on itself, energised (NO) it is on its TMC2209; coils on the motion branch after the E-stop, switched by a bare ULN2003A from one SKR `output_pin` per head, sequenced against the driver enable** (section Head brake) | DEFAULT / CHECK |
| Charger | dual-channel DC charger on the 24 V bus, **iCharger 456DUO class (MODBUS over USB to the Pi, CHECK that the 456DUO speaks the 4010DUO map before buying)**, input limit 6.5 A, end 4.15 V per cell, starts below 35 °C | DEFAULT / CHECK |
| Live bays | 2 (Spark 16 Oct: one per drone size): **Scout bay at −90°** (the bottom module on the cradle), **Overseer bay at 180°** (reserved, wired when built); each bay's XT60E1-M (float carrier) through its relay | DEFAULT |

**Why 6.0 V and not 6.8 V for the servos:** the MG90S is rated 4.8..6.0 V (E); one rail at 6.0 V serves all six with one buck. The grippers need ~0.18 N·m (E, `dimensions.md` I XT60 mate) against ≥ 1.0 N·m available at 6.0 V (E), so the 6.8 V of `box-wiring.md` bought nothing. Alternative: 6.8 V gripper rail + 5 V latch rail (two bucks).

## Box computer I/O (Pi 4, BCM numbers)
| Pin | Use | Notes |
|---|---|---|
| GPIO 2 / 3 | I2C1 SDA / SCL, 100 kHz | PCA9685 0x40, INA226 0x41 / 0x44 / 0x45 (A0 / A1 straps); avoid 0x70 (PCA all-call) |
| GPIO 4 | PCA9685 OE | 10 kΩ pull-up to 3.3 V: servo outputs off until `boxd` pulls it low (no twitch at boot) |
| GPIO 5, 6, 7, 8 | relay IN1..IN4 (K1 keep-alive, K2 bay S, K3 bay O, K4 spare) | active low; these pins boot pulled up (relays off); SPI0 disabled; board VCC = Pi 3.3 V, JD-VCC = 5.1 V bus, JD jumper removed |
| GPIO 17, 27, 22, 23 | hook-latch halls L1, L2, R1, R2 | A3144 open collector on 5 V, 10 kΩ pull-up to 3.3 V at the HAT; **low = plunger fully out** |
| GPIO 24 | KW11 seated | NO to GND, 10 kΩ pull-up; low = seated; ignored from LATCH_OUT to LATCH_IN |
| GPIO 25 | INA226 0x41 ALERT | open drain, 10 kΩ pull-up; set at 2.5 A: fast trip for A3 |
| USB | SKR (Klipper), later the hub camera | |
Unit naming: L = the W −X seated face, R = +X; 1 = Y_C − 100 (stub side), 2 = Y_C + 100 (`dimensions.md` G Hook latch: drone ±y faces are W ∓X).

## SKR Mini E3 V3 (Klipper, `printer.cfg` sketch; pin names from Klipper's SKR Mini E3 V3 sample config, CHECK)
- `[printer] kinematics: none`; three `[manual_stepper]`: **head_a** on the X driver (PB13 / PB12 / PB14, UART addr 0), **head_b** on Y (PB10 / PB2 / PB11, addr 2), **axle** on Z (PB0 / PC5 / PB1, addr 1). TMC2209 UART PC11 / PC10.
- Heads: `rotation_distance` 50.27 (16T m1 pinion, `dimensions.md` I Rack and pinion v5), run current 2.0 A (XT60 insert), last 12 of an insert at ≤ 20 mm/s. Homing sensorless into a **park hard stop** (X / Y DIAG jumpers on, `endstop_pin: tmc2209_<stepper>:virtual_endstop`, 10 mm/s; the hard stop is the arm session's, PR "kept").
- Axle: `rotation_distance` 90 (degrees per motor turn at 4:1); home on the axle A3144 at Z-STOP (PC2, `^`, Z DIAG jumper off), then the detent picks the notch (`box-arm.md` Homing).
- Stall: TMC2209 StallGuard on X / Y reported through `[gcode_button]` on the DIAG pins during fast moves; `boxd` aborts (A5). StallGuard is unreliable at the slow insert speed, so the gate there is the outcome check (mate current, full stroke).
- Hold: `SET_STEPPER_ENABLE` off while idle (the detent holds the arm, `box-wiring.md`).

## Head brake (arm v6 / v7 requirement, hub 24 Oct)
**Why:** unpowered, a head carrying a cartridge outweighs the NEMA17's cogging and falls 77.6 (E, arm v7: head + carriage + cartridge onto the TPU bumper) with the driver off. With an Overseer cartridge that fall reads 2.43 on the 625ZZ (E), so the arm's gate is a passed bench drop before the first Overseer swap. With the Scout, the arm's fallback (sized for the fall, 0.83, E) holds. The numbers are in `dimensions.md` System box swap: arm (Head brake, Results; branch part/swap-arm-v7). With each coil shorted on itself, back-EMF brakes the fall to a creep: about 8 mm/s with a Scout head (391 g, E) and about 19..23 mm/s with an Overseer head (~930 g, E). That uses Kt 0.21 N·m/A and 1.4 Ω per coil, rising to ~1.75 Ω with the 26 AWG bundle, pigtails and contacts (E). It lands gently in both cases.
- **Contacts (2 relays, one DPDT per head motor, one pole per coil):** pole common = coil wire 1, NO = driver output 1, NC = coil wire 2. Coil wire 2 also stays wired to driver output 2. **De-energised: each coil is shorted on itself and driver output 1 is open, so the driver is never shorted.** Energised: the coil is on its driver as usual. KB1 is on head A, between the arm bundle and the SKR X header; KB2 is on head B, on the Y header. The axle motor has none, because the detent holds the arm (Hold).
- **Coils:** 24 V, ~17 mA each (E), from the motion branch after the E-stop (F2). Each is switched low-side by one channel of a **bare ULN2003A** (DIP-16 on a small perfboard, COM pin 9 to the 24 V motion branch for its flyback diodes, pin 8 to the SKR's GND). Do not use a 28BYJ-48 "ULN2003 module": it is rated 5..12 V and its LEDs on COM take ~22 mA each at 24 V (E). **Power off, E-stop, F2 open, a Klipper shutdown or an MCU reset leaves the input low or floating, the relay drops and the brake is on.**
- **Signal (its own pin, never the driver enable):** one free 3.3 V SKR pin per head (EXP1, which pin: CHECK on the SKR Mini E3 V3 pinout), as `[output_pin head_a_brake]` with `value: 0` and `shutdown_value: 0`. The order is fixed so that the relay never switches a live driver (Trinamic: never connect or disconnect a motor on an energised driver):
  - start: release the brake, dwell ≥ 30 ms, then `SET_STEPPER_ENABLE STEPPER=head_a ENABLE=1`, then move;
  - stop: disable the stepper, dwell ≥ 30 ms, then drop the brake pin.

  The dwell covers the G2RL's operate time (≤ 15 ms max, spec) and its release with the ULN's flyback diode (several times its bare ~5 ms, E). The head is unbraked only inside that dwell. `boxd` never moves a head with an implicit auto-enable: every head move goes through this sequence (DEFAULT).
- **Wiring:** the 8 head-motor wires come off the arm bundle at the rear sheet (26 AWG, `box-wiring.md`). Each coil-1 wire goes to its relay common; each coil-2 wire goes to both its relay's NC and its driver output 2. Short 22 AWG pigtails run from NO to the SKR's X / Y motor headers. The ULN board needs 5 wires: 24 V (COM + both relay coil ends), GND to the SKR, and 2 inputs (24 AWG, E). The board sits on 2 nylon standoffs beside the SKR (rerun the placement with the real size, Problem 5).
- **Bench drop test (current-parts Spark, Tests):** a head carrying a loaded **Overseer** cartridge (~930 g with head and carriage, E), driver off, brake on, released at the top of its stroke. It must creep (≤ ~25 mm/s, E: 77.6 in ≥ 3.1 s) and land without a bang. Repeat with the relays dropped by the E-stop. Then once with a Scout cartridge (391 g, ≤ ~10 mm/s, E).

## Keep-alive and mate check (no current is ever switched by the XT60 or the pogo pins)
1. K1 closes only after the latch is out, while the drone's old pack is above 13.0 V: 0 A through the pads (the drone's ORing diode is reverse biased).
2. The pull unmates the drone XT60; the pads take over: INA226 0x41 reads > 100 mA at the end of the pull stroke.
3. The new pack mates: the drone bus rises above 13.0 V and the box current falls: **< 50 mA within 1.0 s = mate** (`dimensions.md` I XT60 mate).
4. K1 opens at ~0 A; the pins are dead before the latch goes in and the drone lifts off.
**Pogo post (box chat's part):** 2 pads (+, −), 2 pins each in parallel (≥ 1.5 A per pin), **travel ≥ 8 (6 lift during the push + 2 preload, E)**, + on the INA226 output, − to the keep-alive buck's −. **Drone side (Scout v3, Overseer):** an ORing diode from the + pad into the bus (Scout Schottky PMEG4050EP class or LM74700; Overseer LM74700) and ~1.2 mF (Scout) / ~4.5 mF (Overseer) (research 2026-10-11, E); the drone must run from 12.6 V during the swap (13.0 less the Schottky).

## Live bays and charging
- Charger CHn + → relay Kn → INA226 (bay side) → 16 AWG → the bay's XT60E1-M +; **CHn − goes straight to the bay's XT60 −, never to box GND** (it would bypass the charger's own current shunt). The INA226 reads the bay's bus voltage through the charger's input ground: check with a meter that each CH − is within 1 Ω of the charger's input − (non-isolated, E, CHECK); if isolated, tie each CH − to the GND bus at one point.
- **A bay mates and unmates only with its relay open:** before any head move into a live bay, `boxd` stops the channel, waits for I < 50 mA, opens the relay (A9 interlock). After a store, **bay mate = bus voltage 13.0..17.0 V with the relay open** (the INA226 bus input, ~830 kΩ, is the only load). The relay then closes onto the idle charger (inrush into the charger's output capacitors on the relay contacts, never on the XT60) and the channel starts.
- Charge end: the charger's done flag, or current under C/10 with a rest voltage ≥ 4.15 V per cell read through the open relay. Scout bay 2C (3.0 A), Overseer bay ~1C (6.0 A) under the 6.5 A input limit (E).
- The other bays hold bench-charged packs. Rotation rule (DEFAULT): take the fullest pack; the dead pack goes to the live bay when it is free, else to the free bay, and the arm shuffles the live bay while the drone flies.

## Layout (rear sheet, `box_electronics.py` search)
Everything electrical sits on the **rear clear sheet's inner face (Y −66)** and the floor behind the stub, in Y −66..−2: the stub face is Y 0 and the motor bodies start at Y 2 (`dimensions.md` I), so nothing reaches the arm sweep. Boards on **10 mm nylon M3 standoffs**, holes drilled in place through each board's own holes (≥ 15 from a sheet edge, as every sheet hole). Wire the whole sheet flat on the bench, then stand it in; only the arm bundle, the latch leads, the post lead and the bay leads cross to the rest of the box.
- **PSU on the sheet:** X 42..157, Z −170..45, its bottom face on the sheet with 4 x M4 x 6 through the 3.0 sheet (3.0 thread engagement = the spec max), terminal end up, air out the bottom end 31 over the floor. **The floor spot in `box-stand.md` (X 25..89, Y −50..−10, kb keep-out) is 64 x 40 and cannot take a 215 x 115 x 30 PSU**; it is released (stand part: "kept").
- **Charger** standing on the floor against the sheet, X −162..−44, Z −200..−29, Y −66..−7 (5 to the stub face), strapped through 2 sheet holes. IEC inlet over the PSU (X 78..128, Z 56..86, cut-out 48 x 28 E); E-stop X 112..152, Z 188..228 (hole 22.5).
- Keep-out: mast + kb X ±40, Z ≤ 50 (mast X ±30, head top Z 40, + 10). The arm bundle leaves the stub v3 lead channel at X −27 and drops beside the mast to the SKR.
- **Search:** 12 boards placed by simulated annealing over centres (2.5 grid) and 90° turns, legal moves only (sheet X ±160, Z −195..232, 6 board-to-board, keep-outs), objective Σ conductors x (0.25 + I cont) x route length; 12 seeds, best taken (`result.json` `search`). Result (centres, W X / Z): SKR (−100, 12.5) next to the bundle; relay (−80, 92.5), INA226s (10, 95), (−25, 87.5), (−135, 85); fuse box (97.5, 122.5) over the PSU; keep-alive buck (65, 182.5); servo buck (−10, 137.5); Wago bus (−10, 215); PCA9685 (−10, 180); 5 V buck (−67.5, 180); Pi (−115, 180). Closest pair 6.3 (PCA9685 to servo buck, C); every board inside Y −2 (C).

## Nets, gauges, drops (`result.json` `nets`; E resistance and ampacity tables in the script)
| Net | Wire | Peak A | Drop at peak (C on E lengths) |
|---|---|---|---|
| Mains (IEC → PSU) | 3 x 18 AWG, fork crimps, PE to the PSU FG | 6.8 (115 VAC, spec) | |
| PSU → fuse box | 2 x 14 AWG | 11.9 | 0.16 % |
| F1 → charger | 2 x 16 AWG (charger input plug CHECK) | 6.5 | 0.33 % |
| F2 → E-stop → SKR, servo buck | 2 x 18 AWG | 5.2 | ≤ 0.23 % |
| F3 → keep-alive buck; F4 → logic buck | 2 x 18 / 2 x 20 AWG | 1.8 / 1.0 | ≤ 0.06 % |
| Keep-alive → K1 → INA → pogo +, pogo − | 18 AWG | 3.0 | 0.18 + 0.19 % |
| Charger → K2 / K3 → INA → bay XT60, CH − → bay | 16 AWG | 6.5 | ≤ 0.31 % |
| 6.0 V servo bus; → PCA9685 V+ | 2 x 18 AWG | 8.0 / 3.2 | 0.43 % |
| 6.0 V gripper pair (in the twist) | 2 x 22 AWG | 5.0 (both stalled) | 3.7 % (servo rail rule 10 %) |
| Latch servos (PCA ch 0..3 → unit) | 3 x 22 AWG servo extension, stock plug at the servo | 0.8 | ≤ 0.74 % |
| Latch halls, KW11, axle hall, PWM | 3 / 2 x 28 AWG | signal | |
| Steppers (SKR → motor, through the twist) | 4 x 26 AWG each (`box-wiring.md`) | 2.0 | 1.0 % |
| 5.1 V → Pi pins 2 / 4, 6 / 9; → 5 V bus | 2 x 20 AWG | 3.5 | 0.22 % |
| I2C chain Pi → PCA → INA x3; relay IN | 4 / 5 x 26 AWG | signal | |
| GND bus → fuse box negative bus (star) | 18 AWG | 8.0 | |
Every net is inside its E ampacity (`result.json` `checks`).

## Connectors
| Where | Connector | Qty |
|---|---|---|
| Mains | IEC C14 fused switched inlet + cord; fork crimps M3.5 on the PSU (3 mains, 2 x 14 AWG out) | 1, 5 |
| Bays | XT60E1-M panel mount on each live bay's float carrier, 16 AWG soldered (the drone receiver's is the Scout's) | 2 |
| Bay leads at the sheet | XT60 pair inline (sheet side female, bay side male) so the sheet unplugs | 2 |
| Pogo post | XT30 pair (keep-alive), JST-XH 2-pin (KW11) inline at the sheet | 1 + 1 |
| Latch units | servo plug at the PCA9685 header (22 AWG extension), hall soldered + heat-shrink at the housing, ferrules at the HAT | 4 + 4 |
| Steppers | JST-XH 4-pin crimped at the SKR end (motor ends keep their pigtails, `box-wiring.md`) | 3 |
| Axle hall | JST-XH 3-pin at Z-STOP (VCC from the 5 V bus) | 1 |
| Gripper servos | signal + GND into PCA ch 4 / 5 (3-pin housing, V+ pin empty); power pair into the 6.0 V Wago | 2 |
| Buses | Wago 221-415 (6.0 V, 5.1 V, GND) | 3 |
| Screw terminals (fuse box, bucks, relay, INA226, HAT) | bootlace ferrules on every stranded end | ~60 |
| Pi → SKR | USB-A to USB-C, 0.3 m | 1 |

## Cut list (sheet side; +20 % and 2 x 20 strip, E; arm side in `box-wiring.md`)
- **14 AWG** red / black: PSU → fuse box 2 x 275.
- **16 AWG** red / black: F1 → charger 2 x 590; CH1 + → K2 340, K2 → INA 115, INA → bay S 780, CH1 − → bay S 605; CH2 + → K3 340, K3 → INA 115, INA → bay O 510, CH2 − → bay O 560.
- **18 AWG** red / black: F2 → E-stop 2 x 200; E-stop → SKR 2 x 570; E-stop → servo buck 2 x 310; F3 → keep-alive buck 2 x 155; buck → K1 325, K1 → INA 155, INA → pogo + 480, buck − → pogo − 520; servo buck → bus 2 x 135; bus → PCA V+ 2 x 85; GND bus → star 280; mains 3 x 205 (3-core cord).
- **20 AWG**: F4 → 5 V buck 2 x 310; 5 V buck → Pi 2 x 100; → 5 V bus 2 x 155.
- **22 AWG**: latch servo extensions 3-core L1 410, L2 650, R1 430, R2 670; relay JD-VCC 2 x 275.
- **26 AWG**: relay IN + VCC 5 x 190; I2C 4-core 170 + 170 + 95 + 175; INA 0x41 ALERT to GPIO 25, 1 x 300 (E, laid with the I2C chain).
- **28 AWG**: latch halls 3-core L1 280, L2 520, R1 560, R2 800; KW11 2 x 675.
- **Arm bundle fixed tails (stub exit → board), now from the layout:** steppers and axle hall 164, servo pair 311, servo signals 276 (C on E positions; was 250 E). Totals per gauge, every net end to end incl. the bundle: 14 AWG 0.6 m, 16 AWG 4.5 m, 18 AWG 5.3 m, 20 AWG 1.1 m, 22 AWG 8.1 m, 26 AWG 10.4 m, 28 AWG 10.5 m (`result.json` `wire_totals_mm_by_awg`).

## Power budget (E; `result.json` `budget`)
| Load | Rail | W cont | W peak |
|---|---|---|---|
| Pi 4 + HAT + a later USB camera | 5.1 V | 6.0 | 9.0 |
| relay coils, halls, PCA, INA logic | 5.1 V | 0.9 | 1.6 |
| SKR + 3 NEMA17 at 2 A (drivers off while idle) | 24 V | 1.5 | 30 |
| 2 gripper + 4 latch servos (budgeted together) | 6.0 V | 1.8 | 49 |
| keep-alive (CC 3.0 A at 13.0 V) | 13.0 V | 8.5 | 39 |
| charger, input limit 6.5 A | 24 V | 156 | 156 |
| **at the PSU (bucks 90 %)** | 24 V | **177** | **296 = 85 % of 350 W (12.3 A)** |
The research's ~600 W of nonstop 2C charging does not fit the LRS-350; the 6.5 A charger input limit keeps the box under it with the Overseer bay at ~1C. Alternative: an LRS-600-24 for the charger alone (research 2026-10-11).

## Swap state machine (`boxd`; DEFAULT, times E; `states.svg`)
| State | Action | Pass | Limit | Fail |
|---|---|---|---|---|
| IDLE | drone away; live bays charge; heads parked; K1 open; latch in | swap request from the mission script | | |
| PRESTAGE | head A picks the fullest charged pack (its bay relay open, I < 50 mA first); head B waits open at 81 under the seat | both heads at pose, no stall | 6 s | A5 |
| WAIT_SEAT | drone lands under its own landing mode, kill at first touch | KW11 low 100 ms | 30 s | mission retries |
| LATCH_OUT | PCA ch 0..3 to OUT (per-unit pulse from calibration); KW11 ignored from here | all 4 halls low | 0.5 s | A1 |
| KA_ON | close K1 (old pack higher: 0 A) | bus 12.8..13.2 V, I < 0.5 A for 0.2 s | 0.5 s | A3 |
| PULL | head B rises to 89.6, closes, retracts with the dead pack (XT60 unmates) | full stroke, no stall, I > 100 mA at its end | 2 s | A4 |
| TURN | axle 90°: head A to the top | no stall, hall matches phi | 0.6 s | A5 |
| INSERT | head A extends, last 12 at ≤ 20 mm/s, to the bolt seat, 1 over-travel into the TPU strip | full stroke | 2 s | A5 |
| MATE_CHECK | hold the stroke, read INA 0x41 | I < 50 mA | 1.0 s | A6 |
| RELEASE | gripper opens, head A to park | park, no stall | 1 s | A5 |
| KA_OFF | open K1 at ~0 A | I < 50 mA before, bus < 1 V after | 0.3 s | A3 |
| LATCH_IN | PCA ch 0..3 to IN | all 4 halls high | 0.5 s | A11 |
| READY | SWAP_DONE to the mission (it arms the drone); take-off | KW11 high | 60 s | mission decides |
| STORE | axle to the free bay (the live bay when free); head B inserts the dead pack, bay relay open | full stroke | 4 s | A5 |
| BAY_MATE | read the bay INA bus voltage, relay open | 13.0..17.0 V | 1.0 s | A7 |
| CHARGE | close the relay at 0 A, start the channel | done flag, or I < C/10 and rest ≥ 4.15 V/cell | | A8 |
| CHARGE_END | stop the channel, wait I < 50 mA, open the relay | relay open | 5 s | A8 |
The box never arms or disarms the drone: it disarms on the kill and the mission script arms it after SWAP_DONE. The swap request comes from the mission script over the network (Moonraker JSON-RPC, DEFAULT).

## Aborts
| Abort | Trigger | Action |
|---|---|---|
| A1 latch not out | a hall still high 0.5 s after OUT | all 4 IN, OUT once more; second fail: IN, SWAP_FAIL, the drone keeps its pack (nothing has moved) |
| A2 latch lost mid-swap | a hall goes high between LATCH_OUT and LATCH_IN | stop all motion, K1 as it is, alarm (the plungers self-lock under load) |
| A3 keep-alive fault | bus outside 12.8..13.2 V, or I > 2.5 A for 0.2 s (ALERT) with the old pack in, or K1 will not open at 0 A | open K1; before PULL: SWAP_FAIL with the old pack in; after PULL: alarm, K1 never opens while the drone runs on it |
| A4 pull / keep-alive lost | I < 100 mA at the end of the pull | the pack did not come out or the pads did not make: head B re-inserts what it holds, mate check, SWAP_FAIL, alarm |
| A5 stall | TMC2209 DIAG on a fast head move, Klipper shutdown, E-stop, or the axle hall out of its window | stop all motion, latch stays out, K1 as it is (the drone stays powered), alarm; Spark clears by hand, heads re-home before the next cycle |
| A6 mate fail | I ≥ 50 mA 1.0 s after the full stroke | back off 5, push again once at 10 mm/s; still ≥ 50 mA: pull the pack back into head A, the drone stays on keep-alive (no time limit), alarm |
| A7 bay mate fail | bay bus not 13.0..17.0 V with the relay open | re-insert once; then leave the pack uncharged, alarm (off the flight path) |
| A8 charge fault | charger error, bay I > 7 A, pack over 45 °C (charger probe, CHECK), or no current 5 s after the start | stop the channel, wait I < 50 mA, open the relay, mark the pack |
| A9 bay interlock | a head move into a live bay with its relay closed or I ≥ 50 mA | refused before the move |
| A10 seat lost | KW11 high before LATCH_OUT | back to WAIT_SEAT |
| A11 latch stuck out | a hall still low 0.5 s after IN | OUT, IN once more; then alarm, no SWAP_DONE: the drone sits disarmed on its new pack |

## Interfaces other parts use (`dimensions.md` I "Box electronics v1")
- Pogo post (box chat): 2 pads x 2 pins, ≥ 1.5 A per pin, travel ≥ 8 (E), 13.0 V CC 3.0 A behind K1; lead 4 conductors (2 x 18, 2 x 28 AWG) from the sheet top.
- Drone (Scout v3, Overseer): ORing diode + bulk capacitance on the + pad; runs from 12.6 V during a swap; pads dead unless latched.
- Arm (v5 rails): a park hard stop per head for sensorless homing; drivers at 2.0 A.
- Hook latch: per unit one 3-core 22 AWG servo extension (stock plug) + one 3-core 28 AWG hall lead; plunger OUT = hall low.
- Stand: the electronics own Y −66..−2 behind the stub on the rear sheet; the PSU leaves the kb floor spot; 4 x M4 + standoff holes drilled in the rear sheet in place; charger strap holes.
- Bays: Scout live bay −90°, Overseer live bay 180°; XT60E1-M + 16 AWG pair per live bay.

## Problems
1. **Driver heat in a closed box:** TMC2209 at 2.0 A on the SKR without a fan. The detent keeps the drivers off between moves, so the duty is low (E). Check the driver temperature in the first soak test; fix = the priced 24 V 4010 fan on SKR FAN1 (`controller_fan`) over the drivers on 2 standoffs.
2. **Charger protocol (CHECK before buying):** the MODBUS map names only the 4010DUO (sold out). Without a protocol, the alternative (2 x ISDT PD60 on USB-C bricks, or a ToolkitRC M8D) charges but someone presses start; the box still switches relays and reads every number it needs through the INA226s.
3. **Charge without balance leads:** `dimensions.md` I Balance charging drops the balance lead; most balance chargers refuse a LiPo program without it. CHECK the charger's non-balance LiPo mode (iCharger: yes per its menu, E); else the bays need the balance path back (Spark).
4. **Pogo pins with ≥ 8 travel** (Q20): not chosen; a long-stroke spring probe rated ≥ 1.5 A, or short pins on a sprung post carriage (the post's design).
5. **Board sizes E** (relay, INA226, bucks, fuse box, Wago strip): the placement keeps 6 between boards; re-run `box_electronics.py` with the real sizes when they arrive (a minute), then drill.
6. **Route lengths are E** off the sheet (latch, post, bays): cut long, trim at fit.
7. **PSU on 3.0 acrylic:** 0.76 kg on 4 x M4 with 3 of thread; add 2 fender washers outside (E). The 115 / 230 V selector: CHECK before first power.
8. **Charger output and input grounds:** measure (Live bays section) before wiring the bay − leads.
9. **Head brake pins (CHECK):** which free SKR Mini E3 V3 pins take the two brake `output_pin`s (EXP1); and `boxd` must route every head move through the brake sequence (an implicit Klipper auto-enable would energise the driver with the brake on: harmless to the driver, since its outputs are open on NO, but the head would not move).

## Status
24 Oct (box structure system session, hub ask): head brake added (section Head brake): 2 DPDT relays, bare ULN2003A, its own SKR pin per head sequenced against the enable; bench drop with the Overseer cartridge in the Spark Tests line.
24 Oct: v1 designed (docs + `box_electronics.py` + `runs/box_electronics/v1/`). Next: Spark buys (`parts-list.md` Box, status to buy); bench-wire the rear sheet; first power with no loads (rails 13.0 / 6.0 / 5.1, K1 at 0 A); latch calibration (OUT / IN pulse per unit until its hall reads); mate check on the bench with a pack and the drone receiver.
