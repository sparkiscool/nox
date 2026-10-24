# Nox: parts list (22 Oct 2026)

Every part of the box, the Scout and the Overseer: what it is, how many, where it stands. One list, so a new mount or an order starts here. **Numbers are not repeated here:** body sizes and mounting holes are in `nox/dimensions.md` (section M for bought parts, the other sections for our own geometry), and the live state of every printed part is only in `nox/current-parts.md`. Sources for section M: `claude/research/2026-10-22-mounting-dimensions.md`.

**Status words:** **on hand** (in the docs as owned or built in); **ordered** (in the 15 Oct cart, arrival not recorded: CHECK); **to buy** (needed by a current design, not ordered); **priced** (on the later list, not needed yet); **CHECK** (the docs do not say). Scout v2 printed parts are retired (`archive/scout-v2/`); Scout v3 has no printed parts yet.

## Scout (aircraft; carried into v3)
| Part | Qty | Role | Status | Mount data |
|---|---|---|---|---|
| SpeedyBee Bee35 frame, aluminium bottom plate (Pro), ducts off | 1 | airframe, 153 wheelbase | on hand | M: Bee35 |
| EMAX ECO II 2004 motor (KV not recorded) | 4 | propulsion | on hand | M: ECO II 2004 |
| Gemfan Hurricane 3520 prop, 3 blade, T-mount | 4 + spares | propulsion, props on top | on hand | M: 3520 |
| MicoAir743v2-AIO-35A (FC + 35 A ESC) | 1 | flight controller | on hand | M, H |
| Grommets for the FC | 4 | soft mount (Spark 14 Oct) | CHECK (size not published) | M |
| RadioMaster XR2 (Nano) ELRS 2.4 | 1 | RC receiver, on the GPS port | on hand | M |
| MicoAir MTF-02P | 1 | optical flow + ToF | on hand | M, H |
| Radxa Zero 3W, RS107-D2E0H0W15 (2 GB, no eMMC, no header) | 1 | companion computer | on hand | M, H |
| microSD ≥ 32 GB A1 | 1 | Radxa boot | CHECK | |
| DECXIN 2633V2 USB global-shutter camera, 102° | 1 | camera | on hand | A (P); measure lens |
| M2.5 heat-set inserts for the camera (OD 3.5 x 5.7 E, Ø3.2 hole) | 4 | camera mount (Spark 23 Oct: the board takes M2.5) | CHECK | A, M |
| 4S 1500 LiPo, XT60 | 1 + CHECK spares | flight pack | on hand | A (P) |
| 220 µF 35 V capacitor | 1 | on the battery pads | on hand | |
| 5 V 5 A UBEC | 1 | Radxa + camera power | on hand | M (typical; measure) |
| SH1.0 4-pin pigtail | 1 | Radxa UART to FC UART4 | to buy (CHECK) | |
| 90° USB-C plug or short cable | 1 | camera to Radxa | to buy (CHECK) | |
| Carbon rod Ø3.0 | per cage design | cage struts, wheel pins (decision 22) | on hand (length CHECK) | G |
| Catch hardware for the box (hold plate, charging contacts) | 1 set | box catch | to design with v3 | G |
| M2 / M3 screws, nylocs, standoffs | as built | stack, plate bolts | on hand | H |

## Overseer (development starting)
| Part | Qty | Role | Status | Mount data |
|---|---|---|---|---|
| S500 frame, PCB version with carbon fibre landing gear, 2015 edition (Spark 23 Oct, AliExpress item 2251832276919319; 500 wheelbase, listing; arms replaced by printed ones) | 1 | airframe | on hand | M: S500, O |
| SunnySky X2216 II 880 KV | 4 | propulsion | on hand | M: X2216 |
| 1045 prop | 4 + spares | propulsion | on hand | M: 1045 |
| ESCs | 4 | motor drive | CHECK (not recorded) | |
| MicoAir743-Lite FC | 1 | flight controller | on hand | M, O |
| Matek FCHUB-12S V2 PDB | 1 | power distribution | on hand | M, O |
| 4S 6000 LiPo, XT60 | 3 | flight pack | on hand | O, M |
| MicoAir MTF-02P | 1 | optical flow + ToF | on hand | M, O |
| Unitree L2 4D lidar | 1 | mapping, at the nose | on hand | M: L2, O |
| DECXIN 2633V2 camera | 2 | front (lidar colouring by yaw) + underside | 1 on hand, 1 to buy | A (P), O |
| RC receiver | 1 | RC (CRSF) | CHECK (model not recorded) | |
| Belly frame, 8 PETG wheels, pogo pads (no steel plate since 23 Oct) | 1 set | box catch (section G) | to design | G |

## Scan stick (hand-held L2 scanner, 23 Oct; `nox/overseer/scan_stick/scan_stick.md`)
Shares the Overseer's L2, camera and 4S 6000 pack (above); the Pi 5 is the Overseer's companion computer (resume, P).
| Part | Qty | Role | Status | Mount data |
|---|---|---|---|---|
| EXTEND-A-REACH 1.5-3 ft extension pole, 3/4-5 ACME tip (B08LL28XZW) | 1 | the stick; the unit screws onto its tip | on hand (Spark 23 Oct) | S |
| Raspberry Pi 5 + microSD ≥ 64 GB (E: ~1 GB/min) | 1 | recorder | on hand (resume) | M: Pi 5 (58 x 49, Ø2.7) |
| 5 V 5 A UBEC | 1 | Pi + camera, taped to the shelf side (Spark 23 Oct) | CHECK (box has one) | M: UBEC |
| 12 V 2 A buck (Pololu D24V22F12 class) | 1 | L2 supply | to buy | S |
| KCD1 rocker switch | 1 | main power | on hand (Parts & Resource sheet) | S |
| 12 mm momentary button + 5 mm LED + 330 Ω (typical) | 1 + 1 | record start / stop | to buy | S |
| 20 mm hook-and-loop strap, 450 mm (E) | 2 | pack | to buy | S |
| M3 heat-set inserts (Ruthex 5.7 long, Ø4.0 hole, 7 deep) | 2 | camera bracket to the head | on hand | M: inserts |
| M4 heat-set inserts (Ruthex RX-M4x8.1, Ø5.6 hole, 9.5 deep) + M4 x 12 | 4 + 4 | head to base flange | CHECK | S, M: inserts |
| M3 x 8, M3 x 12, M2.5 x 6, M2.5 x 6 | 4, 2, 4, 4 | L2 (3 under the head + 5 in the L2), camera bracket (6 pad + 6 in the insert, v7), camera (1.6 board + 4.4 in the post insert), Pi | on hand / CHECK | S |
| Cat5e 0.5 m, XT60 pigtail, DC 3.5 x 1.35 plug lead | 1 each | wiring | CHECK | |

Printed (state only in `nox/current-parts.md`): base, head, cam (nox/topo parts) and the thread test ring; files in the current-parts rows.

## Box: bought parts
| Part | Qty | Role | Status | Mount data |
|---|---|---|---|---|
| NEMA 17 stepper | 3 | axle + 2 arm heads | on hand | M: NEMA 17 |
| BTT SKR Mini E3 V3 | 1 | stepper + servo control | on hand | M: SKR |
| Mean Well LRS-350-24 | 1 | 24 V supply | on hand | M: LRS-350 |
| 5 V buck (servo supply) | 1 | servos, logic | on hand | CHECK size |
| DS3218 20 kg servo | 1 | gripper 1 | on hand | M: DS3218 |
| MG995 servo (2-pack) | 1 | gripper 2 | ordered | M: MG995 (measure ears) |
| 6010-2RS bearing | 2 | hub on the stub | on hand | M: 6010 |
| 2040 extrusion | per stand | arm bars, stand | on hand | M: extrusion |
| 2020 brackets, M5 T-nuts | as needed (8 T-nuts for the arm) | extrusion joints | on hand | M: extrusion |
| GT2 80T pulley, 8 bore | 1 | axle drive, 4:1 | ordered | M: GT2 80T |
| GT2 20T pulley, 5 bore (kit) | CHECK | axle + head drives | on hand | M: GT2 20T |
| GT2 6 mm open belt, 2 m | 1 | 2 head belts + spliced axle loop | ordered | box-arm.md |
| GT2 belt connectors | per head | belt clamp (decision 27) | on hand | box-arm.md |
| GT2 toothless idler (bore 3 or 5) | 2 | head belt turnarounds (v4 demo; out in arm v5) | on hand (kit; bore CHECK) | M: idler |
| 625ZZ bearing 5 x 16 x 5 | 12 | arm v5 carriage V-wheels (2 per wheel) | to buy | box-arm.md v5 |
| M5 eccentric spacer, 6 tall, 0.8 throw (openbuilds type) | 2 | arm v5 carriage preload (one per head) | to buy (or a printed eccentric bush) | box-arm.md v5 |
| M5 x 25 socket head + nyloc | 6 | arm v5 wheel axles | CHECK on hand | box-arm.md v5 |
| Bay contacts: copper-clad sheet, AA spring contacts | 0 | retired 23 Oct (XT60 back) | on hand, not used | |
| XT60E1-M panel mount (the half that mates the packs' plug; gender CHECK, D17) | 3 now (drone receiver + 2 live bays; +1 per later bay) | swap and bay power, on the float carrier | to buy | M: XT60; dimensions I "XT60 mate" |
| MG90S micro servo (hook latch) | 4 | hook latch plungers | to buy (CHECK: 2 may be on hand, D9) | D18; `box/box-hook-latch.md` |
| M2 x 12 screws + nuts | 12 (8 ear screws + nuts, 4 yoke pins) | hook latch | CHECK on hand | `box/box-hook-latch.md` |
| M3 x 10 + M3 x 5.7 heat-set inserts | 16 + 16 | hook latch housings on the panel v7 bosses | on hand (CHECK count) | M: inserts |
| A3144 hall switch + Ø3 x 2 magnet | 4 + 4 | hook latch: plunger fully out | to buy (A3144 pack priced below; Ø3 x 2 magnets CHECK) | `box/box-hook-latch.md` |
| KW11 microswitch | per bay (CHECK) | drone seated | to buy (CHECK) | M: KW11 |
| Pogo pins (Mill-Max 0854 class) | per bay (CHECK) | keep-alive power | to buy (CHECK) | M: pogo pin |
| Endstops | CHECK | homing (or a hard stop, DEFAULT) | on hand | |
| Clear sheet, 3.0 (enclosure) | 4 | stand v7 walls | CHECK | box-stand.md |
| Brass heat-set inserts M3 / M5 | as drawn | printed-part threads | on hand | M: inserts |
| M3 / M5 screws and nuts (M4 for the PSU: CHECK) | per joint | assembly | on hand | box-stand.md Parts, box-arm.md |
| Wire: 26 / 28 / 22 AWG | ~7.8 m / ~2.8 m / ~0.9 m | arm harness | on hand | box-wiring.md cut list |
| Taidacent 12-wire slip ring | 1 | (retired: bounded-travel harness instead) | ordered, not used | |

## Box: priced for later (not ordered)
| Part | Qty | Role | Mount data |
|---|---|---|---|
| A3144 hall module | 1 pack | axle index | M: A3144 |
| 5 x 2 magnets | 1 pack | axle index | |
| M6 ball plungers | 1 pack | axle detent | |
| 24 V 4010 fan | 1 | cooling | M: 4010 |
| ISDT PD60 charger (listed as PD60S) + 65 W USB-C brick | 1 + 1 | balance charging | M: PD60 |
| 8-channel 5 V relay board | 7 | bay switching | CHECK size |
| JST-XH 4S balance pigtails, 5-pack | 2 | bay balance leads | M: JST-XH |
| Hub USB camera, box Raspberry Pi | 1 + 1 | vision, box computer | CHECK model |

## Box: printed parts (state only in `nox/current-parts.md`)
| Part | Qty | Print file |
|---|---|---|
| Funnel panels A + B | 8 (A / B split in `box-funnel.md`) | not in `print/` |
| Funnel hub + corner blocks | 1 + CHECK | design |
| Bay module v4i | 2 | `print/box_baymod_v4i.stl` |
| Hub v5 | 1 | `print/box_hub_v5.stl` |
| Sun post v1 | 1 | `print/box_sun_post_v1.stl` |
| Stub v3 | 1 | `print/box_stub_v3.stl` |
| Detent ring v1 + roller arm v1 | 1 + 1 | `print/box_detent_ring_v1.stl`, `print/box_detent_arm_v1.stl` |
| Idler ends A / B v3 | 1 + 1 | `print/box_idler_end_A_v3.stl`, `print/box_idler_end_B_v3.stl` |
| Pinion v4 | 2 | `print/box_pinion_v4.stl` |
| Fingers v4 (A and B, for heads A and B) | 4 | `print/box_finger_*_v4.stl` |
| Arm heads A / B v5 (v5b with the connector pad) | 1 + 1 | not in `print/` yet |
| Hook latch housing A / B v1, plunger v1 | 2 + 2, 4 | `print/box_hook_latch_housing_A_v1.stl`, `..._B_v1.stl`, `print/box_hook_latch_plunger_v1.stl` |
| Arm v5: rail A / B v1, V-wheel v1 (x6), pinion 16T v1 (x2), hub v6, heads A / B v6 with the carriages | 2, 6, 2, 1, 2 | design (`box-arm.md` v5); replaces the 2040 bars, idler ends, glides, belt clamp |
| Belt clamp coupon set v2 | 1 (on hold, decision 27) | `print/belt_clamp_coupon_set_v2.stl` |
| Mast v6 | 1 | `print/box_mast_v6.stl` |
| KB v7b | 1 | `print/box_kb_v7b.stl` |
| Cradle + front keel v6 | 1 | `print/box_cradle_keel_v6.stl` |
| Cross bar half v8 | 2 | `print/box_bar_half_v8.stl` |
| Toe v7b | 1 | `print/box_toe_v7b.stl` |
| Ring frame mid v6 | 5 | `print/box_frame_mid_v6.stl` |
| Ring frame ends L / R v7b | 1 + 1 | `print/box_frame_endL_v7b.stl`, `print/box_frame_endR_v7b.stl` |
| Corner post segment v7 | 12 | `print/box_post_seg_v7.stl` |
| Ring-to-wall brackets jA v8, jB v7, e v7c | 6 in all (split in `box-stand.md`) | `print/box_bracket_jA_v8.stl`, `print/box_bracket_jB_v7.stl`, `print/box_bracket_e_v7c.stl` |
| Arm harness: wire grommet, end clips, slot clips | 1, 2, 6 | in the wiring chat (`nox/box/wire_parts.py`) |

## Gaps to close (measure or ask Spark)
Dimension gaps (sizes, holes, threads) are the open questions Q1..Q20 at the end of `dimensions.md` M: settled when a mount for that part is designed. The gaps below are about what is on hand.
- Overseer: S500 variant, ESC model, RC receiver model.
- Scout: motor KV, number of spare packs, carbon rod stock, grommet size.
- Box: KW11 and pogo pin counts per bay, 5 V buck and relay board models.
