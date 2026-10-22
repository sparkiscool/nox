# Scout: aircraft and bring-up

Numbers: `nox-dimensions.md` §1-3. Battery parts: `scout-cartridge-pocket.md`. Cage: `scout-cage.md`. **Unverified** = not confirmed on the aircraft.

## What it is for
- Close-up inspection footage where the Overseer's overhead view can't see (under tables, behind racks), live to the operator through the box.
- v1: flies in the fleet, can be taken over by hand, delivers live video. No onboard autonomy.
- Operator gives rough directional commands; the Overseer watches and can refuse a move. ELRS stays as manual override and kill.
- AprilTag stickers on the Scout, read from outside. Demo: video plus bench demos, most likely no live flight.

## As built
| Item | As built |
|---|---|
| Frame | SpeedyBee Bee35, aluminium bottom plate, 153. Pusher (motors inverted, props underneath). Ducts off |
| FC | MicoAir743v2-AIO-35A (FC + Bluejay ESC), mounted upside down. **Port sides (MicoAir ports diagram, read 17 Oct):** MCU side: GPS/I2C 6-pin SH (XR2), DJI 6-pin SH, USB-C, TF slot, BOOT. ESC side (faces up toward the plate as mounted): UART1/UART6 6-pin SH (MTF-02P), **UART4 4-pin SH1.0 next to the USB-C, pins RX4 / TX4 / 5V / GND**, T5/R5 pads (UART5 = TELEM3), T8/R8 pads (UART8 = TELEM4), 5V / G / 12V pads, SWD |
| Motors | EMAX ECO II 2004, 1.5 shaft, T-mount (two M2). KV not recorded |
| Props | Gemfan 3520, 3.5 in, 3 blade (P, Spark 11 Oct) (replacement set; first set destroyed 1 Oct) |
| Receiver | RadioMaster XR2 (ELRS 2.4) on the rear TPU mount, on the **GPS port (UART3)**. RC port (RX6) has a hardwired SBUS inverter that corrupts ELRS: not used |
| Flow + range | MicoAir MTF-02P on TELEM1 (UART1), re-pinned 4-to-6 cable. Lens to the nose, ToF windows to the tail (180° from the Overseer). Goes in underside v5 (printed, not yet fitted): lens at (1.9, 0), 22.5 below the plate |
| Companion | Radxa Zero 3W, **model RS107-D2E0H0W15, rev V1.12J (P, box label, 7 Oct): 2 GB, no eMMC (P, board photo 7 Oct: eMMC footprint on the underside is bare pads; boots from microSD only; card needed, >= 32 GB A1), no GPIO header (wires soldered to the header holes), WiFi 6 / BT 5.4**, on TELEM2 (UART4), uXRCE-DDS 921600. **Board layout (P, silkscreen on the underside, 7 Oct):** along the connector edge: micro HDMI at one end, then the **USB3.0 host** port in the middle, then the **USB_OTG / 5V IN** port at the far end; microSD (TF) slot on the underside at the OTG end, card goes in from the short edge; CSI connector on the short edge at the HDMI end; u.FL at the HDMI end on the top. **Location (Spark, 5 Oct 19:20): in the pocket gap under the cartridge**, envelope in `nox-dimensions.md` §1. **Being installed now (Spark, 7 Oct 00:51)**; wiring plan below |
| Camera | DECXIN 2633V2 (USB, global shutter, 102°), on the pocket nose |
| Power | 4S 1500 (~165 g), XT60 pigtail, 220 µF 35 V cap on the battery pads |
| Size / mass | Size and mass: `nox-dimensions.md` §1 (~197 x 197 with props, ~78 tall, 400 g with battery and no cage, P). All-up 610 g with everything fitted, battery in (P, Spark 7 Oct 22:55; taken as cage + Radxa + BEC on, CHECK). Hover time on the 4S 1500 ~7 min to 20 % reserve (E, not measured: time a 2 min hover from a full pack and read the mAh used) |

## Final wiring (plan, 5 Oct; DEFAULTs until built; **wiring started 17 Oct**)
| From | To | Wires | Notes |
|---|---|---|---|
| Radxa UART3 (header pins 5 TX, 3 RX, 6 GND; UART3_M0, enable overlay `rk3568-uart3-m0` in rsetup) | **FC UART4 socket (4-pin SH1.0, ESC side, next to the USB-C; pin order RX4, TX4, 5V, GND)** = TELEM2 | 3 | Radxa pin 5 to RX4, pin 3 to TX4, pin 6 to GND, **5V position left empty** (Radxa is on its own BEC). 3.3 V both ends, no level shifter. Radxa pins 3 / 5 have I2C pull-ups (Radxa docs): harmless for a UART, line idles high anyway. Not pins 8 / 10: that is UART2, the Radxa debug console, keep it free for bring-up. Device on the Radxa `/dev/ttyS3` (CHECK after the overlay). Needs a 4-pin SH1.0 pigtail |
| 5 V BEC (separate, >= 3 A, from the battery side of the XT60) | Radxa header pins 2 or 4 (+5 V) and 9 (GND) | 2 | **Not the FC's 5 V 2 A rail**: Radxa ~1.5 A + camera ~0.3 A + XR2 0.1 + MTF-02P 0.1 (all E) exceeds it, and a brownout there takes the receiver and the kill switch down. **BEC = the spare 5 V 5 A UBEC from the earlier order, in hand (Spark, 7 Oct 00:55)**, ~20 g (E): weigh (P), check its output selector / voltage reads 5.0-5.2 V before it touches the Radxa. **Never have the PC on the Radxa OTG port while the BEC is live**: pins 2 / 4 and VBUS are the same 5 V rail, two supplies back-feed each other. Bring-up: BEC + ssh over wifi, or PC on USB with the BEC unplugged |
| DECXIN camera USB | Radxa **USB3.0 host port = the middle connector, next to the micro HDMI** (P, silkscreen, 7 Oct; the 5 Oct plan had the two ports swapped). The end port by the SD slot is USB_OTG / 5V IN: bench power and PC link only | 1 cable | Plug must exit through an opening in the gap; see Problems |
| MTF-02P, XR2 | as built (TELEM1, GPS port) | | unchanged |

PX4: `UXRCE_DDS_CFG` = TELEM2 (102), `SER_TEL2_BAUD` 921600, `MAV_1_CONFIG` 0 if it was on TELEM2. Radxa: `micro-xrce-dds-agent serial --dev /dev/ttyS3 -b 921600`. Verify with `uxrce_dds_client status` on the FC console. If no link: swap TX / RX at the Radxa end first, then check baud.

Assembly order: 1 fit the underside v5 (side M3 stack, nose nyloc; Problem 10). 2 Radxa into the gap on 1 mm VHB, connector edge to the opening, header holes soldered (no pin header, saves ~2.5 (E)). 3 UART and 5 V leads out of the gap to the FC under the plate. 4 pocket v12 onto the 3 bolts, camera inserts and camera. 5 continuity, then BEC voltage on the bench PSU before anything touches the Radxa. 6 power up Radxa alone, then with the FC. 7 B7 video path, then cage.

Problems: (a) a straight USB-C plug adds ~20 (E) sideways to the 30 wide board, wider than the 39 cartridge footprint: use a 90° USB-C plug or a short custom cable, measure the opening on the printed v12. (b) heat: RK3566 ~2 W (E) boxed under the pack and over the aluminium plate; it throttles, does not crash: run the demo load on the bench and read the SoC temperature before trusting it in flight. (c) wifi: onboard antenna shadowed above (pack) and below (plate), radiates sideways through PETG only; test range at the demo distance, fall back to the u.FL antenna out of the gap. (d) microSD: the slot is on the board's underside at the OTG end and the card goes in from the short edge (P); it must stay reachable or be loaded before the board is stuck down, and the VHB must not cover the slot; check it clears the gap struts.

## Firmware
- PX4 **1.18.0-rc1**, target `micoair_h743-v2`. MicoAir's bootloader was replaced over DFU (Zadig + dfu-util); files on the PC in `Scout-DFU`.
- Airframe 4050 (Generic 250 Racer). **Tuning is generic default, never adjusted.**
- `SENS_BOARD_ROT 8` (board inverted). Gyro, accel, level calibrated 28 Sept.
- No GPS, no compass. `EKF2_OF_CTRL 1`, `EKF2_RNG_CTRL 2`, `EKF2_HGT_REF 2`. Flow / range offsets currently X 0, Y 0, Z 0.02 m; new B9 + B10 values in `scout_B9_B10_2026-10-04.params`, **not loaded**.
- `SENS_FLOW_ROT 4` (180°), set 1 Oct. **Unverified in flight.**
- Motors from above: FR and RL CCW, FL and RR CW (Spark, 1 Oct). Motors 3 and 4 reversed by swapping wires (QGC DShot direction command broken on rc1, Bluejay telemetry-bit bug, fixed upstream PR #28826).
- `BAT1_V_DIV 21.12`, `BAT1_A_PER_V 14.14`. RC loss: Land.
- No parameter backup yet (B6).

## Radio
Pocket model "Scout" = copy of the Overseer model, XR2 bound. Ch 1-4 sticks (calibration copied from the Overseer, wizard not run). Ch 5 mode: Stabilized / Altitude / Position. Ch 8 kill (same switch as the Overseer). Arm: throttle down, yaw right.
**Unverified:** stick directions on the QGC radio page, kill with motors running, RC-loss failsafe. Check props off.

## Flights
- **By 7 Oct, first stable flight (Spark, reported 7 Oct 00:51):** bring-up finished, flies stable. Mode, what fixed the drift, parameter changes and backup not recorded here: the Firmware, Radio and Bring-up sections below still show the pre-flight state (CHECK with Spark, save a parameter file).
- **1 Oct, first try:** armed in Position, never left the ground. At part throttle the prop hubs split round the screws: two props came off (FR, RL), two cracked. Cause not found. Suspects: 5 mm centre-hole props instead of the 1.5 T-mount version, countersunk screw heads, overtightened screws, generic tune buzzing on the ground.
- **1 Oct, new props:** flies, no damage. Drifts in Position mode.

## Bring-up tasks (all need Spark at the bench), in order
**Spark, 7 Oct 00:51: bring-up finished, first stable flight done. Next is the Radxa (B7 video path). Which rows below were closed and how is not recorded (CHECK).**
| ID | Task |
|---|---|
| B9 | Load flow / range offsets for the underside (lens x +1.9 mm from the FC centre, y 0, face 22.5 below the plate) |
| B1 | Position drift: is flow fused, is `SENS_FLOW_ROT 4` right |
| B10 | PX4 speed caps 1.0 m/s |
| B2 | Level horizon |
| B3 | Props-off radio checks (above) |
| B4 | Prop hub failure cause |
| B7 | Video path |
| B5 | Autotune, once position hold works |
| B6 | Parameter backup |
Fly over textured floor, 0.5-1.5 m, good light: Stabilized, then Altitude, then Position.

## Flight setup brief
- **Must not change:** PX4 version matched to the Overseer; switches matched (mode 5, kill 8); nothing that needs resoldering without asking.
- **Done when:** holds position within ~10 cm over a textured floor; kill and failsafe proven props off; parameter file saved.

## Lights (proposed, not built)
WS2812B status LEDs on the Overseer's colour code, plus a 1-3 W white headlight. LED control must be worked out for PX4 (the proposal assumed Betaflight).

## Where things are
Pocket, cartridge, underside, pack bar: PC `scout\cad\current` (README), work in `wip`. Cage: `scout\cage` (README), v8 in `wip\cage_v8`. Firmware: `firmware`; DFU files in `Scout-DFU`. Old cage STLs v3-v5: `v3`, `v4`, `v5` / `archive`.
