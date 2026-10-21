# Wall standoff and ceiling hold for Scout on PX4 (11 Oct 2026)

**Bottom line**
1. PX4 has no wall-hold or ceiling-hold mode. Collision prevention (CP) only brakes, in Position mode, and skips up and down sensors; EKF2 fuses only the first downward rangefinder [CP source, EKF2.cpp]. Both holds belong on the Radxa as a ROS 2 external mode sending velocity setpoints (E).
2. Sensor: ST VL53L1X in short mode (27° cone, ~1.3 m even in strong light, up to 50 Hz, 0.5 g carrier) [ST L1X, Pololu], one forward by the lens, one upward, on the Radxa's I2C (E). PX4's driver ranges at ~4 Hz and starts downward [L1X driver, rc.sensors].
3. The risks are glass, mirrors and dark walls, not aerodynamics: a ceiling touching Scout's cage adds ~0.8% thrust (E).

## Findings

| Sensor | Range | Cone | Rate | Mass, bus | Src |
|---|---|---|---|---|---|
| VL53L1X | long 3.6 m white, 1.7 m grey 17% (dark), 0.73 m strong light; short 1.35 m | 27° | ≤50 Hz | 0.5 g carrier, I2C | [ST L1X, Pololu] |
| VL53L4CD | 1.2 m white, 0.45 m grey | 18° | ≤100 Hz | I2C | [ST L4CD] |
| VL53L4CX | 5.0 m white, 2.1 m grey, 4 returns | 18° | 30 Hz | I2C | [ST L4CX] |
| VL53L7CX | 4x4 3.5 m dark, 0.65 m at 5 klx | 60x60° | 30 Hz | I2C, 0.2 to 0.3 W | [ST L7CX] |
| TF-Luna | 8 m white, 2.5 m at 10%, min 0.2 m | 2° | ≤250 Hz | <5 g, UART/I2C | [TF-Luna] |
| Evo Mini | 3.3 m | 27° | 16 Hz | 9 g | [Terabee] |
| MTF-02P (fitted, down) | 6 m, ±2 cm to 2 m | 2° | 50 Hz | 1.5 g, UART | [MicoAir, Rotorama] |
| MicoAir MT-06 | 6 m, ±4 cm to 2 m | unpublished | 50 Hz | 0.8 g, UART/I2C | [MT-06] |

- **Dark walls, light:** L1X short mode keeps 1.2 to 1.35 m on white to grey in strong light [ST L1X]; L4CD sees grey only to 0.45 m, too short for a 0.52 m hold [ST L4CD].
- **Glass, mirrors:** a pulse returns from the glass and from behind it; a mirror adds a peak near twice the distance [ST patent]. Single-target sensors output one (E); L4CX and L7CX output several [ST L4CX, ST L7CX]. A sub-300 g quad paired a ToF camera with ultrasound to see glass [arXiv glass].
- **Wet, dust:** no datasheet numbers; wet walls likely act like glass, dust gives early returns (E).
- **Cone:** at 0.5 m, 27° covers ~24 cm, 2° ~2 cm (E). Wide cones catch rack edges; narrow beams slip through gaps (E).
- **Windows:** window plus gap under 1 to 2 mm, then recalibrate crosstalk [AN4907, ST L1X]. Mount bare.
- **Disagreements:** MTF-02P power 0.2 W [MicoAir] vs 0.5 W [Rotorama]; L4CD max 1.2 or 1.3 m in one datasheet.

**Cross-talk.** Two VL53L1X facing each other at 5 cm sometimes read over 6 m, range status 4 or 7 [ST two sensors]. ST saw slight interference, from the other sensor's pulses acting as ambient light, avoided by longer timing budgets [ST multiple, ST two sensors]. Scout's three sensors never face each other (E). The one facing pair: Scout's upward sensor under the Overseer's downward MTF-02P, both 940 nm [Rotorama, ST L1X]. Drop readings without status OK (E).

**PX4 collision prevention (same in v1.18.0-rc1 and main).** Position mode, acceleration-based (default). 72 bins of 5° (doc text also says 36); each sensor fills its h_fov, tilt is projected, up and down sensors are skipped [CP source]. No data for 0.5 s blocks motion; after 5 s, Hold [CP doc]. CP_DIST counts from the sensor; CP_GO_NO_DATA 0 blocks unsensed directions [CP params]. It scales stick acceleration only: sticks centred, nothing returns the drone [CP source]. It stops; it does not hold. Companion OBSTACLE_DISTANCE is marked untested [CP doc].

**Drivers and EKF2.** micoair_h743-v2 builds the VL53L1X and TFmini drivers [board config]. The VL53L1X driver runs long mode, 200 ms budget, 250 ms period, alternating two zones ±6.75° [L1X driver]; rc.sensors starts it downward [rc.sensors]. EKF2 takes the first fresh downward sensor [EKF2.cpp]. TFmini floors at 0.4 m [TFmini driver]. No PX4 height logic reads an upward sensor (source grep, E). Range height reference assumes a flat floor [EKF doc].

**Companion control.** Offboard needs 2 Hz proof of life; after COM_OF_LOSS_T 1.0 s it falls to Position mode [offboard doc, commander params]. ROS 2 external modes (v1.15+, experimental) sit on an RC switch, read sticks, and failsafe if the mode stalls [ROS 2 doc]. rc1 sends no distance_sensor out over DDS, so FC-wired sensors are invisible to the Radxa [dds_topics]. Inbound distance_sensor is single-instance and likely lands on the MTF-02P's (E, uORB code): send obstacle_distance in body frame.

**How others do it.** Elios 2 Distance Lock holds 30 to 200 cm while the pilot slides along a weld [Nexxis, Sphere, Airscope], with 7 vision and distance sensors, split unpublished [Airscope]. Elios 3 lists one ToF sensor plus lidar SLAM [E3 spec]. Voliro T: 6 lidar distance sensors and a depth camera [Voliro spec]. ArduPilot surface tracking follows a ceiling with an upward rangefinder; out of range, throttle returns to climb rate; back in range, the target resets to the current distance [ArduPilot]. ArduPilot wall lock stayed a forum idea [ArduPilot forum].

**Aerodynamics.** On a Crazyflie (r 23 mm) a ceiling raised lift up to ~60% very close, mattered within ~1 r, and pulled the craft in (simulation); a side wall changed lift under 5% [Crazyflie]. The model in [IMAV] gives +3% at 1 r (E). Scout's top ring is 89.5 above the props (2.0 r): +0.8% (E).

## Steal for Nox
- **Firmware (Scout):** "Wall Hold" external mode on a switch: vx = 0.8 x (range - 0.52), capped ±0.3 m/s; sticks keep y, z, yaw (E).
- **Firmware (Scout):** "Ceiling Hold" as ArduPilot: lock distance on engage and after dropouts; on loss or a >0.3 m jump, zero vz, hand back throttle (E).
- **Scout:** VL53L1X short mode, 33 ms, level, beside the lens; 27° sits inside the camera's rod-free view (E).
- **Scout cage (v8c):** upward VL53L1X on the Radxa mount's top clip (E).
- **Firmware fallback:** FC-wired VL53L1X started forward, CP_DIST 0.52, CP_GO_NO_DATA 1: it parks at the wall (E).
- **Scout v2:** VL53L7CX 4x4 forward for distance, wall yaw and glass flags (E).
- **Overseer:** same mode fed by L2 points as obstacle_distance (E).

## For Spark
- `nox/scout.md` "EKF2_RNG_CTRL 2, EKF2_HGT_REF 2": keep, but start any side or top VL53L1X with -R 0 or -R 24, never SENS_EN_VL53L1X alone, or EKF2 may fuse it as height (E).
- `nox/scout.md` "Radxa UART3 (header pins 5 TX, 3 RX...)": usually the I2C pins; find another Radxa I2C bus (CHECK pinout, E).
- `nox/scout.md` "holds position within ~10 cm": Wall Hold fixes one axis only; close B1 and B5 first (E).
- `nox/nox-dimensions.md` lens "drone (92.66, 0, 36.0)", equator "230 across flats": 50 cm off the cage front reads ~0.52 m (E).
- `nox/v2/README.md` budget has no sensor line: ~2 to 3 g for two sensors and wire (E).
- `claude/research/2026-10-11-flyability-elios-tech.md` wall-hold "from the flow/ToF sensors": the MTF-02P looks down; a forward ToF is needed.

## Sources
- [ST L1X: VL53L1X datasheet](https://www.makerguides.com/wp-content/uploads/2024/10/VL53L1X-Datasheet.pdf)
- [ST L4CD: VL53L4CD datasheet](https://www.st.com/resource/en/datasheet/vl53l4cd.pdf)
- [ST L4CX: VL53L4CX datasheet](https://www.st.com/resource/en/datasheet/vl53l4cx.pdf)
- [ST L7CX: VL53L7CX datasheet](https://www.st.com/resource/en/datasheet/vl53l7cx.pdf)
- [AN4907: ToF cover window guidelines (ST)](https://www.st.com/resource/en/application_note/an4907-vl53l0x-ranging-module-cover-window-guidelines-stmicroelectronics.pdf)
- [Pololu: VL53L1X carrier](https://www.pololu.com/product/3415)
- [TF-Luna datasheet (Waveshare)](https://files.waveshare.com/upload/c/c3/SJ-GU-TF-Luna_A05_Datasheet.pdf)
- [Terabee: single-pixel comparison](https://terabee.com/wp-content/uploads/2021/02/Single-pixel-comparison-table.pdf)
- [MicoAir: MTF-02P](https://micoair.com/?p=4526)
- [Rotorama: MTF-02P](https://www.rotorama.com/product/micoair-mtf-02p)
- [MT-06: MicoAir](https://micoair.com/range_sensor_mt-06/)
- [ST two sensors: forum](https://community.st.com/mems-sensors-48/two-vl53l1x-sensors-influencing-their-measurements-52421)
- [ST multiple: forum](https://community.st.com/imaging-sensors-49/multiple-sensors-generate-interference-between-each-other-37067)
- [ST patent: US10594920B2](https://patents.google.com/patent/US10594920)
- [arXiv glass: glass detection on aerial robots](https://arxiv.org/abs/2510.06518v1)
- [CP doc](https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/computer_vision/collision_prevention.md), [CP source](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/collision_prevention/CollisionPrevention.cpp), [CP params](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/collision_prevention/collisionprevention_params.yaml)
- [L1X driver](https://github.com/PX4/PX4-Autopilot/blob/v1.18.0-rc1/src/drivers/distance_sensor/vl53l1x/vl53l1x.cpp), [TFmini driver](https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/distance_sensor/tfmini/TFMINI.cpp), [rc.sensors](https://github.com/PX4/PX4-Autopilot/blob/v1.18.0-rc1/ROMFS/px4fmu_common/init.d/rc.sensors)
- [EKF2.cpp](https://github.com/PX4/PX4-Autopilot/blob/v1.18.0-rc1/src/modules/ekf2/EKF2.cpp), [EKF doc](https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/advanced_config/tuning_the_ecl_ekf.md)
- [Board config: micoair h743-v2](https://github.com/PX4/PX4-Autopilot/blob/v1.18.0-rc1/boards/micoair/h743-v2/default.px4board)
- [dds_topics](https://github.com/PX4/PX4-Autopilot/blob/v1.18.0-rc1/src/modules/uxrce_dds_client/dds_topics.yaml)
- [Offboard doc](https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/flight_modes/offboard.md), [commander params](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/commander_params.yaml)
- [ROS 2 doc: control interface](https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/ros2/px4_ros2_control_interface.md)
- [Nexxis: Elios 2](https://nexxis.com/?p=5486)
- [Sphere: Elios 2 distance lock](https://spheredrones.com.au/2021/02/08/flyability-elios-2-transforms-diesel-tank-inspections/)
- [Airscope: Elios 2 release](https://airscope.ae/flyability-releases-elios-2-at-auvsi-xponential-positioned-to-be-best-indoor-inspection-solution-on-the-market/)
- [E3 spec: Elios 3 tech spec](https://nexxis.com/wp-content/uploads/sites/3/2022/06/20220519_E3_Tech_Spec_web_version.pdf)
- [Voliro spec: Voliro T spec sheet](https://cdn.voliro.com/wp-content/uploads/2025/06/Voliro-T-Spec-sheet-v3.2.pdf)
- [ArduPilot: surface tracking](https://ardupilot.org/copter/docs/terrain-following-manual-modes.html)
- [ArduPilot forum: wall locking drone](https://discuss.ardupilot.org/t/wall-locking-drone/51766)
- [Crazyflie: ground, ceiling and wall effects (NSF PAR)](https://par.nsf.gov/servlets/purl/10267925)
- [IMAV: ceiling effect, IMAV 2024](https://www.imavs.org/papers/2024/37.pdf)
