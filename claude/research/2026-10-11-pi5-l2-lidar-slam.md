# Lidar SLAM on the Overseer's Pi 5 with the Unitree L2 (11 Oct 2026)

**Bottom line**
1. Compute is unlikely to be the limit. FAST-LIO2 took 5 to 55 ms per scan on an older ARM board [FAST-LIO2], and an L2 cloud is only ~5,200 points [SDK]. Expect ~10 to 30 ms per scan on a Pi 5 (E).
2. Timing is the weak spot. Unitree documents no PTP or PPS sync [L2 page, manual], and the driver stamps data with the Pi's arrival time [SDK]. Users report a half-rate time base, irregular IMU stamps, startup drift and doubled walls [SDK issues, point_lio_ros2 #5].
3. Run FAST-LIO2 (ROS 2) on the Pi. A small bridge node levels its pose, adds velocity and sends both to `/fmu/in/vehicle_visual_odometry` over the existing uXRCE-DDS link. Keep flow and range as the fallback. Do loop closure on the box, the same odometry/pose-graph split Hovermap uses (E).

## Findings

**Unitree L2.** View 360 x 90° (96° in wide mode); 64,000 points/s; 5.55 Hz horizontal scan. Range 30 m at 90% reflectivity, 15 m at 10%; ≤2 cm accuracy. Power 10 W, rising to 13 W while it self-heats; it sends no points until warm. $419 [L2 page, manual]. The IMU has 6 axes and no magnetometer; it samples at 1 kHz and reports at 500 Hz [manual].

Sources disagree on the IMU. The manual says IMU output ships off; the SDK says it ships on [SDK]. Unitree's Point-LIO config assumes 250 Hz [point_lio_unilidar]. One user logged ~800 Hz with gaps from 2 µs to 100 ms [point_lio_ros2 #5].

The IMU axes are parallel to the lidar's, offset (-7.7, -14.7, +6.7) mm [SDK]. One cloud is 18 UDP packets of up to 300 points [SDK]: about 12 Hz and 2.5 Mbit/s (E).

**Driver and time.** The ROS 2 node publishes `unilidar/cloud` (with ring and per-point time) and `unilidar/imu`. Unitree tested it only on Ubuntu 20.04 with Foxy [SDK]. Each cloud is stamped with Pi arrival time minus the scan period. Lidar and IMU therefore share the Pi clock but pick up network jitter [SDK].

Open SDK issues [SDK issues]:
- #25: the time base runs at half the real rate (firmware 2.8.11.1).
- #27: walls appear twice, 5 to 20 cm apart. A 10 s wait after power-up and mounting on the supplied base helped.
- #24: SLAM is unstable with the lidar tilted ~45°.
- #17: no data on Humble until `initialize_type` was corrected.

Lidar-inertial odometry (LIO) packages drop data whose timestamps go backwards, so a board that boots with its clock at 1970 breaks them [Point-LIO #121].

**Odometry options**

| Stack | How it works | ARM evidence | Fit (E) |
|---|---|---|---|
| FAST-LIO2 | Iterated Kalman filter; raw points matched to an incremental k-d tree map | Khadas VIM3 (4x A73): 5.2 ms per scan at 756 points, 54.8 ms on dense NCLT [FAST-LIO2]; 39 ms on Orin NX [Super-LIO]; README lists Pi 4B [FAST_LIO_ROS2] | First choice. An L2 user saw less drift than with Point-LIO [point_lio_ros2 #5] |
| Point-LIO | Updates per point; output at IMU rate | No ARM numbers. Unitree L2 config, ROS 1 [point_lio_unilidar]; ROS 2 port [point_lio_ros2] | Backup; L2 drift reports still open |
| KISS-ICP | Point-to-point ICP, constant-velocity deskew, no IMU | 38 to 51 Hz (CPU not stated), 0.50% KITTI drift [KISS-ICP]; ~100 ms on a Pi 4 with an L2 [L2 toolkit] | Cross-check that does not depend on IMU timing |
| LIO-SAM | Factor graph, loop closure | 131 ms per scan on Intel [FAST-LIO2]; needs a 9-axis IMU [LIO-SAM] | No |
| GLIM | GICP odometry, submap global registration | Built GPU-first; CPU configs and ARM64 packages exist [GLIM] | Mapper on the box |

The RK3588 uses the same A76 cores as the Pi 5. On it, the lidar part of FAST-LIVO2 took 56 ms per frame on dense Hilti scans, 2.3x the x86 time [FAST-LIVO2 RC]. Over 5 km, FAST-LIO2 drifted 6.8% against Wildcat's 2.9% [Wildcat]. Room-scale flights should drift under 1% (E).

FAST-LIO2's ROS 2 node publishes pose and covariance at scan end. The pose is in the first IMU frame, which is not levelled, and there is no velocity [FAST_LIO_ROS2]. The Point-LIO port adds world-frame velocity [point_lio_ros2].

**PX4 EKF2 input**
- **Topic.** uXRCE-DDS takes `/fmu/in/vehicle_visual_odometry` [dds_topics]. MAVROS instead sends `/mavros/odometry/out` as ODOMETRY in LOCAL_FRD [PX4 docs]. Pose frames: NED or FRD. Velocity frames: NED, FRD or body FRD [msg].
- **Heading.** With FRD data and external-vision (EV) yaw off, EKF2 rotates EV data by a filtered attitude error. With EV yaw on, it uses EV position without rotating it [EKF2 source]. That frame must then be level (E).
- **Delay.** `EKF2_EV_DELAY` (0 to 300 ms) is subtracted from `timestamp_sample` [EKF2 params]. With `UXRCE_DDS_SYNCT` 1, the client maps Pi timestamps to PX4 time, and a zero stamp becomes arrival time [uXRCE-DDS source].
- **Rate.** The docs ask for 30 to 50 Hz [PX4 docs]. The code needs samples less than 200 ms apart to start fusing and stops after 400 ms with no data [EKF2 source].
- **Mount and resets.** `EKF2_EV_POS_*` corrects only a lever arm, not a mount rotation. A change in `reset_counter` forces a clean EV reset [EKF2 source].
- **Known failure.** A FAST-LIVO2 user on a MicoAir743 V2 lost EV fusion after ~135 s. Mixed NED/FRD labels or mismatched clocks were suspected; it is unresolved [PX4 forum].

**Hovermap and Exyn.** Hovermap runs CSIRO's Wildcat SLAM on an Intel NUC [SubT survey]. Wildcat works like this [Wildcat]:
- Odometry is fitted over a continuous-time sliding window.
- Surfels are bundled into 6 s submaps.
- A pose graph closes loops using surfel ICP plus place recognition.
- Speed: 1 to 4 Hz on a Jetson Xavier, 63 ms per cycle on a Xeon.
- Sensors: a VLP-16 spun at 0.5 Hz and a separate 3DM-CV5 IMU at 100 Hz.

Hovermap's own IMU is not published. The ST-X gives 640,000 points/s over 360 x 290° and ±10 mm indoors [Hovermap specs]. Exyn spins an XT32 lidar on a Core i7 (Voliro/Exyn report). Nox differs from both in four ways: ARM compute, a fixed lidar, the lidar's built-in IMU, and loop closure off board.

## Steal for Nox
- **Overseer firmware:** FAST-LIO2 with the L2's own IMU. A bridge node levels the start frame using gravity, applies the mount rotation, adds velocity and sends FRD `VehicleOdometry` stamped at scan end (E).
- **Overseer firmware:** increment `reset_counter` on every odometry restart and every takeoff from the box [EKF2 source].
- **Box:** use the catcher's known pose as the odometry origin for each sortie, so drift restarts at zero (E).
- **Data product:** log raw L2 UDP on board (~1 GB/h, E) and loop-close submaps on the box (GLIM or a pose graph) [Wildcat, GLIM].
- **Overseer firmware:** feed the L2 into PX4 collision prevention through `/fmu/in/obstacle_distance` (72 bins) [dds_topics].
- **Overseer:** stiff L2 mount; wait 10 s after power-up and keep still during IMU init [SDK #27, point_lio_unilidar].
- **Overseer compute:** fit the Pi 5 RTC battery [Pi 5 page] and run chrony, so timestamps never jump back [Point-LIO #121].

## For Spark
- **FAST-LIO2 settings for the L2 (E):**
  - Point format: `lidar_type` 2 (reads ring and per-point time), `scan_line` 18, `timestamp_unit` 0.
  - `blind` 0.5 m, plus a crop box around the props and catch frame.
  - Filter sizes 0.3, `point_filter_num` 1.
  - Extrinsic T (0.0077, 0.0147, -0.0067), as in Unitree's config [point_lio_unilidar].
  - Driver `cloud_scan_num` 9 for ~24 Hz clouds [SDK].
- **Params.** `overseer_params_2026-09-26.params` has EKF2_EV_CTRL 0, HGT_REF 2, RNG_CTRL 2, EV_DELAY 0, SDLOG_PROFILE 1, CP_DIST -1 and SYS_HAS_MAG 0. A separate LIO file (E):
  - EV_CTRL 15. Yaw must come from EV because there is no magnetometer.
  - HGT_REF 3, RNG_CTRL 1.
  - EVP_NOISE and EVA_NOISE 0.05.
  - EV_DELAY 0, then tune it from logs.
  - SDLOG_PROFILE 129; bit 7 logs EV data [PX4 docs].
  - Keep OF_CTRL 1. Set CP_DIST ~0.7 later.
- **Pi 5 link.** `overseer.md` says "TELEM2 uXRCE-DDS 921600, not tested". This matches the board map (TELEM2 = UART4 on /dev/ttyS3) [PX4 MicoAir page] and UXRCE_DDS_CFG 102. Odometry at 30 Hz uses ~4 of ~92 kB/s (E).
- **L2 mount.** `nox-dimensions.md` §4 lists "Unitree L2, 230 g, 75 x 75 x 65, 12 V 10 W". Allow for the 13 W warm-up peak and wait for points before arming [L2 page] (E). The bridge must undo any mount tilt, because EKF2 will not [EKF2 source]. One user's SLAM failed at ~45° [SDK #24].
- **Benchmark.** `overseer-scout-hardware-reference.md` §1.4 notes it "has not been benchmarked". Replay Unitree's L2 bag through FAST-LIO2 on the Pi 5 before buying a ROCK 5C (E).
- **OS.** Unitree tested only Foxy; one user runs the driver on Jazzy with Ubuntu 24.04 [SDK #15]. That is the likely Pi 5 path (E).

## Sources
- [Unitree L2 product page](https://unitree.com/L2/)
- [Unitree 4D LiDAR L2 user manual](https://oss-global-cdn.unitree.com/static/Unitree%204D%20LiDAR%20L2%20User%20Manual.pdf)
- [SDK: unilidar_sdk2, L2 SDK and ROS 2 driver](https://github.com/unitreerobotics/unilidar_sdk2)
- [SDK issues #15, #17, #24, #25, #27](https://github.com/unitreerobotics/unilidar_sdk2/issues)
- [point_lio_unilidar, Unitree Point-LIO fork](https://github.com/unitreerobotics/point_lio_unilidar)
- [point_lio_ros2 and issue #5](https://github.com/dfloreaa/point_lio_ros2/issues/5)
- [Point-LIO issue #121, L2 timestamps](https://github.com/hku-mars/Point-LIO/issues/121)
- [FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)
- [FAST-LIO2 paper, arXiv 2107.06829](https://arxiv.org/pdf/2107.06829)
- [Super-LIO, arXiv 2509.05723](https://arxiv.org/html/2509.05723v2)
- [FAST-LIVO2 RC: FAST-LIVO2 on resource-constrained platforms, arXiv 2501.13876](https://arxiv.org/pdf/2501.13876)
- [KISS-ICP, arXiv 2209.15397](https://arxiv.org/pdf/2209.15397)
- [GLIM](https://github.com/koide3/glim)
- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)
- [L2 toolkit (CherubelK/Lidar)](https://github.com/CherubelK/Lidar)
- [Wildcat, arXiv 2205.12595](https://arxiv.org/pdf/2205.12595)
- [SubT survey: SLAM in extreme underground environments, arXiv 2208.01787](https://arxiv.org/pdf/2208.01787)
- [Hovermap specifications (Emesent)](https://knowledge.emesent.com/articles/hovermap-specifications)
- [PX4 docs: external position estimation](https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/ros/external_position_estimation.md)
- [PX4 EKF2 EV params](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/ekf2/params_external_vision.yaml)
- [PX4 EKF2 EV source](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/ekf2/EKF/aid_sources/external_vision)
- [PX4 VehicleOdometry msg](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleOdometry.msg)
- [PX4 uXRCE-DDS client and dds_topics](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/uxrce_dds_client)
- [PX4 MicoAir743-Lite page](https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/flight_controller/micoair743-lite.md)
- [PX4 forum: FAST-LIVO2 EKF2 fusion lost after 1 minute](https://discuss.px4.io/t/fast-livo2-ekf2-fusion-lost-after-1-minute/49551)
- [Raspberry Pi 5 product page](https://www.raspberrypi.com/products/raspberry-pi-5/)
