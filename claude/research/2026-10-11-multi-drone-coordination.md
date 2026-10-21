# Multi-drone coordination for Nox: shared frame, submaps, handoff, avoidance (11 Oct 2026)

**Bottom line**
1. Nox can skip collaborative SLAM. Swarm-SLAM, DCL-SLAM, Kimera-Multi and COVINS-G mainly solve robots that start apart, and ran on Jetson Xavier or Intel i7 boards [Swarm-SLAM, DCL-SLAM, COVINS-G]. Both Nox drones boot inside the box, squared by the iris, so the box can be every sortie's origin (E).
2. PX4 v1.18.0-rc1 already has the parts: per-drone DDS namespaces, aux position fusion, and collision prevention fed by an obstacle stream [PX4 source]. Keep DDS on each drone; a Zenoh bridge carries the WiFi leg [PX4 uXRCE-DDS, Zenoh discovery].
3. Demo (E): the box laptop holds the global map, task list and traffic rules. The Overseer sends pose and keyframe submaps; the Scout sends video and pose and takes goals. "Refuse a move" becomes virtual obstacle sectors that PX4 enforces on the Scout.

## Findings

**Shared frame.** LAMP 2.0 fixed one frame by scanning three reflective plates of known position on a start gate [LAMP 2.0]. Swarm-SLAM anchors the team to the lowest-ID robot's first pose [Swarm-SLAM]; Kimera-Multi aligns frames from inter-robot loop closures [Kimera-Multi]. With the magnetometer off, PX4's EKF takes its first attitude from the accelerometer alone [PX4 ekf.cpp], so nose direction and position at boot become the local frame (E, read from code). Command 620 resets heading later (default accuracy 20°) and marks yaw aligned [PX4 EKF2.cpp].

**Map and submap exchange.**
| Stack | Sent between robots | Volume | Ran on |
|---|---|---|---|
| Swarm-SLAM (ROS 2, MIT) | ScanContext descriptors; clouds for match candidates only; pose graphs | 94.95 MB, 3 robots, 3,103 keyframes (~31 kB each, E) | Jetson AGX Xavier |
| DCL-SLAM (ROS 1) | LiDAR-Iris descriptor; cloud on a match; 104 B result | ~29 kB + ~100 kB per keyframe; keyframe per 1 m or 0.2 rad | i7-1165G7 NUC |
| Kimera-Multi | DBoW2 vectors; keypoints on request; stereo VIO only | up to 70% under centralized | CPU |
| COVINS-G | keyframe keypoints to a server | 179.74 kB/s per agent | i7 NUC agents, laptop server |
| LAMP 2.0 | pose graph + keyed scans to a base, batched after dropouts | not given | 64-core base |
| RACER | owned grid cells, trajectories, map chunks over UDP with resend lists | under 50 kB/s plus map | Jetson TX2 |

Sources per row: [Swarm-SLAM, DCL-SLAM, Kimera-Multi, COVINS-G, LAMP 2.0, RACER]. Wildcat peers swap submaps and each optimizes the team graph [Wildcat]; CTU's SubT drones shared a graph of free-space cells [CTU SubT]. Cloudini shrinks clouds to 0.15 to 0.5 of raw at ~1 mm [Cloudini]. No Pi 5 or RK3566 run of any stack was found.

**Handoff and allocation.** In a lidar-explorer plus camera-drone team, the explorer shares voxels flagged occupied, interesting (next to occupied) or visited, then gives each camera drone a region and an entry voxel; allocation is rule-based, not TSP [Explorer-photographer]. CTU down-weights frontiers teammates already explored; the operator can block or force goals [CTU SubT]. RACER deals out grid cells pairwise and orders them by CVRP: four drones covered an office in 35.8 s vs 45.0 to 50.6 s for baselines [RACER].

**Avoidance between drones.** EGO-Swarm broadcasts each new trajectory (under 0.5 KB) with an ellipsoid clearance shortened in z for downwash; relative drift reached 0.5 m, fixed by spotting the other drone in depth images [EGO-Swarm]. MRS shares predicted trajectories at 1 Hz or more, inflates them 5.0 m, and the lower-priority drone climbs 3.0 m, as a last resort [MRS]. Two 34 g drones 30 cm apart feel 9 g of downwash; earlier work needed 60 cm vertical spacing [Neural-Swarm2]. PX4 collision prevention runs only in Position mode, fuses 72 sectors, blocks moves into sectors with no data, ignores data over 0.5 s old and switches to Hold after 5 s without data [PX4 CP doc, PX4 source]; PX4 calls the companion path untested [PX4 CP doc]. PX4 traffic avoidance is ADS-B with a 500 m minimum radius [PX4 params].

**PX4 and ROS 2 plumbing.** UXRCE_DDS_NS_IDX gives /uav_N namespaces; clients sharing an agent need distinct UXRCE_DDS_KEY; commands are dropped unless target_system is 0 or MAV_SYS_ID [PX4 multi-vehicle]. DDS is unauthenticated and writes into uORB, so PX4 says keep it on the cable and localhost [PX4 uXRCE-DDS]. A Zenoh bridge per robot takes a namespace [zenoh-plugin-ros2dds], cut discovery traffic 97 to 99.97% on a TurtleBot [Zenoh discovery], and a race team ran it with topic allowlists and rate caps, pose at 10 Hz [Zenoh IAC]. EKF2 starts vision fusion only with samples under 200 ms apart [PX4 common.h]. Aux global position takes up to 4 sources, horizontal only in rc1, 3 SD gate; it starts once yaw is aligned and seeds the global origin from its first fix if none is set [PX4 AGP code]. ELRS with Model Match off: every receiver built with the phrase passes sticks to its FC [ELRS].

**What runs where.** Pi 5: no H.264 hardware encoder, 1080p30 costs 60 to 150% of one core [Pi H.264]; AprilTags on a 2 MP camera at 20 fps, ~200 ms filtered delay [aprilmav]. Radxa Zero 3W: 4x A55 at 1.6 GHz, 0.8 TOPS NPU, hardware H.264/H.265 encode to 1080p100 [CNX]. Neither should run collaborative SLAM (E).

## Demo architecture (E)
| Where | Runs | Sends | Rate |
|---|---|---|---|
| Overseer Pi 5 | L2 odometry (other report), local map, keyframe publisher, Scout tag fix, Zenoh bridge | pose; 5 cm voxel keyframes, compressed, 10 to 50 kB; Scout fix | 10 Hz; per 1 m or 0.2 rad (20 to 100 kB/s); 10 Hz |
| Scout Radxa | uXRCE agent, hardware H.264, virtual-scan node, Zenoh bridge | 640x480 video; pose | 15 fps at 1 to 2 Mbit/s; 10 Hz |
| Box laptop | Zenoh router, global map in box frame, task list, traffic manager, UI | map slice + Overseer pose to Scout; goals; hold flags | 2 Hz; 1 Hz; on event |

Total under 5 Mbit/s. On link loss each drone holds on its own odometry; no pose streams from the box into EKF2.

## Steal for Nox
- **Box:** catch point = map origin. A drone booted squared in the iris (mag off) starts its EKF in the box frame; the catch gate's yaw under 20° removes the square ring's 90° ambiguity (E).
- **Firmware (both):** MAV_SYS_ID, namespace and DDS key per drone, 1 and 2 [PX4 multi-vehicle].
- **Comms:** a Zenoh bridge per drone with allowlist and rate caps; DDS on localhost [Zenoh IAC, PX4 uXRCE-DDS].
- **Overseer:** keyframe submaps per 1 m or 0.2 rad, queued and batch-sent after dropouts [DCL-SLAM, LAMP 2.0].
- **Data product:** Overseer flags "interesting" voxels it can't see behind; the Scout gets a region and an entry point [Explorer-photographer].
- **Scout firmware:** "refuse a move" = the Radxa turns the map slice and the Overseer's pose into 72 obstacle sectors at 20 Hz, published only while box data is under 0.5 s old, so PX4 stops the Scout by itself [PX4 CP doc, PX4 source] (E).
- **Scout firmware:** tag fixes as aux global position around a fake box origin, after a command 620 heading set, to pull flow drift back [PX4 AGP code] (E).
- **Box:** each drone posts its path at 1 Hz or more; the Scout yields by holding [MRS, EGO-Swarm] (E).

## For Spark
- `overseer_params_2026-09-26.params`: "MAV_SYS_ID 1", "UXRCE_DDS_KEY 1", "UXRCE_DDS_NS_IDX -1", "UXRCE_DDS_PTCFG 0". The Scout is likely also 1 (PX4 default [PX4 params]; `scout.md` lists none). Set Scout to 2, namespace both, PTCFG 1 with localhost discovery (E).
- `scout.md` Radio, "Pocket model "Scout" = copy of the Overseer model": with Model Match off both powered drones could obey one stick set [ELRS] (E, test props off). With it on, the idle drone sees RC loss and lands (`scout.md` "RC loss: Land"; Overseer "NAV_RCL_ACT 3", "COM_RC_LOSS_T 0.5"). Two drones aloft need a second handset or COM_RCL_EXCEPT plus a box kill path (E).
- `nox-dimensions.md` §4 "single IMU, no compass" and params "EKF2_MAG_TYPE 5": keep the ordered compass off indoors so boot heading stays box-aligned (E).
- `overseer-scout-hardware-reference.md` §5 "Full SLAM, the baseline facility map" on the box: matches LAMP 2.0 and COVINS-G. EKF2 drops vision samples over 200 ms apart [PX4 common.h] and CTU's control degraded past 300 ms state delay [CTU SubT]: send box corrections as a slow frame offset applied on the Pi, never a WiFi pose stream (E). Its "Collision prevention, via PX4's built-in feature fed from the L2" is off now ("CP_DIST -1") and works in Position mode only [PX4 CP doc].
- `scout.md` B7 "Video path": encode on the Radxa's hardware [CNX]. The Pi 5 has none [Pi H.264]: keep Overseer video to 640x480 at 10 fps or stills (E).
- `overseer.md` "watches the Scout and can refuse its moves": put refusal in the box (traffic manager plus virtual scan), not a drone-to-drone link. Keep the Scout out of the Overseer's downwash column, starting at 2 m apart (E) [Neural-Swarm2].

## Sources
- [Swarm-SLAM (arXiv 2301.06230)](https://arxiv.org/html/2301.06230v3)
- [Swarm-SLAM repo (MISTLab)](https://github.com/MISTLab/Swarm-SLAM)
- [DCL-SLAM (arXiv 2210.11978)](https://ar5iv.labs.arxiv.org/html/2210.11978)
- [Kimera-Multi (arXiv 2106.14386)](https://ar5iv.labs.arxiv.org/html/2106.14386)
- [COVINS-G (arXiv 2301.07147)](https://arxiv.org/pdf/2301.07147v3)
- [LAMP 2.0 (arXiv 2205.13135)](https://ar5iv.labs.arxiv.org/html/2205.13135)
- [Wildcat (arXiv 2205.12595)](https://ar5iv.arxiv.org/html/2205.12595)
- [CTU SubT: UAVs beneath the surface (arXiv 2206.08185)](https://arxiv.org/pdf/2206.08185)
- [RACER (arXiv 2209.08533)](https://arxiv.org/pdf/2209.08533)
- [Explorer-photographer: cooperative exploration and inspection for heterogeneous aerial systems (arXiv 2403.01225)](https://ar5iv.labs.arxiv.org/html/2403.01225)
- [EGO-Swarm (arXiv 2011.04183)](https://arxiv.org/pdf/2011.04183v3)
- [MRS: inter-UAV collision avoidance docs](https://ctu-mrs.github.io/docs/features/inter-uav-collision-avoidance/)
- [Neural-Swarm2 (arXiv 2012.05457)](https://arxiv.org/pdf/2012.05457)
- [Cloudini: pointcloud compression (ROS Discourse)](https://discourse.openrobotics.org/t/cloudini-pointcloud-compression-for-the-rest-of-us/44009)
- [Zenoh discovery: minimising ROS 2 discovery overhead](https://zenoh.io/blog/2021-03-23-discovery/)
- [Zenoh IAC: experiences from the trenches](https://zenoh.io/blog/2021-09-28-iac-experiences-from-the-trenches/)
- [zenoh-plugin-ros2dds README](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds)
- [PX4 source and docs at tag v1.18.0-rc1 (uxrce_dds.md, ros2/multi_vehicle.md, collision_prevention.md, offboard.md, tuning_the_ecl_ekf.md, CollisionPrevention.hpp, ekf.cpp, EKF2.cpp, common.h, aux_global_position_control.cpp, dds_topics.yaml, navigator_params.yaml)](https://github.com/PX4/PX4-Autopilot/tree/v1.18.0-rc1)
- [Pi H.264: H.264 encoding performance on Raspberry Pi 5 (white paper)](https://pip-assets.raspberrypi.com/categories/685-app-notes-guides-whitepapers/documents/RP-010033-WP-1-H.264%20encoding%20performance%20on%20Raspberry%20Pi%205_series%20computers.pdf)
- [CNX: Radxa Zero 3W specs](https://www.cnx-software.com/2023/10/30/radxa-zero-3w-sbc-features-rockchip-rk3566-soc-up-to-8gb-ram-in-raspberry-pi-zero-2-w-form-factor/)
- [aprilmav (AprilTag indoor navigation for MAVLink)](https://github.com/stephendade/aprilmav)
- [ELRS: Model Match](https://www.expresslrs.org/software/model-config-match/)
