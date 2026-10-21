# Fastest drone landings and catchers (14 Oct 2026)

Research only; Nox docs not changed. Numbers carry a source or E. Builds on `2026-10-11-precision-hover-landing.md` (hover accuracy) and `2026-10-11-dock-swap-mechanisms.md` (dock mechanisms).

**Bottom line**
1. Nobody publishes approach-to-touchdown. The only vendor end-to-end number is DJI Dock 4, launched today: swap under 25 s from cover closed, landing-to-takeoff under 45 s, 3 stored packs [Dock 4 specs, PR Newswire]. Everything else is charge-bound (27 to 40 min) or a swap-only time (Hextronics 80 s).
2. In research the slow part is the settle hover, not the descent. A dynamic-landing controller touched a static pad 3.2 s after first tag sight against 27.2 s for a standard controller [Paris]; a fixed-time controller does the final descent in 1.5 s with 7.9 cm mean error, 10 of 10 [AUKF-NMPC]. So a 3 to 5 s hover-to-seated is realistic for the box (E), and the ±40 funnel capture is wider than those landing errors.
3. The fastest closing mechanism measured is a 20 ms impact-triggered claw (Stanford SNAG); the fastest actuated catch of a hovering quad is a 0.72 s soft gripper. Nothing published catches a hovering quad with a V-groove, funnel or jaw at a stated time; the funnel v3's 0.17 s slide [`v2/box-funnel.md`] would be the number.

## 1. Commercial docks (landing to ready)
| Dock | Landing method | Numbers | What limits speed |
|---|---|---|---|
| DJI Dock 4 + M5TD (14 Oct 2026) | RTK + markers on the pad; in-dock mechanism pulls the top-rear pack, 4 bays, max 3 packs stored | swap < 25 s from cover closed; landing to takeoff < 45 s; 27 min 15 to 95 % per pack; RTK hover ±0.02 m; lands in 12 m/s [Dock 4 specs, PR Newswire, DroneXL] | swap only if a stored pack is > 10 % above the landed one; touchdown to cover closed is the other ~20 s (E) |
| DJI Dock 3 / Dock 2 | RTK + markers, slide-ramp centring | 27 min 15 to 95 % (Dock 3) / 32 min 20 to 90 % (Dock 2); no landing accuracy published [Dock 3 specs, Dock 2 specs, Heliguy] | charging |
| Skydio Dock for X10 | GPS to 3 to 5 m, then AprilTag homing into a funnel cup that centres the pack; hovers ~5 cm up until pose settles, then touches down; patent claims a 5 x 5 cm target | launch ~20 s; 35 min 15 to 95 % (to 73 min with heating or cooling) [Skydio patent, Skydio FAQ] | charging; settle hover 10 to 20 s (E) |
| Hextronics Universal / Global | lands on a drawer with vision tags (FlytBase trigger at 5 to 15 m), drawer retracts, arm swaps the pack | Universal: swap 80 s, ±15 cm landing, 4 slots; Global: swap < 2 min, landing to relaunch < 5 min, 5 packs [Advexure, Drone Nerds, FlytBase] | pack handling plus drawer travel |
| Heisha T200 | contact-pad charging; swap version | "about 2 minutes" to a full pack, no precision number [Heisha PR] | charging |
| Gather AI (warehouse) | battery exchanger | 2 min [Gather AI] | mechanism |
| Airobotics, Asylon, Percepto, Nokia, Sunflower | arm swaps (Airobotics, Asylon) or charging | no times published; Sunflower hover accuracy 30 cm [Sunflower specs]; Asylon arm patent: a camera finds the drone pose and an x-y-z arm with a rotating grasper moves the drone and swaps the pack [Asylon patent] | unknown |

Catch-in-hover products: none with a published time. Insitu's arm patent closes two padded or magnetic fingers on a hovering aircraft once position, velocity and attitude are inside bounds, approach "at walking speed" [US9434481]. DARPA SideArm snags a 180 kg fixed-wing on a rail hook, no cycle time [New Atlas]. Skydio's retractable-pad patent presses the drone down with a roller while the pad retracts into the box [Skydio patent].

## 2. Research: how fast a quad can be landed or caught
| Work | Mechanism | Numbers | Limit |
|---|---|---|---|
| Paris et al. 2019 | dynamic landing in turbulent wind, 564 g quad | static pad 3.2 s from first tag detection (standard controller 27.2 s); moving pad at 1 m/s in 6.8 s [Paris] | approach from several m; settle phase dominates |
| AUKF + NMPC 2026 | fixed-time dynamic landing, mocap | final descent 1.5 s; error mean 7.9 cm, max 11.9 cm, 10 / 10 [AUKF-NMPC] | platform velocity estimate |
| Bi-directional agile landing 2026 | RTK + D435i + AprilTag, 1.86 kg | sim 2.4 s (0.8 m/s pad) to 4.6 s (2.0 m/s pad); real pads 0.4 to 0.7 m/s [Agile landing] | 3 m/s and 3 m/s² caps |
| MBZIRC 2017 (Bonn, ETH) | landing on a 15 km/h truck | detection to touchdown 11.6 s, 2 of 4; ETH capped touchdown vertical speed at 0.75 m/s so the frame absorbs it [NimbRo, ETH] | vertical speed cap, tracking |
| Mao et al. 2021 | visual perch on 60 to 90° Velcro, onboard VIO + tag | normal impact 0.4 to 0.6 m/s, peak 10 m/s², trajectory 1 to 1.4 s, 15 / 20, cm error [Mao] | Velcro needs low normal speed |
| PerchRL 2026 | magnet perch on a moving 65 to 70° plate, 610 g | sim 2.1 to 2.45 s; real planar error 3 to 23 cm; plate to 2.7 m/s [PerchRL] | 0.2 m tolerance, 30 to 54 Hz vision |
| Inverted ceiling landing (Crazyflie, 30 to 38 g) | flip at time-to-contact 0.15 to 0.25 s, sprung hips | approach 2.5 m/s vertical, 94 % four-leg success [Ceiling 1, Ceiling 2] | motor time constants |
| Soft-gripper catch of a hovering Crazyflie | arm-mounted soft gripper, mocap | close 0.72 s (rigid gripper 1.90 s), standoff 0.5 m, target 0.2 m/s [Frontiers] | gripper close time |
| Arm recovery on a moving base | KalmanNet + MPC predicts the drone 0.1 s ahead | > 95 % capture; no time given [Arm catcher] | arm torque, base motion |
| Stanford SNAG | bird-leg claw, impact energy closes the tendon | clutch closes in 20 ms [SNAG] | needs a branch-like target |
| Passive basket (MBZIRC ball) | no actuator | window 0.51 x 0.175 m, 7 to 8 of 10 at 2 to 6 m/s [Basket] | window vs tracking |
| Hand catch (FPV) | hover at arm's reach, disarm at 5 to 10 cm | no timed numbers; > 30 cm bounces [UAVmodel]; ~1 to 2 s from a 1 m hover (E) | human reaction |

## 3. Catching at a nonzero descent speed
No paper deliberately drops a hovering quad at 0.5 to 2 m/s into a damped receiver. Nearest data: adaptive gear drop tests at 1.5 m/s (large UAV) peak at 2.4 to 2.6 kN on a spring-damper ankle [MDPI gear]; an impact-resilient 1.42 kg quad on RC-car shocks survives 2.58 m/s wall hits and 5.9 m/s free fall, 8 to 9 of 10 [Resilient quad]; the Crazyflie lands inverted at 2.5 m/s on 0.08 N·mm/rad hip springs [Ceiling 1]. Energy is small (E): the Scout at 0.5 kg and 1 m/s is 0.25 J, the Overseer at 2 kg and 1 m/s is 1 J; a 20 mm damper stroke means ~2.5 g at 1 m/s, 50 mm ~1 g. The limit is prop-tip and pack shock, not energy.

## Steal for Nox
- Box funnel v3: the catch budget is hover settle, then stage rise 0.35 to 0.85 s at 0.08 m/s, then a 0.17 s slide [`v2/box-funnel.md`]. The settle is the part to attack: Paris shows the controller, not the mechanism, sets it (3.2 s vs 27.2 s).
- Box funnel v3: raise the stage faster. At 0.3 m/s the worst rise is 0.23 s and the face hit is ~0.17 m/s normal, 0.007 J on the Scout (E), still far under every impact figure above; the stage motor's current step still marks the touch.
- Firmware: a box-commanded descent into the funnel at ~0.5 m/s with the kill on first touch would remove the hover hold entirely (E); the 7.9 cm landing errors measured at 1.5 s descents fit inside the ±40 capture. The 35° faces then see ≤ 0.3 m/s normal (E).
- Box: define and time the same clock DJI uses (cover closed to swap done, 25 s) plus hover-gate to swap-done and swap-done to airborne, and publish all three for the pitch.
- Box: a seated check that costs no time, the KW11 under the foot frame, matches Skydio's no-time torque check; keep it.
- Test plan: count catches the MBZIRC way (first-run success, not best of n), and log the settle time separately from the mechanical time.

## For Spark
- `v2/box-funnel.md` Sequence 1 "hovers inside ±40, ±5° yaw, ≤ 0.15 m/s, held 0.2 s": the 0.2 s hold plus the velocity gate is where the seconds go. The funnel's ±40 covers the 7.9 cm mean / 11.9 cm max errors measured with no hold [AUKF-NMPC], so the gate could be ±40 and ≤ 0.3 m/s with no hold (E). Shared number, your call.
- `v2/box-funnel.md` Sequence 2 "stage rises at 0.08 m/s": nothing in the literature needs a rise that slow; 0.3 m/s cuts the worst rise from 0.85 to 0.23 s with a 0.007 J hit (E).
- `v2/box-seat.md` (groove seat) "seated ≤ 0.57 s" after a free drop from 10 above: the funnel at a faster rise seats in under 0.4 s with no drop, so on time the funnel wins (E).
- Pitch: DJI Dock 4's 45 s landing-to-takeoff is now the public bar. Nox needs a measured hover-to-airborne under that, indoors, with no RTK (E).

## Sources
- [DJI Dock 4 specs](https://enterprise.dji.com/dock-4/specs)
- [PR Newswire: DJI 25-second battery swaps](https://www.prnewswire.com/news-releases/dji-delivers-first-drone-dock-with-automated-25-second-battery-swaps-for-always-ready-aerial-support-302908510.html)
- [DroneXL: Dock 4 launch](https://dronexl.co/2026/10/14/dji-dock-4-matrice-5td-battery-swap-launch-no-fcc/)
- [DJI Dock 3 specs](https://enterprise.dji.com/dock-3/specs), [DJI Dock 2 specs](https://enterprise.dji.com/dock-2/specs)
- [Heliguy: Dock 3 precision landing](https://www.heliguy.com/blogs/knowledge-base/how-is-the-precision-landing-of-dji-dock-3-s-aircraft-achieved/)
- [Skydio patent US11897632](https://patents.justia.com/patent/11897632), [Skydio Dock FAQ](https://www.skydio.com/dock/faqs)
- [Advexure: Hextronics Universal](https://advexure.com/products/hextronics-universal), [Drone Nerds: Hextronics Global](https://enterprise.dronenerds.com/?p=3370), [FlytBase precision landing](https://docs.flytbase.com/device-management/device-management/hextronics-docks/settings-precision-landing)
- [Heisha T200 PR](https://www.einpresswire.com/article/675792796/heisha-unveils-t200-dual-drone-charging-solution-with-battery-swapping-system)
- [Gather AI battery swapper](https://gather.ai/platform/vision/features/battery-swapper)
- [Sunflower Labs specs](https://2022.sunflower-labs.com/specs)
- [Asylon patent US20230257139](https://patents.justia.com/patent/20230257139)
- [Insitu US9434481](https://patents.google.com/patent/US9434481), [New Atlas: SideArm](https://newatlas.com/darpa-sidearm-drone-catcher/47766/)
- [Paris et al., arXiv 1909.11071](https://arxiv.org/pdf/1909.11071)
- [AUKF-NMPC fixed-time landing, arXiv 2606.02658](https://arxiv.org/pdf/2606.02658)
- [Agile landing, arXiv 2601.03037](https://arxiv.org/pdf/2601.03037)
- [NimbRo MBZIRC landing](https://www.ais.uni-bonn.de/papers/ECMR_2017_Beul_MBZIRC_Landing.pdf), [ETH MBZIRC, arXiv 1710.08275](https://ar5iv.arxiv.org/html/1710.08275)
- [Mao et al., arXiv 2107.11171](https://arxiv.org/pdf/2107.11171)
- [PerchRL, arXiv 2606.03441](https://arxiv.org/pdf/2606.03441)
- [Ceiling 1, arXiv 2111.03539](https://arxiv.org/pdf/2111.03539), [Ceiling 2, arXiv 2403.00128](https://arxiv.org/pdf/2403.00128)
- [Frontiers soft-gripper catch](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2022.1030515/pdf)
- [Arm catcher, arXiv 2505.09145](https://arxiv.org/abs/2505.09145)
- [SNAG, Science Robotics 10.1126/scirobotics.abj7562](https://www.sciencedaily.com/releases/2021/12/211201145322.htm)
- [Passive basket, Robotica](https://www.cambridge.org/core/services/aop-cambridge-core/content/view/71F511ABDA65FB60203A6D555FC18D12/S0263574721001673a.pdf/design-and-integration-of-a-drone-based-passive-manipulator-for-capturing-flying-targets.pdf)
- [UAVmodel FPV hand catch](https://blog.uavmodel.com/?p=3086)
- [MDPI adaptive gear](https://www.mdpi.com/2226-4310/9/12/776/html), [Resilient quad, arXiv 2011.02061](https://arxiv.org/pdf/2011.02061)
