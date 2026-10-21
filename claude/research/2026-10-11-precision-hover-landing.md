# Precision hover over the Box catcher without motion capture (11 Oct 2026)

**Bottom line**
1. Published accuracy without mocap: about 5 to 11 cm for tags and IR beacons, 10 to 30 cm for UWB, about 9 cm σ plus drift for flow alone. With a tag closing the loop, every method fits the iris ±150 envelope. Flow alone does not fit it reliably.
2. Cheapest setup (E): add no sensor to either drone. The planned box camera tracks each drone's tag and sends the pose back to the drone's EKF2 as external vision (XY and yaw). A box-side upward ToF gives ring height. PX4 precland and VTE are the wrong tools: they always end in a landing, and they expect the target measured from the drone or given in the drone's own local frame.
3. The real risk in the last 20 cm is the box, not the estimator. The MTF-02P height reference steps at the box edge, flow needs light and texture under the head, and the jaws close just under the edge of the Scout's prop disks.

## Findings

**PX4 precision landing (landing_target_estimator).** An IR-LOCK sensor gives a ray to the target. A companion can send the same thing as LANDING_TARGET angles [PX4 receiver]. PX4 scales the ray by range height, and 1D Kalman filters track position and velocity. With LTEST_MODE stationary, the target velocity also goes into EKF2 [PX4 precland]. PX4 gives about 10 cm with IR-LOCK. The vehicle approaches at height until within PLD_HACC_RAD (default 0.2 m), then descends. Below PLD_FAPPR_ALT (0.1 m) it keeps descending even if the target is lost. The mode needs a valid global position [PX4 precland], so indoors the EKF origin must be set first [EKF2 source]. It always ends in a landing. PX4 has no hover-over-target mode (E: none in its docs). ArduPilot has one, Precision Loiter, on aux function 39 [ArduPilot PL].

**PX4 Vision Target Estimator (v1.18, experimental).** It runs at 50 Hz with per-axis filters plus a yaw filter. It takes TARGET_RELATIVE marker reports and replays samples up to 500 ms late. It is off in default builds and must replace the old estimator [VTE]. With vision only, it drifted about 60 cm over 7 s after a dropout, because of a 0.1 m/s velocity error [VTE deep dive]. UWB and IR-LOCK inputs are future work [VTE].

**Fiducials, measured.**
- ArUco with a Pi camera on ArduPilot landed 11 cm off on average (3 to 18 cm), against 85 cm on GPS [Wubben].
- AprilTag pose error averaged about 1 cm at 30 to 70 cm. It reached 16 cm off-axis at an oblique viewing yaw, and a geometric fix cut a 40 cm case to 5 cm [Abbas].
- Relative error was under 3% within 2 m and about 0.5% within 1 m [Tello].
- A Pi 5 runs 2 MP tags at 20 fps with about 200 ms latency, for 5 cm-level indoor fixes [aprilmav].
- On tag range, the survey's 5 to 10 cm worst case conflicts with the Tello result.

**IR beacons.** The IR-LOCK sensor weighs 21.5 g and the kit costs $349 [IR-LOCK]. MarkOne V3 is detected out to 15 m in any light [ArduPilot IR-LOCK]. PX4 warns that many IR rangefinders misbehave near the beacon [PX4 precland], and the MTF-02P ToF works at 940 nm [MTF-02P], so they may conflict (E). UAVLAS publishes no centimetre figure [UAVLAS].

**UWB.** Nooploop quotes 10 cm in 2D and 30 cm in 3D, typical [Nooploop]. Bitcraze calls UWB decimetre class [Bitcraze]. Calibrated DW1000 ranging scattered 2.6 cm σ, but with ground anchors the vertical fix was about 10x worse [IMAV 2018]. PX4 has an SR150 driver and new EKF2 range-beacon fusion that needs each beacon's lat/lon [PX4 UWB]. Good for getting home, not for the last 10 cm (E).

**Flow and range (MTF-02P).** The ToF is accurate to 2 cm out to 2 m, with a 2° beam. Flow works from 8 cm, with a 42° view, and needs at least 60 lux [MTF-02P]. When range is the height reference, PX4's origin moves with the ground. Over an obstacle the range can be rejected, and after 5 s of disagreement the height resets to it [PX4 EKF].

**Ground effect and recirculation.** A single rotor's gain fades by h/R ≈ 2 and reaches +33% at h/R 0.5 [Lee]. A full multirotor feels it out to 5 R. One rotor over an edge creates a moment that pushes the craft away, and a PID can oscillate in and out of the effect [Sánchez-Cuevas]. Crossing a table 8 cm up, a PID quad rose 5 to 10 cm, then sank 10 cm; disturbance rejection held it to 2.5 cm [Hentzen]. A learned model cut the z error crossing a table from 15.3 to 2.7 cm [Neural Lander].

**The last 20 cm elsewhere.** Docks land within ±15 cm, then push the legs (dock and Skydio reports). Wubben hands over to the autopilot at 0.30 m [Wubben]. An arm catcher reached 6 cm RMS and 2.5°, about 95% over 40 tries, and disarms only after the latch is confirmed [Arm catcher].

## Steal for Nox
- Box catcher: send the box camera's tag pose to EKF2 as external vision (EKF2_EV_CTRL bits 0 and 3, EV delay from logs) while the drone is over the head [PX4 EV].
- Scout: the Radxa already runs uXRCE-DDS, so it can relay the box pose over WiFi onto /fmu/in/vehicle_visual_odometry [PX4 DDS].
- Box catcher: an upward ToF under the head centre measures ring height for the z gate (E).
- Box catcher: put a flat, textured surface at a fixed depth under the head and light it to at least 60 lux, so flow and ToF read the same thing on every catch (E).
- Firmware (both): make "altitude only" an offboard setpoint. Set a z position, set x/y position and velocity to NaN, and set x/y acceleration to 0. The XY integrators then stay frozen [PositionControl]. PX4 Altitude mode needs live stick input [PX4 Altitude].
- Firmware: take yaw from the box tag. Neither drone has a compass, so the 20° gate depends on it (E).
- Firmware: always fuse a velocity source alongside vision. Keep flow on [VTE deep dive].

## For Spark
- `nox-dimensions.md` §4 Catch trigger "z ±8": the v2 V-jaw row says it "captures the rod from z -33 to +18". Hover the ring about 7 mm below the seat and that window is about ±25, so the gate could open to about ±20 and pass far more often (E). ±8 is also near the MTF-02P's 2 cm accuracy [MTF-02P], so measure z on the box side (E).
- §4 V-jaw "capped at z 24 (12 under the props)": the flank tops close at the edge of the Scout's disks in plan, 12 under the sweep. Expect a small lift change as they arrive (E).
- `scout.md` "`EKF2_RNG_CTRL 2`, `EKF2_HGT_REF 2`" (the Overseer params match): crossing the box edge will step or reset height. Settle in hover over the head before the gate, or feed box height as EV height (E).
- `overseer.md` catch frame: the long bars sweep under the S500 disks while closing, and the prop height above the ring is unmeasured. With props closer than about 1 D (254) above the bars, expect a lift change (E).

## Sources
- [PX4 precision landing doc](https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/advanced_features/precland.md)
- [PX4 Vision Target Estimator](https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/advanced_features/vision_target_estimator.md)
- [PX4 VTE deep dive](https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/advanced_features/vision_target_estimator_advanced.md)
- [PX4 receiver: mavlink_receiver.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_receiver.cpp)
- [EKF2 source: ekf.h](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/ekf2/EKF/ekf.h)
- [PX4 EKF: tuning the ECL EKF](https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/advanced_config/tuning_the_ecl_ekf.md)
- [PX4 EV: external position estimation](https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/ros/external_position_estimation.md)
- [PX4 DDS: dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml)
- [PositionControl.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_pos_control/PositionControl/PositionControl.cpp)
- [PX4 Altitude mode](https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/flight_modes_mc/altitude.md)
- [PX4 UWB: EKF2 ranging beacon params](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/ekf2/params_ranging_beacon.yaml)
- [ArduPilot PL: Precision Landing and Loiter](https://ardupilot.org/copter/docs/precision-landing-and-loiter.html)
- [ArduPilot IR-LOCK](https://ardupilot.org/copter/docs/precision-landing-irlock.html)
- [IR-LOCK sensor kit](https://irlock.com/products/ir-lock-sensor-precision-landing-kit)
- [UAVLAS (Unmanned Systems Technology)](https://www.unmannedsystemstechnology.com/2020/12/high-precision-autonomous-landing-system-for-uavs-developed/)
- [Wubben et al., Electronics 2019](https://www.mdpi.com/2079-9292/8/12/1532)
- [Abbas et al., AprilTag state estimation, Sensors 2019](https://www.mdpi.com/1424-8220/19/24/5480)
- [Tello: AprilTag positioning, arXiv 2305.05673](https://arxiv.org/pdf/2305.05673)
- [aprilmav](https://github.com/stephendade/aprilmav)
- [Nooploop LinkTrack](https://www.nooploop.com/en/linktrack/)
- [Bitcraze positioning](https://www.bitcraze.io/documentation/system/positioning/)
- [IMAV 2018: UWB with DW1000](https://www.imavs.org/papers/2018/30.pdf)
- [MTF-02P (Rotorama)](https://www.rotorama.com/product/micoair-mtf-02p)
- [Lee et al., JFM 2026](https://www.cambridge.org/core/product/B57E828030CE5C29E4315DF0F05D96D8/core-reader)
- [Sánchez-Cuevas et al., IJAE 2017](https://www.hindawi.com/journals/ijae/2017/1823056/)
- [Hentzen et al., arXiv 1908.03166](https://arxiv.org/pdf/1908.03166)
- [Neural Lander, arXiv 1811.08027](https://arxiv.org/pdf/1811.08027)
- [Arm catcher, arXiv 2505.09145](https://arxiv.org/html/2505.09145v2)
