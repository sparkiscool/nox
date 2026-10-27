# Precision landing accuracy: what error must the box mechanism take? (27 Oct 2026)

Research only; Nox docs not changed. Every number carries a source tag [S#] (URL list at the end) or E (estimate, mine). "Not found" means searched and nothing published; it is stated rather than guessed.

## Bottom line
1. Real-flight touchdown errors in the literature sit at 7 to 14 cm (mean to worst case, often with mocap), not 2 to 5 cm. The 2 to 3 cm figures are all simulation [S9, S10].
2. The best indoor flight dataset found (25 landings, VICON, "near-optimal conditions", a larger UAS) has a 2 sigma lateral error of 0.11 m and longitudinal 0.08 m [S8]. That is ±110 mm at about 95 %, from a good setup.
3. PX4 states "roughly 10 cm" for IR-LOCK landing [S1]; its default acceptance radius to start the descent is 0.2 m [S1]. Commercial pads that rely on mechanical forgiveness quote 200 mm (compact) to 400 mm (modular) [S15].
4. Yaw is the weak spot. ArduPilot's precision landing has no yaw-to-target feature (only a sensor mounting yaw) [S4, S5]; PX4 yaw alignment needs a custom firmware build with the Vision Target Estimator, off by default [S1, S6]. Yaw to the pad therefore has to come from the companion computer or be absorbed by the mechanism.
5. Ground effect and touchdown bounce/slide: little is published at 7 inch scale on a 0.6 m deck. Mostly E; see sections 5 and 6.
6. Verdict: ±40 mm and ±5 deg are the best case, not the planning case. Plan the mechanism for ±80 mm and ±10 deg (E), keep the commit gate at ±30 to ±40 mm so most approaches stay inside the proven funnel capture.

## 1. PX4 precision landing
- Accuracy: IR-LOCK "landing with a precision of roughly 10 cm" [S1]. No flight-test touchdown table is published; the Vision Target Estimator page lists no accuracy numbers at all [S6].
- Parameters (defaults): `PLD_HACC_RAD` 0.2 m (horizontal radius before descent starts), `PLD_BTOUT` 5 s (target timeout), `PLD_FAPPR_ALT` 0.1 m (final approach altitude) [S1].
- Final approach: below `PLD_FAPPR_ALT` the vehicle keeps descending while centred, "regardless of the landing mode" [S1]. A forum reply says that if the target is lost below that height the vehicle just continues to land with no search [S2].
- Target lost earlier: after `PLD_BTOUT` it either searches (required mode: climb to `PLD_SRCH_ALT`, up to `PLD_MAX_SRCH` tries) or lands where it is (opportunistic) [S1]. So a drone that loses the tag in the last metre lands where it drifted: the mechanism takes the drift.
- Yaw: `PLD_YAW_EN` makes yaw follow the target orientation, but only in a build with VTE and `VTE_EN`, `VTE_YAW_EN` set; off by default; VTE is beta, not in default board builds [S1, S6].
- Descent speed: the page gives no descent rate [S1].
- User report (PX4 1.15.4 + IR-LOCK, no rangefinder): "not as accurate as expected", no cm figure; another user claims "a few centimeters" with a different product (Landmark) and attributes it to tuning (autotune) [S2].
- Known caveat: many IR rangefinders perform poorly near the IR-LOCK beacon [S1]. Offboard `LANDING_TARGET` must use `MAV_FRAME_LOCAL_NED`, x, y, z only [S1].
- IR-LOCK detection runs about 25 Hz (algorithm 50 Hz, beacon not always on) [S3].

## 2. ArduPilot precision landing
- Docs claim "centimeter-level accuracy" with no measured table [S4]. No indoor measurement published there.
- Source types: `PLND_TYPE` 1 = MAVLink `LANDING_TARGET` (companion AprilTag/ArUco), 2 = IR-LOCK [S4]. Companion pipelines are expected to send about 10 Hz [S7].
- Descent logic: `PLND_XY_DIST_MAX` blocks descent when the target is too far off; `PLND_ALT_MIN` keeps a vertical descent if the target is lost below that height; `PLND_STRICT`/`PLND_RET_MAX` retry or hover; final descent may be slowed below `LAND_SPD_MS`, bit 2 of `PLND_OPTIONS` allows a faster final descent [S4].
- Yaw: no yaw alignment to the target. `PLND_YAW_ALIGN` is only the camera mounting yaw [S4]. A forum thread shows yaw motion only during retry, toward the last known target direction, and the workaround is to disable retries; aligning to the tag rotation was unresolved, a Lua script was only mentioned [S5].
- Rangefinder: "a noisy rangefinder will degrade performance"; needs a valid horizontal position estimate and steady attitude [S4].
- What users measure: no cm numbers found. Reports are qualitative: "landing error remains too big", inconsistent, target-lost hover wrong, heavy vibration; fixes were autotune, lower `JERK_XY`, check the AprilTag pipeline sends 10 Hz [S7]. IR-LOCK era reports: about 30 cm "as advertised" by one user, jumpy corrections on 3.6.x, hover about 1 m above the beacon [S11].
- Thessaly thesis (ArduPilot, IR-LOCK and a nested ArUco marker, moving vehicle): average landing error 6.79 and 8.51 cm, worst case 13.86 and 11.60 cm for its two subsystems [S12] (as quoted in a search snippet; the PDF was unavailable).

## 3. Fiducial landing with a downward camera on a companion computer
- JPL indoor test (AprilTag2, 48 cm tag bundled with three 15 cm tags, 25 landings, VICON ground truth, 0.9 x 0.9 m pad): 2 sigma landing ellipse 0.11 m lateral, 0.08 m longitudinal; outdoors with light wind 0.37 m and 0.28 m [S8]. Height error 2 sigma 0.32 m at 4 m, below 0.1 m in the last stage [S8]. Authors blame pixel quantisation from vehicle motion and a lever-arm effect from angular tag errors [S8].
- Tag detection range: 48 and 60 cm tags are not detected below 2 m; 10, 15, 20 cm tags stop above 4 m [S8]. A near-field tag must be small: nested tags are needed for a full descent.
- Nested bundle in flight: 564 g quad, 5 x 5 cm tag nested in a 14 x 14 cm tag, 30 fps camera, a 60 x 60 cm pad in leaf-blower turbulence; the paper reports tracking, not a touchdown cm table [S13].
- Real-flight ArUco: mean landing error 0.657 m with 16 cm markers from 2 to 4 m (GPS-assisted outdoor) [S14]; a field system landed within 60 cm on average against 25 cm in SIL/HIL, blamed on GPS drift and wind in the last descent [S16].
- Simulation-only numbers: mean 2.99 cm X and 5.59 cm Y for AprilTag are simulation [S17]; MuJoCo 2.9 cm mean over 40 of 40 [S9]; 2.4 to 3.4 cm on a 1.5 m pad across three simulators [S10]. Do not use these for planning.
- Marker pose accuracy itself: AprilTag3 yaw error 1.24 deg at 2 m, 2.9 deg at 3 m, position 1.6 to 3.4 cm at 2 m [S18]. Pose yaw errors of 3.4 to 4.0 deg are reported in another comparison [S19]. So even a perfect controller cannot hold yaw better than a few degrees from a tag.
- Effect of a 20 to 40 cm pad with a nested tag: a pad this size keeps the tag in view until about 5 to 10 cm (E, from the 15 to 20 cm tag ranges above); a nested inner tag of 5 cm [S13] covers the last 10 cm. No paper isolates the gain in touchdown error from nesting; no number found.
- Late tag loss is the failure mode that matters: below roughly the camera's near limit the estimate is dead-reckoned (optical flow or lidar odometry) and drifts. The VTE docs state that disabling GNSS velocity causes visible relative-position drift when vision drops out [S6]; indoors there is no GNSS, so drift comes from the flow or lidar estimate (E).

## 4. What commercial docks accept
- DJI Dock 3 specs: no landing accuracy or tolerance published. Only hover accuracy ±0.1 m horizontal with RTK, ±0.3 m with vision positioning [S20]. DJI says landing uses RTK plus markers on the pad and on the cover [S21].
- DJI Dock 2: "slide-ramp centering design further enhances landing success rates"; no number [S22]. The pusher-rod docks in the patents found are third party [S23].
- Skydio Dock for X10: AprilTag-style fiducial on the landing surface; no tolerance published; landing success rate is a published metric but no figure found [S24]. An earlier repo note records a Skydio patent claiming a 5 x 5 cm target and a funnel cup [`claude/research/2026-10-14-fastest-landings-catchers.md`].
- Hextronics Universal: ±15 cm landing (from the repo note above, vendor sources).
- BOLO S1 charging pad: "up to a 200 mm landing error" compact, "up to a 400 mm" modular, no moving parts; says vision, IR, RTK and UWB solutions typically give "less than 100 mm" for small and medium drones [S15]. Vendor marketing, not measured.
- Academic magnet dock: 5 cm misalignment accepted [S25] (as summarised by a search; not verified).
- Reading: nobody with a published number relies on better than about ±100 mm. Docks that publish a tolerance accept 150 to 400 mm.

## 5. Ground effect and prop wash (7 inch class, last 20 cm, 0.6 x 0.6 m box)
Published data: thin. Nothing found that measures lateral drift in the last 20 cm over a deck in an enclosure of this size. What exists:
- Ground effect on thrust is measurable out to h/R = 6 [S26]. For a 7 inch prop (R about 89 mm, E) that is about 0.5 m, so the whole last 20 cm is inside it.
- A landing study measured the vehicle hovering about 0.13 m above the commanded height at touchdown until a ground-effect compensator was added [S27].
- Ground effect plus wall proximity "significantly affect" quadrotors, with larger disturbance in slow flight [S28].
- Intermittent ground effect: a classic controller pitched and overshot about 10 deg, a thrust-force controller kept about 2 deg [S29].
- ArduPilot ground-effect compensation lowers barometer weight near the ground and "reduces the bounce sometimes seen when landing vehicles with short legs compared to the length of their propellers"; leave it off if there is no bounce [S30]. It is an altitude-estimate fix, it does not cut motors.
- Not found: a published motor-cut height, fast final-descent profile, or mesh deck result for multirotors. ArduPilot's `PLND_OPTIONS` bit 2 (faster final descent) and PX4's `PLD_FAPPR_ALT` are the nearest firmware hooks [S1, S4].
- Estimates (E, mine): the 7 inch prop wash on a 0.6 m deck re-circulates off the box walls; the lateral gust is a few tenths of m/s, which a position loop at the end of its authority turns into a few cm of drift in the last 0.3 s. Allow 20 to 30 mm of extra drift on top of the pre-hover error (E). Mitigations in order: command the last 10 cm as a committed fast descent (about 0.3 m/s) with the kill on first contact, a perforated or mesh deck (vent the wash; E), tag-based yaw/XY held to the last visible frame, then open loop.

## 6. Touchdown bounce and slide on a hard deck (PETG, carbon tube feet)
- Not found: any measurement of PETG on carbon tube touchdown or friction.
- Related measured data: a fast-landing study on an F450 drops at 1 to 3 m/s vertical, ±1 m/s horizontal, and reports that hard impact plus leg flexibility makes the drone bounce [S31]. A reverse-thrust landing study found thrust reversal nearly doubles the landable slope and minimises bounce, on low and high friction surfaces [S32]. Neither gives a slide distance on a flat deck.
- This project's own assumption: bare PETG μ about 0.3 to 0.4 (E) [`nox/box/box-funnel.md`], which is why the wheels, not friction, centre the drone.
- Estimate (E): sliding distance on touchdown from lateral speed v at μ 0.35 is v^2/(2 μ g); at 0.15 m/s that is 3 mm, at 0.3 m/s 13 mm. A bounce of a couple of cm with 0.15 m/s drift adds a few mm. So at the planned speeds slide is small next to the pre-touchdown error; the risk is a hop with tilt, not distance.
- Failure risk with a land-detector disarm: PX4 and ArduPilot disarm a delay after landing detection [S33]; a noisy touchdown can trip the detector early or hold the motors on late. Not measured here.

## 7. What the repo already assumed
- Funnel design capture: ±40 mm offset, ±5 deg yaw, 0.15 m/s drift in and out, 2 deg tilt; 204 MuJoCo drops 100 % at μ 0.4 and 0.7 [`nox/box/box-funnel.md`].
- Full flight-physics check: every drop inside a ±30 gate centred (300 of 300); at 40 off drifting outward the drone reaches the face 46 to 61 off, past the mouth, and hangs; a taller mouth only reaches 97 %; best fix is to commit only inside ±30 (open) [`nox/box/box-funnel.md`].
- Prior research: 7.9 cm mean and 11.9 cm max errors in mocap fixed-time landings; suggested ±40 and ≤ 0.3 m/s with no hold [`claude/research/2026-10-14-fastest-landings-catchers.md`].

## 8. Does the literature support ±40 mm and ±5 deg?
- Position: partly. ±40 mm is inside the 7.9 cm mean of the mocap case and below the 7 to 9 cm means (worst 12 to 14 cm) of real flights [S12, 14 Oct note], and below the JPL 2 sigma of 110 mm [S8]. So ±40 is about a median-to-good outcome, not a 95 % bound. The gate (±30 to ±40) is what makes it safe: it does not assume the drone is that accurate, it refuses to land otherwise. The cost is retries, so the gate rate (E: 30 to 60 % of first tries inside ±40 on a Pixhawk without mocap) sets the time per swap.
- Yaw: not supported. Tag yaw alone is off by 1 to 4 deg [S18, S19]; ArduPilot has no yaw align [S4, S5]; PX4's needs a custom build [S1, S6]. With the FC's own heading hold indoors (compass poor next to the pack, motors, steel; E), ±5 deg is only reachable if the companion commands yaw from the tag. Plan ±10 deg, and make the funnel proof against ±15 deg for a failed lock (E).
- Approach speed: 0.15 m/s lateral is within normal hover quality (DJI vision hover ±0.3 m [S20]); no published touchdown speed for docks. Keep 0.15 m/s lateral at the commit gate, ≤ 0.3 m/s vertical at contact (E).

## Recommended landing error budget for the mechanism
| Item | Plan for | Commit gate (firmware) | Basis |
|---|---|---|---|
| Position radius at touchdown | **±80 mm** (E) | ±30 to ±40 mm | JPL 2 sigma 110 mm [S8], means 7 to 9 cm / worst 12 to 14 cm [S12], +20 to 30 mm ground effect drift (E) |
| Yaw | **±10 deg**, survive ±15 (E) | ±5 deg | tag yaw 1 to 4 deg [S18, S19], no FC yaw align [S4, S5] |
| Approach speed | ≤ **0.15 m/s** lateral, ≤ **0.3 m/s** vertical at contact (E) | same | repo sim [`nox/box/box-funnel.md`], slide 3 to 13 mm (E) |
| Tilt at contact | ≤ 5 deg (E) | n/a | repo used 2 deg; ground effect pitch kicks of 2 to 10 deg [S29] |

One line: no published system shows better than about 110 mm at 95 % on real hardware, so the mechanism takes ±80 mm and ±10 deg (a design that works only inside ±40 mm and ±5 deg is a best-case design), and the firmware gate refuses a commit outside ±30 to ±40 mm and ±5 deg so most landings are easy and only a bad one retries.

Check on the funnel numbers (open item for the box owner): the existing sim passes 100 % only inside ±40 and fails at 40 off drifting outward in full flight physics. Re-run the full-physics catch at ±80 offset, ±10 deg yaw, 5 deg tilt, 0.15 lateral, 0.3 vertical before adopting this budget; if it hangs, the gate must stay at ±30 and the retry rate matters more than the funnel.

## Sources
- S1 https://docs.px4.io/main/en/advanced_features/precland
- S2 https://discuss.px4.io/t/px4-1-15-4-ir-lock-precland-accuracy-search-issues/47247
- S3 https://discuss.px4.io/t/a-couple-of-questions-about-precision-landing/28887 (IR-LOCK about 25 Hz, from a search summary)
- S4 https://ardupilot.org/copter/docs/precision-landing-and-loiter.html
- S5 https://discuss.ardupilot.org/t/fixed-yaw-precision-landing/85618
- S6 https://docs.px4.io/main/en/advanced_features/vision_target_estimator
- S7 https://discuss.ardupilot.org/t/precision-landing-tuning/118323
- S8 https://arxiv.org/pdf/1810.05683 (Long-Duration Autonomy for Small Rotorcraft UAS including Recharging; read from the PDF text)
- S9 https://github.com/Saireddy21072007/uav-apriltag-landing-sim (simulation)
- S10 https://github.com/Gayathri-Vaishnav/Drones_S5_CD4_Apriltags_quaternion_Landing- (simulation)
- S11 https://discuss.ardupilot.org/t/copter-3-6-9-precision-landing-issue-ir-lock-solved/43329 and https://discuss.ardupilot.org/t/irlock-makes-copter-hover-1-m-above-target-during-landing/20776
- S12 https://ir.lib.uth.gr/xmlui/bitstream/handle/11615/53952/21845.pdf (search snippet only)
- S13 https://arxiv.org/pdf/1909.11071 (Paris et al.; tag bundle, 564 g quad, read from PDF text)
- S14 https://www.researchgate.net/publication/386890420_Computer_Vision-Driven_Precision_Landing_in_Quadcopter_Drones_Using_ArUco_Markers (search snippet; the 0.657 m figure came from a ResearchGate listing of a related paper, not verified)
- S15 https://www.skycharge.de/drone-charging-pad
- S16 https://arxiv.org/pdf/2505.12176
- S17 https://publikasi.mercubuana.ac.id/index.php/sinergi/article/view/34439 (simulation)
- S18 https://arxiv.org/pdf/2502.03510
- S19 https://doi.org/10.3390/s21051829
- S20 https://enterprise.dji.com/dock-3/specs
- S21 https://www.dji.com/support/product/dock-3
- S22 https://www.heliguy.com/blogs/knowledge-base/how-is-the-precision-landing-of-dji-dock-2-achieved/
- S23 https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10507938 (third party patent)
- S24 https://www.skydio.com/dock/faqs
- S25 https://arxiv.org/pdf/2604.23074 (summary only)
- S26 https://link.springer.com/chapter/10.1007/978-3-319-65283-2_19
- S27 https://www.researchgate.net/publication/281693006_Autonomous_Landing_of_Quadrotor_based_on_Ground_Effect_Modelling
- S28 https://arxiv.org/pdf/2109.04918
- S29 https://arxiv.org/pdf/2607.10099
- S30 https://github.com/ArduPilot/ardupilot_wiki/blob/master/copter/source/docs/ground-effect-compensation.rst
- S31 https://www.researchgate.net/publication/395794700_Friction_Shock_Absorbers_and_Reverse_Thrust_for_Fast_Multirotor_Landing_on_High-Speed_Vehicles
- S32 https://www.researchgate.net/publication/343030620_Improving_Multirotor_Landing_Performance_on_Inclined_Surfaces_Using_Reverse_Thrust
- S33 https://docs.px4.io/main/en/advanced_config/land_detector and https://autotest.ardupilot.org/Parameters/Copter/Parameters.html

Limits of this research: no source measures lateral drift in the last 20 cm of a 7 inch drone in a 0.6 m box, nor PETG/carbon touchdown friction; the bounce and drift allowances above are estimates. Several papers were read only through search summaries (marked); the JPL and Paris papers were read from the PDF text.
