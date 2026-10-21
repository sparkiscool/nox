# Flyability Elios 3 / Elios 2: tech and steal list (11 Oct 2026)

Corrected 11 Oct: the Elios motor reversal is in-flight anti-tip after a hit, not ground self-righting (Bottom line 2, Flip recovery, Steal list, For Spark).

**Bottom line**
1. Elios 3 is a rigid carbon cage with a soft coating, hung on TPE suspensions, built from modular pieces, with a bottom opening for the battery and a front opening for the payload. No gimbal cage since Elios 2. Cage-only mass is still unpublished.
2. Its real crash trick is software: fast-reversing motors that stop it tipping over after a hit, in flight (US12134488B2; ground self-righting is not documented), plus lidar + 3-camera SLAM fed into stabilization. Collision rating: 100 hits without damage at 2.4 m/s from any direction (2 m/s frontal), structural damage at 3 m/s (spec sheet).
3. Best steals for Nox: in-flight anti-tip on contact and ground turtle recovery on Scout (see `claude/research/2026-10-11-px4-crash-flip.md`), a shock-retracting camera mount, off-axis lighting panels, and the "battery out the bottom opening, <10 s swap" layout Overseer already follows.

Not repeated here (see `claude/cage-clean-sheet-research.md`): the suspension patent recipe (US20200189712A1), the 2 to 5x stiffness rule, GimBall numbers.

## Findings

**Cage and structure (Elios 3).** Carbon cage, soft coating, modular subcomponents, TPE suspensions; airframe in carbon-Kevlar, magnesium, aluminium and thermoplastics. 480 mm wide x 380 mm high (spec). Collision: frontal <2 m/s and any direction <2.4 m/s, each with no damage over >=100 hits; structural damage at 3 m/s; full-speed hits in ATTI/SPORT "will kill the drone" (spec). Elios 2: <400 mm sphere, 3 m/s flat / 1.5 m/s sharp, and the camera sits on a **retractable mount that absorbs frontal shocks** (Elios 2 spec). Field case: a full-speed tunnel hit broke one cage pentagon, nothing else (Inside Unmanned Systems).

**Flip recovery.** Flyability lists "patented collision-tolerant design" as cage + custom flight controller + propeller reversal, and claims it is the only drone that recovers from flips without crashing (product page). Motors are "fast-reversing brushless" (spec). Hobby equivalent: Betaflight turtle mode, which needs DShot, a working accelerometer, 180° arm angle, and clear props; stalled motors draw high current (Oscar Liang). Correction (11 Oct): the patent behind this, US12134488B2, reverses propellers in flight to stop tipping after an obstacle contact and does not describe ground righting, so "recovers from flips" most likely means mid-air (see `claude/research/2026-10-11-px4-crash-flip.md`).

**Gimbal: gone.** Elios 1's rotating cage is now a separate Flyability patent family, US11708160B2 (active outer-frame actuation, priority 2017, expires 2040 per Google). Elios 2/3 use a fixed cage and a camera pod that tilts -90° to +90° (both specs).

**Sensing and SLAM.** Elios 3: Ouster OS0-32 lidar, 3 vision cameras, ToF distance sensor, IMU, mag, baro, Nvidia Xavier NX (spec). FlyAware SLAM fuses cameras + lidar and tells the flight controller to cancel small drifts, i.e. SLAM is in the stabilization loop, not just mapping (Flyability press release). OS0 Rev 7: 447 g, 14 to 20 W, 90° x 360° FOV, ~35 m at 10% reflectivity, 0.5 m minimum range (Generation Robotics listing). Cost of the lidar on E3: +450 g (1,900 to 2,350 g) and flight time 12.5 to 9 min (spec). Surveying payload upgrade: OS0-128 Rev 7 with FARO Connect, volumes within 1% of terrestrial scans (Commercial UAV News). Elios 2 had 7 vision/distance sensors and Distance Lock holding 30 to 200 cm from a wall (Elios 2 spec).

**Lighting.** Elios 2: 10,000 lm, LEDs front, top and bottom, modes indirect/dustproof, close-up, oblique (spec). Elios 3: four panels, 20 W normal, 40 W boost, 100 W peak (~16,000 lm, temporary); dust mode lights only the two outer panels; oblique mode lights one side only; beam follows camera pitch (spec).

**Battery and swap.** Elios 3: 6S HV LiPo smart pack, 99.2 Wh, 620 g, plug-and-play swap <10 s through the cage's bottom opening, ~1 h charge, rated 50 flights (spec). Battery is ~33% of base AUW (620/1,900, E). High-capacity pack: 187 Wh, 1,100 g, 17 min (Nexxis). Elios 2: 5S, 98.8 Wh, 550 g, swap <1 min, 10 min flight (spec). A tether power unit removes the time limit (product page).

**Adjacent IP.** Flybotix (ASIO) holds US12275542B2 / EP4168310B1 (granted 2025): a foldable cage of arched ribs joined by end rings with a **string mesh** threaded through rib holes, ribs individually replaceable (WO2021260497A1).

## Steal for Nox
- **Scout:** in-flight anti-tip by reversing the far props on contact (what Elios actually does), plus ground turtle recovery as Skydio R10 ships; PX4 path in `claude/research/2026-10-11-px4-crash-flip.md`.
- **Scout camera mount:** retract-on-impact camera mount, as Elios 2, so a nose hit loads TPU, not the camera.
- **Scout / Overseer lights:** two outer light panels off the camera axis for dust; one-side oblique mode for crack/texture shots.
- **Scout cage:** modular, replaceable cage panels; the field data shows one panel failing as the fuse.
- **Overseer:** keep the lidar light. E3 lost 28% of flight time (E) to a 447 g sensor; the L2 at 230 g is the right class.
- **Overseer / box:** fuse lidar odometry into position hold (FlyAware pattern); a Distance Lock style wall-hold (30 to 200 cm) is a cheap demo feature from the flow/ToF sensors.
- **Box swapper:** E3 proves bottom battery access through the cage and <10 s plug-and-play is a sellable spec.

## For Spark
- `scout.md` (Firmware): Scout runs PX4 with Bluejay ESCs. PX4 has no DShot turtle mode; `claude/research/2026-10-11-px4-crash-flip.md` has the patch path. Elios's feature is in-flight anti-tip, not ground righting (corrected 11 Oct).
- `scout-cage.md` / cage v9 direction: Flybotix's granted rib + string-mesh cage patent sits next to the Dyneema-net idea in `cage-clean-sheet-research.md`; fine for a prototype, get a claims read before selling (E).
- `scout-camera-tilt.md` / camera mount: consider a sprung or TPU-hinged camera mount that folds back on a frontal hit (E, from Elios 2).
- `overseer.md` (Lidar mount, open item 2): OS0 on E3 sits on the front/payload opening; Elios carries the lidar as a front payload, consistent with the nose plan (E).
- Benchmarks for decks: Elios 3 rates 2.4 m/s omni, same as Scout's current 25 g standoff limit noted in `cage-clean-sheet-research.md`.

## Sources
- [Elios 3 tech spec PDF (Nexxis)](https://nexxis.com/wp-content/uploads/sites/3/2022/06/20220519_E3_Tech_Spec_web_version.pdf)
- [Elios 3 product page (Flyability)](https://www.flyability.com/elios-3)
- [Elios 2 spec PDF (MFE)](https://mfe-is.com/wp-content/uploads/2024/09/MFE-Rentals-Elios-2.pdf)
- [Elios 3 High Capacity Battery (Nexxis)](https://nexxis.com/product/high-capacity-battery-for-elios-3/)
- [Flyability Elios 3 press release (PRNewswire via Daum)](https://v.daum.net/v/20220520092031986)
- [Elios 3 drone (The Drone Girl)](https://www.thedronegirl.com/2022/05/19/elios-3-drone-flyability/)
- [Elios 3 Surveying Payload (Commercial UAV News)](https://www.commercialuavnews.com/surveying/flyability-and-faro-technologies-announce-new-elios-3-surveying-payload-for-centimeter-accuracy-in-challenging-environments)
- [Elios 2: Assessing Danger in a Cage (Inside Unmanned Systems)](https://insideunmannedsystems.com/elios-2-assessing-danger-in-a-cage/)
- [US11708160B2, Flyability (Google Patents)](https://patents.google.com/patent/US11708160)
- [US12134488B2, Flyability (Google Patents)](https://patents.google.com/patent/US12134488)
- [WO2021260497A1, Flybotix foldable cage (Google Patents)](https://patents.google.com/patent/WO2021260497A1/en)
- [Ouster OS0 Rev 7 (Generation Robotics)](https://www.generationrobots.com/en/404054-ouster-ultra-wide-fov-os0-lidar-rev-7.html)
- [Elios 3 UT Payload (JGC)](https://www.jgc.gr/products/flyability/elios-3-ut-payload/?lang=en)
- [Turtle mode setup (Oscar Liang)](https://oscarliang.com/setup-turtle-mode-flip-over-after-crash/)
