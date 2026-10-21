# Nox research queue

Driven by a continuous research loop in Spark's research chat. Spark, 11 Oct 15:37: keep running in every direction until he says stop. Spark, 11 Oct 20:58: ramp up, and stop Monday 12 Oct at 7:00 am when his usage limit resets (am assumed). Waves of 10 agents in parallel, one topic and one report each at `claude/research/<YYYY-MM-DD>-<slug>.md`, up to 60 searches+fetches each, every wave time-boxed to ~90 min. WebSearch is capped at 200 per turn, shared by all agents, so each wave runs in its own turn (the loop logs a wave, schedules a 1-minute wake-up, and the next wave starts with a fresh budget; 18 WebSearch per agent, WebFetch for the rest). A scheduled stop fires Mon 12 Oct 7:00; the last wave ends by 6:40. Between waves the loop alone edits this doc: items to Done, follow-ups straight into Open, one Digest line per report, one Run log line per wave. The Mon-Fri scheduled task "Nox research" stays paused.

Spark: add, reorder or delete items freely. Top of Open = next wave. Items starting `!` jump the queue. Other chats may add items, Done or Digest lines; they are kept.

Focus (Spark, 11 Oct): tech first. Competitors' hardware and designs Nox can borrow, then every other direction. Business, market and regulation items sit lower but still run.

Rules for reports:
- Research only. Never edit `nox/` docs, numbers or parts. A finding that suggests changing a shared number or part goes under "For Spark" in the report, tagged E, with the source, naming the Nox doc and quoting the Nox value it was compared against.
- Competitor items: the mechanism (how it actually works, with patents or teardowns where found), the key numbers, and a "Steal for Nox" list naming the Nox part, one line each.
- Every number carries its source, or E. No em dashes. Short, under ~1000 words.
- Primary sources first: datasheets, papers, maker docs, patents, teardowns, product pages.
- Check `claude/*-research.md` and `claude/research/*.md` for overlap first, and build on them.

## Running now (wave 4, 11 Oct ~22:50 to ~00:20)
- Two-drone RC and failsafe setup: ELRS Model Match, second handset, COM_RCL_EXCEPT, box-side kill path, datalink-loss action, MAV_SYS_ID and DDS namespaces (comms, Scout, Overseer firmware)
- PX4 EKF2 without GPS or compass: yaw from external vision or a box tag, EV dropouts and resets, height-reference switching over the box edge (Scout, Overseer firmware)
- Box camera tag pipeline: upward camera tracking a tag on a drone 0.3 to 1.5 m up, latency and pose accuracy, relay latency to PX4 (Box X11)
- Ground effect and downwash at the catcher: sparse bars and closing jaws under 3.5 in and 10 in props (Box catcher, Overseer catch frame)
- Thin carbon-strip rib cages: flexural data, arch buckling and inward crumple, hook mounts, weights (Scout v2 cage)
- Overseer lidar mount: L2 field of view vs props and frame, tilt, motorized sweep (Overseer)
- In-flight collision recovery on caged drones: Flyability US12134488B2 anti-tip, contact-recovery controllers, PX4 options (Scout firmware)
- Inspection payloads that sell on small drones: thermal, gas, UT, zoom; masses, prices, what customers pay for (Scout, Overseer, business)
- US drone sourcing rules: FCC Covered List, NDAA, Blue UAS, state bans; what they mean for the MicoAir FC, Unitree L2, ELRS and Radxa (business)
- Manway and hatch sizes in target facilities vs the Scout cage (277.4 across corners) and a Scout v2 envelope target (Scout v2 cage, pitch)

## Open
1. Box charger control: iCharger 456DUO / X6 / X8 MODBUS map vs the 4010DUO (sold out), ISDT CM1620 CAN / RS-485 protocol and balance-port behaviour, cheaper per-slot CC/CV modules (Box X10; from fast charging and balance/ID reports)
2. Inspection data delivery and export: Orbit, Korial Connect, Gecko Cantilever, Flyability Inspector, Skydio 3D Scan, Nearthlab grading, ScoutDI API into integrity systems, point-cloud viewers; report contents and CMMS/EAM links customers expect (data product)
3. Radxa Zero 3W hardware: Wi-Fi chip, 5 GHz, AP and monitor mode, throughput with the u.FL antenna; 40-pin I2C buses vs UART3 on pins 3/5; Linux / ROS 2 drivers for VL53L1X / L7CX on the RK3566 (Scout; from comms and standoff reports)
4. ROS 2 over Wi-Fi for two drones and the box: Zenoh vs a Fast DDS discovery server, multicast cost, uXRCE-DDS namespaces (comms, firmware; from comms and multi-drone reports)
5. Downwash and wake of a 2 kg S500 on a 0.5 kg caged quad: safe horizontal and vertical separation indoors, measured data (Overseer, Scout; from multi-drone report)
6. Unitree L2 firmware and timing: SDK issue #25 half-rate time base, IMU stamp gaps, 2.8.x firmware notes; how to bench-measure L2-to-Pi latency (Overseer; from Pi 5 SLAM report)
7. Market: will Ondas/Airobotics + Cyberhawk and Beam/ScoutDI dock-plus-data combos move indoors; what they charge per site or subscription; facility inspection buyers and budgets (business, Box pitch; from inspection players report)
8. Batteries for small inspection drones with swaps: LiPo vs HV LiPo vs Li-ion (21700, pouch) vs semi-solid; energy density, C-rate, cycle life at a swap-every-flight cadence (cartridge, Overseer)
9. Motor and prop choice for 3 to 3.5 in caged quads: efficiency data, KV, prop pitch, flight time per gram (Scout, Scout v2)
10. Smart-BMS 4S packs that charge through the main leads and report over CAN or SMBus, so a CM1620-style charger works and the bays need no balance pins (cartridge, Box X5; from fast charging report)
11. Pack temperature after a 4 to 5C hover and time to cool below ~35 °C; bay airflow (Box charger, swapper; from fast charging report)
12. Pin-keyed vs push-push latches for a bottom pack: hold force, misuse retention, pusher overtravel (Overseer cartridge X9; from swap mechanisms report)
13. Robot-mateable signal pads or connectors for 5 to 6 low-current contacts in ~12 x 12 mm, plus a cheap NFC reader (PN532 / RC522) beside metal and LiPo (cartridge v19 pads, Scout gripper; from balance/ID report)
14. px4-ros2-interface-lib external modes on the Radxa: build, latency, maturity of velocity setpoints, SITL failsafe test on v1.18.0-rc1 (Scout firmware; from standoff report)
15. PX4 collision prevention fed from a companion obstacle_distance stream on v1.18 (path marked untested): SITL and bench reports (Scout firmware; from multi-drone report)
16. Glass and mirror detection for facility demos: multi-return ToF (VL53L4CX / L7CX histograms) vs a few-gram ultrasonic sensor (Scout v2, cage; from standoff report)
17. Loop-closure backends on the box (GLIM CPU, pose graph + Scan Context) fed with Overseer submaps; output map formats (Box, data product; from Pi 5 SLAM report)
18. Relay options: Overseer as relay or a box-dropped relay; 802.11s mesh on Pi 5 USB radios vs Doodle Labs Nano and Silvus SL5200; measured hop throughput, latency, real quotes (Overseer, Box; from comms report)
19. Swap-station papers: full texts of Oulu 2024 (5 failures in 200 swaps) and Ure 2015 / Michini 2011 (MIT drum swap): failure causes, hot-swap power path, latch design (Box swapper, test plan)
20. Swap and capture patent claims: DJI US9139310B1 family (US9346560B2, US9440545B2, US10363826B2, US11332033B2) and US11708003B2; Skydio US12673792B2 / US12649593B2; Brookhurst US12286252B2 / US12312105 / US12522387 (and who Brookhurst Garage is); Percepto US10507938B2; H3 US11912438B2; Krauss/Kliner US20200165008A1; against the box swapper, iris, D5 sled, a rotary Overseer magazine and Scout v2 contacts (Box IP; claim pages cut off in waves 2 and 3, try other mirrors)
21. CNIPA and other Chinese databases: DJI dock battery-swap filings 2020-2026 (Box IP; from DJI swap dock report)
22. Measured cell drift of hobby 4S LiPos cycled without balancing at 1C to 2C, to check the 1.1 to 1.6 mV/cycle model (Box charger policy; from balance/ID report)
23. Quick-turn PCB fabs that plate 30 µin hard gold on ordinary pads (not just edge fingers): cost and lead time, vs Mill-Max gold target contacts (Scout v2 pads, Overseer cartridge pads)
24. Leaf and blade battery contacts (TE, Molex, Harwin, Preci-Dip, Würth): current, force, wipe and cycles vs pogo pins for blind-mate packs (Scout v2 and Overseer cartridges)
25. Rod-band keep-alive contact: gold flex vs plated brass sleeve on 3 and 4 mm carbon rods, plating cracks at r 1.5, wear against PTFE-taped jaws (Box catcher, Scout v2 cage, Overseer catch frame)
26. Flyability US11661188B2 family (continuations, EP and WO members): do break-first TPU nodes or snap-on bumpers read on its claims 31 and 33? (Scout cage v9, S2 cage)
27. Synchrophasing on a quad: can AM32 (or Bluejay) report or hold rotor angle well enough (~5°) for hover-only phase lock under PX4? (from noise report 11 Oct)
28. MicoAir743 AIO power tree: can the 5 V or VBAT pads be back-fed from a box keep-alive, battery sensing, schematic or vendor docs (Scout, Overseer keep-alive)
29. Fraunhofer RAPID full papers (OCEANS 2023; IEEE Access 2024, DOI 10.1109/ACCESS.2024.3353046): actual keep-alive power path and contact design (Box swapper)
30. Reverse-rotation thrust of 3 to 3.5 in props: published static thrust data, to check the 0.4 reverse fraction used for Scout turtle (Scout firmware)
31. Small-quad indoor positioning without GPS: optical flow, VIO on Radxa-class boards, UWB
32. Lightweight lidar alternatives to the L2 for Overseer: specs, price, ROS/PX4 support
33. Cage designs for collision-tolerant small quads (academic): geometry, materials, crash energy data
34. TPU 95A vs other flexible filaments for impact parts: printed property data, layer adhesion
35. Defect detection at the edge: crack and corrosion models that run on a Radxa NPU or Pi 5, public datasets (software, data product)
36. Manufacturing scale-up: printed vs injection-molded vs MJF for cages, cartridges and box parts; cost at 10 / 100 / 1000 units (business, tech)
37. Funding for a drone startup: SBIR/STTR, NSF, DoD programs, drone-focused investors 2025-2026, UW resources (business)
38. Safety standards that buyers ask about: ANSI/UL 3030, ASTM drone standards, ATEX/IECEx for hazardous facilities (later)
39. Regulations: FAA rules for commercial indoor and BVLOS facility inspection, Part 107 waivers for a demo (later)

## Ideas and tests for Spark (from reports; not research items)
- Firmware: PX4 DShot patch sending reverse commands 20/21, run as disarmed motor-test bursts; SITL test on release/1.18 (PX4 crash-flip report)
- Bench: EMAX ECO II 2004 with Gemfan 3520 on 4S, reverse vs forward thrust and current (PX4 crash-flip report)
- Bench: PX4 battery estimate across a powered swap; then BAT1_CAPACITY -1 or a patch (keep-alive report)
- Box: seated check by commanding a small yaw torque with the jaws shut and reading IMU rotation (Skydio report)
- Box: the box as shared map origin, every takeoff from a known pose (Voliro/ANYbotics/Exyn/Emesent report)
- Overseer: L2 + Pi on a quick-release pod, pre-map the demo room by hand (same report)
- Scout v2 cage: ASIO-style flat carbon ribs from top ring to bottom ring, hook mount, 1 mm strip bend coupon (small caged drones report)
- Box keep-alive: 13.0 V current-limited feed through gold spring pins in the iris V-jaw apex onto bands on the ring rods (keep-alive report)
- Box: keep bay contacts covered until a rod or cartridge seats (Spot Dock cones that lower to bare the connector), with an interlock (ground robots report)
- Box: a head-free flag both drones check before approach; hold if occupied, as Spot skips an occupied dock (ground robots report)
- Data product: one mission list with tasks assigned by reach; inspection points set on the Overseer map, each visit compared with that point's baseline; a lidar map diff of what moved (ground robots report)
- Box: wired offload of video and logs while the drone is held (Corvus); re-scan only where the Overseer map changed (Verity) (indoor fleet report)
- Charger: size pack count with Verity's beehive math, packs = 1 + charge time / (flight + swap time) (indoor fleet report)
- Scout cage v9 / S2: fix ribs or rods at both ends and lace each net bay to its own rails; keep rods bare and put the compliance in nodes and mounts (patent design-arounds; patent report)
- Box catcher: a micro-switch at each V-jaw apex as a seat check (patent report)
- Overseer swapper: 6 x Mill-Max 0854 per pole on hard-gold cartridge pads, two coned dowels do the locating; no floating plug (contacts report)
- Box keep-alive: two 0854 per V-jaw at the apex, tips 1.9 proud, each pin wired separately so the box can self-test the contact before a swap (contacts report)
- Scout v2: hard-gold PCB pad strips (30 µin or more) in place of the brass pads (contacts report)
- After 14 Oct 8 AM EDT: re-read DJI's swap dock spec page (swap time, pack count, size, mass, price, latch, whether the drone stays powered) and add a row to `catcher-speed-research.md` §1 (DJI swap dock report)
- Pitch: after 14 Oct a battery swap alone is not novel; lead with the indoor hover catch, two drone types, a swap with no reboot and a timed turnaround (DJI swap dock report)
- Overseer swapper: rotary pack carousel turning each bay under a fixed Z pusher at the catch point; 4 bays fit ~0.4 to 0.5 m across; under 270° of turn so bay wires wrap with no slip ring (DJI swap dock report)
- Charger: cut a bay's current before its pack is pulled, keep idle spares near 65%, fill only the next pack due; box script X8 checks a full pack is ready before a catch, else holds and reports (DJI swap dock report)
- Overseer cartridge: put the latch release in two cone-mouthed bores only the pusher's pins reach (US20190127085A1; cones align to 1 mm) (swap mechanisms report)
- Box catcher: close the iris at low current to center, then raise current to hold; read the bar gap from the step count at stall as the caught-or-missed check (Percepto US10507938B2) (swap mechanisms report)
- Test plan: 200 timed swaps with every failure cause logged; 20 clean catches only bound the failure rate under ~15% (swap mechanisms report)
- Data product: grade each finding 1 to 5, trend it per visit, set a repair-by window (Nearthlab), push it by API into the customer's integrity system (ScoutDI) (inspection players report)
- Business: multi-year site subscriptions sold through NDT firms (Acuren, Mistras), as Avestec and Skygauge do; one remote operator supervises both drones through the box (Sees.ai) (inspection players report)
- Box power: give the chargers their own LRS-600-24; keep the LRS-350-24 for motion, logic and keep-alive, since an overload sends it into hiccup and could reset the SKRs mid-swap (E) (fast charging report)
- Charger: iCharger DUO on MODBUS driven by the laptop script (start/stop, cell voltages, IR, temps, cycle count); charge at 2C to ~4.15 V/cell; log IR and retire packs on a rise (fast charging report)
- Box X5 for v18 cartridges: one CC/CV buck per slot off 24 V at 16.6 V, ~3 A, with an INA226; XT60 only, resting-voltage check, mAh cap, bench balance every ~10 cycles; drop the 6 balance pins (balance/ID report)
- Cartridge v19 / Overseer X9 / Scout v2: recessed B-, C1, C2, C3, B+ and ID pads only the bay pins reach (~1 g), plus an NTAG213 sticker read by the gripper for cycle counts (balance/ID report)
- Box catcher: box camera tag pose sent to EKF2 as external vision (XY and yaw, since neither drone has a compass), relayed by the Radxa; upward ToF under the head for the z gate; textured surface under the head lit to 60 lux or more (precision landing report)
- Firmware: a box-commanded catch as an offboard z-only setpoint (XY NaN, XY accel 0), since PX4 Altitude mode needs live sticks (precision landing report)
- Box: the catch point is the map origin; a drone that boots squared in the iris with the mag off starts its EKF in the box frame; the 20° catch yaw gate removes the square ring's 90° ambiguity (multi-drone report)
- Scout firmware: the Radxa turns the box map slice plus the Overseer's pose into 72 obstacle sectors for PX4 collision prevention; a feed older than 0.5 s stops the Scout (multi-drone report)
- Comms: one Zenoh bridge per drone with a topic allowlist and rate caps, DDS kept on localhost; MAV_SYS_ID, DDS key and namespace 1 and 2 (multi-drone report)
- Overseer: FAST-LIO2 on the Pi with the L2's own IMU; a bridge node levels by gravity, applies the mount rotation and sends FRD VehicleOdometry with velocity, EKF2_EV_CTRL 15; reset_counter at each takeoff; Pi RTC battery and chrony, 10 s L2 warm-up, stiff mount, raw L2 logging (~1 GB/h), loop closure on the box (Pi 5 SLAM report)
- Scout firmware: "Wall Hold" ROS 2 external mode on the Radxa on one switch, vx = 0.8 x (range - 0.52 m) capped at ±0.3 m/s, with two 0.5 g VL53L1X in short mode (forward beside the lens, up on the Radxa mount) on Radxa I2C; never SENS_EN_VL53L1X; zero-code fallback CP_DIST 0.52 + CP_GO_NO_DATA 1, which only stops (standoff report)
- Comms: ELRS 2.4 GHz for sticks and kill, one handset per drone; one 5 GHz Wi-Fi AP at the box for MAVLink, ROS 2 and video (~2 g, 0.5 to 1 W per drone); a second wired AP on a 10 to 20 m cable as a range extender; Wi-Fi power save off; hold if operator packets stop for 0.5 s; video only from the selected drone; Scout ELRS at 250 Hz LoRa, not F1000 (comms report)

## Corrections to older research
- `claude/research/2026-10-11-flyability-elios-tech.md`: said Elios motor reversal flips it back upright. Its patent US12134488B2 reverses props in flight to stop tipping after a hit and does not describe ground righting. Fixed in that report (source: PX4 crash-flip report).
- `claude/cage-clean-sheet-research.md`: assumed 5 in props where Scout flies 3.5 in; called Flybotix ASIO duct-as-airframe, but primary sources show a spar cage with no duct (small caged drones report).
- `claude/catcher-speed-research.md` §4: puts the keep-alive contact on the catcher forks, retired 9 Oct; the iris V-jaws are the place now (keep-alive report).
- `claude/twin-arm-swap-concept.md`: "box current -> 0 = seated" goes true when the XT60 pins touch, before the cartridge seats; pair it with the hall trip in `scout-swap-arm.md` (keep-alive report).
- `claude/research/2026-10-11-dock-swap-mechanisms.md`: the pushers, marker sensor and seat sensors it puts under US12286252B2 are the subjects of Brookhurst's same-day siblings US12312105 (alignment) and US12522387 (pack seat sensors) (patent report).
- `claude/cage-clean-sheet-research.md` and `nox/scout-cage.md` v9 cite Flyability US20200189712A1, which granted 30 May 2023 as US11661188B2; its claim 1 covers soft-layer beams (under 50% modulus, 3 to 30% thick), wider than the recipe quoted (patent report).
- "Fine for a demo" (dock report) and "prototype OK" (`nox/scout-cage.md` v9): the US experimental-use defence is narrow and an investor demo is commercial, so patent risk is low, not exempt (E, ask counsel) (patent report).
- `claude/research/2026-10-11-keepalive-swap-power.md` "2 per pole (3 on the Overseer)": one iris catches both drones, so the jaw pin count can't differ by drone; two per pole carry 18 A against a 2 to 3 A limit (contacts report).
- `claude/overseer-compatibility-audit.md` "~25 A hover / 11-12 min" assumes 1,860 g AUW; `nox-dimensions.md` §4 has 2.0 kg, so hover current is likely ~7% higher (E) (indoor fleet report).
- Energy Robotics is now Korial (energy-robotics.com redirects to korial.com) (ground robots report).
- `claude/research/2026-10-11-patent-landscape.md`: misses DJI US11708003B2 (granted Jul 2023, active to ~2036: pushers on perpendicular rails move a landed drone to an exchange zone and an arm reaches up through a hatch; claim 1 needs the plate, hatch and door) and Airobotics-inventor US20200165008A1 (raisable boundary around a hovering vehicle); its US9139310B1 entry leaves out rotating pack stores, which bear on the `box.md` D5 sled (DJI swap dock and swap mechanisms reports).
- `claude/research/2026-10-11-spot-gecko-ground-robots.md` For Spark (DroneXL headline "not opened"): now opened; DJI confirms only the 14 Oct date and slogan, and "Dock 4" is a leaked name (DJI swap dock report).
- `claude/catcher-speed-research.md` §1: Hextronics times disagree across sources (Universal / Global 80 s vs Global full refuel under 5 min; Atlas 60 s spec vs a 90 s average on the same page); the table needs a DJI swap dock row after 14 Oct (swap mechanisms and DJI reports).
- `claude/research/2026-10-11-dock-swap-mechanisms.md`: Heisha "found arms costlier and less stable", but Heisha has since announced a T200 with battery swapping (press release, no hardware details) (swap mechanisms report).
- `claude/research/2026-10-11-indoor-fleet-drones.md`: treats 2C as a ~30 min recharge; with the constant-voltage tail a full 2C charge takes ~33 min (E) (fast charging report).
- `claude/hover-catch-mechanism-survey.md`: calls PX4 Altitude mode the compliant mode, but Altitude mode needs live manual control, so a box-commanded catch needs an offboard z-only setpoint; its 5 to 10 cm raw tag range error conflicts with arXiv 2305.05673 (~0.5% under 1 m) (precision landing report).
- `claude/research/2026-10-11-voliro-anybotics-exyn-emesent.md`: raw L2 data is ~2.5 Mbit/s (E), not ~8 Mbit/s (Pi 5 SLAM report).
- `claude/overseer-scout-hardware-reference.md` §4 and `claude/overseer-wiring-plan.md` §4: "port 6101" is the L2's sending port; the Pi listens on UDP 6201 (Pi 5 SLAM report).
- `claude/overseer-scout-hardware-reference.md` §4: its EKF2 config of record (HGT_REF Vision, RNG_CTRL 1) conflicts with the flown params (HGT_REF 2, RNG_CTRL 2, EV_CTRL 0, SYS_HAS_MAG 0, MAG_TYPE 5); lidar-odometry flights need their own param file (Pi 5 SLAM report).
- `claude/overseer-scout-hardware-reference.md` §5: collision prevention "fed from the L2", but the params have CP_DIST -1 (off) and PX4 runs it in Position mode only; full SLAM on the box only works as a slow frame offset applied on the Pi, because EKF2 rejects external-vision samples more than 200 ms apart (multi-drone report).
- `claude/overseer-scout-hardware-reference.md` §1.5 "915/433 illegal to transmit on in the US": 902 to 928 MHz is a legal Part 15 band (47 CFR 15.247, up to 1 W); its §4 "WiFi 6E" ground link is barred, since 47 CFR 15.407(d)(1)(v) excludes 5.925 to 7.125 GHz for drone control or communications (comms report).
- `claude/research/2026-10-11-flyability-elios-tech.md` steal "Distance Lock from the flow/ToF sensors": Scout's only ToF (MTF-02P) looks down; a wall hold needs a forward ToF and companion code (standoff report).
- `claude/research/2026-10-11-small-caged-indoor-drones.md` upward-ToF ceiling hold: nothing in PX4's height control reads an upward sensor, so it needs companion code (standoff report).

## Nox doc conflicts found (for Spark; Nox docs not changed)
- SAFETY: `scout.md` Radio "Pocket model Scout = copy of the Overseer model": with ELRS Model Match off both drones may follow the same sticks; with it on, the unselected drone sees RC loss and lands (NAV_RCL_ACT 3, COM_RC_LOSS_T 0.5). Needs a second handset or a tested setup and a props-off check (comms and multi-drone reports)
- SAFETY: `overseer_params_2026-09-26.params` has MAV_SYS_ID 1, UXRCE_DDS_KEY 1, UXRCE_DDS_NS_IDX -1, and `scout.md` lists no system ID, so the Scout is likely also 1: IDs and /fmu topics clash once both are on one network. Scout needs ID 2 and each drone its own namespace (comms and multi-drone reports)
- SAFETY: Overseer params NAV_DLL_ACT 0: a Wi-Fi or datalink drop currently triggers nothing (comms report)
- Overseer power board: `nox-dimensions.md` §4 lists the FCHUB-12S, but BAT1_A_PER_V 78.43 in `overseer_params_2026-09-26.params` matches the 4-in-1 shunt in `overseer-wiring-plan.md`. Which is fitted? (keep-alive report)
- Scout ESC firmware: `scout.md` says Bluejay; ArduPilot's MicoAir743-AIO page lists AM32. Which is flashed? Both take reverse commands 20/21 (PX4 crash-flip report)
- `scout.md` Firmware: the PR #28826 fix is on PX4 release/1.18 but in no tag; v1.18.0-rc1 still has the bug (PX4 crash-flip report)
- `nox/v2/s2-contacts.md` calls ENIG "the later upgrade", but ENIG gold (2 to 5 µin) wears through in 10 to 20 sliding matings; contact pads need hard gold of 30 µin or more (contacts report)
- `nox/v2/s2-contacts.md` fallback Same Sky CPG: the CPG-91 datasheet rates it 12 Vdc, under a full 4S's 16.8 V; cycle rating 10,000 (datasheet) vs 100,000 (press release) (contacts report)
- `nox/v2/README.md` "Contact preload ≥ 12 N on the pack, always on" vs `nox/v2/dimensions.md` 7.9 N on the hooks; and `s2-contacts.md` Problem 2 "pins sit mid-stroke", while on the hooks they sit at 0.9 against Mill-Max's 1.14 mid-stroke (contacts report)
- `nox/v2/s2-contacts.md` Problem 5: a charge cradle with power pins only never balances S2 packs (balance/ID report)
- `nox-dimensions.md` §2 "Bay balance pins" / "Cartridge ID values", `box.md` X5 "6 spring pins" and `scout-cartridge-pocket.md` "Bays in the box" assume the pad PCB that cartridge v18 removed: v18 cartridges have no balance or ID path (indoor fleet and balance/ID reports)
- `box.md` power: one Mean Well LRS-350-24 (350 W) is too small if both drones' chargers run nonstop (~500 W average, ~600 W peak, E) (indoor fleet and fast charging reports)
- `box.md` X10: ISDT CM1620 for 7 bays via relays (+ ADCs, Nano). It is a single 20 A channel with no balance port; one unit shared through relays would have to refill a Scout pack every ~10 min (~5C, E), and charging a group in parallel would cross-charge full and empty packs (fast charging and balance/ID reports)
- `box.md` "Overseer 3: two full, one empty" with 3 slots (D5): nonstop at 2C needs 4 packs and a 4th slot; with 3, a launch waits ~3 min every third flight (fast charging report)
- `scout.md` "~7 min to 20% reserve" vs `nox-dimensions.md` §1 "~9 min": at 2C, 7 min needs 5 Scout packs and 9 min needs 4 (fast charging report)
- `nox-dimensions.md` §4 conflicts with itself: the "Catch envelope (iris)" and "Iris long bar" rows still describe the v1 lift onto ledges, while "Catcher concept" and "Catcher hold-down" say v2 has no lift, ledges or dogs; `box.md` "How it works" and "To buy" (4x MG90S) still describe v1 dogs and ledges (patent report)
- `nox-dimensions.md` §4 "Catch trigger" z ±8 is much tighter than the v2 V-jaw capture window (about ±25, E) and close to the MTF-02P's 2 cm accuracy (precision landing report)
- `scout.md` and the Overseer params use EKF2_RNG_CTRL 2 (range as the height reference): crossing the box edge can step or reset height (precision landing report)
- `scout.md` wiring puts Radxa UART3 on header pins 5 and 3; on the common 40-pin layout these are the I2C pins (CHECK the Radxa pinout) (standoff report)
- `scout.md` Problems (b) and (c) say the Radxa is boxed under the pack, but `nox-dimensions.md` §1 moved it to the rear cage triangle on 8 Oct (ground robots and comms reports)

## Done
- DJI's teased battery-swap dock. Report: `claude/research/2026-10-11-dji-swap-dock.md` (11 Oct, wave 3)
- Swap-station mechanisms and their patents. Report: `claude/research/2026-10-11-swap-station-mechanisms.md` (11 Oct, wave 3)
- Other inspection drone makers and services (Sees.ai, Nearthlab, Avestec, Terra Drone, Cyberhawk, Skygauge, ScoutDI). Report: `claude/research/2026-10-11-inspection-drone-players.md` (11 Oct, wave 3)
- Fast charging and the box power budget. Report: `claude/research/2026-10-11-fast-charging-box-power.md` (11 Oct, wave 3)
- Balance charging and pack ID without the pad PCB. Report: `claude/research/2026-10-11-balance-charging-pack-id.md` (11 Oct, wave 3)
- Precision hover and landing on a small target. Report: `claude/research/2026-10-11-precision-hover-landing.md` (11 Oct, wave 3)
- Multi-drone coordination. Report: `claude/research/2026-10-11-multi-drone-coordination.md` (11 Oct, wave 3)
- Lidar SLAM on the Pi 5 with the L2. Report: `claude/research/2026-10-11-pi5-l2-lidar-slam.md` (11 Oct, wave 3)
- Wall standoff and ceiling hold on PX4. Report: `claude/research/2026-10-11-standoff-ceiling-hold.md` (11 Oct, wave 3)
- Indoor comms for multi-drone ops. Report: `claude/research/2026-10-11-indoor-comms.md` (11 Oct, wave 3)
- Ground-robot competitors: Spot/Orbit, Korial (ex Energy Robotics), Gecko. Report: `claude/research/2026-10-11-spot-gecko-ground-robots.md` (11 Oct, wave 2)
- Indoor fleet drones: Verity, Corvus, Gather AI, Ware, Tando, Corax. Report: `claude/research/2026-10-11-indoor-fleet-drones.md` (11 Oct, wave 2)
- Patent landscape: swap stations, hover capture, cages. Report: `claude/research/2026-10-11-patent-landscape.md` (11 Oct, wave 2)
- Swap contacts and blind-mate connectors. Report: `claude/research/2026-10-11-swap-contacts.md` (11 Oct, wave 2)
- Skydio X10 / R10 and Skydio Dock. Report: `claude/research/2026-10-11-skydio-x10-dock.md` (11 Oct, wave 1)
- Voliro, ANYbotics, Exyn, Emesent Hovermap. Report: `claude/research/2026-10-11-voliro-anybotics-exyn-emesent.md` (11 Oct, wave 1)
- Small caged and ducted indoor drones under 500 g. Report: `claude/research/2026-10-11-small-caged-indoor-drones.md` (11 Oct, wave 1)
- Crash-flip (turtle) recovery on PX4. Report: `claude/research/2026-10-11-px4-crash-flip.md` (11 Oct, wave 1)
- Keep-alive power through the catch and swap. Report: `claude/research/2026-10-11-keepalive-swap-power.md` (11 Oct, wave 1)
- Drone docks with automatic battery swap or charge (DJI Dock 2/3, Skydio Dock, Percepto, Heisha, Hextronics, RAPID + patents). Report: `claude/research/2026-10-11-dock-swap-mechanisms.md` (11 Oct)
- Flyability Elios 3 (and Elios 2) tech: cage, collision tolerance, lidar/SLAM, lighting, battery swap. Report: `claude/research/2026-10-11-flyability-elios-tech.md` (11 Oct)
- Indoor drone noise reduction and on-board anti-noise (ANC), asked by Spark in chat. Report: `claude/research/2026-10-11-indoor-drone-noise.md` (11 Oct)

## Digest (newest first)
- 2026-10-11, Indoor comms: keep ELRS 2.4 GHz for sticks and kill with one handset per drone, and run MAVLink, ROS 2 and video over one 5 GHz Wi-Fi AP at the box (~2 g, 0.5 to 1 W per drone, E); NIST plant data shows blockage, not echoes, sets range (~25 to 35 m Wi-Fi, 100 to 200 m ELRS behind machinery, E); mesh radios (7 to 123 g, 4 to 13 W) are a later Overseer or box item.
- 2026-10-11, Wall standoff and ceiling hold: stock PX4 can do neither (collision prevention only brakes in Position mode and skips up/down sensors; EKF2 uses only the first downward rangefinder), so the Scout demo needs a ROS 2 external mode on the Radxa fed by two 0.5 g VL53L1X in short mode; glass and mirrors are the main sensing risk.
- 2026-10-11, Pi 5 lidar SLAM with the L2: compute is likely enough (FAST-LIO2 at 5 to 55 ms per scan on older ARM; ~5,200 points per L2 cloud); the real risk is L2 timing (no hardware sync, arrival-time stamps, half-rate time base reports); send levelled FRD pose plus velocity to EKF2 with EV_CTRL 15 and loop-close on the box.
- 2026-10-11, Multi-drone coordination: Nox doesn't need collaborative SLAM; the box catch point is the shared origin, PX4 v1.18.0-rc1 already has namespaces, aux position fusion and companion-fed collision prevention, and the box laptop is the brain with a Zenoh bridge per drone.
- 2026-10-11, Precision hover and landing: tags and IR beacons reach ~5 to 11 cm, UWB 10 to 30 cm, flow ~9 cm σ plus drift, so all fit the iris ±150 envelope; cheapest is a box camera tag pose fed back as external vision plus a box-side upward ToF; the risks are the height-reference step at the box edge, lighting and texture under the head, and lift changes as the jaws close.
- 2026-10-11, Balance charging and pack ID: for v18 cartridges, charge through the XT60 only, one charger per slot at 4.15 V/cell, resting-voltage check, mAh cap, bench balance every ~10 cycles; ID by slot tracking now, NFC later; balance and ID pads return on the next carriers.
- 2026-10-11, Fast charging and box power: both pack classes take 2C or more and faster charging cuts cycle life; nonstop flying at 2C to ~4.15 V/cell needs 4 Scout and 4 Overseer packs on ~5 channels, ~500 W average and ~600 W peak (E) against the 350 W PSU; only the iCharger 4010DUO publishes a control protocol (MODBUS).
- 2026-10-11, Inspection drone players (Sees.ai, Nearthlab, Avestec, Terra Drone, Cyberhawk, Skygauge, ScoutDI): acquirers pay for recurring inspection data (Ondas-Cyberhawk ~$125M, Beam-ScoutDI ~$24M); every job found had a crew on site, and none docks indoors, swaps packs or pairs a mapper with a close-in drone, so Nox's opening is unattended, repeated indoor rounds from a box.
- 2026-10-11, Swap-station mechanisms: no commercial swapper publishes hardware; patents and lab rigs all center, grip a feature built into the pack, push to a spring latch, then confirm by switch or contact; published swaps take 60 s to ~2 min, and the only failure count found is 5 in 200 (Oulu 2024).
- 2026-10-11, DJI swap dock: DJI confirms only an Enterprise launch on 14 Oct 2026, 8 AM EDT; leaks suggest a carousel of 5+ packs lifting one into the drone's top-rear bay; names, mass and the ~$30k price are unconfirmed; after 14 Oct the Box pitch should rest on the indoor hover catch, two drone types, a no-reboot swap and a timed turnaround.
- 2026-10-11, Swap contacts: spring pins on hard-gold pads (1,000,000 cycles) beat floating plugs (100 to 200 cycles); keep the Mill-Max 0854 on Scout v2, reuse it for the Overseer cartridge (two coned dowels locate) and the iris V-jaw apex, and use hard gold of 30 µin or more, not ENIG or brass.
- 2026-10-11, Patent landscape: a fixed rib or rod cage with nets most likely falls outside Flybotix US12275542B2 claim 1; Flyability US11661188B2 (soft-coated beams, break-first modular joints) is the nearer cage risk; no claim found on bars that clamp a hovering drone with no pad; the Skydio 2024 hub and Brookhurst sibling claims are still unread.
- 2026-10-11, Indoor fleet drones (Verity, Corvus, Gather AI, Ware, Tando, Corax): the leaders build their own drones, charge in place and publish no mass or dock specs; Verity's 10 min flight per 30 min charge explains its four-drone docks; safety is separation plus climb-to-avoid, no certification found; nonstop Nox needs ~2C charging and ~500 W of chargers (E), more than the box's 350 W supply.
- 2026-10-11, Ground robots (Spot/Orbit, Korial (ex Energy Robotics), Gecko): Spot sells 90 min walks, self-docking, zoom/thermal/acoustic/gas sensing and per-point change detection; drone + ground robot pairings share software and data but no dock, so a box that catches and swaps two drone types has no commercial match; Gecko's money is coverage-priced service plus a per-asset finding list.
- 2026-10-11, Keep-alive power: on-drone electrolytics can't bridge a swap (5 s needs 0.41 F, ~1 kg) and a supercap string is a ~20 g fallback for 5 to 7 s; best path is a 13.0 V current-limited box feed through gold spring pins in the iris V-jaws onto ring-rod bands (~5 g on Scout), and PX4 carries the old pack's mAh count onto the new pack.
- 2026-10-11, PX4 crash-flip: PX4 has no DShot turtle mode (v1.18.0-rc1, release/1.18, main), but Bluejay and AM32 take the temporary reverse commands 20/21, so a ~200-line DShot patch run as disarmed motor-test bursts gives Scout turtle; each far motor needs ~141 gf reversed (E).
- 2026-10-11, Small caged indoor drones: Avata 2, IBIS2 and Dronut build the guard into the airframe (165 to 212 mm, 243 to 440 g) and publish no crash rating; Flybotix ASIO's composite spar cage is ~10% of craft mass, crumples inward and hooks off, which puts the v2 55 g cage budget in reach.
- 2026-10-11, Voliro/ANYbotics/Exyn/Emesent: commercial lidar SLAM pods weigh 1.6 to 2.4 kg against the Overseer's L2 + Pi 5 (~305 g, E), and contact inspection needs tilt-rotors; the steals are multi-robot methods: box as shared map origin, compact submaps, frontier handoff to the Scout, contacts powered only after a seated check.
- 2026-10-11, Skydio X10/R10/Dock: the dock doesn't rely on a precise landing (99.91% of dock landings succeeded, Q2 2026); fences push the legs to center, a rising hub mates the battery with angled horns, two pins, magnet-centered floating contacts and a sprung bail lock, and seating is confirmed by commanding torque and seeing no IMU rotation.
- 2026-10-11, Indoor noise: anti-noise speakers on a drone are directional, tones-only and too heavy (best 2026 study: loudness only 6.5 to 8.5% lower); real wins are a lighter Scout, 48 kHz ESC PWM, smooth tune, tractor props, bigger slower props; synchrophasing is the only "cancellation" worth a later look.
- 2026-10-11, Drone docks: everyone lands then centers passively (ramps, V-grooves) or by pushing legs, and DJI/Skydio/Heisha charge in place via leg contacts rather than swap; best steal is ASU's polarity-free V-groove contact with a diode bridge for the box keep-alive.
- 2026-10-11, Flyability Elios 3/2: the crash trick is motor-reversal flip recovery plus SLAM-in-the-loop stabilization on a TPE-suspended rigid cage (2.4 m/s omni x100 hits); top steals are a Scout flip-recovery mode and a retract-on-impact camera mount. Corrected 11 Oct: the motor reversal is in-flight anti-tip, not ground righting (PX4 crash-flip report).

## Run log (newest first)
- 2026-10-11, wave 3 (10 agents, 21:05 to 22:35): all 10 reports written and read back; 46 to 67 searches+fetches each. WebSearch turned out to be capped at 200 per turn, shared by all agents; it ran out around 22:20, so the last agents finished on WebFetch and cloned repos. Fix: each wave now runs in its own turn via a 1-minute scheduled wake-up, with 18 WebSearch per agent and WebFetch for the rest. Common blocks: Google Patents search (robots.txt) and claim pages cut off, verity.net and docs.px4.io render empty, ScienceDirect, TUM and Stuttgart repositories, maker sites for Tattu, CNHL and HOTA.
- 2026-10-11, wave 2 (6 launched ~18:00): 2 were declined at launch (camera mounts/lighting, catcher safety), not re-run unless Spark asks; 4 ran and all 4 reports were written and read back; 45 to 53 searches+fetches each, slowest 2.5 h. Blocked: verity.net renders empty; Skydio and Brookhurst patent claims cut off on Google Patents and Justia; USPTO ppubs empty; curl to patent sites refused by the proxy; a BigQuery patents check was denied. Spark paused the loop, then at 20:58 asked to ramp up with a stop at Mon 7:00. Wave 3 brief: 10 agents, cap 60, 90 min deadline.
- 2026-10-11, wave 1 (5 agents, 15:38 to 17:50): all 5 reports written and read back. 36 to 64 searches+fetches each, over the 40 target on 3; slowest agent 2.2 h. Workarounds: project_write rejects a scratchpad local_path, inline content works; docs.px4.io renders empty, read the PX4 repo; GitHub search API blocked, git clone works; IEEE and Fraunhofer publica blocked. Wave 2 brief: cap 45, inline writes, skip a source after 2 failed tries, report corrections found.
- 2026-10-11, test firing: did 2 items (Elios, docks) in one session because the test message re-triggered the job; ~4 min per item; Projects read/write worked. Fixed: one item per firing, follow-ups moved out of Open, Run log added, 5 runs/week at 25-40 searches.
