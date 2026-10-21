# Voliro, ANYbotics, Exyn, Emesent: inspection robots and multi-robot workflows (11 Oct 2026)

**Bottom line**
1. Lidar SLAM pods weigh 1.57 to 2.4 kg without battery [Hovermap brochure, Nexys sheet], 5 to 8x Overseer's L2 + Pi 5 (~305 g, E).
2. Voliro T presses probes on walls at up to 30 N [Voliro solution], vectoring thrust with tilting rotors [Uptime]. Nox's plain quads must tilt to push, so contact inspection is not near-term (E).
3. The steals are SubT and ANYmal workflow patterns: a known launch pose so maps share a frame, compact submaps, the small robot sent where the mapper's frontier ends, contacts live only once seated.

## Findings

**Voliro T.** T-shaped tri-copter whose rotors tilt independently, so it can push on a surface without tilting its body [Uptime]. 6.0 kg MTOW, 1.0 kg payload, 10 to 14 min, ±90° pitch, ±2 cm contact accuracy [Voliro spec]; sources disagree on whether its 3.7 kg includes batteries [Voliro spec, MFE]. In LPS tests the pilot flips a switch 0.3 to 0.6 m out; the drone closes in, holds constant force until the reading is accepted, then backs off [Uptime]. Payloads of 0.10 to 0.67 kg: UT, EMAT, DFT, PEC, and an LPS needle probe on a 250 m tether [Voliro payloads] (about 300 m per [Uptime]) to a ground ohmmeter. UT needs gel couplant; EMAT needs none and reads across a 4 mm gap [Voliro payloads]. The LPS probe is angle-compliant and presses at a constant 30 N [MFE]. Subscription or lease, no public price [Uptime, MFE]. Force control is unpublished; ETH's 2018 Voliro used geared-BLDC rotor tilt and a proportional force loop [Kamel 2018], and a later ETH rig estimated contact force without a sensor (1.8 N) [Bodie 2019].

**ANYmal D / X.** D: 50 kg, 932 Wh swappable pack, 90 to 120 min (2 to 3 h per [EPFL talk]), 3 h charge, 5.7 kg pan-tilt head (zoom, thermal, ultrasonic mic, light) [ANYmal D spec]. X reaches Zone 1 ATEX with a nitrogen-pressurized body [ANYmal X spec]. On low battery it docks, charges and resumes [ANYmal D spec]. Docking: SLAM approach, four QR codes refine the fix, it squats onto a low rear port, and the dock goes live only once comms confirm it is seated [UST]. Contacts wear, and cm-level positioning forces compliance into the dock [EPFL talk]. A separate legged-robot dock patent tests contacts at 5 V through 200 Ω, then 1 A, expects under 1 mΩ, and stops near 60 °C [US12560937]. No list price found.

**Emesent Hovermap ST-X.** 1.57 kg, 32 channels, ±10 mm indoors; one pod for drone, vehicle, backpack, handheld or ground robot [Hovermap brochure]. From $70,310 with a dovetail quick-release [Freefly]. Pilot Assist feeds lidar SLAM to the flight controller inside an elliptical keep-out shield; Autonomy takes taps or drawn 3D zones on the live cloud, explores, and keeps a battery-safe return, beyond comms range [Emesent modes].

**Exyn Nexys Pro.** 2.4 kg (3.1 kg with battery), Hesai XT32 on a ~1 Hz gimbal, Core i7, 75 W, ±1 cm [Nexys sheet]. Level 4 (Exyn's own scale): the operator draws a volume; the drone flies to the frontier view with the most new data until nothing is unknown, then flies home on its map, at up to 2 m/s with no GPS or comms [IEEE Spectrum]. It backs out of heavy dust [Exyn tech]. No public price.

**Multi-robot.** CSIRO (SubT): a ground robot carried a Hovermap drone and gave it a pose hint at launch; robots traded 5 s surfel frames at 164 to 270 kbit/s, store-and-forward [CSIRO]. CERBERUS: start-gate AprilTags set the shared frame, and a caged drone (5 to 6 min) took shafts and stairs [CERBERUS]. NTNU/ETH: an ANYmal carried a 1.4 kg collision-tolerant lidar drone (12 min) on an electropermanent magnet; an elastic strap pushes it off on release. It launched where ground frontiers ran out but higher ones remained, seeded with the walker's pose, and landed alone [Marsupial].

## Steal for Nox
- **Box catcher:** the iris centers and squares each drone, so takeoff starts at a known pose: make the box the map origin, tagged so it can be re-fixed if moved [CERBERUS, UST] (E).
- **Firmware (Scout/Overseer split):** send the Scout where the Overseer's reachable frontier ends [Marsupial].
- **Comms:** trade keyframe submaps, store-and-forward, not raw points [CSIRO].
- **Swapper and charging bays:** energize contacts only after a seated check, a low-current continuity test and a temperature cutoff [UST, US12560937].
- **Overseer:** L2 + Pi on a quick-release, so the same pod can pre-map the demo room by hand [Hovermap brochure, Freefly] (E).
- **Overseer firmware:** Pilot Assist first (L2 into PX4 collision prevention), then draw-a-box exploration; on low battery, return, swap, resume [Emesent modes, IEEE Spectrum, ANYmal D spec] (E).
- **Scout:** one switch to approach, hold a set standoff and back off [Uptime]; needs a forward ToF (E).

## For Spark
- `overseer.md` open item 2, "Lidar mount and its view past the props and the catch frame": the L2 sees 360 x 90° (Sept hardware reference), about a half-sphere, with no Nexys-style gimbal [Nexys sheet]. Tilt picks that half: forward-down at the nose sees the floor and the Scout's zone, not up or rear (E).
- Sept `overseer-scout-hardware-reference.md`: Pi 5 lidar odometry "has not been benchmarked", while Nexys runs a Core i7 [Nexys sheet]; replay an L2 log on the Pi first (E). Raw L2 at 64,000 pts/s (same doc) is ~8 Mbit/s (E, 16 B/point) vs 164 to 270 kbit/s frames [CSIRO]: send submaps to the box (E).
- `nox-dimensions.md` §2, "10.0 k pull-up to 3.3 V": the cartridge ID read could be the seated check before X10 charging energizes a bay, but cartridge v18 dropped the pad PCB (E).
- `scout.md`, "No onboard autonomy": products ship assisted modes first (Voliro's switch [Uptime], Emesent's tap-to-fly [Emesent AL2]); a Scout tap-the-map mode fits (E).
- `box.md`, "swap its cartridge by script, repeatably": ANYmal charges 3 h at its dock [ANYmal D spec] and the NTNU/ETH drone does not re-dock on its carrier, so catch-and-swap stands out. Overseer parts (~¥4,759 plus ~$300 to 400, Sept reference) cost under 2% of one $70,310 Hovermap bundle [Freefly], at lower accuracy (E).

## Sources
- [Voliro spec: Voliro T spec sheet v3.2](https://cdn.voliro.com/wp-content/uploads/2025/06/Voliro-T-Spec-sheet-v3.2.pdf)
- [Voliro payloads: Voliro T payloads spec sheet v3.2](https://cdn.voliro.com/wp-content/uploads/2025/11/Voliro-T_Payloads_Spec-sheet_v3.2.pdf)
- [Voliro solution: Stable interaction with structures (Voliro)](https://voliro.com/voliro-solution)
- [MFE: Voliro T ultrasonic NDT drone (MFE Inspection Solutions)](https://mfe-is.com/en/product/voliro-t-ultrasonic-ndt-drone)
- [Uptime: Inspection with Voliro contact-enabled drones (Uptime Wind Energy podcast)](https://weatherguardwind.com/inspection-voliro-contact-enabled-drones/)
- [Kamel 2018: Voliro, an omnidirectional hexacopter with tiltable rotors (arXiv)](https://arxiv.org/pdf/1801.04581)
- [Bodie 2019: An omnidirectional aerial manipulation platform for contact-based inspection (arXiv)](https://arxiv.org/pdf/1905.03502)
- [ANYmal D spec: ANYmal technical specifications (ANYbotics)](https://www.anybotics.com/anymal-technical-specifications.pdf)
- [ANYmal X spec: ANYmal X technical specifications (ANYbotics)](https://www.anybotics.com/anymal-x-technical-specifications.pdf)
- [EPFL talk: ANYbotics at the EPFL PELS IEEE workshop](https://www.epfl.ch/labs/pel/wp-content/uploads/2022/11/1_ANYbotics_PELS-IEEE-Workshop.pdf)
- [UST: ANYmal feature, UST magazine 40](https://ust-media.com/ust-magazine/UST040/32)
- [US12560937: Docks for legged robots and related technology (USPTO)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12560937)
- [Hovermap brochure: Hovermap ST range (Emesent)](https://es.emesent.com/wp-content/uploads/2023/02/Hovermap-ST-Range-brochure.pdf)
- [Freefly: Hovermap ST-X Astro Max autonomy bundle](https://store.freeflysystems.com/products/emesent-hovermap-st-x-astro-max-autonomy-bundle)
- [Emesent modes: Mission modes (Emesent knowledge base)](https://knowledge.emesent.com/docs/mission-modes)
- [Emesent AL2: Ready, Tap, Fly (Emesent)](https://cn.emesent.com/autonomy-level-2/)
- [Nexys sheet: Nexys Pro product sheet (KOREC)](https://www.korecgroup.com/wp-content/uploads/2025/04/Nexys-Pro_Product-Sheet-122024a.pdf)
- [Exyn tech: Autonomous lidar (Exyn)](https://www.exyn.com/our-technology)
- [IEEE Spectrum: Exyn brings level 4 autonomy to drones](https://spectrum.ieee.org/exyn-brings-level-4-autonomy-to-drones)
- [CSIRO: Team CSIRO Data61's approach to the DARPA SubT Challenge (arXiv)](https://arxiv.org/pdf/2104.09053)
- [CERBERUS: Legged and aerial exploration in the DARPA SubT tunnel and urban circuits (arXiv)](https://arxiv.org/pdf/2201.07067)
- [Marsupial: Marsupial walking-and-flying robotic deployment, NTNU/ETH (arXiv)](https://arxiv.org/pdf/2205.05477)
