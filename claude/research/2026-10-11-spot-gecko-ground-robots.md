# Spot, Korial (ex Energy Robotics), Gecko: what ground robots give a facility buyer (11 Oct 2026)

**Bottom line**
1. Spot sells a closed loop: 90 min walks, self-docking, zoom, thermal, acoustic and gas payloads, and Orbit, which flags acoustic change per inspection point [Spot SDK, Robot Report 5.1]. Nox can't match its endurance or payload; its openings are turnaround, equipment tops and tight gaps (E).
2. The drone + ground robot pairings found (Shell on Korial, DroneDeploy, Percepto, Gecko) share software and data; each robot keeps its own dock or none. None catches and swaps two drone types in one box (E).
3. Gecko sells robots as a service priced by asset coverage, plus a data platform subscription [Sacra]. The product is the per-asset finding list, not the robot (E).

## Findings

**Spot.** 32.5 to 33.8 kg, 500 mm wide, 14 kg payload, 90 min typical, IP54 [Spot SDK, Spot page]. Pack figures disagree: 605 Wh with a 120 min charge [Spot SDK] vs 564 Wh with 60 min [Spot page]. Base price $74,500 in 2020 [Tecnoblog]; dock, cameras and Orbit are quote only [Spot page].

**Spot Dock.** 22.9 kg, 58 V 12 A, gigabit Ethernet for data; 80% in 50 min at 25 °C but 2.5 h at 35 °C. On low battery Spot walks to a free dock, then resumes its work [Dock sheet]. It reads the dock's unique fiducial, fits a dock model to its point cloud, and plans steps on a 1 cm map. It stands over two cone-topped towers and lowers its body until a belly connector on its centreline meets the rear tower's terminal. A dock that scores badly (occupied, or not a dock) is skipped [US20220194245]. The cones lower as Spot docks, baring the connector; an interlock keeps it covered otherwise. Docking ends powered off (motors, E), and the docking service never powers Spot back on [Spot docking docs].

**Payloads and Orbit.** Spot CAM+IR: 30x PTZ, 640 x 512 thermal at ±5 °C, 7.5 kg [CAM+IR sheet]. Spot CAM 2 (Jan 2026): 25x 4K PTZ tilting to 90° up, 640 x 480 thermal, 9.5 kg [CAM 2 sheet], plus a bay for an acoustic imager [Robot Report 5.1] such as the 64-microphone Fluke SV600 for air, gas and steam leaks [SV600 sheet]. A Blackline detector reads LEL, H2S, CO, SO2 and ammonia; rising LEL sends Spot home [Robotics Tomorrow]. Orbit 5.1 sets inspection points on its site view without walking Spot there [BD blog] and compares acoustic captures with each point's normal state [Robot Report 5.1]. How data leaves Orbit was not found.

**Long runs.** AutoInspect (Spot with a lidar payload) ran 49 days (673 of 730 actions done) and 35 days, with 78 h and 140 h between interventions. One bug surfaced only on long runs, from CPU thermal throttling. Change detection diffs octree maps between missions [AutoInspect].

**Korial and other pairings.** Energy Robotics is now Korial: a vendor-neutral AI layer with virtual mission tests and an IT/OT connector [Korial]; an ANYbotics post title names a 2024 partnership [ANYbotics news]. At Shell Rheinland one interface runs a tracked ExR-2 in Ex zones, Spot on stairs and in tight spots, and a docked drone over tank roofs. It reads gauges and valves, checks thermal, and flags ppm gas and drift from baseline [Korial Shell]. Energy Robotics says the drone does 8 h of manual roof checks in 30 min [DroneDJ]. DroneDeploy files Spot, Unitree Go2-W and DJI Dock captures in one project [DroneDeploy]. Percepto's AIM added Spot beside its drones in 2020, an integration with shared analytics; no joint site or handoff is described [Construction Equipment]. Ground robots that carry drones appear only in research (see the Voliro/ANYbotics report).

**Gecko.** Magnetic-wheel crawlers with 24 UT probes, 250 readings per ft, up to 18 m/min [Gecko robots]; a Navy hull got 3.3 M points vs 6,000 before [PopSci]. A 2021 boiler job used three Toka 4s, six inspectors and scaffolds for 25.3 M readings, delivered as lowest readings by tube and elevation, C-scans and a portal; interpretation stayed with the customer [KY PSC]. Cantilever turns robot, drone and sensor data into corrosion rate and remaining life [Robot Report Cantilever]. Gecko also sells drone scans of roofs, aircraft and confined spaces [Gecko drones]. Navy deal $71 M over five years; valuation $1.25 B [Sacra] or $1.3 B [Robotics.press].

**Drones vs ground robots.** Drones win on roofs and high structures [Korial Shell] and need no scaffold (E). They lose on endurance (Spot 90 min [Spot SDK] vs Scout ~9 min, E), payload, Ex zones, doors, dense contact UT and long-run proof. Prop noise and downwash would swamp an acoustic imager and stir gas plumes (E).

## Steal for Nox
- **Box catcher and bays:** contacts untouchable until a rod or cartridge seats, like Spot's covered connector [Spot docking docs] (E).
- **Box catcher:** both drones share one head; each checks a head-free flag before approach, else holds [US20220194245].
- **Box script (X8):** catch and swap never arm a drone; the operator does [Spot docking docs] (E).
- **Comms:** offload footage over Wi-Fi while caught [Dock sheet] (E).
- **Data product:** points set on the Overseer's map; each visit stores photo, clip, pose and time against that point's baseline [Robot Report 5.1].
- **Overseer:** diff each lidar map with the last; list what moved [AutoInspect].
- **Overseer/Scout split:** one mission list, tasks assigned by reach, one data pipeline [Korial Shell].
- **Firmware:** dry-run each route in PX4 SITL on the Overseer's map [Korial] (E).
- **Data product (business):** sell coverage and a per-asset finding list; export to Korial or DroneDeploy instead of building fleet software [Sacra, Korial] (E).

## For Spark
- `box.md` "Scout 4: three full, one empty", X10 "charger choice open"; `nox-dimensions.md` §1 "~9 min to 20 % reserve": nonstop Scout flight needs each pack recharged in ~27 min (3 x 9), at least 1.8C average (E). Heat tripled Spot Dock's time to 80% [Dock sheet]: cool the bays (E). Docked Spot works 43 to 60% of the time (E, from [Spot SDK, Spot page]).
- `scout.md` problem (b), "it throttles, does not crash": throttling caused AutoInspect's late bug [AutoInspect]. The Radxa has moved to the cage (`nox-dimensions.md` §1); soak it at demo load for a full day anyway (E).
- `box.md` "Done: not set": log per-step success and hours between interventions [AutoInspect] (E).
- `scout.md` purpose, "under tables, behind racks": Spot's zoom tilts to 90° up [CAM 2 sheet], so gauges with a floor view are its turf. Demo a target with none (rack top) or behind a gap under 500 mm [Spot SDK]; leave acoustic and gas to ground robots (E).
- `box.md` "swap its cartridge by script, repeatably": a DroneXL headline (8 Oct) says DJI teased a battery-swap dock for 14 Oct; not opened [DroneXL]. Check before pitching swap as unique (E).

## Sources
- [Spot SDK: About Spot (Boston Dynamics developer docs)](https://dev.bostondynamics.com/docs/concepts/about_spot)
- [Spot page: Spot product page (Boston Dynamics)](https://bostondynamics.com/products/spot/)
- [Tecnoblog: Boston Dynamics sells Spot to companies for US$74.5k](https://tecnoblog.net/346372/boston-dynamics-vende-cao-robo-spot-para-empresas-por-us-745-mil/)
- [Dock sheet: Spot Dock datasheet (Boston Dynamics)](https://bostondynamics.com/wp-content/uploads/2020/10/spot-dock.pdf)
- [US20220194245: Robust docking of robots with imperfect sensing (Boston Dynamics)](https://patents.justia.com/patent/20220194245)
- [Spot docking docs: Docking (Spot SDK)](https://dev.bostondynamics.com/docs/concepts/autonomy/docking)
- [CAM+IR sheet: Spot CAM+IR datasheet](https://www.bostondynamics.com/wp-content/uploads/2023/05/spot-cam-plus-ir.pdf)
- [CAM 2 sheet: Spot CAM 2 datasheet](https://bostondynamics.com/wp-content/uploads/2026/01/spot-cam-2.pdf)
- [SV600 sheet: Fluke SV600 datasheet (Boston Dynamics)](https://bostondynamics.com/wp-content/uploads/2024/09/fluke-sv600.pdf)
- [Robotics Tomorrow: Spot outfitted with Blackline Safety gas detector](https://www.roboticstomorrow.com/news/2026/06/11/boston-dynamics-spot-robot-dog-outfitted-with-blackline-safety-gas-detector/26700/)
- [BD blog: A new perspective for facilities inspection](https://bostondynamics.com/blog/a-new-perspective-for-facilities-inspection/)
- [Robot Report 5.1: Boston Dynamics releases Spot and Orbit 5.1 with new Spot Cam](https://www.therobotreport.com/boston-dynamics-releases-spot-and-orbit-5-1-with-new-spot-cam/)
- [AutoInspect: Towards long-term autonomous industrial inspection (arXiv)](https://arxiv.org/pdf/2404.12785)
- [Korial: Korial, the new name for Energy Robotics](https://www.korial.com/)
- [ANYbotics news: ANYbotics and Energy Robotics partnership, ACHEMA 2024](https://www.anybotics.com/news/anybotics-energy-robotics-partnership-achema-2024/)
- [Korial Shell: Shell Rheinland customer story](https://www.korial.com/customer-stories/shell-rheinland)
- [DroneDJ: Drone tank inspection by Energy Robotics](https://dronedj.com/2024/06/26/drone-tank-inspection-energy-robotics/)
- [DroneDeploy: Meet Spot](https://dronedeploy.com/solutions/meet-spot)
- [Construction Equipment: Drone, Spot the robot team up for inspections](https://www.constructionequipment.com/drone-spot-robot-team-inspections)
- [Gecko robots: Gecko Robotics robots](https://www.geckorobotics.com/company/robots)
- [PopSci: The Navy's version of a Roomba inspects ships](https://www.popsci.com/technology/gecko-robotics-machine-inspects-navy-ships/)
- [KY PSC: Gecko Robotics 2021 U2 boiler report (Kentucky PSC case 2024-00370)](https://psc.ky.gov/pscecf/2024-00370/jessica.fitch-snedegar@ekpc.coop/01032025040034/Staff1_-_29e._U2_BOILER_GECKO_ROBOTICS_2021_REPORT.pdf)
- [Robot Report Cantilever: Gecko Robotics launches Cantilever platform](https://www.therobotreport.com/gecko-robotics-launches-cantilever-platform/)
- [Gecko drones: Gecko drone inspections](https://geckorobotics.com/aerial-surface-scans)
- [Sacra: Gecko Robotics company profile](https://sacra.com/c/gecko-robotics/)
- [Robotics.press: Gecko Robotics gets $125M, hits $1.3B valuation](https://robotics.press/news/gecko-robotics-125m-series-d-1-3b-valuation/)
- [DroneXL: DJI Enterprise teaser, battery swap dock, October 14 (headline only)](https://dronexl.co/2026/10/08/dji-enterprise-teaser-battery-swap-dock-october-14/)
