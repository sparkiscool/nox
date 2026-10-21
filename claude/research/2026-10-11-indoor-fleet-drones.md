# Indoor fleet drones that self-dock: Verity, Corvus, Gather AI, Ware (11 Oct 2026)

**Bottom line**
1. Verity and Corvus sell complete systems: their own drones, charged in place, with no published mass or dock specs. Gather AI and Ware put software on bought drones. Verity flies 10 min per 30 min charge [DroneXL Gent] from four-drone docks [AllAboutLean], so about one drone per dock is airborne (E).
2. Safety around people is separation: night or third-shift flights, blocked aisles, avoidance that climbs away. No safety certification or price was found.
3. Steals: beehive pack math, the box as a data dock, re-scanning only what changed. Flown nonstop, Nox's packs need ~2C charging and ~500 W (E), over the box's 350 W supply (`box.md`).

## Findings

**Verity (Swiss).** No GPS, no pilot; perception, planning and avoidance run onboard. About 200 warehouses; 2 to 3 battery replacements a year [IFR]. At IKEA Gent 6 drones fly nightly, 10 min per charge, 30 min to recharge [DroneXL Gent]. A store of ~7,000 locations runs 8 drones in two "beehives" of four: a full count on closed Sundays, then weeknight counts only where stock moved [AllAboutLean]. Series B totals $43 M (2023) [DroneXL 2023]. Mass and sensors: not found.

**Dock patents.** Verity's tray [US11498700B2, active to ~2040]: feet drop into sloped recesses that can force one yaw. Stations under the tray push posts up through holes and lift the drone. Contact force comes from weight, springs, or magnets weaker than lift-off thrust. Terminals sit four at 90° (4 yaws), three at 45° (8 yaws), or in concentric rings, likely any yaw (E). Alignment keys are tall enough that a misaligned tray never touches the contacts. A 2019 property-monitoring dock makes its funnel wider than a ±10 cm landing error [US11572197].

**Corvus Robotics (Corvus One).** In-house drone [MIT News], 14 cameras for omnidirectional collision prevention [Corvus One]. An earlier version ran visual-inertial SLAM on 10 cameras, ROS over PX4, all onboard [IEEE Spectrum]. No beacons, stickers or Wi-Fi [Robotics 24/7]. Rack-end docks charge and move data; staff rough-map with tape measures, drones refine it, and customers set no-fly zones and schedules [MIT News]. Installs need power and Ethernet; data stays on-prem [Corvus One]. Aisles from 50 in, walking speed, climbs to avoid people and forklifts [Robotics 24/7]. Racks up to ~20 m, 200 to 400 pallet positions per drone-hour including recharge, emergency landing on a clear flat spot, often third shift; people keep out of an aisle being scanned [IEEE Spectrum]. The only compliance mark shown is AICPA SOC, for data security [Corvus site]. $18 M Series A, 2024 [Pulse 2.0]; $20 M, Aug 2026, 300+ devices [Runtime Wire].

**Gather AI.** Software on DJI drones and, from 2025, ModalAI's US-made Starling 2 Logis: aisles from 4 ft 5 in (5 ft 4 in on DJI), 1,500 pallets/h (was 900); GEODIS is a customer [BusinessWire 2025]. No dock found. $40 M Series B, 2026 [Technical.ly].

**Ware.** Software on Skydio 2; drones land in charging nests, and clients block aisles first [SkyWatch].

**Other indoor docks.** Indoor Robotics' Tando docks on a ceiling station and works as a security camera while docked [DroneDJ]. Corax (Corvus Drones, a different firm, E): 330 g, 120 mm props, VIO and ToF on a VOXL 2, PX4, 25+ min, a self-centering 400 x 400 mm dock, and a pack that reports cycles and age [ModalAI]. Sees.ai flies under remote pilots [eeNews]. Avestec and Nearthlab: not reached.

## Steal for Nox
- Swapper and charger: packs per drone = 1 + charge time / (flight + swap time), Verity's beehive math (E).
- Box catcher: set the keep-alive pin tips in the V-jaw apex so only a rod bearing on both flanks reaches them (Verity's keys align before contact).
- Comms: offload full video and logs while the drone sits in the box; stream only a preview in flight (Corvus docks carry data).
- Cartridge: count charge cycles per pack and retire packs by count (Corax pack; Verity's yearly replacements).
- Data product: after one full scan, re-scan only where the Overseer map changed (Verity).
- Firmware (Scout): on an Overseer person alert, stop and climb clear (Corvus).
- Firmware (both drones): on failsafe, land on a pre-mapped clear spot, not in place (Corvus).

## For Spark
- `box.md` "Scout 4: three full, one empty" means 4 packs in rotation. At "~9 min to 20 % reserve" (`nox-dimensions.md` §1) plus ~1 min swaps (E), each must recharge in ~30 min: ~2C, ~3 A (E). "Overseer 3: two full, one empty" at 11 to 12 min (`claude/overseer-compatibility-audit.md`) needs ~25 min: ~2.4C, ~14 A (E). At 1C, nonstop flying needs ~7 Scout and ~6 Overseer packs (E). Check pack charge ratings before the X10 charger choice.
- `box.md` "Mean Well LRS-350-24": nonstop charging draws ~110 W for the Scout ("~108 W hover", §1) and ~380 to 420 W for the Overseer (audit "~25 A" at 1,860 g; §4 has 2.0 kg) (E). Give the chargers their own supply.
- `nox-dimensions.md` §2 "Bay balance pins" and `box.md` X5 "6 spring pins" assume the pad PCB that cartridge v18 dropped. Decide how bays balance-charge and tell packs apart; tracking packs by slot in the swap script is the cheap fix (E).
- `v2/README.md` all-up "~520" to "~400": hover power scales as mass^1.5, so ~1.5x hover time per pack (E). Corax fits: Scout-like energy (~23 Wh, E) at ~40% of Scout's hover power gives 25+ min (E).
- `scout.md` "RC loss: Land" lands in place, maybe near people: demo in a taped, cleared zone (E).
- `README.md` "Buy and modify": Corvus built its own drone for low-level control [MIT News]; Gather AI added a US-made drone [BusinessWire 2025]. Nox's Chinese FC, frames and lidar may matter to US buyers later (E).
- IP: US11498700B2 covers drone trays; the box has none (`box.md` D1), so likely clear (E, claims not read).

## Sources
- [IFR: IERA Award 2026 goes to flying warehouse robots by Verity](https://ifr.org/ifr-press-releases/news/iera-award-2026-goes-to-flying-warehouse-robots-by-verity)
- [DroneXL Gent: IKEA Gent uses Verity drones for inventory counts](https://dronexl.co/zh/2022/12/07/ikea-gent-verity-drones-inventory-counts/)
- [AllAboutLean: How IKEA uses drones for inventory management](https://www.allaboutlean.com/drones-at-ikea/)
- [DroneXL 2023: Verity inventory drones, Series B extension](https://dronexl.co/zh/2023/07/17/verity-inventory-drones/)
- [US11498700B2: Charging system and tray for flying machines (Verity)](https://patents.google.com/patent/US11498700)
- [US11572197: Stations for unmanned aerial vehicles (USPTO)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11572197)
- [MIT News: Corvus autonomous drones precisely track warehouse inventories](https://news.mit.edu/2024/corvus-autonomous-drones-precisely-track-warehouse-inventories-1220)
- [Corvus One: product page](https://www.corvus-robotics.com/corvus-one)
- [Corvus site: Corvus Robotics home page](https://www.corvus-robotics.com/)
- [IEEE Spectrum: Corvus Robotics warehouse drones](https://spectrum.ieee.org/drone-warehouse-corvus-robotics)
- [Robotics 24/7: Corvus brings autonomous inventory to lights-out warehouses](https://robotics247.com/article/corvus_robotics_brings_autonomous_inventory_management_system_to_lights_out_warehouse)
- [Pulse 2.0: Corvus Robotics $18 M Series A](https://pulse2.com/corvus-robotics-18-million-series-a-secured-to-advance-autonomous-inventory-management-systems/)
- [Runtime Wire: Corvus raises $20 M as Kabir becomes CEO](https://runtimewire.com/article/mohammed-kabir-corvus-robotics-ceo-20m-funding)
- [BusinessWire 2025: Gather AI adds Starling 2 Logis drones](https://www.businesswire.com/news/home/20250210907896/en)
- [Technical.ly: Gather AI Series B](https://technical.ly/entrepreneurship/gather-ai-series-b-raise/)
- [SkyWatch: How Ware optimizes warehouse inventory with indoor drones](https://www.skywatch.ai/blog/ware-indoor-drones)
- [DroneDJ: Tando indoor drone (Indoor Robotics)](https://dronedj.com/2022/06/24/tando-indoor-drone/)
- [ModalAI: Corax by Corvus Drones](https://www.modalai.com/en-il/pages/corax)
- [eeNews Europe: Drone system for BVLOS industrial inspections (Sees.ai)](https://www.eenewseurope.com/en/drone-system-for-bvlos-industrial-inspections/)
