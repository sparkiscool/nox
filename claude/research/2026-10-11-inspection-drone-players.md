# Industrial and confined-space inspection players (11 Oct 2026)

**Bottom line**
1. Acquirers pay for recurring inspection revenue. Ondas agreed to buy Cyberhawk for ~$125M on a $45M+ revenue forecast and a $95M backlog [IUS Ondas; Ondas 8-K]; Beam Global is buying ScoutDI for ~$24M [R&A News].
2. Confined-space hardware has split into tethered drones (ScoutDI, Avestec, a Terra option) and contact-UT drones (Avestec, Skygauge). Jobs that name a crew had 2 to 3 people on site [Evident; IUS boiler]. No source shows any of the seven docking indoors, swapping packs or pairing a mapper with a close-in drone (E).
3. Nox's opening is unattended indoor rounds from a box: the same points each visit, with change reported. Copy Sees.ai's remote control room, Nearthlab's graded findings and ScoutDI's map-tagged portal. Leave UT contact and tethers alone.

## Findings

**Sees.ai (UK).** Started as command-and-control software for bought drones, sold mainly through service firms [Revolution]. Pilots in a distant control room supervise automated drones built for GPS loss, magnetic noise and weak links [eeNews]. Since 2023 it has held routine CAA BVLOS on two National Grid line runs; output is georeferenced images and 3D models that Keen AI analyses [Unmanned Airspace]. Raised £3.65M in Oct 2025 [sUAS News]. No indoor flight found.

**Nearthlab (Korea).** Wind blades on its own airframe: under 15 min a turbine [IUS Elevation], up to 15 a day [UST]. Onboard vision flies it relative to the blade, with a lidar backup and no waypoints; defect AI runs later in the cloud. Sold as a service or as an app for customers' own drones [IUS Elevation]. Zoomable grades defects 1 to 5, estimates energy loss and sets repair windows [IUS Elevation]. Raised KRW 65B [Daum]. After 100,000+ turbines, defense was 85.6% of H1 2026 revenue [Edaily].

**Avestec (Canada).** SKYRON: ~7 kg, 80 cm wide, tethered, run by a pilot and a Level II inspector [Evident]. Tether 50 to 100 (m, E), 3,500 peak (W, E) [Wevolver spec]. It cleans the spot, then an arm on a ±90° electromagnetic hinge attaches the drone to the surface. A stripped-down Evident 38DL PLUS reads 25 points in a 2 x 2 in patch, and the ground station writes the report [Evident]. Billed as the first non-entry confined-space UT (2023, with Acuren): a 50 m refinery stack, 76 readings in 64 min, every link through a wired relay [Wevolver case]. Funding not found.

**Terra Drone (Japan).** Terra Inspectioneering flies its own collision-tolerant UT drone in Vopak tanks at 11 sites, to EEMUA-159 and API-653, and delivers an annotated 2D and 3D cloud report, up to 60% faster [Terra Vopak]. Terra Xross 1 (Jan 2025): in-house, lidar plus visual odometry, battery or tether, 4K camera tilting 180°, LEDs, under $25,000. That is a third of rivals' price [Terra launch], or a third less [DroneDJ]. Dealers now list it as the Nolvis X1 (E, from their URLs): 1,800 g, 36 x 31 x 21 cm, 10 min, 10,480 lm, caged [DSLRPros; Maverick]; $23,980 [DSLRPros] or $30,980 [Maverick]. The group listed in Tokyo in Nov 2024 on revenue of ¥1.81B (year to Jan 2024), mostly crop spraying and flight-management software [THE BRIDGE].

**Cyberhawk (UK).** Crewed service on unnamed drones (bought, E) plus iHawk, a cloud digital twin with AI analytics; 500,000+ assets, 300+ customers including PG&E, Shell and SSE [IUS Ondas]. Its first internal boiler job took 1 day, against 3 to 4 by rope access, with a crew of 3 [IUS boiler]. EBITDA margin is high single digits, with 25% targeted by 2030 [Ondas 8-K]. IUS says ~95% of revenue recurs [IUS Ondas]; the 8-K says only "high", and its 95% is the cash share of the price, so treat the IUS figure as unconfirmed.

**Skygauge (Canada).** Thrust-vectoring drone that can push and twist against a surface [Robot Report seed]. The arm tilts 90° up or down, couplant goes on just before touch, a force sensor squares the probe and forward lidar holds a fixed approach speed. It splits to pass 55 cm manways. Payloads: UT, dry film, wire brush, EMAT; lease or buy [Nexxis]. Claims $45,000 of crane rental saved on one stack; partners include Cyberhawk and Mistras [Skygauge]. Seed of CAD 3.3M in 2020 [Robot Report seed].

**ScoutDI (Norway, added).** The Scout 137 Gen 3 has no battery [Business Norway] and runs on a 60 m tether [Measur]. A 128-beam lidar flies it in the dark; 4K camera with 3.5x zoom, gas sensor standard, UT optional. Findings are pinned to a 3D model, and an API feeds them into integrity systems [Business Norway]. Customers include ExxonMobil, Chevron and Ørsted. A DoD conditional approval exempts it from the FCC Covered List [R&A News].

**Gaps none fill (E).** Unattended indoor work: every indoor job above has a crew on site. Endurance past a 50 to 100 m tether without swapping packs by hand. A mapper and a close-in drone on one job. Weekly repeat rounds, where rivals trend yearly [Daum] or every six months [Robot Report 2020].

## Steal for Nox
- Data product: grade each finding 1 to 5, trend it per visit and set a repair-by window, as Zoomable does [IUS Elevation].
- Data product: the box writes the report right after the catch, as SKYRON's ground station does [Evident].
- Data product: push findings by API into the customer's integrity system, as ScoutDI does [Business Norway].
- Business: sell multi-year site subscriptions, which the 8-K credits Cyberhawk with [Ondas 8-K], through NDT firms (Acuren, Mistras) as Avestec and Skygauge do [Wevolver case; Skygauge].
- Comms: one remote operator supervises both drones through the box, like Sees.ai's control room [eeNews]. Inside a vessel, relay every link through a node at the hatch, as Avestec did [Wevolver case].
- Overseer: lidar plus visual odometry flies Terra's sub-$25,000 drone [Terra launch]; L2 plus flow is the same recipe (E).
- Scout firmware: keep a second position source live, as Nearthlab runs lidar beside vision [IUS Elevation]. For the Scout, that means Overseer tag fixes beside flow (E).
- Cartridge (Overseer): stay under 100 Wh, as the Nolvis (99.4 Wh [Maverick]) and Elios 3 (99.2 Wh, Elios report) do. The 4S 6000 is ~89 Wh (E).

## For Spark
- `box.md` pitch: swaps give endurance without a tether; tethered rivals stop at 50 to 100 m [Wevolver spec; Measur] (E). For "Done: not set": a boiler took 1 day against 3 to 4 by rope [IUS boiler], and scaffold costs $10,000 to $40,000+ per asset (dealer figure, unaudited) [Measur]. Score the demo in entries avoided and catch-swap cycles per hour (E).
- `nox-dimensions.md` §3 / §4: the Scout cage is 277.4 across corners (C), ~285 with nodes (E), so it should pass an 18 in (457 mm) manway whole (E). Skygauge has to split for 55 cm [Nexxis], and the Overseer's 608 prop span (C) cannot pass (E). Pitch: the Overseer maps the hall, the Scout enters the vessel (E).
- `scout-camera-tilt.md`: Terra's camera tilts through 180° [Terra launch], so buyers will expect a down view. Option 1 there (a fixed down camera, no pocket change) is the cheapest match (E).
- `overseer-scout-hardware-reference.md` §8 dropped the Mid-360 (~¥3,700) for the L2 (¥1,779). The Nolvis lidar specs (200,000 pts/s, 40 m at 10%) [DSLRPros] match a Mid-360 (E), which makes it the proven fallback if L2 SLAM falls short, at ~+35 g (E).
- Rivals and buyers: Ondas owns Airobotics, the swap-dock maker in the research queue, and is buying Cyberhawk [IUS Ondas]. Beam Global sells drone recharging and is buying ScoutDI [R&A News]. Dock makers are buying inspection data (E). With caged lidar drones listed from $23,980 [DSLRPros], pitch the box and the data, not the airframe (E).
- Pitch risk: even Norway's ScoutDI needed a DoD exemption from the FCC Covered List [R&A News]. Nox's FC, lidar and radios are Chinese-made (E). Fine for a demo; check the rules before pricing a US product (E).

## Sources
- [Revolution: Revolution.Aero Uplift, command and control, Sees.ai](https://www.revolution.aero/?p=287)
- [eeNews: Drone system for BVLOS industrial inspections (eeNews Europe)](https://www.eenewseurope.com/en/drone-system-for-bvlos-industrial-inspections/)
- [Unmanned Airspace: Sees.ai secures routine BVLOS permission over powerlines](https://unmannedairspace.info/?p=22436)
- [sUAS News: Sees.ai secures £3.65m](https://www.suasnews.com/2025/10/sees-ai-secures-3-65m-to-accelerate-world-first-centralised-autonomous-drone-operations-for-critical-national-infrastructure/)
- [UST: NearthWIND Pro 2 (Unmanned Systems Technology)](https://www.unmannedsystemstechnology.com/company/nearthlab/pro2/)
- [IUS Elevation: AI-driven drones reshaping turbine inspections (Inside Unmanned Systems)](https://insideunmannedsystems.com/elevation-ai-driven-drones-reshaping-turbine-inspections/)
- [Daum: Nearthlab profile, Mar 2026 (Korean)](https://v.daum.net/v/20260317101920110)
- [Edaily: Nearthlab H1 2026 defense revenue (AI summary via StockHub)](https://stockhub.kr/en/news/news_9a7d45e62d55)
- [Evident: Tethered UT drones for work at heights](https://ims.evidentscientific.com/en/insights/tethered-ut-drones-a-new-era-for-ultrasonic-inspection-work-at-heights)
- [Wevolver spec: SKYRON specifications](https://www.wevolver.com/specs/skyron)
- [Wevolver case: First non-entry UT inspection in a confined space using an aerial robot](https://wevolver.com/article/first-non-entry-ultrasonic-testing-ut-inspection-in-a-confined-space-using-an-aerial-robot)
- [Terra Vopak: Terra Inspectioneering's UT drones to aid Vopak](https://terra-drone.net/global/2020/06/30/terra-inspectioneering-ut-drones-inspect-tank-vopak/)
- [Terra launch: Terra Drone launches Terra Xross 1 in Japan and the US](https://terra-drone.net/global/2025/01/31/terra-drone-launches-japan-made-indoor-inspection-drone-terra-xross-1-simultaneously-in-japan-and-the-us)
- [DroneDJ: Terra Xross indoor inspection drone](https://dronedj.com/2025/02/04/terra-xross-indoor-inspection-drone/)
- [DSLRPros: Terra Xross 1 listing (now Nolvis X1)](https://www.dslrpros.com/products/terra-xross-1-indoor-inspection-drone)
- [Maverick: Nolvis X1 listing](https://www.maverickdrone.com/es/products/terradrone-terra-xross-confined-space-uav)
- [THE BRIDGE: Terra Drone files for IPO](https://thebridge.jp/en/2024/10/terra-drone-files-for-ipo)
- [IUS Ondas: Ondas to acquire Cyberhawk in $125M deal (Inside Unmanned Systems)](https://insideunmannedsystems.com/ondas-to-acquire-cyberhawk-in-125-million-critical-infrastructure-push/)
- [Ondas 8-K: exhibit 99.1, Cyberhawk acquisition](https://www.sec.gov/Archives/edgar/data/0001646188/000121390026069889/ea029504301ex99-1.htm)
- [IUS boiler: Cyberhawk completes first internal steam boiler drone inspection](https://insideunmannedsystems.com/cyberhawk-innovations-completes-first-internal-steam-boiler-drone-inspection/)
- [Robot Report 2020: Skygauge offers inspection drones](https://www.therobotreport.com/skygauge-offers-inspection-drones-make-industrial-sites-safe-reopen/)
- [Robot Report seed: Skygauge raises $2.5M seed](https://www.therobotreport.com/skygauge-robotics-raises-2-5m-seed-infrastructure-inspection-drone/)
- [Nexxis: Skygauge drone](https://nexxis.com/product/skygauge-drone/)
- [Skygauge: skygauge.com home page](https://www.skygauge.com/)
- [Business Norway: Scout 137 Gen 3](https://businessnorway.com/solutions/the-scout-137-gen-3-drone-lets-asset-inspectors-see-in-the-dark)
- [Measur: Reducing scaffolding costs with drone inspections](https://measur.ca/pages/learn/reducing-scaffolding-costs-drone-inspections)
- [R&A News: Beam Global to acquire ScoutDI for $24M](https://roboticsandautomationnews.com/2026/10/07/beam-global-to-acquire-scoutdi-for-24-million-and-manufacture-inspection-drones-in-us/105671/)
- Elios report: `claude/research/2026-10-11-flyability-elios-tech.md` (internal)
