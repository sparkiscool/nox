# Keep-alive power through the catch and swap (11 Oct 2026)

**Bottom line**
1. Electrolytic caps cannot carry a drone through a swap: 5 s on the Scout needs 0.41 F, about 1 kg (E). Box power through the iris V-jaws costs the Scout ~5 g and has no time limit. A supercap string is the fallback (~20 g, 5 to 7 s, E).
2. Avionics draw 8.5 to 12.5 W (Scout) and ~24 W (Overseer) (E). A Schottky on the box path loses up to 0.34 / 0.7 W, an ideal diode 20 / 40 mW (E). No bridge is needed, because the catch fixes yaw.
3. Two traps. PX4 carries the old pack's mAh count onto the new pack. Box current reaching zero proves the XT60 pins touched, not that the cartridge seated.

## Findings
**Load.** Radxa Zero 3W: under 3 W idle, ~4 W at full CPU [Mehatronika]. Pi 5: 3.0 W idle, 8.8 W loaded, though a table on the same site says 12 to 25 W [raspberry.tips]. With camera, FC, receiver and flow (3.5 W) behind an 88 % BEC, the Scout draws ~8.5 W (E); `catcher-speed-research.md` used 12.5 W. The Overseer adds the 10 W L2 and a ~7 W Pi: ~24 W (E).

**Hold-up.** A cap falling from 12.6 V to a 7 V regulator dropout gives 55 J per farad (E).
- Contact chatter (10 ms): the Scout's 220 µF plus 1000 µF covers 5 to 8 ms; the Overseer needs ~4.4 mF (E).
- No contact: the pack is out 1 to 1.5 s in the Nox motion budgets, 2 to 5 s on a first build. 5 s at 8.5 W needs 0.41 F (E).
- Supercaps: 8 Eaton HV1020 in series (5 F, 40 mΩ, 2.3 g each [Eaton]) make 0.63 F at 21.6 V: 5 to 7.6 s, ~20 g with balancing (E). Fresh packs then need a charge resistor with a bypass diode, or they arc. Overseer: 8 x HV1030 (10 F, 3.2 g [Eaton]), 5.4 s at 24 W (E).
- LTC4040: one Li-ion cell backs a 3.5 to 5.5 V rail at up to 2.5 A [ADI LTC4040]. It would keep the Radxa up, but not the Scout's FC, which runs from VBAT.

**OR-ing losses** (E, from the parts below)

| Box path | Drop | Scout 1 A | Overseer 1.9 A |
|---|---|---|---|
| Schottky bridge | ~0.7 V | 0.7 W | 1.4 W |
| One Schottky | ~0.35 V | 0.34 W | 0.7 W |
| LT4320 ideal bridge | ~0.04 V | 60 mW | 100 mW |
| LM74700 ideal diode | 0.02 V | 20 mW | 40 mW |

PMEG4050EP: 0.34 V at 1 A [Nexperia]. LM74700-Q1: one N-FET held at 20 mV, reverse current blocked within 0.75 µs, 3.2 to 65 V [TI]. LT4320: four N-FETs as a bridge, 9 to 72 V; at 2 A it warmed 0.6 °C against 15 °C for a diode bridge [ADI LT4320]. All keep the contacts dead in flight.

**Polarity-free contacts.** ASU publishes no diode count or wear data [ASU]. US20190023133 lays + and - tiles in stripes or a checkerboard with gaps wider than a contact; four or five drone contacts on diode pairs find both poles in any pose [Justia].

**Inrush.** Charging a cap always dissipates ½CΔV². A bus held at 12.6 V meets a full pack with 4.2 V, not 16.8 V: 0.011 J instead of 0.17 J on the Scout (E).

**Plating.** Gold needs 30 to 40 g normal force, tin 100 g; 10 µin gold usually fails gas tests, 30 µin passes [Samtec]. Gold lasts 100 to 1000+ cycles, silver ~50, tin under 50 [Telewire]. Gold spring pins: 2 A, 20 mΩ max, 1,000,000 cycles [Mill-Max]. That rates the pin, not its landing surface (E). Brass has no data; load it like tin (E).

**Fraunhofer RAPID.** The station senses the landing, pulls the platform in, swaps both packs and supplies enough voltage to keep the drone online. How is unpublished [RAPID]. The page claims ~60 s; the papers report under 3 min landing to takeoff [OCEANS 2023; IEEE Access 2024]. An IPN design instead targets a swap under 10 s with no keep-alive; past that, it says, ESC calibration is lost [MDPI].

**PX4.** A pack counts as removed only below 2.1 V. Discharged mAh resets only at boot, and with a capacity set it caps the charge estimate [PX4 main; CHECK on 1.18].

## Steal for Nox (sizes E)
- Box catcher: gold spring pins in the long V-jaw apex, 2 per pole (3 on the Overseer), ~1 N each; + on the +y jaw, - on the -y jaw.
- Cage / Overseer catch frame: a 30 long band at mid-span of each ±y ring rod: brass shim on Kapton (carbon conducts), or a gold-plated flex-PCB strip, which brings its own insulation.
- Scout: Schottky from the + band into the bus, plus 1000 µF 35 V, ~5 g. It also blocks a reversed catch.
- Overseer: LM74700 plus 4 x 1000 µF 35 V, ~14 g.
- Swapper: no anti-spark plug or pre-charge step; the held bus does that.
- Scout v2: 30 µin gold or more on the pack pins and pads.
- Firmware: BAT1_CAPACITY -1 for the demo, or reset the estimate on a voltage step while disarmed.

## For Spark
- `catcher-speed-research.md` §4 puts the contact "on the catcher forks", retired 9 Oct. The iris v2 V-jaws hold the rods through the swap (`nox-dimensions.md` §4: "20 N mid-span"), so put the pins there (E). The bands touch the cage: your call.
- `twin-arm-swap-concept.md`: "box current -> 0 = seated". The XT60 pins touch within the last 9 (§2), so zero comes before the seat. Pair it with the hall trip in `scout-swap-arm.md` (E).
- `box.md` still says "no powered swap in v1", and its Power line has only the servo buck. Add a 13.0 V keep-alive buck with an INA226 once the contact is OK'd (E).
- `overseer_params_2026-09-26.params`: BAT1_CAPACITY 6000. After a powered swap the old count caps the new pack's estimate, which may block arming. Bench test, then set -1 or patch (E). Check the Scout too.
- `nox-dimensions.md` §4 lists the FCHUB-12S, but BAT1_A_PER_V 78.43 matches the 4-in-1 shunt in `overseer-wiring-plan.md`. That plan's buck-boost holds the L2 at 12 V on a 13 V bus; the FCHUB's buck-only 12 V may not. Confirm which is fitted (CHECK).

## Recommended architecture (E)
1. Box: LRS-350-24 into a 13.0 V CC/CV buck, limit 2 A (Scout) / 3 A (Overseer), INA226. 13.0 V stays under any landed pack.
2. Contact: gold spring pins in the V-jaw apex onto the ±y ring-rod bands. Fixed polarity; add an LT4320 bridge only if 180° catches are ever allowed.
3. Drone: Schottky (Scout) or LM74700 (Overseer) on the box path only; ~1.2 mF (Scout), ~4.5 mF (Overseer).
4. Sequence: jaws shut, box on (0 A while the old pack is higher); pull the pack; push to the hall trip; under 50 mA confirms the mate; box off; arm; jaws open. The pins never switch current.
5. Safety: bands dead in flight; with the pack out, the pocket XT60 pins carry 12.6 V behind the current limit. Arm only after box current reads zero.

## Sources
- [RAPID system components (Fraunhofer CML)](https://rapid2020.eu/system-components/)
- [Automated battery hot swap with SeaML:SeaLion, OCEANS 2023 (UWS portal)](https://research-portal.uws.ac.uk/en/publications/automated-battery-hot-swap-system-that-integrates-unmanned-aerial/)
- [Cooperative UAV and USV for maritime coverage, IEEE Access 2024 (UL repository)](https://researchrepository.ul.ie/entities/publication/92ddacd2-edf9-4169-9385-969e7789af93)
- [Low-cost battery exchange for a quadrotor (MDPI Drones 2023)](https://www.mdpi.com/2504-446X/7/4/270)
- [US11465776B2, Arizona State](https://patents.google.com/patent/US11465776B2/en)
- [US20190023133, Systems for Charging Aerial Vehicles (Justia)](https://patents.justia.com/patent/20190023133)
- [LT4320 datasheet (ADI)](https://www.analog.com/media/en/technical-documentation/data-sheets/4320fb.pdf)
- [LM74700-Q1 (TI)](https://www.ti.com/product/LM74700-Q1)
- [PMEG4050EP datasheet (Nexperia)](https://assets.nexperia.com/documents/data-sheet/PMEG4050EP.pdf)
- [LTC4040 datasheet (ADI)](https://www.analog.com/media/en/technical-documentation/data-sheets/4040fb.pdf)
- [Eaton HV1020-2R7505-R supercapacitor](https://www.eaton.com/us/en-us/skuPage.HV1020-2R7505-R.html)
- [Eaton HV1030-2R7106-R supercapacitor](https://www.eaton.com.cn/cn/zh-cn/skuPage.HV1030-2R7106-R.html)
- [Radxa Zero 3W review (Mehatronika)](https://magazinmehatronika.com/radxa-zero-3w-recenzija)
- [Raspberry Pi power consumption, all models (raspberry.tips)](https://raspberry.tips/en/raspberrypi-tutorials/raspberry-pi-power-consumption-update-2026-all-models-compared)
- [Connector plating FAQs (Samtec)](https://blog.samtec.com/post/connector-plating-faqs-normal-force-cycles-temperature-and-other-essentials/)
- [Gold vs tin vs silver plating guide (Telewire)](https://www.telewiretech.com/blogs/technical-resources/connector-plating-comparison-gold-vs-tin-vs-silver-selection-guide)
- [Mill-Max 0900/0906 spring-loaded pins (Mouser)](https://www.mouser.com/new/mill-max/millmax_0900_0906/)
- [PX4 battery.cpp](https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/src/lib/battery/battery.cpp)
- [PX4 battery.h](https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/src/lib/battery/battery.h)
