# Fast charging and the Box power budget (11 Oct 2026)

**Bottom line**
1. Both pack classes take 2C or more: a CNHL 1500 4S lists 5C max, a Tattu 6000 4S 1 to 3C recommended and 5C max. In a 2023 test, faster charging aged all five cell types faster, so charge at 2C to ~4.15 V per cell (E).
2. At 2C, nonstop flying needs 4 Scout and 4 Overseer packs on ~5 charger channels (E). `box.md` plans 4 and 3.
3. Charging averages ~500 W and peaks near 600 W (E), over the LRS-350-24's 350 W. Of the chargers checked, only iCharger publishes a control protocol (MODBUS, 4010DUO); the ISDT CM1620 has CAN and RS-485 but lists no protocol or balance port.

## Findings

**Pack charge ratings.** Scout class: CNHL Black 1500 4S 100C, 5C max, 183 g [Unmanned Tech]. Tattu R-Line 1550 [GetFPV R-Line] and GNB 1550 [Pyrodrone GNB] list no charge rate. Overseer class: Tattu 6000 4S 35C, 1 to 3C recommended, 5C max, 590 g, 147 x 45 x 43 [GetFPV 6000]; HRB 6000 lists none [rcdrone HRB]. Molicel P42A Li-ion: 1C standard, 2C on its chart, charge at 0 to 45 °C [Molicel].

**Life cost of 1C, 2C, 3C.**
- Charge current ages cells more than discharge current, worst at high and very low charge; sub-second pulses gain nothing [Keil & Jossen].
- Faster charging sped capacity loss in all five cell types tested; a Samsung 25R lost 33% in 900 fast cycles. Similar cells differed widely [Parschau].
- 124 LFP cells on fast-charge protocols lasted 150 to 2,300 cycles [Severson].
- LiPo pouches cycled at 1C fell from 88-94% to 73-84% in 250 cycles. Each 0.1 V lower end voltage roughly doubles cycle life: 300 to 500 at 4.20 V, 400 to 700 at 4.13 V with 90% energy [BU-808]. Long-life rate: 0.8C [BU-401a].
- Constant current ends near 85%; more current shortens it but lengthens the voltage hold [BU-409]. So (E): full in ~60 / 33 / 23 min at 1C / 2C / 3C, ~92% in ~51 / 24 / 16.

**Heat.** Charge at 0 to 45 °C, best 10 to 30 °C; above 45 °C cells swell, and many chargers stop at 50 °C [BU-410]. Packs land warm after ~4 to 5C hover (E). At 2C a pack makes under ~2 W (E); the heat is the chargers' ~10% (~20 W per Overseer channel, E) and the PSU's 12% [Mean Well 350]. One 24 V fan over bays and chargers (E).

**Chargers**

| Charger | Ports | DC in | Per port | Balance | Size, mass | Price | Control |
|---|---|---|---|---|---|---|---|
| ISDT CM1620 [ISDT] | 1 (3 parallel) | 11-70 V | 1000 W, 20 A | none listed | 105x165x41, 636 g | $140 [BuddyRC CM] to $180 [Pyrodrone CM] | CAN, RS-485, USB-C host, BT |
| iCharger 4010DUO [PRC 4010] | 2 | 9-50 V | 1400 W, 40 A | 1.2 A | 210x140x80, 1470 g | $350, sold out | MODBUS, USB-HID or UART [MODBUS doc] |
| iCharger 456DUO [PRC 456] | 2 | 10-49 V | 45 A, 2200 W total | 2 A | 171x118x59, 920 g | $320 | none listed |
| iCharger X6 [BuddyRC X6] | 1 | 7-32 V | 800 W, 30 A | 2 A | 83x65x37, 168 g | $110 | logging app |
| ToolkitRC M8D [ToolkitRC M8D] | 2 | 10-49 V | 800 W, 30 A | 2 A | 130x97x42, 450 g | $168 [RDQ] | none listed |
| ToolkitRC Q4AC [ToolkitRC Q4AC] | 4 | 10-18 V | 50 W, 5 A | 0.24 A | 150x112x37, 450 g | | none listed |
| SkyRC Q200neo [SkyRC] | 4 | 10-30 V | 100 W, 10 A; 400 W total | 0.8 A | 123x119x78, 680 g | | app, no SDK |
| ISDT K4 [ISDT K4] | 2 | 10-34 V | 600 W, 20 A | 1.5 A | 176x183x57, 1230 g | | phone app |

2C needs ~50 W per Scout pack and ~200 W per Overseer pack (E): the Q4AC and Q200neo are Scout-only. The MODBUS map reads cell voltages, internal resistance, temperatures and cycle count, and starts a charge by unlock, program, channel, run [MODBUS doc]. A builder ran a 4010DUO from a Linux board this way [RC-Thoughts]. The map names only the 4010DUO. HOTA pages failed twice.

**Pack count.** packs = 1 + charge time / (flight + swap) (Nox fleet report). Flight 9 min Scout, 11.5 Overseer, 1 min swap, 4 min cooling (E):

| Charge | Scout, to ~92% / full | Overseer, to ~92% / full |
|---|---|---|
| 1C | 7 / 8 | 6 / 7 |
| 2C | 4 / 5 | 4 / 4 |
| 3C | 3 / 4 | 3 / 4 |

The formula assumes a free charger per pack. A 3 h rotation sim at 2C to ~92% (E): Scout 4 packs on 3 channels never wait, on 2 channels launches wait 2 to 4 min; Overseer 4 packs on 2 channels never wait, 3 packs wait ~3 min every third flight.

**Power** (E unless tagged). Per flight: Scout ~16 Wh, Overseer ~73 Wh. With 90% chargers, nonstop flying averages ~110 W (Scout) plus ~390 W (Overseer) at the charger inputs. Two Overseer packs at 2C draw ~440 W; with three Scout packs, ~600 W. LRS-350-24: 14.6 A, 88%, hiccup on overload [Mean Well 350]. LRS-600-24: 25 A, 91%, 225 x 124 x 41, off after 3 s of overload [Mean Well 600].

## Steal for Nox
- Box charger: iCharger DUOs on MODBUS let the laptop script start, stop and read every bay; the planned Nano and ADCs drop out (E).
- Box charger: end at ~4.15 V per cell; ~7% less energy, roughly 1.5x the cycles (E, BU-808 trend).
- Swapper: the pack just pulled goes to the coolest bay; charge starts below ~35 °C (E).
- Cartridge: log internal resistance and cycle count after each charge; retire packs on a rise [MODBUS doc].
- Box PSU: the LRS-350-24 keeps motion, logic and keep-alive; chargers get their own supply (E).
- Firmware (box script): hold a launch rather than fly a pack under ~90% (E).

## For Spark
- `box.md` "Overseer 3: two full, one empty": nonstop at 2C needs 4 packs (E); `nox-dimensions.md` §4 has "3 on hand". Buy one, which also needs a 4th Overseer slot (`box.md` D5 "3 slots at 95 pitch"), or accept ~3 min holds.
- `box.md` "Scout 4: three full, one empty" works at 2C with 3 channels if flights are "~9 min" (`nox-dimensions.md` §1); `scout.md` still says "~7 min to 20 % reserve", which needs 5 packs (E).
- `box.md` "Power: Mean Well LRS-350-24": too small for nonstop charging (~500 W average, ~600 W peak, E); its overload hiccup [Mean Well 350] would likely reset the SKRs mid-swap (E). Add an LRS-600-24 for the chargers only (E).
- `box.md` "ISDT CM1620, relays, ADCs, Nano": one 20 A channel, no balance port listed [ISDT]; it suits BMS packs, not bare LiPos (E). Demo set (E): two iCharger DUOs, one per drone, relays to the bays; add an X6 as a fifth channel for nonstop Scout runs. The 4010DUO is sold out, so confirm the 456DUO speaks the same MODBUS before buying.
- `nox-dimensions.md` §2 "Bay balance pins" assumes the pad PCB that cartridge v18 dropped. Every balance charger above needs that path back.
- Overseer pack: read its label's charge rating before 2C; `claude/overseer-scout-hardware-reference.md` gives only "25–35C" discharge (E).

## Sources
- [ISDT: CM1620 product page](https://www.isdt.co/cm1620.html)
- [ISDT K4: product page](https://www.isdt.co/k4.html)
- [BuddyRC CM: ISDT CM1620 modular charger](https://www.buddyrc.com/products/isdt-cm1620-modular-charger)
- [Pyrodrone CM: ISDT CM1620 1100W 20A charging module](https://pyrodrone.com/collections/isdt-products/products/isdt-cm1620-1100w-20a-lipo-battery-charger-charging-module-for-rc-models-2-16s-life-lipo-lihv-battery)
- [MODBUS doc: iCharger MODBUS Protocol 2019-09-11 (Junsi)](https://jetiforum.de/media/kunena/attachments/44/iCharger_MODBUS_Protocol_2019-09-11.pdf)
- [RC-Thoughts: iCharger touch software](https://www.rc-thoughts.com/icharger-touch-software/)
- [PRC 4010: iCharger 4010DUO (ProgressiveRC)](https://www.progressiverc.com/products/icharger-4010duo)
- [PRC 456: iCharger 456DUO (ProgressiveRC)](https://www.progressiverc.com/products/icharger-456duo)
- [BuddyRC X6: iCharger X6 800W 30A 6S](https://buddyrc.com/products/icharger-x6-800w-30a-6s-balance-battery-charger)
- [ToolkitRC M8D: product page](https://www.toolkitrc.com/m8d/)
- [RDQ: ToolkitRC M8D (RaceDayQuads)](https://www.racedayquads.com/products/toolkitrc-m8d-1600w-50a-1-8s-touch-screen-dual-channel-dc-smart-charger-xt60)
- [ToolkitRC Q4AC: product page](https://www.toolkitrc.com/q4ac/)
- [SkyRC: Q200neo product page](https://www.skyrc.com/q200neo)
- [Unmanned Tech: CNHL Black Series 1500mAh 4S 100C](https://www.unmannedtechshop.co.uk/products/cnhl-black-series-1500mah-14-8v-4s-100c-lipo-battery)
- [GetFPV R-Line: Tattu R-Line v4 1550mAh 4S 130C](https://www.getfpv.com/tattu-r-line-version-4-0-1550mah-4s-130c-lipo-battery-xt60.html)
- [Pyrodrone GNB: GNB 1550mAh 4S 120C](https://pyrodrone.com/collections/4s-batteries/products/gaoneng-gnb-1550mah-14-8v-4s-120c-lipo-battery-xt60)
- [GetFPV 6000: Tattu 6000mAh 4S 35C](https://www.getfpv.com/batteries/commercial-consumer-batteries/tattu-6000mah-4s-35c-lipo-battery-xt60.html)
- [rcdrone HRB: HRB 4S 6000mAh](https://rcdrone.top/products/hrb-4s-14-8v-lipo-battery-6000mah)
- [Molicel: INR21700-P42A datasheet](https://www.molicel.com/wp-content/uploads/INR21700P42A-V4-80092.pdf)
- [Keil & Jossen: Charging protocols for Li-ion and their impact on cycle life, J. Energy Storage 2016 (abstract)](https://cn.gamry.com/service-support-3/literature-database-zh-cn/publication/7151)
- [Parschau: Cycle tests on the influence of charging currents, Batteries 9(2) 83, 2023](https://www.mdpi.com/2313-0105/9/2/83)
- [Severson: Data-driven prediction of battery cycle life, Nature Energy 2019](https://www.nature.com/articles/s41560-019-0356-8)
- [BU-808: How to prolong lithium-based batteries](https://batteryuniversity.com/article/bu-808-how-to-prolong-lithium-based-batteries)
- [BU-401a: Fast and ultra-fast chargers](https://batteryuniversity.com/article/bu-401a-fast-and-ultra-fast-chargers)
- [BU-409: Charging lithium-ion](https://batteryuniversity.com/article/bu-409-charging-lithium-ion)
- [BU-410: Charging at high and low temperatures](https://batteryuniversity.com/article/bu-410-charging-at-high-and-low-temperatures)
- [Mean Well 350: LRS-350 datasheet](https://www.meanwell.com/Upload/PDF/LRS-350/LRS-350-SPEC.PDF)
- [Mean Well 600: LRS-600 datasheet](https://www.meanwell.com/Upload/PDF/LRS-600/LRS-600-SPEC.PDF)
