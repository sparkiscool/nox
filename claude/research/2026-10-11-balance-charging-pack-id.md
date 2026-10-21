# Balance charging and pack ID without the pad PCB (11 Oct 2026)

**Bottom line**
1. X5 slots with printed v18 cartridges: charge through the XT60 only, with one small charger per slot, to 4.15 V per cell (16.6 V). Add a resting-voltage check, an mAh cap and a steel tray. Bench-balance each pack every ~10 cycles, and retire any pack past a 50 mV spread (E).
2. ID: the swap script tracks each pack by its slot (0 g). Once people start moving packs by hand, add an NFC sticker to each cartridge and a reader in the gripper. That adds about 0.1 to 1 g per pack and needs no contacts (E).
3. Next carriers (cartridge v19, Overseer cartridge, S2 pack frame): give each pack recessed balance and ID pads that only the bay pins reach (~1 g per pack, E). Skip SMBus smart packs for the demo. They add ~5 g of BMS (E) and balance at only 10 mA [BQ40Z50], and the one kit PX4 documents is discontinued [Batmon].

## Findings
**Why balance.** Cells from one batch differ in capacity, self-discharge and impedance, so their voltages drift apart. The weak cell reads high at full charge and ages faster [SLYT322]. Renesas works a 4S example without balancing: it drifts 1.1 to 1.6 mV per cycle and loses 9% of capacity by 100 cycles and ~50% by 400. A 100 mV spread costs ~5.9%, and 10 mA of balancing needs over 16 h to clear it [AN141]. Cycle life is 300 to 500 at 4.20 V. At 4.13 V it is 400 to 700, keeping 90% of the energy [BU-808].

**No balance lead.** The charger holds only the pack total. One high cell sits 3/4 of the spread above the average. So at 16.6 V it reaches 4.20 V only at a 67 mV spread, and 10 cycles of drift add 11 to 16 mV (E). The cost is ~7% of the energy (E). A shorted cell would push the other three toward 5.5 V each. So refuse any pack resting under 13.2 V, and stop at ~110% of the mAh flown out (E).

**Balance lead through the bay.** JST-XH is rated 3 A at 2.5 pitch, 10 mΩ new and 20 mΩ after environmental tests. Neither JST sheet I read gives a mating-cycle rating [JST XH], and its shroud suits a hand, not a robot (E). Spring pins on pads last 1,000,000 cycles at ~$1.5 each (contacts report). Amass's XT30 (2+2) adds two signal pins [LCSC]. That is enough for ID or SMBus, but not for three cell taps (E).

**Balancer on the pack.** HY2213: one SOT-23-6 per cell switches an external FET and resistor at 4.20 V ±25 mV and draws 2.5 µA [HY2213]. ABLIC's S-8209B works the same way; its example bleeds ~80 mA [S-8209B]. TI BQ77915: 3 to 5 cells, no MCU, balances on its own at up to 50 mA, 8 µA [BQ77915]. Cheap 4S active boards move 0.5 to 1.2 A, and their idle draw is unpublished [Sigmanortec]. A bleed stuck on at 80 mA empties a 1500 mAh cell in under a day (E).

**Smart packs.** BQ40Z50-R2: 1 to 4 cells, SMBus, SHA-1, 10 mA bypass, external high-side FETs, 336 µA normal, 4 x 4 mm [BQ40Z50]. BQ76905 balances at 50 mA, under host control [BQ76905]. PX4's batt_smbus reads cell voltages every cycle. It reads serial, cycles and health only at start, and never checks for a new serial [batt_smbus]. DJI's Phantom 3 pack (BQ30Z55 + MSP430) still brings out its cell taps. The Mavic 2 pack (BQ9003) exposes only power and SMBus [o-gs]. Spektrum Smart G2 replaces the balance lead with one data wire. Its 4S 2200 weighs 260 g [Spektrum], 8.5 mAh/g against 9.1 for the Scout pack (E). Tattu Plus balances itself after 6 h idle [Tattu], but the smallest found is 6S 10,000 mAh [Gensace]. Two of the Mavic's 1.55 mΩ FETs [o-gs] would burn ~2.8 W at 30 A (E).

**Pack ID** (ground through the XT60, E)
| Method | On the pack | Box side | Notes |
|---|---|---|---|
| Slot tracking | nothing | script | lost when a person moves a pack (E) |
| Resistor | 0603 | 1 pad, ADC | 7 codes in today's ladder; no memory |
| 1-Wire DS28E07 | ~0.1 g (E) | 1 pad, 1-Wire master | 64-bit ID, 1 kbit, 10,000 writes, $0.86; a package made for spring contacts [DS28E07] |
| NFC NTAG213 | 0.1 to 1 g (E) | reader ~$10 (E) | 7-byte UID, 144 bytes, 100,000 writes, ~10 cm [NTAG213]; ferrite inlays for metal [Avery] |
| SMBus gauge | ~5 g (E) | 2 pads | serial, cycles and health into PX4 [batt_smbus] |

## Steal for Nox
- Box charger: in each slot, a CC/CV buck off the 24 V rail at 16.6 V and ~3 A. It stops at a ~150 mA taper or a timer, with an INA226 to read it (keep-alive report parts) (E).
- Swapper (Scout gripper): read the cartridge's NFC tag on every pick, and keep the pack's cycle count in it (E).
- X5 slots: fit pins only where a pad exists. On v18, leave the bores empty (E).
- Cartridge v19 and Overseer cartridge (X9): B-, C1, C2, C3, B+ and ID pads, recessed so only the bay pins reach them (E).
- Scout v2: three mid-tap pads beside the power strips, for the cradle (E).
- Firmware: with smart packs, restart batt_smbus after each swap, or PX4 keeps the old serial and cycle count (E).
- Box software: keep a per-pack log of cycles, mAh in vs out and overnight voltage drop, and flag outliers (E).

## For Spark
- `box.md` X5 "pocket floor + 6 spring pins" and `scout-cartridge-pocket.md` Bays "Six pins push up 3-6 N total": v18 has no pads, so these pins would only lift the cartridge. Print slot v1 without them (E).
- `nox-dimensions.md` §2 "Scout S1-S4: 1.0 k / 2.2 k / 4.7 k / 10 k": nothing reads these on v18. Track by slot now, and add NFC or a DS28E07 later (E).
- `box.md` "ISDT CM1620, relays, ADCs, Nano": it is a one-channel module [Zbotic]: 1 to 20 A, 2 to 16S, CAN and RS-485, 636 g, $179.99 [Pyrodrone]. Buyers report a 16-pin balance port [BuddyRC]. Shared by relays, it must refill a Scout pack every ~10 min (~5C, E). I found nothing from ISDT on charging with that port empty, or to 16.6 V (CHECK on the bench).
- `box.md` X10 "two groups, 7 bays": in a group charged in parallel, a full pack and an empty one dump current into each other. Use one charger per slot (E).
- `scout-cartridge-pocket.md` "where the cell-voltage pads go on the next battery pack": §2's "26 x 8" strip folds to ~12 x 12 as two rows of three (E). Placement is a CHECK on the mesh. Keep the "0.5 A PTCs inline" (E).
- `v2/s2-contacts.md` Problem 5 "a charge cradle with 8 of the spare pins": S2 packs would never get balanced. `v2/dimensions.md` already routes the "balance lead" out the tail, so end it on mid-tap pads (E).
- `nox-dimensions.md` §1 "~9 min to 20 % reserve": this becomes ~8.2 to 8.4 min at 4.15 V per cell (E).

## Sources
- [TI SLYT322: Cell balancing buys extra run time and battery life](https://ti.com/lit/pdf/slyt322) [SLYT322]
- [Renesas AN141: Benefits of cell balancing](https://www.renesas.com/en/document/apn/an141-benefits-cell-balancing) [AN141]
- [Battery University BU-808: How to prolong lithium-based batteries](https://batteryuniversity.com/article/bu-808-how-to-prolong-lithium-based-batteries) [BU-808]
- [JST XH connector datasheet](https://lms.ee.hm.edu/~seck/AlleDateien/EMUF/DataSheets/JST_B12B-XH-A/EXH.pdf) [JST XH]
- [JST XH contact sheet](https://cdn3.components.ru/pdf/9886.pdf) [JST XH]
- [LCSC: Amass XT30PW(2+2) power and signal plug](https://www.lcsc.com/product-detail/C19268030.html) [LCSC]
- [HYCON HY2213 datasheet](https://static.chipdip.ru/lib/240/DOC041240870.pdf) [HY2213]
- [ABLIC S-8209B application note](https://www.ablic.com/en/doc/datasheet/battery_protection/S8209B_AN_E.pdf) [S-8209B]
- [TI BQ77915 product page](https://www.ti.com/product/BQ77915) [BQ77915]
- [Sigmanortec: 4S 1.2 A active balancer](https://sigmanortec.ro/en/active-balance-mode-lithium-4s-12a) [Sigmanortec]
- [TI BQ40Z50-R2 product page](https://www.ti.com/product/BQ40Z50-R2) [BQ40Z50]
- [TI BQ40Z50-R2 datasheet](https://www.ti.com/lit/ds/symlink/bq40z50-r2.pdf) [BQ40Z50]
- [TI BQ76905 datasheet](https://www.ti.com/lit/pdf/SLUSE97) [BQ76905]
- [PX4 batt_smbus.cpp](https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/src/drivers/batt_smbus/batt_smbus.cpp) [batt_smbus]
- [PX4 batt_smbus.h](https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/src/drivers/batt_smbus/batt_smbus.h) [batt_smbus]
- [PX4 docs: Rotoye Batmon (discontinued)](https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/docs/en/smart_batteries/rotoye_batmon.md) [Batmon]
- [o-gs: Phantom 3 battery intelligent board](https://github.com/o-gs/dji-firmware-tools/wiki/P3-Battery-Intelligent-board) [o-gs]
- [o-gs: WM240 (Mavic 2) battery intelligent board](https://github.com/o-gs/dji-firmware-tools/wiki/WM240-Battery-Intelligent-Board) [o-gs]
- [Kiwiquads: Spektrum 2200 mAh 4S Smart G2 LiPo](https://kiwiquads.co.nz/product/spektrum-2200mah-4s-14-8v-smart-g2-lipo-30c-ic3/) [Spektrum]
- [Tattu Plus 1.0 manual](https://cdn.shopify.com/s/files/1/0836/7770/8622/files/6_Tattu_Plus_1.0_Manual.pdf) [Tattu]
- [Gensace: Tattu Plus 10000 mAh 6S](https://www.gensace.de/tattu-plus-10000mah-22-2v-25c-6s1p-lipo-battery-pack-with-as150-xt150-plug.html) [Gensace]
- [ADI DS28E07 product page](https://www.analog.com/en/products/ds28e07.html) [DS28E07]
- [ADI DS28E07 datasheet](https://www-cce.cldnet.analog.com/media/en/technical-documentation/data-sheets/DS28E07.pdf) [DS28E07]
- [Wakdev: NXP NTAG213](https://www.wakdev.com/en/knowledge-base/nfc-chips/nxp-ntag213.html) [NTAG213]
- [Avery Dennison: Circus On-Metal NFC inlay](https://rfid.averydennison.com/en/home/product-finder/circus-on-metal.html) [Avery]
- [Zbotic: ISDT CM1620 1-channel charger module](https://zbotic.in/product/isdt-cm1620-1-way-1100-w-industrial-charger-module-20-a-1-channel-supports-2-16s-lipo-life-lihv-can-rs-485/) [Zbotic]
- [Pyrodrone: ISDT CM1620](https://pyrodrone.com/collections/isdt-products/products/isdt-cm1620-1100w-20a-lipo-battery-charger-charging-module-for-rc-models-2-16s-life-lipo-lihv-battery) [Pyrodrone]
- [BuddyRC: ISDT CM1620, buyer reviews](https://www.buddyrc.com/products/isdt-cm1620-modular-charger) [BuddyRC]
