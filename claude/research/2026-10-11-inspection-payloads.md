# Inspection payloads that sell on small drones (11 Oct 2026)

**Bottom line**
1. The market leader flies the cheapest thermal core: Elios 3 carries a FLIR Lepton 3.5, 160 x 120 at 9 fps [E3 spec]. It weighs 0.9 g, draws 150 mW and costs $370 on a USB board [SparkFun]. Getting close matters more than pixels (E).
2. Gas, acoustic and UT payloads fight the drone itself. Point gas sensors take 10 to 60 s and read low in downwash [Burgués review]. Laser-methane and acoustic drone payloads weigh 0.22 to 1.4 kg [BL-CH4; MFE CRY]. UT needs contact, gel and a ~US$40k payload (E, from [CITF]).
3. Recommendation (E): Scout gets a Lepton 3.5 beside the DECXIN, for the demo and the first pilot. Overseer adds nothing for the demo; for a paid pilot it gets a 640 x 512 thermal plus 4K gimbal (SIYI ZT6 class). Gas, UT and acoustic wait until a customer asks.

## Findings

| Payload | Mass, power | Price | Interface | Output |
|---|---|---|---|---|
| FLIR Lepton 3.5 | 0.9 g core; 150 mW, 650 at shutter [SparkFun] | $370 on a USB board [SparkFun] | SPI + I2C (E); UVC on the board [SparkFun] | 160 x 120, 8.7 Hz, 57°, ±5 °C or 5% [SparkFun]; 56° [E3 spec] |
| InfiRay Tiny1-C | <2.5 g; 240 mW [InfiRay] | not found | SPI + I2C [InfiRay] | 256 x 192, 25 Hz, ±2 °C or 2% [InfiRay] |
| InfiRay P2 Pro | 9 g; 0.35 W [RR] | AUD 329 [RR] | phone USB-C; UVC plus vendor commands [P2Pro-Viewer] | 256 x 192, 25 Hz, 56 x 42° [RR] |
| Hikmicro Mini2Plus | 20 to 24 g [ICS] | dealer: no longer available [ICS] | Android USB-C [ICS] | 256 x 192, 25 Hz, <40 mK [ICS] |
| FLIR Boson+ 640 | 7.5 g without lens [GroupGets]; from 500 [OEM FLIR] or 900 mW [GroupGets] | $5,393 radiometric [GroupGets]; $8,295 as a drone module [Adorama] | USB, CMOS, MIPI [OEM FLIR] | ≤20 mK, 60 Hz [OEM FLIR] |
| SIYI ZT6 | 197 g; 6.5 W avg, 15 peak, 11 to 25.2 V [IRLock] | $3,700 [IRLock] | Ethernet, HDMI, UART; PX4 [IRLock] | 640 x 512 (±2 °C high gain) + 4K [IRLock] |
| SIYI A8 mini | 95 g; 5 W avg, 12 peak [SIYI] | not found | Ethernet, UART [SIYI] | 4K 25 fps, 6x digital [SIYI] |
| TDLAS methane | 220 g, 3 W [BL-CH4]; 360 g [SPH]; ~1 kg [Measur] | quote; ~$24k (E, unit not shown) [Maverick] | DJI E-Port [BL-CH4]; SkyHub [SPH] | 1 to 50 m, 5 ppm·m [BL-CH4]; 10 to 40 m [SPH] |
| Elios 3 LEL | 45.5 g [MFE gas] | quote | Elios only | 13 gases, T90 <20 s [MFE gas] |
| Elios 3 UT | not published | HK$310,185 [CITF] | Elios only | 0.8 to 250 mm steel, ±0.1 mm [Nexxis UT] |
| CRY2626G acoustic | 1.4 kg [MFE CRY] | quote | DJI SkyPort V2 [MFE CRY] | 128 mics, 2 to 48 kHz, 0.5 to 35 m [MFE CRY] |

**Thermal.** An uncooled microbolometer images heat at 8 to 14 µm [IRLock]; a radiometric core gives each pixel a temperature (E). Rivals all ship one: Elios 3 the Lepton [E3 spec]; Skydio X10 a 640 x 512 at <30 mK in both its sensor packages, tele and wide [MFE Skydio]; Spot a 640 x 512 at ±5 °C on a 7.5 kg head [CAM+IR]. Thermal is table stakes (E). Cores at 9 Hz or under fall in a lighter export class (6A993.a) than faster ones (6A003) [GroupGets ECCN; OEM FLIR], hence Lepton's 8.7 Hz (E). A reading needs ~3 x 3 pixels, so the Lepton's smallest spot is ~19 mm at 1 m and ~56 mm at 3 m; a Boson+ 640 on its 50° lens gets ~12 mm at 3 m (E). It finds loose or corroded connections, overloads and phase imbalance [Delta Wye]. Who pays: NFPA 70B wants electrical thermography every 12 months (6 for critical gear), surveys cost $500 to $15,000, and insurers push it [Delta Wye]. Staff open the panels and load should be over ~40% [Delta Wye], so a drone covers only exposed gear: bus duct, motors, conveyors, transformers, steam lines, roofs (E). Severity is the rise above normal: over 40 °C act now, 20 to 40 fix within 30 days, 10 to 20 monitor [Delta Wye].

**Gas.** TDLAS shines a 1.65 µm laser that methane absorbs and reads ppm·m along the beam [Measur]. It samples at 10 Hz and ignores downwash [Burgués review], but sees only methane. Buyers: pipelines, compressor stations, LNG, refineries, landfills [Measur; BL-CH4]. SENSIA sells on-board gas imaging cameras, not TDLAS, rated 0.35 g/h (cooled) to 1.4 g/h (uncooled) of methane [Develops]. Point sensors struggle: electrochemical cells take 30 to 60 s, MOX takes 10 to 20 s and swings with humidity, and both read low on a drone without a pump [Burgués review]. Rotor airflow fades 40 to 50 cm above and 70 to 80 cm beside the drone, but reaches metres below [Burgués review]. A MOX Crazyflie at 1 m/s missed a ceiling source by 3.3 to 4.0 m [Bitcraze]. Elios 3's gas sensor is a safety alarm at a set LEL; it stays powered 3 min through a pack swap, so no new bump test [MFE gas].

**UT.** A gel-coupled probe on the wall times echoes to read thickness, single-echo or echo-to-echo [Nexxis UT]. Elios 3 adds a gel dispenser, cleaning module and folding arm [Flyability UT]. Skygauge tilts its rotor arms with 8 servos so the body stays level, and carries an Olympus 38DL PLUS [DroneDJ]. A Hong Kong price-ceiling list puts the Elios 3 Premium package (lidar, 6 packs, 3 years of software) at HK$541,044, ~US$69k (E) [CITF]. Flyability claims up to 90% savings with two-person teams [DroneLife]. Buyers: ship class surveys (IACS-compliant probes) and integrity programs [Flyability UT].

**Zoom and stills.** Elios 3 gets 0.18 mm/px at 30 cm [E3 spec]; Skydio's 190 mm-equivalent tele reads a plate at 800 ft [MFE Skydio]. Indoors, subjects sit 0.3 to 10 m away, so light and stand-off beat zoom (E). The A8 mini's 4K over ~81° gives ~2 mm/px at 5 m, enough for a 100 mm gauge (E).

**Acoustic.** Arrays of 64 to 128 MEMS microphones locate leak hiss and partial discharge by beamforming [SV600; MFE CRY]. Handhelds work from 0.3 to 130 m at 1.25 to 1.7 kg [TEquipment]; the drone version needs three absorber layers and rotor-noise filtering [MFE CRY]. Buyers: utilities for partial discharge, pipelines for gas leaks [MFE CRY], and plants, where leaks can waste 20 to 30% of a compressor's output [DOE].

**Other.** Elios 3 sells a radiation payload tested to 10,000 R/h; Flyability says over 80% of US reactors have its drones [JGC RAD]. Elios 3 lifts 600 g of payload on its 1,900 g base [E3 spec].

## Steal for Nox
- **Scout:** Lepton 3.5, the Elios 3 core [E3 spec], on a PureThermal Mini Pro as a UVC camera into the Radxa [SparkFun], overlaid on the DECXIN view (E).
- **Scout v2:** Tiny1-C (<2.5 g, 256 x 192, 25 Hz) on SPI, if a Linux driver exists (E) [InfiRay].
- **Overseer:** ZT6-class thermal plus 4K for a paid pilot, a Boson+ 640 where US-made parts matter, an A8 mini (95 g) for stills only [IRLock; OEM FLIR; SIYI].
- **Data product:** rank hot spots by rise over a like part (other phase, twin motor), since Lepton's absolute error is ±5 °C [SparkFun; Delta Wye] (E).
- **Business:** sell rounds between annual NFPA 70B surveys, on gear seen without opening panels [Delta Wye] (E).
- **Box:** keep payloads powered through the swap, as the Elios 3 gas sensor is [MFE gas] (E).

## For Spark
- `scout.md`: the DECXIN takes the USB3.0 host port; the other is "USB_OTG / 5V IN: bench power and PC link only". A USB thermal camera needs that port in host mode or a tiny hub; or wire a bare Lepton's SPI and I2C to the header holes (E, CHECK).
- `scout.md` 5 V budget "Radxa ~1.5 A + camera ~0.3 A + XR2 0.1 + MTF-02P 0.1 (all E)": the Lepton adds ~30 mA (~130 mA at shutter) plus the board's unpublished draw (E, from [SparkFun]). The 5 A UBEC covers it.
- `nox-dimensions.md` §1 "500 g or a little more": Lepton plus board ~5 g, ~1.5% flight time (E). Its view is narrower than the DECXIN's 102° diagonal, so the v8c "no rod inside the frame" result should hold beside the lens (E, CHECK). `v2/README.md` "All-up ... ~400" has no payload line; add ~5 g (E).
- `nox-dimensions.md` §4 "6.6 kg static, T/W ~3.3": a ZT6 (197 g) gives T/W ~3.0 and ~15% less flight time with its own 6.5 W, an A8 mini ~8% less (E). SIYI streams over Ethernet, which the L2 already uses (`overseer.md`, "Ethernet to the Pi"): add a USB Ethernet dongle or switch, or move the L2 to its 4 Mbps UART (`overseer-scout-hardware-reference.md` §1.3) (E, CHECK).
- Sourcing (E): Hikmicro is part of Hikvision, so likely barred under NDAA §889; the Lepton is NDAA compliant and ITAR free [FLIR Lepton]. Check InfiRay and SIYI against the sourcing report before a US pilot.

## Sources
- [E3 spec: Elios 3 tech spec PDF (Nexxis)](https://nexxis.com/wp-content/uploads/sites/3/2022/06/20220519_E3_Tech_Spec_web_version.pdf)
- [Flyability UT: Elios 3 UT payload (Flyability)](https://www.flyability.com/elios-3-ut-payload)
- [Nexxis UT: Elios 3 UT payload (Nexxis)](https://nexxis.com/product/elios-3-ut-payload/)
- [DroneLife: Flyability unveils Elios 3 UT payload](https://dronelife.com/?p=100883)
- [CITF: price ceilings for Elios 3 packages (HK CITF PA25-014)](https://citf.cic.hk/product_Photo/files/PA_Technologies/PA25-014%20upload.pdf)
- [MFE gas: Elios 3 Flammable Gas Sensor (MFE)](https://mfe-is.com/product/elios-3-flammable-gas-sensor/)
- [JGC RAD: Elios 3 RAD payload (JGC)](https://www.jgc.gr/flyability/drones-and-accessories/elios-3-rad-payload/?lang=en)
- [SparkFun: PureThermal Mini Pro JST-SR with Lepton 3.5](https://www.sparkfun.com/purethermal-mini-pro-jst-sr-with-flir-lepton-3-5.html)
- [FLIR Lepton: Lepton product page (Teledyne FLIR)](https://www.flir.com/products/lepton/)
- [OEM FLIR: Boson+ (Teledyne FLIR OEM)](https://oem.flir.com/en-au/products/boson-plus/)
- [GroupGets: Radiometric FLIR Boson+ 640](https://groupgets.com/boson640)
- [GroupGets ECCN: Radiometric FLIR Boson 640 listing](https://groupgets.com/products/in-stock-radiometric-flir-boson-640)
- [Adorama: Freefly Boson+ 640R module for LR1](https://www.adorama.com/freefly-boson-640r-thermal-pro-camera-module/p/fr91000904)
- [InfiRay: Tiny1-C micro IR module](https://www.infiray.com/products/tiny1-c-micro-uncooled-ir-camera-sensor-module.html)
- [RR: InfiRay P2 Pro (Reduction Revolution)](https://reductionrevolution.com.au/products/infiray-p2-pro)
- [P2Pro-Viewer (GitHub)](https://github.com/LeoDJ/P2Pro-Viewer)
- [ICS: Hikmicro Mini2Plus (ICS Schneider)](https://www.ics-schneider.de/produkte/mini-serie-mini2plus-hikmicro-waermebildkamera-mit-manuellem-fokus-aufloesung-256x192-25hz-und-usb-c/?lang=en)
- [IRLock: SIYI ZT6 mini optical pod](https://irlock.com/products/siyi-zt6-mini-optical-pod-dual-sensors-4k-8mp-6x-digital-zoom-gimbal-camera)
- [SIYI: A8 mini specs](https://siyi.biz/en/product/tri-axis-single-camera-gimbal/a8-mini/spec)
- [MFE Skydio: Skydio X10 (MFE)](https://mfe-is.com/product/skydio-x10/)
- [CAM+IR: Spot CAM+IR datasheet (Boston Dynamics)](https://www.bostondynamics.com/wp-content/uploads/2023/05/spot-cam-plus-ir.pdf)
- [Delta Wye: Electrical thermography inspection guide 2026](https://deltawye.com/electrical-thermography-inspection/)
- [Measur: Laser Falcon methane detection](https://measur.ca/pages/learn/methane-detection-laser-falcon)
- [SPH: Falcon Plus TDLAS methane detector (SPH Engineering)](https://shop.sphengineering.com/products/falcon-plus-tdlas-methane-detector)
- [Maverick: Laser Falcon 2 listing](https://www.maverickdrone.com/products/pergam-laser-methane-detector-laser-falcon-2-plus)
- [BL-CH4: BLV BL-CH4 Mini methane detector (GNSS.ae)](https://gnss.ae/drones/payloads-sensors/blv-bl-ch4-mini-methane-detector-for-dji-m3d-td-dock-2/)
- [Develops: SENSIA on-board OGI cameras](https://develops.com.es/?p=12326)
- [Burgués review: Environmental chemical sensing using small drones (Sci. Total Environ. 2020)](https://diposit.ub.edu/dspace/bitstream/2445/203887/1/239333.pdf)
- [Bitcraze: Gas sensing with a Crazyflie (Burgués, 2019)](https://www.bitcraze.io/?p=7583)
- [DroneDJ: Skygauge inspection drone pre-order](https://dronedj.com/2020/07/01/skygauge-inspection-drone-now-available-for-pre-order/)
- [MFE CRY: CRY2626G UAV acoustic imager (MFE)](https://mfe-is.com/product/crysound-2626g-uav-acoustic-imager/)
- [SV600: Fluke SV600 datasheet (Boston Dynamics)](https://bostondynamics.com/wp-content/uploads/2024/09/fluke-sv600.pdf)
- [TEquipment: Fluke ii910 vs FLIR Si124](https://www.tequipment.net/Fluke-ii910-vs-Flir-Si124-Cordless/)
- [DOE: Compressed air tip sheet on leaks (NREL)](https://nrel.gov/docs/fy01osti/28008.pdf)
