# Indoor radio links for multi-drone inspection in steel-heavy plants (11 Oct 2026)

**Bottom line**
1. Demo: ELRS 2.4 GHz for sticks and kill, one handset per drone; one 5 GHz Wi-Fi access point at the box for MAVLink, ROS 2 and live video; full video recorded onboard. About 2 g and 0.5 to 1 W added per drone (E).
2. Steel halls hurt through path loss and fading, not echoes: NIST measured path loss exponents up to 5.9 but mean delay spreads of only 36 to 162 ns [NIST TN 1951]. Behind machinery, ~10 Mbps Wi-Fi reaches ~25 to 35 m, ELRS ~100 to 200 m (E).
3. Field teams bring an antenna closer rather than fly mesh radios: Elios 3 has one 100 mW link, a 20 m cabled antenna and return-to-signal; SubT drones joined dropped nodes over Wi-Fi. Mesh and relay radios (7 to 123 g, 4 to 13 W) suit the Overseer or box later.

## Findings

**ELRS control.** Slower rates hear weaker signals: 2.4 GHz LoRa 250 Hz -108 dBm, FLRC F1000 -104; 900 MHz 100 Hz -117 [ELRS signal health]. MAVLink mode carries ~1,470 B/s down at 333 Hz Full [ELRS MAVLink]: telemetry, not video. With Model Match on, a receiver numbered for another model stays bound but feeds its FC nothing [ELRS Model Match].

**MAVLink, two vehicles.** Each FC needs its own MAV_SYS_ID; all send to UDP 14550 and QGC lists each [MicoAir guide]. One PX4 user's central mavlink-router filter mixed vehicles up; a router on each drone, each to its own port, fixed it [PX4 forum].

**Video.**
- Companion Wi-Fi: the Radxa Zero 3W hardware-encodes H.264/H.265 to 1080p60; its Wi-Fi bands are unpublished [Radxa]. Over UDP a bad link drops frames instead of stalling [ModalAI].
- OpenIPC / WFB-ng: monitor-mode packet injection, no association or ACKs; FEC rebuilds 4 lost packets in 12 [wfb-ng]. RunCam WiFiLink 2: 25 to 30 g, 28 dBm [OpenIPC docs]; ~25 ms, but ~4 Mbps 720p that breaks up near obstacles [Oscar Liang].
- DJI O4: 8.2 g, 20 ms with DJI goggles, 5 GHz only [DJI O4].
- HaLow (902 to 928 MHz): 15 Mbps PHY at 4 MHz, 21 dBm [MM6108], ~0.7 W transmitting (E). A rover got usable 480p to 200 to 300 m, with bursty loss [ArduPilot forum].

**Mesh radios.**
| Radio | Band, max out | Mass | Power | Rate, latency |
|---|---|---|---|---|
| Doodle Mini OEM [Doodle Mini] | 2.4 GHz, 1.25 W | 36.5 g | 5 W avg, 8 peak | 80 Mbps; 1.5 to 10 ms |
| Doodle Nano [Doodle Nano] | 1.35 to 2.51 GHz | 25 to 26 g | n/a | 80 Mbps |
| Silvus SL5200 [Silvus SL5200] | L, S band, 2 W | 26 to 52 g | 4 to 13 W | 100 Mbps; 7 ms |
| Rajant DX2 [Rajant DX2, CMU thesis] | 5 GHz | 123 g | n/a | 300 Mbps |
| Microhard pMDDL2450 [Microhard] | 2.4 GHz, 1 W | 7 g module | n/a | 25 Mbps; PTP, PMP, relay |

Prices are quote-only; one distributor search result lists a Doodle RM-2450-12M3 at $1,305 [Dasenic].

**Steel and multipath.** NIST sounded a car plant, machine shop and steam plant at 2.2 and 5.4 GHz [NIST TN 1951]. Mean delay spreads: 116 to 162, 70 to 74, 36 to 45 ns. Path loss exponents: 3.9 to 5.9, 1.6 to 1.9, 2.7 to 2.8. NIST says spreads under 500 ns may not matter at 256 kbps; Wi-Fi's 0.8 µs guard interval mostly covers them (E), so blockage sets range. Reach behind clutter (E: 17 dBm Wi-Fi, 250 mW ELRS, 18 dB margin, free space to 10 m, then exponent 3.9 to 5.9): 5 GHz Wi-Fi 24 to 36 m, HaLow 60 to 110 m, ELRS 2.4 GHz 95 to 220 m, ELRS 900 MHz 190 to 560 m.

**Field practice and relays.** Elios 3: 2.4 GHz, 20 dBm or less, 18 Mbps down, 500 m line of sight, no indoor figure [E3 spec]. Its extender is an antenna on a 20 m cable [JGC]; on link loss it retraces up to 300 m, with 5 s to cancel [DroneDJ]. CERBERUS walkers dropped 245 g Rajant nodes where a human chose; drones joined the nearest node over Wi-Fi; only the selected robot streamed cameras [CERBERUS]. CMU planned on 30 m per node underground [CMU thesis]. CoSTAR robots out of range return or carry data back [NeBula]. A one-radio Wi-Fi repeater halves throughput (E).

**US rules.** 902 to 928, 2400 to 2483.5 and 5725 to 5850 MHz: 1 W conducted, less with few hop channels or antennas over 6 dBi [eCFR 15.247]. DFS channels must stop within 10 s of radar; 5.925 to 7.125 GHz is barred for drone control or comms [eCFR 15.407].

**Demo setup (all E).**
| Node | Control | Data and video | Added |
|---|---|---|---|
| Scout | XR2, 250 Hz LoRa, own handset | Radxa on 5 GHz: MAVLink, DDS, 720p at 2 to 4 Mbps; 1080p to SD | ~2 g dipole, 0.5 to 1 W |
| Overseer | RP3 V2, own handset | Pi 5 on 5 GHz: MAVLink, submaps, stills | 0 g, 0.5 to 1 W |
| Box | | AX3000T as AP on a 1.5 to 2 m mast, wired to the laptop | mains |
| Operator | two handsets, two people | QGC, two system IDs | |

## Steal for Nox
- Box: AP on non-DFS 5 GHz (36 to 48, 149 to 165), 20 MHz wide, leaving 2.4 GHz to ELRS [eCFR 15.407] (E).
- Box: Elios-style extender, a second wired AP on a 10 to 20 m cable, placed by hand at a doorway [JGC, CERBERUS] (E).
- Firmware (both companions): Wi-Fi power save off; hold position if operator packets stop for 0.5 s (E).
- Firmware (both): return-to-signal along the logged path, 5 s to cancel [DroneDJ].
- Comms: essential traffic always, video only from the selected drone [CERBERUS]; ROS 2 discovery via a discovery server, since Wi-Fi sends multicast at its lowest rate (E).
- Scout: ELRS 250 Hz LoRa, not F1000, for 4 dB more margin [ELRS signal health].
- Overseer (later): SL5200 or Doodle Nano relaying the Scout behind racks, 25 to 52 g, 4 to 13 W [Silvus SL5200, Doodle Nano].

## For Spark
- `scout.md` Radio, "Pocket model "Scout" = copy of the Overseer model": both receivers bind to one module (E). With Model Match off, both drones may take the same sticks, arm gesture included. With it on, the deselected drone loses RC and lands ("NAV_RCL_ACT 3", Overseer params). Fix: a second handset with its own phrase and a safety pilot for the Overseer; test props off with both powered (E).
- `claude/overseer_params_2026-09-26.params`: "NAV_DLL_ACT 0", so a Wi-Fi drop triggers nothing, and "COM_OF_LOSS_T 1" fires only if the companion itself stops sending. Add the companion watchdog above (E). Also: "MAV_SYS_ID 1", so set the Scout to 2; "UXRCE_DDS_NS_IDX -1" in domain 0 on both merges their /fmu topics, so give each a namespace (E).
- `claude/overseer-scout-hardware-reference.md` §1.5, "915/433 illegal to transmit on in the US": 902 to 928 MHz is a Part 15 band [eCFR 15.247]. Its §4 "WiFi 6E": 6 GHz is barred for drone links [eCFR 15.407].
- `scout.md` Problem (c), "fall back to the u.FL antenna": the Radxa now sits on the rear cage triangle (`nox-dimensions.md` §1), near the XR2 mount. Fit a dipole, confirm 5 GHz with `iw list`, keep Wi-Fi off 2.4 GHz (E).
- `claude/overseer-scout-hardware-reference.md` §4, "Stream capped at ~15 fps": the Pi 5 lacks a hardware H.264 encoder (E), so Overseer video competes with its unbenchmarked lidar odometry. Let the Scout carry operator video (E).

## Sources
- [ELRS signal health: Signal Health (ExpressLRS)](https://www.expresslrs.org/info/signal-health/)
- [ELRS MAVLink: MAVLink (ExpressLRS)](https://www.expresslrs.org/software/mavlink/)
- [ELRS Model Match: Model Config Match (ExpressLRS)](https://www.expresslrs.org/software/model-config-match/)
- [MicoAir guide: MicoAir WiFi Link multi-drone networking guide](https://micoair.com/docs/micoair_wifi_link_multi_drone_networking_guide/)
- [PX4 forum: mavlink-router filtering by MAV_SYS_ID](https://discuss.px4.io/t/mavlink-router-filtering-by-mav-sys-id/44875)
- [Radxa: Radxa Zero 3W product page](https://radxa.com/products/zeros/zero3w/)
- [ModalAI: Low latency video streaming](https://docs.modalai.com/camera-video/low-latency-video-streaming/)
- [wfb-ng: WFB-ng repository (GitHub)](https://github.com/svpcom/wfb-ng)
- [OpenIPC docs: RunCam WiFiLink 2](https://docs.openipc.org/hardware/runcam/vtx/runcam-wifilink-v2/)
- [Oscar Liang: Exploring OpenIPC](https://oscarliang.com/openipc)
- [DJI O4: O4 Air Unit specs](https://dji.com/o4-air-unit/specs)
- [MM6108: Morse Micro MM6108-MF08651 US data sheet](https://morsemicro.com/wp-content/uploads/2025/01/MM6108-MF08651-US-Data-Sheet-v1-1.pdf)
- [ArduPilot forum: HaLow WiFi telemetry + video](https://discuss.ardupilot.org/t/halow-wifi-telemetry-video/138734)
- [Doodle Mini: Mini OEM Mesh Rider 2450 datasheet (Mouser)](https://mouser.com/datasheet/2/895/miniOEM_2450-3446604.pdf)
- [Doodle Nano: Nano Mesh Rider radio](https://doodlelabs.com/product/nano-mesh-rider-radio)
- [Dasenic: Doodle Labs RM-2450-12M3 listing](https://dasenic.com/product/doodle-labs-rm-2450-12m3-8186357)
- [Silvus SL5200: SL5200 OEM module datasheet](https://silvustechnologies.com/wp-content/uploads/2024/10/Silvus_SL5200_OEM_Module_Datasheet_DL1_DIGITAL.pdf)
- [Rajant DX2: DX2 spec sheet (Matco)](https://ftp.matco.com.mx/uploads/solutions/Folleto_de_Especificaciones_DX2_Matco.pdf)
- [Microhard: pMDDL2450 brochure (ARK Electronics)](https://arkelectron.com/wp-content/uploads/2021/02/pMDDL2450.Brochure.Rev_.1.2.0.pdf)
- [NIST TN 1951: Industrial wireless systems, radio propagation measurements](https://nvlpubs.nist.gov/nistpubs/TechnicalNotes/NIST.TN.1951.pdf)
- [E3 spec: Elios 3 tech spec (Nexxis)](https://nexxis.com/wp-content/uploads/sites/3/2022/06/20220519_E3_Tech_Spec_web_version.pdf)
- [JGC: Flyability range extender](https://www.jgc.gr/products/flyability/range-extender/?lang=en)
- [DroneDJ: Elios 3 return-to-signal](https://dronedj.com/2023/12/11/elios-3-drone-signal-loss/)
- [CERBERUS: Team CERBERUS wins the DARPA SubT Challenge (arXiv)](https://arxiv.org/pdf/2207.04914)
- [CMU thesis: Communications coverage in unknown underground environments (Tatum)](https://publications.ri.cmu.edu/storage/publications/2020/07/mtatum_thesis_final.pdf)
- [NeBula: Team CoSTAR at the DARPA SubT Challenge (arXiv)](https://arxiv.org/pdf/2103.11470)
- [eCFR 15.247: 47 CFR 15.247](https://www.ecfr.gov/current/title-47/chapter-I/subchapter-A/part-15/subpart-C/subject-group-ECFR2f2e5828339709e/section-15.247)
- [eCFR 15.407: 47 CFR 15.407](https://www.ecfr.gov/current/title-47/chapter-I/subchapter-A/part-15/subpart-E/section-15.407)
