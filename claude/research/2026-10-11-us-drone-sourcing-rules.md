# US drone sourcing rules a facility buyer will ask about (11 Oct 2026)

Not legal advice. Items marked "counsel" need a lawyer before any sale.

**Bottom line**
1. Since 22 Dec 2025, every new foreign-made drone and every new drone-specific part is on the FCC Covered List. That covers flight controllers, radios, motors, batteries, sensors, cameras and ground stations. Covered gear gets no new FCC authorization, but models authorized before that date can still be sold and used [FCC PN; FCC FAQ]. A US-made drone is clear if it is on the Blue UAS list, has more than 65% US component cost, or holds a DoW conditional approval. The first two exemptions run to 1 Jan 2028 [DA 26-768].
2. Purchase bans bind four groups:
   - federal agencies, contractors and grant-funded work (ASDA, from 22 Dec 2025);
   - DoD and its contractors, under the s848 parts list: flight controllers, radios, data links, cameras, gimbals, ground stations and software from China;
   - Unitree-type 1260H companies, which DoD shuts out (s805);
   - public bodies in states such as Florida and Tennessee.
   No rule found binds a private utility, refinery or chemical plant, though their security teams copy these lists (E).
3. Nox today fails the DoD parts test on the FC, ELRS radios and camera, and likely on the flow sensor. That is fine for the investor demo (E). A federal-ready variant costs about $1.1k to $2k more per Scout and $5.5k to $7k more per Overseer, mostly for the lidar (E).

## Findings

**FCC Covered List (enacted)**
- On 22 Dec 2025 the FCC added UAS and UAS critical components produced in a foreign country. It also added all DJI and Autel communications and video gear under FY2025 NDAA s1709 [FCC PN]. The basis was a 21 Dec 2025 determination by a White House interagency group [FCC PN].
- The critical-component list is not limited to these: data links and communications, flight controllers, ground stations and controllers, navigation, sensors and cameras, batteries and BMS, motors, plus their software [FCC PN]. The FCC counts only parts designed mainly for drones, so a multi-use camera is outside [FCC FAQ].
- Effect: covered gear gets no new equipment authorization. The applicant certifies in good faith where the item was produced; the maker's nationality does not matter [FCC FAQ]. Models authorized earlier can still be imported, sold and used [FCC FAQ]. Their firmware updates are allowed to at least 1 Jan 2029 [Steptoe].
- Exemptions:
  - Blue UAS items and Buy American domestic end products, from 7 Jan 2026 [FCC FAQ]. On 21 Jul 2026 these were extended to 1 Jan 2028 [DA 26-768]. "Domestic end product" means US-made with over 65% US component cost, rising to 75% in 2029 [Morgan Lewis].
  - Toy drones, from 15 Jun 2026 [DA 26-768]: under 150 g, no GPS, no camera, no brushless motors [Steptoe].
- Conditional approvals:
  - How to apply: email drones@fcc.gov. DoW and DHS decide, based on a parts list with origins, owners holding 5% or more, and a dated onshoring plan for every critical part [Steptoe; FCC FAQ].
  - Who has one: the first four came on 18 Mar 2026, including ScoutDI [Covington]. By 21 Aug 2026 there were about 20, including Verity Series 4 and Sees.ai [DA 26-768; DA 26-875].
  - How long: an approval lasts while the onshoring plan is kept [DA 26-875].
- Gap: the FAQ clears a US-built drone with foreign parts only if it is Blue-listed or Buy American. It says nothing about a drone that is neither [FCC FAQ]. Counsel.
- Related FCC rules:
  - Enacted 25 Nov 2025: devices with covered modular radios can't be authorized, and a process can limit existing grants [FR 2025-21001].
  - Enacted 11 Sep 2026: devices with logic hardware from Covered List entities are barred [FR 2026-18535].
  - Proposed only, 7 Aug 2026: bar any covered part and require hardware and software bills of materials [FR 2026-16197].
  - Foreign-made routers joined the Covered List on 23 Mar 2026 [DA 26-768].

**American Security Drone Act (enacted in the FY2024 NDAA, Dec 2023)**
- Targets drones made or assembled by an entity on the FASC list in SAM [FAR 52.240-1]. That list holds every Consolidated Screening List entity, with DJI and Autel named [Rochester; ABC].
- From 22 Dec 2025, contractors may not operate such a drone on a federal contract, or buy or run one with federal money. The clause flows down to all subcontracts [FAR 52.240-1].
- It also covers grant and cooperative-agreement work [Rochester].
- It looks at who made the drone, not where the parts came from (E).

**DoD parts rule (FY2020 NDAA s848, enacted)**
- DoD may not buy or operate a drone that is made in China or has any of these from China:
  - flight controllers, radios, data transmission devices, cameras or gimbals;
  - a ground station or operating software developed there;
  - network or data storage run by a Chinese entity [s848].
- Contractors are bound through DFARS 252.225-7972 [Rochester].
- Buyers usually mean this list when they say "NDAA compliant" (E). Motors, ESCs and batteries are not on it [s848].

**1260H list and s805 (FY2024 NDAA, enacted)**
- DoD may not contract with listed Chinese military companies from 30 Jun 2026. From 30 Jun 2027 it may not buy goods that contain their products [OSD 805].
- Unitree was designated on 10 Jun 2026 [FR 1260H].

**FY2026 NDAA (enacted Dec 2025; likely P.L. 119-60 of 18 Dec 2025, E [Cornell])**
- The only sourcing change found extends the ban to Chinese ground robots (s899A to C) [sUAS News]. The other drone provisions found are counter-drone powers.
- Full text not read.

**Blue UAS**
- DIU moved the Cleared List (whole drones) and the Framework (parts and software) to DCMA on 3 Dec 2025 [DIU]. At 19 Nov 2025 it held 39 platforms, 165 components and 81 companies [DIU]. The list is now at bluelist.dcma.mil [IUS DCMA].
- Recognized Assessors check NDAA compliance [DIU]. The cost of getting listed is not published.

**Section 232 tariffs (enacted by proclamation, 13 Aug 2026)**
- 100% on drones over 25 kg or with thermal imaging, their docks and some parts [Wiley].
- 25% on other drones from 3 Sep 2026, and on their listed parts from 9 Feb 2027 [Wiley].
- 15% from the EU, Japan, Korea, Switzerland and Taiwan; 10% from the UK [Wiley; Thompson Hine].
- Annex contents (which hobby parts are listed) not checked.

**States**
- Florida: state, county and city bodies may buy only from approved makers, and had to stop using others by 1 Jan 2023 [Fla. 934.50]. A 2023 rule replaced the approved list with country-of-concern limits and added $25M for replacements [Officer].
- Tennessee: from 1 Jul 2023, government buys only federally approved models [Officer].
- Arkansas: police phase out drones from covered entities by 2027 [Officer].
- Texas: HB 41 (2025) would bar drones from makers based in, or controlled from, a DNI-named country, with use banned from 1 Jan 2026 [Tex. HB 41]. Proposal; whether it passed is not confirmed.
- City- and state-owned power utilities fall under these laws (E). No binding state rule was found for private infrastructure (E). The CISA and FBI guidance (Jan 2024, E) is advice only.

**What Nox's parts mean (origin is E)**
| Nox part | FCC list | DoD s848 | Swap, cost |
|---|---|---|---|
| MicoAir743 AIO / Lite (China) | covered | fails | Scout: ARK FPV FC $194.99 + ARK 4IN1 ESC $218.50; Overseer: ARKV6X $400 [ARK] |
| ELRS XR2, RP3, Pocket (China) | covered | fails | federal variant only: Microhard carrier $320 to $949.50 [ARK], heavier (E) |
| MTF-02P (China) | covered | likely fails, has a camera (E) | ARK Flow $250, MR $350 [ARK] |
| DECXIN camera (China) | multi-use, likely outside [FCC FAQ] | fails | US or allied USB camera, $100 to $300 (E) |
| Radxa Zero 3W (China) | general-purpose, likely outside (E) | possibly a data link (counsel) | Raspberry Pi Zero 2 W or CM (UK-made, E), $15 to $60 (E) |
| Pi 5 (UK, E) | outside (E) | passes (E) | keep |
| Unitree L2 (1260H) | robot lidar, likely outside (E) | DoD indirect ban from 30 Jun 2027 | non-Chinese lidar, $4,000+, heavier (E) |
| Motors, ESCs, LiPos (China) | covered, but need no FCC grant (E) | not listed | swap only for the 65% test, US motors ~$100 each (E) |
| Box router, Xiaomi AX3000T | foreign routers covered | n/a | US-market FCC-authorized AP, $100 to $300 (E) |

## Steal for Nox
- Business: keep a one-page origin sheet per drone (part, maker, country, FCC ID, s848 class); buyers' security teams ask for it (E).
- Business: aim the product at Buy American (US build, over 65% US part cost) or a conditional approval with an onshoring plan, as Verity and Sees.ai did [DA 26-768].
- Scout v2: make the FC and ESC bay take a US board (ARK FPV plus ARK 4IN1), so a federal variant is a part swap (E).
- Overseer: keep a non-Chinese lidar path open. With the L2, DoD sales end on 30 Jun 2027 [OSD 805].
- Comms: record the FCC ID of every radio (XR2, RP3, Pocket, Radxa and Pi Wi-Fi, box AP) (E).
- Box: assemble it in the US from US or allied parts; the FCC counts ground stations as critical parts (E).
- Business: sell first to private and investor-owned sites. Public bodies in Florida and Tennessee need approved or federally listed drones [Fla. 934.50; Officer].

## For Spark
- `overseer.md` lidar and `overseer-scout-hardware-reference.md` §1.3: Unitree is a 1260H company [FR 1260H]. Keep the L2 for the demo, but don't pitch the Overseer to federal or DoD sites with it (E). Score origin in queue item 32 (E).
- `scout.md` As built: the FC, XR2 and DECXIN fail s848, and the MTF-02P likely does too. Change nothing for the demo (E).
- `overseer-scout-hardware-reference.md` §1.5: the Xiaomi AX3000T and Pocket/RP3 came from Taobao. A China-market unit may lack a US FCC grant; check before using it in a US facility (E).
- Pitch: showing an unauthorized prototype is allowed within limits, but selling or leasing it is not (E, 47 CFR 2.803). Ask counsel whether a US-assembled Nox with Chinese parts can be certified (E).
- Cost: the 25% Section 232 tariff on drone parts from 9 Feb 2027 may hit Chinese FCs and motors if they are in the annex [Wiley] (E).

## Sources
- [FCC PN DA 25-1086 (22 Dec 2025)](https://docs.fcc.gov/public/attachments/DA-25-1086A1.pdf)
- [FCC FAQ: fact sheet and FAQs, 7 Jan 2026](https://docs.fcc.gov/public/attachments/DOC-417528A1.pdf)
- [DA 26-768: conditional approvals, 24 Jul 2026](https://docs.fcc.gov/public/attachments/DA-26-768A1.docx)
- [DA 26-875: conditional approvals, 21 Aug 2026](https://docs.fcc.gov/public/attachments/DA-26-875A1.txt)
- [Akin Gump: foreign UAS added to Covered List](https://www.akingump.com/en/insights/alerts/fcc-adds-all-foreign-made-uas-and-uas-critical-components-to-covered-list)
- [Morgan Lewis: FCC exempts certain drones](https://www.morganlewis.com/pubs/2026/01/fcc-exempts-certain-drones-and-components-from-covered-list-to-address-national-security-risks)
- [Covington: conditional approvals, Mar 2026](https://www.globalpolicywatch.com/2026/03/fcc-updates-covered-list-to-conditionally-approve-the-use-of-certain-drones/)
- [Steptoe: drone Covered List update](https://www.steptoe.com/a/web/8XSVgfCDS1m4Zh2ftgPtuu/drone-covered-list-client-update419067661.pdf)
- [Greenberg Traurig: FCC bans foreign drones and parts](https://www.gtlaw.com/en/insights/2026/2/fcc-bans-import-of-foreign-drones-and-critical-components)
- [DroneXL: FCC exempts Blue UAS](https://dronexl.co/2026/01/08/fcc-exempts-blue-uas-drones-from-foreign-ban)
- [FR: FCC equipment authorization rules (2025-21001, 2026-16197, 2026-18535)](https://www.federalregister.gov/api/v1/documents.json?conditions%5Bterm%5D=%22modular%20transmitters%22%20%22covered%20list%22&conditions%5Bagencies%5D%5B%5D=federal-communications-commission&order=newest)
- [FAR 52.240-1](https://www.acquisition.gov/far/52.240-1)
- [ABC: foreign drone rules on federal projects](https://www.abc.org/News-Media/Newsline/new-rules-prohibiting-certain-foreign-manufactured-drones-on-federal-projects-now-effective)
- [Rochester: drones in federally funded research](https://www.rochester.edu/university-research/compliance/research-security/prohibitions-on-certain-drones-in-federally-funded-research/)
- [s848: FY2020 NDAA text (WIFCON)](https://www.wifcon.com/dodauth20/dod20_848.htm)
- [OSD 805: Section 805 waiver page](https://www.businessdefense.gov/805-waiver-request.html)
- [FR 1260H: DoD notice naming Unitree (search)](https://www.federalregister.gov/api/v1/documents.json?conditions%5Bterm%5D=Unitree&order=newest)
- [Tech Times: Pentagon flags Unitree](https://www.techtimes.com/articles/321011/20260720/unitree-humanoid-robot-enters-europe-days-after-pentagon-flags-it-chinese-military-tech.htm)
- [sUAS News: FY2026 NDAA and ground drones](https://www.suasnews.com/2025/12/beyond-dji-us-defense-bill-expands-ban-to-chinese-ground-drones-and-clones/)
- [Cornell: 10 USC 4872 notes](https://www.law.cornell.edu/uscode/text/10/4872)
- [DIU: Blue UAS to DCMA](https://www.diu.mil/latest/dius-blue-uas-list-to-transition-to-dcma)
- [IUS DCMA: Blue UAS portal](https://insideunmannedsystems.com/?p=41554)
- [IUS: FCC drone comment period](https://insideunmannedsystems.com/?p=42129)
- [Wiley: Section 232 drone tariffs](https://www.wiley.law/alert-Trump-Administration-Imposes-Section-232-Tariffs-on-Drones-and-Their-Parts)
- [Thompson Hine: Section 232 UAS tariffs](https://www.thompsonhinesmartrade.com/2026/08/president-trump-announces-section-232-tariffs-on-imports-of-unmanned-aircraft-systems-uas/)
- [Fla. 934.50](http://www.leg.state.fl.us/statutes/index.cfm?App_mode=Display_Statute&URL=0900-0999/0934/Sections/0934.50.html)
- [Tex. HB 41 committee analysis](https://capitol.texas.gov/tlodocs/89R/analysis/pdf/HB00041H.pdf)
- [Officer: police face foreign drone bans (2023)](https://www.officer.com/command-hq/technology/security-surveillance/uav-uas/article/53066482/compromised-air-space-police-face-foreign-made-drone-bans)
- [ARK: ARK Electronics shop](https://arkelectron.com/shop/)
