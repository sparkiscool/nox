# DJI's teased battery-swap dock (11 Oct 2026)

**Bottom line**
1. Confirmed: an unnamed DJI Enterprise launch on 14 Oct 2026, 8 AM EDT [DJI trailer]. DroneXL reads the 6 s teaser render as a rail-fed carousel of 5+ packs lifting one into the drone's empty rear bay [DroneXL teaser]. Names (Dock 4, Matrice 5TD), masses and prices are leaks or guesses. Swap time, size and dock mass are not public.
2. The layout resembles DJI's 2014 patents: a rotating store, a lift, and a swap-or-charge choice by pack charge [US9139310B1, active to ~2034]. A limited search found no DJI swap-dock patent filed 2020 to 2026.
3. Others already sell swap docks [Dronehub]; with DJI in, swapping becomes the norm. DJI still lands on an outdoor pad, serves one aircraft and faces a US authorization block [Dronehub]. Pitch the indoor hover catch, two drone types, the no-reboot swap and a timed turnaround (E).

## Findings

**Teaser.** DJI's page shows only the date, time and the slogan "Always There, Always Ready" [DJI trailer]. DroneXL describes the card as a dock outline with four green battery LEDs [DroneXL rumors]. Its reading of the wireframe clip: rails feed a carousel with a central gear hub and at least five bays, which turns a pack under the drone and lifts it into the empty top-rear bay of a Matrice 4D-style folding drone [DroneXL teaser]. I did not view the clip; PickDrones repeats DroneXL [PickDrones].

**Likely mechanism (E).** The Matrice 3D dock drone's pack locks with a locking arm worked by hand: open it, push the pack to the click, close it [Heliguy M3D]. A dock swap needs a latch a machine can work, so expect a new bay on the 5TD. Carousel index plus one lift is a two-actuator pack path; the spent pack likely leaves the same way. The final insertion direction is unclear. Dock 2 centers on slide ramps [dock-swap report]; a swap needs tighter placement, perhaps pushers as in US11708003B2 below. Bay charging isn't shown, but a swap dock needs it.

**Leaks and filings.**
| Item | Value | Status [source] |
|---|---|---|
| Aircraft | Matrice 5TD (M5TD), light class, power-line and pipeline patrol | China CAAC registry screenshot, 21 Sep [DroneXL 5TD] |
| Mass | 2.6 kg empty, 3.05 kg max takeoff (4TD: 1.85 / 2.09 kg) | Same [DroneXL 5TD] |
| Dock | Named Dock 4, coming to stores soon | Leaker post, no source [NC stores] |
| Photos | Prototype with top dome (lidar?) and five-window gimbal, 26 Jul; 5TD on a dock, 2 or 10 Oct (page dates conflict) | Leaks [DroneXL M5; NC dock] |
| Price | Dock 4 $16,500 to 20,500; with drone $27,000 to 33,000 | Blogger estimate from Dock 3 prices [NC price] |
| Indoor flight without GNSS | Claimed | Unsourced [NC price] |
| Swap time, size, dock mass, IP, ship date | Not public | |

No FCC filing found (FCC ID search not run). New foreign-made drone and dock models have needed an FCC exemption since 22 Dec 2025 [Dronehub]. DJI told the Ninth Circuit this blocks 25 planned 2026 launches [DroneXL teaser]. A July 2026 proposal would also end sales of covered DJI docks to US non-government buyers; pending on 28 Sep [Dronehub].

**Dock 3 baseline (charge only).** 640 x 745 x 770 mm closed, 55 kg, 800 W max, IP56, 15 to 95% in 27 min [Dock 3 specs]; $15,890 alone [NC price].

**DJI swap patents.**
- US9139310B1 (2014, to ~2034): rods on the drone settle into conical recesses. An elevator arm with a rack-and-pinion C-clamp pulls the pack into an empty slot; the store rotates (8 packs in the example) and the arm inserts a full one. Slots charge by contact or induction. It swaps below a charge threshold, and charges in place if no full pack is ready. Its summary also covers slots that move together or rotate, always beside a landing area [US9139310B1]. Claims of it and its 4 continuations not read.
- US11708003B2 (2014, granted 2023, to ~2036): DJI argues a rough landing plus pushers is faster than a precise landing. Two pushers on perpendicular rails shove the drone to an exchange zone and hold it. A hatch in the landing plate opens; a 3-axis arm's carriage holds a full and a spent pack, so one visit trades both. Claim 1 needs that plate, hatch and door [US11708003B2].
- US10259332B2 (expired) picks a grid store as smaller than a carousel [US10259332B2]. A 2017 DJI filing is a lidded charging station [youuav].
- Others: Youfei charges each of 6 slots from below 30% to 65%, or 100% if due soon, and cuts a slot's charging before the arm takes its pack [CN115743661B]. State Grid's pending arm goes compliant past a force limit and stops on the latch-click force [CN122501567A].

**Gaps.** Clip not viewed. CNIPA not searched, the Google Patents search page was blocked, and Justia's DJI list (page 1) showed no dock patent [Justia DJI]. Re-check on 14 Oct for swap time, size, mass and price.

**Box vs DJI.**
| | DJI swap dock (expected) | Nox Box | Nox move (E) |
|---|---|---|---|
| Site | Outdoor, IP56 class [Dock 3 specs]; indoor claim unsourced [NC price] | Indoor | Avoid lid, sealing, 55 kg |
| Arrival | Lands on a pad | Caught from a hover, ±150 [nox-dimensions §4] | Beat: show it |
| Aircraft | One | Scout and Overseer | Beat |
| Pack store | 5+ bay carousel [DroneXL teaser] | 7 slots, two sizes [box.md] | Match for the Overseer |
| Swap time | Not public; market 50 s to 3 min [catcher-speed research] | Target under 10 s; v1 as drawn 63 to 75 s [catcher-speed research] | Beat, once measured |
| US sale | Blocked without exemption [Dronehub] | Open | Beat while rules hold |
| Price | ~$30,000 with drone, estimate [NC price] | Not set; parts under ~$2,000 (E) | DJI sets the anchor |

## Steal for Nox
- Swapper (Overseer): a carousel indexing each bay under a fixed Z pusher at the catch point, as in the teaser. Four 180 x 78 bays [nox-dimensions §4] fit ~0.4 to 0.5 m across; staying within 270° of turn lets bay wires wrap, no slip ring (E).
- Charger: cut a bay's current before its pack is pulled; hold idle spares near 65% and fill only the next one due [CN115743661B].
- Charger: a 4-LED charge bar per bay, like the teaser's lit pack [DroneXL teaser], so viewers see a full pack go in (E).
- Firmware (box script, X8): before a catch, confirm a full pack is ready, as DJI chooses swap or charge [US9139310B1]; if none, hold off and report (E).
- Box catcher: rough arrival then pushers is DJI's own argument [US11708003B2]; an open head with no plate or hatch stays outside its claim 1 (E, lay reading).
- Business: quote launch and swap times; DJI's dock first-responder page promises launch under 15 s [DroneXL teaser].

## For Spark
- `box.md` Brief, "catch a hovering drone, secure it, swap its cartridge by script, repeatably": after 14 Oct a swap alone won't impress. Demo the catch, both drones and the no-reboot swap, timed trigger to liftoff (E).
- `box.md` D4 "slide x 0-410, Z pusher ~230 stroke" and D5 "Sled: one piece, two tiers, 300 travel in y": `twin-arm-swap-concept.md` moves the Scout slots onto posts, so one rotary axis could replace the Overseer slide and sled (E).
- IP: US9139310B1 describes rotating or moving pack stores, and the patent report lists the family as "Swapper: unknown". The `box.md` D5 sled and a carousel both resemble it. Its aspects all include a landing area, while the iris holds the drone with no pad. Ask counsel, and read the continuations' claims first (E).
- `box.md` "Slots: 7 (Overseer 3: two full, one empty...)": with ~11 min flights (`overseer-compatibility-audit.md`), each pack must refill 20 to 100% in ~22 min, about 2.2C. A fourth Overseer pack and slot drops that to ~1.5C (E). DJI's 5+ bays for one aircraft [DroneXL teaser] lean the same way. Touches "7 cartridge slots in two sizes": your call.

## Sources
- [DJI trailer: DJI Enterprise launch page](https://enterprise.dji.com/trailer)
- [DroneXL teaser: DJI teases Dock 4 with battery swapping for October 14, not for the US](https://dronexl.co/2026/10/08/dji-enterprise-teaser-battery-swap-dock-october-14/)
- [DroneXL rumors: DJI rumors, October 2026](https://dronexl.co/drone-companies/dji/dji-rumors/)
- [DroneXL 5TD: Matrice 5TD surfaces in China filing, 750 g heavier than 4TD](https://dronexl.co/2026/09/27/dji-matrice-5td-china-registration-leak-750g-heavier-matrice-4td/)
- [DroneXL M5: Matrice 5 leak shows integrated lidar and five-window gimbal](https://dronexl.co/2026/07/27/dji-matrice-5-leak-integrated-lidar-five-window-gimbal/)
- [PickDrones: DJI Dock 4, battery swap, launch 14/10 (PT)](https://pickdrones.com/pt/dji-dock-4-troca-bateria-14-outubro-2026/)
- [NC stores: New Camera, Dock 4 confirmed for stores, first labelled 5TD photo](https://thenewcamera.com/?p=68615)
- [NC dock: New Camera, new clear look at Matrice 5TD on dock](https://thenewcamera.com/new-clear-look-at-dji-matrice-5td-on-dock-days-before-oct-14-launch/)
- [NC price: New Camera, Matrice 5TD and Dock 4, arrival and price](https://thenewcamera.com/dji-matrice-5td-and-dock-4-october-arrival-price-and-more/)
- [Dock 3 specs: DJI Dock 3 specifications](https://enterprise.dji.com/dock-3/specs)
- [Heliguy M3D: Installing and removing Matrice 3D batteries](https://www.heliguy.com/blogs/knowledge-base/how-do-i-install-and-remove-the-dji-matrice-3d-series-intelligent-flight-batteries/)
- [Dronehub: Drone-in-a-box systems compared 2026](https://dronehub.ai/blog/drone-in-a-box-systems-compared-2026)
- [US9139310B1: DJI, Systems and methods for UAV battery exchange](https://patents.google.com/patent/US9139310B1/en)
- [US11708003B2: DJI, UAV base station system and method](https://patents.google.com/patent/US11708003)
- [US10259332B2: DJI, UAV dock and battery replacement device](https://patents.google.com/patent/US10259332)
- [youuav: DJI base station and charging system patent (CN)](https://www.youuav.com/news/detail/202108/50481.html)
- [Justia DJI: Patents assigned to SZ DJI Technology](https://patents.justia.com/assignee/sz-dji-technology-co-ltd)
- [CN115743661B: Guangzhou Youfei, drone battery-swap dock](https://patents.google.com/patent/CN115743661B/zh)
- [CN122501567A: State Grid Intelligence, dock battery replacement control (PatSnap)](https://eureka.patsnap.com/patent/CN122501567A)
