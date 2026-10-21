# Patent screen: battery receptacle art (latch, rails, contacts, push-through, hot swap)

Lay screen for Nox Scout v2 (S2 slider v2 + S2 cartridge v2), 13 Oct 2026. NOT legal advice. Status and expiry
dates are Google Patents estimates (E) unless a lapse was recorded; all risk levels are my estimates (E).
Scope: general battery-receptacle art. UAV swap stations and TECO US11787566B2 are covered by the other agent
(stations noted here only where a claim also reaches the drone side).

## 1. Bottom line

- No active patent found that claims our core combination (push-through tunnel + two spaced pin groups per
  pole + per-group diode). The closest patent on that exact combination, **IBM US9660224B2**, lapsed in 2021
  (unpaid fees), and the push-through + per-battery diode tunnel of **Vermont Medical US6936376B2** lapsed in
  2013. Both are free to use and are also strong prior art against anyone else's later claims.
- Real risks are narrow and avoidable by keeping a few rules (section 3):
  1. **Sony US12049333B2** (active to ~2041): UAV battery holder open at both ends, where a fresh external
     battery hits the old one and pushes it out the far opening. Reads closely on our swap. Medium (E).
  2. **Autel US11254429B2 / US10745128B2** (active to ~2036): UAV + battery in a shell + "clamp button" that
     latches it + a restorable elastic piece that is a **separate component** from the button. Our one-piece
     flexure pawls avoid it; a separate spring would not. Low now, High if a separate spring is added (E).
  3. **Lenovo US9851755B2** (active to ~2035-12, fees paid 2025): pass-through recess with **contact rails on the
     device walls** + lock at the floor. We use discrete pins on the device side, rails only on the pack.
     Low-medium (E).
  4. Box side only: **Skyyfish US11807130B2** (two multi-bay holders + piston shoving fresh in and old into a
     bay of the second holder) and **DJI US10611252B2** (station + onboard *backup battery*). Low if the old
     cartridge drops into a plain chute and no extra onboard battery is added (E).

## 2. Candidates checked (45)

### 2a. Closest ACTIVE patents, claims read

| # | Patent | Title / assignee | Status, expiry (E) | Independent claim elements (lay) | Our matching features | Risk (E) | Cheapest design-around |
|---|---|---|---|---|---|---|---|
| 1 | US12049333B2 | Unmanned aerial vehicle / Sony Corp + Sony Europe | Active, ~2041-09 | Cl.1: UAV with battery holding portion having first and second openings at opposite ends, holding the first battery "in a position away from a body of the UAV"; when the first battery directly impacts, at the first opening, a second battery external to the UAV, it gets an impulse from relative motion, is released out the second opening, and the second battery replaces it. Cl.17: same without the two openings, plus a tapered protrusion on one battery mating a tapered cavity on the other. | Tunnel open at both ends; fresh cartridge (external) pushes old one out the nose; fresh one becomes held. | **Medium** | (1) Keep the slider bolted flat on the frame plate, inside the airframe envelope; never on a column, arm or boom (the spec's "away from the body" embodiments use a column / bent arm). (2) No tapered nose-into-tail nesting between cartridges: flat end faces; put any lead-in taper on the slider mouth, not on the cartridge. (3) Fallback if needed: avoid battery-to-battery contact (box pulls old cartridge from the nose while pushing fresh in from the tail with a few mm gap; the 40 mm pin-group spacing still bridges power). |
| 2 | US11254429B2 | Battery used for UAV / Autel Robotics | Active, ~2036-11 | Multirotor UAV (body with compartment, arm, propeller); battery assembly = shell + battery body; clamp button that detachably attaches the battery to the main body; restorable elastic piece that resets the button and contacts it; **elastic piece and button are two separate components**. | Cartridge = shell holding the LiPo; pawls latch it to the body. | **Low** with monolithic flexures; **High** if a separate spring is added | Keep every pawl a single printed flexure (tooth, bump and spring = one part). No metal leaf, coil spring, O-ring or TPU pad behind a pawl. Avoid calling it a button / making it user-pressable. |
| 3 | US10745128B2 | Same family / Autel | Active, ~2036-10 | Same as #2 (clamp button + separate elastic piece resetting it). | Same | Same | Same |
| 4 | US10224526B2, US10442533B2, US10090496B2, US10115944B2, US10074836B2 | Same family / Autel | Active ~2036 (US10044013 and parent-like US9979000 lapsed) | All require the clamp button **mounted on the battery shell**, with a hook catching a clamping portion of the compartment, and an elastic piece on the shell. | None: our pawls are on the slider (body side), cartridge has only passive notches. | Low | Never put a sprung latch on the cartridge. |
| 5 | US9851755B2 | Hot swapping batteries in a mobile device / Lenovo | Active, ~2035-12 (4th and 8th yr fees paid) | Cl.1/18: device housing with recess between upper and lower wall over a planar surface, open on lateral sides; processor; locking mechanism located at the planar surface; first and second **contact rails** on the device's upper / lower walls (or adjacent floor). Cl.8 (battery): contact **pins** on top/bottom + flat contact rails on top and bottom; pins = charging, rails = discharging. | Pass-through recess, latch, processor (FC). | Low-Medium | Device side keeps **discrete spring pins**, no elongated rails on slider walls. Put pawls in the side walls (not the floor) where practical. Battery side: no charging *pins* on the strip faces; balance contacts stay as flat pads on the tail face. |
| 6 | US11807130B2 | Battery exchange and charging system for drones / Skyyfish (now Skyfish Corp) | Active, ~2042-08 | Station: first and second battery holders, each with **multiple movable docking bays** with a rail suspending the module; landing surface; bays and drone battery aligned on an axis; piston moves fresh module into drone and simultaneously pushes depleted module into the aligned bay of the **second holder**. | Push-through by a box pusher on one axis. | Low-Medium (box) | Old cartridge exits into a passive chute / bin, never into a bay of a movable multi-bay holder. Fine to feed fresh ones from a magazine. |
| 7 | US10611252B2 | UAV battery power backup / DJI | Active, ~2035-02 | Cl.1: station + landing area; UAV coupled to first battery and a backup source that is **another battery onboard**; replacement member decouples first battery while backup powers UAV. Cl.4: processors assess reliability of two backup sources and select one. | UAV kept powered during swap. | Low | No second onboard battery for hold-up; 1000 uF bus cap and the incoming fresh pack only. No "pick the more reliable backup source" logic. |
| 8 | US9434267B2 | UAV battery power backup / DJI | Active, ~2034-08 | UAV with first battery powering propulsion + loads and a **second onboard battery that powers only the non-propulsion loads**, switching between modes. | None. | Low | Same as #7. Fresh pack powers the whole bus incl. ESCs. |
| 9 | US12522387B2 | Aerial drone and battery pack / Brookhurst Garage | Active, ~2044-01 | Drone body; slide guide suspended from body surface forming a channel the pack slides in; **slide-guide contact sensor** on the surface detecting pack contact; connection port + **port contact sensor** detecting full insertion. | Slide channel. | Low | Do not add both a "pack touching surface" sensor and a "pack fully inserted" sensor. Seating can be read from bus voltage on pin group 2 or the box camera. |
| 10 | US12368207B2 | Battery pack / Panasonic Energy | Active, ~2042-10 | Holder with cells; circuit board with parallel slidable side edges in insertion guides; elastic extensions fixed **at both ends between two slits**, with protrusion/recess snap locking the board. | Our gold strips are PCBs on a printed cartridge. | Low (only if we snap-slide the strips in) | Fix strip PCBs with screws, adhesive or heat-stakes; no slide-in guide + slit-beam snap. |
| 11 | US11967730B2 | Power tool having a multi-latch battery interface / Black & Decker | Active, ~2041-09 | **Power tool** with receptacle, terminal block, guide rail, multi-latch with first interface element for pack 1 and second for pack 2 along the receiving axis. | Two staggered pawls along x. | Low | Not a power tool; our two pawls both act on one cartridge. Check continuation US12620667 claims before any power-tool-like product. |
| 12 | US11179841B2 | Battery pack interface / Milwaukee | Active, ~2039-04 | Latch mechanism in the device + a **switch operated with the latch** that inhibits power transfer while the latch is between positions. | Latch. | Low | Never gate power with a latch-position switch. |
| 13 | US11296384B2 | Battery mounting apparatus / DJI | Active, ~2037-01 | Mounting plate, locking structure, **ejection structure** with sliding-slot member, fastening member and elastic part fastened to the plate. | None (no ejector). | Low | No sprung ejector on the slider. |
| 14 | US11088414B2 | Battery housing apparatus / DJI | Active (E) | Locking assembly displaced by the battery on insertion and restored to lock + **indicator / key assembly** showing seated state; cl.15 locking slider + spring + bracket with guide slot. | Displaced-and-restored lock. | Low | No mechanical seated indicator coupled to the pawl; no separate slider + spring. |
| 15 | US11411257B2 | Hot swap battery module / ETI CA Battery | Active, ~2041-03 | **Battery module** with charge + discharge switches, series power FET, smart chip turning FET off when cell < bus, and ideal-diode controller on the same FET. | Ideal diodes. | Low | Keep ideal diodes on the drone side, per pin group; pack stays a plain LiPo (no BMS + ideal diode in the cartridge). |
| 16 | US11316225B2 | Battery container having a floating connector / Kwang Yang | Active, ~2040-10 | Charging-station container with rear wall hole, **floating connector** + coupling board sliding on the rear wall when the battery is inserted. | Charging cradle with tail-face balance pads. | Low | In the cradle use fixed spring pins; no floating connector plate. |
| 17 | US12252215B2 / US12097921B2 | E-bike energy store holding device / Bosch (E) | Active | Locking unit with a second locking element blocking the first + prestress; or tensioning unit clamping pack against a stop. | Latch. | Low | Single flexure pawl, no blocking second element, no clamping tensioner. |
| 18 | US12300838B2 | Patient support apparatus battery retention / Stryker (E) | Active | Patient support apparatus; pivoting receiver; latch pawl retracted by pivoting. | Pawl in notch. | Low | Different field; nothing pivots. |
| 19 | US8993153B2 | Ramped battery contact / Symbol (Zebra) | Active, ~2032-07 | Battery with ramped contact pad usable both by pogo pins (perpendicular) and leaf springs (parallel swipe). | Pins wipe pads. | Low | Flat strips, no ramp-plane pad serving two contact modes. |
| 20 | US11699870B2 | Spring contact on battery / Hilti | Active, ~2038-04 | Power tool; damping element on contact element counteracting relative motion. | None. | Low | No damper on contacts. |

### 2b. Other active art checked, low or no relevance
US9502702B2 (bicycle battery holder with rail-moved support, Shimano (E)); US9967377B2 (phone with permanent
backup battery in parallel); US9424697B2 (Gogoro, compartment unlock by authentication / charge state);
US12049274B2 (Gogoro-style cable lock); US10850838B2 (Sony, electromagnet drop-and-grab battery swap);
US9705116B2 (B&D, method of making pack with guide-aligned PCB); US12341212B2 (power-tool handle with two
battery latches); EP4556380A1 (Rogoz, drone frame with manual pusher; no US counterpart found);
Gogoro / Honda Mobile Power Pack / NIU vehicle-compartment patents (all claim scooter compartments,
authentication, solenoid locks; no fit to a passive drone slider).

### 2c. EXPIRED / lapsed / abandoned art checked
US9660224B2, US6936376B2, US7339349B2, US6240631B1, US6722024B2, US20070152630A1 (abandoned), US5028806A,
US9979000B2, US10044013B2, US7005831B2, US7160132B2, US10230080B2, US7121854B2, US6909940B2, US6551123B2,
US4993973A, US5764477A. Details of the useful ones in section 4.

## 3. Design-around rules (keep these and the screen stays clean)

1. **Pawls are one-piece flexures.** Tooth, release bump and spring are the same printed part. No separate
   spring, metal leaf, O-ring or elastomer behind a pawl, and no user button. (Autel family.)
2. **All latches live on the slider; the cartridge stays passive** (notches, flanges, flat faces only). (Autel
   US10224526 family, DJI US11296384.)
3. **Slider sits flat on the frame plate, within the airframe envelope.** No battery bay on a column, arm,
   pylon or boom. (Sony US12049333.)
4. **Flat cartridge end faces.** No tapered protrusion on one cartridge nesting a tapered cavity on the next.
   Lead-in chamfers go on the slider mouth and guide grooves. (Sony cl.17.)
5. **Device side = discrete spring pins; elongated strips only on the cartridge.** (Lenovo US9851755.) No
   charging pins on the strip faces; balance contacts remain flat pads on the tail face.
6. **No second onboard battery** for hold-up and no backup-source selection logic; bus capacitor only.
   (DJI US9434267 / US10611252.)
7. **Ideal diodes on the drone, one per pin group; pack has no BMS/FET.** (ETI US11411257.)
8. **No latch-position switch gating power; no mechanical seated-indicator; at most one seating sensor.**
   (Milwaukee US11179841, DJI US11088414, Brookhurst US12522387.)
9. **Box: old cartridge drops into a passive chute, not into a movable multi-bay holder.** (Skyyfish
   US11807130.) Cradle uses fixed pins, no floating connector plate. (Kwang Yang US11316225.)
10. **Strip PCBs are screwed / glued / heat-staked** to the cartridge, not slid in and snapped by a slit beam.
    (Panasonic US12368207.)

## 4. "Steal" list: expired / lapsed / abandoned designs, free to copy (E)

| Patent | Status | What to take |
|---|---|---|
| **US9660224B2** (IBM, prio 2014) | Lapsed 2021-06 (fees) | Our contact scheme exactly: compartment with entrance and exit, backup battery slides in and pushes the current one out; per pole at least two contacts spaced along the slide so they touch either one belt or one belt of each battery. Copy freely; cite as our prior-art anchor. |
| **US6936376B2** (Vermont Medical, prio 2002) | Lapsed 2013 (fees) | Tunnel open both ends, new battery pushes old one out the far side, **a diode in each battery path** so the fresh pack never back-feeds the old one while the load stays powered. Also a lever variant for same-side swap. |
| **US7339349B2** (Dallab, prio 2005) | Lapsed 2016 (fees) | **Latch released by the incoming battery**: pivot lever with a square pin and an angled pin; old battery stops halfway on pin 2, new battery's nose pushes the angled pin which frees the old one and then locks the new one. One-way motion only. Four staggered terminal contacts so two always touch a strip. Direct precedent for our T-pawl bump. |
| **US6240631B1 / US6722024B2** (GSL, prio 1999) | Lapsed 2009 / 2008 | Push-pop coin cell: new cell enters one opening and pushes the old out the other; new cell's terminals touch before old one leaves (make-before-break); door projection finishes the push. |
| **US20070152630A1** (Motorola, 2005) | Abandoned, published | Two connector sets spaced in the holder, **elongated / C-clip battery contacts** spanning both, back-to-back P-FET pairs per connector plus a hold-up capacitor between them; a control pin shorted at full seat moves power to connector 2. Good circuit reference for our LM74700 pair + cap. |
| **US5028806A** (Dell, prio 1989) | Expired | A **shorter "early-break" terminal** that loses contact first and warns the circuit before the power contacts break. Cheap idea: one shortened pad on the strip as a "swap in progress" signal to the FC. |
| **US7005831B2** (B&D, prio 1998) | Expired (fees) | **Two-stage rails**: loose fit for the first ~60 mm, tight rail-and-groove for the last ~22 mm to align the terminals; length / rib **lockouts** for keying. Apply: loosen C-groove clearance at the tail entry, tighten over the seated zone. |
| **US7160132B2** (B&D, prio 2004) | Expired (fees) | Engagement order: keys first, then power, then data; floating terminal blocks; tapered alignment projection; open-sided female terminals so debris falls out. |
| **US10230080B2** (B&D, prio 2001) | Expired (fees) | Rail profile with upper and lower outward projections and an open gap facing sideways (DeWalt-style rail), latch + button on pack. |
| **US7121854B2** (TTI/Ryobi, prio 2003) | Expired | Latch that is driven down by wedges before an ejection arm pushes the pack off; dual-latch both-buttons release. Useful if a manual release is ever needed for the charger cradle. |
| **US9979000B2** (Autel, prio 2015) | Lapsed 2026-05 | S-shaped mirror-symmetric spring under a hooked clamp; hook groove so the body does not snag. **Caution:** sibling patents (rows 2-4 above) are still active, so take only the S-spring geometry idea into a one-piece flexure. |
| **US6551123B2** (prio 1996) | Expired (E) | Coarse guide then fine guide right at the contacts, chamfered lead-in, U-shaped projection with stop face; a **key in a slot for DC polarity** (same purpose as our asymmetric flanges). |
| **UC Berkeley "Midair Hotswap"** | Academic | Already our concept source (push-through, pogo pins on copper plates). |

## 5. Notes and limits

- Status for most active patents comes from Google Patents "Active" plus adjusted expiry; only Lenovo's fee
  record was checked in detail. Lapses listed above were recorded "Expired - Fee Related" events.
- Pending applications were not searched systematically (e.g. Brookhurst US20260138761A1 and any Sony
  continuation could still issue). Re-check Sony and Brookhurst families before production.
- Using commercial ideal-diode ICs (LM74700) and Mill-Max pins carries the chip / part makers' own patent
  rights with the purchase (exhaustion); the risk is in the system claims, which the rules above address.
- Before any sale, a patent attorney should give a freedom-to-operate opinion, starting with Sony US12049333.

## Sources
- https://patents.google.com/patent/US9660224B2/en
- https://patents.google.com/patent/US6936376B2/en
- https://patents.google.com/patent/US7339349B2/en
- https://patents.google.com/patent/US6240631B1/en
- https://patents.google.com/patent/US6722024B2/en
- https://patents.google.com/patent/US20070152630A1/en
- https://patents.google.com/patent/US5028806A/en
- https://patents.google.com/patent/US9851755B2/en
- https://patents.google.com/patent/US12049333B2/en
- https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12049333
- https://patents.google.com/patent/US9979000B2/en
- https://patents.google.com/patent/US10224526B2/en
- https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10442533
- https://patents.google.com/patent/US10745128B2/en
- https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10745128
- https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11254429
- https://patents.google.com/patent/US11254429B2/en
- https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10115944
- https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10090496
- https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10074836
- https://patents.google.com/patent/US11807130B2/en
- https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9434267
- https://patents.google.com/patent/US9434267B2/en
- https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10611252
- https://patents.google.com/patent/US10611252B2/en
- https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12522387
- https://patents.google.com/patent/US12522387B2/en
- https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12368207
- https://patents.google.com/patent/US12368207B2/en
- https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11967730
- https://patents.google.com/patent/US11967730B2/en
- https://patents.justia.com/patent/11179841
- https://patents.google.com/patent/US11296384B2/en
- https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11088414
- https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11411257
- https://patents.google.com/patent/US11411257B2/en
- https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11316225
- https://patents.google.com/patent/US11316225B2/en
- https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12252215
- https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12097921
- https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12300838
- https://patents.google.com/patent/US8993153B2/en
- https://patents.google.com/patent/US11699870B2/en
- https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9502702
- https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9967377
- https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9424697
- https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12049274
- https://patents.google.com/patent/US10850838B2/en
- https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10850838
- https://patents.justia.com/patent/9705116
- https://patentsgazette.uspto.gov/week25/OG/html/1535-4/US12341212-20250624.html
- https://data.epo.org/publication-server/rest/v1.2/patents/EP4556380NWA1/document.html
- https://patents.google.com/patent/US7005831B2/en
- https://patents.google.com/patent/US7160132B2/en
- https://patents.google.com/patent/US10230080B2/en
- https://patents.google.com/patent/US7121854B2/en
- https://patents.google.com/patent/US6909940B2/en
- https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/6551123
- https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/7619387
- https://patents.google.com/patent/US4993973A/en
- https://apps.dtic.mil/sti/pdfs/ADD020218.pdf
