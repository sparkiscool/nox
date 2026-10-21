# Nox Scout v2 battery slider: lay US patent screen (13 Oct 2026)

Not legal advice. A lay screen by a research agent, all risk ratings are estimates (E). Legal status and expiry dates come from Google Patents, which marks them as "an assumption, not a legal conclusion". Before selling, pay a US patent attorney for a freedom-to-operate opinion on the two HIGH/MEDIUM items below.

Design screened: /home/claude/research/CONTEXT.md (S2 slider v2 + S2 cartridge v2). Feature tags used below:

| Tag | Feature (current design) |
|---|---|
| F1 | Slider on top of the frame plate: short C-groove segments, tunnel open at both ends |
| F2 | Push-through swap: box pusher drives fresh cartridge in from tail, fresh nose shoves old cartridge out the nose, one 0.35 s stroke |
| F3 | T pawl (tail lock) flexure on slider, tooth in notch near cartridge tail, released when the fresh cartridge nose lifts a bump on the same leaf |
| F4 | N pawl (nose ratchet) sawtooth on slider, blocks -x, lets +x pass; T and N staggered along x |
| F5 | 16 horizontal Mill-Max spring pins in slider walls (B+ left, B- right), two groups 40 mm apart; gold strips on cartridge sides |
| F6 | Each pin group through its own ideal diode (LM74700 + FET) to the bus; old and fresh packs both connected during the stroke, never paralleled; 1000 uF bus cap |
| F7 | Asymmetric flanges (3.9 / 2.9 mm) as polarity key |
| F8 | Balance pads on cartridge tail face (cradle charging) |
| F9 | Optional printed ID tag read by box camera; no ID contact |
| B1-B4 | Box: seat (landing area), pusher, fresh-pack magazine, chute for old packs |

---

## 1. TECO US11787566B2 (the known concern)

- **Title:** Drone and battery exchange system thereof
- **Assignee:** Teco Image Systems Co., Ltd. (Taiwan). Inventor Ting-Chun Chen.
- **Priority:** 2020-12-18 (TW 109145032). US filed 2020-12-28 (17/135,095). Granted 2023-10-17.
- **Status:** Active. Adjusted expiration 2041-11-13.
- **Family:** US20220194630A1 (pub); TWI779431B / TW202225038A; JP7254135B2 / JP2022097373A. No CN or EP members listed. No US continuation listed.

### Claims (6 total, only claim 1 independent; all claims are drone claims, the station is described but NOT claimed)

**Claim 1 (paraphrase keeping every element; quoted fragments are exact):**
1. "A drone, comprising:"
2. a frame "having a battery accommodation groove",
3. the groove "comprises a first positioning component and a second positioning component",
4. the two positioning components are "staggered along a first direction";
5. a battery "received within the battery accommodation groove",
6. the battery has "a first positioning portion and a second positioning portion",
7. the first positioning portion "is matched with the first positioning component" and the second "is matched with the second positioning component";
8. the battery "has an electrode region",
9. the electrode region "has at least one positive electrode, at least one negative electrode, and at least one identification component",
10. the positive electrode, negative electrode and identification component are "overlapped with each other or separately disposed with each other".

**Dependent claims:** 2 positioning portions point- or line-symmetric about the battery center; 3 guide rail in the groove along the first direction; 4 each positioning component = spring + conical frustum, each positioning portion = recess, frustum seats in recess; 5 metal plate springs in the groove abut the +, - and ID contacts; 6 battery insertable along the first direction or its reverse.

### What the terms mean in the spec
- **Positioning component (bay side):** a spring-loaded conical frustum (small end toward the battery). Spec says the portion is "coupled, engaged, or latched" with the component; "matched" means "disposed corresponding to". Read broadly, any bay-side detent/pawl can be a "positioning component".
- **Positioning portion (battery side):** "Each of the first positioning portion 31 and the second positioning portion 32 is a recess."
- **Staggered:** "the first positioning portion 31 and the second positioning portion 32 are not overlapped on the first direction d1" (i.e. at different stations along the insertion axis).
- **Identification component:** "an identification resistor or an identification chip, but not limited herein." It sits in the electrode region and is contacted by its own plate spring (claim 5).
- **Exchange in the spec:** station transport rail 91 mates the drone's guide rail 82 and pushes a stored battery along the first direction into the groove, pushing the frustums aside. Removal of the old battery is not clearly described.

### Element-by-element vs our design
| Claim 1 element | Our design | Present? |
|---|---|---|
| Drone frame with battery accommodation groove | Slider tunnel on frame plate (F1) | Yes (likely) |
| First + second positioning components in the groove | T pawl and N ratchet on the slider (F3, F4) | Yes (likely, "latched" is in the spec) |
| Staggered along a first direction | T and N staggered along x | Yes |
| Battery with first + second positioning portions matched to them | T notch near tail, N sawtooth on cartridge | Yes |
| Electrode region with + and - | Side strips (F5) | Yes |
| At least one identification component in the electrode region | None. No ID resistor/chip, no ID contact (F9 is a camera-read print) | **No** |

**Verdict: LOW risk (E) as long as there is no identification component.** The ID component is the only missing element, and it alone defeats literal infringement of the only independent claim.

### Design changes that each remove an element of claim 1 (claim 1 is the only independent claim)
1. **Keep: no identification component anywhere on the pack** (no ID resistor, no ID chip, no EEPROM, no thermistor reachable through any contact, including the balance pads). A thermistor or "ID resistor" between a balance pad and B- would be the easiest way to walk into the claim. If pack ID is ever needed, keep it optical (printed tag), and put the tag on a face that carries no electrodes (top face, not on the strips or the tail pad area).
2. Optional belt-and-braces: **put T and N at the same x station** (not staggered). Today they are staggered, so this would cost a redesign of the pawl leaves.
3. Optional: **move retention onto the pack** (spring elements on the cartridge, fixed notches in the slider). This flips which part carries the "components" vs "portions", but it walks toward the Autel "clamp button + elastic piece on the battery shell" claims (US10224526 family, section 2.13), so it is NOT recommended.

Recommendation: rule 1 only. Do not add an ID contact or thermistor.

---

## 2. Other active US patents screened

Candidates checked (35+): US12049333, US10611252, US9434267, US11332033, US10363826, US9139310, US9346560, US9440545, US11807130, US12286252, US12522387, US11387515, US11749860, US9851755, US11411257, US11211806, US10850838, US9551989, US11708003, US10259332, US12552284, US11993373, US10903474, US11296384, US11767135, US11111033, US10040578, US20170283090A1, US10017265, US11424491, US9227601, US4983903, US7339349, US10224526 / US10090496 family, Workhorse "Flying vehicle systems and methods" family (US11332264, US11498701, US11383859, US12227306, US12037137, US12448156, US11820533, US11603219, US11472572, US11370561, US11407527, US12365494, US11180263, US11254446, US11485518), KR102677782B1, EP3876335A1 / US20210249870A1, CA3090641.

### Risk summary (closest first)

| # | Patent | Owner | Expiry (E) | Reads on | Risk (E) | Cheapest design-around |
|---|---|---|---|---|---|---|
| 1 | **US12049333B2** | Sony | 2041-09-19 | F1+F2+F3 (push-through, new battery ejects old) | **HIGH** | Old pack must not be pushed out by the fresh pack (box ejector contacts the old pack), and no tapered nose/tail mating |
| 2 | **US10611252B2** | DJI | 2035-02-28 | F6 + B1/B2 (station keeps UAV powered from another onboard battery during decoupling) | **MEDIUM** | Bridge the gap with box power (not an onboard battery), break-before-make between packs |
| 3 | US9346560B1 | DJI | 2034-08-08 | Box only, if the seat carries an orientation fiducial | MEDIUM (box) | Marker on the drone, camera in the box; or no fiducial |
| 4 | US10363826B2 | DJI | ~2034-08 | Box only, if magazine indexes during removal | LOW-MED (box) | Magazine stationary during the eject stroke |
| 5 | US11807130B2 | Skyyfish | 2042-08-02 | B2-B4 push-through station | LOW-MED (box) | Old pack exits into a fixed chute/bin, not a bay of a movable holder; packs not hung from rails |
| 6 | US11332033B2 | DJI | 2035-11-03 | Box only, if carousel | LOW-MED (box) | Linear magazine, no rotating holding station |
| 7 | US9139310B1 | DJI | 2034-08-08 | Box logic | LOW | Do not decide "swap vs charge in place" before landing from wireless SoC |
| 8 | US9551989B2 | Boeing | 2034-05-12 | Box sensing | LOW | No pad/surface sensor used to locate the UAV before swap |
| 9 | US12522387B2 (+ pending cont. US20260138761A1) | Brookhurst Garage | 2044-01-29 | F1 + presence sensing | LOW (watch) | No pack-on-surface sensor and no port-seated sensor |
| 10 | US12286252B2 | Brookhurst Garage | 2043-07 | Box | LOW | No two-flap gripper + carrier-to-charger |
| 11 | US11387515B2 | Amtran | 2041-03-30 | F3 (hook + bump pawls) | LOW | Keep contacts off the pawls; seated pack must not close hooks via bumps |
| 12 | US9434267B2 | DJI | 2034-08-08 | F6 | LOW | Fresh pack powers the whole bus incl. ESCs; no "avionics only" mode switch |
| 13 | US9851755B2 | Lenovo | 2035-12-15 | F5 | LOW | Rails stay on the pack, pins on the drone; no charging pins on strip faces |
| 14 | US11411257B2 | ETI CA Battery | 2041-03-04 | F6 | LOW | Ideal diodes stay on the drone, no switches/smart chip in the pack |
| 15 | US11749860B2 | Coretronic | 2041-11-13 | Box-released latch (alt. architecture) | LOW | Avoid carrier-latch + catching-hook drag station |
| 16 | US10224526B2 family | Autel | 2036-10 | Latch on pack (alt.) | LOW | Keep latches on the slider, not as spring buttons on the pack |
| 17 | US11211806B2 | DJI Osmo | 2037-08-09 | F6 firmware | LOW | No "new pack capacity / counterfeit check -> shut down" logic |
| 18 | Workhorse / Aero Velocity nest family | Aero Velocity | 2041+ | Box keep-alive via leg contacts | LOW (watch) | Keep-alive/charge contacts not on a tapered nest sidewall the legs slide into |
| 19 | US11111033B1 | Modular Flight | 2038-05-11 | Box keep-alive pad | LOW | Fixed polarized contacts, no segmented pad + switch matrix |

Expired / abandoned and free to copy (ideas): **US7339349B2 (Dallab, push-through battery swap with continuous power, lapsed 2016 and past its 2026-02-23 term)**, US4983903 (Samsung AGV), US10017265 (Amazon, fee lapse), US10040578 (Skycatch, fee lapse), US10259332 (DJI, fee lapse), US9979000 and US10044013 (Autel family, fee lapse), US20170283090A1 (Skycatch, abandoned), US20180312069A1 (Miasole, abandoned), US20210249870A1 (DJI, abandoned).

Non-US only (no US risk): KR102677782B1 (Daejin Univ., rail + latch drone battery with conductive rails keeping power during swap; no US family listed).

---

### 2.1 US12049333B2 "Unmanned aerial vehicle" (Sony Corp / Sony Europa BV): HIGH

- Inventor Nicolas Pierre Rose. Priority 2018-03-28, filed 2019-03-21 (16/981,752), granted 2024-07-30. Active, adjusted expiry 2041-09-19. Pub US20210031915A1. No EP/CN/WO family or continuations listed on Google.
- Spec is close to our design: retractable lugs "pushed back" by the incoming battery and re-seated by a spring; electrodes running the full bay length so contact "is maintained before, during and after the battery exchange operation".

**Independent claim 1 (exact fragments in order):**
1. "An unmanned aerial vehicle (UAV), comprising: a battery holding portion having first and second openings at opposite ends"
2. "and configured to releasably hold a first battery to provide electrical power to the UAV,"
3. "the battery holding portion being configured to hold the first battery in a position away from a body of the UAV"
4. "such that, upon the first battery directly impacting, at the first opening, a second battery external to the UAV,"
5. "the first battery receives a mechanical impulse from the second battery due to relative motion between the UAV and the second battery,"
6. "causing the first battery to be released from the battery holding portion via the second opening,"
7. "whereby the second battery replaces the first battery to become held by the battery holding portion to provide electrical power to the UAV."

**Independent claim 17:** same, minus the two openings and minus "directly", but adds "wherein one of the first and second batteries comprises a tapered protrusion and the other ... comprises a tapered cavity of corresponding shape".
**Claim 10:** battery replacement apparatus (box) holding the second battery so that on impact it imparts the impulse releasing the first battery (references the UAV of claim 1, so it inherits claim 1's elements). Claim 15: a battery with tapered protrusion at one end and tapered cavity at the other. Claim 16: system = claim 1 + claim 10.
**Dependents:** 2 tapered engagement; 3 friction fit; 4 retractable projections; 5 magnetic fit; 6 horizontal travel; 7 vertical travel; 8 vertical ejection; 9 wireless battery characteristics/selection; 11-14 apparatus details (charger, transporter, selector).

| Claim 1 element | Our design | Match |
|---|---|---|
| Holding portion with openings at opposite ends | Tunnel open at tail and nose (F1) | Yes |
| Releasably holds first battery for power | Yes | Yes |
| Holds battery "in a position away from a body of the UAV" | Cartridge rides on a slider on top of the frame plate | Arguable (unclear) |
| First battery directly impacted at first opening by an external second battery | Fresh cartridge nose hits old cartridge tail at the tail opening (F2) | Yes |
| Impulse due to relative motion UAV vs second battery | Pusher moves fresh cartridge relative to the seated drone | Yes |
| First battery released via second opening | Old cartridge exits through the nose into the chute | Yes |
| Second battery becomes held and powers UAV | Yes, T/N re-engage on the fresh cartridge | Yes |

**Prosecution note:** the published claim 1 (US20210031915A1) did not contain "directly impacting", "first and second openings at opposite ends" or "away from a body". These were added before grant, so the patentee is presumptively barred (prosecution history estoppel) from stretching those three elements by equivalents. That makes designing around one of them more reliable. I did not read the file wrapper itself.

**Prior art that could invalidate it (defense, not a design-around):** UC Berkeley UAV@Berkeley "Midair Hotswap" (Spring 2017, page dated 2017-05-01: "a linear sliding rail to push out the old battery case and simultaneously slide in a new one", spring-pin contacts on copper plates, interference-joint latch) and US7339349B2 (Dallab, 2005: new battery pushes old battery along a channel, terminals stay in contact, the incoming battery trips the lever that frees the old one). Both predate Sony's 2018 priority. Neither appears to have been cited (I could not read Sony's citation list; Google's page text was truncated). An invalidity fight is expensive; do not rely on it.

**Design-arounds, cheapest first:**
- **A. Fresh pack never touches the old pack (box ejector does the pushing).** The pusher sled carries a fork/finger (box part) that reaches past the fresh cartridge and pushes the old cartridge's tail (e.g. on tabs or the tail cross-member), with a gap of a few mm between the fresh nose and the old tail for the whole stroke. The fork, not the fresh nose, lifts the T-pawl bumps. This removes "directly impacting ... a second battery" and "receives a mechanical impulse from the second battery" (claims 1, 10, 16). Claim 17 is avoided only if neither cartridge end has a tapered protrusion / matching tapered cavity, so keep both nose and tail faces flat (no cone-in-socket nesting). Same single actuator, same 0.35 s stroke. Re-check that the fresh strips still reach the tail pin group before the old strips leave the nose group with the added gap (lengthen strips by the gap if needed).
- **B. Two-phase swap (eject, then insert).** Box ejector pushes the old pack out first, then the fresh pack goes in; the drone is held up by box power (see 2.2). Avoids Sony (no impulse from the second battery) and DJI US10611252 at the same time. Costs a keep-alive path and about 2x stroke time.
- **C. Old pack exits sideways or downward (not through the opposite end).** Kills claim 1's "openings at opposite ends" but NOT claim 17 or the "impulse from the second battery" theme. Weaker; not recommended alone.
- Not recommended: relying on "away from a body". It is the vaguest element.

### 2.2 US10611252B2 "Systems and methods for UAV battery power backup" (SZ DJI): MEDIUM

- Priority 2014-08-08 (PCT/CN2014/083957), continuation of 14/800,487 (= US9434267). Granted 2020-04-07. Active, adjusted expiry 2035-02-28.

**Claim 1 (station), exact fragments in order:** "An unmanned aerial vehicle (UAV) energy provision station comprising: a) a landing area that supports a UAV when the UAV is landed at the station, wherein the UAV is operably coupled to (1) a first battery that provides power to the UAV and (2) a backup power source that provides power to the UAV when the first battery is decoupled from the UAV, wherein the backup power source is another battery onboard the UAV; and b) a battery replacement member that decouples the first battery from the UAV such that the first battery no longer provides power to said UAV, and wherein the backup power source provides power to the UAV prior to or concurrent with said decoupling, thereby allowing the UAV to remain powered before, during, and after the first battery is decoupled from the UAV."

**Claim 4 (method):** power from battery; with processors, assess reliability of a first and a second backup energy source; select one based on reliability; power the UAV from it before/during decoupling.
**Dependents:** 2 renewable source onboard; 3 movable storage unit with holding stations; 5-13 reliability by SoC / consistency, default source and thresholds; 8 second backup = station power unit; 14-15 landing area + replacement member.

| Claim 1 element | Our design | Match |
|---|---|---|
| Station landing area | Box seat (B1) | Yes |
| UAV coupled to first battery + backup power source that powers it when first is decoupled | During the stroke, the fresh cartridge feeds the bus through the tail pin group | Arguable |
| Backup power source is "another battery onboard the UAV" | Fresh cartridge is onboard during the stroke, but it is the replacement, not a resident backup (spec's backup is a separate onboard battery) | Arguable, the key fight |
| Battery replacement member decouples first battery | Pusher (via fresh pack) | Yes |
| Backup powers UAV prior to or concurrent with decoupling, UAV stays powered | Make-before-break by the two packs (F6) | Yes |

Claim 4 needs processor-based reliability assessment and selection; our hardware ideal-diode OR-ing is not that. Keep it hardware-only.

**Design-around:** keep the UAV alive through the gap with **box power (a station supply through a dedicated seat contact pair), not a battery on the UAV**, and make the packs **break-before-make** (old pack leaves its pins before the fresh pack touches any). Then no "battery onboard the UAV" powers it during decoupling. The 1000 uF cap alone cannot do it (about 0.6 A load for 0.35 s is roughly 210 V of droop on 1000 uF), so either the box supply or a supercap (a capacitor is not a battery) is needed. If the box supply route is used, avoid the contact layouts in 2.18 / 2.19.
Cheaper but riskier: keep make-before-break and argue the fresh replacement pack is not a "backup power source". Medium risk until 2035.

### 2.3 US9346560B1 / US9440545B2 / US9139310B1 / US10363826B2 / US11332033B2 (SZ DJI "UAV battery exchange" family): box-side

All priority 2014-08-08; expiry about 2034-08-08 (US11332033: 2035-11-03). Google lists them active.
- **US9346560 claim 1:** battery changing station; landing area supporting a UAV coupled to a first battery; "the UAV landing area comprises a detectable marker configured to aid the UAV in landing"; battery storage unit with at least one holding station storing a battery; "the detectable marker indicates an orientation of the UAV to be landed"; orientation "facilitates exchange of the first battery with the battery stored in the holding station". Method claims 21/24/27 (UAV detects the marker and generates a command). **MEDIUM for the box if the seat carries an AprilTag/ArUco used for heading.** Design-around: put the fiducial on the drone underside and the camera in the box (box sends corrections), or land with no station marker (mechanical funnel + SLAM pose).
- **US9440545 claim 1:** marker displayed based on remaining charge of the first battery received by a communication unit; claim 2: marker indicates SoC of stored batteries. Avoid dynamic markers tied to SoC.
- **US9139310 claim 1:** station with landing area, second battery, a charging unit able to charge the UAV's first battery, processors receiving SoC wirelessly and generating "prior to the UAV landing" an instruction choosing exchange vs charge. Avoid: always swap, or decide after landing.
- **US10363826 claim 1 (method):** support UAV on landing area; remove first battery with a replacement member; move a holding station within the storage section storing the second battery "at the same time" the first battery is removed. Avoid: the magazine does not index/move while the old pack is being ejected; the fresh cartridge moving out of a stationary magazine is not the holding station moving (E).
- **US11332033 claim 1:** landing area; movable storage unit with a holding station that rotates about an axis; replacement member retrieving the second battery from the holding station and coupling it to the UAV. Avoid: linear (stack or gravity) magazine, no carousel/rotary holder.

### 2.4 US11807130B2 "Battery exchange and charging system for drones" (Skyyfish): LOW-MEDIUM (box)

- Priority 2020-06-03, granted 2023-11-07, active, adjusted expiry 2042-08-02. GB2615734A, CA3090641 related.
- **Claim 1:** first battery holder with a plurality of movable docking bays, each with "a rail for suspending said battery module"; second holder likewise; landing surface; holders and surface positioned so a first-holder bay, the depleted module and a second-holder bay "can be aligned on an axis"; "a first piston" moving a fresh module into the drone that "simultaneously causes the depleted battery module to enter" the aligned bay of the second holder. Claim 21 is the same in substance. Dependents: rotating cylinders, pad aligning the drone, locks, second piston for reverse, wheels on rails, drone bay apertures on two opposing sides.
- Matches our push-through stroke (F2/B2). Avoid by: old pack drops into a fixed chute/bin (no movable multi-bay "second holder"), and fresh packs are not "suspended" from rails (Skyyfish's spec: hanging from a rail by wheels). Either one defeats both independent claims.

### 2.5 US12522387B2 "Aerial drone and battery pack" (Brookhurst Garage): LOW, watch

- Priority 2023-07-07, filed 2023-07-21, granted 2026-01-13, adjusted expiry 2044-01-29. **Pending continuation US 19/440,643 (US20260138761A1, filed 2026-01-06): claims may be broadened. Re-check.**
- **Claim 1:** drone body carrying a battery pack; a slide guide that "suspends from a surface of the drone body" forming a channel the pack slides along; a slide-guide contact sensor on that surface detecting pack contact; a connection port; a port contact sensor detecting the pack has slid into the port. Claim 13 = base station with pack carrier + that drone. Dependents: spring-pin proximity sensor, slots/ledges/tabs, spring-loaded latch (11) and guide blocks one direction while latch blocks the other (12).
- We have a slide channel but no two sensors. Rule: no switch/sensor detecting "pack against the body surface" plus "pack seated in port". If seat detection is ever wanted, use one method only (e.g. pack voltage on the pin groups), not two sensors.

### 2.6 US12286252B2 "Base station for drone battery swap" (Brookhurst Garage): LOW (box)

- Priority 2023-07-07, granted 2025-04-29, expiry about 2043-07-21.
- **Claim 1:** parking plate; gripper of two flaps pivoting to an erected position to secure the drone; chargers each with a battery latch and power port; a pack carrier movable among chargers and the parking position that removes the first pack to a first charger and carries a second pack from a second charger and installs it. Claims 12 (system) and 18 (method) similar. Dependent 4: gripper charging port powers the drone during the swap.
- Our box: no two-flap gripper, old pack goes down a chute. Avoid a pivoting two-flap clamp.

### 2.7 US11387515B2 "Battery assembly, system for replacing battery and method thereof" (Amtran): LOW

- Priority 2018-11-08 (TW 107139718), active, adjusted expiry 2041-03-30.
- **Claim 1:** battery assembly on a drone; chassis space; multiple swing arms pivotally connected, partly in the space; each with a hook and a bump, hooks face each other, bumps face each other; when the battery is in the space it "compresses" the bumps, rotating the arms so the hooks buckle the battery; an electrical connector on the hooks contacts a conductor on the battery side. **Claim 10:** same but connector "within the chassis between the hooks". **Claim 5 / 11:** station tray whose bump pushes the hooks apart to release.
- Our T pawl has a tooth and a bump and faces its mirror twin, but our seated pack does not close the hooks by pressing the bumps (bumps are pressed by the incoming pack to open). Rules: keep spring pins off the pawl leaves; do not adopt a "seated pack presses bump to close hook" toggle. Claim 10 (connector between hooks, conductor on the battery side) is the closer one; our pins are in the walls at different x from the pawls, which with the open-vs-close logic keeps it low.

### 2.8 US9434267B2 (DJI, parent of 2.2): LOW

- Active, expiry 2034-08-08. **Claim 1:** UAV with propulsion unit and onboard power unit with a first battery (powers propulsion + power-consuming unit) and a second battery (powers the power-consuming unit), switching to a second mode where the second battery powers the power-consuming unit "and not providing power to the propulsion unit". Claims 15, 23 similar (23: first battery charges the second).
- Our packs both feed the full bus including ESCs. Rule: no swap-time mode that cuts ESC power while a second source feeds avionics only.

### 2.9 US9851755B2 "Hot swapping batteries in a mobile device" (Lenovo): LOW

- Priority 2015-12-15, active, expiry about 2035-12-15. **Claim 1:** device housing with a recess (upper wall, lower wall, planar surface) open on lateral sides; processor; locking mechanism at the planar surface; first contact rail at the upper wall and second at the lower wall, coupled to the processor. **Claim 8:** battery with contact pins (charging pins) on top/bottom and flat contact rails (discharging rails) on top and bottom.
- Our rails are on the pack, pins on the drone, so claim 1 misses; our pack has no charging pins on the strip faces, so claim 8 misses. Rule: keep that split; cradle charging through the strips and tail pads, not through extra pins on the strip faces.

### 2.10 US11411257B2 "Hot swap battery module and control method thereof" (ETI CA Battery): LOW

- Priority 2019-12-25 (TW), adjusted expiry 2041-03-04. **Claim 1:** hot-swap battery module containing voltage terminal, battery, charge switch, discharge switch, series power transistor, a smart chip turning it off when battery voltage is below terminal voltage, and an ideal diode controller. Rule: keep ideal-diode OR-ing on the drone; the pack stays dumb (cells + strips).

### 2.11 US11749860B2 (Coretronic Intelligent Robotics): LOW

- Priority 2020-06-29, adjusted expiry 2041-11-13. **Claim 1:** pick-and-place mechanism for a bay with a first latch; multi-axis slide table, carrier with a second latch that releases the first latch, pick-and-place device with a catching hook that drags the battery. Relevant only if we move to a box-released latch: release with a simple pusher-mounted cam, not a carrier latch plus a hook that drags the pack.

### 2.12 US11211806B2 (SZ DJI Osmo): LOW

- Priority 2017-02-16, adjusted expiry 2037-08-09. Claims: detect a new battery on an unused interface, if its capacity >= the operating battery make it the operating battery, and shut down if it is non-smart/counterfeit. Rule: no capacity compare + authenticity shutdown logic tied to the second pin group.

### 2.13 Autel "Battery used for unmanned aerial vehicle" family (US10224526, US10090496, US10115944, US10074836 active; US9979000, US10044013 lapsed): LOW

- Priority 2015-12-14, about 2036. Claim 1: multirotor with battery compartment; battery with shell, clamp button on the shell with a hook, restorable elastic piece between shell and button; compartment clamping portion. Rule: keep latches on the slider, not on the pack. (This is why "move retention to the pack" is a bad TECO workaround.)

### 2.14 US9551989B2 (Boeing): LOW (box)

- Priority 2012-05-17, adjusted expiry 2034-05-12. Claims 1/10/15: determine UAV position on the landing pad with a surface sensor, then remove and couple batteries with a replacement mechanism. Rule: no pad-embedded pressure/optical sensor used to locate the drone before the swap; use the passive seat and the drone's own report.

### 2.15 Others read, low relevance
- US10850838B2 (Sony Interactive, 2038): electromagnet releases first battery, drops it, grabs replacement. No electromagnet in our design.
- US12552284B2 (BGU, 2042): replacement battery dropped by a second vehicle from above into an inlet by gravity.
- US11708003B2 / US10369975B2 (DJI, 2036 / 2035): base station with hatch, hatch door, robotic arm moving the pack vertically through the hatch then horizontally (11708003 claims read; 10369975 claims not read).
- US11993373B2: backbone drone with two side batteries and end-piece locks. US10903474B2: mushroom projections + key-hole plate + pivot lever. US11296384B2 (DJI, 2037): mounting plate, lock, spring ejection structure with sliding slot member. US11767135B2: damper between modules. None match our features.
- US11424491B2 (Starship, 2038): robot battery with lid pins, sliding latches, damping pins, IR comm component (claims not read; low because we have no pins on the pack and no comm component).
- US9227601B2 (Corfitsen, 2032): robot draws battery out with suction cups. Not push-through.

### 2.16 Workhorse / Aero Velocity "Flying vehicle systems and methods" family: LOW, watch
- Priority 2020-04-06, about 2041. 15 members checked: claims cover winches, line severing, arches, and **tapered landing nests with electrical contacts on the nest sidewall that the landing legs slide into** (US12037137, US11603219, US11407527, US12365494, US11370561, US11180263, US11332264). The spec also describes sliding battery compartments with latches and installing a second battery before removing the first so the controller stays powered, but no member read claims that. More continuations may be pending (not checked). Rule: if the box feeds keep-alive or charge power, do not put the contacts on a tapered nest sidewall engaged by the legs.

### 2.17 US11111033B1 (Phirst / Modular Flight): LOW
- Priority 2017-05-12, about 2038. Landing platform with many conductive regions + charge sensor + two switch sets that route polarity to whichever regions the legs land on. Rule: fixed, keyed, polarized keep-alive contacts; no segmented pad + switching matrix.

---

## 3. New architecture screen (added 13 Oct, second pass)

Proposed architecture (from the coordinator):
- **N1 (fork ejector):** the box pusher sled carries a two-tine fork. The tines run in side channels of the slider tunnel next to the fresh cartridge and push the OLD cartridge by its tail corners. The fresh cartridge rides on the sled 3 to 5 mm behind the tine tips and never touches the old one. The tines, not the fresh cartridge, lift the passive tail-lock pawls. The old cartridge leaves through the nose into a fixed chute. One linear stroke.
- **N2 (box keep-alive):** before the stroke the pusher head docks on the slider's tail end face. Two spring pins on the head touch two flat, polarized pads on the slider tail face, feeding box power through a drone-side ideal diode. One pin group (8 pins, 4 per pole) on the drone, and the packs break before they make. Box power also keeps the drone powered while it is parked.
- **N3 (alternative keep-alive):** an onboard supercapacitor bank plus boost converter, no battery, packs break before they make.

### 3.1 Targeted searches (round 2)
Queries covered: (a) station or external power fed to a UAV through contacts during a battery swap, with contacts on the bay, the actuator, the gripper or the seat; (b) an ejector or pusher that removes a UAV battery while the replacement follows; (c) a fork or rod pushing a battery out of a channel; (d) supercapacitor hold-up during a battery swap.

New candidates checked: US11267583, US11095129, US11225331, US10696185, US12606988, US12030668, US9612327, US11745898, US11440679, US9826485 / US10531393 / US10667219, US6847192 / US7091701, US9944191, and EP4556380 (no US member). I also re-checked claims already read in pass 1 against N1 to N3: US12049333, US10611252, US9434267, US12286252, US11807130, US10363826, the Workhorse / Aero Velocity family and US11111033.

**Result: no active US claim found that covers (a) station power through actuator or pusher contacts during a UAV swap, (c) a fork pushing a battery out of a channel, or (d) supercap hold-up during a UAV swap as independent-claim subject matter.** The closest items:

| Patent | Owner, expiry (E) | Independent claim gist | vs N1 / N2 / N3 | Risk (E) | Design-around rule |
|---|---|---|---|---|---|
| US12049333B2 (re-check) | Sony, 2041-09 | Claims 1/10/16: first battery "directly impacting" a second battery and receiving "a mechanical impulse from the second battery". Claim 17: same impulse theme without "directly", plus a tapered protrusion on one battery and a matching tapered cavity on the other | N1: the old pack is pushed by box tines, and the fresh pack never touches it. Claim 1's "directly" was added in prosecution, so stretching it by equivalents is presumptively barred | **LOW** (was HIGH) | Keep the 3 to 5 mm gap through the whole stroke, including end-of-stroke overshoot. The fresh pack must never reach the old one, even when the tines bend under load (make the tines stiff, or put a hard stop on the sled). Flat cartridge ends, no tapered nesting. The tines, not the fresh nose, lift the pawls |
| US10611252B2 (re-check) | DJI, 2035-02-28 | Claim 1: station; backup power source "is another battery onboard the UAV" powering the UAV during decoupling. Claim 4: processors assess the reliability of two backup sources and select one (dep. 8: second source = a station power unit) | N2: box supply is not onboard and not a battery. Break-before-make means no pack bridges the swap. N3: a supercap is not a battery, but an equivalents argument is possible | N2 **LOW**; N3 LOW-MED | Hardware-only OR-ing of box supply and pack (ideal diode). No firmware that measures or compares "reliability" of the box supply vs a pack and then selects one (that is claim 4 + dep. 8) |
| US9434267B2 (re-check) | DJI, 2034-08-08 | UAV with an onboard power unit: first battery powers propulsion + loads; second battery powers only the loads ("not providing power to the propulsion unit") in a second mode | N2: box power is not onboard, so no match. N3: an onboard supercap feeding avionics only during the swap is this claim's structure with a capacitor in place of the "second battery" | N2 **LOW**; N3 **LOW-MED** | For N3: if used, feed the whole bus (ESCs included) from the boost output rather than an avionics-only rail, and do not call or switch it as a "mode". This is weaker than N2 |
| US12286252B2 (re-check) | Brookhurst Garage, 2043-07 | Station: parking plate, gripper with two pivoting flaps, chargers each with latch and power port, pack carrier moving packs between the drone and chargers. **Dep. claim 4: a gripper charging port powers the drone during the swap** | N2 is close to dependent claim 4 in spirit, but the independent claim needs the two-flap gripper and the carrier-to-charger loop. We have neither | LOW (watch) | No two-flap pivot clamp. The old pack goes to a fixed chute, not carried to a charger. **Watch Brookhurst continuations** (US20260138761A1 is pending on the drone-side patent; WO2024025163A1 family). A continuation could claim "station contact powering the drone during swap" more broadly |
| US11807130B2 (re-check) | Skyyfish, 2042-08 | Piston pushes fresh module into the drone and "simultaneously causes the depleted battery module to enter" an aligned bay of a second holder with movable rail bays | N1: the old pack goes to a fixed chute, not a movable holder bay | LOW | Fixed chute/bin. Packs not hung from rails |
| US10363826B2 (re-check) | DJI, ~2034-08 | Method: holding station in the storage section moves "at the same time" the first battery is removed | N1: the sled moves the fresh pack, the magazine stays still | LOW | The magazine/holding station is stationary during the stroke. Index it only after the stroke |
| US12030668B2 | Workhorse / Aero Velocity, ~2041 | UAV with first and second battery compartments for sliding insertion, a latch (channel + spline + movable flange), control system "remain[s] at least partially active under power supplied by the first battery when the second battery is removed" | N2/N3: one bay, keep-alive not from another battery | LOW | Single battery bay. Do not add a second permanently installed pack bay |
| US11267583B2 | M. Fehler (individual), 2038-12-11 | Adaptor securing a power source to a drone. One of two energy connections is a pair of energy links movable between a supply-and-secure position and an interrupt-and-release position | N2: the pusher-head pins are fixed spring pins that only compress; they do not move to secure the pack | LOW | Keep-alive and pack contacts must not double as the retention (no contacts that extend to lock the pack and retract to release it) |
| US9826485B2 / US10531393B2 / US10667219B2 | Symbol Technologies (Zebra), ~2035-11 / 2036-03; litigated (ITC 337-TA-1306) | Mobile device: detect removal of the removable power source, a hot-swap mode on a backup source (dep.: a supercapacitor) that deactivates a first component, then a suspend mode for a second period, plus timestamps or wakelock / persisting RAM data | N3 (and N2) only if firmware runs a staged hot-swap-then-suspend sequence | LOW | Firmware rides through the swap without staged modes: no "deactivate component group A, then after time T enter suspend", no saving an app list or RAM to flash on pack removal |
| US6847192B2 / US7091701B2 | CAP-XX, **expired 2021** | Capacitive storage powers the load in the gap between disconnecting one battery and connecting the next; capacitor inside the device housing | N3 exactly | **Free prior art** | Copy freely. This is the supercap hold-up idea |
| US11095129B2 | ~2036 (CIP of 2016 app.) | UAV with a capacitor energy storage system of metacapacitors with specific metadielectric properties | N3 uses ordinary EDLC supercaps | LOW | Use commercial EDLC supercaps |
| US11225331B2 | DJI, **expired (fee)** | Thermoelectric waste heat to supercap, powering the UAV while the battery is replaced | - | Free | - |
| US10696185B2 | DJI, ~2034-08 (active) | Battery replacement device: three stacked linear axes (Cartesian) + clamp gripping the battery | N1 is one linear axis, no clamp | LOW | No 3-axis gantry with a battery clamp |
| US9612327B2 | ~2033 (2013 prov.) | Docking station supplying power + mission control ordering returns by count/position/payload | Fleet logic only | LOW | No return-order logic by payload weight + position + count |
| US11440679B2 | Cowden Technologies, ~2041 | Dock: battery charger via power cable through the securing mechanism, docking module with charging + data ports | N2 is not a cable through a securing mechanism | LOW | - |
| US11745898B2 | Hyundai / Kia, 2041-06 | Seat with tensioned wire that the landing gear hangs on | - | LOW | No wire-catch seat |
| US9944191B2 | (KR 2015) | Swap from another UAV in flight | - | LOW | - |
| EP4556380A1 | Rogoz (PL) | Spring pusher in the bay wall ejects the battery; latch release force ranges | EP only, no US member found | none in US | - |

Not found / not read: any patent on a **fork or tines** pushing a battery out of a channel, other than power-tool pack ejectors (e.g. US6729413, US7661486, US8181717, US9306198 for Black & Decker/Milwaukee-type packs). Those are spring ejectors inside the tool, not a station fork; I did not read their claims. CN utility models for drone nests with push rods (CN122482022A, CN122035362A, CN122501567A) have no US members that I found.

### 3.2 Re-check: Workhorse / Aero Velocity nest family and Modular Flight US11111033 vs N2
- **Workhorse / Aero Velocity:** every nest claim I read (US12037137, US11603219, US11407527, US12365494, US11370561, US11180263, US11332264, US11472572) needs a **tapered nest whose sidewall carries contacts (or contact pads at/below the lower opening) that the UAV's landing legs/feet slide into or land on**. US11332264 claims 5/18 add a 2D barcode giving orientation plus two isolated contact pads charged with a voltage differential. N2 puts the contacts on a docking pusher head and flat pads on the slider tail face. The legs carry no power and the seat has no contact pads, so **no match (LOW)**. Rules:
  - Never route box power through the seat, the legs or the feet.
  - If the seat is tapered and also carries a fiducial, keep it contact-free.
  - Pass 1 also found US12030668: single bay, so LOW.
- **Modular Flight US11111033:** all three independent claims need a landing platform with **multiple conductive regions**, two polarized vehicle contacts landing on any two regions, and a charge sensor with **two switch sets** routing + and - to whichever regions are touched. N2 uses one fixed, keyed, polarized pin pair on a docking head, with no landing-surface regions and no switch matrix, so **no match (LOW)**. Rule: keep the polarity fixed by mechanics (keyed head), not by sensing and switching.

### 3.3 Verdict: N2 or N3?
**N2 (box keep-alive through the docking pusher head) is safer.**
- It is literally outside DJI US10611252 claim 1 ("another battery onboard") and US9434267 (an onboard power unit with a second battery).
- It avoids the Workhorse and Modular Flight contact claims as long as the pins stay off the seat and legs.
- No active US claim was found on station contacts on an actuator powering a UAV during a swap. The nearest is Brookhurst's dependent claim 4, which is not reachable without their two-flap gripper and carrier.
- It also gives parked power.

**N3 (onboard supercap) is LOW-MEDIUM.** The supercap idea itself is free (CAP-XX expired). But an onboard capacitor powering avionics while the pack is out is the structure of DJI US9434267, and close to US10611252 claim 1. Both are open to an argument that a supercap is equivalent to a "second / another battery" until 2034 to 2035. Use N3 only as a small ride-through buffer behind N2 (e.g. covering the dock moment). If N3 is ever used alone, feed the whole bus from it, not an avionics-only rail.

### 3.4 Rules added by this pass
1. The fork tines keep a hard minimum gap (stop on the sled) so the fresh pack never touches the old pack, even with tine bending or overtravel. The tines lift the pawls. Cartridge ends are flat.
2. Keep-alive contacts sit on the docking head and the slider tail face only: fixed, keyed, polarized, spring pins that only compress. Never on the seat, legs, nest sidewall or landing surface, and never doubling as pack retention.
3. Box supply and pack are OR-ed by hardware ideal diodes. No firmware "reliability" selection between sources, and no staged hot-swap/suspend firmware modes.
4. Single battery bay. The magazine stays stationary during the stroke. The old pack goes into a fixed chute. No 3-axis gantry + clamp, and no two-flap gripper + carrier-to-charger.
5. Watch the Brookhurst continuations (US20260138761A1; any continuation of 18/224,985) for broadened "station contact powers drone during swap" claims.
6. Feasibility note (not patent): the docking head carrying the keep-alive pins has to stay still relative to the drone while the sled and tines travel through it. Mount the pins on a part separate from the moving sled.

---

## 4. Free prior art worth stealing (expired or never US-active)

- **US7339349B2, Dallab (2005, lapsed 2016-03-04; would have expired 2026-02-23).** Channel open for one-way travel; first and second batteries slide in; "the first battery can be pushed by the second" while battery terminals stay on the terminal switches so power continues; two pairs of terminal switches (two contacts on the + strip, two on the - strip); spring-loaded lever with a square pin and a triangular pin: the incoming battery lifts the triangular pin and the lever lifts the square pin out of the old battery's recess. This is our F2 + F3 + F5 concept almost exactly, and it is free. It also helps against Sony as invalidity art.
- **UC Berkeley UAV@Berkeley Midair Hotswap (2017):** linear rail pushes out old case while sliding in the new one, spring pins on copper plates, interference-joint latch. Public, free.
- **US4983903A (Samsung, AGV battery exchange, expired 2009):** hook on rodless cylinder pulls the pack out, pushes the new one into jacks.
- Skycatch US10040578 (fee lapse), Amazon US10017265 (fee lapse), DJI US10259332 (fee lapse), Skycatch US20170283090A1 (abandoned): station-arm and latch-handle ideas free to use.

---

## 5. Design-around rules (consolidated, pass 1; see 3.4 for pass-2 additions)

Slider/cartridge (drone side):
1. **No identification component** on the pack: no ID resistor, chip, EEPROM or thermistor reachable through any contact (kills TECO claim 1).
2. **The fresh cartridge must not push the old cartridge** (Sony claims 1/10/16). Either a box-side ejector fork pushes the old pack with a gap to the fresh pack (A), or eject-then-insert (B). The T-pawl release bump is lifted by the box fork, not by the fresh nose.
3. **No tapered protrusion / tapered cavity** on cartridge nose or tail faces (Sony claims 15, 17). Flat ends.
4. Latches stay on the slider, not as spring buttons on the pack (Autel family). Spring pins stay off the pawl leaves (Amtran).
5. Pack stays dumb: no switches, smart chip or ideal-diode controller in the pack (ETI). Ideal diodes on the drone, hardware only, no processor-based "reliability" source selection (DJI US10611252 claim 4) and no capacity/authenticity shutdown logic (DJI Osmo).
6. No pair of presence sensors (pack-on-surface + pack-in-port) (Brookhurst US12522387).
7. Rails (strips) on the pack, pins on the drone; no extra charging pins on the strip faces (Lenovo).

Keep-alive:
8. Safest: bridge the swap with **box power or a supercap, with break-before-make between packs** (avoids DJI US10611252 claim 1 "another battery onboard the UAV"). Cheaper option: keep pack-to-pack make-before-break and accept MEDIUM risk until 2035-02-28.
9. Box keep-alive contacts: fixed polarized pair on a flat seat face; not on a tapered nest sidewall touched by the legs (Workhorse), not a segmented pad with switch matrix (Phirst).

Box:
10. Old pack exits into a fixed chute/bin, not a bay of a movable holder; packs not hung from rails (Skyyfish).
11. Linear magazine, no rotary carousel (DJI US11332033); magazine does not index while the old pack is being ejected (DJI US10363826).
12. No orientation fiducial on the seat; fiducial on the drone, camera in the box (DJI US9346560). No SoC-dependent marker display (US9440545). No pre-landing "swap or charge" decision from wireless SoC (US9139310). No pad surface sensor locating the UAV (Boeing US9551989). No two-flap pivot gripper + carrier-to-charger (Brookhurst US12286252).

---

## 6. What I could not do / caveats

- Google Patents could not be downloaded directly (proxy blocked); claims were read through a summarizing fetch tool that limits verbatim quotes. Claim text above is exact in quoted fragments and paraphrased elsewhere with every element kept. US12049333 claim 1 and US10611252 claim 1 were obtained as full consecutive fragments from the USPTO PDFs.
- Prosecution histories (file wrappers) were not read. The Sony estoppel point is inferred from comparing the published claim 1 to the granted claim 1.
- Legal status and expiry are Google estimates. Terminal disclaimers and maintenance-fee status were not checked at USPTO except where Google showed a lapse.
- Not read: claims of US10369975, US11424491, US10017265 (expired anyway), CN/TW/JP family claims, the Brookhurst pending continuation US20260138761A1, and any pending Workhorse/Aero Velocity, Sony or TECO continuations not shown on Google.
- Search was keyword-based (Google/USPTO via web search), not a CPC-complete search of B64U 50/39, B60L 53/80, H01M 50/247/262/296. Chinese-assignee US filings beyond DJI, DJI Osmo, Autel, Coretronic, Amtran, TECO and ETI may exist.
- Sony's citation list could not be read (page truncated), so I cannot confirm whether Berkeley or Dallab were before the examiner.

---

## Sources
- TECO US11787566B2: https://patents.google.com/patent/US11787566B2/en
- Sony US12049333B2: https://patents.google.com/patent/US12049333B2/en ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12049333 ; published app https://patents.justia.com/patent/20210031915
- DJI US10611252B2: https://patents.google.com/patent/US10611252B2/en ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10611252
- DJI US9434267B2: https://patents.google.com/patent/US9434267B2/en ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9434267
- DJI US11332033B2: https://patents.justia.com/patent/11332033 ; https://patents.google.com/patent/US11332033B2/en
- DJI US9346560B1: https://patents.google.com/patent/US9346560B2/en ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9346560
- DJI US9440545B2: https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9440545
- DJI US9139310B1: https://patents.google.com/patent/US9139310B1/en ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9139310
- DJI US10363826B2: https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10363826
- Skyyfish US11807130B2: https://patents.google.com/patent/US11807130B2/en ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11807130 ; CA: https://brevets-patents.ic.gc.ca/opic-cipo/cpd/fra/brevet/3090641/sommaire.html
- Brookhurst US12286252B2: https://patents.google.com/patent/US12286252B2/en ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12286252
- Brookhurst US12522387B2: https://patents.google.com/patent/US12522387B2/en ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12522387
- Amtran US11387515B2: https://patents.google.com/patent/US11387515B2/en ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11387515
- Coretronic US11749860B2: https://patents.google.com/patent/US11749860B2/en ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11749860
- Lenovo US9851755B2: https://patents.google.com/patent/US9851755B2/en ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9851755
- ETI US11411257B2: https://patents.google.com/patent/US11411257B2/en ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11411257
- DJI Osmo US11211806B2: https://patents.google.com/patent/US11211806B2/en ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11211806
- DJI EP3876335A1 / US20210249870A1: https://patents.google.com/patent/EP3876335A1/en
- Sony Interactive US10850838B2: https://patents.google.com/patent/US10850838B2/en
- Boeing US9551989B2: https://patents.google.com/patent/US9551989B2/en ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9551989
- DJI US11708003B2: https://patents.google.com/patent/US11708003B2/en ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11708003
- DJI US10259332B2: https://patents.google.com/patent/US10259332B2/en ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10259332
- BGU US12552284B2: https://patents.google.com/patent/US12552284B2/en ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12552284
- US11993373B2: https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11993373
- US10903474B2: https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10903474
- DJI US11296384B2: https://patents.google.com/patent/US11296384B2/en
- US11767135B2: https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11767135
- Phirst US11111033B1: https://patents.google.com/patent/US11111033B1/en ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11111033
- Skycatch US10040578B2: https://patents.google.com/patent/US10040578B2/en ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10040578
- Skycatch US20170283090A1: https://patents.google.com/patent/US20170283090A1/en
- Miasole US20180312069A1: https://patents.google.com/patent/US20180312069A1/en
- Amazon US10017265B1: https://patents.google.com/patent/US10017265B1/en
- Starship US11424491B2: https://patents.google.com/patent/US11424491B2/en
- Corfitsen US9227601B2: https://patents.google.com/patent/US9227601B2/en
- Samsung US4983903A: https://patents.google.com/patent/US4983903A/en
- Dallab US7339349B2: https://patents.google.com/patent/US7339349B2/en
- Autel family US10224526B2: https://patents.google.com/patent/US10224526B2/en ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10224526 ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10090496
- Workhorse family: https://patents.google.com/patent/US11332264B2/en ; PDFs .../downloadPdf/11332264, 11498701, 11383859, 12227306, 12037137, 12448156, 11820533, 11603219, 11472572, 11370561, 11407527, 12365494, 11180263, 11254446, 11485518 (all at https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/<number>)
- KR102677782B1: https://patents.google.com/patent/KR102677782B1/en
- UC Berkeley Midair Hotswap: https://uav.studentorg.berkeley.edu/2017/05/01/midair-hotswap/ ; https://uav.studentorg.berkeley.edu/projects/midair-hotswap
- Berkeley HiPeRLab flying batteries (different project): https://spectrum.ieee.org/swappable-flying-batteries-keep-drones-aloft-almost-forever ; https://www.hackster.io/news/berkeley-engineers-design-flying-swappable-batteries-for-drones-to-maintain-flight-91c0d2bddf5e

### Sources added in pass 2 (section 3)
- Fehler US11267583B2: https://patents.google.com/patent/US11267583B2/en ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11267583
- Metacapacitor UAV US11095129: https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11095129 ; https://patents.justia.com/patent/11095129
- DJI US11225331B2: https://patents.google.com/patent/US11225331
- DJI US10696185B2: https://patents.google.com/patent/US10696185B2/en ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10696185
- Caterpillar US12606988B2: https://patents.google.com/patent/US12606988B2/en
- Workhorse US12030668: https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12030668
- US9612327: https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9612327
- Hyundai/Kia US11745898B2: https://patents.google.com/patent/US11745898B2/en
- Cowden US11440679B2: https://patents.google.com/patent/US11440679B2/en
- Symbol family: https://patents.google.com/patent/US10667219B2/en ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10667219 ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9826485 ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10531393
- CAP-XX US6847192B2 / US7091701B2: https://patents.google.com/patent/US6847192B2/en ; https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/6847192
- US9944191: https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9944191
- Rogoz EP4556380A1: https://data.epo.org/publication-server/rest/v1.2/patents/EP4556380NWA1/document.html
