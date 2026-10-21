# Patent landscape: swap stations, hover capture, cages (11 Oct 2026)

Not legal advice: closeness calls are lay readings of claim text (E) for counsel to check. Status and expiry are Google Patents estimates. Patents in earlier reports are not repeated.

**Bottom line**
1. A fixed rib or rod cage with nets most likely falls outside Flybotix US12275542B2 claim 1, which needs ribs folding on end rings, a string sized to their spread and a removable lock (E). The nearer cage risk is Flyability US11661188B2: soft-coated cage beams, and joints that part before beams break.
2. This limited search found no claim on bars that clamp a hovering drone's ring with no pad. Nearest: Navy US11878815B2, pivoting arms gripping a UAV's ring over a platform [Navy 815].
3. Still unread and most relevant: Brookhurst US12312105 (leg pushers, gripper, marker check) for the catch, and Skydio's 2024 hub family (to ~2045 [Skydio 792]) for the Overseer swapper.

## Findings

**Cages**

| Family | Owner, status | Core claim | Nox closeness (E) |
|---|---|---|---|
| US12275542B2, EP4168310B1 | Flybotix; granted 2025, to ~2041 [Flybotix US; Flybotix EP] | Ribs on two end rings that let them fold out; a string through the ribs, first to last, as long as the full spread; a removable ring cap, or hooked rods joining first and last rib. EP claim 1 matches | v8c, v10, v9: low. Folding v2 ribs: medium |
| US11661188B2 (was US20200189712A1) | Flyability; granted 2023, to ~2040 [Flyability GP] | Cage beams with a core and a softer impact-side layer under half its modulus, 3 to 30% of its diameter thick. Other independents: polygon frame modules with matching suspension modules; modular frames whose joints part at 70 to 90% of beam rupture strain [Flyability claims] | Bare rods: low (node sleeves are 55 to 67% of rod diameter, C). Coated rods: high. Break-first nodes: ask |

Fixed ribs with nets: every other Flybotix claim builds on claim 1, and the rope mesh in its description is not claimed [Flybotix US]. ASIO itself folds ten spars on two pole fasteners [UST036]. Residual Nox risk: ribs pivoting in pole hubs, one lacing through every rib, or a first-to-last-rib hook (E).

Second ASIO cage patent: UST036 credits two [UST036], Justia shows one cage family plus a blade-pitch actuator [Justia Flybotix], and EP4424593A1 claims a payload frame, cage optional [Flybotix EP2]. Unresolved; the EP twin or the Swiss priority filing may be counted (E).

**Swap stations**

| Family | Owner, status | Core claim | Nox closeness (E) |
|---|---|---|---|
| US12673792B2, US12649593B2 | Skydio; granted 2026, to ~2045; PCT filed [Skydio 792; Skydio 593] | Claims not reached. Abstract: land on a platform, align to a charging hub, reposition by hub contact, deflect the hub to line up contacts | Catch: low if platform landing is claimed. Overseer swapper: unknown |
| US12312105, US12522387, US12637238 | Brookhurst Garage (inventory drones, per titles); filed Jul 2023, to ~2043 (E) [Brookhurst] | Abstracts only: plate pushers move the legs, a gripper holds the body, a sensor reads a marker; a pack channel with touch and fully-in sensors; pre-charge and swap | Catch: close in function, low if a plate is claimed. Scout cartridge: low if sensors are claimed |
| US9139310B1 and 4 continuations | DJI; granted 2015, to ~2034 [DJI 310] | Claims not read. Summary: a station that picks swap or charge from charge state | Swapper: unknown |

Also low: Skydio US12409949B2, a locking platform sliding sideways from under a cover [OG 949]; US20230257139A1, an arm swapper that also claims an enclosure, HVAC and weather station [Arm 139]. Not reached: Percepto, Hextronics, H3 Dynamics, newer DJI docks, academic filings.

**Capture and centering**

| Family | Owner, status | Core claim | Nox closeness (E) |
|---|---|---|---|
| US11878815B2 | US Navy; granted 2024, to ~2042 [Navy 815] | Platform arms on two pivots, one riding in a track, whose recesses or hooks grip a ring on the UAV; vehicle or buoy mounts in dependents | Iris v2: medium in function, low in structure (sliding bars, no pivots) |
| US11655049B1 | Two individuals; granted 2023, to ~2041 [Rail 049] | Landing pad; one motor turns a plate whose curved channels drive opposing rails inward | Low (belts, no pad) |
| US11655048B2 | Ford; granted 2023, to ~2039 [Ford 048] | Slotted plate, roof mount in a dependent; a motor-turned plate drives centering pins | Low |
| US20190202578A1 | Airobotics; US application abandoned 2021 [Airobotics 578] | Moving sloped pairs trap the legs at their apex | V-jaws: same idea, no US patent |

Abstract or title only: Airobotics US10913546, homing targets [Airobotics]; US9434481B2 and US11542036, hover and aerial recovery [Hover 481; Aerial 036]. Nets and funnels not searched.

## Steal for Nox
- Scout cage (v9, Scout v2): fix ribs or rods at both ends, lace each net bay to its own rails, bond it into one cage on TPU mounts; no lacing through every rib, no cap or first-to-last hook, no clip-apart polygon panels.
- Scout cage: keep rods bare, with no sleeve, tape or EPP wrap along them; a 0.1 to 0.9 mm coat on a 3 mm rod is the 3 to 30% band (E). Put compliance in nodes and mounts.
- Box catcher: belt-driven bars sliding over an open head; no curved-slot drive plate, no pivoting tracked arms.
- Box catcher: a micro-switch at each V-jaw apex as a second seated check, as Airobotics describes [Airobotics 578].
- Swapper: keep seat sensing box-side (camera, hall trip, tug), not touch and fully-in sensors in the pack channel (E).
- Overseer swapper: plain vertical push with two pins and fixed contacts; hold horns, a magnet-floated head and a bail until Skydio's claims are read (E).

## For Spark
- `nox/scout-cage.md` v9 cites the "Flyability patent US20200189712A1 recipe"; it granted as US11661188B2 [Flyability app]. Claim 1 (under 50% modulus) is wider than the "under 20%" recipe in `claude/cage-clean-sheet-research.md`; suspension ratios are claimed only with coated beams or polygon modules [Flyability claims]. Bare rods on TPU mounts look low risk. Ask counsel whether the Elios report's modular-panel steal, or nodes built to let go before a rod snaps (`nox/README.md` breaking order, "TPU first"), read on the 70 to 90% joint claims (E).
- `nox/v2/current-parts.md` S2 cage and the queue's "ASIO-style flat carbon ribs": keep ribs fixed at both rings with per-bay nets; a hook clipping the cage to the frame is not Flybotix's first-to-last-rib hook (E).
- `claude/research/2026-10-11-dock-swap-mechanisms.md` describes pushers, a marker check and seat sensors under US12286252B2. By abstract, they are the subjects of same-day siblings US12312105 and US12522387 [Brookhurst]. Widen queue item 1 to those plus Skydio 792 and 593, which the Skydio report's horn and bail idea for X9 rests on (E).
- `nox/box.md` (How it works; 4x MG90S under To buy) and the `nox-dimensions.md` §4 "Catch envelope" row still describe a lift onto ledges and swing dogs, which §4's hold-down row retired. This read assumes v2 V-jaws; dogs pinning the ring to ledges sat nearer the Navy method (E).
- Earlier reports call these patents fine for a demo. The US experimental-use defence is narrow and an investor demo is commercial: low risk, not exempt (E, ask counsel). Nothing found claims the padless ring catch; a US provisional before any public demo or video keeps foreign filing open (E).

## Sources
- [Flybotix US: US12275542B2, Foldable and removable protective cage (Google Patents)](https://patents.google.com/patent/US12275542B2/en)
- [Flybotix EP: EP4168310B1 (Google Patents)](https://patents.google.com/patent/EP4168310B1/en)
- [Flybotix EP2: EP4424593A1, Drone and a supporting structure for payload (Google Patents)](https://patents.google.com/patent/EP4424593A1/en)
- [Justia Flybotix: Flybotix SA patents (Justia)](https://patents.justia.com/assignee/flybotix-sa)
- [UST036: Flybotix ASIO dossier, Unmanned Systems Technology 036](https://www.ust-media.com/ust-magazine/UST036/61/)
- [Flyability GP: US11661188B2 (Google Patents)](https://patents.google.com/patent/US11661188B2/en)
- [Flyability app: US20200189712A1 (Google Patents)](https://patents.google.com/patent/US20200189712A1/en)
- [Flyability claims: US11661188 (Justia)](https://patents.justia.com/patent/11661188)
- [Skydio 792: US12673792B2, Methods of docking UAVs with base stations (Google Patents)](https://patents.google.com/patent/US12673792B2/en)
- [Skydio 593: US12649593B2, Charging hubs with fixed and movable brackets (Google Patents)](https://patents.google.com/patent/US12649593)
- [OG 949: US12409949B2, Official Gazette (USPTO)](https://patentsgazette.uspto.gov/week36/OG/html/1538-2/US12409949-20250909.html)
- [Brookhurst: Brookhurst Garage patents (Justia)](https://patents.justia.com/assignee/brookhurst-garage-inc)
- [DJI 310: US9139310B1, Systems and methods for UAV battery exchange (Google Patents)](https://patents.google.com/patent/US9139310B1/en)
- [Arm 139: US20230257139A1, Autonomous drone docking station (Justia)](https://patents.justia.com/patent/20230257139)
- [Navy 815: US11878815B2, UAV self-centering and capture system (Google Patents)](https://patents.google.com/patent/US11878815B2/en)
- [Rail 049: US11655049B1, Drone box landing system (Google Patents)](https://patents.google.com/patent/US11655049)
- [Ford 048: US11655048B2, Self-centering landing platform (Google Patents)](https://patents.google.com/patent/US11655048B2/en)
- [Airobotics 578: US20190202578A1, Positioning and locking system (Google Patents)](https://patents.google.com/patent/US20190202578A1/en)
- [Airobotics: Airobotics Ltd patents (Justia)](https://patents.justia.com/assignee/airobotics-ltd)
- [Hover 481: US9434481B2, Launch and retrieval of a hovering aircraft (Google Patents)](https://patents.google.com/patent/US9434481)
- [Aerial 036: US11542036, Aerial launch and/or recovery for unmanned aircraft (USPTO)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11542036)
