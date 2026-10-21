# Bottom-mounted cartridge: crash-proof latch (14 Oct 2026)

Research only; no Nox doc or shared number changed. Context: Spark is weighing a battery on the Scout's underside so the landing / catch mechanism is also the swap (memory 14 Oct), with contacts that carry no mating force, so all holding force comes from the latch. Numbers tagged P / M / E; loads from `v2/README.md`.

## 1. What changes when the pack hangs underneath
Top-mounted, every vertical case pushes the pack into its seat. Bottom-mounted, the common ones pull it out:
| Case | Direction on the pack | Load (pack 165 P + cartridge 9.1 M = 174 g) |
|---|---|---|
| Hover | out | 1.7 N (E), forever: creep |
| Flight 4 g up | out | 6.8 N (E) |
| Landing / funnel catch 6 g | out (drone stops, pack keeps going down) | 10.2 N (E), every swap |
| Upright drop onto the foot frame, 35 g | out | 59.7 N (E); README sizes 40.5 N at 25 g on the pack alone |
| Side hit 35 g | shear on the latch | 59.7 N (E) |
So the latch moves from "never loaded" to "loaded in its release direction on every flight, landing and the most common crash". This is the whole design problem.

## 2. How batteries actually fall out (field data)
- GoPro Karma recall (2016): the battery latch let the connection go loose and drones lost power in flight; fixed by a latch redesign. [Unofficial Networks]
- AAIB, Mavic 2 Enterprise Advanced (2024): pack came off in flight, likely not fully latched; swollen packs stop the latch seating; side buttons can look locked while the pack is not flush. [AAIB, CHIRP]
- Phantom 4 drop: the plastic the lower latch snaps into broke on impact and the pack came out. [PhantomPilots]
- Yuneec H: pack flew with the latch not clicked, worked loose. [YuneecPilots]
- Mavic 1 / 2 owners strap packs after mid-air ejections. [MavicPilots]
- Skydio X10 (2025 notice): vibration at the battery connector wore it over time, resistance rose, one dock drone lost power; fixed with a shim on the vehicle that cuts battery movement (seating gets tighter). [Skydio NTO]
- Bosch PowerTube (e-bike, pack drops out downward): an unlocked pack bounced out over a bump. [Pedelecs]
Lessons: (1) half-latched is the top cause, (2) the catch the hook bears on breaks before the hook, (3) pack swelling must not touch the latch, (4) heat softens plastic latches, (5) any play at the contacts wears them.

## 3. What others do (vertical or bottom insert)
| Product | Insert | Primary hold | Second safety | Seated check |
|---|---|---|---|---|
| DJI Matrice 3D (Dock 2) | slides in, clicks | snap latch | locking arm closed over it | orange mark hidden when locked [Heliguy] |
| DJI patent US11088414 | slides along rails | spring locking slider moved sideways by the battery rail, snaps back behind it | second rail-and-slot pair locks the other axis | key sits flush only when seated [Google Patents] |
| Freefly Astro | slides onto top rails | two spring latches click | | check both latches fully down, then tug the pack backward [Freefly docs] |
| Bosch PowerTube (bottom drop, e-bike) | into downtube from below | key lock | on unlock the pack drops into a safety restraint; press it to free the pack [Bosch manual] | |
| Motorola radio pack (on the bottom of the radio) | snap on | stamped metal cantilever arms with catches, latch sits in a socket so a drop can't hit it; US5895729, lapsed 2007, free to copy | | |
| Motorola US8741464 | | undercut so drop load closes the hook | | |
| Berkeley Midair Hotswap (bottom slide) | slides into the underside | interference joint | | |
Pattern: one primary snap plus either a second independent catch or a visible / tested seated state. Nobody relies on spring force alone for crash hold.

## 4. Mechanism options
| Option | Holds 60 N out? | Swap from below | Parts / risk | Verdict |
|---|---|---|---|---|
| **A. Two opposed self-locking hooks, drone side, one-piece flexure; cartridge passive with two ledges; box tray + blades release them** | yes: load goes through the hook face into the hinge in tension, not by bending the flexure | push up = snap in; tray lifts, blades through ledge slots open the hooks, tray lowers the pack | 0 extra parts on the drone; latch on drone avoids Autel / Motorola claims | **Best** |
| B. Same, hook sprung by a steel feeler-gauge leaf | yes | same | +2 parts; Spark already has the gauge; Motorola's lapsed stamped-arm latch is the model | fallback if PETG flexure fatigues or creeps |
| C. Bayonet / quarter-turn | yes, very strong | box must rotate the pack | rotary actuator, slower swap | no |
| D. Push-push / rotating tension latch (Oliver US12359687, active to ~2041) | yes | push to latch, push to release | a crash push-in followed by pull-out can release it; patent | no |
| E. Magnets | ~60 N needs large N52 + steel, weak in shear | yes | mass, slides sideways | no |
| F. Servo lock on the drone | yes | yes | +servo, power, a failure mode | no |
| G. Ball detent / friction (Berkeley Midair Hotswap used an interference joint) | holding = spring force, not geometry | yes | lets go above the spring force | no for crash |
| H. Breakaway latch (Rogoz EP4556380 releases at 4 to 27 N on purpose) | no | | opposite goal | no |

## 5. The geometry that makes A crash-proof (current concept, chat 14 Oct, DEFAULT, all E)
- **Retaining face self-locking.** Snap-fit guides: a 90° retaining face cannot be undone except through a release window, and faces past 90° (backpack-buckle style) get stronger under load. [Fictiv, FirstMold]
- **Pull-out closes the hook (Motorola US8741464, "inertia enhanced latching").** Tilt the hook's bearing face so the line of the pull-out force passes on the closing side of the flexure root: face angle θ ≥ atan(DN / DT). Their example: 1.72 / 18.84 mm → 5.19°, built at 7°. Patent active to ~2031 but claims the latch on the battery housing; ours is on the drone.
- **Flexure carries nothing in bending at rest.** The flexure only returns the hook; pull-out goes up the hinge in tension. PETG flexure strain stays under ~3 to 4 % yield. [UAVModel]
- **Overlap and seating pocket.** 1.5 overlap left ~0.85 worst case after ±0.2 print tolerance per part and 0.25 lateral play: too thin. Now 3.0 overlap, 10° undercut, 30° lead-in under the tooth, hook body 20, hinge 1.0 x 8 (strain 1.03 %, return 1.0 N), travel stop at 3.3 open, worst overlap left 2.35, plus a 0.8 pocket in each ledge underside that the tooth tip sits up in, so the tooth can't slide out sideways while any pull-out load or the preload is on it.
- **Release sequence (two box motions).** The pocket means the pack must be lifted before the hooks can open, and the hooks must stay open while the ledges pass the teeth on the way down. So: (1) tray rises under the pack and lifts it 1 mm; (2) blades on a small lift on the tray rise through a slot in each ledge and push the tooth slopes (1.9 N per blade); (3) tray lowers with the pack while the blades ride beside the teeth and hold them open; (4) blades retract. Insert: blades down, tray pushes the fresh pack up, ledge chamfers cam the hooks open, 1 mm over-travel, settle into the pockets, tray drops away (tug test). The ledge slot splits each ledge into two fingers either side of the blade (ledge index roughly doubles, ~0.2).
- **Either hook alone holds the crash** (72 N on one hinge: index ~0.19), which gives Milwaukee-style dual retention without a second part.
- **Ledges on the cartridge frame, not on the pack.** Swelling then can't stop it seating.
- **Release slots recessed** inside the cartridge outline so no crash surface or foot-frame rod can reach the hooks.
- **Hook inertia:** 0.22 N in a 35 g side hit vs 1.0 N return; with the pocket it can't open anyway.

## 6. Half-latch protection (the top field failure)
Put the contact strips so they reach the pins only after the hooks have dropped behind the ledges. A half-in pack then has no power and the drone cannot arm. It is geometry, not a switch, so it stays clear of Milwaukee US11179841 (a switch moved by the latch that cuts power), but ask the attorney (Waiting 7). Free automated tug test (Freefly's pull check): after inserting, the tray drops away; an unlatched pack follows it down, and the box's KW11 / camera sees it.

## 7. Preload
Contacts with no mating force need the latch side to press the cartridge against them. Set preload ≥ the 6 g landing load (10.2 N E) so the pads never lift on a catch; README already asks ≥ 12 N (E). Skydio's X10 connector wear shows why: any movement at the contacts wears them. Simplest source: a TPU 95A crush strip between cartridge top and drone, squeezed when the hooks seat. The hooks are sized for crash + preload: ~72 N total, ~36 N per hook (E).

## 8. Patent notes (lay, E)
Hooks on the drone, cartridge passive, one-piece flexures: stays clear of Autel (clamp button + separate spring on the battery), Motorola US8741464 (latch on the battery housing), DJI US11296384 (sprung ejector), DJI US11088414 (displaced locking slider + spring + flush indicator key). Box blades that release the drone's latch resemble Coretronic's carrier-releases-bay-latch claim: not read, check before selling.

## 9. Not decided here
Hook count and positions come from a sweep (load path to the plate bolts, guard and underside keep-outs); face angle and flexure length from the same run. Bottom mount itself is still Spark's call (13 Oct decision was battery on top; the underside stack sits where a bottom pack would go). DJI Dock 4's aircraft is reported to keep a top-rear bay, not a bottom one. [DroneXL]

## Sources
- https://unofficialnetworks.com/2017/02/02/gopro-puts-karma-drone-back-on-market-3-months-after-getting-recalled-for-catastrophic-failures/
- https://www.gov.uk/aaib-reports/aaib-investigation-to-dji-mavic-2-enterprise-advanced-uas-registration-n-slash-a
- https://chirp.co.uk/?p=31141
- https://phantompilots.com/threads/p4-dropped-from-the-sky.147455/
- https://yuneecpilots.com/threads/1st-real-crash-of-a-yuneec-drone.17928/post-201021
- https://mavicpilots.com/threads/mid-air-battery-fall-out-mavic-pro.94342/post-1683825
- https://support.skydio.com/hc/en-us/articles/36967027973275-NTO-Skydio-X10-Power-Loss-During-Flight-due-to-Battery-Connector-Wear-13-May-2025
- https://www.heliguy.com/blogs/knowledge-base/how-do-i-install-and-remove-the-dji-matrice-3d-series-intelligent-flight-batteries/
- https://patents.google.com/patent/US11088414
- https://docs.freeflysystems.com/astro/other-user-manuals/ecosystem/components/batteries
- https://www.manualslib.com/manual/2173491/Bosch-Powertube-750.html?page=14
- https://www.pedelecs.co.uk/forum/threads/bosch-battery-flew-down-the-road.14507
- https://patents.google.com/patent/US5895729
- https://patents.google.com/patent/US8741464B2/en
- https://patents.google.com/patent/US12359687B2/en
- https://data.epo.org/publication-server/rest/v1.2/patents/EP4556380NWA1/document.html
- https://uav.studentorg.berkeley.edu/2017/05/01/midair-hotswap/
- https://www.fictiv.com/articles/how-to-design-snap-fit-components
- https://firstmold.com/tips/snap-fits/
- https://blog.uavmodel.com/?p=4638
- https://patents.google.com/patent/US10276844 (Milwaukee dual retention)
- https://patents.justia.com/assignee/coretronic-intelligent-robotics-corporation?page=2
- https://dronexl.co/2026/10/08/dji-enterprise-teaser-battery-swap-dock-october-14/
