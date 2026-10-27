# Dock and landing-station centring mechanisms (27 Oct 2026)

Research only; Nox docs not changed. Builds on `2026-10-11-skydio-x10-dock.md` (Skydio fences, rising hub, horns, pins; not redone) and `2026-10-14-fastest-landings-catchers.md` (DJI Dock 2/3/4 and Hextronics numbers, Skydio 5 x 5 cm target; not redone). Numbers carry a source key in brackets, or E for estimate. "Not found" means the sources searched do not say; no teardown of any DJI dock was found.

**Bottom line**
1. Two families dominate: active pushers that squeeze the drone after it lands (22 % of landing-station patents, 42 % of those parallel pushers) and passive funnels or ramps (63 %) [Galimov].
2. Pushers decouple centring accuracy from landing accuracy; funnels need the better landing [Galimov]. Landing errors accepted by commercial docks sit at 10 to 17 cm in the datasheets [Hextronics], and the mechanism finishes the job.
3. Almost every single-motor centring design closes all axes at once (spiral plate, belts, rotating spokes). The only X-then-Y sequence found uses two motors [US12084211].
4. Multi-size support is rare. The review says parallel pushers and iris diaphragms suit "different sizes"; V/W pushers, rotating pushers and per-leg funnels do not [Galimov]. Commercial multi-model docks swap floors or brackets instead.

## 1. DJI Dock 1 (Matrice 30 dock bundle)
- Mechanism: "driving rods" on the landing pad. Before calibration they are pulled back; after landing "the dock pushes the driving rods forward into place and closes the covers"; before takeoff they pull back again [DJI M30 setup, p. on flight task and pre-flight list].
- The manual never says how many rods, which axes, or the actuator count. A reseller summary describes a front/rear pair and a left/right pair that push the aircraft to the pad centre and check its position; this was repeated in one search summary and not confirmed in the DJI text I could open (E) [DJI M30 setup; Dock 1 search summary].
- Yaw: the aircraft heading must match the arrow on the pad when placed by hand; the Dock 2 manual says otherwise "the aircraft may be damaged" [DJI M30 setup; DJI Dock 2 manual]. So the rods centre in X and Y and rely on the landing heading (E).
- Tolerance: not published. Hold-down: not stated; the rods end "pushed forward into place" with the cover closed.
- Rod motors stop below -20 C [DJI M30 setup].

## 2. DJI Dock 2
- DJI calls it a "slide-ramp centring design" on the landing pad, "further enhances landing success rates"; no geometry, no actuator count [DJI Dock 2 FAQ; Heliguy Dock 2]. A reseller says it needs fewer actuators than Dock 1 and moved to wireless charging (E, seller claim) [Heliguy product page].
- Landing is RTK plus four markers on the pad; no numeric landing tolerance [DJI Dock 2 FAQ; DJI Dock 2 manual]. Operating wind 8 m/s measured at the dock; takeoff and landing stop above it [DJI Dock 2 manual].
- Heading arrow on the pad must be matched when hand-placing [DJI Dock 2 manual]. A passive ramp that sets yaw would not need this, so yaw is probably set by the landing or by gear shape (E).

## 3. DJI Dock 3
- No mechanical centring description found. Official line: RTK plus visual markers on the pad and the cover's inner side; hover ±0.1 m with RTK, ±0.3 m horizontal vision-only [DJI Dock 3 FAQ; DJI Dock 3 support]. The Dock 2 slide-ramp is not confirmed for Dock 3.

## 4. Easy Aerial (rotating spokes), US11220352B2 (expired)
- One rotary motor turns a wheel with a hub and four spokes (two, three or more allowed) under the landing surface. A spoke hits one landing leg, which pivots the drone; the next spoke hits another; after up to one full turn all four legs touch spokes and the drone is centred on the hub. The wheel then turns a further predefined angle so two conductive legs meet the charge terminals, and counter-rotates to clear the legs for takeoff [US11220352].
- DOF: 1 actuator, centres X, Y and yaw together, but only for four legs on a known pattern. Not clamped; hold-down not stated [US11220352]. Tolerance not stated; approach is GPS to 1 to 10 ft, then an LED on the hub [US11220352].
- The review lists it as tied to one drone size and leg count and "does not fix the UAV" [Galimov]. Easy Aerial's product pages were not found.

## 5. Hextronics (Universal, Atlas, Nano, Global)
- Mechanism not found in any source. Datasheet landing errors: Universal ±15 cm, Nano USA ±10 cm, Atlas 300 "radial landing error recovery within 17 cm" [Remote Robotic Universal; Rippingit Nano; Rippingit Atlas 300]. A battery-swap arm or gantry then works on the drone; the radial recovery implies an active centring or ramp stage (E).
- Multi-model: Universal takes Mavic 3 and Anafi variants with conversion kits [Advexure Universal].

## 6. Percepto Air Max base, Airobotics Optimus, Asylon, Nokia, Skycharge, FlytBase docks
- Percepto: "patented docking and charging mechanism" and "patented landing technology" only; no detail [Percepto Base]. US11454988B2 not read.
- Airobotics Optimus: a platform rises through a hatch; a robotic arm swaps battery or payload in under 1.5 min (Janes) [Janes; The Drive]. Centring patent WO2017130181A1: eight trapezoid centring aids lie flat, rise on an actuator into a closed volume, and a ferromagnetic weight on a winch cable is pulled to a magnetic centre trap, then the winch draws the drone down. No tolerance, no drone sizes [WO2017130181A1]. The review also files Airobotics under V/W pushers (one size and leg count) [Galimov].
- Asylon: DroneHome base, fully automated landing, battery swap under 3 min; mechanism not published; arm patent is in the earlier note [Asylon Guardian; 14 Oct note].
- Nokia drone-in-a-box: only product-level text; no centring detail [Nokia 2023].
- Skycharge: nothing found. FlytBase is software; it triggers precision landing with tags on Hextronics docks [FlytBase].
- FoxIT: not found beyond a retailer's aside that it "uses bars" (E, unverified) [Hextronics search].

## 7. Patents that give mechanism detail
- **US12084211 (VTOL nest, Chinese filer)**. X-Y clamp-centring on the inner cabin top plate. X motor drives two X lead screws (opposite hand) through a right-angle drive, closing the Y assemblies symmetrically; stop on a limit switch. Then a Y motor drives two Y lead screws to close two push plates. 2 motors, 2 axes, sequential: X first, then Y. After a rotating platform of 2100 mm diameter (landing leg footprint 410 x 350 mm, coverage 99.6 % of landings, ±500 mm offset), gear pushing puts the aircraft within 10 mm. Pneumatic clamps shaped to the gear profile, spaced to the drone's gear distance, then lock and charge [US12084211]. Only size setting is the clamp spacing.
- **US11655049B1 (Drone box landing system)**. One motor turns a plate with spiral channels; drive pins in the channels move guide rails inward. Rails close from a square to a # shape; one rail pair sits slightly higher so the pairs do not collide. Alternative: hinged arms pull the rails in. Holds the drone "by press-fit pressure" while the platform flips it upside down. Prior art cited: four motors, four drives, at least four limit sensors. No tolerance or sizes [US11655049].
- **US11655048 (Self-centering landing platform)**. One motor and an internal gear turn a plate below a slotted fixed plate; four pins follow offset slots on a spiral toward the centre together. Engages a closed, symmetric "landing structure" on the drone (circle, ellipse, square). Translation only, no yaw; clamping not stated; no tolerance or sizes [US11655048].
- **US11633058 (parcel cabinet)**. One motor, four rollers, four belts, four push rods in a # layout, all moving at once, rods rest at the pad edges [US11633058]. Tolerance, sizes, hold not stated.
- **US20190202578A1 (abandoned)**. Two sloped-face elements driven by two linear motors close on four legs; three legs touch first, further travel rotates the drone until the fourth touches, then it shifts until all four sit in apices (micro switch each). Corrects heading "up to 90 degrees". A second stage lifts four rods into recesses for tight positioning and rigid hold; centring elements can stay pressed on the legs [US20190202578A1]. No numbers beyond counts.
- **US12312108 (logistics station)**. Three centring bars at 120 degrees, each on an actuator, move to the centre; separate roller locks close on arms or feet. No tolerance or sizes [US12312108].
- **CN114261529A**. Belt-driven sliders in grooves along two directions, the two sets staggered; the drone sits on the sliders; sequencing not specified [CN114261529A]. **CN223803859U** (filed May 2025): synchronous belts drive two transverse and two longitudinal centring rods at the pad edge; drive under the pad to shrink footprint; motor count and order not stated [163 article].
- **RU2722249C9**: iris diaphragm petals push the support to the centre [RU2722249C9]. **US20170073084A1**: passive inner and outer ramps for a circular landing gear, no actuators [US20170073084A1].
- **US11939080B2**: electromagnets plus tapered securing rods enter tapered holes in a docking module on the drone; no centring numbers; sizes "small, medium or large" frames [US11939080B2].
- **US20190002127A1**: 45 degree conical legs seat in cones carried on a chain loop moved by one motor; "millimetre precision" fit, drones slightly over 1 m, 0.5 to 3 kg [US20190002127A1].
- **US12071261**: truncated pyramid docking portion with a matching open-bottom shell on the drone and two actuators that only level it; alignment itself passive [US12071261].

## 8. Skydio (from the 11 Oct note)
Legs land in depressions, two members slide in and park the pack over the hub, a hub rises with angled horns and two pins, then a sprung bail latches; 99.91 % dock landings in Q2 2026 [11 Oct note; Skydio]. Patent claims a 5 x 5 cm target [14 Oct note]. Magnets hold the pack.

## 9. Review of the field
Galimov, Fedorenko, Klimchik (Sensors 2020, 20, 3648): classifies landing-station positioning. Usage in patents: passive 63 %, active 22 %, none 8 %, other 7 %; parallel pushers 42 % of active, one funnel for all legs 39 % of passive. Landing precision for a two-part contact site must be half the contact side length. Multi-size: parallel pushers "suitable for use with UAVs of different sizes"; iris diaphragms configurable; V/W pushers, rotating pushers and per-leg funnels "only a given size and number of legs" [Galimov].

## Synthesis

**Families.** (a) Parallel linear pushers or bars, 2 to 4, closing symmetrically (DJI Dock 1 rods, US12084211, US11655049, US11633058, Hextronics possibly). (b) Rotating pushers (Easy Aerial spokes, the spiral-slot pins of US11655048). (c) Iris or V/W shapes. (d) Passive funnels, ramps, depressions (Skydio, DJI Dock 2 slide-ramp, Dock 3 unknown). (e) Arm or capture hardware (Airobotics magnet trap, Asylon arm, US20190202578A1 rods). (f) Cone-on-belly (US20190002127A1, Sunflower-like whole-body funnels).

**Tolerances.** Accepted landing error: ±10 to 17 cm on Hextronics datasheets, ±0.1 m (RTK) to ±0.3 m (vision) hover on Dock 3, 5 x 5 cm target on Skydio, ±500 mm offset with 99.6 % coverage on a 2100 mm turntable nest. Final centring: 10 mm after gear pushing on US12084211; "millimetre" on a cone fit (US20190002127A1); micro-switch seating on US20190202578A1. No source quotes a yaw tolerance; the only yaw numbers are 90 degrees correctable (US20190202578A1) and the hand-placement arrow rule on DJI docks.

**Hold-down.** Most pushers hold only by staying closed on the legs: DJI rods "pushed forward into place" (wind limit 8 m/s at the dock), US11655049 press-fit, US20190202578A1 centring elements left pressed. Dedicated holds: pneumatic clamps shaped to the gear (US12084211), roller locks (US12312108), rods into recesses (US20190202578A1), pins plus bail plus magnets (Skydio), winch (Airobotics). Easy Aerial and the US11655048 pins do not claim a hold.

**One actuator, two axes.** All found designs close the axes together, not in sequence: spiral plate with all rails (US11655049), spiral slots with all pins (US11655048), one motor and four belts (US11633058), one rotating wheel (US11220352). The collision problem when X and Y bars cross is solved by height offset (one rail pair higher, US11655049) or staggered grooves (CN114261529A). The only X-then-Y sequence is two motors (US12084211). No single-actuator lost-motion, cam-dwell or detent sequence was found in patents or products (E: such a design would be Nox's own).

**Two drone sizes on one mechanism.** None found as a stated feature. Commercial answers are modular floors (OmniDock: two Mavic 3 Enterprise or one Matrice 30T [Advexure OmniDock]), adjustable fixing brackets (Heisha 6-drone dock [Heisha]) and conversion kits (Hextronics Universal). The review says parallel pushers can serve different sizes and per-leg funnels cannot [Galimov].

## Relevance for the flat deck (two drones, same width, 155 mm vs 400 mm long, two parallel belly tubes at one spacing)
All E, inferred from the sources above.
- **Width and yaw come free from the shared tube spacing.** A pair of parallel V-grooves, rails or a pair of pushers acting across the width centre both drones on the same geometry; because the tubes are parallel, grooves also fix yaw (the Easy Aerial and US20190202578A1 four-leg designs need extra travel for yaw, this does not). A 155 mm drone is short for a yaw lever, so keep the grooves or pusher faces long enough that 155 mm of tube spans at least two contact points.
- **Length is the axis that differs.** A symmetric bar pair on the length axis centres either drone on the deck centre if both are pushed on their end nodes or pointed ends; stroke is the size difference: (400 - 155) / 2 = 122.5 mm per side at least, plus landing error. A one-sided pusher against a fixed datum stop is simpler and needs half the stroke per bar but puts the two drones' centres 122.5 mm apart; pick the datum that matches the swap arm's reach.
- **Pointed ends** (400 mm) self-centre on V faces of a pusher, a sloped end face acting like the V/W type; flat end nodes (155 mm) need a flat bar face. A bar face with a shallow V (or a pad plus V) handles both.
- **Hold-down:** DJI and US11655049 rely on closed pushers; with 6 mm and 3 mm tubes the grooves can add a captive lip, which the sources do not show, so the hold is from groove depth, not from the pushers (E).
- **Sequencing from one actuator:** width is passive (grooves) and length is one active axis, so the two-axis sequencing problem can be removed instead of solved; if both axes must be active, copy the height-offset rail pair of US11655049 and drive both from one spiral plate (simultaneous), since no sequential single-actuator design exists to copy.
- **Open questions:** the deck stroke budget against the landing error ±40 mm from `v2/box-funnel.md`; whether a 3 mm tube can take pusher load or only the 6 mm.

## Sources
- [Skydio dock note](claude/research/2026-10-11-skydio-x10-dock.md); [Catchers note](claude/research/2026-10-14-fastest-landings-catchers.md)
- [DJI M30 setup: Matrice 30 dock bundle Installation and Setup Manual v1.4](https://dl.djicdn.com/downloads/m30_series_dock_bundle/SetupManual/20231115/M30_Series_Dock_Bundle_Installation_And_Setup_Manual_v1.4_ENIII.pdf)
- [DJI Dock 2 manual: Matrice 3D flight manual v2.0](https://dl.djicdn.com/downloads/DJI_Dock_2/User_Manual/20240814/DJI_Dock_2_User_Manual_V2.01_en.pdf)
- [DJI Dock 2 FAQ](https://enterprise.dji.com/dock-2/faq); [Heliguy Dock 2 precision landing](https://www.heliguy.com/blogs/knowledge-base/how-is-the-precision-landing-of-dji-dock-2-achieved/); [Heliguy Dock 2 product](https://www.heliguy.com/products/dji-dock-2-dock-only/)
- [DJI Dock 3 FAQ](https://enterprise.dji.com/dock-3/faq); [DJI Dock 3 support](https://www.dji.com/support/product/dock-3)
- [US11220352B2 Easy Aerial](https://patents.google.com/patent/US11220352)
- [Galimov et al., Sensors 2020](https://pmc.ncbi.nlm.nih.gov/articles/PMC7374299/)
- [Remote Robotic Universal](https://remoterobotic.com/products/universal-drone-docking-system); [Rippingit Nano](https://rippingitoutdoors.com/products/hextronics-nano-usa-drone-docking-station); [Rippingit Atlas 300](https://rippingitoutdoors.com/products/hextronics-atlas-300-battery-swapping-drone-docking-station); [Advexure Universal](https://advexure.com/products/hextronics-universal); [FlytBase docks](https://flytbase.com/blog/dji-compatible-docking-stations)
- [Percepto Base](https://percepto.co/?p=12025); [WO2017130181A1 Airobotics](https://patents.google.com/patent/WO2017130181A1/en); [Janes Aero A2Z Airobotics](https://www.janes.com/defence-news/air-platforms/latest/aero-a2z-airobotics-partner-to-manufacture-autonomous-drone-in-a-box-solution); [The Drive Airobotics](https://www.thedrive.com/tech/4113/this-new-drone-system-flies-recharges-and-swaps-payloads-without-human-help)
- [Asylon Guardian](https://asylonrobotics.com/solutions/guardiandrone/); [Nokia 2023](https://www.nokia.com/newsroom/nokia-unveils-first-ce-certified-5g-automated-drone-in-a-box-solution-for-secure-reliable-public-sector-and-industrial-operations/)
- [US12084211](https://patents.google.com/patent/US12084211); [US11655049](https://patents.google.com/patent/US11655049); [US11655048](https://patents.google.com/patent/US11655048); [US11633058](https://patents.google.com/patent/US11633058); [US20190202578A1](https://patents.google.com/patent/US20190202578A1/en); [US12312108](https://patents.google.com/patent/US12312108)
- [CN114261529A](https://patents.google.com/patent/CN114261529A/zh); [163 article on CN223803859U](https://www.163.com/dy/article/KJFV4RIQ0519QIKK.html); [RU2722249C9](https://patents.google.com/patent/RU2722249C9/en); [US20170073084A1](https://patents.google.com/patent/US20170073084A1/en); [US11939080B2](https://patents.google.com/patent/US11939080B2/en); [US20190002127A1](https://patents.google.com/patent/US20190002127A1/en); [US12071261](https://patents.google.com/patent/US12071261)
- [Advexure OmniDock](https://advexure.com/products/omnidock-standard-by-counterdrone); [Heisha 6-drone dock](https://heishatech.com/heisha-6-drone-custom-drone-dock-is-released/); [Skydio dock FAQ](https://www.skydio.com/dock/faqs)
