# Swap contacts and blind-mate connectors (11 Oct 2026)

**Bottom line**
1. Scout cartridge: keep the eight Mill-Max 0854 pins (9 A at 10 °C rise, 1,000,000 cycles [08xx]), but land them on hard gold of 30 µin or more [AtlasPCB]. At 0.98 N each they have ~3x what gold needs but only tin's minimum [Samtec]; treat brass like tin (E).
2. Overseer swapper: the same pin, 6 per pole on the drone (54 A at 10 °C rise, E), on hard-gold cartridge pads located by two coned dowels. A floating ODU or Stäubli pin-and-socket is the step up if logged current needs it [ODU; Stäubli].
3. Iris V-jaws: two 0854 per jaw at the apex, tips 1.9 proud (E), onto a gold band on each ±y ring rod. A 90° V seats any rod 0.41 x its radius off the apex, so the 3 and 4 mm rods both land near mid-stroke (E).

## Findings
**Contact types.** A spring pin is a plunger in a barrel on a spring. Current should cross from plunger to barrel; through the spring it can burn it [CCP]. A pin pressed straight onto a pad barely wipes, while leaves and sockets (MULTILAM, SPRINGTAC, RADSOK) wipe as they engage (E).

| Family | A per contact | Resistance | Cycles | Force | Float or stroke; notes |
|---|---|---|---|---|---|
| Mill-Max 0854 pin [08xx] | 9 at 10 °C rise | ≤ 20 mΩ | 1,000,000 | 0.25 N preload, 1.18 N mid | stroke 2.29; 0.5 µm (~20 µin) gold; ~0.4 g (E); ~$1.5 (s2-contacts) |
| Same Sky CPG-91 pin [CPG-91] | 25 | 10 to 30 mΩ | 10,000 | 2.6 N | stroke 1.5; rated 12 Vdc; 0.25 µm gold; series from $0.55 [Same Sky] |
| Mill-Max Maxnetic [Maxnetic] | 7.2 | n/p | 1,000,000 | n/p | polarized magnets self-align; hold force n/p |
| Stäubli CombiTac align 15 [Stäubli] | 100 (Ø6), 10 (Ø2.5) | ≤ 0.25, ≤ 1.1 mΩ | 100,000 | 11.5, 3 N sliding | float ±15 x ±10, 20 axial, 2°; Ag; IP23 mated |
| ODU-MAC / ODU DOCK [ODU] | 25 to 58 (3 mm, by insert and wire) | n/p | ≥ 100,000 (most modules) | n/p | float ±0.6 to ±2.5 by frame |
| Harting Han-Modular float frame [Harting] | by module | n/p | ≥ 10,000 | n/p | float ±2, lock-in ±4; frame 15.9 g |
| Amphenol floating RADSOK [Amphenol] | 120 | n/p | 100 to 200 | n/p | float 2.4 |
| Amass LCB50 plug [Amass] | 55 | ≤ 0.6 mΩ | 100 | ≤ 73.5 N in, ≥ 108 N out | no float; Ag |

n/p: not published in the source read. Same Sky's release says 100,000 cycles for the CPG; its datasheet and Mouser say 10,000 [Same Sky; Mouser; CPG-91].

**Pins in service.** On gold pads at mid-stroke, an unnamed Mill-Max pin (~0.55 N) read 7.2 mΩ new and 6.4 mΩ after 1M cycles, with no break over 1 µs at 10 G. Its one vibration miss came at minimum stroke [Mill-Max test]. Pins are unsealed: the plunger gap admits liquid and dirt [Mill-Max FAQ].

**Surface and force.** ENIG is 2 to 5 µin of gold and wears through in 10 to 20 sliding matings; hard gold is 30 to 50 µin and lasts 500 to 1000+ [AtlasPCB]. Gold needs 0.3 to 0.4 N; tin needs 1 N [Samtec], or over 2 N at end of life [Molex]. Gold on tin corrodes [Molex].

**Float and guides.** Float serves pin-and-socket; pins on pads take error in pad margin and stroke (E). Stäubli floats the charger half into a funnel on the vehicle; its battery-swap version runs on guide pins in a cone [Stäubli]. The contact housing must never guide [DuraDock]. Float does not buy life: the floating RADSOK rates 100 to 200 cycles [Amphenol].

**Robot docks.** Spot takes 58 V, 12 A and Gigabit Ethernet through one connector, located by two indexing cones and exposed only with a robot present [Spot Dock; BD docking]. ANYmal's exposed contacts wear [ANYbotics]. A legged-robot dock patent springs its seat pins so body weight never loads them, and rocks the robot to wipe when contact reads poor or hot [US12560937]. DJI puts contacts on the landing gear; Skydio's magnets centre a floating head (earlier Nox reports).

**Bands under a clamp** (E). Carbon conducts, so a band needs insulation. The ~10 N close (`nox-dimensions.md` §4) puts ~7 N on each 45° flank, 20x gold's need, so wear matters more than force. The rod meets the apex tips from the side in its last ~1.3 mm of travel.

Not read (45-call cap): Preci-Dip, TE and Molex pin data; leaf contacts; DJI's dock-side contact.

## Steal for Nox
- Scout v2: 0.6 to 0.8 mm hard-gold PCB strips in the brass outline; 2 oz copper on both faces matches the brass's conductance at ~0.2 g vs 0.36 g each (E).
- Overseer cartridge: Scout v2-style pads plus two coned dowels, round bore and slot, as on the Scout v1 cartridge (`scout-cartridge-pocket.md`); no floating head (E).
- Swapper (Overseer): the dowels take the push; the contact block never guides [DuraDock].
- Box bays and charge cradle: keep the pins dead until a pack is sensed, as Spot's dock exposes its connector only with a robot present [BD docking] (E).
- Box catcher: wire each jaw's two pins separately so the box can test them through the band at a few mA; on a fail, re-close to wipe, as the patent robot rocks (E).
- Cage (Scout) and Overseer catch frame: a 30 mm hard-gold flex band on the outer half of each ±y ring rod, polyimide down; bench a coupon, as plating bent to r 1.5 may crack (E).

## For Spark
- `v2/s2-contacts.md`, "gold (ENIG) PCB pads are the later upgrade": ENIG wears through in 10 to 20 matings [AtlasPCB]. Order hard gold, 30 µin or more, now, or use Mill-Max target contacts [Mill-Max FAQ] (E).
- Problem 1, "16 W at the 20 mΩ max": at the test pin's 6.4 to 7.2 mΩ, 40 A makes 5 to 6 W (E). Four per pole carry 36 A at 10 °C rise but share unevenly; for bursts over 30 A lasting seconds, use 5 (E).
- Problem 2, "the pins sit mid-stroke": on the hooks they sit at 0.9, under the 1.14 mid, giving 7.9 N (`v2/dimensions.md`) against the README's "≥ 12 N". Tips 0.25 higher (z 26.45) give 1.15 and 9.5 N, at most 4.75 N per hook, under the sustained 6 N case latch V6 passed (`scout-cartridge-pocket.md`) (E). Keep potting epoxy out of the open plunger gap [Mill-Max FAQ] (E).
- Fallback "Same Sky CPG": rated 12 Vdc (a full 4S is 16.8 V) and 10,000 cycles [CPG-91]. The 0855, same family with a crimp tail, is closer [08xx]; check it fits the 3.4 bore.
- Problem 3, "~0.17 J": the box keep-alive cuts it to ~0.011 J (keep-alive report); a 10 Ω precharge pin needs the box's slow push-in (`box.md`) (E).
- `box.md`, "pushes a full one up (~30 N" (Overseer): 6 pins per pole carry 54 A at 10 °C rise against ~25 A hover, but full throttle is ~109 A (`overseer-scout-hardware-reference.md`); log current first. They push back ~14 N. An LCB50-class plug needs ≥ 108 N to pull [Amass], past the 40 N hold (E).
- `nox-dimensions.md` §4, "3 mm rod touches both flanks": at 1.5 from the apex, so keep the apex pin hole under ~2.1 wide, plunger only; check the 0854 plunger diameter. Two pins take ~2.6 N of each jaw's "~10 N" close (E).
- Keep-alive report, "2 per pole (3 on the Overseer)": the same jaws catch both drones; 2 per pole carry 18 A against a 2 to 3 A limit (E).

## Sources
- [Mill-Max 0850 to 0878 spring-loaded pin datasheet (DigiKey)](https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/7138/54_028M.pdf) [08xx]
- [Mill-Max spring-loaded contact mechanical and climatic test report v2.0](https://www.mill-max.com/sites/default/files/external/assets/2017-07/Mill-Max-Spring-Loaded-Contact-Mechanical-and-Climatic-Test-Report_ver2.0.pdf) [Mill-Max test]
- [Mill-Max pogo pin FAQs](https://www.mill-max.com/engineering-notebooks/spring-loaded-pogo-pins-connectors/faqs) [Mill-Max FAQ]
- [Mill-Max Maxnetic connector technology](https://www.mill-max.com/engineering-notebooks/maxnetic-connector-technology) [Maxnetic]
- [Same Sky CPG-91-SMT-B datasheet](https://jp.sameskydevices.com/product/resource/cpg-91-smt-b.pdf) [CPG-91]
- [Same Sky high-current pogo pin release](https://www.sameskydevices.com/news/press-releases/new-high-current-pogo-pin-models-boast-ratings-up-to-25-amps) [Same Sky]
- [Mouser: Same Sky high-current pogo pins](https://www.mouser.com/new/same-sky/same-sky-high-current-pogo-pins/) [Mouser]
- [CCP high-current solution catalog](https://www.ccpcontactprobes.com/sites/default/files/2022-11/CCP_High_Current_Solution_Catalog.pdf) [CCP]
- [Stäubli CombiTac align catalog](https://www.staubli.com/content/dam/ecs/catalogs-brochures/IND/CT-align-11014023-en.pdf) [Stäubli]
- [Stäubli DuraDock power catalog](https://www.staubli.com/content/dam/ecs/catalogs-brochures/WA/DL-DuraDockPower-11014074-en.pdf) [DuraDock]
- [ODU-MAC and ODU DOCK automatic docking brochure](https://odu-connectors.com/fileadmin/media/products/modular-connectors/uebergreifend/downloads/product-brochure/odu-mac-odu-dock-silver-line-automatic-docking-product-brochure-en.pdf) [ODU]
- [Harting docking frame, float mount, 09 14 006 1701](https://harting.com/en-IT/p/09140061701) [Harting]
- [Amphenol Industrial floating mate connectors (Mouser)](https://www.mouser.in/new/amphenolindustrial/amphenol-industrial-floating-mate-connectors/) [Amphenol]
- [Amass LCB50](https://www.china-amass.net/lcb50-product/) [Amass]
- [Samtec connector plating FAQs](https://blog.samtec.com/post/connector-plating-faqs-normal-force-cycles-temperature-and-other-essentials/) [Samtec]
- [Molex: gold or tin vs gold and tin](https://www.molex.com/en-us/blog/gold-or-tin-vs-gold-and-tin) [Molex]
- [AtlasPCB: hard gold vs ENIG for contacts](https://atlaspcb.com/blog/hard-gold-vs-enig-pcb-connector-edge-contact-finish) [AtlasPCB]
- [Spot Dock spec sheet (Boston Dynamics)](https://bostondynamics.com/wp-content/uploads/2020/10/spot-dock.pdf) [Spot Dock]
- [Spot SDK: docking concepts](https://dev.bostondynamics.com/docs/concepts/autonomy/docking) [BD docking]
- [ANYbotics at the IEEE PELS workshop (EPFL)](https://www.epfl.ch/labs/pel/wp-content/uploads/2022/11/1_ANYbotics_PELS-IEEE-Workshop.pdf) [ANYbotics]
- [US12560937, Docks for legged robots (Justia)](https://patents.justia.com/patent/12560937) [US12560937]
