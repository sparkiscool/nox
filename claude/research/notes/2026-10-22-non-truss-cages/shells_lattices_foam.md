# Shell, lattice, foam and inflatable protective enclosures for small multirotors (non-truss)

Scope: continuous-surface or cellular energy-absorbing protection vs a rod-and-node truss, for Scout (~520 g E, 3.5 in props on top of arms, ~17 x 17 cm P, ~110 mm tall P, current cage = 3.0 mm solid carbon rods + TPU nodes).

Reference energies for Scout (E, my calc, 0.52 kg): 1 m/s = 0.26 J; 2 m/s = 1.04 J; 3 m/s = 2.34 J; 4.4 m/s (1 m drop) = 5.0 J; 5 m/s = 6.5 J.

Tags: P measured, M mesh, C CAD, E estimate, F frame model. "Derived" = my arithmetic from a cited number.

Blocked or unreadable pages (not used): pmc.ncbi.nlm.nih.gov/articles/PMC8433625 (TPU lattice cut-direction paper, captcha); pmc.ncbi.nlm.nih.gov/articles/PMC10181497 and mdpi.com/2073-4360/15/9/2059 (EPP crushing responses, permission timeout); nature.com/articles/s41598-026-36271-1 (TPU lattice rebound, site error); sciencedirect S2589004226012435 (robots disallowed); mdpi.com/2227-7080/14/9/536 (TPU 95A infill, permission timeout); hit.globalimpact.cn TPU lattice abstract (permission timeout); SPIE graded TPU honeycomb (bot wall); spj.science.org tumbler MAV (site error).

## 1. Printable TPU / nylon lattices vs EPP foam vs carbon rods: specific energy absorption (J/g)

### Takeaway
Per gram, low-density EPP foam (about 2-3 J/g to 50-60% strain, derived from datasheet) beats FDM-printed TPU lattices (measured roughly 0.05 to ~1 J/g quasi-static) by about 2-10x, and is comparable to a carbon rod used as an elastic Euler spring (~1.6 J/g per rod, derived). TPU's edge is near-total elastic recovery over many cycles (up to 99.5%), while EPP takes ~20% compression set after a 50% crush. Printed TPU is a good recoverable bumper skin or node material, not a mass-efficient absorber.

### Cited Findings
- ARPRO EPP typical compressive strength (kPa, converted from psi): 20 g/L: 62 @10%, 83 @25%, 145 @50%, 290 @75%; 30 g/L: 103 / 131 / 200 / 414; 45 g/L: 200 / 241 / 331 / 724; 60 g/L: 276 / 359 / 483 / 1014. Compression set after 25% strain ~8-11%, after 50% strain ~18-21%. Typical values, not spec limits - [Arplank / ARPRO EPP physical properties, 10-2024](https://arplankdirect.com/wp-content/uploads/2024/12/PhysicalPropertyInformation_EPPproducts_ARPLANK_10-2024-R1.pdf)
- Derived from the ARPRO table (trapezoid integration of stress-strain points, E): energy absorbed to 50% strain ~43 kJ/m3 (20 g/L), ~64 kJ/m3 (30 g/L), ~115 kJ/m3 (45 g/L); SEA ~2.1 J/g (20 g/L), ~2.1 J/g (30 g/L), ~2.5 J/g (45 g/L); to 60% strain 30 g/L reaches ~88 kJ/m3, ~2.9 J/g - same source, my calculation.
- Steinbach EPP datasheet: 20 kg/m3: 70 kPa @10% (ISO 844), 80 kPa @40% (ISO 3386); 30 kg/m3: 105 kPa @40%; 45 kg/m3: 180 kPa @10%, 125 kPa @40%; 80 kg/m3: 170 kPa @40% - [Steinbach S 3507a](https://steinbach-ag.de/fileadmin/user_upload/_temp_/S_3507a_-SW__ENG.pdf). Note the two datasheets use different test standards and disagree in magnitude at 40-50% strain (Steinbach lower).
- EPP static compression, 5 densities: strength 0.235 MPa (20 g/dm3), 0.421 (30), 0.954 (80), 0.98 (120), 1.10 MPa (200); residual plastic strain 8.5-16% - [Materials 2021, 14(2), 249](https://www2.mdpi.com/1996-1944/14/2/249)
- EPP 20 to 60 kg/m3: energy absorption +187%, SEA +54%, mean crushing strength +245%; thickness 30 to 75 mm: SEA -4.7% (static) - [Polymers 2023, 15(9), 2059 abstract via DOAJ](https://doaj.org/article/6753b53539f24a16a14296d94d07a065)
- EPP and EPS tested 3.2e-3 to 3.2e-1 1/s: energy-vs-stress envelopes almost coincide across rates (little rate effect in that range); CT showed permanent cell deformation and wall buckling in compressed EPP-90 - [Polimi EPS/EPP compression study](https://re.public.polimi.it/retrieve/e0c31c0c-bd11-4599-e053-1705fe0aef77/Compression%20of%20polystyrene%20and%20polypropylene%20foams%20for%20energy%20absorption%20applications_a%20combined%20mechanical%20and%20microstructural%20study.pdf)
- 3D-printed TPU hexagonal honeycombs, relative density 0.18-0.49: energy absorption 0.01-0.34 J/cm3, max energy absorption efficiency 0.36 (comparable to closed-cell PU foam), repeated compression to densification without failure, elastic recovery; strain rate 0.03-0.3 1/s - [Bates, Farrow, Trask 2016, Univ. Bristol](https://research-information.bris.ac.uk/en/publications/3d-printed-polyurethane-honeycombs-for-repeated-tailored-energy-a/)
- Derived (E): with TPU ~1.15-1.2 g/cm3, those honeycombs weigh ~0.21-0.59 g/cm3, giving SEA ~0.05 to ~0.6 J/g (assumes max energy pairs with max density).
- FDM TPU 95A (BASF Ultrafuse), PEBA and TPC honeycomb/TPMS lattices, quasi-static 10 mm/min, 3 cycles: TPC SEA up to 3.03 J/g; PEBA 0.9-1.39 J/g; TPU generally below 1 J/g (above 1 J/g only for the pomelo-inspired honeycomb). TPU had best recovery: up to 99.5% (2D honeycomb) and 96.6% (3D TPMS) - [Polymers 2025, 17(8), 1111](https://www.mdpi.com/2073-4360/17/8/1111)
- TPU cubic lattices, 49 mm cubes, 0.2 mm layers, relative density 0.25-0.53, 29.6-73.2 g per sample; quasi-static 1 mm/min to 70% strain; absolute SEA not reported (only relative gains, e.g. re-entrant honeycomb +603% over base) - [Materials Research, SciELO](https://new.scielo.br/j/mr/a/bTgwLNzzpVCG4SfPYWmtpvn/?lang=en)
- TPU SuperFlex lattices (octet, BFCC, diamond, truncated octahedron, rhombic octahedron), volume fraction 0.2-0.4, 4.5 mm cells, impacted at 15 m/s: truncated octahedron best; SEA falls with increasing volume fraction (except diamond); numeric SEA only in a figure - [arXiv 2412.06547](https://arxiv.org/html/2412.06547v1)
- eTPU, TPU, PEBA gyroid at 30-70% relative density, 15 mm/min: eTPU ~0.13 MPa (30%) to ~0.45 MPa (70%); TPU lower than eTPU - [iScience / PMC13186003](https://pmc.ncbi.nlm.nih.gov/articles/PMC13186003/)
- Hockey-helmet liner, static: 3D-printed TPU insert absorbed 38.6% of loading energy vs EPP 41.8% vs vinyl nitrile 15.8%; in dynamic NOCSAE impacts EPP was best at front locations, TPU best at side/rear - [McGillivray 2020 thesis, Lakehead](https://knowledgecommons.lakeheadu.ca/jspui/handle/2453/4704?mode=full)
- PLA reinforced gyroid at ~15.5% solid fraction (full FSAE attenuator, 834.8 g) absorbed 7350 J; derived ~8.8 kJ/kg = 8.8 J/g quasi-static; densification ~66% strain. Rigid PLA, crushes permanently - [Int. J. Automotive Technology 2024](https://link.springer.com/article/10.1007/s12239-024-00136-1)
- Carbon rods as Euler springs (EPFL AirBurr): 1 mm radius pultruded CF rods, ~20 cm long, 1.5 J absorbed per rod at compression ratio 0.34; 24 rods + ends = 40 g = 14% of a 282 g robot; foam of equal mass transmitted significantly more force at high drop heights, similar at low heights - [Klaptocz et al., Euler Spring Collision Protection, IROS](https://infoscience.epfl.ch/record/187769/files/cage-IROS.pdf?version=1)
- Derived (E): one 2 mm x 20 cm CF rod ~0.92 g, so ~1.6 J/g per rod; system level ~0.9 J/g (36 J / 40 g, if all 24 rods loaded, which never happens in one hit).
- Polyurea foams 140-220 kg/m3: 0.05-0.13 MJ/m3 quasi-static; under 5-7 J drop impacts peak force up to 28.6-36.9% lower than EVA/TPU footwear foams - [Ramirez & Gupta, arXiv 1810.03705](https://www.arxiv.org/pdf/1810.03705)

### Inferences
- For a 2.34 J hit (Scout at 3 m/s), 30 g/L EPP crushed to 50% needs ~37 cm3 of engaged foam (E): e.g. a 4 x 4 cm contact patch 2.3 cm thick; plateau ~130-200 kPa x 16 cm2 gives ~210-320 N peak (~40-60 g on 520 g) (E). That foam patch weighs ~1.1 g, but a full shell must place that thickness everywhere, so total mass is set by coverage area x thickness, not by SEA.
- A full EPP skin over Scout (17 x 17 x 11 cm box, surface ~0.13 m2) at 20 mm thick is ~2.6 L = ~78 g at 30 g/L (E). That is likely heavier than a rod cage and blocks props unless heavily cut away; foam makes sense as local bumpers (corners, rim ring), not an enclosure.
- TPU lattice SEA (~0.05-1 J/g) means a TPU lattice bumper sized for 2.3 J needs roughly 3-40 g of engaged lattice (E); much heavier than EPP for the same energy. TPU's advantage: prints in-house, survives many hits, bonds to the frame, holds dock geometry tightly.
- EPP compression set (~20% after 50% crush) means bumpers lose thickness after hard hits, which matters for a geometry-only funnel dock fit.
- Carbon rods in bending are already near the best printable/available option per gram for elastic (repeatable) absorption; the case for switching away from rods rests on coverage, simplicity and dock fit, not on J/g.

### Gaps
- No found source gives measured SEA of FDM TPU lattices at drone-relevant impact rates (1-5 m/s) with absolute numbers in text; the 15 m/s study puts SEA only in a figure.
- No direct measured SEA for nylon (PA, PA-CF) printed lattices found within budget.
- EPP SEA numbers above are derived from datasheet stress points, not from a published J/g measurement (the Polymers 2023 EPP paper with absolute SEA was blocked).

## 2. EPP/EPS foam shells and bumpers on small drones (Mambo, Rolling Spider, Crazyflie, Tello, Avata, Flyability-like)

### Takeaway
Consumer toy drones use clip-on plastic or EVA/EPP bumper rings of roughly 10-40 g; spec sheets give mass but never impact data. The best research data on foam vs a structured absorber (EPFL Euler springs) says foam performs similarly at low drop heights and worse at higher ones for the same mass. Flyability, the commercial benchmark for indoor collision tolerance, uses a carbon cage on elastomer suspensions, not foam.

### Cited Findings
- Parrot Rolling Spider: 55 g claimed (57 g measured) without wheels, 68 g with the two roll-cage wheels; wheels ~10 g each; with wheels it rolls along walls and ceilings and can still flip - [PC Watch review (Japanese)](https://pc.watch.impress.co.jp/docs/column/ishii/20140730_660012.html)
- DJI Tello propeller guards: listed weight 10 g (unclear if per set) - [Generation Robots](https://www.generationrobots.com/en/403959-propeller-guards-for-tello-or-tello-edu.html)
- StartRC Avata aftermarket guard: high-density EVA foam, 40 g per pair, 209.75 x 95 x 28.6 mm each; vendor claims only, no test data - [KiwiQuads listing](https://kiwiquads.co.nz/product/startrc-dji-avata-eva-propeller-guard-orange-crash-protection-ring/)
- Bitcraze Crazyflie 2.1 Brushless guards: optional because guards add weight, disrupt propeller airflow and raise inertia; designed to detach in high-force impacts to protect PCB arms - [Bitcraze blog 2024](https://www.bitcraze.io/2024/05/the-crazyflie-2-1-brushless-is-getting-optional-propeller-guards/)
- Flyability Elios 3: 1900 g base, 48 x 38 cm, four 5-inch props, cage = carbon fiber with soft coating on thermoplastic-elastomer suspensions; rated at least 100 frontal collisions below 2 m/s and 100 any-direction collisions below 2.4 m/s without damage; below 3 m/s avoids structural damage; full-speed hits in ATTI/SPORT "will kill the drone" - [Elios 3 tech spec](https://nexxis.com/wp-content/uploads/sites/3/2022/06/20220519_E3_Tech_Spec_web_version.pdf)
- EPFL GimBall: ~370 g, 34 cm, elastic spherical cage on a passive carbon double-ring gimbal; flew hundreds of meters in a forest hitting trees - [EPFL news](https://actu.epfl.ch/news/gimball-a-crash-happy-flying-robot); survey gives a gimbal-cage design at 385 g with 84 g frame (~22%) and 56 g gimbal (~15%) - [Collision-tolerant Aerial Robots survey, arXiv 2212.03196](https://arxiv.org/pdf/2212.03196)
- EPFL Euler-spring vs foam of equal mass (40 g on 282 g robot): similar at low heights; at high drop heights foam transmits significantly more force; Euler cage survived 46/50 in-flight wall hits - [Klaptocz et al.](https://infoscience.epfl.ch/record/187769/files/cage-IROS.pdf?version=1)
- EPFL dual-stiffness frame (Mintchev 2017): 0.3 mm fiberglass outer frame with magnetic breakaway joints and elastic bands; protection 4.66 g = 9% of mass (survey); >50 drops from 2 m without permanent damage - [Robohub](https://robohub.org/insect-inspired-mechanical-resilience-for-multicopters), [survey](https://arxiv.org/pdf/2212.03196), [IEEE Spectrum](https://spectrum.ieee.org/epfl-bioinspired-hard-to-destroy-drone) (Spectrum notes cages can grow a drone's size by "upwards of 60 percent").
- Miniature 25 g quadrotor with 3 g rigid surround frame (~12%) survives up to 4 m/s - [survey arXiv 2212.03196](https://arxiv.org/pdf/2212.03196)

### Inferences
- No source found that publishes a foam-shell drone with mass and measured impact survival; foam on small drones is mostly accessory guards. Treat foam as unproven-by-data in this niche.
- Scaled from Elios 3 (E): its rating (~2-2.4 m/s for 100 hits, <3 m/s structural) is the industry bar for indoor inspection; Scout at 520 g sits in a much easier energy regime per hit (1-2.3 J at 2-3 m/s).

### Gaps
- Parrot Mambo bumper mass and Crazyflie guard mass were not found in fetched pages.
- No published impact test of EPP shells on any small drone found.

## 3. Ducted monocoque designs (cinewhoops, Avata, Bee35 ducts, Skydio R10)

### Takeaway
Ducts are the only enclosure type that can pay back part of their mass in thrust: measured static thrust gains of ~4-25% at 0.25-0.72% tip clearance, but gains vanish or reverse with low-pitch props, loose clearance or forward speed. Commercial duct durability data is anecdotal. Ducts protect the prop plane only; top and bottom stay open.

### Cited Findings
- Ducted vs open static (6000 rpm): thrust coefficient ratio 1.04-1.25 and power-loading ratio 1.06-1.50 for 139-207 mm props; 254 mm low-pitch props lost thrust (ratio 0.87-0.95); duct length 0.28-0.53 D; tighter tip clearance helps, blade pitch above ~10 deg needed for gains; static advantage (+25% CT at J=0) disappears beyond advance ratio J ~0.2 due to duct external drag; duct mass not reported - [Zhao & Shkarayev, Int. J. Micro Air Vehicles 2019](https://journals.sagepub.com/doi/10.1177/1756829319837661)
- Same paper cites ~80% gain for a 1.17 D long duct and ~3% for a 0.17 D short duct - [Zhao & Shkarayev](https://journals.sagepub.com/doi/10.1177/1756829319837661)
- Inflatable perching drone (1.444 kg) with low-density foamed-PLA ducts: hover PWM 69.1% no guards, 68.8% with ducted guards, 69.6% with both duct + hemispherical guards, vs 71.1% for bare motors carrying equivalent guard mass: ducts recovered roughly their own weight in thrust; foamed PLA ~17% lighter per duct than regular PLA - [arXiv 2509.07496](https://arxiv.org/pdf/2509.07496)
- BetaFPV Pavo35 (3.5 in class): frame 113.9 g, 148 mm wheelbase, injection-molded PC whoop ducts, 92 mm duct ID - [BetaFPV](https://betafpv.com/collections/all/products/pavo35-brushless-whoop-frame)
- SpeedyBee Bee35: frame 138 g, 153 mm wheelbase, 90 mm (3.5 in) props; duct mass not listed; durability claims are vendor prose only - [GetFPV](https://www.getfpv.com/speedybee-bee35-3-5-cinewhoop-frame-kit.html)
- Skydio R10 (indoor): 0.785 kg, 26 x 26 x 6.2 cm, composite frame with built-in prop guards, ~0.10 m props; active "turtle mode" flip, not passive righting - [Wikipedia: Skydio R10](https://en.wikipedia.org/wiki/Skydio_R10)
- Shrouded Parrot AR-Drone 2.0 was used for 11,500 deliberate crashes in a learning study - [survey arXiv 2212.03196](https://arxiv.org/pdf/2212.03196)

### Inferences
- Scout's props sit on top of the arms, so a duct ring can mount to the arm plate; a 3.5 in (89 mm) prop in a ~92 mm ID duct gives ~1.7% clearance per side (E), looser than the tested 0.25-0.72% that produced the gains; expect ~0-10% gain, not 25% (E).
- A duct ring sitting near the prop plane does not block the slipstream, unlike a lid or mesh; it is the airflow-safe "shell" option. A top dome or bottom pan above or below the prop must stay well clear (see Q4).

### Gaps
- No measured DJI Avata / Bee35 duct crash load, break speed or thrust delta found; only forum/vendor claims.

## 4. Thin-shell domes (vacuum-formed PETG/PC, composite shells), perforated shells: weight and airflow penalty

### Takeaway
No published measured data found for a thin dome over a small quad's props. Available evidence: obstructions upstream of a drone-scale prop become negligible beyond ~0.5 D. Mesh-shell research protection exists (ABS mesh shields under 50 g for 1.2 kg drones) but with simulation-only and implausible stress results.

### Cited Findings
- Upstream large obstructions near drone-scale props: effect becomes negligible at roughly 0.5 propeller diameter separation; "significant" when closer (magnitudes in full paper) - [Caltech, AIAA 2021-1648 abstract](https://authors.library.caltech.edu/records/zqdwv-3c566)
- Lightweight mesh shield: two 3D-printed ABS mesh semi-ellipsoid shells joined by a ring, under 50 g on a ~1.2 kg drone; only Ansys simulation of a 6 m drop (~10 m/s); reported stresses (1257 vs 11049 MPa) are physically implausible - [Atlantis Press paper](https://atlantis-press.com/article/126004172.pdf)
- Propeller guard drag guidance says cage-type guard flow restriction is not verified on actual cage guards; no thrust numbers - [Propeller Guard Information Center](https://www.propellersafety.com/propeller-guard-reduced-drag/)
- Origami Miura-ori cushion made of thin plastic sheet reduces impact force (unquantified in press release) - [Imperial College news 2018](https://www.imperial.ac.uk/news/188697/origami-cushions-protect-drones-from-collisions/)
- Origami foldable arm quad: 51.2 g total, 14.9 g airframe (fiberglass tiles + Kapton hinges), folds in <0.15-0.2 s on ~1.5 m/s wall hit; folding work ~0.5-2 mJ - [arXiv 1907.07056](https://arxiv.org/pdf/1907.07056)
- MiniCoRe: laser-cut, folded PET sheet body and bumpers with TPU inserts (no masses on page) - [Bilkent repository](https://repository.bilkent.edu.tr/items/ebfaa61b-d4d5-43d0-9068-0d08bd33663c); survey lists it at ~1 m/s collisions - [arXiv 2212.03196](https://arxiv.org/pdf/2212.03196)

### Inferences
- For 3.5 in props (D = 89 mm), keep any solid dome or lid >= ~45 mm above the prop disc (0.5 D) to avoid measurable inflow loss (E, from the Caltech rule). Scout is ~110 mm tall total, so a full solid lid over the props would push height up; a perforated or open-center lid over each prop is the practical version.
- Thin PETG or PC shell (E): 0.75 mm sheet at 1.27-1.2 g/cm3 is ~0.9 g per 10 cm2; a 20 x 20 cm dome is ~40-50 g before cut-outs. Shells crack/dent rather than absorb repeatably; they spread load but store little energy.

### Gaps
- No measured thrust loss vs open-area fraction for perforated domes or grid lids on small quads found.
- No vacuum-formed shell crash data found.

## 5. Inflatable / airbag protection

### Takeaway
Inflatable frames are the lightest absorbers in absolute terms (SoBAR frame 10 g vs 120 g rigid F450 frame) and cut peak impact force ~3-8x by stretching impact time ~10x, but they need pressure (pumps or sealed bladders), deflect under thrust (cos-theta loss) and have leak and puncture risk. Deployable airbags are fall-safety devices for people below, not repeat-hit protection.

### Cited Findings
- SoBAR (ASU): TPU bladder in sewn nylon shell, frame 10 g vs 120 g rigid DJI F450 frame; system 1.14 kg (paper also lists 864 g, unreconciled); flown at 207 kPa; drop tests 2.2 and 3.1 m/s and wall hits ~2 m/s; rigid frame peak ~430-449 N vs soft 52.9 N (x) and 133.3 N (+); impact duration ~10x longer (rigid 8-22 ms vs soft 72-147 ms); at 69 kPa arms split and chassis hit ground; arm deflection under 10 N thrust 14.9 deg (69 kPa) to 5.8 deg (207 kPa); flight time 1 min 30 s - [arXiv 2204.13155](https://arxiv.org/pdf/2204.13155)
- Inflatable perching drone: TPU heat-sealed airbags, joints max 50 kPa; bottom airbag held ~20 kPa to absorb landing; fatigue 289 cycles avg at 50 kPa, 2,144 at 40 kPa; 1.444 kg - [arXiv 2509.07496](https://arxiv.org/pdf/2509.07496)
- Pneumatic silicone soft arms (~1 kg quad, nylon spine): downwash bends arms and causes "considerable reduction" of available control thrust (no number) - [arXiv 2502.12716](https://arxiv.org/html/2502.12716v2)
- SIU drone airbag: 12 in disc, Nylon 6, black-powder inflation in as little as 0.033 s, ~66% impact-force reduction (people-below safety) - [SIU News, June 2026](https://news.siu.edu/2026/06/061726-safer-skies-siu-masters-student-designs-drone-airbag-to-protect-people-below.php)
- HoLoArm (deformable TPU-ligament arms, not inflatable): survived 3 m drop (~7.6 m/s) where the rigid frame broke; peak force 16.5 N vs 24.0 N at 3 m; recovery 0.27-0.75 s - [arXiv 2605.25790](https://arxiv.org/pdf/2605.25790)

### Inferences
- Inflatables conflict with Scout's "no moving parts" and dock requirements: pressure loss changes geometry, a punctured bladder on a construction site (nails, rebar) is a likely failure, and a pump adds parts. A sealed, passively inflated TPU bumper ring is the only variant worth considering, and puncture risk remains (E).

### Gaps
- No sealed (pump-free) inflatable bumper on a small drone with leak-rate or puncture data found.

## 6. Self-righting via shell shape

### Takeaway
Passive self-righting by geometry is proven on ground robots and in a toy-drone patent family, and was used on early AirBurr (gravity returns it to takeoff pose). The rule is geometric: the inverted pose must be an unstable equilibrium (top surface curvature tight relative to CoM height) and CoM must be low. Most modern drones (Skydio R10, tensegrity drones, AirBurr V8) instead use active flipping, which Scout forbids.

### Cited Findings
- Patent family (Yan & Dees, US 8,528,854; 9,067,667; app. 2015/0291272): two or more intersecting elliptical vertical frames, flattened bottom, spherical-tip protrusion on top, weighted mass (battery and electronics) low on the central axis; inverted vehicle rests on the single top point, offset weight tips it, righting moment carries it to the flat base; no powered action required - [Justia patent 20150291272](https://patents.justia.com/patent/20150291272)
- MediumANT passive self-righting shell (ground hexapod): foam-core plates whose edges' perpendicular bisectors are tangent to a circle around the CoM; righted from all 28 faces, and after 15/15 falls off a 1 m ledge; theoretical max incline ~16 deg; an early solid shell raised CoM so much it failed from some faces; material removed above CoM - [King & Revzen, ICRA 2024](https://www.birds.eecs.umich.edu/assets/pdf/2024-King-Revzen-ICRA.pdf)
- AirBurr V1 (25 g): roll cage and gravity return it to takeoff pose passively; V8 uses carbon caging + CoM to roll onto its side, then active legs; righted within 25 s on flat ground, struggled above 10 deg slopes - [IEEE Spectrum](https://spectrum.ieee.org/airburr-mav-can-now-selfright-is-utterly-unstoppable)
- Icosahedron tensegrity (300 g, 95 g shell): active reorientation controller using bi-directional ESCs; survived 7.8 m/s wall hit and 11.7 m/s drop; a prop-guarded 250 g quad fractured at 8.0 m/s - [Berkeley HiPeRLab](https://hiperlab.berkeley.edu/wp-content/uploads/2023/06/TensegrityAerialVehicle.pdf)
- Skydio R10 uses active turtle-mode flip - [Wikipedia](https://en.wikipedia.org/wiki/Skydio_R10)

### Inferences
- For Scout (E, physics): to make "upside down" unstable, the top of the enclosure must present either a single point or a surface whose radius of curvature is smaller than the distance from that surface to the CoM; e.g. if CoM sits ~35 mm above the bottom and the top is at ~110 mm, any top dome with radius < ~75 mm rolls off; a flat top lid makes inverted stable (bad). Side faces must also be unstable (rounded or pointed edges) so it cannot rest on its side.
- The bottom must be the only flat, low-CoM rest face; a flattened bottom also suits a funnel dock.
- A lid with a raised central point (patent style) is cheap in mass and leaves the prop discs open, compatible with the 0.5 D clearance rule if the point is on the center body, not over props.

### Gaps
- No small multirotor with published passive self-righting test data (success rate vs landing pose) found; tumbler-MAV paper unreachable.

## 7. Evidence that shells / foam / lattices beat rod cages on protection per gram at 1-5 m/s

### Takeaway
Weak to none. The only head-to-head equal-mass test found (EPFL) shows foam matching carbon Euler-spring rods at low heights and losing at higher ones; tensegrity rod structures beat a prop guard of equal mass in simulation (34 vs 100 MPa) and in drops. Soft/inflatable frames win on peak force reduction, not on mass-normalized energy. Mass fractions of successful protections cluster at 9-32% of vehicle mass.

### Cited Findings
- Protection mass fractions: Mintchev dual-stiffness 9%; 25 g micro quad rigid frame ~12% (4 m/s); Euler spring cage 14% (282 g); GimBall-type cage ~22% + 15% gimbal; origami foldable 29%; tensegrity 32% (95/300 g, survives 7.8 m/s wall) - [survey arXiv 2212.03196](https://arxiv.org/pdf/2212.03196), [Klaptocz](https://infoscience.epfl.ch/record/187769/files/cage-IROS.pdf?version=1), [HiPeRLab](https://hiperlab.berkeley.edu/wp-content/uploads/2023/06/TensegrityAerialVehicle.pdf)
- Equal 50 g structure mass in Monte Carlo sim: tensegrity mean max stress 34.4 MPa vs prop guard 100.5 MPa; tensegrity under half the guard's stress in 80% of samples - [ar5iv 2211.12045](https://ar5iv.labs.arxiv.org/html/2211.12045)
- Equal-mass foam vs Euler springs: similar at low heights, foam transmits more force at high heights - [Klaptocz](https://infoscience.epfl.ch/record/187769/files/cage-IROS.pdf?version=1)
- Soft inflatable frame: ~3-8x lower peak force than rigid frame at ~2-3 m/s, frame mass 10 g vs 120 g - [arXiv 2204.13155](https://arxiv.org/pdf/2204.13155)
- Suspended rim cage on cables (BU smart cage, carbon tube rim + printed joints) claimed "minimal sacrifice in payload capacity", no numbers - [Liu & Tron, NSF PAR](https://par.nsf.gov/servlets/purl/10288483)

### Inferences
- At Scout energies (0.26-6.5 J for 1-5 m/s), a rod structure with compliant nodes is already in the efficient region; the realistic gain from non-truss options is (a) a TPU or EPP bumper ring at the prop plane / duct for local hits and dock fit, (b) a pointed or tightly domed lid for passive righting, not a full enclosing shell (E).
- If Scout's cage is ~15-25% of 520 g (~80-130 g, E), an EPP ring bumper (e.g. 15 x 15 mm section around a ~60 cm perimeter = ~135 cm3 = ~4 g at 30 g/L, E) is extremely light but only protects the equator; overhead hits (swinging plank) still need a lid or top rods.

### Gaps
- No study found comparing protection-per-gram of a continuous shell vs a rod cage on the same small drone at 1-5 m/s.
- No swinging-object (plank) impact test on any drone found.
