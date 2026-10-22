# Compliant, tensegrity and elastic-rod protective cages for small multirotors (for Scout, ~520 g E, 17 x 17 x 11 cm P, 3.5-inch props)

Scope note: all numbers below are as reported by the source unless tagged "(E)" = my estimate/derivation. Scout reference energies (E): 0.52 kg at 2 m/s = 1.04 J; at 3 m/s = 2.34 J; at 4 m/s = 4.16 J. A swinging plank is a moving-mass hit, not a wall hit, so its energy depends on plank mass and speed and is not covered by any paper found.

## Tensegrity drones and cages: construction, mass fraction, impact speeds, failure modes, materials

### Takeaway
The only built tensegrity cage drones with hard numbers are the Berkeley HiPeR Lab icosahedron vehicles (Zha, Mueller): 6 mm CF struts + braided nylon/fishing line, 20 to 32% of vehicle mass, surviving 6.5 to 7.8 m/s wall hits and an 11.7 m/s drop, with string snapping as the failure mode. They self-right only actively (bidirectional props), and their mass fraction is far above what Scout could afford at the same impact speed, but at Scout's indoor speeds a much lighter tensegrity is plausible.

### Cited Findings
- Zha et al., IROS 2020 (first tensegrity vehicle): total 252 g, tensegrity 50 g (about 1/5 of mass); struts carbon fiber, 6 mm OD, 20 cm long; cables high-tensile braided fishing line; survived a 6.5 m/s wall collision and kept flying; "all parts within the tensegrity frame remain intact"; no drop tests and no peak accelerations reported — [arXiv 2003.03417](https://arxiv.org/pdf/2003.03417)
- Zha et al., 2022/2023 journal version: vehicle 300 g, shell 95 g (about 32% (E, 95/300)); batteries 75 g, electronics 50 g, motors 80 g; 6 mm OD CF rods, braided nylon strings, string pretension 20 N in model; 2.5-inch props in simulation table — [arXiv 2211.12045](https://arxiv.org/pdf/2211.12045)
- Same paper: survived a 7 m drop (11.7 m/s landing); failed at 10.5 m drop (14.4 m/s) when a string snapped; a commercial 250 g prop-guarded quad fractured after a 3.25 m drop (8.0 m/s); in-flight wall collision survived at 7.8 m/s with vehicle flyable afterwards; target operating speed 6 m/s — [arXiv 2211.12045](https://arxiv.org/pdf/2211.12045)
- Same paper, Monte Carlo (2000 simulated 5 m/s wall hits): mean max stress 34.4 MPa for tensegrity vs 100.5 MPa for a prop guard; tensegrity stress was under half the guard's in 80% of samples; guard lower in only 2.7%; worst case is when the rods carrying the quadcopter nodes hit the wall perpendicular — [arXiv 2211.12045](https://arxiv.org/pdf/2211.12045)
- Note on source discrepancy: the 2022 paper text says rod length 20 mm while the figure caption says 20 cm; the 2020 paper gives 20 cm, so 20 cm is almost certainly correct — [arXiv 2211.12045](https://arxiv.org/pdf/2211.12045), [arXiv 2003.03417](https://arxiv.org/pdf/2003.03417)
- Known limitations: IMU saturation in high-impact collisions; torque limits on slopes/deep ground indentation for re-orientation — [arXiv 2211.12045](https://arxiv.org/pdf/2211.12045)
- Zappetti, Sun, Gevers, Mintchev, Floreano, "Dual Stiffness Tensegrity Platform for Resilient Robotics" (Advanced Intelligent Systems, 2022): struts that buckle give load-bearing ratios (before vs after buckling) up to 10.5 per strut and up to 5.74 for full tensegrity structures; a drone integrated with the platform survived a 2 m fall, a rover a 5 m fall; mass not in abstract — [DOAJ abstract](https://doaj.org/article/66224449d7834e188975f64c2ad42fc3)
- Collision-tolerance survey lists Zha 2020 at TWR up to 3.4:1 and safe collisions up to 6.5 m/s — [Collision-tolerant Aerial Robots: A Survey, arXiv 2212.03196](https://arxiv.org/pdf/2212.03196)

### Inferences
- Tensegrity is over-built for Scout's speeds as published: both Berkeley builds were sized for 6+ m/s. At 2 to 3 m/s the energy is 4 to 9x lower than at 6 m/s (E), so struts could be much thinner (Scout's 3 mm CF stock) and the shell could plausibly fall to well under the 20 to 32% fraction. No one has published a sub-20% tensegrity drone with test data (gap).
- Icosahedron tensegrity is a poor fit for the funnel dock: it is a near-sphere of loose, tensioned geometry that changes shape under load; slide-in docking on pure geometry wants defined, repeatable outer surfaces. A "flat-bottomed" partial tensegrity is an unexplored variant.
- The failure mode to design against is cable rupture at nodes, not strut breakage.

### Gaps
- No peak acceleration (g) data on the electronics for any tensegrity drone was found.
- No tensegrity drone near Scout's mass (~500 g) or prop size (3.5-inch) has been tested.
- Zappetti 2022 full-text mass and drone details were not retrieved.

## Elastic / buckling-rod cages and flexible-arm frames (EPFL Floreano lab and others)

### Takeaway
Several built designs absorb impacts by bending or buckling and show 3 to 8x lower peak force or 3 to 10x longer impact time than rigid frames at 1 to 7 m/s. The best-documented "rods that buckle" design is Klaptocz's Euler-spring cage (40 g on a 282 g robot, about 1.5 J per rod), and the closest analogue to Scout's current construction is de Azambuja's CogniFly (CF rods + TPU 95A joints, <250 g, survives ~7 m/s drops).

### Cited Findings
- Klaptocz et al., "Euler Spring Collision Protection for Flying Robots" (IROS 2013): platform 282 g, protective structure incl. end pieces 40 g (about 14%); 24 pultruded CF rods, 2 mm diameter, about 20 cm long, arranged as 8 tetrahedra of 3 Euler springs around a 12 cm radius frame with 10 cm coaxial rotors; end pieces laser-sintered ABS, joints nylon fishing line; CF E = 113 ± 15 GPa — [EPFL infoscience PDF](https://infoscience.epfl.ch/record/187769/files/cage-IROS.pdf?version=1)
- Same: about 1.5 J absorbed per rod at a compression ratio of 0.34 (equated to a 300 g platform falling about 51 cm); several rods usually share a hit; rigid protection produced high forces even at 10 cm drops; Euler-spring force stayed nearly flat as drop height rose to 75 cm, whereas foam transmitted significantly more force at higher drops; accelerometers saturated at ±100 g; 50 in-flight wall collisions, 4 caused damage (2 soft rotors flexing into each other, 2 nylon string failures) — [EPFL infoscience PDF](https://infoscience.epfl.ch/record/187769/files/cage-IROS.pdf?version=1)
- (E) Klaptocz rod buckling load from the paper's formula with r = 1 mm, L = 20 cm is about 22 N per rod; per-rod mass about 0.9 g — derived from [EPFL infoscience PDF](https://infoscience.epfl.ch/record/187769/files/cage-IROS.pdf?version=1)
- AirBurr (Klaptocz, EPFL LIS): rigid central core surrounded by tetrahedral buckling springs; earlier V7 used a carbon cage; upright recovery is active, using spring-loaded CF legs (an earlier V2 demonstrated gravity-based self-recovery) — [EPFL LIS AirBurr page](https://www.epfl.ch/labs/lis/research/completed/airburr-3)
- Mintchev et al., "Insect-Inspired Mechanical Resilience for Multicopters" (RA-L 2017): 50 g quad; dual-stiffness frame rigid in flight, softens/folds on impact; 0.3 mm fibreglass external frame held by 4 magnetic joints that break away, elastic bands pull it back; >50 drops from 2 m with no permanent damage — [EPFL infoscience](https://infoscience.epfl.ch/record/225943?ln=en), [NCCR Robotics summary](https://nccr-robotics.ch/wp-content/uploads/2020/11/32-Insect-Inspired-Mechanical-resilience-for-multicopters.pdf)
- Survey: Mintchev compliant elements weigh 4.66 g, 9% of total — [arXiv 2212.03196](https://arxiv.org/pdf/2212.03196)
- de Azambuja et al., "A Flexible Exoskeleton for Collision Resilience" (CogniFly, Polytechnique Montreal): carbon fiber rod cage with 3D-printed TPU 95A flexible joints plus a rigid printed central part; flexible frame drone 241 g vs a 239 g all-ABS rigid comparison frame; 210 x 210 x 120 mm; fitted model k = 7040 (units not stated, presumably N/m), c = 46; repeated drops from 262 cm (about 7 m/s) with no damage; median peak acceleration of the rigid frame at 30 cm drop was higher than CogniFly at 150 cm — [arXiv 2107.11090](https://arxiv.org/pdf/2107.11090)
- Caveat: CogniFly's "five-fold" improvement is the ratio of drop heights (150/30), not a measured acceleration ratio; exact g values appear only in a figure — [arXiv 2107.11090](https://arxiv.org/pdf/2107.11090)
- Liu & Karydis (UC Riverside), "Toward Impact-resilient Quadrotor Design..." (passive prismatic spring arm): 1.419 kg; each arm a sprung brass slide with an RC-car shock; recovered from wall hits up to 2.58 m/s (8/10 single-arm) and 1.92 m/s (9/10 two-arm); free fall at 5.9 m/s (1.8 m) without damage; with arms locked by jigs and recovery off the robot fell immediately; no force/acceleration data — [arXiv 2011.02061](https://arxiv.org/pdf/2011.02061)
- SoBAR (soft-bodied aerial robot, arXiv 2204.13155): 1.14 kg; inflatable nylon-fabric/TPU-bladder arms, soft frame about 10 g vs 120 g for a DJI F450 rigid frame; impact time 68 to 165 ms soft vs 8 to 22 ms rigid; peak force at 207 kPa about 52.9 N ("x") and 133.3 N ("+") vs about 449 N and 430 N rigid (about 88% and 69% reductions, from tables; text claims 11.2x, which tables do not support); drop tests at 2.21 and 3.1 m/s; wall hits up to about 2 m/s — [arXiv 2204.13155](https://arxiv.org/pdf/2204.13155)
- Bioinspired soft ring quadrotor (arXiv 2511.05426, 2025): 405 g, 216 mm diameter; ring airframe of four woven glass-fibre strips 145.5 x 54 x 0.5 mm; frame stiffness 0.1 N/mm (vs 100 N/mm rigid comparison); survived frontal collisions at 3 and 4.5 m/s (abstract says 5 m/s); peak control-unit acceleration 25.2 g at 3 m/s; impact duration about 10x a rigid drone; glancing-collision peak loads reduced up to 39x; proposed optimal stiffness range 0.006 to 0.77 N/mm — [arXiv 2511.05426](https://arxiv.org/html/2511.05426v1)
- DART dual-stiffness aerial robot (arXiv 2504.18780, 2025): 1.3 kg, PLA arms on bearings with extension springs, servo cam locks arms rigid; fitted ground stiffness 5500 N/m rigid vs 750 N/m flexible; contact time 46 vs 177 ms (0.05 m drop) and 52 vs 145 ms (0.2 m drop, about 2 m/s) — [arXiv 2504.18780](https://arxiv.org/html/2504.18780v1)
- Patnaik et al. foldable quadrotor with torsion-spring arms (arXiv 2105.12273): 1.112 kg sim mass, chassis + guards about 400 g (80 g heavier than conventional); 0.3 Nm/rad torsion springs; impact duration about 25 ms vs about 1 ms rigid; flight tests 1 to 2.5 m/s; rebound about 0.28 m/s after 1.4 m/s (simulation) — [arXiv 2105.12273](https://arxiv.org/pdf/2105.12273)
- SQUEEZE (Patnaik et al., IROS 2020): torsional springs at every arm hinge; no numbers in abstract — [arXiv 2010.12726](https://arxiv.org/abs/2010.12726v1)
- Rotorigami (Sareh, Kovac et al., Science Robotics 2018): rotating Miura-ori-type origami ring made from a thin plastic sheet protects all props from side hits, reduces impact force and prevents post-collision spin; survey gives about 2 m/s safe interaction; mass not reported in sources found — [Imperial news](https://www.imperial.ac.uk/news/188697/origami-cushions-protect-drones-from-collisions/), [arXiv 2212.03196](https://arxiv.org/pdf/2212.03196)
- MiniCoRe (Dilaveroglu & Ozcan, Bilkent): origami foldable shrouds; survives low-speed (about 1 m/s) collisions; reduces force by lengthening impact time — [Bilkent repository](https://repository.bilkent.edu.tr/items/789631e8-f3bc-44e6-9845-717fae2bda37/full), [arXiv 2212.03196](https://arxiv.org/pdf/2212.03196)
- Kornatowski et al. foldable origami cargo cage (EPFL): CF tubes with 3D-printed flexible joints, opens/closes in 1.2 s; no impact numbers — [NCCR Robotics summary](https://nccr-robotics.ch/wp-content/uploads/2020/11/40-A_foldable_cargo_drone.pdf)

### Inferences
- Scout's current CF rod + TPU node cage is already in the CogniFly family (compliant joints, rigid rods). The published evidence says the compliance comes mostly from the TPU joints; so tuning node durometer/geometry is the cheapest lever, not a new architecture.
- Euler-spring sizing for Scout (E): pinned-pinned buckling F = pi^2 E I / L^2. With E about 113 GPa: 3.0 mm rod, L = 100 mm gives about 440 N; L = 150 mm about 200 N; 2.0 mm rod, L = 100 mm about 88 N; 1.5 mm rod, L = 100 mm about 28 N. For Scout (0.52 kg) to stay under about 50 g peak, total buckling force across rods in contact must be under about 255 N (E). So Scout's 3.0 mm stock is too stiff to act as an Euler spring at Scout's ~100 mm spans; it would need 1.5 to 2 mm rods or longer spans. Buckling plateau force is also flat with speed, which is exactly the property wanted (Klaptocz).
- Euler springs and fibreglass rings store energy elastically and return it (rebound), unlike TPU/foam which dissipate some. For a dock and for not tumbling into the inverted pose, some damping (TPU nodes) is useful.
- Energy budget check (E): Klaptocz about 1.5 J/rod; Scout needs about 1 to 2.3 J at 2 to 3 m/s, so 2 to 3 rods buckling together would cover it, which matches how a tetrahedral layout loads.

### Gaps
- Mintchev 2017 full paper (stiffness values, speeds) could not be fetched (robots blocked).
- No paper found reports peak electronics g for both a rigid CF truss and a compliant cage of identical mass at the same speed; most rigid baselines are ABS/PLA frames or are tested at lower heights only.
- Rotorigami mass and tested speed beyond the survey's "about 2 m/s" not retrieved.

## Spring-steel, nitinol, fiberglass or bent-carbon hoop guards that flex as one piece

### Takeaway
The best evidence for one-piece flexing hoops is the 2025 glass-fibre ring quadrotor (0.5 mm woven GF strips, 0.1 N/mm, 25.2 g peak at 3 m/s) and Mintchev's 0.3 mm fibreglass external frame. No peer-reviewed spring-steel or nitinol hoop cage for a multirotor with impact data was found.

### Cited Findings
- Woven glass-fibre ring airframe, four strips 145.5 x 54 x 0.5 mm, stiffness 0.1 N/mm, 405 g vehicle survives 3 to 4.5 m/s frontal hits, 25.2 g peak at 3 m/s — [arXiv 2511.05426](https://arxiv.org/html/2511.05426v1)
- Mintchev 0.3 mm fibreglass external frame on a 50 g quad, 2 m drops x50+ — [NCCR Robotics summary](https://nccr-robotics.ch/wp-content/uploads/2020/11/32-Insect-Inspired-Mechanical-resilience-for-multicopters.pdf)
- SoBAR uses pre-formed spring-steel tape springs, but in the grasper, not the cage — [arXiv 2204.13155](https://arxiv.org/pdf/2204.13155)
- Self-righting frame patent family (US 8528854 etc.): two or more intersecting vertical elliptical hoops (horizontal major axis), flattened bottom, low heavy mass on the axis, small spherical apex on top; hoop materials not specified — [US 8528854](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/8528854)

### Inferences
- Thin GF or CF laminate strips are the hoop material best matched to Spark's composites skills: a 0.3 to 0.5 mm woven GF strip hoop is directly layable. Stiffness can be tuned by strip width/thickness (k scales with w t^3) toward the 0.006 to 0.77 N/mm range the 2025 paper proposes (that range was for a 405 g vehicle; Scout is about 1.3x heavier (E)).
- A hoop cage gives smooth, defined outer surfaces, which suits funnel docking better than tensegrity or loose rod nodes.

### Gaps
- No measured data found for nitinol or spring-steel wire cages on multirotors.
- No mass-per-protection data for hoop vs rod cage.

## Compliant mounts that decouple a cage from the airframe: do they reduce transmitted acceleration at <3 m/s?

### Takeaway
Yes, but the evidence is mostly in impact-time and peak-force terms at 1 to 3 m/s, from compliant arms/frames rather than a separate cage-on-isolators. Measured effects: impact time 3 to 25x longer and peak force 69 to 88% lower. Rotational decoupling (gimbal cages) addresses attitude disturbance, not shock.

### Cited Findings
- SoBAR at 2.2 to 3.1 m/s: peak force 52.9 to 133.3 N soft vs 430 to 449 N rigid; impact time 68 to 165 ms vs 8 to 22 ms — [arXiv 2204.13155](https://arxiv.org/pdf/2204.13155)
- DART at about 1 to 2 m/s: contact time 3 to 4x longer in compliant mode, stiffness ratio about 7x; no newton values — [arXiv 2504.18780](https://arxiv.org/html/2504.18780v1)
- Foldable torsion-spring quad at 1 to 2.5 m/s: impact duration about 25 ms vs about 1 ms rigid — [arXiv 2105.12273](https://arxiv.org/pdf/2105.12273)
- Glass-fibre ring quad: impact duration about 10x rigid; 25.2 g peak at 3 m/s — [arXiv 2511.05426](https://arxiv.org/html/2511.05426v1)
- Klaptocz: Euler-spring transmitted force stays nearly flat with drop height up to 75 cm (about 3.8 m/s (E)); rigid protection gives high forces even at 10 cm drops — [EPFL infoscience PDF](https://infoscience.epfl.ch/record/187769/files/cage-IROS.pdf?version=1)
- GimBall (Briod et al., EPFL): 370 g, 34 cm spherical cage on a passive 3-axis gimbal so impact friction does not disturb inner-frame attitude; flew autonomously through a forest with repeated collisions; survey cites frame 84 g and gimbal 56 g on a 385 g system — [Robohub](https://robohub.org/?p=22147), [EPFL infoscience](https://infoscience.epfl.ch/record/190790), [arXiv 2212.03196](https://arxiv.org/pdf/2212.03196)
- Liu & Karydis: with compliance locked the robot fell immediately after a wall hit; compliant it stayed on the wall then fell (recovery control off) — behavioral, not acceleration data — [arXiv 2011.02061](https://arxiv.org/pdf/2011.02061)

### Inferences
- At <3 m/s, compliance that adds even 10 to 20 mm of stroke cuts average deceleration a lot (E): stopping 3 m/s over 15 mm averages about 30 g; over 3 mm about 150 g. This is the dominant physics; the concept type matters less than available stroke.
- For Scout, a few millimetres of TPU node travel is probably the stroke today (unverified); an isolator layer between cage and arm plate (TPU bushings) adds stroke without changing the outer shape, which keeps the dock geometry.
- A gimbal (rotating) cage is a moving part, so it violates Scout's "no moving parts" rule; Rotorigami's rotating ring likewise.

### Gaps
- No paper isolates "same cage, with and without compliant mounts" and reports electronics g at <3 m/s.
- No data on plank-strike (moving-object) loads.

## Which concepts beat a rigid rod truss per gram, and what supports that?

### Takeaway
Evidence for better protection per gram is indirect but consistent: compliant designs survive 2 to 5x the impact height/energy of rigid frames of similar mass (CogniFly 241 g vs ABS 239 g; Berkeley tensegrity vs commercial guard), and soft frames can even be lighter than the rigid frame they replace (SoBAR 10 g vs 120 g). No study compares a CF rod truss and a compliant cage at equal mass with g measurements.

### Cited Findings
- CogniFly flexible exoskeleton (241 g) at 150 cm drop gave lower median peak acceleration than a 239 g rigid ABS frame at 30 cm — [arXiv 2107.11090](https://arxiv.org/pdf/2107.11090)
- Tensegrity 300 g survived 11.7 m/s drop; commercial 250 g prop-guarded quad broke at 8.0 m/s; simulated stress about one third — [arXiv 2211.12045](https://arxiv.org/pdf/2211.12045)
- Klaptocz Euler cage: 14% of mass, 50 wall hits with 4 damage events, flat transmitted force vs height — [EPFL infoscience PDF](https://infoscience.epfl.ch/record/187769/files/cage-IROS.pdf?version=1)
- Mintchev dual-stiffness: compliant elements 4.66 g (9%) — [arXiv 2212.03196](https://arxiv.org/pdf/2212.03196)
- Glass-fibre ring: "fourfold higher collision resilience than rigid quadrotors" (abstract claim), 39x lower glancing peak loads — [arXiv 2511.05426](https://arxiv.org/html/2511.05426v1)
- Survey states collision tolerance comes with "non-negligible additional weight" and does not rank designs per mass — [arXiv 2212.03196](https://arxiv.org/pdf/2212.03196)

### Inferences
- Mass fraction ranking from builds found (E): Mintchev 9% < Klaptocz 14% < Zha 2020 20% < GimBall frame+gimbal about 36% < Zha 2022 32%. Only Klaptocz and Mintchev are cage-like and under 15%.
- For Scout, the most mass-efficient direction is likely: keep a rod cage but let the rods/hoops bend or buckle (thinner rods or GF strips, longer unsupported spans) and soften nodes, rather than add a heavier rigid truss or switch to tensegrity.

### Gaps
- No equal-mass, equal-speed rigid-vs-compliant cage comparison with electronics g data.
- No data at Scout's mass/prop size specifically.

## Self-righting behavior in these designs

### Takeaway
None of the compliant/tensegrity drone papers achieve passive self-righting; Berkeley tensegrity uses active thrust-based rolling (with bidirectional props), AirBurr used actuated spring legs. The only passive method found is geometric: low center of mass plus a top shape that cannot support the inverted pose (self-righting frame patents; AirBurr V2 "gravity-based" recovery).

### Cited Findings
- Zha 2020: re-orientation is active, needs bidirectional propellers, uses A* planning over icosahedron faces — [arXiv 2003.03417](https://arxiv.org/pdf/2003.03417)
- Zha 2022: thrust bursts 2.8 N per motor; 20 faces, 4 or 9 take-off-ready; payload margin before re-orientation fails 12 to 30 g — [arXiv 2211.12045](https://arxiv.org/pdf/2211.12045)
- AirBurr: active recovery via spring-loaded CF legs; earlier V2 showed gravity-based self-recovery — [EPFL LIS](https://www.epfl.ch/labs/lis/research/completed/airburr-3)
- Klaptocz Euler cage paper describes no self-righting — [EPFL infoscience PDF](https://infoscience.epfl.ch/record/187769/files/cage-IROS.pdf?version=1)
- Patent self-righting frame: elliptical intersecting hoops, flattened bottom, heavy low mass (battery/electronics) on the axis, small spherical apex so the inverted pose rests on one point and gravity rolls it upright — [US 8528854](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/8528854)
- GimBall keeps the inner frame upright through the passive gimbal while the cage rolls — [Robohub](https://robohub.org/?p=22147)

### Inferences
- For Scout, the patent geometry maps directly onto a hoop cage: arcs over the top that converge to a small rounded apex, a flat or wide bottom footprint, and battery mounted as low as possible. This needs no moving parts. Whether Scout's CoM is low enough for a ~110 mm tall body to right itself is a CAD check (gap).
- Compliance helps self-righting indirectly by reducing rebound energy; a stiff elastic cage can bounce the vehicle into an unpredictable final pose (Klaptocz notes a releasing spring can spin the platform).

### Gaps
- No peer-reviewed passive self-righting compliant cage drone with test data was found.
- No quantitative CoM-height vs righting criteria for drones found in these sources.
