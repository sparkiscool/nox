# Tension-based and bio-inspired protective structures for small drones (for Scout: ~520 g E, 17 x 17 cm P, ~110 mm tall P, 3.5" props, 3.0 mm carbon rod + TPU node cage)

Tag key used here: P = measured/reported by the source; E = my estimate or derivation; anything without a source link is an inference and lives under Inferences.

Scout impact energy reference (E, KE = 0.5 m v^2, m = 0.52 kg): 1 m/s = 0.26 J; 2 m/s = 1.04 J; 3 m/s = 2.34 J; 4 m/s = 4.2 J; 5 m/s = 6.5 J. A 1 m drop lands at ~4.4 m/s (E).

## 1. Net / mesh prop guards and fabric/membrane cages: mass, airflow loss, durability, entanglement

### Takeaway
There is almost no peer-reviewed, measured data on net/mesh/fabric cages for small multirotors. The collision-tolerant-aerial-robots survey finds no net or fabric guard designs. The best airflow numbers available are for solid ducts and folded PVC/PEEK kirigami sheets, not porous mesh. Fabric is proven at the extreme end (Mars airbags), and thin folded film guards are proven at toy-drone scale.

### Cited Findings
- A 2022 survey of collision-tolerant aerial robots (De Petris et al.) covers rigid, compliant, origami, tensegrity, rotating-cage, morphing and bio-inspired protection. It lists **no net, mesh or fabric guard designs**. Its closest item is Librado et al. 2022 (RA-L) on meshing and insulation for caged power-line UAVs — [Collision-tolerant Aerial Robots: A Survey, arXiv 2212.03196](https://arxiv.org/pdf/2212.03196)
- **Kirigami/origami film guard (Sensors 2023, doi 10.3390/s23042150):**
  - Build: a cylindrical PEEK-film guard of **60.7 g (P)**, around a **290 g test mass (P)**, opened and closed by SMA springs.
  - Test: 600 mm drop onto a 150 N load cell.
  - Results: bare drone peak force **71.78 N (0 deg) and 60.46 N (90 deg) (P)**. Guarded peak force **34.8 to 46.0 N (0 deg) and 13.2 to 27.2 N (90 deg) (P)**, a **35.7 to 78.2% reduction (P)**.
  - Airflow tests used PVC sheet with triangular or circular kirigami holes. No commercial guard was compared, and no energy-absorbed figure is given — [MDPI Sensors 23(4):2150](https://www.mdpi.com/1424-8220/23/4/2150)
- **Inflatable/TPU-airbag morphing aerial robot (arXiv 2509.07496):**
  - Mass: 1.444 kg (P), 350 x 350 x 250 mm (P). TPU airbags work at 40 to 50 kPa. Fatigue life is **289 cycles at 50 kPa and 2144 cycles at 40 kPa (P)**.
  - Duct plus hemispherical guard made of PLA foam: hover PWM was **69.1% bare, 68.8% duct only, 69.6% duct plus hemispherical guard (P)**. So the guard cost roughly 1% of throttle, and the duct alone slightly helped.
  - Bottom airbags absorbed a 0.11 m perch drop, peaking at 28 kPa (P). No formal collision-force test was run — [arXiv 2509.07496](https://arxiv.org/pdf/2509.07496)
- **Mars Pathfinder airbags (fabric at the far end of the scale):**
  - Layout: 24 interconnected Vectran spheres with up to 4 fabric layers (P).
  - Impact: initial impact at **18 m/s (P)**, then **15 bounces (P)**. Outer layers are designed to tear sacrificially and protect the inner bladders (P) — [NASA NTRS 20090039418](https://ntrs.nasa.gov/api/citations/20090039418/downloads/20090039418.pdf)
  - JPL gives landing speed as about 20 to 24 m/s, with a ~535 kg test spacecraft plus airbags. MER used 4 airbags with 6 lobes each, made of six 100-denier Vectran outer layers and 200-denier bladders (P) — [JPL, How to land softly on a hard planet](https://www.jpl.nasa.gov/news/how-to-land-softly-on-a-hard-planet)
  - Note: the NTRS (18 m/s) and JPL (20 to 24 m/s) speeds conflict, possibly because they describe different missions or test conditions.
- **Polypropylene safety net vs. spinning propeller (CTU Prague thesis):** a test method was developed for net sections hit by a rotating propeller. The abstract gives no mesh size, speeds or entanglement results — [CTU dspace, Safety net resistance assessment in a case of propeller drone impact](https://dspace.cvut.cz/entities/publication/161c1de4-c5ae-4dd6-a73d-1e2ea7af2d3b)
- **Commercial caged drones (qualitative only):**
  - Adding a cage to a Mavic 2 roughly halves its flight time, from ~30 to ~15 min (P, vendor blog). No cage mass is given — [COPTRZ, Cage match](https://coptrz.com/blog/cage-match-the-cheap-fix-vs-elios-2/)
  - IEEE Spectrum, summarizing EPFL, says protective cages such as Flyability's can enlarge a drone by up to 60% and add mass, reducing lift and range — [IEEE Spectrum](https://spectrum.ieee.org/epfl-bioinspired-hard-to-destroy-drone)
- **Spherical gimbal cage (GimBall, EPFL 2014):**
  - Mass and size: **370 g total, 34 cm diameter (P)**.
  - Test: survived repeated tree-trunk hits flying hundreds of meters through a forest (P) — [EPFL news, GimBall](https://actu.epfl.ch/news/gimball-a-crash-happy-flying-robot)
  - The survey gives Briod et al. 2014 as **385 g total, 84 g outer frame (~22%, E derived), 56 g gimbal** — [arXiv 2212.03196](https://arxiv.org/pdf/2212.03196)

### Inferences
- **Porous mesh over the prop disc.** A screen in the inflow or outflow costs thrust that scales with its solidity (open-area fraction). No small-UAV measurement was found, so the loss can't be quantified.
- **Small prop clearance.** At 3.5 inches, prop-to-wall clearance is small. A tensioned mesh that deflects inward under impact can contact the props unless it stands off by more than its expected deflection (E).
- **Entanglement.** The risk is real for coarse nets and string near spinning props, and for site debris (rebar ties, cable). No drone data was found, so treat it as a design risk. Mitigation: keep nets off the prop disc plane and run strings only in planes the prop tips cannot reach.
- **Sacrificial outer fabric layer.** The Vectran lesson transfers directly: a replaceable outer skin protects the structural layer underneath.
- **Thin film guards (PEEK, PETG sheet, polycarbonate).** At ~60 g for a 290 g drone (~21% mass fraction, E), their protection per gram is in the same band as GimBall's 22% cage. They are not obviously better than a rod truss.

### Gaps
- No measured thrust loss for wire mesh or net cages on 3 to 5 inch props was found. A dedicated bench test (thrust stand, mesh at 5 to 15 mm from the disc) would be needed.
- No durability or entanglement data for nets or fabric on drones was found.
- The PMC copy of the Sensors kirigami paper was blocked by a captcha. The airflow-test results by hole pattern (Models 1 to 5) were not extracted.

## 2. Tensioned-wire / string (nylon, Dyneema, Kevlar) cages and tensegrity shells that spread load to stiff points

### Takeaway
Tensegrity shells, made of carbon rods held only by tensioned braided line, have the best published survivable impact speeds of any drone cage. They reached **7.8 m/s in-flight wall hits and an 11.7 m/s drop**. The cost is a ~32% structure mass fraction, and the shell is 20 cm or larger. String failure is the dominant failure mode. Tensegrity also gives near-spherical geometry that helps (active) re-orientation on the ground.

### Cited Findings
- **Zha et al. 2020 (IROS), first icosahedron tensegrity quad:**
  - Build: **252 g total (P)**, 6 carbon rods of 20 cm, braided high-tensile fishing line.
  - Result: survived collisions at **up to 6.5 m/s (P)**, and can re-orient from upside-down on the ground and resume flight (P) — [arXiv 2003.03417](https://ar5iv.labs.arxiv.org/html/2003.03417)
- **Zha et al. 2022/2023, model-based tensegrity aerial vehicle (Berkeley HiPeRLab):**
  - Mass: **300 g total, 95 g shell (P)**, i.e. ~32% (E). Batteries are 75 g, electronics 50 g, motors 80 g.
  - Rods: 6 mm OD carbon. Strings: braided nylon.
  - Drops: a **7 m drop (11.7 m/s landing) survived (P)**. A **10.5 m drop (14.4 m/s) failed when a string snapped (P)**.
  - Comparison: a 250 g prop-guarded quad fractured after a **3.25 m drop (8.0 m/s) (P)**.
  - In flight: an **in-flight wall hit at 7.8 m/s was survived and the vehicle flew again (P)**. It also hit a tree in a forest test and kept going.
  - Thrust-to-weight is ~3:1 (P). Re-orientation from any face was shown at friction mu = 0.2, using bi-directional ESCs (P).
  - Reported failure modes (P): string rupture, IMU saturation on hard hits, and re-orientation torque limits on steep slopes — [HiPeRLab PDF](https://hiperlab.berkeley.edu/wp-content/uploads/2023/06/TensegrityAerialVehicle.pdf)
- **Same group, simulation (2,000 random 5 m/s wall impacts):** mean peak stress was **34.4 MPa for the tensegrity vs 100.5 MPa for a prop guard (P, simulated)**. The tensegrity peak was under half the guard's in 80% of cases. Stress was worst when the rods carrying the quad nodes hit head-on. Simulation parameters: carbon rods, braided nylon at 20 N pretension, a 50 g structure on a 250 g quad, 2.5" props — [arXiv 2211.12045](https://ar5iv.labs.arxiv.org/html/2211.12045)
- **NASA Ames tensegrity UAV study (Bjorn and Agogino):**
  - Configurations: 1.5 kg and 3 kg (0.5 m and 1 m rods), 24 simulated configurations.
  - Findings were qualitative only: the frames take "severe deformations without permanent damage", and small stiff-cabled tensegrities behave like rigid quads — [NASA NTRS 20210013185](https://ntrs.nasa.gov/api/citations/20210013185/downloads/BjornAgoginoTensUAV.pdf)
- **Zappetti et al. 2022 (Advanced Intelligent Systems), dual-stiffness tensegrity:**
  - Struts stay rigid in normal use, then buckle without breaking. Load-bearing ratio before vs after buckling is **up to 10.5 per strut and 5.74 per structure (P)**.
  - Integrated into a drone and a rover, which survived **2 m and 5 m falls (P)** — [Advanced Intelligent Systems, DOAJ](https://doaj.org/article/66224449d7834e188975f64c2ad42fc3)
- **Tribar tensegrity robot (Yale, arXiv 2501.15078):**
  - Build: 0.4 kg, 360 mm rods of 1/8 in (3.175 mm) carbon, Power Pro fishing-line cables, silicone end-cap covers.
  - Drop survival: **5.7 m (P)**.
  - Published tensegrity drop-survival table (P): tensegrity drone (94 mm bars) **2 m**, tensegrity rover 5 m, NASA SUPERball (2 m bars) 3.4 m — [arXiv 2501.15078](https://arxiv.org/pdf/2501.15078)
- **Yale critical-drop-height model, 2026:**
  - Aluminum-rod Tribar yields at **0.5 m (P)**. Titanium rods raise this to ~1.5 m (P).
  - Measured peak impact force was **618 N with ~2.4 ms contact (P)**.
  - Fishing-line stiffness was **6.67 N/mm (P)**.
  - The electronics enclosure alone survived 5 m (P), so the rods and end caps, not the electronics, set the limit — [Yale Faboratory, Cimatti 2026](https://www.eng.yale.edu/faboratory/publications/conference/2026/Cimatti%20-%202026%20-%20Modeling%20the%20critical%20drop%20height%20of%20tensegrity%20robots.pdf)
- **Euler-spring cage (Klaptocz et al., IROS 2013, EPFL):**
  - Geometry: 24 pultruded carbon rods, **2 mm diameter and ~20 cm long (P)**, used as post-buckling springs, arranged as 8 tetrahedra with 3 rods each and tied with nylon string.
  - Mass: the protection weighs **40 g on a 282 g robot, ~14% (P)**.
  - Energy: one rod absorbs **1.5 J at 0.34 compression (P)**, equal to a 51 cm free fall of 300 g.
  - Force profile: the buckling rods gave a nearly flat force vs drop height up to 75 cm. A stiff frame alone gave high force even at 10 cm, and foam transferred much more force at high drops (P).
  - Flights: in **50 wall-strike flights, 4 caused damage (8%) (P)**. Two of those were soft rotors flexing into each other, and **two were nylon string failures (P)** — [EPFL Infoscience, Euler Spring Collision Protection (IROS 2013)](https://infoscience.epfl.ch/record/187769/files/cage-IROS.pdf?version=1)
- **String-spoke analogy outside drones:** Berd bicycle wheels use UHMWPE (Dyneema) string spokes in place of steel, carrying load in pure tension — [Dyneema, reinventing the wheel with Berd spokes](https://www.dyneema.com/sectors/sports-and-consumer-products/reinventing-the-wheel-with-berd-spokes)

### Inferences
- **The Euler-spring cage is the closest analogue to Scout's existing rod cage, and it already beats a rigid truss per gram.**
  - Same rods, different load path: rods loaded axially to buckle elastically, as a constant-force limiter, instead of rigid TPU-noded joints that pass a load spike.
  - Scaling from 2 mm to 3 mm rods at the same length: buckling force scales with d^4, so ~5x (E). Energy per rod at the same compression ratio also rises ~5x (E), to about 7 J per rod at 20 cm (E).
  - Scout's 3 mm rods are shorter (~10 to 17 cm, E). Shorter rods buckle at higher force (about 1/L^2) but absorb less energy per rod. This needs sizing so the plateau force stays below what the TPU nodes and electronics tolerate.
- **Weak link.** Across tensegrity and Euler cages, braided nylon string rupture is the common failure mode. Dyneema/UHMWPE or Kevlar line (much higher strength per diameter, low creep for Kevlar) would likely remove that weak point. This is not tested on drones in these sources (E).
- **Pure tensegrity vs. Scout's constraints.**
  - Size: published shells are 20 cm rods on 250 to 300 g vehicles. Scout at 520 g with 17 cm span would need a shell around 22 to 25 cm (E).
  - Dock fit: the shell's compliance makes position in a funnel dock less repeatable, which works against the "slides into funnel on pure geometry" requirement (E).
  - Mass: at ~32% structure fraction, Scout's shell would be ~165 g (E), probably heavier than the current cage.
- **Hybrid that fits Scout best (E).** Keep a stiff inner ring or plate as the dock-reference datum and the electronics bay. Hang a string-tensioned outer net or rod layer off it, so outer hits are spread to a few stiff nodes. This is the "load to a few stiff points" idea, and it is consistent with the Klaptocz and Zha results.

### Gaps
- No drone paper using Dyneema/Kevlar strings (as opposed to nylon/fishing line) was found.
- No tensegrity drone sized for 500+ g with measured mass fraction was found.
- No data on tensegrity shells docking repeatably in a funnel was found.

## 3. Origami / kirigami foldable or energy-absorbing drone structures: tested impact data

### Takeaway
Origami protection is well published, but impact numbers are modest. Tested speeds are mostly 1.5 to 2 m/s for folding mechanisms. The best quantified data is the 2023 kirigami PEEK guard (35 to 78% peak-force cut on a 290 g mass). EPFL's dual-stiffness origami works as a rigid-until-threshold arm, but no numeric impact data was obtainable from public pages.

### Cited Findings
- **Rotorigami (Sareh, Chermprayong, Emmanuelli, Nadeem, Kovac, Science Robotics 3(22):eaah5228, 2018):**
  - Build: a Miura-ori-variant folded plastic sheet on a rotating inner frame (P).
  - Claims: reduces impact force and stops post-collision spinning (P, qualitative).
  - The survey reports safe interaction at **~2 m/s (P)**. No mass is given publicly — [Imperial news](https://www.imperial.ac.uk/news/188697/origami-cushions-protect-drones-from-collisions/); [arXiv 2212.03196](https://arxiv.org/pdf/2212.03196)
- **Shu and Chirarattananon 2019 (RA-L), origami folding airframe:**
  - Mass: **51.2 g total, 14.9 g airframe (~29%) (P)**.
  - Build: laminated 300 um fiberglass tiles, 25 um Kapton hinges and PSA. Thrust holds the arms open against a 110 deg stop. An impact above the thrust torque folds the arms around the body in **under 0.15 to 0.2 s (P)**.
  - Tests: a mid-air hit at **~1.5 m/s** left the structure intact (P). Fold work measured **0.5 to 1 mJ (P)**.
  - Limits: does **not** protect against top or bottom collisions (P), and no self-righting — [arXiv 1907.07056](https://arxiv.org/pdf/1907.07056)
- **Mintchev, Shintake, Floreano 2018, "Bioinspired dual-stiffness origami" (Science Robotics, doi 10.1126/scirobotics.aau0275):**
  - Mechanism: a prestretched elastomer membrane sandwiched between rigid tiles. The structure is stiff until a designed moment, then the plates separate and the arm bends. It springs back after impact and stores folding energy for self-deploy (P).
  - Arm retention: two magnet couples per arm (P).
  - Spec page: mass **30** and size **160 x 160 x 30**, units not stated (likely g and mm, E) — [EPFL news](https://actu.epfl.ch/news/an-insect-inspired-drone-deforms-upon-impact/); [Wevolver spec page](https://www.wevolver.com/specs/bioinspired.dual-stiffness.origami.drone)
- **Kornatowski, Mintchev, Floreano 2017 (IROS), origami cargo drone:** the cage shields the props in flight and folds to an **8% storage volume (92% reduction) (P)** — [arXiv 2212.03196](https://arxiv.org/pdf/2212.03196)
- **Other entries from the survey:**
  - MiniCore (Dilaveroglu and Ozcan, RoboSoft 2020): foldable, safe at ~1 m/s (P).
  - Miura-oRing (Sareh 2018): spinning cyclic Miura-ori, no numbers.
  - Source: [arXiv 2212.03196](https://arxiv.org/pdf/2212.03196)
- **Kirigami PEEK guard (Sensors 2023):** see section 1. 60.7 g guard, 290 g mass, 600 mm drop, peak force down 35.7 to 78.2% (P) — [MDPI Sensors 23(4):2150](https://www.mdpi.com/1424-8220/23/4/2150)

### Inferences
- **Folding-arm origami (Shu, Mintchev 2018) does not fit Scout.** It protects only side hits, at ~1.5 to 2 m/s, on sub-60 g vehicles. Scout has props on top of the arms and needs top and bottom protection. Folding also conflicts with the rigid dock geometry.
- **The transferable idea is the dual-stiffness joint.** A preloaded elastomer (TPU) or membrane between rigid plates is rigid up to a set moment, then gives. A printed TPU node could do the same if designed as a preloaded "snap-through" or plate-separation joint rather than a uniformly soft blob (E).
- **Kirigami or Miura-ori film bumpers (PETG or polycarbonate sheet)** are a credible light bumper ring at the props' plane. This is what Rotorigami and the Sensors 2023 paper show. Durability over repeated hits is unreported.

### Gaps
- Full Science Robotics papers (Rotorigami, dual-stiffness origami) are paywalled. Masses, threshold moments and impact speeds could not be verified.
- No origami or kirigami structure for drones was found tested above ~2 m/s in flight.

## 4. Insect-wing / exoskeleton-inspired dual-stiffness or breakaway-and-reset joints; magnetic breakaway mounts

### Takeaway
This family has the strongest directly transferable evidence for Scout. CogniFly (EPFL/Polytechnique, 2021) uses **3 mm carbon rods plus TPU 95A flexible joints**, nearly Scout's exact materials. At 241 g and 21 x 21 x 12 cm it survived ~7 m/s impacts and a 2.62 m drop. Its 150 cm drop peak acceleration was at least 5x lower than a rigid ABS frame dropped from only 30 cm. Magnetic breakaway joints (Mintchev 2017, X-Morf 2017) survive 50+ crashes and self-reset with elastic bands.

### Cited Findings
- **CogniFly, exoskeleton-inspired soft quad (de Azambuja et al., arXiv 2103.04423):**
  - Build: **3 mm carbon fiber rods, 3D-printed TPU 95A joints and motor mounts**, rigid ABS/PLA central body. **241 g (P)** vs 239 g for a rigid ABS frame. **210 x 210 x 120 mm (P)**.
  - Impacts: survived **~7 m/s (P)** and a **262 cm drop with no damage (P)**. The frame bent and the drone bounced back airborne.
  - Drop data: median peak acceleration from **150 cm was at least 5x lower than the rigid ABS frame from 30 cm (P)**. Their mass-spring-damper model predicts payload-to-ground contact at drops of **150 cm and above (P)**.
  - Durability: only 2 propeller sets in ~1 year of crashing (P). It can bounce off walls and continue without landing (P) — [arXiv 2103.04423](https://arxiv.org/abs/2103.04423)
- **Mintchev, de Rivaz, Floreano 2017 (RA-L), "Insect-inspired mechanical resilience for multicopters" (wasp-wing resilin joint analogue):**
  - Build: **0.3 mm fiberglass frame (P)** joined to the central case by 4 magnetic joints, each "two magnets and a spring" (P). Rigid in flight, it detaches on impact and elastic bands pull it back so the magnets reconnect (P).
  - Mass: magnets, foam and elastics total **4.66 g, ~9% of robot mass (P)**, implying ~52 g total (E).
  - Tests: **dropped over 50 times from 2 m with no permanent damage (P)**. Flight performance is equivalent to a rigid frame (P) — [NCCR Robotics news PDF](https://nccr-robotics.ch/wp-content/uploads/2020/11/32-Insect-Inspired-Mechanical-resilience-for-multicopters.pdf); [IEEE Spectrum](https://spectrum.ieee.org/epfl-bioinspired-hard-to-destroy-drone); [arXiv 2212.03196](https://arxiv.org/pdf/2212.03196)
- **X-Morf (Desbiez et al., RED-UAS 2017):** arms joined to the body by magnetic and electrical joints so it separates on crash. Arm span changes **up to 28% in 0.5 s (P)**. Prototype **380 g (P)** — [arXiv 2212.03196](https://arxiv.org/pdf/2212.03196); [HAL](https://hal-amu.archives-ouvertes.fr/hal-01644528v1)
- **Other compliant designs:**
  - Liu and Karydis 2021 (ICRA): passive compliant arm, survived **5.9 m/s (P)**.
  - Patnaik et al. 2021 (AIM): torsional-spring quad, **2.5 m/s (P)**.
  - Source: [arXiv 2212.03196](https://arxiv.org/pdf/2212.03196)
- **Ring-shaped soft quadrotor, no central body (arXiv 2511.05426):**
  - Build: **405 g (P)**, 216 mm diameter, four FR-4 glass-fiber strips of 0.5 mm, 4" props, k = 0.1 N/mm.
  - Frontal wall hits: **3 and 4.5 m/s** with no damage. Control-unit peak at **25.2 g at 3 m/s (P)**. The abstract claims 5 m/s, but the body text tests only up to 4.5 m/s.
  - Glancing prop-level hits at 5 m/s vs a same-mass rigid carbon quad: peak loads **reduced up to 39x, destabilizing torques ~15x (P)**. Impact duration ~10x longer (P).
  - Squeeze trade-off: thrust efficiency fell from 79% to 68 to 73% when squeezed (P) — [arXiv 2511.05426](https://arxiv.org/pdf/2511.05426)
- **Tombo propeller (dragonfly-nodus-inspired, arXiv 2202.07177):**
  - Build: silicone nodus with nylon tendons on a 9" prop.
  - Thrust: **5.6% lower thrust at 2000 rpm (P)**.
  - Impact: mean peak blade-impact force **127 N vs 269 N rigid (~53% lower) (P)**, with the best configurations at ~80 N. Blade recovers in ~0.46 s.
  - Without bounce-back control the drone still crashed (P) — [arXiv 2202.07177](https://arxiv.org/pdf/2202.07177)
- **Biology behind these designs:**
  - Resilin joints in honeybee wings (Ma et al. 2015, J Exp Biol), wasp costal break (Mountcastle and Combes 2014), beetle hindwing folds (Phan and Park 2020).
  - Source: cited in [arXiv 2212.03196](https://arxiv.org/pdf/2212.03196)

### Inferences
- **CogniFly is nearly Scout's own architecture** (3 mm CF rods plus TPU 95A nodes, ~21 cm box, 120 mm tall), validated to ~7 m/s at 241 g. Scout is ~2x heavier at 520 g, so impact energy at the same speed is ~2x (E).
  - Joint compliance would need retuning. Stiffer TPU (98A) or larger node cross-sections are the likely route (E).
  - A worked check: at the same peak deceleration and twice the mass, the required stroke at a given speed is unchanged. But the force at the nodes doubles (E).
  - So the existing Scout cage is already an "exoskeleton" design. Its performance depends on whether the TPU nodes are tuned to act as designed compliant hinges, and on whether payload-to-ground contact (the ~150 cm threshold in CogniFly) is prevented (E).
- **Magnetic breakaway on an outer bumper ring** is light (~9% mass in Mintchev 2017) and resets passively with elastics. Two cautions:
  - Magnets near the FC compass or IMU may interfere (E).
  - A detached ring in a funnel dock is a failure risk. The elastic return must fully re-seat before docking (E).
  - It counts as "no moving parts" only loosely: it is passive, but the parts are separable.

### Gaps
- No CogniFly per-node stiffness or TPU node geometry numbers were extracted. The full arXiv 2107.11090 and its repository may have them.
- No magnetic breakaway joint was found tested above ~2 m drops or on vehicles over 400 g.

## 5. Bio-inspired shells: Voronoi/radiolarian geodesic, beetle elytra sandwich, bird-bone lattices, pangolin scales

### Takeaway
Very little built or tested evidence exists of these shell types on drones or robots for impact. Beetle-elytra work on drones is about protective, retractable wings and self-righting for fixed-wing aircraft, not sandwich panels. Elytra-plate and Voronoi lattices are characterized as materials in compression only. No drone impact tests were found.

### Cited Findings
- **Robotic Elytra (Vourtsis, Stewart, Floreano, RA-L 2021):** retractable beetle-inspired protective wings that cover folding main wings, with "increased lift generation and stall delay at high angles of attack" (P, abstract, no numbers) — [EPFL Infoscience 290070](https://infoscience.epfl.ch/record/290070)
- **Insect-inspired self-righting for fixed-wing drones (Vourtsis, Casas, Serrano et al., RA-L 2021):** elytra add lift in flight and right the drone when it lands upside-down (P, no numbers in abstract) — [EPFL Infoscience 287482](https://infoscience.epfl.ch/record/287482?ln=en)
- **Beetle elytron plate (BEP) cores:** there is literature on 3D-printed grid BEP shear and compression properties, e.g. Springer Sci China Tech Sci 2019 and J Mater Sci 2020, and a Bristol thesis on elytra-inspired cellular sandwich cores. None of these are applied to drones — [Springer s11431-019-1470-5](https://link.springer.com/doi/10.1007/s11431-019-1470-5); [Bristol thesis](https://research-information.bris.ac.uk/en/studentTheses/beetle-elytra-inspired-cellular-cores-for-sandwich-structures/)
- **Pangolin-scale armor:** found only as stretchable armor for wearables and soft robotics (Ajou University / DGIST). No drone application was found — [DGIST, Artificial stretchable armor](https://dgist.elsevierpure.com/en/publications/artificial-stretchable-armor-for-skin-interfaced-wearable-devices/)
- **Single-tow architected composite drone frames (arXiv 2509.09024):** lattice composite frames for low mass. This is not impact data — [arXiv 2509.09024](https://arxiv.org/pdf/2509.09024)

### Inferences
- **Geodesic or Voronoi shells.** Their main value for Scout is geometric: a near-spherical or rounded envelope gives self-righting and dock-guiding surfaces, and the strut network spreads load. A geodesic rod cage is the same structure as a truss. A Voronoi printed PETG shell would likely be heavier per joule than buckling-rod or tensegrity options (E, no data).
- **Elytra or sandwich panels.** These make sense only as a stiff, local electronics lid (top cover), not as the whole cage (E).

### Gaps
- No drone or robot impact test of radiolarian, diatom, Voronoi, bird-bone or pangolin structures was found.
- No mass or energy data for printed elytra-core panels at drone scale was found.

## 6. Which beat a rigid rod truss per gram, and which help passive self-righting?

### Takeaway
Per gram, the measured winners over a rigid frame or prop guard are:
- tensegrity, ~32% mass fraction, 7.8 m/s wall and 11.7 m/s drop;
- Euler-spring buckling rods, 14% mass, flat force plateau up to 75 cm drops;
- exoskeleton CF rod plus TPU hinge, ~7 m/s and 5x lower g than rigid at similar mass;
- magnetic dual-stiffness, ~9% mass, 50+ 2 m drops.

Passive self-righting in the literature comes from geometry: a low center of gravity plus a top protrusion or rounded shell, shown on ground robots and in patents. No published passive-self-righting cage on a quadrotor with measured results was found.

### Cited Findings
- **Mass fractions and speeds (P unless marked):**
  - Mulgaonkar 2015/2017: 3 g frame on 25 g, ~12% (E), 4 m/s.
  - Euler cage: 40 g on 282 g, 14%, about 1 m drops.
  - GimBall: 84 g on 385 g, ~22% (E).
  - Shu origami: 14.9 g on 51.2 g, 29%, 1.5 m/s.
  - Tensegrity: 95 g on 300 g, ~32% (E), 7.8 m/s.
  - Sources: [arXiv 2212.03196](https://arxiv.org/pdf/2212.03196); [EPFL Euler](https://infoscience.epfl.ch/record/187769/files/cage-IROS.pdf?version=1); [HiPeRLab](https://hiperlab.berkeley.edu/wp-content/uploads/2023/06/TensegrityAerialVehicle.pdf); [arXiv 1907.07056](https://arxiv.org/pdf/1907.07056)
- **Prop-guarded quad vs tensegrity:** the 250 g prop-guarded quad fractured at an 8.0 m/s drop. The tensegrity survived 11.7 m/s (P) — [HiPeRLab](https://hiperlab.berkeley.edu/wp-content/uploads/2023/06/TensegrityAerialVehicle.pdf)
- **Foam vs. buckling rods vs. stiff frame:** foam passes much more force at high drops, and a stiff frame alone gives high force even at 10 cm (P) — [EPFL Euler-spring cage](https://infoscience.epfl.ch/record/187769/files/cage-IROS.pdf?version=1)
- **Passive self-righting shell (MediumANT hexapod, King and Revzen, ICRA 2024):**
  - Build: foam-core plates. Material above the center of mass was removed because an early solid shell raised the CoM too high to right from some faces (P).
  - Righting: **28/28 face trials self-righted (P)**, and **15/15 1 m ledge falls** righted and resumed (P). Maximum self-righting incline ~16 deg (P).
  - Failure cases: curved landing surfaces and obstacles (P) — [U. Michigan BIRDS Lab PDF](https://birds.eecs.umich.edu/assets/pdf/2024-King-Revzen-ICRA.pdf)
- **Self-righting VTOL frame patent (US 2016/0375983):** intersecting elliptical or semicircular vertical hoops, a flattened bottom, low weighted mass, and a top protrusion. Landing inverted gives a single unstable contact point, so gravity rolls the vehicle upright (P) — [Justia patent 20160375983](https://patents.justia.com/patent/20160375983)
- **EPFL active uprighting leg (spring leg on a tailsitter):** an active approach, used as a contrast — [EPFL Infoscience 175011](https://infoscience.epfl.ch/record/175011?ln=fr)
- **Tensegrity re-orientation is active:** it uses motor thrust and bi-directional ESCs, not passive geometry (P) — [HiPeRLab](https://hiperlab.berkeley.edu/wp-content/uploads/2023/06/TensegrityAerialVehicle.pdf)

### Inferences
- **Recommended direction for Scout (E, simplest design that meets the requirements):**
  1. Keep the 3 mm CF rod plus TPU node architecture. It is validated as CogniFly. Design the TPU nodes as deliberate hinges or dual-stiffness joints, and consider letting long outer rods act as Euler buckling springs (axially loaded, end-pinned) instead of bending members.
  2. Add a top "righting protrusion" (patent geometry) or a rounded top hoop, plus a flat, wide bottom. Keep the CoM low (battery low), and remove cage mass above the CoM as in MediumANT. This gives passive self-righting with no moving parts.
  3. Tension a Dyneema or Kevlar line net only across large open faces outside the prop disc. It spreads plank hits to stiff nodes without choking airflow. Leave the prop inflow above clear, or use a coarse line grid with low solidity there.
- **Self-righting check (E):** with a height of 110 mm, a 170 mm span, and a CoM near the bottom plate, a single top apex will tip the drone off its back. The CoM must sit below the "rocking" circle formed by the side profile. MediumANT's 16 deg incline limit suggests the drone will not self-right on slopes steeper than ~15 deg.
- **What does not help self-righting:** fully spherical shells (GimBall, tensegrity) are neutral without active control. Folding origami arms leave the drone resting arbitrarily.

### Gaps
- No quadrotor with a measured passive (geometry-only) self-righting cage plus impact data was found.
- No head-to-head per-gram comparison under one protocol (same mass, same speed) across cage types exists. The cross-paper numbers above come from different test setups.
- Swinging-plank (lateral, non-rigid impactor) tests were not found in any source. All tests are wall, floor or drop impacts.
