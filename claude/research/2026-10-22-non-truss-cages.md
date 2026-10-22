# Keep the truss, soften it, crown it

> **Correction (22 Oct 13:30):** the apex / rounded-crown fix for passive self-righting does not work on Scout's wide, flat lantern. Checked on the v13 hull (`nox/v2/lantern/dome_probe14.py`, E): a rounded hub only moves the inverted rest; clearing every rest needs a dome to apex z ~210, roughly doubling cage height. Cage v13 keeps the blip recovery.


Non-truss cages exist, and several beat a rigid frame per gram in their own tests. None of them beats a *compliant* rod-and-node cage in a way that survives Scout's hard requirements. The families in the literature are tensegrity shells, buckling-rod "Euler spring" cages, soft glass-fibre ring frames, inflatable frames, ducts and thin shells, foam and lattice bumpers, folding origami, and gimbal or rolling cages. Every one that wins does it through compliance and stroke, not topology. The closest proof is CogniFly: **3 mm carbon rods in TPU 95A joints, 241 g (P), surviving about 7 m/s (P)** ([arXiv 2103.04423](https://arxiv.org/abs/2103.04423)). That is nearly Scout's own construction. Tensegrity, gimbals, inflatables, magnetic breakaway and folding origami each fail on no-moving-parts, dock repeatability or passive righting. Foam and full shells fail on mass, airflow or compression set.

The direction this supports has four parts:
- Keep the 3.0 mm carbon and TPU exoskeleton.
- Put the give in TPU isolators and hinge-shaped nodes, because 3.0 mm rods are too stiff to flex usefully at Scout's spans.
- Replace any flat top with 2 to 4 thin molded glass-fibre strip hoops that meet at a small rounded apex, so Scout cannot rest upside down.
- Mount the battery as low as it goes.

The evidence is thin. No source tests a swinging object, no source compares cage types at equal mass and speed, and nothing has been tested near 520 g. The plank arithmetic below (E) puts the real design hit at about **1.7 to 2.3 J (E)**. That is 7 to 9x the 0.25 J load case in the 11 Oct rules and 3 to 5x the lantern's 0.5 J. Three cheap physical tests matter more than any further reading.

## A swinging plank delivers about 2 J, then throws Scout into the wall

No paper or product has tested a moving impactor. Every result in these notes is a drone flying into a wall or dropping onto a floor, so the plank case has to be built from first principles (all E):
- **The plank.** A 2.4 m 2x4 weighs about **4 kg (E)**. Swung from one end, its tip acts like a mass of about **1.35 kg (E)** (a third of the plank).
- **Free plank, 3 m/s tip.** Against a free-flying 0.52 kg Scout, the cage must absorb up to **1.7 J (E)**. Scout leaves at **2.2 m/s (E)** if the cage fully damps the hit, and up to **4.3 m/s (E)** if it bounces elastically.
- **Plank held by a worker.** The bounding case is effectively infinite mass: **2.34 J (E)** absorbed and up to **6 m/s (E)** exit.

That changes what matters. The plank strike itself is a 2 J problem. The second impact, Scout hitting a wall or the floor at 2 to 6 m/s, is the 1 to 9 J problem (E). A cage that dissipates energy cuts both. A cage that stores and returns it, such as a bare Euler-spring rod or a springy glass-fibre hoop, hands the energy back as launch speed. This is where the Bilkent result matters most. Across 1200+ collisions, compliant bumpers made hits more elastic but did not increase dissipation. A softer inner frame raised damping and lowered accelerations ([Abazari et al. 2024](https://repository.bilkent.edu.tr/items/1033b7aa-8f57-4739-8f5a-b596f5078dad/full)). TPU is the damper in Scout's material set, and it belongs between cage and core.

A plank edge is also a "sharp object". Flyability halves Elios 2's rating for sharp objects, from **3 m/s flat to 1.5 m/s sharp (P)** ([Elios 2 spec](https://www.xboom.in/wp-content/uploads/2025/02/Copy-of-Eliso-2_compressed-1.pdf)), and no commercial cage claims sharp-edge tolerance above 1.5 m/s. Stroke sets the peak g. Stopping 3 m/s averages **about 30 g over 15 mm and about 150 g over 3 mm (E)**. For comparison, FlexiQuad's soft ring measured **25.2 g at 3 m/s (P)** ([arXiv 2511.05426](https://arxiv.org/html/2511.05426v1)).

**Against the earlier notes.** This overturns the 1 m/s, 0.25 J load case from the 11 Oct report as the design hit, if the plank requirement is real. The 10 to 15 mm stroke rule still holds, and so does the design-outcome note's reversal of "no suspension".

## Compliance and stroke explain every winner, not the shape

Reading across families, the result is the same each time: the structures that cut peak force are the ones that move further.
- **Soft ring.** The FlexiQuad ring is four 0.5 mm glass-fibre strips at **0.1 N/mm (P)**. It stretches impact time **about 10x (P)** and cuts glancing peak loads **up to 39x (P)** against a same-mass rigid quad ([arXiv 2511.05426](https://arxiv.org/html/2511.05426v1)).
- **Inflatable frame.** SoBAR's frame weighs **10 g against a 120 g F450 frame (P)**. Peak force drops from **430 to 449 N rigid to 53 to 133 N soft (P)** at 2.2 to 3.1 m/s ([arXiv 2204.13155](https://arxiv.org/pdf/2204.13155)).
- **Compliant arm hinges.** HoLoArm's TPU hinges give only **about 30% lower peak force (P)**, and its rigid frame broke at the 3 m drop ([arXiv 2605.25790](https://arxiv.org/pdf/2605.25790)).
- **Tensegrity.** It has the highest survival speeds: **7.8 m/s wall and 11.7 m/s drop on 300 g (P)**. But it pays a **95 g shell, 32% of mass (P)** ([HiPeRLab](https://hiperlab.berkeley.edu/wp-content/uploads/2023/06/TensegrityAerialVehicle.pdf)). It is sized for 6 m/s, roughly 4 to 9x Scout's energy per kg at 2 to 3 m/s (E).
- **Euler-spring cage.** It holds the per-gram record for a rod structure: **40 g on 282 g, 14% (P), about 1.5 J per 2 mm rod (P)**, and nearly flat transmitted force up to 75 cm drops ([Klaptocz et al.](https://infoscience.epfl.ch/record/187769/files/cage-IROS.pdf?version=1)).

The per-gram claims are weak in one specific way. The only equal-mass comparisons anywhere are two:
- Foam against Euler-spring rods at 40 g: similar at low drops, foam worse at high ones ([Klaptocz et al.](https://infoscience.epfl.ch/record/187769/files/cage-IROS.pdf?version=1)).
- Tensegrity against a prop guard in simulation at 50 g: **34.4 against 100.5 MPa mean peak stress (P)** ([arXiv 2211.12045](https://arxiv.org/pdf/2211.12045)).

CogniFly's "five-fold" gain is a ratio of drop heights (150 cm against 30 cm), not a measured g ratio ([arXiv 2107.11090](https://arxiv.org/pdf/2107.11090)). Every other cross-design number comes from a different rig. Specific energy absorption does not favour leaving rods either:
- EPP foam at 30 g/L reaches about **2 to 3 J/g (E, from datasheet)** ([ARPRO data](https://arplankdirect.com/wp-content/uploads/2024/12/PhysicalPropertyInformation_EPPproducts_ARPLANK_10-2024-R1.pdf)).
- FDM TPU lattices measure **below 1 J/g (P)** ([Polymers 2025](https://www.mdpi.com/2073-4360/17/8/1111)).
- A buckling carbon rod gives about **1.6 J/g (E)**.

Foam wins per gram but loses on coverage. A 20 mm EPP skin over Scout's box would weigh about **78 g (E)**, block the props and take **about 20% compression set after a 50% crush (P)**. That set alone breaks a "fits like a glove" funnel.

The failure hierarchy agrees with the 19 Oct lantern report and extends it. Printed nodes, glue lines and ties break, and carbon rarely does. In tensegrity and Euler cages the first failure is **nylon string rupture (P)** ([HiPeRLab](https://hiperlab.berkeley.edu/wp-content/uploads/2023/06/TensegrityAerialVehicle.pdf); [Klaptocz et al.](https://infoscience.epfl.ch/record/187769/files/cage-IROS.pdf?version=1)). The 19 Oct conclusion that "nobody has shown a sphere beats the lantern at equal mass" still stands, and it now extends to every non-truss family.

## Ranking the non-truss families against Scout's rules

| Rank | Concept | Best evidence (P) | Verdict for Scout | Why |
|---|---|---|---|---|
| 1 | **Compliant exoskeleton: CF rods + TPU hinge nodes + TPU cage-to-core isolators** | CogniFly 241 g, ~7 m/s, 262 cm drop ([arXiv 2103.04423](https://arxiv.org/abs/2103.04423)); Elios 2/3 TPE suspensions ([Elios 3 spec](https://nexxis.com/wp-content/uploads/sites/3/2022/06/20220519_E3_Tech_Spec_web_version.pdf)) | **Keep and tune** | Same materials as Scout. Stroke and damping sit where Bilkent says they must. TPU recovers up to 99.5% ([Polymers 2025](https://www.mdpi.com/2073-4360/17/8/1111)), so the resting shape stays repeatable. |
| 2 | **Molded glass-fibre strip hoops** (FlexiQuad ring material in a Yan-Dees hoop layout) | 0.5 mm GF strips, 0.1 N/mm, 25.2 g at 3 m/s on 405 g ([arXiv 2511.05426](https://arxiv.org/html/2511.05426v1)); hoop righting frames ([US 8528854](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/8528854)) | **Complement: top crown and righting apex** | Gives the compliance 3.0 mm carbon cannot, plus a smooth convex shell for the dock. Fits Spark's composites skills. Unproven as a cage, and elastic, so it needs TPU damping behind it. |
| 3 | **Local TPU bumpers or a slick sleeve** on equator rods and plank-exposed edges | Flyability elastomer sleeve 5 to 15% of diameter, claimed +95 to >300% strain energy ([US 2020/0189712](https://patents.justia.com/patent/20200189712)) | **Complement** | A few grams. Fixes the sharp-edge penalty. A slick skin also captures part of what a gimbal buys, since Gimball's benefit vanished on near-frictionless contact ([Briod 2014](https://infoscience.epfl.ch/record/190790/files/collision_resilient_flying_robots_LIS2014.pdf)). |
| 4 | **Dyneema/Kevlar line across large open faces** outside prop discs | No drone data. Strings are the proven weak link only in nylon ([HiPeRLab](https://hiperlab.berkeley.edu/wp-content/uploads/2023/06/TensegrityAerialVehicle.pdf)) | **Optional, untested** | Could spread a plank edge to stiff nodes at near-zero mass. Snag and entanglement risk near props is unquantified. |
| 5 | **Duct ring at the prop plane** | Ducts gain 4 to 25% thrust only at 0.25 to 0.72% tip clearance ([Zhao & Shkarayev](https://journals.sagepub.com/doi/10.1177/1756829319837661)) | **Weak fit** | At about 1.7% clearance (E) the gain is roughly 0 to 10% (E). It protects the prop plane only, and a plank comes from above too. |
| Out | Tensegrity shell | 32% mass, active righting with bidirectional ESCs ([HiPeRLab](https://hiperlab.berkeley.edu/wp-content/uploads/2023/06/TensegrityAerialVehicle.pdf)) | Ruled out | About **165 g (E)** at Scout's mass. The shape shifts under load, which breaks the dock. A near-sphere is neutral without active rolling. |
| Out | Gimbal or rolling cage | Gimball gimbal 56 g of 385 g ([Briod 2014](https://infoscience.epfl.ch/record/190790/files/collision_resilient_flying_robots_LIS2014.pdf)); Rotorigami rotating ring ([Imperial](https://www.imperial.ac.uk/news/188697/origami-cushions-protect-drones-from-collisions/)) | Ruled out | Moving parts. About **100 to 120 g (E)**. Arrives at the dock in a random orientation. |
| Out | Inflatable frame or airbag | SoBAR, 69 kPa arms split ([arXiv 2204.13155](https://arxiv.org/pdf/2204.13155)) | Ruled out | Puncture on a construction site, geometry drifts with pressure, pump or leak management. |
| Out | Magnetic breakaway, folding origami arms | Mintchev 9% mass, 50+ 2 m drops ([NCCR](https://nccr-robotics.ch/wp-content/uploads/2020/11/32-Insect-Inspired-Mechanical-resilience-for-multicopters.pdf)); Shu origami side hits only at 1.5 m/s ([arXiv 1907.07056](https://arxiv.org/pdf/1907.07056)) | Ruled out | Separable or moving parts. Geometry is not guaranteed at the dock. Magnets sit near the compass. Only tested at 50 to 400 g. |
| Out | Full foam shell, thin solid dome, Voronoi/elytra shells | No drone impact data for shells (0.5 D rule: [Caltech AIAA 2021-1648](https://authors.library.caltech.edu/records/zqdwv-3c566)) | Ruled out | A solid lid must sit about 45 mm (0.5 D, E) above the props, which blows the 110 mm height. Foam takes a set. Bio-shells are untested. |

The dual-stiffness idea from origami, a preloaded joint that stays rigid up to a set moment and then gives, survives as a node design rule, not as a concept ([Mintchev 2018 via EPFL](https://actu.epfl.ch/news/an-insect-inspired-drone-deforms-upon-impact/)).

## 3.0 mm carbon is a stiff member, so the give must come from somewhere else

The notes confirm the 11 Oct finding: 3.0 mm rods cannot act as springs at Scout's scale. With pinned ends and E = **113 GPa (P, EPFL measured)**, a 3.0 mm rod buckles at:
- **about 440 N at 100 mm**
- **about 230 N at 139 mm (E)**

139 mm is Klaptocz's optimum length, L = √(8/3) × platform radius, at an 85 mm radius ([Klaptocz et al.](https://infoscience.epfl.ch/record/187769/files/cage-IROS.pdf?version=1)). Two or three rods share a hit, so the plateau lands at **460 to 690 N, or 90 to 135 g on Scout (E)**. A 50 g ceiling allows about **255 N total (E)**.

Bending into a hoop is worse. Surface strain is r/R, so an 85 mm-radius hoop strains a 3.0 mm carbon rod to **1.8% (E)**, past the **1.34% compressive failure strain Bristol measured (P)** ([Bristol](https://research-information.bris.ac.uk/en/publications/the-experimental-investigation-of-the-compressive-performance-of-/)). Carbon at this diameter cannot be cold-bent into a Scout-sized hoop at all. Gentle bows of about R 300 mm or more keep preload strain near **0.5% (E)**, but they add little give.

So the trade-off has two branches.

**Change the rod (shared number, Spark's sign-off).** Two variants:
- **2.0 mm carbon.** Buckles at about **45 N per rod at 139 mm (E)**, which makes a working Euler cage. But it bends **about 18 mm (E)** under a mid-span hit at 155 mm, through the prop keep-out, as the 11 Oct span table showed. It only works where geometry loads the rod along its axis.
- **3.0 mm glass-fibre rod.** Keeps every socket and node unchanged. It is about **3x more compliant and 32% heavier (P)**, bends to a **50 to 60 mm radius (E)** ([Glasforms](https://electronics.polyone.com/es/node/10713); [R&G](https://r-g.de/en/art/610001)), and buckles near **77 N per rod at 139 mm (E)**. That puts three rods right at the 255 N line. It is the cleanest rod-level change if one is ever needed, and could be limited to the crown.

**Keep 3.0 mm carbon and add compliance elsewhere.** Four options:
- **TPU isolators between cage and core.** These already exist on S2 cage v1 at about 6.4 N/mm per holder strut (C). They give the most stroke per gram, and they are the damper.
- **Hinge-shaped TPU nodes** in place of solid blocks.
- **Composite strip hoops** for the top. Their stiffness scales as w·t³, so it can be tuned toward FlexiQuad's proposed **0.006 to 0.77 N/mm band (P)**, which was derived for a 405 g vehicle.
- **Longer spans.** This lever is mostly spent, because a 200 mm span bows about 25 mm against 12.5 mm of clearance (lantern report, E).

On the dock worry the 19 Oct report left open: an elastomer mount is not a moving part. It returns to one geometry. Its sag under Scout's own weight on four 6.4 N/mm isolators is about **0.2 mm (E)**. The real risk is creep and compression set, which only test 3 below can settle. **DEFAULT: keep 3.0 mm carbon project-wide.** Ask Spark about the glass-fibre rod only if test 2 shows isolators plus nodes cannot hold peak g under about 30 g at 3 m/s.

## One apex and a low battery make the inverted rest impossible

This section overturns the 19 Oct lantern verdict that passive righting was "off the table" and that recovery should rely on a motor blip or turtle firmware. Turtle mode is now forbidden, and the shape fix is cheap. Self-righting is only hard for uniform bodies like the gömböc. With an off-centre mass, a sphere goes monostable with about **5% bottom ballast (P, simulation only)**. Cubes and cylinders never did, even at **30% (P)** ([Couey, arXiv 2604.17095](https://arxiv.org/pdf/2604.17095)).

The rule, in 2D, applies to both the front and side views of the full hull (cage plus prop tips), with G the centre of mass:
- A corner or node is never a stable rest.
- A flat face is stable only if the perpendicular from G lands inside it, and only the bottom face may pass that test.
- A curved arc of radius ρ is stable if G is closer to the contact than ρ. So every arc except the bottom needs its centre of curvature on the far side of G, and for the top dome that means above G.
- On a faceted cage, every top or side facet that catches G's perpendicular foot is a turtle trap. This is why the lantern's flat hub rests inverted.

Applied to Scout:
- A single 85 mm sphere topping out at 110 mm puts its centre **25 mm above the floor (E)**. G would have to sit below that, while the notes estimate G at **35 to 50 mm (E)** for props-on-top.
- With G at about 35 mm, any top dome of radius **under about 75 mm (E)** rolls off.
- Side arcs under **85 mm (E)** radius are automatically unstable.

The cheapest form is the Yan-Dees layout ([US 8528854](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/8528854); [US 2015/0291272](https://patents.justia.com/patent/20150291272)): intersecting hoops, a flattened bottom, a low battery, and a small rounded protrusion on top. Inverted, the drone sits on a single point and tips off. The apex goes over the centre body, not the prop discs. It must stand proud of the prop tips by more than its own deflection. A sparse crown upstream of the props costs little, since EPFL measured a 7% solidity grid upstream at **0.6% lift loss (P)** ([Kornatowski et al.](https://infoscience.epfl.ch/record/277795/files/09091043.pdf?version=1)).

Two margins matter.
- **Righting moment.** It is small: m·g·e·sinθ peaks at about **0.10 N·m (E)** for e = 20 mm. A node digging into grit can hold it, so top nodes must be smooth.
- **Slope tolerance.** King and Revzen's tangent-circle rule gives atan(r/d). Their shell righted from **28/28 faces and 15/15 1 m falls, up to about 16° (P)** ([King & Revzen 2024](https://www.birds.eecs.umich.edu/assets/pdf/2024-King-Revzen-ICRA.pdf)). Removing mass above G helps.

The multi-orientation resting-height check runs on the CAD hull and the measured G:
1. For about 5,000 unit directions n, compute h(n) = max over hull vertices of (v − G)·n. This is G's height when the drone rests with face n down.
2. Count the local minima of h.
3. One minimum, at the bottom, means passively monostable. Each extra minimum is a trap, and its basin size is the risk.

This mirrors the Equilibrium Count method in Couey's study, and it is about 20 lines of Python. Static monostability does not guarantee righting after a tumble, so the physical floor test still decides.

## Conclusion: build the hoop-crowned exoskeleton, then break it three ways

The literature does not offer a better topology. It offers better places to put compliance. Keep the 3.0 mm carbon and TPU node cage as the stiff skeleton and dock reference. Make the TPU isolators and hinge-shaped nodes the deliberate stroke and damper, aiming for 10 to 15 mm of travel. Sleeve the equator rods and any edge a plank can reach. Replace the top with 2 to 4 molded 0.5 mm glass-fibre strip hoops that meet at a small rounded TPU apex over the centre body. The flat bottom stays as the funnel glove, with the battery as low as the cartridge allows. Nothing moves, the outer shell is smooth and convex for the funnel, and the rod size stays fixed.

What changes our understanding is the plank. It turns the design hit from a 0.25 to 0.5 J bump into a roughly 2 J strike followed by a 2 to 6 m/s throw (E). That favours damping over elastic rebound, and no one has measured it.

Three cheap tests would confirm or kill this direction:
1. **Rest-pose test (about 1 hour).** Find G by hanging Scout from two points. Run the h(n) script on the CAD hull. Tape a printed apex on the current cage, put the battery in, then set Scout down upside down, on each side and on each corner, on a flat floor and on a 15° board. Pass means it rolls upright from every pose.
2. **Pendulum plank strike.** Weigh a 2x4 and pivot it from one end. Release it from heights that give 2 and 3 m/s at the tip. Hit Scout hanging free on a string, isolators free and then shimmed rigid. Log peak g with a high-g accelerometer, since EPFL's ±100 g sensors saturated ([Klaptocz et al.](https://infoscience.epfl.ch/record/187769/files/cage-IROS.pdf?version=1)). Film the exit speed. Pass means under about 30 g with no prop contact (chalk the prop tips).
3. **Drop-and-refit test.** Drop from **20, 46 and 82 cm (about 2, 3 and 4 m/s, E)** onto concrete: apex down, side down, corner down, 10 times each. After each set, slide Scout into the funnel and check it seats and centres as before. This catches TPU set, hoop cracking and node creep, which no paper has measured.
