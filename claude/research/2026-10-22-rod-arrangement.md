# Cage rod arrangement: topology and geometry rules for Scout's 3.0 mm carbon cage

Scope: how to arrange the rods of the Scout v2 lantern cage (29 rods, 3.42 m, 36 g; 17 printed parts ~100 g; ~152 g total, E) for the most impact protection per gram. Tags: P measured, M mesh, C CAD, E estimate, F frame model. "(E, calc)" = my beam arithmetic on the stated inputs: 3.0 mm solid pultruded rod, E 130 GPa, rho 1.55 g/cm3 (10.5 g/m, matches 36 g / 3.42 m), allowable 1000 MPa on clean or sleeved spans, 622 MPa at socket mouths (both from the Oct 11 cage note). Script for the panel comparison: `panel_grillage.py` in this folder.

## 1. The physics that decides the layout (E, calc)

**Energy per gram of a struck rod is fixed by the material, not the span.** A rod hit mid-span stores U = sigma^2 A L / (24 E) at peak stress sigma (simply supported, SS). That is 0.207 J per gram of rod at 1000 MPa whatever the length. Longer spans store more per hit only because more grams get involved. So "many short spans" vs "few long spans" is decided by three other limits: bow, force, and node mass.

**Bow sets the longest span.** Mid-span bow at the allowable stress is delta = sigma L^2 / (12 E c) (SS). With a 6 mm bow budget (12 mm prop clearance, half kept as margin):

| End condition | Max span L* for 6 mm bow | Energy one span holds at L* | Load at allowable (100 mm span) | Moment at the node |
|---|---|---|---|---|
| SS, 1000 MPa | 118 mm | 0.27 J | 106 N | 0 |
| Continuous over supports (through-rod), 1000 MPa | 136 mm | 0.34 J | 155 N | 0.079 PL (46 % of mid-span) |
| Clamped in sockets, 1000 MPa | 168 mm | 0.38 J | 212 N | 0.125 PL (= mid-span peak, at the socket mouth) |
| SS, 622 MPa (socket-mouth allowable) | 150 mm | 0.13 J | 66 N | 0 |
| SS, 1500 MPa (near failure) | 97 mm | 0.49 J | 159 N | 0 |

Continuous-beam coefficients from a 40-element FE check: SS mid-load deflection 0.0208 PL^3/EI and moment 0.250 PL; a span inside a 3 to 9 span continuous rod 0.0110 and 0.171 PL, support moment 0.079 PL. A continuous rod therefore bows **47 % less** and peaks **32 % lower** than the same span ending in two pinned sockets. Clamped sockets do as well on paper but put the full peak moment at the socket mouth, which is where the v12 / v13 neck tests already fail.

The lantern's mean rod length is 3.42 m / 29 = 118 mm (C), right at L* for SS. **Spans are not the problem; the end condition and the nodes are.**

**One rod cannot take the stated hits.** Peak force for an energy U over a total stroke s is at least U / s (ideal constant force) and 2U / s (linear spring):

| Hit | Stroke 6 mm | 10 mm | 15 mm |
|---|---|---|---|
| 0.25 J (0.5 kg at 1 m/s) | 42 / 83 N | 25 / 50 N | 17 / 33 N |
| 1.0 J | 167 / 333 N | 100 / 200 N | 67 / 133 N |
| 2.3 J (3 m/s, or the plank) | 383 / 767 N | 230 / 460 N | 153 / 307 N |

(constant / linear). One 3 mm span at 100 to 120 mm carries 90 to 155 N before 1000 MPa. So above roughly 0.3 J per mid-span hit, **either several rods must share the hit, or stroke must come from somewhere other than the struck rod** (node elastomer, isolators, a sacrificial release). Above ~1 J nothing local in a 12 mm clearance survives intact on 3 mm rods; the layout must spread load and the 2.3 J cases become "no prop strike, a part releases" cases.

**3 mm rods make poor Euler springs.** EPFL's Euler-spring cage gets ~1.5 J per rod from 2 mm x 200 mm rods at compression ratio 0.34 (P, Klaptocz). A 3 mm rod capped at 1000 MPa bending in post-buckling shortens only 0.7 / 2.3 / 5.3 mm at L = 100 / 150 / 200 mm (Fcr 510 / 227 / 128 N), storing 0.34 / 0.51 / 0.68 J (0.31 J/g) and bowing sideways 5 / 12 / 21 mm (E, calc). Better per gram than mid-span bending (0.21 J/g) and flat-topped in force, but stroke is tiny and the side bow must miss the props. With one rod size, buckling struts are only worth it at L >= 200 mm, pointing away from the props.

## 2. Triangulated vs open panels (E, calc, grillage)

Panel 200 x 100 mm, rigid pinned frame (the rings / columns), 3.0 mm rods, point load normal to the panel anywhere at least 20 mm from the frame, allowable 1000 MPa. Worst point = lowest energy to reach the allowable.

| Layout in the panel | Rod g | Crossings | Worst-point energy | Max bow at allowable | J per g of panel rod |
|---|---|---|---|---|---|
| A: 3 parallel rods, 100 mm spans, 50 mm pitch | 3.3 | 0 | 0.23 J | 4.3 mm | 0.069 |
| B: 1 rod, 200 mm span | 2.2 | 0 | 0.45 J | **17 mm (fails clearance)** | 0.207 |
| C: grid, 3 across + 1 long, lashed | 5.5 | 3 | 0.22 J | 5.0 mm | 0.041 |
| D: X in each square cell, lashed | 6.2 | 2 | 0.31 J | 8.5 mm | 0.050 |
| E / G: Warren zigzag or diamond, 4 rods | 4.9 | 0 | 0.25 J | 5.3 mm | 0.052 |
| F: two-way diagonal lattice, 7 crossings | 9.3 | 7 | 0.15 J | 0.5 mm | 0.016 |

What it says:
- **For a hit normal to the skin, in-plane triangulation does nothing for energy.** The rods bend out of plane either way. Adding diagonals or crossings creates short sub-spans that hit the allowable at small deflection, so the worst-point energy stays flat or drops while mass rises: J/g falls 1.4x to 4x versus open parallel rods (A vs C, D, F).
- **Crossings buy bow and opening size, not energy.** F has 0.5 mm bow and small holes but 0.15 J. Use crossings only where an opening or a bow limit demands it.
- **Lashed = rigid at crossings for normal hits.** C, D and F gave the same worst-case energy with crossings that only tie transverse displacement (lashed) as with fully rigid crossing joints. A lashed or clipped crossing is enough; no printed node is needed there.
- **Triangulation earns its mass in three cases only:** (1) nodes that are compliant or pinned (TPU, lashed, rotating), where an untriangulated frame is a mechanism; (2) node hits on a curved shell, where a triangulated surface carries the load as axial force (Berkeley: axial load paths cut mean peak rod stress from 100.5 to 34.4 MPa vs a bending-loaded prop guard, F/sim, Zha et al.); (3) racking of tall open bays. With rigid PA12-CF nodes and a 26.8 mm tall drum (C), the lantern needs none of these on its side faces.

Stretch vs bending (general lattice theory, Ashby; Maxwell b >= 3j - 6 for a pin-jointed space frame to be rigid): stretch-dominated frames scale stiffness ~ density, bending-dominated ~ density^2. For impact at a bounded stroke this is not "triangulated is better": a stiffer cage shortens stroke and raises force (Klaptocz: stiff protection passes nearly all impact energy to the frame). The goal is the stiffness that uses the whole clearance at the allowable stress, no stiffer.

## 3. How published cages arrange rods

| Cage | Topology | Rods / nodes / valence | Span | Mass split | Hit rating | Source |
|---|---|---|---|---|---|---|
| EPFL Euler-spring (AirBurr) | 8 tetrahedra of 3 rods round a 12 cm-radius core | 24 rods (2 mm x ~200 mm pultruded, E 113 +/- 15 GPa P); ends in SLS-ABS end pieces on rotating joints tied with nylon line | L = sqrt(8/3) x core radius = 19.6 cm | 40 g total incl. end pieces (14 % of 282 g); rods ~22 g, ends + line ~18 g (E) | 1.5 J per rod; 46 of 50 wall flights undamaged; failures were nylon ties and rotor contact | [Klaptocz et al. IROS](https://infoscience.epfl.ch/record/187769/files/cage-IROS.pdf?version=1), [thesis](https://infoscience.epfl.ch/record/180234/files/EPFL_TH5438.pdf?version=3) |
| EPFL Gimball | Spherical polyhedron, r 17 cm, rods are polyhedron edges | Count not published; if a truncated icosahedron, 90 rods / 60 nodes / valence 3 / edge ~69 mm (E, geometry inference only) | ~69 mm (E) | 84 g cage (22 % of 385 g) + 56 g gimbal | Up to 3 m/s; above that printed cage parts or gimbal joints broke, not rods | [Briod et al. 2014](https://infoscience.epfl.ch/record/190790/files/collision_resilient_flying_robots_LIS2014.pdf) |
| Flyability Elios 1 | Sphere < 400 mm of CFRP rods in printed reinforced-plastic nodes, joined into hexagon / pentagon modules held by ties | Not published (UST 8 gives none; spares sold as "cage pentagons type 1 / type 2", 2 module types) | n/a | drone 700 g, cage ~100 g (secondary, E) | 15 km/h | [UST 8](https://www.ust-media.com/ust-magazine/UST008/files/assets/basic-html/page-43.html), [IKM sheet](https://www.ikm.no/getfile.php/1385906-1591746760/Import/new-equipment-catalog/WUAV/439389.pdf) |
| Flyability patent family | 12 pentagons, or 12 hexagons + 6 squares; each module has polygon edges plus radial beams to a centre apex (triangular openings); beams end in corner connectors (male / female, 4 to 6 per module) that release at 70 to 90 % of beam rupture strain | Edge valence 3 at corners, 5 to 6 at apexes | n/a | n/a | 1 to 10 J design range; TPE-class sleeve 5 to 15 % of core diameter | [US 2020/0189712](https://patents.justia.com/patent/20200189712) |
| Flybotix ASIO | 10 arcuate spars from pole ring to pole ring + strings | **2 nodes (polar rings), valence 10**; flat ribs 0.5 to 5 mm thick, 2 to 20 mm wide, wider mid-span | full meridian | < 100 g on ~1 kg | 1.5 m drop undamaged | [Flybotix patent](https://ipiq.io/patents/353320), [UST 36](https://www.ust-media.com/ust-magazine/UST036/61/) |
| Berkeley tensegrity | Icosahedron tensegrity | 6 rods (6 mm CF), 12 nodes, 24 strings (E from 4 per node); each node 1 rod + 4 strings, printed fibreglass-filled caps | rod ~20 cm | 95 g shell (32 % of 300 g) | 7 m drop (11.7 m/s), 7.8 m/s wall; failed by a string at 14.4 m/s | [Zha et al.](https://arxiv.org/pdf/2211.12045) |
| CogniFly | Box exoskeleton, CF rods in TPU 95A joints folded round a rigid centre | not published | 210 x 210 x 120 mm | 241 g whole drone | 262 cm drop (~7 m/s), no damage | [arXiv 2107.11090](https://arxiv.org/pdf/2107.11090) |
| Cinewhoop ducts / AR.Drone hull | Continuous moulded ring or EPP foam | 0 nodes | n/a | AR.Drone foam 60 g, pushed into a prop by 6 N | n/a | [Klaptocz thesis](https://infoscience.epfl.ch/record/180234/files/EPFL_TH5438.pdf?version=3) |
| **Scout lantern v13** | Drum (2 clipped-square rings) + 8 columns + crown of 8 spokes to 1 hub + 8 glove rods + sled | 29 rods / 17 printed parts / 58 rod ends; hub valence 8 | mean 118 mm (C) | rods 36 g, printed ~100 g, **nodes : rods = 2.8 : 1**, **~1.7 g printed per rod end** (C) | design 0.25 to 2.3 J | `nox/v2/s2-layout.md` |

Patterns: every rod cage that failed in test failed at a printed node, tie or glue line, not a pultruded rod (Gimball, AirBurr, EPFL origami cage, Berkeley strings). Node mass benchmarks: AirBurr ~0.7 g per rod end (E), lantern v10 0.74 g (C), lantern v13 ~1.7 g (C). Cages built for repeated crashes either minimise nodes (ASIO: 2), make nodes compliant (CogniFly TPU, AirBurr rotating), or make them releasable (Flyability). No source publishes an equal-mass bake-off of sphere vs lantern vs polyhedron.

## 4. Nodes vs rods: where the grams are

- **One average printed part (~6 g, C) weighs as much as ~0.55 m of 3 mm rod.** Deleting a node is worth more than shortening any rod. Every layout decision should be priced in nodes first.
- **Through-rods cut both mass and the weak spot.** A rod passing through a bore instead of ending in two sockets: one bore instead of two socket mouths (about half the socket mass per crossing, E), 47 % less bow and 32 % less peak moment at mid-span (calc above), and the node carries 0.079 PL instead of the full peak at a socket root. Through-bores still need a bearing collar and a lead-in radius; glue them or pin them, the moment there is small.
- **Through-rods must be straight or nearly so.** Bending a 3 mm rod to radius R prestresses it to E c / R: 1740 MPa at R 112 (impossible), 975 MPa at R 200, 390 MPa at R 500, 244 MPa at R 800 (E, calc). Keep any bend at R >= 800 mm (prestress <= 25 % of allowable). So a through-rod runs along a straight line: a ring flat, a column, a ruled-surface line, a spoke crossing the hub.
- **Straight lines through many nodes come from ruled surfaces.** Flats of a prism (the drum flats at +/-112), cone generators (the 45 deg glove faces), and the two line families of a hyperboloid (Shukhov's lattice towers, which used straight bars on a doubly ruled surface, [Wikipedia](https://en.wikipedia.org/wiki/Hyperboloid_structure)). A twisted drum (two skew families of straight rods ring to ring, crossing each other) gives a closed, curved, diagonal skin with zero bent rods and only lashed crossings.
- **Fewer, higher-valence nodes beat many low-valence ones** because every printed part pays a fixed base (body, skin, bed face) on top of its sockets. The crown hub (valence 8) is the model; the 8 drum columns (valence ~3 to 4 each) are the opposite.
- **Lash or clip crossings instead of noding them** where two rods merely cross (grillage: no loss for normal hits). Kevlar thread + thin CA or a snap clip is well under 1 g per crossing (E).

## 5. Optimization methods that fit this problem

- **Ground structure, worst-case load.** Ben-Tal and Nemirovski (SIAM J. Optim. 1997) showed single-load optimal trusses are often mechanisms: a secondary load of 0.5 % of the primary raised a console's compliance from 1.00 to over 8.4, while the robust design over a load ellipsoid cost only 0.24 to 0.9 % on the primary case ([PDF](https://www2.isye.gatech.edu/~nemirovs/SIOPT_RTTD_1997.pdf)). Lesson for the cage: optimising for a few named hits gives a fragile layout; "hit anywhere" robustness is nearly free if built in from the start.
- **Infinite load directions.** Alcazar, Oliveira, Senhora, Ramos, Paulino, Struct. Multidisc. Optim. 67:136 (2024): min-max compliance over a continuum of load directions in a ground structure, with smooth max regularisation of the max-eigenvalue problem ([Princeton](https://collaborate.princeton.edu/en/publications/a-smooth-maximum-regularization-approach-for-robust-topology-opti/)).
- **Frames (bending members), multi-load, global.** Tyburec, Kocvara, Kruzik: global weight optimisation of frames under bounded compliance in multiple load cases via moment-sum-of-squares ([arXiv 2211.14066](https://ar5iv.labs.arxiv.org/html/2211.14066)); Kanno's mixed-integer frame formulations with discrete sections ([Kanno slides](https://www.mi-kanno.mist.i.u-tokyo.ac.jp/kanno/presen/wccm2014.pdf)).
- **Crash loads.** Equivalent static loads (ESL) turn a nonlinear dynamic crash into linear static load cases for topology optimisation ([SMO 2022](https://link.springer.com/article/10.1007/s00158-022-03309-7)).
- **No paper found that optimises a drone protective cage layout with a ground structure.** Closest: lattice / topology optimisation of drone frames for flight loads, not impact-anywhere.
- **Formulation that fits Scout (DEFAULT):** ground structure of candidate straight rods between candidate node sites on the funnel-compatible envelope; beam elements; load cases = a point load at every node and every mid-span, plus line loads along edges at 0 / 45 / 90 deg on every face; objective = worst-case energy at sigma_allow with bow <= 6 mm and no prop-disc intrusion; cost = rod grams + a fixed charge per node (~6 g) + per socket (~1 g) minus a credit for a through-bore; one rod diameter, so the only variables are which candidate rods exist and whether they run through or end. That is a fixed-charge (mixed-integer) problem: small enough to enumerate symmetric candidates (8-fold or 4-fold) with the `noxopt` frame model.

## 6. Rules for the redesign

1. **Span: 100 to 135 mm, no shorter, run continuous.** At 6 mm bow and 1000 MPa, L* is 118 mm SS and 136 mm continuous (E, calc). Shorter spans add nodes without adding energy per gram.
2. **Make rods continuous through nodes wherever the line is straight.** -47 % bow, -32 % peak moment, node moment 0.079 PL instead of the full peak at a socket root. Bends only at R >= 800 mm.
3. **Price layouts in nodes.** One printed part ~6 g = 0.55 m of rod (C). Targets: node mass <= rod mass (lantern is 2.8 : 1), <= 1 g printed per rod end (lantern ~1.7), fewer parts of higher valence.
4. **No in-plane triangulation on side faces with rigid nodes.** It cuts worst-point energy per gram 1.4x to 4x for normal hits (grillage). Add a diagonal only to close an opening or hold bow, and lash it where it crosses.
5. **Every face must put >= 2 rods under any edge direction.** Parallel-only rods let a plank edge parallel to them land on one rod or slip between. Two skew families (crossed diagonals, or a twisted ruled drum) guarantee >= 2 rods for every edge angle and double the energy per edge hit.
6. **Opening size from wedge penetration.** An edge of included angle alpha enters a slot of clear width w to depth w / (2 tan(alpha/2)): w/2 for a 90 deg edge, 1.2 w for 45 deg. Keep (penetration + bow) under the prop clearance on every opening. Check the drum: rings 26.8 mm apart (C) leave ~23.8 mm clear, so a horizontal 90 deg edge reaches ~11.9 mm, right at the 12 mm clearance (E). It is 5.4 mm above the prop plane, so it probably misses, but it is the first opening to verify.
7. **Hits above ~0.3 J need load sharing or stroke outside the rod.** One 3 mm span holds 0.27 to 0.34 J within 6 mm bow; 1 J needs >= 3 rods or ~10 mm of node / mount stroke; 2.3 J needs 230 to 460 N at 10 mm, so design it as "no prop strike, a joint releases" (Flyability: release at 70 to 90 % of rod rupture strain).
8. **Optimise against hit-anywhere, not named cases.** Worst case over every node, every mid-span and every edge angle, with a fixed node charge. Named-case designs end up mechanism-prone (Ben-Tal / Nemirovski: 0.5 % side load, 8.4x compliance), and robustness costs < 1 % if it is in from the start.

## Sources
- Klaptocz et al., Euler Spring Collision Protection for Flying Robots, IROS 2013: https://infoscience.epfl.ch/record/187769/files/cage-IROS.pdf?version=1
- Klaptocz PhD thesis EPFL TH5438: https://infoscience.epfl.ch/record/180234/files/EPFL_TH5438.pdf?version=3
- Briod et al., A collision-resilient flying robot (Gimball), JFR 2014: https://infoscience.epfl.ch/record/190790/files/collision_resilient_flying_robots_LIS2014.pdf
- Zha et al., icosahedron tensegrity aerial vehicle: https://arxiv.org/pdf/2211.12045
- CogniFly flexible exoskeleton: https://arxiv.org/pdf/2107.11090
- Flyability patent US 2020/0189712 (cage modules, connectors, sleeve): https://patents.justia.com/patent/20200189712 ; US11708160: https://patents.google.com/patent/US11708160
- Flybotix foldable cage patent: https://ipiq.io/patents/353320 ; UST 36: https://www.ust-media.com/ust-magazine/UST036/61/
- UST 8 (Elios): https://www.ust-media.com/ust-magazine/UST008/files/assets/basic-html/page-43.html ; IKM sheet: https://www.ikm.no/getfile.php/1385906-1591746760/Import/new-equipment-catalog/WUAV/439389.pdf
- Ben-Tal and Nemirovski, Robust truss topology design via SDP, 1997: https://www2.isye.gatech.edu/~nemirovs/SIOPT_RTTD_1997.pdf
- Alcazar et al., SMO 2024, smooth max robust ground structure: https://collaborate.princeton.edu/en/publications/a-smooth-maximum-regularization-approach-for-robust-topology-opti/
- Tyburec et al., global frame weight optimisation: https://ar5iv.labs.arxiv.org/html/2211.14066
- ESL crash topology optimisation, SMO 2022: https://link.springer.com/article/10.1007/s00158-022-03309-7
- Hyperboloid (Shukhov) structures: https://en.wikipedia.org/wiki/Hyperboloid_structure
- Prior Nox notes: `nox/claude/research/2026-10-11-crash-intact-light-cages.md`, `2026-10-19-cage-collision-designs.md`
