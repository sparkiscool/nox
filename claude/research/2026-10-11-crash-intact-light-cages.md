# Spans and sockets size Scout's lightest cage

> **Design outcome, 11 Oct (added after the S2 cage design work; see `nox/v2/s2-cage.md`).** This report was written before the frame model ran. Two of its rules did not survive it:
> - **Rule 10 (no suspension):** a rigidly mounted cage is not intact at 1 m/s. A ceiling hit loads the struts with about 1.2 kN (C), and short-span pole hits overload the sockets. S2 cage v1 sits on four TPU 95A isolators, about 6.4 N/mm along each holder strut (C).
> - **Rule 4 (3 mm rods on every exposed span):** with the isolators, 2.0 mm rods on the top ring and struts and 2.5 mm on the equator flats pass every case (C), sleeved in heat-shrink. The 3 mm catch ring stays.
> Everything else (load case, allowables, glue practice, speed cap) was used as written.


Tags: (E) marks my estimates and arithmetic on cited inputs, at 0.45 kg unless stated. (Nox) marks a current Scout value from the Nox docs.

**The lightest S2 cage that stays intact at 1 m/s is a closed hull with 3 mm solid carbon on every span a pole or edge can reach, the fewest and smallest TPU for AMS nodes, sockets sized for the moment they carry, and no suspension.** A 1 m/s hit carries only 0.20 to 0.25 J (E), which 1 to 5 g of guard could absorb at published rates (E). Three local limits set the mass instead: one rod struck mid-span with the whole drone behind it, the socket mouth holding that rod, and the 12 mm prop keep-out (Nox). To stop that hit, a 155 mm span of 2 mm rod must reach about 1,200 MPa and bend 18 mm (E). That rules out the proposed 2 mm rods (Nox) on exposed spans; 3 mm solid does it at 800 MPa and 8 mm (E). Most of the mass is in the nodes: 64.3 g of cage v10's 101 g (Nox). The 55 g budget (Nox) fits the published 9 to 20% band (E), but 30 g of 3 mm rod and 5 g of glue leave about 20 g for nodes, a third of v10's (E).

## A 1 m/s crash carries 0.2 J, less than any commercial rating

No standard defines a crash the vehicle itself must survive. FAA Part 107 caps the energy transferred to people at 11 and 25 ft-lb ([eCFR](https://www.ecfr.gov/current/title-14/chapter-I/subchapter-F/part-107/subpart-D)), and MIL-STD-810H sets crash-hazard shock for flight equipment at 40 g and functional shock at 20 g ([Trenton](https://www.trentonsystems.com/en-us/resource-hub/blog/mil-std-810-shock-testing-overview-method-516.8)). **Flyability is the only maker that publishes a collision speed. Elios 3 is rated for 2 m/s frontal and 2.4 m/s from any direction, matching its 2 m/s Assist-Sport speed cap** ([Elios 3 specs](https://mfe-is.com/wp-content/uploads/2024/12/Elios-3_tech-specs.pdf)). Flybotix cites one 1.5 m drop that left ASIO undamaged ([UST](https://ust-media.com/ust-magazine/UST036/61)); hobby makers publish no numbers, and labs fly repeated wall hits instead.

Scout's 1 m/s equals a 5.1 cm drop, or 0.5 J/kg, which is a sixth of Elios 3's 2.88 J/kg (E). A motor cut from a 1 to 2 m hover delivers 4.4 to 8.8 J (E), a separate major-crash class. **Define the minor crash as 1 m/s through the CG into a wall, a 90 degree edge and a pole, at a node and at the worst mid-span on every face. Intact means no permanent set, no prop contact and no socket movement after 50 repeats (E).**

## Proven enclosures weigh 9 to 20% of all-up mass

| Design | AUM | Guard (share) | Intact result | Source |
|---|---|---|---|---|
| EPFL Euler cage, 2 mm CF rods | 282 g | 40 g (14%) | 46 of 50 wall flights undamaged | [EPFL](https://infoscience.epfl.ch/record/187769/files/cage-IROS.pdf?version=1) |
| Berkeley tensegrity v1, 6 mm CF rods | 252 g | 50 g (20%) | 6.5 m/s wall, all parts intact | [arXiv](https://arxiv.org/pdf/2003.03417) |
| CogniFly, CF rods in TPU 95A joints | about 241 g | not reported | 262 cm drop (about 7 m/s), no damage | [arXiv](https://arxiv.org/pdf/2107.11090) |
| ETH FlexiQuad, props exposed | 405 g | about 29 g (7%, E) | 3 and 4.5 m/s frontal, no damage | [arXiv](https://arxiv.org/pdf/2511.05426) |
| Flybotix ASIO, composite spars | 1 kg | under 100 g (under 10%, E) | 1.5 m drop, undamaged | [UST](https://ust-media.com/ust-magazine/UST036/61) |
| Crazyflie 2.1 Brushless guards | 37 g | 3 g (9%, E) | built to snap off on hard hits | [Bitcraze](https://www.bitcraze.io/2024/05/the-crazyflie-2-1-brushless-is-getting-optional-propeller-guards/) |

Cage v10 is 20 to 25% of a 400 to 500 g Scout (E). Per gram of guard, the Euler cage survived 0.05 J, tensegrity v1 0.11 J and FlexiQuad 0.14 J (E). First failures are ties, joints and members pushed into props: the Euler cage's strings, a tensegrity string at 14.4 m/s ([arXiv](https://arxiv.org/pdf/2211.12045)), and the AR.Drone's foam hull, which 6 N pushes into a propeller ([EPFL thesis](https://infoscience.epfl.ch/record/180234/files/EPFL_TH5438.pdf?version=3)). In side-by-side tests the rigid frames broke first ([HoLoArm](https://arxiv.org/pdf/2605.25790), [SoBAR](https://arxiv.org/pdf/2204.13155)).

## Exposed spans and socket mouths set rod and node sizes

| Material | Published values | Allowable for 100+ hits | Source |
|---|---|---|---|
| Pultruded CF rod | flexural 1.83 GPa, 131 GPa, 1.5% failure strain | 622 MPa at sockets and bare contact; up to 1,000 MPa on clean or sleeved spans (E) | [Rock West](https://www.rockwestcomposites.com/47316-4.html) |
| CF tube | flexural 1.37 GPa | wall cracks near 2 to 8 N per mm of line contact (E) | [Goodwinds](https://www.aircraftspruce.com/catalog/pdf/Goodwinds%20RodsTubes%20Specs.pdf) |
| TPU for AMS, XY / Z | 1,190 / 600 MPa; UTS 22.4 / 11.2 MPa; elongation over 650 / 31% | derate 0.7 if not annealed (E) | [Bambu](https://cdn.shopify.com/s/files/1/1339/4265/files/Bambu_TPU_for_AMS_Technical_Data_Sheet.pdf) |
| TPU 95A HF, XY / Z | 9.8 / 7.4 MPa; UTS 27.3 / 22.3 MPa | bumpers and hinges only (E) | [Bambu](https://www.additive-x.com/shop/mpattachments/file/viewonline/id/823/product_id/2584) |
| PETG Basic, XY / Z | 1,460 / 1,120 MPa; bending 81 / 64 MPa; elongation 11.2 / 5.7% | stiff sockets only (E) | [Bambu](https://www.additive-x.com/shop/mpattachments/file/viewonline/id/580/product_id/2330) |

The current 622 MPa limit (Nox) is 0.34 to 0.37 of flexural strength (E), below ACI 440's sustained-load cap of 0.55 f_u for CFRP ([Gajdosova](https://engmechx.it.cas.cz/improc/2017/0322.pdf)). 1,000 MPa is 0.76% strain (E), close to the 0.75% below which 90 degree UD carbon survived 10⁹ cycles ([MEJ](https://doaj.org/article/c655e2fb4430423a9d7cfa186352a22e)).

A solid rod struck mid-span stores U = Vσ²/(24E), where V is its volume (E). Here is one 155 mm span, like v10's equator flats (Nox), absorbing 0.225 J (E) alone with pinned ends:

| Rod (all values E) | Mass per span ([R&G rods](https://r-g.de/en/art/700006), [tubes](https://www.r-g.de/en/art/730302)) | Peak stress | Deflection | Verdict |
|---|---|---|---|---|
| 2 mm solid | 0.73 g | about 1,200 MPa | about 18 mm | fails, prop strike |
| 2.5 mm solid | about 1.2 g (E) | about 960 MPa | about 12 mm | fails on deflection |
| 3 x 2 mm tube | 0.93 g | about 900 MPa | about 9 mm | wall crushes at contact |
| 3 mm solid | 1.64 g | about 800 MPa | about 8 mm | passes if sleeved |

Clamped ends halve the deflection but put the same stress at the socket mouth (E). Swapping all 2.98 m of v10's rod (Nox) to 2 mm saves just 17.6 g, about four average v10 nodes (E).

Socket mouths are the second limit. A rod moment M bears on a socket of depth L at 6M/(dL²) (E). A 3 mm rod at 622 MPa loads a 9 mm socket to 41 MPa (E), against 22.4 MPa for TPU for AMS. Staying at 20 MPa takes a 13 mm socket, and 10 MPa takes 18 mm; a 2 mm rod needs 9 or 12 mm (E). The 10 mm holder sockets (Nox) see 33 MPa (E). v10's pull-out limits of 54 and 81 N (Nox) assume 0.95 MPa bond shear (E). DP420 epoxy reaches 2.2 to 3.8 MPa on abraded ABS ([3M](https://multimedia.3m.com/mws/media/1235389O/dp420-technical-data-sheet.pdf&fn=DP420.pdf)). TPU is harder: Stratasys calls it "difficult" to bond and rates CA and rigid epoxy as low-flexibility ([Stratasys](https://stratasys.com/siteassets/abm-pages/aerospace/bp_fdm_bonding-tpu_0220a.pdf)).

## Spend the stroke inside the cage, not on a suspension

| Mechanism | J per gram | Reuse | Source |
|---|---|---|---|
| CF rod, mid-span bending at 622 / 1,000 MPa | 0.08 / 0.21 | elastic | (E) |
| CF rod, Euler buckling at EPFL's design point | about 1.6 | 46 of 50 hits clean | [EPFL](https://infoscience.epfl.ch/record/187769/files/cage-IROS.pdf?version=1), (E) |
| CF rod, Euler buckling capped at 0.6% strain | about 0.19 | elastic | (E) |
| Printed soft TPU honeycomb | about 0.6 | elastic over 50 cycles | [Bates](https://research-information.bris.ac.uk/ws/files/183973872/1_s2.0_S0264127518308256_main.pdf), (E) |
| EPP 40 to 70 g/L, to 50% strain | 2.7 to 3.2 | 25 to 28% compression set | [Neopolen](https://www.knauf-industries.com/wp-content/uploads/2024/04/Neopolen-P-9230K-.pdf), (E) |

Foam leads per gram but needs thickness: 0.225 J takes 0.1 g of EPP, yet 9 mm of it (E). Buckling gives a flat force, but a 2.8 mm strut needs 150 mm of length for 10 mm of stroke even at 1.5% strain (E). So Scout's rods and nodes must absorb the hit.

Over a 10 mm stroke, the hit peaks at 45 N on a linear spring and 22 N at constant force (E). A mount that holds the catcher's 15 to 40 N (E), doubled for suddenly applied loads ([SOMC](https://pawel.jasion.pracownik.put.poznan.pl/wp-content/uploads/2025/05/SOMC_lecture_04.pdf)), cannot give below 80 N, so it stays rigid (E). Flyability's suspension, built for 1 to 10 J, claims compliance 2 to 5 times the cage's at 50 to 90% of rod buckling load, a damping ratio of 0.4 to 0.6, and 100 to 400 MPa materials with 80 to 200% elongation ([USPTO](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11661188)). Neither Bambu TPU falls in that range, and TPU 95A datasheets disagree sevenfold, 9.2 against 67 MPa ([Bambu](https://www.3drepublika.com/wp-content/uploads/2023/10/Bambu-TPU-95A-Technical-Data-Sheet.pdf), [Ultimaker](https://um-support-files.ultimaker.com/materials/2.85mm/tds/TPU-95A/TDS-TPU95A-V2.01-250915.pdf)). Future elastomers belong in shear, never tension ([Mecanocaucho](https://www.mecanocaucho.com/en/news/rubber-to-metal-mounts-design-en/)).

## Design rules for the S2 cage

Crash energy is the weakest constraint here. Spans, sockets and node count decide the mass.

1. **Load case:** 0.25 J (0.5 kg at 1 m/s) until the S2 is weighed, full mass, any direction, at every node and exposed mid-span, against a wall, a 90 degree edge and a pole (E).
2. **Stroke:** at most 10 mm of total travel, keeping 2 mm of the 12 mm keep-out (Nox); that holds the hit near 10 g (E), under MIL-STD-810H's 20 g functional level ([Trenton](https://www.trentonsystems.com/en-us/resource-hub/blog/mil-std-810-shock-testing-overview-method-516.8)).
3. **Coverage:** no opening may let the smallest expected pole or edge reach the keep-out (E).
4. **Rods:** 3 mm solid on every exposed span, about 110 to 170 mm long. 2 mm rods and bare tubes only where nothing can load them mid-span (E). Keep the 3 mm catch ring (Nox).
5. **Sleeves:** 0.15 to 0.45 mm of elastomer on contact zones (E). That is Flyability's preferred 5 to 15% of diameter at under a fifth of rod modulus, which the patent says raises strain energy to failure by 95% to over 300% ([patent](https://patents.google.com/patent/US20200189712A1/en)).
6. **Rod allowables:** 622 MPa at socket mouths and bare contact, 1,000 MPa on clean or sleeved spans. Check each batch reads 120 to 140 GPa in a 3-point bend (E).
7. **Nodes:** cut node count before rod diameter; v10 nodes weigh 2.6 to 5.3 g each (Nox). Print in TPU for AMS only: v10 needs at least 120 MPa (Nox), and PETG's 5.7% Z elongation ([Bambu](https://www.additive-x.com/shop/mpattachments/file/viewonline/id/580/product_id/2330)) cracks where TPU yields (E).
8. **Sockets:** mouth bearing near 10 MPa across layers and 20 MPa in XY (E), so 13 to 18 mm deep for 3 mm rods. The alternative is near-pinned necks with triangulated rods; axial load paths cut mean peak stress from 100.5 to 34.4 MPa in Berkeley's tensegrity study ([arXiv](https://arxiv.org/pdf/2211.12045)). Holder sockets 13 mm deep or collared (E).
9. **Glue:** abrade rods and bores at 80 grit, wipe with IPA, no MEK on TPU ([Stratasys](https://stratasys.com/siteassets/abm-pages/aerospace/bp_fdm_bonding-tpu_0220a.pdf)). Flexible adhesive plus a cross-pin, designed to 1 MPa bond shear on TPU and 2 MPa on PETG until five pulled coupons say otherwise (E). No press fits: soft TPU is still relaxing after 6 hours ([Bruere](https://link.springer.com/article/10.1007/s40964-023-00418-7)).
10. **Mount:** no suspension or breakaway mount at 1 m/s. Above 2 m/s, adopt Flyability's ratios with a progressive stop 1 to 2 mm before prop contact (E).
11. **Major crash:** joints release before a rod or the holder breaks, at 70 to 90% of rod rupture strain as Flyability claims ([USPTO](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11661188)); nodes swappable (E).
12. **Speed cap:** cap near-obstacle speed at the rated 1 m/s, as Flyability does ([Elios 3 specs](https://mfe-is.com/wp-content/uploads/2024/12/Elios-3_tech-specs.pdf)).
