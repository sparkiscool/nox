# Manway and hatch sizes vs the Scout envelope (11 Oct 2026)

**Bottom line**
1. The smallest common openings are vessel and boiler manways: 12 x 16 in (305 x 406 mm) elliptical, 15 to 16 in (381 to 406 mm) round. Tanks, ducts, sewers and ship tanks start at 20 in, 20 x 20 in, 24 in or 600 x 600 mm.
2. Scout v1 passes all of them (E). Nose-on it fits a ~242 mm circle; in plan the bottom soles reach ~293 mm. A 12 x 16 in manway leaves ~32 mm a side through a wall, ~20 from above. Elios 3 (480 mm) needs 20 in.
3. Pilots fly in from outside. Scout v2 target (E): plan inside a 275 x 375 mm ellipse and a 305 mm circle, nose-on inside a 275 mm circle. That is a 12 x 16 in manway with 15 mm a side and a 16 in one with 50. v1 already fits, so v2 only has to not grow.

## Findings

**Pressure vessels.** Vessel size decides whether a manhole is needed, not its size: 18 to 36 in ID may use two handholes or NPS 2 plugs; over 36 in need a manhole unless impractical [CA462; ET-402958]. UG-46(g)(1) minimums, as engineers quote them: 12 x 16 in elliptical or obround, 16 in round [ET-409104; ET-9070556]. California's air-tank rules keep older values: 11 x 15 or 10 x 16 in, 15 in round [CA462], likely an older edition (E). Practice runs larger: 18 in to save cost, 20 in ID or 24 in OD for SCBA, 24 in in many company standards, DN750 (~30 in) now often asked [ET-409104; ET-9070556].

**Boilers.** Settings need 12 x 16 in unless smaller still allows inspection; vertical fire-tube boilers over 36 in need 12 x 16 in (11 in minimum) or 15 in round [CA768]. Flyability shows the pilot at the entrance manhole while the drone flies a closed boiler [Elios-brochure].

**Storage tanks (API 650).** Shell manholes: 20, 24, 30 or 36 in nominal ID [OpenExam], gasket bores the same [Garlock]. Roof manholes: 20 or 24 in [Garlock]; 10 to 18 in roof ports are inspection ports [ET-383164]. API 653 text not reached.

**Stacks, ducts, penstocks.** Elios 3 cases (4 m x 120 m chimney, 8 m x 180 m stack, 7 min penstock) give no door sizes [FA-chimney; FA-stack; FA-penstock]. Grease ducts big enough to enter need one 20 x 20 in opening [AKDPS]. Elios 2's 400 mm cage entered 1400 mm ducting through manways; a precipitator target sat past a 450 mm baffle gap [Nexxis-mill].

**Sewers.** US frames: 24 in clear [CalPoly] over a 48 in barrel [CalPoly; ODOT]. EN 124 advises 600 mm; UK roads want 700 mm round, or a 600 mm rectangle with a 700 mm diagonal [Wrekin]. A crew of 2 flew Elios 3 down 10 m shafts [FA-sewer], range extender at the entry [FA-sewerpage].

**Ship tanks.** IMO: horizontal openings 600 x 600 mm, vertical 600 x 800 mm, corners rounded; flags may approve smaller [IMO686]. That 1995 guideline was later folded into SOLAS with the same sizes (E).

**Entry rules.** Entry starts when any body part breaks the opening's plane [OSHA]. Portals of 24 in or less are restricted: too small for a rescuer in SCBA [OSHA-F]. Drones are flown in from outside [COPTRZ; FA-sewer]; reaching in to place one is an entry (E).

**Drones vs openings** (mm a side, E; worst of wall and top unless split):
| Drone | Size | Maker's opening claim | 12 x 16 in | 16 in | 20 in |
|---|---|---|---|---|---|
| Elios 1 / 2 | under 400 sphere [Elios-brochure; DroneUA] | just under the smallest standard manhole [Elios-brochure] | no | ~3 | ~54 |
| Elios 3 | 480 W x 380 H [E3-spec] | fits manholes, no size [FA-E3] | no | no | ~14 if round |
| ASIO X | Ø459, 1.15 kg [Frontier] | none | no | no | ~24 |
| Scout 137 | 613 x 382 x 303, 3.2 kg, 40 m tether [Measur]; IKM lists 680 x 530 x 400, 17 kg, likely the kit (E) [IKM] | none | no | no | ~10, wall only |
| IBIS2 | 194 x 198.5 x 58, 243 g [MONOist] | 30 cm ports, 50 cm pipes [MONOist] | yes | yes | yes |
| Scout v1, wall / top | ~242 circle nose-on, ~293 plan (E) | none yet | ~32 / ~20 | ~82 / ~57 | ~133 / ~108 |

Elios 1 / 2 match the 16 in round minimum (E). Scout v1 nose-on (E): equator ~240 with nodes, top ring and soles all inside a ~242 circle on the prop plane.

## Steal for Nox
- Business: pitch "passes a 16 in manway and a 12 x 16 in ellipse"; Elios 3 needs 20 in (E).
- Scout v2 cage: adopt the envelope target; lights, ToF, bumpers and antennas stay inside it (E).
- Scout v2 cage: rounded nodes, no proud rod ends, so a manway lip slides off (E).
- Scout ops: fly through from outside, centred by the forward ToF from the standoff report (E).
- Demo: plywood gauges (12 x 16 in ellipse, 16 and 18 in rings, ~150 mm neck) on the demo floor (E).
- Overseer: 608 across its prop disks (C), so it stays outside; it maps the hall, the Scout goes in (E).
- Box: set the box or a relay at the opening, as Flyability puts its range extender at the entry [FA-sewerpage].

## For Spark
- `nox-dimensions.md` §3 "277.4 across corners" is equator rod centres; with §4 "sole centred on the corner" (CHECK) the plan is ~293 (E). Measure across the soles (P) before quoting a manway.
- `nox/v2/dimensions.md` §C: suggested envelope row (E), from Bottom line 3. The kept "194 square at z -40" ring and its soles set the plan corners; the proposed top ring "at z 77.5" brings height to ~126 (E).
- Same doc, "12 from the prop disks": with 3.5 in props the smallest equator is ~224 across flats (E) vs v1's "230 across flats". A cage diet opens no common manways; only smaller props would (3 in: ~210, E).
- Out of reach for a Bee35 Scout (E): 10 x 16 in from above, round top ports under ~300 mm (IBIS2's niche), vessels of 36 in ID or less with handholes only [CA462]. Say so in the pitch.
- `claude/research/2026-10-11-inspection-drone-players.md`: "~285 with nodes" is ~293 with soles (E), and "should pass an 18 in" also holds for 16 in round and 12 x 16 in elliptical. Its Scout 137 "60 m tether" conflicts with Measur's guide (40 m), perhaps another generation (E).

## Sources
- [API 650 Manhole Torque Values (Garlock)](https://legacy.garlock.com/sites/default/files/documents/en/API-650-Manhole-Torque-Values.pdf)
- [API 653 study guide, appurtenances (OpenExam)](https://open-exam-prep.com/study-guides/api-653/shell-roof-components-inspection/appurtenances-nozzles-connections)
- [Shell and roof manhole (ET-383164)](https://eng-tips.com/threads/shell-and-roof-manhole.383164)
- [Manway minimum size (ET-409104)](https://eng-tips.com/viewthread.cfm?qid=409104)
- [Manway size replies (ET-9070556)](https://www.eng-tips.com/posts/9070556)
- [Inspection opening, UG-46 (ET-402958)](https://eng-tips.com/threads/inspection-opening.402958)
- [Cal. Code Regs. T8 §462, air tanks (CA462)](https://regulations.justia.com/states/california/title-8/division-1/chapter-4/subchapter-1/article-3/section-462)
- [Cal. Code Regs. T8 §768, boilers (CA768)](https://www.dir.ca.gov/title8/768.html)
- [OSHA 1910.146 (OSHA)](https://www.osha.gov/laws-regs/regulations/standardnumber/1910/1910.146)
- [OSHA 1910.146 App. F (OSHA-F)](https://www.osha.gov/laws-regs/regulations/standardnumber/1910/1910.146AppF)
- [IMO MSC/Circ.686, means of access (IMO686)](https://deutsche-flagge.de/de/redaktion/dokumente/interpretationen/solas/xi-1/msc-1-circ-686-guidelines-on-the-means-of-access-to-structures-for-inspection-and-maintenance-of-oil-tankers-and-bulk-carriers.pdf)
- [Cal Poly 6610, sewer manhole (CalPoly)](https://afd.calpoly.edu/facilities/planning-capital-projects/construction-standard/docs/standard-details/6610-Sewer-and-Storm-Manhole.pdf)
- [Oregon DOT RD335 (ODOT)](https://www.oregon.gov/ODOT/Engineering/202601/RD335.pdf)
- [Clear openings of manhole covers (Wrekin)](https://www.wrekinproducts.com/services/knowledge-base/product-specification-guidance/understanding-clear-openings-manhole-covers)
- [Kitchen duct construction, NFPA 96 (AKDPS)](https://dps.alaska.gov/getmedia/53f87aee-cbea-4f97-ba62-267aae894dfd/KitchenDuctConstruction2010_19)
- [Flyability Elios brochure (Elios-brochure)](https://cms.esi.info/Media/documents/29815_1548756746854.pdf)
- [Elios 2 (DroneUA)](https://drone.ua/en/blogs/news/elios-2-malenkiy-dron-s-ogromnyimi-vozmozhnostyami)
- [Elios 3 tech spec (E3-spec)](https://nexxis.com/wp-content/uploads/sites/3/2022/06/20220519_E3_Tech_Spec_web_version.pdf)
- [Elios 3 product page (FA-E3)](https://www.flyability.com/elios-3)
- [Elios 2 power station mill case (Nexxis-mill)](https://nexxis.com/case-study-elios-2-delivering-successful-power-station-mill-inspections/)
- [Surveying sewers with the Elios 3 (FA-sewer)](https://www.flyability.com/casestudies/drone-sewer-survey)
- [Sewer drones (FA-sewerpage)](https://www.flyability.com/sewer-drones)
- [Chimney tower inspection (FA-chimney)](https://www.flyability.com/casestudies/chimney-tower-inspection)
- [Stack inspection (FA-stack)](https://www.flyability.com/casestudies/stack-inspection)
- [Penstock inspection (FA-penstock)](https://www.flyability.com/casestudies/drone-penstock-inspection)
- [Wastewater inspections (COPTRZ)](https://coptrz.com/blog/transforming-wastewater-inspections-drones-vs-traditional-methods/)
- [IBIS2 at Japan Drone 2023 (MONOist)](https://monoist.itmedia.co.jp/mn/spv/2306/30/news098.html)
- [Flybotix ASIO X spec flyer (Frontier)](https://shop.frontierprecision.com/wp-content/uploads/2026/06/Frontier-Precision-Flybotix-ASIO-X-NDAA-Specification-Flyer.pdf)
- [Scout 137 and ASIO X guide (Measur)](https://measur.ca/blogs/news/guide-to-the-top-2-confined-space-drones-scout-137-drone-system-and-flybotix-asio-x)
- [IKM UAV 26A, Scout 137 (IKM)](https://www.ikm.com/ikm-testing-norway/equipment-catalog/uav-drones/uav-26a)
