# Swap-station mechanisms: how docks pull, insert and latch packs (11 Oct 2026)

**Bottom line**
1. The robot swappers (Airobotics/Ondas, Hextronics, DBOX, Heisha T200) publish no swap hardware. The only pack lock found is in a filing by an Airobotics inventor. Percepto's and H3 Dynamics' patents describe pusher docks that charge in place.
2. Every rig found runs one order: center the drone, grip a feature built into the pack, push until a spring latch catches, confirm by switch or contact. Swaps take 60 s to about 2 min [Advexure; arXiv; Oulu; EIN]. The only failure count found is 5 in 200 runs [Oulu].
3. Top steals: an Overseer latch only the pusher's pins can release; an iris that centers soft, then holds and reads its bar gap at stall; a 200-run swap test.

## Findings

**Products** (Hextronics Universal and RAPID: see `2026-10-11-dock-swap-mechanisms.md`)

| System | Centering | Pull, insert, latch | Packs, time | Mass, size | Patent, core claim |
|---|---|---|---|---|---|
| Airobotics / Ondas Optimus | Sloped movers trap the legs [578] | An arm trades the battery module [New Atlas]. In the filing, two tube grabbers enter bores in a module on the pack. Spreading them flexes two sheet locks off tabs on the drone's handle; closing them re-locks. Cone entries take the drone's pins; 1 mm alignment cited [085] | 11 packs, 9 payloads, under 1.5 min (container version) [Janes] | Drone 6.8 kg [Drive]; base dumpster-sized [TechCrunch] | US20190127085A1, inventors include Meir Kliner, named on Airobotics' patents [Kliner]; filed 2017, grant not found. A pack module whose lock trips only along two straight guides [085] |
| Hextronics Atlas | ±15 cm [Advexure]; 17 cm radial [Drone Nerds] | Not published | 8 packs; 60 s in the specs, 90 s average on the same page; 150 s land to launch; 99.99% claimed [Advexure] | 190 kg; 1.4 x 1.1 x 1.0 m; 1600 W peak [Advexure] | None found [Justia H] |
| Percepto Base / Air Max (charges in place [Melnyk]) | Two belt-driven pusher pairs: the far pair centers lightly, the near pair drives in to mate, then the far pair adds pressure. An encoder stops them at a set gap [938] | A toothed connector on a pusher meets teeth on a skid; 12+ contacts, 4 power, 7 balance [938] | 40 min recharge [C4ISRNET] | Over 700 lb; ~5 ft cube [C4ISRNET] | US10507938B2, granted 2019, active to ~2037. Pushers carry a toothed connector into the drone once a landing is detected [938] |
| H3 Dynamics (DBX method not published) | One motor drives four beams in; they stop when foot electrodes touch the plates [438] | Two 300 x 100 mm plates, 48 mm each side of center; + and - on diagonal feet; a polarity switch [438] | Not published | Not published | US11912438B2, granted 2024, active to ~2041: plates plus four pushing beams (abstract) [438]. US11168487: an inclined overhead guide [H3] |

DBOX (DroneTeam) presses packs home with a 3D-printed arm [DBOX]. Heisha chose pads over arms for the D135 (dock report) but now claims a T200 swap: a full pack in about 2 min vs 40 min charging [EIN]. Skysense charges through two spring contacts at any pose [Drone Below].

**Swap patents with no product found**
- DJI EP3412568 (B1 2020) [EP568; EP568 B1]: arms push the landed drone to a swap zone; the rising carriage pushes sprung hatch doors open. An XYZ robot's two-arm grabber pulls the pack by its end. The carriage holds a full pack too, so one trip swaps both; 8 slots. A1 claim 1: take a pack from a slot, insert it, release.
- SkyyFish US11807130B2 (granted 2023): a rack-driven piston pushes a new pack through the drone's bay, shoving the flat one into an empty bay. Sprung contacts snap into recesses as latch and power path. Under 1 min [130].
- Sony US10850838B2 (active to ~2038): a concave pad centers by gravity. The old pack drops through a hole, and a stand lifts the new one to the drone's electromagnet [Sony].
- Krauss and Kliner US20200165008A1 (filed 2016, grant not found): the hovering drone lowers a magnetic weight on a winch line into a raised cone of flaps. A magnet traps it and the winch pulls the drone down [008].

**Academic rigs**
- MIT ACL / Boeing (2011, 2015): two 4-bay drums, hot swap [DSpace]; over 100 swaps in a 3 h, three-drone mission [ITU]. The lab page says 7 packs and over 90 swaps [ACL]. No swap time found. Lee et al. (ICUAS 2015): 60 s, 4 packs [arXiv; S-SERQ].
- Oulu (2024): an arm sets the drone in a printed jig and pulls the pack out the rear. Mean 118.27 s; 5 failures in 200 runs [Oulu].
- U Twente (ICRA 2018): the arm sets the drone on two rails and a short funnel; a switch detects it, two clamps lock it. A tongue hooks a notch in the pack carriage and pulls it into a Geneva-indexed magazine [Twente].
- IIT Bombay (2024): S500, NEMA 17 and GT2, like the box. Belt-driven plates slide into the legs; load cells confirm landing. A lead screw on two carbon rods pushes the pack up to trip push latches; a magnet holds it as it comes away [ICAS].
- IPN (2023): inverted-V guides take ±0.3 rad yaw. Under 10 s target, since its ESCs lose calibration unpowered (DShot needs none, E) [S-SERQ]. KAIST (2011) was not reachable [KAIST]; an inverted dock (2022) only estimates ~1.5 min [Inverted].

Failures reported: Oulu's 5 in 200 (causes not read), IIT Bombay's dock board flexed [ICAS], IPN's 10 s ESC limit. No product publishes failure modes.

## Steal for Nox
- Overseer cartridge: hide the latch release in two cone-mouthed bores only the pusher's pins reach; hold the pins spread until the pack is clear (Airobotics, E).
- Overseer cartridge: with spring-pin contacts, the unlatched pack drops onto the pusher head by gravity and pin preload, so removal pulls nothing on the iris (Sony, E).
- Swapper (Overseer): DJI's two-pack carriage backs the twin slide in `catcher-speed-research.md` §6: pull, index, push in one trip.
- Swapper (Overseer): a magnet in the pusher head carries the pack down (IIT Bombay, E).
- Gripper (Scout): Airobotics' filing uses the jaw's order, release the latch with the grip stroke and keep it open through the first lift [085]; its 1 mm cone entries back the frustum nub (Open 6, E).
- Scout v2 cartridge: two cone bores on top locate the gripper and hide the release, so the jaws need no side room in the cage (Airobotics, E).
- Scout v2 cartridge: sprung contacts that snap into recesses as latch and power path (SkyyFish, E).
- Box catcher: close the iris at low current to center, then raise it to hold (Percepto, E).
- Box catcher firmware: bar gap from the step count at stall; ring width means caught (Percepto, E).
- Box catcher: a load cell under the head confirms the drone's weight after thrust cut, no camera needed (IIT Bombay, E).
- Box keep-alive: stop closing on pin continuity (H3, E).
- Test plan: 200 timed swaps, each failure cause logged [Oulu].

## For Spark
- `box.md` Overseer swapper: "pushes a full one up (~30 N vs ~20 N drone weight". Push latches like IIT Bombay's release on any upward push, so a bump from below could drop the pack, against the README breaking order (battery retention never breaks). For X9, use a pin-keyed release and size the hold for README P3 "Flight 4 g vertical", ~25 N for a ~0.65 kg pack (E).
- `box.md` Brief leaves Done unset; the Skydio report proposes "20 clean catches in a row". Zero failures in 20 runs only bounds the failure rate under ~15% (rule of three, E); 60 clean runs bound it under 5% (E).
- `catcher-speed-research.md` §1 lists Hextronics "Universal / Global | 80 s / under 90 s". Drone Nerds gives Global under 5 min to refuel, so sources disagree. Add Atlas 60 s and 150 s land to launch [Advexure], Airobotics under 1.5 min [Janes] and Oulu's measured 118 s [Oulu].
- IP (E, not legal advice): Percepto US10507938B2 (connector on a centering pusher) and H3 US11912438B2 (beams stop on electrode contact) sit near iris bars carrying keep-alive pins; the iris has no landing surface, teeth or plates, likely differences. US20200165008A1 is hover-capture prior art the patent report did not list; flag it to counsel before any provisional. Add all three to queue item 12.

## Sources
- [578: Airobotics Ltd patents, incl. US20190202578 (Justia)](https://patents.justia.com/assignee/airobotics-ltd)
- [085: US20190127085A1, Payload exchange facilitating connector (Justia)](https://patents.justia.com/patent/20190127085)
- [008: US20200165008A1, Centering and landing platform for hovering flying vehicles (Justia)](https://patents.justia.com/patent/20200165008)
- [Kliner: Meir Kliner filings (Justia)](https://patents.justia.com/inventor/meir-kliner)
- [New Atlas: Airobotics system](https://newatlas.com/airobotics-system-drones/43985/)
- [Janes: Aero A2Z and Airobotics drone-in-a-box](https://www.janes.com/defence-intelligence-insights/defence-news/air/aero-a2z-airobotics-partner-to-manufacture-autonomous-drone-in-a-box-solution)
- [Drive: Airobotics system (The Drive)](https://www.thedrive.com/tech/4113/this-new-drone-system-flies-recharges-and-swaps-payloads-without-human-help)
- [TechCrunch: Airobotics raises $28.5M](https://techcrunch.com/2016/06/21/airobotics-raises-28-5m-to-pursue-autonomous-drone-platforms-for-heavy-industry/)
- [Advexure: Hextronics Atlas](https://advexure.com/collections/drone-docking-stations/products/hextronics-atlas)
- [Drone Nerds: Introducing Hextronics](https://enterprise.dronenerds.com/?p=4822)
- [Justia H: patent search for Hextronics](https://patents.justia.com/search?q=hextronics)
- [Melnyk: Autonomous drones for infrastructure inspection 2026 (blog)](https://vadmelnyk.com/blog/autonomous-drones-critical-infrastructure-inspection-2026)
- [938: US10507938B2, Percepto, anchoring a UAV on a ground station (Google Patents)](https://patents.google.com/patent/US10507938B2/en)
- [C4ISRNET: What a drone in a box learned from the Army](https://www.c4isrnet.com/unmanned/2019/03/14/what-a-drone-in-a-box-learned-from-the-army/)
- [438: US11912438B2, H3 Dynamics landing platform with charging (Google Patents)](https://patents.google.com/patent/US11912438B2/en)
- [H3: H3 Dynamics patents (Justia)](https://patents.justia.com/assignee/h3-dynamics-holdings-pte-ltd)
- [DBOX: SolidWorks DBOX case study (PDF)](https://www.solidworks.com/assets/invest/2024-10/solidworks_dbox_case-study.pdf)
- [EIN: Heisha T200 release](https://www.einpresswire.com/article/675792796/heisha-unveils-t200-dual-drone-charging-solution-with-battery-swapping-system)
- [Drone Below: Skysense automated charging at Interdrone](https://dronebelow.com/?p=9171)
- [EP568: EP3412568A1, UAV base station system and method, DJI (EPO)](https://data.epo.org/publication-server/rest/v1.2/patents/EP3412568NWA1/document.html)
- [EP568 B1: EP3412568B1 (EPO)](https://data.epo.org/publication-server/rest/v1.2/publication-dates/20200603/patents/EP3412568NWB1/document.html)
- [130: US11807130, Battery exchange and charging system for drones (Justia)](https://patents.justia.com/patent/11807130)
- [Sony: US10850838B2, UAV battery form factor and insertion/ejection (Google Patents)](https://patents.google.com/patent/US10850838)
- [DSpace: Michini et al. 2011, Automated battery swap and recharge (MIT)](https://dspace.mit.edu/handle/1721.1/81474?show=full)
- [ACL: Automated battery management system (MIT ACL)](https://acl.mit.edu/projects/automated-battery-management-system)
- [ITU: Ure et al. 2015, IEEE/ASME Transactions on Mechatronics](https://research.itu.edu.tr/en/publications/an-automated-battery-management-system-to-enable-persistent-missi/)
- [arXiv: UAVs as mobile infrastructure, addressing battery lifetime](https://arxiv.org/pdf/1807.00996)
- [S-SERQ: Low-cost battery exchange for a quadrotor (MDPI Drones 2023)](https://www.mdpi.com/2504-446X/7/4/270)
- [Oulu: Vilkki et al. 2024, jig-assisted battery exchange (MDPI Machines)](https://www.mdpi.com/2075-1702/12/11/818/htm)
- [Twente: Barrett et al. 2018, battery exchange with a mobile ground base](https://ris.utwente.nl/ws/files/103952838/autonomous.pdf)
- [ICAS: Kute et al. 2024, docking and battery swapping in UAVs](https://www.icas.org/icas_archive/icas2024/data/papers/icas2024_1116_paper.pdf)
- [Inverted: De Silva et al. 2022, inverted docking station (MDPI Drones)](https://www.mdpi.com/2504-446X/6/3/56)
- [KAIST: Automatic battery replacement system for UAVs, ICUAS 2011](https://koasas.kaist.ac.kr/handle/10203/169117)
