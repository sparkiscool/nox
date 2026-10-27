# Overseer core v3: concept (system pass), 27 Oct 2026

Ask (cage + arm sessions, 27 Oct): the 4 L2 braces move off the front arm beams onto the core; 6.2 x 15 sockets (+0.6 lead-in, 2.5 walls); load 1066 N a side summed along the braces in the top hit at the top-plate roots, 712 N at tower roots (F on E, no SF; cage frame model). Script `nox/overseer/core_v3.py` (v2 + the braces), pictures `nox/overseer/core_v3_view.py` -> `preview.png`, `alternatives.png`; checks `checks.txt`, `result.json`.

Artifact (layout and alternatives): https://claude.ai/artifact/JSamk2HosoRiVZgo3KHBH3

## Layout (chosen, B; round 3 after two concept reviews)
The hub deck's front hub becomes a tower beside the keel bosses (concept body x 86..112, |y| 9..31, z 0..38) carrying 4 brace sockets, tube ends lo (97, ±26, 22), hi (97, ±14, 32), aimed at the head lugs lo (379.7, ±46, 54.2), hi (383.3, ±46, 67.8). **Second chord (reviews 1-2):** deep side webs from the tower back to the mid feet beside the UBEC / buck (x 58..92, |y| 18..34, z 0..40; the bays' columns stay open to place them straight down; the arm roots fill the diagonals between the plates, the arm-free wing is |y| <= x - 24), and 2 more feet at the plate's free Ø3.5 holes (60, ±17): tower, web and the 4 mid / front feet a side close a triangle both ways. Round 2's pads under the top plate are dropped (they bore one way only, bent the 1.6 plate ~450 MPa, E, and their struts blocked the buck bay). One print with the hub deck; 8 feet.

## Alternatives on one basis (SF 1.1, the cage frame model's per-case root forces `core_root_forces.json`, worst case each)
- A, a block on the top plate's front edge (roots x 53..57, z 50): 765..788 N on each of 4 fasteners in the 1.6 top plate (584 shear). Rejected.
- B, tower alone on its feet: the 2 front screws pried with up to 9.7 kN; with only the floor bars to the mid feet 684 N (628 pull-out). Rejected (review 1).
- B + pads under the top plate (round 2): forward tip only; plate bending. Rejected (review 2).
- **B + deep webs to 4 mid feet (chosen):** forward tip 19.9 N m a side (top hit) -> ~240 N on each mid insert; back tip 13.8 N m (front hit) -> ~412 N on each front insert over 33.5 to the mid feet (E; an M3 x 5.7 insert in PETG ~600 N pull-out, E: a coupon measures it, P); rigid 8 feet <= 242 N. Cost on the cage side: head 198 -> 166 Hz, shell tube 1.68 -> 2.17 (theirs).
- Roots lower (z 8/14): 347 N on the front 4; costs the head's pitch triangle. Kept in reserve.

## Checks (C on E)
Tower + webs x arms, top and bottom plate, UBEC, buck, Pi, wires, props, cage nodes and tubes: 0.0 mm³; the 2 new heads at (60, ±17) sit in the battery slot's reserved top 6 (pockets Ø7 x 3.5 the receiver mount clears, DEFAULT); links: 170.7 mm³, all of it the keels and braces at their own roots (links minus those tubes 0.0, review 5). Fit FAIL 28.0 mm³ (flow clip vs the cage's parked front tie node) and probes / mass FAILs as v2 (other sessions').

## Breaks in the right order (review 6)
The deck now holds 6 epoxied tubes (2 keels, 4 braces). DEFAULT: the sockets are sized at index <= 1.0 crash like the deck; the cheap parts that should go first are the cage's head-end nodes and lugs (cage session, asked). Alternative: bolted socket inserts in the tower so the deck comes off without cutting tubes (+4 screws). Open: the cage's node indices vs the deck's.

## Hub deck v6 (what the run loads, review 7)
Cases: each crash k (35 g x 14) as one set, the payload at 35 g + that case's keel and brace forces from `core_root_forces.json` (crash class SF); flight 4 g with its root forces; 2 g x / y, landing 6 g; USB 20 N; press 50 N. Supports: the 8 feet (screws + inserts). Design room: the deck layer, the hub room x 80..118, |y| <= 34, z 0..40, and the web room x 58..92, |y| 18..34, z 0..40 (the bays' columns cut, the arms cut). Modes: `hub_deck_modes.py` (+ the brace sockets) after the run. Mass: slot 55 g (E); the summary will say what the tower's grams buy.

## Concept review round 2 -> round 3
1. Back tip: closed by the deep webs to the 4 mid feet (the pads are gone); the insert pull-out is measured on the keel hub coupon v2 (its 4 inserts: pull an M3 out with a spring scale or weights, P) before the deck is called ready.
2. Top plate under pads: no pads now; nothing of the core loads the top plate.
3. Access: the bays' columns are cut from the design room (`core_topo_v3.py` bays), so the UBEC and buck still go straight down onto the deck.
4. Outside the core's rows: `dimensions.md` System Overseer v5 "Arm requirements" still roots the L2 braces in the front beam (72.95, ±62.11): the cage and arm sessions' rows (named in the PR).
