# Overseer core v3: concept (system pass), 27 Oct 2026

Ask (cage + arm sessions, 27 Oct): the 4 L2 braces move off the front arm beams onto the core; 6.2 x 15 sockets (+0.6 lead-in, 2.5 walls); load 1066 N a side summed along the braces in the top hit at the top-plate roots, 712 N at tower roots (F on E, no SF; cage frame model). Script `nox/overseer/core_v3.py` (v2 + the braces), pictures `nox/overseer/core_v3_view.py` -> `preview.png`, `alternatives.png`; checks `checks.txt`, `result.json`.

Artifact (layout and alternatives): https://claude.ai/artifact/JSamk2HosoRiVZgo3KHBH3

## Layout (chosen, B; round 2 after the concept review)
The hub deck's front hub becomes a tower beside the keel bosses (concept body x 86..112, |y| 9..31, z 0..38) carrying 4 brace sockets, tube ends lo (97, ±26, 22), hi (97, ±14, 32), aimed at the head lugs lo (379.7, ±46, 54.2), hi (383.3, ±46, 67.8). **Second chord (review 1):** a strut a side from the tower back and up over the UBEC / buck to a pad (x 42..50, |y| 12..20, z 41..45) bearing up on the top plate's underside, inside the arm-free front wing (the front arm roots fill the diagonals between the plates in to (26, ±26)). One print with the hub deck; the 6 feet unchanged.

## Alternatives on one basis (review 2: SF 1.1, the cage frame model's per-case root forces `core_root_forces.json`, worst case each)
- A, a block on the top plate's front edge (roots x 53..57, z 50): 765..788 N on each of 4 fasteners in the 1.6 top plate (584 shear). Rejected.
- B, tower alone on its feet: the 2 front screws pried with up to 9.7 kN (toe lever 4..8 mm); with the floor bars to the (65, ±22.5) feet, 684 N (628 pull-out, over an insert's ~600, E). Not sound alone (the review's finding).
- **B + pads (chosen):** forward tip (top hit, crash 4): 343 N pressing each pad up on the carbon (5.4 MPa on 8 x 8); back tip (front hit, crash 0): 13.8 N m a side, ~445 N pull on each front insert over 31 mm to the mid feet (E, the tight spot: the run's FE reports it); 6 feet <= 247 N (rigid). Cost on the cage side: head 198 -> 166 Hz, shell tube 1.68 -> 2.17 (theirs).
- Roots lower (z 8/14): front 4 feet 347 N; costs the head's pitch triangle (cage: below 166 Hz). Kept in reserve.

## Checks (C on E)
Tower + struts + pads x arms, top and bottom plate, UBEC, buck, Pi, wires, props, cage nodes and tubes: 0.0 mm³; links: 170.7 mm³, all of it the keels and braces at their own roots (links minus those tubes 0.0, review 5). Fit FAIL 28.0 mm³ (flow clip vs the cage's parked front tie node) and probes / mass FAILs as v2 (other sessions').

## Breaks in the right order (review 6)
The deck now holds 6 epoxied tubes (2 keels, 4 braces). DEFAULT: the sockets are sized at index <= 1.0 crash like the deck; the cheap parts that should go first are the cage's head-end nodes and lugs (cage session, asked). Alternative: bolted socket inserts in the tower so the deck comes off without cutting tubes (+4 screws). Open: the cage's node indices vs the deck's.

## Hub deck v6 (what the run loads, review 7)
Cases: each crash k (35 g x 14) as one set, the payload at 35 g + that case's keel and brace forces from `core_root_forces.json` (crash class SF); flight 4 g with its root forces; 2 g x / y, landing 6 g; USB 20 N; press 50 N. Supports: the 6 feet (screws + inserts); in the forward-tipping cases also the 2 top pads (compression: a separate support group per case). Design room: the deck layer, the hub room x 80..118, |y| <= 34, z 0..40, and the strut room x 42..90, |y| 12..22, z 17..45 over the bays. Modes: `hub_deck_modes.py` (+ the brace sockets) after the run. Mass: slot 55 g (E); the summary will say what the tower's grams buy.
