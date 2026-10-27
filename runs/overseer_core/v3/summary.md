# Overseer core v3: concept (system pass), 27 Oct 2026

Ask (cage + arm sessions, 27 Oct): the 4 L2 braces move off the front arm beams onto the core; 6.2 x 15 sockets (+0.6 lead-in, 2.5 walls); load 1066 N a side summed along the braces in the top hit at the top-plate roots, 712 N at tower roots (F on E, no SF; cage frame model). Script `nox/overseer/core_v3.py` (v2 + the braces), pictures `nox/overseer/core_v3_view.py` -> `preview.png`, `alternatives.png`; checks `checks.txt`, `result.json`.

## Layout (chosen, B)
The hub deck's front hub becomes a tower (x 86..112, |y| 9..31, z 0..38 concept body) beside the keel bosses, carrying 4 brace sockets with tube ends lo (97, ±26, 22), hi (97, ±14, 32) aimed at the head lugs lo (379.7, ±46, 54.2), hi (383.3, ±46, 67.8). One print with the hub deck (fewest pieces); the 6 feet unchanged; the keels' push and the braces' pull meet inside the tower.

## Alternatives weighed (`alternatives.png`)
- A: a block on the top plate's front edge (roots x 53..57, z 50; the cage's first ask): 4 fasteners in the 1.6 carbon top plate carry 765..788 N each (584 shear) at SF 1.1 (rigid, E); PETG keys in the plate's Ø6 holes bear ~64 MPa (PETG ~50) and the front motor leads cross there. Rejected: not sound.
- B: tower (chosen): rigid split 120..190 N a foot with the top-hit pairing (keels push + braces pull); front-4 bound <= 688 N (braces pull alone, E: a guess until the cage's per-case root forces arrive). Cost on the cage side: head 198 -> 166 Hz (>= 100), shell tube 1.68 -> 2.17 (theirs).
- C: roots elsewhere (the cage offered): B is the lowest the plates and arms allow ahead of the arm roots.

## Checks (C on E)
Tower x arms, top and bottom plate, UBEC, buck, wires, props, cage nodes and tubes: 0.0 mm³; the brace tubes only inside their bores. Fit FAIL 28.0 mm³ (flow clip vs the cage's parked front tie node) and probes / mass FAILs as v2 (other sessions').

## Next
Per-case keel + brace forces at the tower roots from the cage frame model (asked); hub deck v6 = v5's setup + the tower bosses and room, loaded case by case; smoke, run; FE reactions and modes (`hub_deck_modes.py`). Hub deck v4 / v5 (keels only) are superseded by v6.
