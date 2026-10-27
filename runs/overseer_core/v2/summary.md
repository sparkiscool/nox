# Overseer core v2: concept (system pass), 27 Oct 2026

Spark 27 Oct: "the body needs to also include the part that the nodes from the lidar goes to, this should be everything except the arms and the nodes"; "check if a run is already using the camera mount" (yes: the cage session's head_mount v1, L2 + both cameras, left alone); "we didnt need to generate the part thats on the battery, the drone side just have an empty rectangle slot for now".

Script `nox/overseer/core_v2.py`, picture `nox/overseer/core_v2_view.py` -> `preview.png`, artifact https://claude.ai/artifact/JSamk2HosoRiVZgo3KHBH3, checks `checks.txt`, `system_check.json`, `result.json`.

## Layout (chosen, A)
- Hub deck (one PETG print on the bottom plate): Pi 5 posts, UBEC + buck bays, and the keel front hub: 2 sockets 6.2 x 15, boss Ø13, on the head keels' axes from the v5 roots (96, ±12, 8). 6 feet with M3 heat-set inserts on the plate's 360° holes (24, ±36.5), (-65, ±22.5), (96, ±17); M3 up from under the plate (screwdriver from below, the slot empty).
- Flow clip (PETG, drawn): MTF-02P under the front wing at (94, 0), clamped by the deck's 2 front screws.
- Battery: empty slot x -121.8..61.9, y ±33.4, z -76.9..-2.1; latch set masses kept for balance.
- FC stack: unchanged (top plate, turned 45° into its slots).

## Alternatives weighed
- B: a separate front hub block on (96, ±17) + (60, ±17) for the keels, deck unchanged: 3 pieces and 2 more screws; the keel loads reach the plate the same way. Rejected: more pieces (Spark: as few as possible).
- C: flow on the deck, looking down through the plate: the plate has no opening near the front that clears a 42° cone, and under the plate the slot blocks any view aft of x 92. Rejected.
- Keel bolts: 4 feet only (v1) carry 199 N a bolt when both keels push; 6 carry 129 N (E on F loads). Chosen 6.

## Checks (C on E)
x arms, Pi, bays, stack, slot: 0 mm³; flow view clear; access 35/35; keel tubes inside the bores (cut in the run). Fit FAIL 26.5 mm³: the flow clip's nose and the cage's parked front core-tie root (100.5, 0, -1) (body nodes parked by Spark; asked the cage to keep off). Probes FAIL at -X (slot unshielded) and mass 147 % are the cage's / whole drone's, as in v1.
