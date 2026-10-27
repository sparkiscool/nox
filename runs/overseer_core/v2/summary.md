# Overseer core v2: concept (system pass), 27 Oct 2026

Spark 27 Oct: "the body needs to also include the part that the nodes from the lidar goes to, this should be everything except the arms and the nodes"; "check if a run is already using the camera mount" (yes: the cage session's head_mount v1, L2 + both cameras, left alone); "we didnt need to generate the part thats on the battery, the drone side just have an empty rectangle slot for now".

Script `nox/overseer/core_v2.py`, pictures `nox/overseer/core_v2_view.py` -> `preview.png` and `alternatives.png` (A / B / C, plan), artifact https://claude.ai/artifact/JSamk2HosoRiVZgo3KHBH3, checks `checks.txt`, `system_check.json`, `result.json`.

## Layout (chosen, A)
- Hub deck (one PETG print on the bottom plate): Pi 5 posts, UBEC + buck bays, and the keel front hub: 2 sockets 6.2 x 15, boss Ø13, on the head keels' axes from the v5 roots (96, ±12, 8). 6 feet with M3 inserts (Ø4.0 through) on the plate's 360° holes (-24, ±36.5), (65, ±22.5), (96, ±17), every head outside the slot; each keel closes a triangle (96, ±17)-(65, ±22.5) + the cross bar at x 96; M3 up from under the plate.
- Flow clip (PETG, drawn, `print/overseer_flow_clip_v1.stl` 5.0 g, solid_check PASS): MTF-02P under the front wing at (94, 0), walls and floor 3.0, a 3.0 flange under the deck's 2 front M3 x 10 (only 3 of PETG in the keel joint's clamp).
- Battery: empty slot x -121.8..61.9, y ±33.4, z -76.9..-2.1; latch set masses kept for balance.
- FC stack: unchanged (top plate, turned 45° into its slots).

## Alternatives weighed
- B: a separate front block for the keels on (96, ±17) + (60, ±17), deck as v1: 3 pieces, 8 M3, 4 bolts within 36 of each keel root; but (60, ±17) heads land in the slot's reserved top. A now takes B's strength in one piece: 4 front feet (96, ±17) + (65, ±22.5) triangulated, all outside the slot.
- C: flow on the deck, looking down through the plate: the plate has no opening near the front that clears a 42° cone, and under the plate the slot blocks any view aft of x 92. Rejected.
- Keel bolts (both keels push 387 N, F): rigid deck 128 N a bolt over the 6; compliant-deck bound (the front 4 feet alone) 212 N a bolt (E). Preload is capped by the insert's ~600 N pull-out (E), so friction holds ~120 N a bolt (x 0.2): flight loads (keels <= ~40 N, E) stay in friction; a crash slips the Ø3.5 front holes' 0.25 radial play and runs in bearing, ~34 MPa on the carbon plate and ~9 MPa on the insert (E), within both. DEFAULT: accept the 0.25 play (alternative: ream the two front holes to Ø3.1 is not possible, a sleeve Ø3.5/3.0 is). The topology run reports the FE bolt forces.

## Checks (C on E)
x arms, Pi, bays, stack, slot: 0 mm³; flow view clear; access 35/35; keel tubes inside the bores (cut in the run). Fit FAIL 26.5 mm³: the flow clip's nose and the cage's parked front core-tie root (100.5, 0, -1) (body nodes parked by Spark; asked the cage to keep off). Probes FAIL at -X (slot unshielded) and mass 147 % are the cage's / whole drone's, as in v1.

## Concept review 27 Oct: FAIL -> fixes
1. Alternatives drawn: `alternatives.png` (A / B / C plan, per-bolt numbers on A and B).
2. Keel load path: feet moved to (65, ±22.5) beside (96, ±17), triangle sides and cross bar drawn in the concept body; compliant bound 212 N a bolt.
3. Clip in the clamp: the clip's pads are a 3.0 flange (3.0 is the solid gate's neck; 2.0 failed it), M3 x 10.
4. A vs A+B: done in one piece (2 above).
5. Slot top: the rear feet moved off (-65, ±22.5) to (-24, ±36.5); every head clears the slot (`checks.txt`: heads x slot 0.0 mm³); access no longer ignores the slot top.
6. Clip mass 5.0 g (M) on its 5 g slot.
7. Cage ask carries the numbers (nose tie root 98.1..102.9, z -3.9..1.9 vs MTF-02P to x 104.8, clip to x 106.8); fallback on our side (sensor turned 90° at x 92) kept in reserve.
Smoke (CLAUDE.md Measure before quoting) found two setup faults before any run: the keel bosses had no room to be joined (hub room x 80..118, |y| 20, z 0..16 opened) and the zip-tie tunnels cut the deck in two at x 68 (now channels on the plate face, 4.0 above).

Re-review 27 Oct: PASS. Nits fixed: bearing case above, previews `runs/overseer_core_coupons/v2/*_preview.png`, cage ask numbers (clip to x 107.8, 28.0 mm³), C title wrapped, docstring feet.

## Print files (solid_check PASS on each, print_orient)
- `print/overseer_flow_clip_v1.stl`: 5.0 g, floor down, bed 507 mm², overhang 234 mm², 9.8 tall; preview `runs/overseer_core_coupons/v2/overseer_flow_clip_v1_preview.png`
- `print/overseer_core_coupon_keel_hub_v2.stl`: 13.0 g (M), feet Ø14 as the deck, plate face down, bed 982 mm² (M), overhang 484 mm² (M) (the bosses past the seats: tree supports), 17.6 tall; preview `runs/overseer_core_coupons/v2/overseer_core_coupon_keel_hub_v2_preview.png`
