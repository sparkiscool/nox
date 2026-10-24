# Swap arm v7 system pass (round 3d, grip line Y_C 167.1 for latch v4, hub v7, concept check PASS; recheck with heads v7's plate top 74.35; 24 Oct 2026, swap arm system worker)

System: box swap arm v7, assembled worst index 0.49 in the Overseer insert at the drone 60 N (16T on the rack tail; 0.487 at Y_C 167.1, F; was v6's 0.44 in the Scout drone insert), fit 0.0 mm³ at 10 poses (M), mass 6534 g (E: the assembly at park with all 9 receivers (latch v4), the seated packs and both carried packs; Overseer cycle 6865 g, E; turning parts 1620 g, Scout cycle, E; v6's 3232 g (E) had the demo bay only, not like for like; no budget_g for the assembly), CG on the axle within 11.7 mm, unbalance 18.96 g·m at park (E; worst over the cycles: the Overseer cycle bay pick, 1.044 N·m). Rails v5 (wide ribbon channel, bolt heads under it) since the plate 74.35 recheck: −3 g, nothing else moves.

- **Grip line Y_C 167.1 (24 Oct, DEFAULT, latch session agreed):** latch v4's Overseer receiver v2 lead end is 2.15 longer (its 3.0 capture); the trim it would allow needs ±0.7 along Y and the arm holds ±1..1.8 (E), so Y_C moves 165 → 167.1 (Y_C_min 167.1, F). Rerun with hub v7's mesh: sweeps 0 overlaps on both cycles, fit 0.0 at 10 poses, access 9/9 (M); swap 0.448 / 0.487, XT60 x 1.28° / 1.07° (stub allowed 10.8 / 14.4 mrad), z 0.74° / 0.85°, unbraked power cut 1.34 / 2.80 (F). Margins at the floor again: Y_C = Y_C_min.
- **Recheck cost (plate top 74.35, DEFAULT):** the margins the plate took are at their floor: Y_C 165 = Y_C_min 165.0 (C), and the sweep's closest pairs axlesA / rec−180 and axlesB / rec0 read 1.000 (M, `sweep_log_v7_overseer_-180_0.txt`; headA / rec−180 and headB / rec0 1.000 in `sweep_log_v7_scout_-210_30.txt`). Any growth of the carriage stack in Y now moves Y_C (alt: keep the plate at 72.75 and give heads v7 1.6 less, its ask refused).

- **Model:** `nox/box/swap_system.py` (NOX_ARM v7). Layout: `arm_v7_layout.py` (LOW stack, Y_C 167.1 (was 165), BAYS, 5:1, detent x1.3). Latch: main's `battery_latch.py` (v4).
- **Assembly:** `nox/box/box_swap_arm.assembly.json`, with the Overseer cycle in `runs/arm/v7/box_swap_arm_overseer.assembly.json`. Latch 258becf, staged read-only.
- **Results:** `system.json` (frame, both packs; XT60 per axis; capture; unpowered arm; sets on the grip). `all_log.txt` holds the stack, alternatives, frame, options, Overseer and system_check runs.
- **Sweeps:** `sweep_log_v7_scout_-210_30.txt`, `sweep_log_v7_overseer_-180_0.txt`, both 0 overlaps (C). Access 9/9 at the bay pick and return (C).
- **Requirements:** `nox/dimensions.md` "System box swap: arm".

**Worst index per class (E):**

| Class | Scout | Overseer | Notes |
|---|---|---|---|
| swap | 0.45 | 0.49 | the 16T at the 60 N drone insert, on the rack tail |
| misuse, free | 0.57 | 0.61 | the 16T at the 74 N stall |
| handling | 0.29 | 0.43 | |
| sustained | 0.21 | 0.27 | |

- **Power cut carrying, unbraked:** 1.29 / 2.73 (625ZZ, E). The head brake is required for both packs; the gate is a bench drop plus a self-test at every boot.
- **Seat state:** Scout ≤ 0.42 mm / 0.42°, Overseer ≤ 0.42 mm / 0.43° (E; target 0.5 / 0.5, DEFAULT).
- **XT60 per axis** (+ the arm RSS 0.41°, E; floats C, latch):
  - About x: 1.25 / 1.04° (E) vs ±2.37° (C). The stub may tilt 11.3 / 14.9 mrad (E) at a 20 % margin (DEFAULT), so the requirement is ≤ 10 mrad (DEFAULT).
  - About z: 0.73 / 0.84° (E) vs ±1.06° (C), with the per-bay gravity offset (required) and the receiver's x side faces.
- **Capture:** 0.55 of the ±3 lead-ins (E; lead-ins C), 0.34 with the gravity offset (E). The imposed offset is 7.6 N / 1.05 N·m on the receiver (E).
- **Unpowered arm** (detent v6, springs x1.3, hold 1.065 N·m, E):
  - index past a notch at the slow pass 1.28 (1.51 on holding torque) (E);
  - power cut mid-swap carrying 1.20 (E);
  - a head just out of a horizontal Overseer bay 1.09 (E).
- **Compact, arm alone (M):**
  - Scout cycle: park 11.3 L, 228 x 210 x 236, largest 236; worst 12.5 L / 253.
  - Overseer cycle: park 11.6 L; worst 15.5 L / 266.

**Visuals:**
- Layouts: https://claude.ai/artifact/1fFDw6W66sMhcSjbZyd8TQ (Overseer) and https://claude.ai/artifact/TM1SSFJTYUVG4eYfdE9X4A (Scout), from `arm_v6_motion.py`.
- Alternatives: https://claude.ai/artifact/F7cY19HjCUZFY53HUB5k6d (`arm_v7_alternatives.py`).
- Heads v7 grabber concept: https://claude.ai/artifact/JWnuGJqAAvRXxATL5hyBya (`head_v7_concept.py`).

**Pieces (DESIGN, piece pass):**
- rails A / B v5 (`runs/arm_rail/v5/`; v4 superseded);
- pinion16 v3 (`runs/arm_pinion16/v3/`);
- detent v6: ring v4, arm v3 (`runs/detent/v6/`);
- hub v6 topology (run/hub-v6);
- heads v7 (from the requirements).
