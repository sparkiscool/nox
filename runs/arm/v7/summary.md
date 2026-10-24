# Swap arm v7 system pass (round 3d, concept check PASS; 24 Oct 2026, swap arm system worker)

System: box swap arm v7, assembled worst index 0.49 in the Overseer insert at the drone 60 N (16T on the rack tail; was v6's 0.44 in the Scout drone insert), fit 0.0 mm³ at 10 poses (M), mass 5580 g (E: the assembly at park with all 9 receivers, the seated packs and both carried packs; turning parts 1611 g, Scout cycle, E; v6's 3232 g (E) had the demo bay only, not like for like; no budget_g for the assembly), CG on the axle within 11.8 mm, unbalance 19.1 g·m at park (E; worst over the cycles: the Overseer cycle bay pick, 1.04 N·m).

- **Model:** `nox/box/swap_system.py` (NOX_ARM v7). Layout: `arm_v7_layout.py` (LOW stack, Y_C 165, BAYS, 5:1, detent x1.3).
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

- **Power cut carrying, unbraked:** 1.30 / 2.75 (625ZZ, E). The head brake is required for both packs; the gate is a bench drop plus a self-test at every boot.
- **Seat state:** Scout ≤ 0.42 mm / 0.42°, Overseer ≤ 0.42 mm / 0.43° (E; target 0.5 / 0.5, DEFAULT).
- **XT60 per axis** (+ the arm RSS 0.41°, E; floats C, latch):
  - About x: 1.24 / 1.04° (E) vs ±2.37° (C). The stub may tilt 11.4 / 15.0 mrad (E) at a 20 % margin (DEFAULT), so the requirement is ≤ 10 mrad (DEFAULT).
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
- rails A / B v4 (`runs/arm_rail/v4/`);
- pinion16 v3 (`runs/arm_pinion16/v3/`);
- detent v6: ring v4, arm v3 (`runs/detent/v6/`);
- hub v6 topology (run/hub-v6);
- heads v7 (from the requirements).
