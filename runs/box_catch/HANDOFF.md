# Box catch concepts: handoff (27 Oct 2026, design center session)

Session `session_01EtHMZMB13ed4juqR8ktuXy`, branch `claude/clever-edison-w5z7us`, PR https://github.com/sparkiscool/nox/pull/125 (open, not merged; concept review FAIL twice, the hook concept not yet reviewed). Everything here is concept stage: no saved part, no print file. The box catch session owns the catch; this work hands it the concept.

## The task as Spark gave it (in order, 27 Oct)
1. Scout bottom looks bad and wastes performance: make it compact without losing structure.
2. "If we go with B how will the Overseer fit": it does not (two head stops 61 apart, `bottom_concept.py` overseer_fit).
3. "A funnel that works for both and only depends on the bottom opening": posts into a shared rim (D), then "no rim, ramps clear all current cages" (E).
4. "Scout matches the rectangle part of the Overseer" (the 4 side nodes of its base loop), ramps to those; confirm versions with both sessions (done).
5. Servo hook levers that pop up after a flat landing, then one servo, then **a stepper: "design it so we have the most optimal centering mechanism possible"**. Keep-alive contacts dropped.
6. Materials: PETG nodes (rule on main), sockets 6.2 / 3.2 (rule on main, PR 135); every saved part gets 6.2 / 3.2 holes, built parts re-cut, never a run restart.

## Current direction (DEFAULT, from Spark's last design message)
Hook catch, one stepper: drone lands on a flat deck (claws folded flush), a NEMA 17 on the SKR Mini E3 V3 spare **E driver** (TMC2209, StallGuard for seat sense) turns a **T8 x 2 single-start lead screw** (self-locking: holds with the motor off, no over-centre) that pulls a **lift frame** under the deck round the swap column; **sprung links** from the frame turn each claw in its own plane; claws pull the drone into V-seats and lock it. One NEMA 17 to buy (the 3 on hand are the arm's). Seat sense: frame position (step count + one hall at locked) and the stall.

Measured on the real cages (`catch_ramps_concept.py` loaders, seat frame = catch centre, z 0 = the Overseer loop plane):
- **Corner nodes cannot be grabbed:** on both cages shell rods leave each corner node inward at ring height, 3 to 5 mm from a claw's sweep.
- **Grab ring tubes mid-span:** Scout long sides clear s 22..126 of 155; Overseer long sides have a link (part "spokes, ties, keels, L2 guard ring") just inside the tube (w +5..8, z 5..10) for s 19..136, so a claw must hook from outside over the top with ~2 mm to spare. Scout end tubes clear 118 (nose) / 205 (tail) on the Scout, but inside the Overseer's opening (its feet, receiver, pods). Overseer tip tubes: the Scout is clear there; the Overseer's own rods crowd near the tip nodes.
- Proposed layout (not checked): 2 claws per long side (y + yaw, shared), x from claws on the Scout end tubes and the Overseer tip tubes; every claw rises on every landing, so each must clear the drone it is not for.

## Fallback: ramps (concept E, checked)
`nox/box/catch_ramps_concept.py` (+ view `catch_ramps_view.html`, published https://claude.ai/artifact/F5b6L8uScqbaDSsXPXTSmz): 2 shared side ramps (25 high, 20 clear of the nodes), 1 chamfered pad per Overseer tip edge (25 high, from 35 to 80 along the edge), Scout end pads |y| 52..96 at both ends (20 high). Kinematic drop check (first contact the ring, net push inward, slide clearance 1.0), yaw ±2.5: **Scout ±20 fore-aft / ±25 side / 25 diag; Overseer ±25 / ±25 / 40**. At ±3.5° yaw the Overseer's tail-tip tubes cross onto the wrong face. The what-if run (Overseer pods inside |y| 46, side ramps 35) was stopped at its time limit: not known.

## Versions (both on main now)
- Overseer cage v5 (PR 122, merged): `nox/overseer/cage_v5_shell.json`, `nox/overseer/overseer.assembly.json`; base loop z −57.0 (cartridge bottom −3), nodes r 10.15, tubes r 3: (−98.9, ±119.7), (55.8, ±116.1), (161.2, 0), (−242.6, 0) in drone x; swap centre x −30. PR 123 will lower its belly ~20 (loop follows the cartridge). Belly near the loop: sled |y| 55 (9 above the loop), flow pod |y| 50, TPU feet at (rel ±68, ±34) 4 under the loop.
- Scout v11 (PR 133, merged, dea0b51): `runs/scout_layout/v11/`, `nox/scout/v3/scout.assembly.json`; catch x −3.5, plane z −49.62 (cartridge bottom −44.5 − 3 − 2.12: its r 1.5 ring sits 2.12 lower on a 45° face than the Overseer's r 3, so both cartridges seat level); bottom nodes on the hexagon's side nodes (85.8, ±116.1), (−68.9, ±119.7) about the catch. **v12 in review (PR 141): catch 8.5 forward, loop moves with it, nose low node (106.7, 0, −19.0): switch `SCOUT_REF` / `SCOUT_DIR` / `SCOUT_CATCH_X` in `catch_ramps_concept.py` when it merges.**
- Loader constants: `SCOUT_REF`, `SCOUT_DIR`, `SCOUT_CATCH_X`, `SCOUT_PLANE_FIX`, `OVR_REF` at the top of `catch_ramps_concept.py` (git refs, read with `git show`).

## Files (branch)
- `nox/scout/v3/bottom_concept.py` + `bottom_concept_view.html`: Scout bottom layouts A..E, Overseer fit, `runs/scout_bottom/concept/` (A today, B shell catch, C belly crown, D rim on posts, E rectangle ring).
- `nox/box/catch_posts_concept.py`: D (posts into a 220 rim), `runs/box_catch/posts_concept/`.
- `nox/box/catch_ramps_concept.py`: E on the hexagon, modes `--html` (rebuild the page) and `--scout` (Scout-only rerun), env `YAW_TEST`, `TIP_MODE`, `TIP_PAD0`, `RAMP_*_H`; `runs/box_catch/ramps_concept/`.
- `nox/box/catch_hooks_view.html` → `runs/box_catch/hooks_concept/hooks.html`, published https://claude.ai/artifact/GiKhw8MwEfpqMDCC6N52zk (still shows one servo and an over-centre lock: update to the stepper + lead screw).
- A full ramp sweep takes ~15 min for both drones and both configs on this box; run one at a time (`run_in_background`), and kill with `pkill -f "^python3 -u nox/box/catch_ramps"` (a bare `pkill -f catch_ramps` kills the calling shell).

## Next (in order)
1. Hook page: stepper + T8 x 2 lead screw (no over-centre), keep-alive gone; republish.
2. Claw model: claw = hook arm on a horizontal pivot under the deck in its pull plane; sweep it through its full travel and check against both drones (`load_drone` points) at seated and at landings ±40, yaw ±10: only the target ring tube may be touched; capture = the offsets whose tube lies in the claw's mouth. Size claw count, places and mouth widths; settle the Overseer's x (tip-tube claws or side claws on reachable tubes).
3. Concept review (review agent, concept mode) on the hooks with ramps as the alternative; fix; PASS.
4. PR 125: body update, check_state, merge on PASS; hand the box catch session the requirements (it draws the frame, claws, seats, retires panels v7 / v9 plain and hook latch v2).

## Sessions
- Scout v3: `session_01EtHMZNhc68Ghbc2KCNMUzU` (tells me each Scout version; asked for: ring heights, ties out of the band, tail low node gone, all done).
- Overseer cage: `session_01Em8yMe1ArHE9XePmNwV7RS` (confirmed v5 loop; offered belly pods inside |y| 46 if the catch needs it: send the limit).
- Spark's run rule: never restart a run for a constant change; re-cut a built part on its result.
