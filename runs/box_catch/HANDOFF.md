# Box catch concept: handoff (27 Oct 2026, design center session)

Session `session_01EtHMZMB13ed4juqR8ktuXy`, branch `claude/clever-edison-w5z7us`, PR https://github.com/sparkiscool/nox/pull/125. Everything here is concept stage: no saved part, no print file. The box catch session owns the catch; this work hands it the concept, its check and its requirements. The box master session (`session_01EtHMZPwaEwEqxCkwFCWf7X`) reserves the top-of-box slot for it (no funnel) and asks for one line when a PR on the landing merges.

## Spark's brief (27 Oct, last word)
"Your goal is to create the most error-tolerant most well-designed one-motor latch system that works for both current drone cages. Take your time and research first." Earlier the same day: flat landing, one motor (a stepper on the SKR Mini E3 V3 spare E driver), the Scout's bottom = the rectangle part of the Overseer's base loop, no keep-alive contacts.

## Research (27 Oct, `claude/research/`)
- `2026-10-27-precision-landing-accuracy.md`: real touchdown error 7..14 cm, tag yaw 1..4 deg; plan the mechanism for +-80 mm, +-10 deg, survive +-15 deg; keep the firmware commit gate at +-30..40 mm.
- `2026-10-27-dock-centering-mechanisms.md`: industry = parallel push bars (DJI, Hextronics) or funnels; single-actuator designs close all axes together; none sequences, none takes two drone lengths. Hold-down is rare.
- `2026-10-27-one-motor-sequencing-and-locks.md`: lost motion + spring for "close A then B"; T8 x 2 self-locks (mu needed 0.09; bronze / POM 0.15..0.20), T8 x 8 does not; thrust through an F8-16M bearing; StallGuard on the E driver = Klipper manual_stepper, bench test, switch fallback; PETG on steel mu 0.24..0.29.

## Concept F, push-bar catch (DEFAULT; `nox/box/catch_bars_concept.py`, page `nox/box/catch_bars_view.html`)
Flat deck. One NEMA 17 on a T8 x 2 screw moves one carriage. Two phases set by force, no cam:
1. Two low end pads (3 tall) close along x at 1 : K_T (nose : tail, 1.349) through a preloaded spring (the pad slide). Pad face profile along y: a notch (+-6, 120 deg) that cradles an Overseer tip node (its tip rods leave the node surface at |y| 7.9, so nothing wider touches the node alone); a relief beside it receding 14 (the rods run there at deck height); V wings (|y| 28..80, 18 deg, outer corners leading) that push an off-centre node inward, carry the Scout's straight end tube on their corners, and sit under the rods. Stops: Overseer apex at 205.9 (node 11.7 ahead of the apex), Scout apex at 102.4 (tube on the wing corners); both receivers on the swap centre.
2. The carriage runs 60 more against the spring; that relative motion alone drives two 50-long side bars (L-section, lips) from park (|y| 258) onto the shared side tubes to a rigid stop (gap 0.5), ratio 2.3. The bars wait until x is clamped because the side tubes taper 3.6 over 155 (a fixed gap fits at one x) and the Overseer's side rods leave the side nodes at 16 deg (a bar near a node before x is centred gets a rod across its top). The lips hold the 74 N head stall; the screw holds the bars; stall = seated; the carriage travel (150 Overseer / 262 Scout) says which drone.
Scout: 6 mm printed sleeves on its side rods over x +-32 (DEFAULT) so one gap and one lip height serve both; if the Scout session adopts the 6/4 sized ring (its frame model v2 puts all 4 bottom-loop rods in the 6/4 list) the sleeves fall away and its plane goes to -47.5.

## Check (quasi-static push model, both real cages, 9 x 9 offsets to +-80, yaw -15..15)
First sweep (bars parked at |y| 206, `runs/box_catch/bars_concept/v1_park85/`): Scout 462 / 567, every landing within +-40 at every yaw, +-60 nearly all, seated within 5.7 mm / 0.85 deg (worst at the edge), 0.3 mm / 0.7 deg inside +-20. Overseer 247 / 567: every x offset at y 0 for every yaw; y +-20 to yaw +-10; beyond that (a) the drone lands on a parked bar when a side node is within 36 of it (fixed by parking at 258: the rerun), (b) in phase 2 a lip meets a side rod when the Overseer is still > ~12 deg yawed at the clamp. (b) is a property of its tip geometry: a node in the notch / relief zone (|y| 6..28) is pushed outward by the notch corner while the other node on a wing is pushed inward; the yaw grows until both nodes are on wings (about 13 deg). Wings cannot start further in (rod territory at deck height); 30 deg wings made it worse (unequal lever arms on an off-centre drone). So the Overseer's yaw envelope is about +-10 at |dy| <= 20, +-5 at 40, 0 at 80; the landing controller's commit gate (+-30..40, +-5 deg, already planned) covers it.
Second sweep (park 258, phase 2 of 60, stroke 80): running when this was written; `python3 nox/box/catch_bars_concept.py --merge` joins the six chunk files and builds the page.

## Alternatives weighed
- Ramps (concept E, `catch_ramps_concept.py`, https://claude.ai/artifact/F5b6L8uScqbaDSsXPXTSmz): passive, Scout +-20 / +-25, Overseer +-25, yaw +-2.5; no hold-down; needs mu <= 0.2 faces.
- Hook claws (the earlier direction, `catch_hooks_view.html`, https://claude.ai/artifact/GiKhw8MwEfpqMDCC6N52zk): every claw rises outside the ring where both shells flare out at 16..20 deg, so claws collide with shell rods under any offset; 12 claws. Not pursued.

## Files (branch)
- `nox/box/catch_bars_concept.py`: the model (bodies as orthogonal boxes in rotated frames, exact sphere contacts; least-norm push with the pad slide as a 4th unknown in phase 2). Modes `--smoke`, `--drone overseer|scout --tag _x` with `OFFS_X="-80,-60,-40"` (chunks), `--merge`, `--html`. Env: `SLEEVE`, `PARK_EXTRA`, `PARK_EXTRA_Y`, `S2`, `LM`, `WING_DEG`; the Scout inputs `SCOUT_REF`, `SCOUT_DIR`, `SCOUT_CATCH_X` (in `catch_ramps_concept.py`; v12 = `origin/main`, `runs/scout_layout/v12`, 5.0; its ring stays on the hexagon side nodes).
- `nox/box/catch_bars_view.html` -> `runs/box_catch/bars_concept/bars.html` (the artifact).
- `runs/box_catch/bars_concept/`: `result.json` (merged), chunk files, logs; `v1_park85/` the first sweep.
- A full sweep is ~75 min a drone single-threaded on this box; run 3 chunks a drone (6 processes on 4 cores, ~60 min). Kill with `pkill -f "^python3 -u nox/box/catch_bars"` (a bare `pkill -f catch_bars` kills the calling shell).

## Next (in order)
1. Merge the second sweep, build and publish the page, write `runs/box_catch/bars_concept/summary.md`, the log line, the box catch row (funnel panels, hub, hook latch rows to mark superseded), PR 125 body; `review` agent in concept mode; merge on PASS. Then one line to the box master session; reply to the Scout v3 session (both seats taken: sleeves or the 6/4 ring).
2. Box catch session: draw the deck, pads, pad slide, carriage, lever, compound pinion, side bars, lips; `nox/box/box_catch.assembly.json` + `catch_system.py` on the new load cases (head stall 74 N on the lips, pad spring 30 N preload 0.5 N/mm, carriage 120 N at the stall, E); rerun this check at the saved geometry and at Scout v12; the 74 N lip FE; chamfer the pad tops (rods ride up to 2 mm), round the notch corner.
3. Bench: StallGuard on the E driver at 30 rpm (Klipper manual_stepper), the spring vs the Overseer's drag on TPU feet.
4. Scout session: the side-rod sleeves or the 6/4 ring; the plane (-50.7 with sleeves, -47.5 with the 6/4 ring) so both cartridges sit 7.0 above the deck.
5. Overseer session: nothing needed; its tip rods set the yaw envelope (an option if more is wanted: lower the two tip rods' first 30 mm, or a 2 mm higher tip node).

## Sessions
- Scout v3: `session_01EtHMZNhc68Ghbc2KCNMUzU` (v12 on main, catch 5.0 forward; sized-ring decision open with Spark).
- Overseer cage: `session_01Em8yMe1ArHE9XePmNwV7RS` (cage v5; messages on every cage change).
- Box master: `session_01EtHMZPwaEwEqxCkwFCWf7X`.
- Spark's run rule: never restart a run for a constant change; re-cut a built part on its result.
