# Box catch concept F: push-bar catch, one stepper, both drones (27 Oct 2026)

Concept check (`docs/generative.md` The rule, 0, Concept check), no saved part. Artifact: https://claude.ai/artifact/XfUFHJZZrfnNdi9nnuuHQB (source `nox/box/catch_bars_view.html`, data `result.json` here). Script `nox/box/catch_bars_concept.py`. Spark's brief (27 Oct): "the most error-tolerant most well-designed one-motor latch system that works for both current drone cages. Take your time and research first." Research in `claude/research/2026-10-27-*.md` (landing budget +-80 mm, +-10 deg, survive +-15; industry push bars; lost motion + spring for a one-motor sequence; T8 x 2 self-locking; StallGuard on the E driver).

## The design
Flat deck round the swap column. One NEMA 17 on a T8 x 2 lead screw (self-locking, thrust bearing) moves one carriage under the deck. Two phases, set by force (lost motion), no cam:
1. **Pads, x.** Two end pads 3 tall (top 1 under the ring tube centres, C) close along x at 1 : 1.349 nose : tail (the tail pad through a reversing compound pinion on the pad slide; the slide is on the carriage through a preloaded spring). The pad face is a profile along y: a 12-wide 120 deg notch for an Overseer tip node (its tip rods leave the node surface at |y| 7.9, C, so nothing wider touches the node alone), a relief beside it receding 14 (the rods run there at deck height), V wings |y| 28..80 at 18 deg with their outer corners leading (they push an off-centre node inward; the Scout's straight end tube rests on their corners; the tip rods pass over them >= 0.6, C). The pads stop where the drone stops them: nose apex 205.9 for the Overseer (node 11.7 ahead), 102.4 for the Scout (tube on the corners); the ratio puts both receivers on the swap centre (C). Nose travel at the clamp 90.5 / 197 (C): the step count says which drone.
2. **Bars, y and yaw, hold-down.** The carriage runs 60 more against the pad spring; that relative motion, through a 2.3 : 1 lever, drives two side bars (50 long, centred between the side nodes, L-section) from park (|y| 258) onto the shared side tubes to a rigid stop, gap 0.5 (C). The bars wait because the side tubes taper 3.6 over 155 (a fixed gap fits at one x) and the Overseer's side rods leave the side nodes at 16 deg (a bar near a node before x is centred gets a rod across its top). The nodes ride the pad relief into the notches; the spring gives (stroke 80, used 71 / 62 median, 74 max, C). The lips (3 thick, 1.5 over the Overseer's tube top, edge 2 inside the tube line, 3 clear of its inner link, C) hold the 74 N head stall as a cantilever (5 MPa, E). Motor stall on the stop = seated; carriage travel 150 / 257 (C). Release: back out.

Scout: 6 mm printed sleeves on its side rods over x +-32 (DEFAULT, 2 x ~1 g) so one gap and one lip height serve both; its plane then −50.7 so both cartridges sit 7.0 above the deck (C; today 8.1). If the Scout adopts its 6/4 sized ring (frame model v2 lists all 4 bottom-loop rods) the sleeves fall away and the plane is −47.5 (Scout session).

## The check
Quasi-static push model on the real cages (Overseer cage v5 and Scout layout v11 at dea0b51, every low part of both assemblies, surface samples under the lip top): the drone is rigid on the deck; each 0.5 mm of carriage travel it is moved the least that resolves every contact (translation and yaw; in phase 2 the pad slide is a fourth unknown, sprung within its stroke). Bodies are orthogonal boxes in rotated frames, so sphere contacts are exact. A pushing face may only touch ring tubes, nodes and the tip-node structure; a lip touching anything, a body running under anything, a rod climbing a top edge more than 2 mm, or a landing on a parked body fails that landing. 9 x 9 offsets to +-80, yaw −15..15 (567 landings a drone, 6 chunks, ~60 min on this box).

| | Overseer | Scout |
|---|---|---|
| Seat | 259 / 567 | 544 / 567 |
| Every yaw −15..15 | y 0 at every x; |dy| ≤ 20 to yaw ±10; ≤ 40 to ±5; ≤ 60 at 0 | every landing within ±60; ±80 but 23 corners at yaw ±15 |
| Seated x error, median / 90 % / max | 0.8 / 2.7 / 6.0 mm | 0.5 / 2.7 / 5.8 mm (0.2 / 0.9 / 1.0 inside ±20) |
| Seated y, yaw | ≤ 0.23 mm, ≤ 0.9 deg | ≤ 0.22 mm, ≤ 0.9 deg |
| What fails | a lip meets a side rod in phase 2 when the Overseer is still > ~12 deg yawed at the clamp | the bars pinch a ring still 7 deg yawed (corners only) |

All C from the model. The Overseer's yaw limit is its tip geometry: a node in the notch / relief zone (|y| 6..28) is pushed outward by the notch corner while the other node on a wing is pushed inward, so the yaw grows until both nodes are on wings (~13 deg); wings cannot start further in (rod territory at deck height); 30 deg wings made it worse (unequal lever arms). The planned firmware commit gate (±30..40 mm, ±5 deg) sits inside the envelope. The x error is a bias when one pad has pushed the drone far (the sprung slide re-seats late): inside ±20 it is under 1 mm; a second short cycle (back out 10, close) or the receiver's float takes the rest (open).

First sweep (bars parked at |y| 206): `v1_park85/`: Overseer 247, Scout 462; the 200 extra fails were landings on the parked bars, removed by parking at 258.

## Alternatives weighed
- Ramps (concept E, `runs/box_catch/ramps_concept/`): passive; Scout ±20 / ±25, Overseer ±25, yaw ±2.5; no hold-down; needs μ ≤ 0.2 faces. A quarter of the room.
- Hook claws (the earlier direction, `runs/box_catch/hooks_concept/`): every claw rises outside the ring where both shells flare out at 16..20 deg, so the sweeps collide with shell rods under any offset; 12 claws on sprung links.

## Rule checks (concept mode)
Cage first: nothing of the box stands inside the landing room above 11.5 (bars) / 3 (pads), both outside the seated cages; the drones' cages are untouched. Compact: the whole mechanism is under the deck plus 4 low pushers. Load path: the stall load goes lips → bars → guides → deck, not through the screw; the x clamp force is the spring (30 N preload, 0.5 N/mm, E), the carriage at the stall ~120 N (E) inside a NEMA 17 on T8 x 2 at 10..15 N·cm (research). Fit: the box session draws it; this check is kinematic. Mass: pads, bars, slide, lever, pinion, racks in PETG, 1 motor, 1 screw, 1 spring (E ~400 g with the motor).

## Hand-off (box catch session)
Draw the deck, pads, pad slide, carriage, lever, compound pinion, side bars, lips; `box_catch.assembly.json` and `catch_system.py` on the new cases (head stall 74 N on the lips, spring, stall); rerun this check at the saved geometry and at Scout v12 (`SCOUT_REF=origin/main SCOUT_DIR=runs/scout_layout/v12 SCOUT_CATCH_X=5.0`; its ring stays on the hexagon side nodes); lip FE; chamfer the pad tops (rods ride up to 0.8), round the notch corner; bench StallGuard on the E driver at 30 rpm (Klipper manual_stepper) and the spring against the Overseer's drag on TPU feet. Retires: funnel panels v7 / v9 plain, hook latch v2, the 8 catch wheels, the keep-alive probes.
