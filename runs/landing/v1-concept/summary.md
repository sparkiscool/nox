# Box landing v1 concept (27 Oct 2026): three feet on the shared bottom ring in three V-grooves

Spark 27 Oct: "this session is in charge of designing the new funnel/ landing system, using the new layout of the overseer cage, the bottom is no longer a funnel, i would like to find what the best way we can still land the drones reliably with error tolerance and also the scout needs to be able to land as well"; "the newest design should have two cameras"; "just really care about the bottom opening, since that will likely be the only thing thats the same with the scout and the overseer".

Script `nox/box/landing/landing_sim.py` (MuJoCo 3.17, ~45 min on 4 cores), page `landing_view.py` + `landing_page.html`. Visual: https://claude.ai/artifact/Nh3LqpJzkExyn8Vsk2C3Uw (source `landing.html`). Part doc `nox/box/box-landing.md`; numbers `dimensions.md` System box landing.

## Cases (E)
Offsets 0..60 in 8 directions x yaw −5 / 0 / +5° (24 drops per offset), drift 0.15 m/s turning with the case, tilt 2°, descent 0.30 m/s, thrust holding the weight to the first touch then spinning down (tau 0.08 s). Pass at 4.0 s: ring centre within 2 mm, yaw within 1° of the centred seat (recount: the first pass also asked for z within 2 and speed under 5 mm/s, which failed drones still rattling at 0.0 mm). Overseer 3380 g at the v5 part centroids (E masses, M geometry); its cage, belly, pods and cartridge point-sampled against the box along every drop (`clear`, caught drops only). Scout 520 g E, ring + feet (its cage is not drawn).

## Result (seated share by offset; capture = largest offset with 24 / 24)
| Drone | Surface | Angle | μ (E) | Foot | 0..30 | 40 | 50 | 60 | Capture | Least gap (caught) |
|---|---|---|---|---|---|---|---|---|---|---|
| Overseer | 3 feet + grooves, ball tips (DEFAULT) | 30° | 0.03 | 39.6 | 100 % | 96 % | 71 % | 58 % | ±30 | 1.5 |
| Scout | same | 30° | 0.03 | 39.6 | 100 % | 92 % | 62 % | 29 % | ±30 | |
| Overseer | ball tips, dirty | 30° | 0.06 | 39.6 | 100 % | 96 % | 71 % | 58 % | ±30 | −1.8 |
| Scout | ball tips, dirty | 30° | 0.06 | 39.6 | 100 % | 92 % | 67 % | 29 % | ±30 | |
| Overseer | ball tips | 20° | 0.03 | 26.8 | 100 % | 88 % | 75 % | 46 % | ±30 | 1.6 |
| Scout | ball tips | 20° | 0.03 | 26.8 | 100 % | 75 % | 58 % | 33 % | ±30 | |
| Overseer | PTFE glide tips | 30° | 0.12 | 39.6 | 100 % | 100 % | 71 % | 54 % | ±40 | 0.3 |
| Scout | PTFE glide tips | 30° | 0.12 | 39.6 | 75..100 % | 67 % | 42 % | 29 % | ±0 | |
| Overseer / Scout | PTFE glide tips | 40° | 0.12 | 55.3 | 96..100 % | 96 / 71 % | | | ±20 / ±10 | 6.9 |
| Overseer / Scout | bare PETG tips | 30° | 0.35 | 39.6 | 33..71 % / 0..8 % | | | | 0 | |
| Overseer / Scout | tray, rolling ring nodes | 10° | 0.03 | | 88..100 % / 83..100 % | 71 / 67 % | | | 0 / ±10 | −18.8 (cage hits) |
| Overseer / Scout | one cone (dish) | 10° | 0.03 / 0.12 | | 0..4 % | | | | 0 | −33 / −4 |

## Why (from the drops)
- **Dish:** a wide ring on a cone tilts instead of rising when off-centre; the centring force is about W·e·2 tanθ / L, a ~1 m pendulum at 10°: the drone stops 20..30 mm off even rolling, and today's cage touches the cone.
- **Tray:** a flat floor with walls only outside centres, but only rolling beats friction, and rolling has no damping: the drone runs across the floor and drops a node into the swap hole.
- **Kinematic coupling:** each ball-in-V contact acts through the ball centre, so the seat is exact (0.0 mm, 0.0° in every caught drop). Jam rule: the lightest foot (fraction f of the weight) must drag the other two along their grooves, f·(sinθ − μ cosθ) > μ (1 − f) / cosθ. The Scout's CG sits on the receiver, 41 behind the feet centroid, so its front feet carry 27 % each: PTFE (μ 0.12) jams it at 30°; ball tips (μ 0.03) pass. The Overseer (CG 91 ahead of the receiver, fronts 41 %) passes either way.
- **Limit of capture:** at 40 the misses are at the front groove pair's shared edge (front feet 117 apart) or with yaw +5 adding 19 at the rear foot; the gate commits inside ±25 mm, ±5° (DEFAULT).

## Chosen (DEFAULT) and the alternative
3 feet at F1 (150, 58.4), F2 (150, −58.4), F3 (−176, 0) on the ring (ring frame: x forward along the receiver, origin at the receiver centre), ball-transfer tips 39.6 below the ring node bottoms; 3 PETG V-blocks, 30° flanks, half-width 60, length ±60, grooves to the centroid (41.3, 0), tops 5 under the ring node bottoms; swap hole receiver + 15 a side. Alt: PTFE glide tips at 40° flanks, feet 55.3 (no rolling part; Scout ±10 only).

## Open
- Overseer: least gap to the blocks 1.5 mm in the worst caught drop (−1.8 with dirty balls; 12.7 seated): its next cage pass keeps ≥ 3 over the blocks through the capture (or the blocks drop 3).
- Hold (clamp over each foot), keep-alive probes on F1 / F2, the box-top system pass, the AprilTag plate.
