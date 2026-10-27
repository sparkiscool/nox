# Box landing v1 (concept, 27 Oct): three feet on the shared bottom ring, three V-grooves on the box

Replaces the collar funnel (`box-funnel.md`, panels v7 / v9 plain and the hook latch) for both drones. Numbers: `nox/dimensions.md` "System box landing" (the only source; this page explains them). Script `nox/box/landing/landing_sim.py` (MuJoCo drops, keep-out check), page `landing_view.py` + `landing_page.html`, results `runs/landing/v1-concept/` (`sweep.json`, `traj.json`, `summary.md`). Visual: https://claude.ai/artifact/Nh3LqpJzkExyn8Vsk2C3Uw (source `runs/landing/v1-concept/landing.html`).

## Brief (Spark 27 Oct)
- "this session is in charge of designing the new funnel / landing system, using the new layout of the overseer cage, the bottom is no longer a funnel, i would like to find what the best way we can still land the drones reliably with error tolerance and also the scout needs to be able to land as well"
- "the newest design should have two cameras" (front + down, both drones: the down camera looks through the bottom opening)
- "just really care about the bottom opening, since that will likely be the only thing thats the same with the scout and the overseer"

## Why this concept
The shared interface is the cage's bottom opening ring (Overseer cage v3: 8 nodes round the swap zone, receiver + 30 a side). Everything near that ring is shallow: the v5 cage underside rises only 7.3° from the ring, the cartridge sits 3 above the ring plane, the down camera pod 9 above the node bottoms. So any steep catch surface near the ring touches the cage, and any shallow one has to beat friction. Three findings from the drops:
1. **One cone through the ring (dish) centres weakly.** A wide ring on a cone tilts when off-centre instead of rising: the centring force is about W·e·2 tanθ / L (L the ring length), a pendulum ~1 m long at 10°. With rolling contact (μ 0.03) it stops ~30 mm off; with sliding it never centres.
2. **A flat-floored tray with rolling ring nodes centres, but cannot be damped.** The drone rolls down the wall, across the floor and drops a node into the swap hole.
3. **A kinematic coupling centres exactly.** A ball in a V touches both flanks and both forces pass through its centre, so the drone cannot rock its way out: off-centre, each foot sits on one flank, pushed back with a force that does not fade near the bottom. Three feet in three grooves fix x, y and yaw (Maxwell coupling). Rule found in the drops: the lightest-loaded foot must drag the other two along their grooves, so its push f·(sinθ − μ cosθ) must beat μ/cosθ·(1 − f) of the others. The Scout's lightest foot carries 27 % (its CG sits on the receiver, 41 behind the feet centroid): sliding PTFE (μ 0.12) jams it at 30°; rolling tips (μ 0.03) pass with margin.

## The design (DEFAULT)
- **On every drone (shared, fixed to the receiver):** the bottom opening ring of Overseer cage v3, and three feet hung from it: F1 / F2 on the front-diagonal ring tubes at (150, ±58.4), F3 on the rear end tube at (−176, 0) (ring frame: x forward along the receiver, origin at the receiver centre, node centres at z 0). Each foot: printed leg, ball-transfer tip (bought), tip bottom 39.6 below the ring node bottoms. They replace the four TPU floor feet. Nothing else of the drone below the ring node bottoms.
- **On the box:** three static PETG V-blocks, one per foot, grooves pointing at the feet triangle's centroid (41.3, 0): flanks 30°, half-width 60, length 60 each way from the seated foot, block tops 5 under the ring node bottoms. The swap hole (receiver + 15 a side, 209.6 x 106.2) stays open between them; the front blocks are cut by it.
- **Capture (sim, 24 drops per offset, yaw ±5°, drift 0.15 m/s):** both drones 100 % to ±30 mm, 96 % (Overseer) / 92 % (Scout) at ±40; every caught drop ends on the seat (0.0 mm, 0.0°).
- **Landing gate:** the down camera reads an AprilTag on the box top (precision landing); the drone commits inside ±25 mm, ±5° yaw, else climbs and retries.
- **Down camera and flow:** the feet are ≥ 70° off their axes, out of view; the opening stays clear.

## Alternatives weighed
- PTFE glide tips instead of ball tips: no rolling part and the Overseer catches ±40, but the Scout jams (±0 at 30°, ±10 at 40° flanks with 55 mm feet).
- Tray with rolling ring nodes: 8 ball transfers per drone (~+60 g E), overshoots into the swap hole.
- One shallow cone: fits under today's cage untouched, stops 30 mm off.
- Not run: a collar round the whole cage (bears on shell facets that differ between drones, ~900 wide); pushers (old design, out unless Spark asks).

## Next
1. Hold for the swap: a clamp over each foot from its V-block (replaces the hook latch).
2. Keep-alive: spring probes in the front blocks onto pads on F1 / F2.
3. System pass on the box top (box structure v2): V-blocks, tag plate, clamps; cases landing 6 g at the feet, swap pull 50 N, hand lift 100 N; then the piece runs.
4. Overseer and Scout system passes add the feet (requirements in `dimensions.md` System box landing).
