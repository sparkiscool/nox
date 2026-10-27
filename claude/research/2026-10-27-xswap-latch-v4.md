# xswap latch v4: research before the redesign (27 Oct 2026)

Asked by Spark (via Alex, 26 Oct): deeper finger tip and slot in the latch; bigger latch; small finger stroke, large
latch travel; tolerance on every contact (never cubes on cubes); pincer narrower than the battery, latch under the pack
instead of beside it; endurance over thousands of cycles; simplest solution wins. Scope: latching and unlatching only.

## What the existing research already says (claude/research/2026-10-14-bottom-cartridge-latch.md)
- Field failures of hanging packs: half-latched is the top cause; the catch the hook bears on breaks before the hook;
  swelling must not touch the latch; any play at the connector wears it.
- Nobody holds a crash on spring force alone: hold is geometry, the spring only returns the latch.
- Pull-out should load the latch in its locked direction (Motorola "inertia enhanced latching").
- Release slots recessed inside the outline so a crash can't reach them.

## New this round
- **Gate latch (Spark's photo; US5106131, expired):** pivoting bar, flat shoulder takes the load, rounded / bevelled
  lead-in spreads the latch on entry, released only from one side. One moving part per latch, pivot on a screw.
  https://patents.google.com/patent/US5106131
- **Spring bolt (Spark's other photo):** straight bolt, spring behind it. Simple, but to get more bolt travel than
  actuator travel it needs a wedge or a lever in between.
- **Snap-fit practice (Core77, Unionfab):** size by strain, not force; bend in-layer (XY), halve allowables if a beam
  must bend across layers; 0.3 to 0.4 mm clearance on pivot and slide fits; 20 to 30 deg lead-ins; 90 deg retaining
  faces only come apart through a release window; PETG fatigue is middling, so don't make PETG the spring.
  https://www.core77.com/posts/65318/How-to-Design-Snap-Fit-Components
  https://www.unionfab.com/blog/2025/06/3d-print-snap-fit
- **Springs:** a steel compression spring worked well below its solid length lasts far longer than a printed flexure
  (general spring practice, E). So the springs are bought steel; the printed parts carry load, never flex.

## What this means for v4
1. One rigid printed lever per end, on a steel dowel, returned by a steel spring: the gate latch.
2. Lever ratio ~4.8: the finger moves ~2.2 mm in the drive, the latch tip moves ~10.7 mm.
3. Load face flat, at 90 deg to the pull-out; friction under load (0.3 x 476 N) dwarfs the spring and the lever's own
   inertia, so a loaded latch can't be opened, even by the pincer: the pincer first pushes the pack up to unload it.
4. The two levers release in opposite directions, so a side hit that flicks one open pushes the other one shut.
5. Every first contact is a slope or a round: round finger posts in rounded slots, chamfered slot mouths, chamfered
   tip ends, leg lead-ins, bullet guide posts.
