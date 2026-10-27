# Overseer core v1: system pass for everything inside the cage but the arms (27 Oct 2026, concept, round 5)

Spark 27 Oct: "this chat is in charge of designing everything else left on the newest overseer design inside the cage, excluding the arms, it needs to hold all components but ignore the actual mechanism of the battery cartridge and pocket for now"; "also ignore all nodes on the body for now"; "no powerboard the stack for the fc has a esc under it, the camera and the lidar isnt your job ... make sure you get the right dimensions and give the appropriate space for the parts, and then design coupons for the least amount of materials for me to confirm mounting"; "make sure bolts are accessible through a screwdriver, the whole thing need to be in as little pieces as possible and easy to work with"; "the fc mounts to the top plates lines so just move that there"; "make the apropreiate adjustments to optimize the space"; "the fc does not need a mount it already goes to the carbon plate"; "confirm all parts mounting hole sizes and positions".
Script `nox/overseer/core_v1.py` (~1 min), visuals `nox/overseer/core_v1_view.py`, assembly `nox/overseer/overseer_core.assembly.json` (v5 / cage v3 context unchanged). Requirements: `nox/dimensions.md` "System Overseer core". Coupons: `runs/overseer_core_coupons/v1/`.

Visual: https://claude.ai/artifact/JX9skx2qYkJGBnjFHrWAo6 (source `motion.html`: assembly order, core in colour, v5 context grey, alternatives A and C beside it as their own groups; stills `preview.png` (side, top, section) and `alternatives.png` (A / B / C side by side with their numbers))

System: Overseer core v1, cage first 7.5 mm (E), shell 131 rods closed, fit 170.2 mm³ (E) (down camera / receiver) FAIL, extents 743 x 694 x 246 mm (E), mass 3420 of 2200 g (E), CG 0.8 mm off the thrust axis (E), 24.8 pack out (L exception), probes -178 mm (E) edge at -X (receiver, unshielded) FAIL, access 32/33 (the Pi's aft plugs blocked by the v5 cage body ties, 53 mm³: cage session, Spark: body nodes later). The FAILs are other sessions' parts against the real belly: the v5 down camera (camera owner), the v5 shell (cage session).

## Rounds 4 and 5 (Spark 27 Oct: the FC on the top plate with no mount; optimise the space; review fixes)
- Round 5: the MicoAir743-Lite on its 4-in-1 ESC (MicoAir 55A AM32 assumed: 44 x 43.5 x 5.2, 14 g, listing; the 17.5 / 24 g stack on MicoAir's drawing, spec; CHECK) bolts straight to the carbon top plate (Spark: no mount), turned 45° so its 30.5 holes fall in the plate's slots on its axes ("the lines", r 21.57; the +X hole sits at the central cutout's edge, CHECK; review ray probe of the STEP), on 4 M3 studs set on the bench, nylon nuts from above: FC top z 67.1 (C), clear of the props' band at z 77; motor leads leave the ESC's corners (on the plate axes once turned) and run out over the arm tops; its 4 screws go in with everything assembled.
- Every mounting hole checked: the bottom plate's from its STEP cylinder faces (360° only), the bought parts' from their makers; the table is in `dimensions.md` System Overseer core.
- The Pi 5 comes back between the plates, right side up on the **hub deck** (board z 8, C): its Active Cooler (13.7 tall, intake on top, product brief) has 21.7 (C) of air under the top plate; ports aft through the gap between the rear roots. UBEC and buck sit on the front wing ahead of it.
- Rear hub deck feet moved to the real Ø3.1 holes (-65, ±22.5): the review found (-72.5, ±31.75) are R3 outline fillets in the STEP, not holes (`s500_cad.vertical_holes` keeps every vertical cylinder face; the 90° fillet faces should be dropped there, noted for its owner).
- Two printed pieces: hub deck, sled. Every screw straight down with a screwdriver (`dimensions.md` Assembly order). Plan area in the prop wash: hub deck + Pi + bays 1.1 % (was 2.4), sled 4.8 %, receiver 1.7 % (C).
- Feet go to the cage's opening ring nodes (review alternative B). Round 1's skids are in `alt_a/` and drawn in the artifact.

## Alternatives weighed (numbers E)
- **A (round 1): skids carrying the pods and the feet.** Feet span the CG on the drone itself (tip 28° forward), but the front feet sat 100 ahead of the last joint (joint loads to ~1.6 kN in a one-side -Z crash, review recompute), 2 rails 19 x 55 in the rear prop wash, +2 x 45 g. Not taken.
- **B (chosen): feet on the cage's opening ring nodes.** The floor load goes into the cage (designed for 35 g through the node that touches first, L), the core keeps 2 pieces (99 g of slots); the shell must reach z -82.9 under the CG (+6 on what it needs anyway). Cost: the drone cannot stand on the floor until the cage session places them.
- **C: receiver straight on the bottom plate (no 6.0 sled).** Belly 6 shallower, the sled's 78 g down to a ~20 g rear bracket, but the front receiver bolts (56.85, ±27.8) land inside the front arm roots (u 59.9, v -20.5, C) and the 2 XT60-end bolts sit 13 past the plate edge (C). Not taken.

## Pieces
| Piece | Holds | Fixed by | Worst reaction (E) | Slot |
|---|---|---|---|---|
| hub deck | Pi 5, UBEC, buck | 4 M3 x 12 down through the plate into the sled | 16.5 N a bolt | 24 g |
| sled | receiver (6 M3), flow sensor | the same 4 M3 (Spark: "do we really need that many": 4 carry 193 N a bolt, 6 carried 137, E) | receiver 104.6 N, plate 193.4 N a bolt | 75 g |

Mass 3420 g, T/W 1.93 at 1650 gf a motor (E). Carbon rods: none in the core, every piece printed.

## Next
Spark prints the two coupons and checks the mounting; the P numbers go to `dimensions.md` M; then the hub deck and sled piece runs (nox/topo) on these requirements.
