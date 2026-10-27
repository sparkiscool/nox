# Overseer core v1: system pass for everything inside the cage but the arms (27 Oct 2026, concept, round 3)

Spark 27 Oct: "this chat is in charge of designing everything else left on the newest overseer design inside the cage, excluding the arms, it needs to hold all components but ignore the actual mechanism of the battery cartridge and pocket for now"; "also ignore all nodes on the body for now"; "no powerboard the stack for the fc has a esc under it, the camera and the lidar isnt your job ... make sure you get the right dimensions and give the appropriate space for the parts, and then design coupons for the least amount of materials for me to confirm mounting"; "make sure bolts are accessible through a screwdriver, the whole thing need to be in as little pieces as possible and easy to work with".
Script `nox/overseer/core_v1.py` (~1 min), visuals `nox/overseer/core_v1_view.py`, assembly `nox/overseer/overseer_core.assembly.json` (v5 / cage v3 context unchanged). Requirements: `nox/dimensions.md` "System Overseer core". Coupons: `runs/overseer_core_coupons/v1/`.

Visual: https://claude.ai/artifact/JX9skx2qYkJGBnjFHrWAo6 (source `motion.html`: assembly order, core in colour, v5 context grey; still `preview.png`: side, top, section through the Pi deck)

System: Overseer core v1, cage first 7.5 mm (E), shell 131 rods closed, fit 170.2 mm³ (E) (down camera / receiver) FAIL, extents 743 x 694 x 246 mm (E), mass 3429 of 2200 g (E), CG 2.0 mm off the thrust axis (E), 21.2 pack out (L exception), probes -178 mm (E) edge at -X (receiver, unshielded) FAIL, access 37/37 clear. The FAILs are other sessions' parts against the real belly: the v5 down camera (camera owner), the v5 shell and body ties (cage session, Spark: body nodes later).

## Round 3 (after the concept review FAIL of round 1 and Spark's corrections)
- No PDB: the MicoAir743-Lite sits on its 4-in-1 ESC (MicoAir 55A AM32 assumed: 44 x 43.5 x 5.2, 14 g; the 17.5 / 24 g stack on MicoAir's drawing = FC 10 + this ESC, CHECK) at the plate centre, FC top z 23.5.
- The Pi 5 Active Cooler is 13.7 tall with its blower intake on top (product brief): over the stack it would breathe through 2 mm. The Pi sits right side up on the rear wing, wholly aft of the top plate edge and the arm roots, cooler in the open, ports aft.
- Two printed pieces (was five): **hub deck** (stack posts, Pi deck, UBEC / buck / RX bays, lead + cap channel) and **sled** (receiver face, flow pod, Ø9 driver holes). Every screw goes straight down with a screwdriver in assembly order (`dimensions.md` Assembly order); the Pi's 4 screws are reachable with the top plate on.
- Feet go to the cage's opening ring nodes (review alternative B): no 100 mm cantilevered skids, no rails in the prop wash; the camera pod is the camera owner's.
- The skids of round 1 are kept for the record in `alt_a/`.

## Alternatives weighed (numbers E)
- **A (round 1): skids carrying the pods and the feet.** Feet span the CG on the drone itself (tip 28° forward), but the front feet sat 100 ahead of the last joint (joint loads to ~1.6 kN in a one-side -Z crash, review recompute), 2 rails 19 x 55 in the rear prop wash, +2 x 45 g. Not taken.
- **B (chosen): feet on the cage's opening ring nodes.** The floor load goes into the cage (designed for 35 g through the node that touches first, L), the core keeps 2 pieces (108 g of slots); the shell must reach z -82.9 under the CG (+6 on what it needs anyway). Cost: the drone cannot stand on the floor until the cage session places them.
- **C: receiver straight on the bottom plate (no 6.0 sled).** Belly 6 shallower, the sled's 78 g down to a ~20 g rear bracket, but the front receiver bolts (56.85, ±27.8) land inside the front arm roots (u 59.9, v -20.5) and the 2 XT60-end bolts sit 13 past the plate edge. Not taken.

## Pieces
| Piece | Holds | Fixed by | Worst reaction (E) | Slot |
|---|---|---|---|---|
| hub deck | FC stack, Pi 5, UBEC, buck, RX, lead + cap | 6 M3 through the plate + 2 aft legs into the sled | 11.7 N a bolt | 30 g |
| sled | receiver (6 M3), flow sensor | the same 8 M3 | receiver 104.6 N, plate 130.2 N a bolt | 78 g |

Prop wash (plan area under the disks): hub deck + Pi + bays 2.4 %, sled 4.8 %, receiver 1.7 % (reported, L 8). Mass 3429 g, T/W 1.92 (E). Carbon rods: none in the core, every piece printed.

## Next
Spark prints the two coupons and checks the mounting; the P numbers go to `dimensions.md` M; then the hub deck and sled piece runs (nox/topo) on these requirements.
