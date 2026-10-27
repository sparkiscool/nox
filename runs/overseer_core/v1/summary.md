# Overseer core v1: system pass for everything inside the cage but the arms (27 Oct 2026, concept)

Spark 27 Oct: "this chat is in charge of designing everything else left on the newest overseer design inside the cage, excluding the arms, it needs to hold all components but ignore the actual mechanism of the battery cartridge and pocket for now"; "also ignore all nodes on the body for now".
Script `nox/overseer/core_v1.py` (~1 min), visuals `nox/overseer/core_v1_view.py`, assembly `nox/overseer/overseer_core.assembly.json` (v5 / cage v3 parts as context, unchanged). Requirements: `nox/dimensions.md` "System Overseer core".

Visual: (artifact link in the PR; source `motion.html`: the assembly sequence, core in colour, v5 context grey; still `preview.png`: side, top, section at the pods)

System: Overseer core v1, cage first 3.3 mm (E) at -Z (feet), shell 131 rods closed, fit 1006 mm³ (E) (cage shell tubes / skid L) FAIL, extents 743 x 694 x 248 mm (E), mass 3531 of 2200 g (E), CG 1.9 mm off the thrust axis (E), 20.5 pack out (L exception), probes -177 mm (E) edge at -X (receiver, unshielded) FAIL, access 21/21 clear. Both FAILs are the v5 shell against the real belly: the cage session regenerates the shell on this belly (no cage node is placed on the body, Spark).

## What changed against v5
- **The real latch set in pose** (battery latch v4 Overseer receiver v2, cartridge v6, carrier, pack): it hangs 68.8 under its mount plane, v5's stand-in 52. With the 6.0 sled under the bottom plate the pack top is z -19.9 and the cartridge bottom z -76.9: **22.9 deeper than v5** (C on M meshes). XT60 end aft, lead straight up behind the plate to the PDB.
- Feet: v5's feet sat at x -98 / 38 with the CG at x 59: the drone would tip onto its nose on the floor. The skids now carry the feet at x -48 / 112, y ±76: tip-over 28° forward, 47° back, 37° side (C on E).

## The pieces (all PETG but the feet; envelopes for the piece runs)
| Piece | Holds | Fixed by | Worst reaction (E) | Slot |
|---|---|---|---|---|
| hub frame | FC (30.5 grommets at the plate), Pi 5 deck over it | 4 columns: bottom plate (±24, ±22.5), top plate (±24.51, ±24.51) | 4.4 N a bolt; USB plug 20 N | 30 g |
| power tray | PDB on 30.5, UBEC, 12 V buck, RX | 4 M3 through plate + sled | 13.5 N | 18 g |
| sled | receiver (6 M3 inserts) | the 8 hub and tray bolts | receiver bolts 104.6 N, plate bolts 163.7 N (35 g crash) | 70 g |
| skid L / R (mirror) | down camera pod / flow pod, 2 feet each | 2 joints into the sled ribs | 155.6 N a bolt (-Z crash through the feet) | 45 g each |
| TPU feet x4 | floor | M3 into the skid legs | landing 105 N each on 2 feet | 6 g |

Checks (C on E, `checks.txt`): every hub piece and part 0 mm³ with the v5 arm meshes; Pi +x plugs 81.5 of 82 between the roots; all 8 arm bottom screw keys clear below the sled (Ø9 holes through it); down camera and flow views clear (lenses on the cartridge bottom plane); nothing in the swap column (receiver + 30 a side below the cartridge); pod first mode ~2150 Hz (E, target 100).

## Two alternatives weighed
- **B: feet on the shell's lowest nodes, pods hung from the sled only.** Saves ~50 g of skid (E) and the skids' 174 length, but the floor and the -Z crash (1226 N, E) go into cage nodes and the shell must reach z -86 under the CG fore and aft (cage session work, and Spark said no body nodes for now). Not taken.
- **C: the receiver straight on the bottom plate (no 6.0 sled).** Belly 6 shallower (cartridge bottom -70.9), but the 2 XT60-end bolts sit 13 past the plate's rear edge and need a bracket anyway, and the hub frame and tray lose the shared clamp through the sled. Not taken.

## DEFAULTs taken
Sled 6.0 thick; pods 20 ahead of v2's place (alt v2's place with a board unplugged); Pi powered on its GPIO 5 V pins (alt right-angle USB-C); FC alone at the plate (ESCs on the arms: arm session); feet 9 under the cartridge bottom; pods mirror-sized for balance.

## Next
Piece runs (nox/topo, one worker each): hub frame, power tray, sled, skid pair; feet drawn. Then the assembled recheck on the cage session's new shell.
