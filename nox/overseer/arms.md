# Overseer printed arms (v1, 27 Oct 2026)

Four printed PETG arms between the S500 top and bottom plates, from layout v5 / cage v3. Front pair (arm 1, mirror arm 4) and rear pair (arm 2, mirror arm 3) differ; each right arm is the left one mirrored. Numbers: `nox/dimensions.md` "System Overseer: arms" (requirements, cases, targets) and O. Script `nox/overseer/arm_topo_v1.py` (nox/topo, organic), page `nox/overseer/arm_view_v1.py` -> `runs/overseer_arms/v1/arms.html`. Runs `runs/overseer_arm_front/v1/`, `runs/overseer_arm_rear/v1/`.

Left off v1 (Spark 27 Oct): cage nodes, spoke sockets and the L2 brace sockets. The cage pass adds them (spoke socket off the motor axis: the screw pocket under the pad is the key path) and the arms rerun.

## What each arm is
- **Root block** between the plates: 2 x M3 heat-set inserts from the top plate, 2 from the bottom plate (S500 STEP hole positions), contact faces 9 thick. The stock Ø6 pegs are dropped so the top prints flat on the bed; the 4 screws locate.
- **Motor pad** Ø36 x 6 on the arm top (z 45, the plate underside level): X2216 M3 holes as radial slots r 8..9.5 on all four 45° lines, so either hand takes the motor with its leads to the root; Ø8.6 shaft bore (CHECK the shaft clip, P); a Ø27 pocket under the pad, open below, for the screw heads and the key.
- **Front pair only: 4 x 3.0 carbon rods** (140.9 each), a tapered space frame from two shoulders beside the root to the motor pad, top and bottom on each side. Sockets 18 long at both ends; the topology's webs grip the free span with collars the bore cuts through them.
- Everything else is the topology's (organic finish).
- ESC: a 4-in-1 under the FC (Spark 27 Oct). The FC + ESC stack sits on top of the top plate (core session, 27 Oct); the motor leads run along each arm's flat top to the ESC corners, zip-tied round the top rods' free spans (front) or round the beam (rear).

## Print
PETG, Bambu A1 mini, top face (z 45 plane: root top, motor face) on the bed, tree supports, 100 % infill (the FE and the mode check assume solid PETG, E), 0.2 layers. Front arm fits the bed on the diagonal (about a 167 square at 44°). Print 1 + 1 mirrored of each pair.

## Assembly
1. Heat-set 4 M3 inserts in each root (2 from the top face, 2 from the bottom face), flush.
2. Front arms: ream each rod line with a long 3.3 drill from the open shoulder end to the blind tip socket (the flat-printed bores sag a little). Dry-fit each rod.
3. Inject epoxy into the tip socket, push the rod in from the shoulder, then wick epoxy into each grip collar and the shoulder socket. 24 h cure.
4. Motor on the pad with 4 x M3 x 8 from below through the pocket (2 mm into the X2216 base thread, 2.8 deep), leads toward the root.
5. Arm between the plates: 2 x M3 down through the top plate, 2 x M3 up through the bottom plate into the inserts (access per layout v5: 3 rear-arm bottom screws need a pod board unplugged).
6. Leads along the arm top, zip-tied, to the 4-in-1 ESC on the top plate.
