# Hub v7 concept: rule checks on the assembled layout (24 Oct 2026, swap arm worker)

`NOX_ARM=v7 python3 nox/box/swap_system.py all` with the hub v7 drawn bodies (no topology mesh yet: `arm_kinematics` loads `runs/hub/v7/box_hub_v7.stl` when it exists, else the drawn hub) and the axle loop on the 16T (`L.P20_OD` = 16T pitch d in v7). Log `runs/arm/v7/all_log.txt`.

- Fit 0.0 mm³ at all 10 poses (M); hub / belt_axle is a checked pair (`arm_kinematics` ALLOWED excludes it).
- Access 9/9 at every bay pose (M): rail M5 keys, motor M3 keys.
- Swap, XT60, power cut and head-out numbers unchanged from the plate 74.35 recheck (F; the hub is a frame beam in the model).
- Mass 5591 g at park (E; Scout cycle), 5923 g Overseer.
- Hub gaps (`hub_v6.checks()`, v7 stack, loop as strands): min 0.30 arm / tower (the pivot seat), 0.90 arm / ring, 0.99 hub / 80T, 1.00 springs / tower, 1.05 hub / loop- (the column), 1.35 hub / wheel-axle tails (C).
- Tower (drawn minus cuts) solid_check PASS; the drawn set's only islands are the r 78 pads and the tie sockets (joined by the topology).
