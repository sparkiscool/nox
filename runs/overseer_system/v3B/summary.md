# Overseer system v3B (layout B, no tail, 24 Oct)

Spark 24 Oct: "go with B for the Overseer" (pack 30 behind the thrust axis); "go with default positions, update me when you have the cage layout done with no tail"; "make sure when landing the overseer can fit in the funnel with the battery part". Earlier asks still hold: cage first, lidar mounted tight, the cage takes impact, lands without toppling, weight no cap but light, two forward cameras.

`SYS_RESIZE=1 SYS_FINAL_DM=152.2 SYS_TOPO=spine,keels_lo,keels_x,chin SYS_VIDEO=0 SYS_WRITE_ASM=1 SYS_ASM_VER=v3 python3 nox/overseer/overseer_system.py runs/overseer_system/v3B` (~25 min; `run.log`, `checks.txt`). Picture: `layout_v3B.png` (`python3 nox/overseer/render_system.py`).

## Method
- **Layout B** (Spark): the swap set's CG 30 behind the thrust axis, the catch centre on the latch set centre (24.8 behind). The motor square moves until the CG sits on it; the wheel height steps down until the funnel fits with the latch v1 set (cartridge, pack, receiver, XT60; `runs/box_system/v2/latch_ctx`); the L2 moves out until its field is clear.
- **No tail**: the top node sits on a mast at the thrust centre, braced by a keel spine from the front hub to the L2 apex; the rear tower and top bar are gone. Mast links shorter than the 33 carbon break-even are printed into one mast node block.
- **Cage** (`cage_opt2.py`): 12-DOF beam FE; crash by the energy model (2.0 m/s, 10 mm stroke, ≥ 200 N, DEFAULT) in 14 directions **on every cage node within 3 mm of the leading support** (62 hit points; a 741 g sizing passed only because one corner hit fell on a single node by 0.02 mm), L2 crash 35 g, handling 50 N, catch and floor landing 6 g, latch 175 N, receiver misuse (74 N up, 43.5 N across, 6.1 N m); L2 and belly modes ≥ 100 Hz. Per-rod caps: prop guard 12, funnel faces 1.0 and the keep-alive band G (8) 9, motor bells, camera boxes, the plate stack box, the L2 body and dome, prop wash (≤ 8 od where ≥ 15 of a rod lies in a disk in plan). Fully stressed sizing per run, cold start at each layout; topology search whenever a sizing fails.
- **Balance** (`overseer_system.py` final stage): moving the motor square moves the cage, the L2 and the set with it; only the plates (~0.3 kg) stay, so a 1.5 mm shift in the moving parts' CG moves the balance point ~16 mm. The final stage brackets the square position with the cage sized cold at each point and takes the first point with |CG - thrust centre| ≤ 1.5 and the frame passing (here the second point tried).

## Result (`checks.txt`)
- Motors 144.9 ahead of the plate centre (front arms r 317, rear r 140, tip gaps 25); catch centre x 120.1; L2 base (439.2, 0, 44), field clear (1.1 behind it), nose rods block 0.1 % of it.
- Cage topology: two octagon rings, posts, spokes, V to the L2 sockets, wheel ring + struts, nose box with rear-bottom bar, keel spine, lower and crossed keels, chin rods, arm spars, top spider, mast, rear skirt, belly frame with vertical skid legs, rails, TPU feet and printed runners. Rods 649 g + spars 31 g (C), ~800 g with nodes (E): 3.0 solid 14 x 1.77 m, 4.0 solid 16 x 1.91 m, 5.0 solid 5 x 0.43 m, 6/4 tube 23 x 3.17 m, 8/6 tube 40 x 7.21 m, 10/8 6 x 0.60 m, 12/10 4 x 0.40 m, 14/12 6 x 0.45 m, 16/14 2 x 0.11 m, 20/18 20 x 1.86 m. Shortest carbon run 47 (spine lo), all ≥ 33.
- Frame model (F): worst rod 0.80, buckling 0.99, sockets 0.92, arm 0.10; touching node ≤ 6.4 (≤ 9); L2 mode 110 Hz, belly ≥ 100 Hz (cage first mode 79 Hz, reported); L2 crash 0.07, handling 0.50, latch 0.17; violation 0.000. Crash 691 N.
- **Breaking order (L)**: worst cage node socket 0.92, arms at 0.11 of it (≤ 0.67) PASS. Held sockets (hub, L2 apex, belly frame) must hold 2400 / 1862 / 1129 N to stay at ≤ 1/1.5 of the node index: the requirement for those pieces. The worst rod (0.99, buckling) is at the node's level, reported beside it.
- Guard 12.0 plan / 17.5 band; funnel 2.6, plungers 2.0, G (8) 10.3, below the edges -1.0; the latch set's own funnel line (receiver guide posts 183.6 long here vs 179.3 in its check) is the latch session's item.
- Prop wash: each disk shadowed 19.7 % (front) / 13.9 % (rear) by cage rods in plan (C, reported; thin rods ≤ 8 od where they run in a disk).
- Cameras: lenses 17.4 behind the L2 base plane at y ±75, 54.7 % of the L2 field, rods block 0.4 % of each image.
- Landing: stance x 38..202, y ±71, CG 143 up; tip-over forward 21.4°, back 37.0°, sides 26.2 / 26.6° (DEFAULT 30: the feet must sit inside the 191 x 168 opening, a B cost; tipped forward it rests on the nose cage at ~24°).
- Mass ~3.46 kg (E). T/W 1.9 (6.6 kg static thrust, O Propulsion). 637 x 533 x 230 (rear prop tips to nose); assembled box 666 x 597 x 278.
- **System line** (`python3 nox/assembly.py nox/overseer/overseer.assembly.json --check`, `runs/overseer_assembly/v3/check.txt`): cage first 11.5 mm, fit 0.0 mm³, balance 2.3 mm pack in (≤ 3) / 11.8 pack out (≤ 15, Spark's B exception), probes +1.9 mm, access 1/1: PASS.
- Openings (DEFAULT): the L2 field; the swap path under the belly (105° about -z) for pack, cartridge and XT60; the set's ends (45° about ±x: they face the funnel pit walls 0.7 from the opening, a member there would sit in the wall; alt a shorter set from the latch session); the belly sensors. The receiver frame is structure (printed latch frame like the sled); its XT60 stays protected.
- Hub joint loads (F, `hub_loads.txt`): rear hub 2149 N (3.9 N m), chin 1299 N, front hub bottom 1056 N, top 488 N, arm roots 504 N (46 N m).
- PX4 from the CG (FRD, m): flow x -0.026, y +0.048, z +0.117; IMU x -0.146, z -0.014.

## What B costs against A (v2A)
Pack out moves the CG 11.8 (A 2.4); tip-over forward 21.4° (A 29.8°). B is shorter at the back (no tail) and puts the motors 145 ahead of the plates (A 214).

## Next
Printed pieces from these loads: ring and corner nodes, mast node block, apex block with lugs and camera arms, front and rear hub blocks (socket ratings above), belly frame corners, belly pan, skid runners; the arms with their sockets. The latch session: receiver guide posts (183.6 vs 179.3).
