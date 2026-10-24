# Overseer system v2A (layout A, 24 Oct)

Spark 24 Oct: "this cage is way too flimsy ... the lidar mounted tight ... the cage can take impact and the bottom can land without toppling over, run a serious design optimization"; "weight doesn't matter now, just focus on design and keep it light"; "two cameras ... both pointing forward angled so that they capture as much of the lidar's view as possible".

`SYS_RESIZE=2 SYS_TOPO=tip_rails,chin,nose_x SYS_VIDEO=1 SYS_WRITE_ASM=1 python3 nox/overseer/overseer_system.py runs/overseer_system/v2A` (~35 min; video not committed). One script lays out the whole drone round the cage, balances it, sizes the cage on its own final geometry, checks it and writes the assembly file.

## Method
- **Layout A** (DEFAULT; alt B pack 30 aft, `runs/overseer_system/v1B`): pack on the thrust axis (balance rule). The motor square moves ahead of the plates until the CG sits on it; the wheel height steps down until the funnel fits; the L2 moves out until its field is clear and the V rods keep 12 off the front disks.
- **Cage** (`nox/overseer/cage_opt2.py`): 12-DOF beam FE, rigid joints, carbon rods 3.0 solid to 20/18 tube, PETG arms with 4 carbon spars. Cases (`dimensions.md` L): crash 35 g x 3.0 kg (E) at the touching node in 14 directions, L2 crash 35 g, handling 50 N per node, catch landing and floor landing 6 g, latch 100 N, swap pull 50 N; deflection ≤ 9, L2 and belly (down camera, flow) modes ≥ 100 Hz. Geometric constraints with per-rod size caps: prop guard (12 in plan, 12 off the swept band), funnel (1.0 off the faces, inside the 168 opening at ±5° yaw), motor bells (1.0), hook latch plungers (wheel ring t6), skid t10, rear ties t8. Fully stressed sizing per rod run, symmetric left / right, strain-energy stiffening for the modes. Topology: 1024 option sets searched on the system geometry whenever the warm-started sizing fails (`cage_search.json`: the last search, 62 feasible, on a geometry within 5 mm of this one); then the layout and the sizing alternate, and the final layout is drawn with the final sizes.
- **Two front cameras**: `runs/overseer_cams/v1` (mirror pair, 54.7 % of the L2 field).

## Result (the final layout, `checks.txt`)
- Motors 205.7 ahead of the plate centre (front arms r 372, rear r 154, C); L2 base (499.4, 0, 42), field clear (closest 1.0 behind it), thin nose rods block 0.1 % of it.
- Cage topology: rings, posts, spokes, V, keels, wheel ring, struts, nose box with rear-bottom bar, rear tower, rear skirt, belly frame + skid (fixed) + tip rails, lower keels, crossed keels, chin rods, arm spars, nose cross, top spider. Rods 765 g + spars 40 g, ~889 g with nodes (E; 3.0 solid x 12 up to 20/18 x 4).
- Frame model (F): worst rod 0.93, buckling 0.93, sockets 0.81, arm 0.10; touching node ≤ 7.5 (≤ 9); L2 mode 106 Hz, cage first mode 105 Hz; L2 crash 0.09, handling 0.48, latch 0.15; violation 0.000. Frame and mounts stay under the cage (arm 0.10, L2 0.09 vs node 0.81).
- Guard 12.6 in plan / 14.6 off the band; funnel 2.1, plungers 2.0, below the opening 82.6 of 84; tip-over ≥ 30.3° on the feet (DEFAULT ≥ 30).
- Cameras: lenses 17.4 behind the L2 base plane at y ±75; rods block 0.6 % of each image; props, arms, motors and the L2 out of both views.
- Mass ~3.42 kg (E); 634 x 533 x 193 (rear prop tips to nose, feet to top).
- **System line** (`python3 nox/assembly.py nox/overseer/overseer.assembly.json --check`, `runs/overseer_assembly/v1/check.txt`): cage first 9.5 mm, fit 0.0 mm³, balance 1.4 / 1.7 mm, probes +2.9 mm, access clear: PASS. (`checks.txt` shows fit 4.2 mm³ arms / hub blocks: the run predates the contact pair now in the script, the front hub blocks seated on the arm roots.)
- Openings (DEFAULT): the L2 field; the pack swap path (it fills the 168 opening under the wheels, the cartridge walls take a side hit there; alt end bumpers need a wider funnel); the belly sensors (under the swap path, alt a removable guard bar).
- Hub joint loads (F, `hub_loads.txt`, `python3 nox/overseer/hub_loads.py runs/overseer_system/v2A`): chin 1906 N, front hub bottom 1430 N (15 N m), front hub top 1300 N, rear hub 701 N, arm roots 733 N (110 N m), tower feet 332 N.

## Next
Printed pieces from these loads (ring and corner nodes, apex block with lugs and camera arms, front and rear hub blocks, belly pan, skid runners) and the arms with their sockets; the scan stick mount is open.
