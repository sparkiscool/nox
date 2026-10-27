# Scout cage frame model v1 (F on E properties: carbon E 100 GPa, allowables 1000 / 600 MPa; buckling k 1.0 shell rods, 0.7 saddle ties)

Cage `runs/scout_layout/v11/cage_shell.json`: 17 nodes, 44 shell rods + 6 saddle ties, every rod 3.0 solid; the ties' saddle ends fixed (the core held); the camera + cradle (27 g E) rigid on its 4 clips. Drone 680 g (E): crash 233 N through the most extreme node, 14 directions; flight 4 g; handling 50 N at mid-span (closed form).

Lowest modes (rigid joints, lumped masses): 79 Hz (the camera moves), 198 Hz (the camera moves), 225 Hz (the camera moves)

## Buckling: 0 members at or over 1.0 x their Euler load

## Crash stress over 600 MPa: 0 members

## Handling 50 N at mid-span: 0 rods over 600 MPa, 0 bending over 10 mm

Worst shell rod buckling 0.40, worst tie 0.09; node socket loads (largest axial / side per member and node over all cases) in node_loads.json.
