# Scout cage frame model v1 (F on E properties: carbon E 100 GPa, allowables 1000 / 600 MPa; buckling k 1.0 shell rods, 0.7 saddle ties)

Cage `runs/scout_layout/v12/cage_shell.json`: 18 nodes, 47 shell rods + 8 saddle ties, every rod 3.0 solid; the ties' saddle ends fixed (the core held); the camera + cradle (27 g E) rigid on its 4 clips. Drone 730 g (E): crash 250 N through the most extreme node, 14 directions; flight 4 g; handling 50 N at mid-span (closed form).

Lowest modes (rigid joints, lumped masses): 138 Hz (the camera moves), 209 Hz (the camera moves), 301 Hz

## Buckling: 0 members at or over 1.0 x their Euler load

## Crash stress over 600 MPa: 0 members

## Handling 50 N at mid-span: 0 rods over 600 MPa, 0 bending over 10 mm

Worst shell rod buckling 0.59, worst tie 0.09; node socket loads (largest axial / side per member and node over all cases) in node_loads.json.
