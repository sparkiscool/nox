# Overseer cage frame model v1 (F, on E properties: carbon E 100 GPa, allowables 1000 / 600 MPa; buckling k 1.0 cage tubes, 0.7 links with a core end, 0.5 keels / braces (PETG both ends), E)

Cage `nox/overseer/cage_v5_shell.json` (24 nodes, 63 tubes 6/4, 495 g E) + 36 links on the core (fixed; 30 tube 6/4, the rest rod 3.0); sensor head rigid on 2 keels + 4 braces, holding node 17 and struts to nodes [15, 16, 18, 19]. Drone 3215 g: crash force 1104 N through the first-touching member, 14 directions; flight 4 g; handling 50 N at mid-span (closed form).

Lowest modes (rigid joints, lumped masses, the head 362 g at its mount): 144 Hz, 214 Hz (the head moves), 249 Hz; TPU joints and the head's own mode: runs/overseer_head_cage/concept/.

## Buckling (k L): 4 members at or over 1.0 x their Euler load
- tube 3-18: 1.59 x Pcr (L 300; crash 4 [0.0, 0.0, 1.0] via head guard)
- tube 1-19: 1.59 x Pcr (L 300; crash 4 [0.0, 0.0, 1.0] via head guard)
- tube 12-19: 1.29 x Pcr (L 417; crash 13 [-0.58, -0.58, -0.58] via node 12)
- tube 11-18: 1.29 x Pcr (L 417; crash 11 [-0.58, 0.58, -0.58] via node 11)

## Stress over 600 MPa: 0 members

## Handling 50 N at mid-span: tubes bending over 10 mm: 6
- tube 15-16: 18 mm (L 446)
- tube 18-19: 17 mm (L 438)
- tube 12-19: 15 mm (L 417)
- tube 11-18: 15 mm (L 417)
- tube 5-12: 11 mm (L 377)
- tube 5-11: 11 mm (L 377)

Worst tube buckling 1.59, worst link 0.63; node socket loads (largest axial / side per member and node over all cases) in node_loads.json.
