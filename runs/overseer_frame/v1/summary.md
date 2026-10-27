# Overseer cage frame model v1 (F, on E properties: carbon E 100 GPa, allowables 1000 / 600 MPa; buckling k 1.0 cage tubes, 0.7 links with a core end, 0.5 keels / braces (PETG both ends), E)

Cage `nox/overseer/cage_v5_shell.json` (24 nodes, 63 tubes 6/4) + 34 links 3.0 on the core (fixed); sensor head rigid on 2 keels + 4 braces, holding node 17 and wing struts to nodes [18, 19]. Drone 3196 g: crash force 1097 N through the first-touching member, 14 directions; flight 4 g; handling 50 N at mid-span (closed form).

## Buckling (k L): 4 members at or over 1.0 x their Euler load
- tube 11-18: 1.15 x Pcr (L 417; crash 3 [0.0, -1.0, 0.0] via node 9)
- tube 12-19: 1.15 x Pcr (L 417; crash 2 [0.0, 1.0, 0.0] via node 10)
- core tie left.1: 1.06 x Pcr (L 276; crash 11 [-0.58, 0.58, -0.58] via node 11)
- core tie right.1: 1.06 x Pcr (L 276; crash 13 [-0.58, -0.58, -0.58] via node 12)

## Stress over 600 MPa: 0 members

## Handling 50 N at mid-span: tubes bending over 10 mm: 6
- tube 15-16: 18 mm (L 446)
- tube 18-19: 17 mm (L 438)
- tube 12-19: 15 mm (L 417)
- tube 11-18: 15 mm (L 417)
- tube 5-12: 11 mm (L 377)
- tube 5-11: 11 mm (L 377)

Worst tube buckling 1.15, worst link 1.06; node socket loads (largest axial / side per member and node over all cases) in node_loads.json.
