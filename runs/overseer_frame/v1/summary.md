# Overseer cage frame model v1 (F, on E properties: carbon E 100 GPa, allowables 1000 / 600 MPa; buckling k 1.0 cage tubes, 0.7 links with a core end, 0.5 keels / braces (PETG both ends), E)

Cage `nox/overseer/cage_v5_shell.json` (24 nodes, 63 tubes 6/4, 496 g E) + 31 links on the core (fixed; 25 tube 6/4, the rest rod 3.0); sensor head rigid on 0 keels + 0 braces, holding node 17 and struts to nodes []. Drone 3405 g (E): crash force 1169 N through the first-touching member, 14 directions; flight 4 g; handling 50 N at mid-span (closed form).

Lowest modes (rigid joints, lumped masses, the head 404 g (E) at its mount): 104 Hz (the head moves), 123 Hz (the head moves), 148 Hz; TPU joints and the head's own mode: runs/overseer_head_cage/concept/.

## Buckling (k L): 3 members at or over 1.0 x their Euler load
- tube 12-19: 1.59 x Pcr (L 417; crash 13 [-0.58, -0.58, -0.58] via node 12)
- tube 11-18: 1.59 x Pcr (L 417; crash 11 [-0.58, 0.58, -0.58] via node 11)
- tube 15-16: 1.21 x Pcr (L 446; crash 8 [0.58, -0.58, 0.58] via node 16)

## Stress over 600 MPa: 0 members

## Handling 50 N at mid-span: tubes bending over 10 mm: 6
- tube 15-16: 18 mm (L 446)
- tube 18-19: 17 mm (L 438)
- tube 12-19: 15 mm (L 417)
- tube 11-18: 15 mm (L 417)
- tube 5-12: 11 mm (L 381)
- tube 5-11: 11 mm (L 381)

## Load into the arms (summed link force per group, core rigid; `arm_loads.json`)
- L2 braces +1: 0 N (head inertia 9 [0.58, -0.58, -0.58])
- L2 braces -1: 0 N (head inertia 9 [0.58, -0.58, -0.58])
- arm 1 spoke node: 1147 N (crash 7 [0.58, 0.58, -0.58] via node 18)
- arm 2 spoke node: 2027 N (crash 11 [-0.58, 0.58, -0.58] via node 11)
- arm 3 spoke node: 2027 N (crash 13 [-0.58, -0.58, -0.58] via node 12)
- arm 4 spoke node: 1147 N (crash 9 [0.58, -0.58, -0.58] via node 19)

Worst tube buckling 1.59, worst link 0.74; node socket loads (largest axial / side per member and node over all cases) in node_loads.json.
