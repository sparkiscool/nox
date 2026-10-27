# Overseer cage frame model v1 (F, on E properties: carbon E 100 GPa, allowables 1000 / 600 MPa; buckling k 1.0 cage tubes, 0.7 links with a core end, 0.5 keels / braces (PETG both ends), E)

Cage `nox/overseer/cage_v5_shell.json` (24 nodes, 63 tubes 6/4, 495 g E) + 36 links on the core (fixed; 30 tube 6/4, the rest rod 3.0); sensor head rigid on 2 keels + 4 braces, holding node 17 and struts to nodes [15, 16, 18, 19]. Drone 3214 g (E): crash force 1103 N through the first-touching member, 14 directions; flight 4 g; handling 50 N at mid-span (closed form).

Lowest modes (rigid joints, lumped masses, the head 362 g (E) at its mount): 143 Hz, 200 Hz (the head moves), 232 Hz; TPU joints and the head's own mode: runs/overseer_head_cage/concept/.

## Buckling (k L): 4 members at or over 1.0 x their Euler load
- tube 3-18: 1.68 x Pcr (L 300; crash 4 [0.0, 0.0, 1.0] via head guard)
- tube 1-19: 1.68 x Pcr (L 300; crash 4 [0.0, 0.0, 1.0] via head guard)
- tube 12-19: 1.35 x Pcr (L 417; crash 13 [-0.58, -0.58, -0.58] via node 12)
- tube 11-18: 1.35 x Pcr (L 417; crash 11 [-0.58, 0.58, -0.58] via node 11)

## Stress over 600 MPa: 0 members

## Handling 50 N at mid-span: tubes bending over 10 mm: 6
- tube 15-16: 18 mm (L 446)
- tube 18-19: 17 mm (L 438)
- tube 12-19: 15 mm (L 417)
- tube 11-18: 15 mm (L 417)
- tube 5-12: 11 mm (L 377)
- tube 5-11: 11 mm (L 377)

## Load into the arms (summed link force per group, core rigid; `arm_loads.json`)
- L2 braces +1: 1063 N (crash 4 [0.0, 0.0, 1.0] via head guard)
- L2 braces -1: 1063 N (crash 4 [0.0, 0.0, 1.0] via head guard)
- arm 1 spoke node: 1013 N (crash 7 [0.58, 0.58, -0.58] via node 18)
- arm 2 spoke node: 2354 N (crash 11 [-0.58, 0.58, -0.58] via node 11)
- arm 3 spoke node: 2354 N (crash 13 [-0.58, -0.58, -0.58] via node 12)
- arm 4 spoke node: 1013 N (crash 9 [0.58, -0.58, -0.58] via node 19)

Worst tube buckling 1.68, worst link 0.65; node socket loads (largest axial / side per member and node over all cases) in node_loads.json.
