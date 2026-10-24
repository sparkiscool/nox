# scan_stick_head v18 (24 Oct)

nox/topo (organic), `unit_topo_v5.py --piece head 2.0 16 --ver v18`: the v17 layout rebuilt (`tools/topo_remap.py`, 8 design cells) with Ø3.6 rod bores (Spark: 3.3 had to be drilled on Scout v2); v17 had Ø3.2.
System: scan stick v6 (`unit_topo_v6.py`, `dimensions.md` S Pieces), the front module: L2 ring + 4 bosses, camera posts, cage rails, hooked under the spine lips + 2x M3 x 12 at z 180. Assembled: fit 0 mm³ with adapter v2 (M); spine v3 clears the head plate (v2 read 46.7 mm³ at a lip corner, M).
Weight: 167.8 g (M), was 168.2 g (v17)
Strength: worst index 0.18 (M, as-built voxel FE, misuse class) in unit 50 g 1; loads with the 90 g head estimate (E)
Solid: solid_check PASS on `print/scan_stick_head_v18.stl`
Print: plate back face down, bed 9778 mm² (M), 159.9 x 84.2 x 27.9 mm (M), tree supports; file `print/scan_stick_head_v18.stl` (sha256 8824f4e64ae6)
