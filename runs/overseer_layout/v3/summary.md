# Overseer layout v3 / cage v1: geodesic sphere (26 Oct 2026, concept)

Spark 26 Oct (Elios 3 photo): every future cage in this format, the Overseer's v0 ring cage scrapped; "ignore how the funnel is right now, we will create a new funnel for this design ... make sure the opening at the bottom is large enough with room to spare for the battery and the swapping"; "go with the larger cells that save weight, since we will use 6mm diameter tubes".
Script `nox/overseer/layout_v3.py` (chosen, ~20 s), `V3_L2=nose ... runs/overseer_layout/v3/alt_nose` (alternative), `layout_v3_view.py` (artifact). Rule: CLAUDE.md "Cages are geodesic spheres", `docs/generative.md` rule 0 Geodesic cage, check `system_check` shell.

Visual: https://claude.ai/artifact/8Z7jnbm67Eq2DL47tJnUL8 (source `runs/overseer_layout/v3/motion.html`; chosen on the right, the nose alternative 900 to its left)

System: Overseer, cage first 15.0 mm (E) at -Z (cartridge), shell 117 rods closed, h/w 0.95, fit 0.0 mm³ (E), bbox 377 L (E), extents 747 x 711 x 710 mm (E), mass 3332 of 2200 g (152 %) (E) over budget (total), CG 1.2 mm off thrust axis (E) [+0.7, +1.0, -104.2], 1.6 mm without swap parts, probes 8.5 mm (E) edge at (-0.01, -0.71, -0.71) (cartridge)

## Layout (C on E sizes)
- Shell: class-I icosahedral geodesic, frequency 2, R 363.2 at (13.8, 0, 82.0) (C), the smallest sphere that holds the swept prop disks 12 off (3D) and every other part 3 inside its inner face planes (0.934 R). 42 nodes, 117 tubes, 76 triangles; largest cell Ø124 between tube surfaces (C). Tubes 20.6 off the swept disks (C).
- Opening: ring of 6 nodes at z -242.8, clear Ø305 (C) for the swap zone 240 x 136 = receiver 179.6 x 76.2 (M) + 30 a side (DEFAULT), diagonal Ø276: 14.5 a side to spare beyond the 30 (C). Swap zone column 0 mm³ (M). Cartridge 3 above the ring plane, TPU feet 12 below it.
- L2 on top, axis up (DEFAULT 26 Oct, as the photo): base (13.8, 0, 122.1), every non-cage point under its -6° cone; seat on an 8-rod 3.0 mast from the arm-top screw pads (core, not shell). 9.8 % of its upper hemisphere crossed by tubes and nodes (E). Front camera rigid to the seat, 30° up, props in 7 % of its view (E, mask).
- Pack bay on the thrust axis at the opening; hanger of 8 3.0 rods from the arm roots to the receiver, 8 ties up into the shell; 12 spokes (3 per arm tip, 12.3 off the disks).
- Motors 13.8 ahead of the plate centre (balance), front arms r 207, rear r 188 (C).

## Alternatives weighed (C on E)
- **L2 at the nose (24 Oct mount) inside the sphere:** symmetric arms, the pack 76 aft to balance; opening Ø440, 3304 g, but pack + cartridge out puts the CG 24.3 mm off the thrust axis (L Balance ≤ 3: FAIL); balancing it with the pack on the axis needs front arms r ~324 (into 1P). Wins: Spark's 24 Oct L2 pointing. Loses: balance.
- **Frequency 3 (the photo's cell density):** cells Ø~80, 3819 g (+487 g), T/W 1.73. Spark chose the larger cells.
- **Pack under the plates (not drawn):** shorter hanger, but the swap head would reach ~180 into the sphere; rejected for the swap.

## Carbon
- Shell: 117 tubes 6 x 4 (Spark 26 Oct; 6/4 DEFAULT wall, 6/5 alt ~-270 g E), 24.81 m, 163 .. 270 long; socket bore 6.3.
- Links: 36 rods 3.0 solid, 6.56 m, shortest 75 (mast leg), longest 261 (bay tie): every one over the 33 break-even; printed struts: none. Sizes by the frame model next.

## Mass (E)
3332 g (v2 2330 E, which counted the latch set at 4 % fill; here 338.5 M): shell 802 (tubes 604 E, nodes 198 E), links 72 (C). Budget 2200 (soft): +1132. T/W 1.98 at 1650 gf a motor (C on E). The grams buy the full sphere Spark asked for; lighter levers: 6/5 tubes (~-270 E), lighter nodes after the frame model.

## Next
Frame model on the shell (crash 35 g in 14 directions, floor landing on the feet, handling, pack hanger at 35 g), then nodes and arms; new funnel round the shell (box catch session); T/W for Spark.
