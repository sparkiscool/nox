# Overseer layout v4 / cage v2: geodesic disk on the v2 layout (26 Oct 2026, concept)

Spark 26 Oct: "it needs to follow the format we had initially, with the lidar towards the nose and the arms compensation and the weight balance, also the cage needs to be in this geometric shape but not necessarily be a sphere, it needs to form itself to the drone and try to be as compact as possible, probably closer to a disk"; "think of it as adding the triangle shapes to the original drone layout we had designed, replacing that cage only"; "the battery follows the same idea, it needs to be as close to the drone body as we can, disregard the funnel for now". Supersedes layout v3 / cage v1 (sphere, L2 on top).
Script `nox/overseer/layout_v4.py` (~4 min: balance x start scan), alternatives `V4_PACK=axis` → `alt_pack_axis/`, `V4_FREQ=3` → `alt_f3/`; artifact `layout_v4_view.py`. Rule: CLAUDE.md "Cages are geodesic shells formed to the drone", `docs/generative.md` rule 0 Geodesic cage, check `system_check` shell.

Visual: https://claude.ai/artifact/Ni6GjkecK9jeu7EBqf667E (source `runs/overseer_layout/v4/motion.html`, still `preview.png`; chosen at the origin, pack-on-axis alternative at y +850, frequency 3 at y -850)

System: Overseer, cage first 9.2 mm (E) at -Z (down camera), shell 130 rods closed, h/w 0.35, fit 0.0 mm³ (E), bbox 146 L (E), extents 736 x 723 x 274 mm (E), mass 3280 of 2200 g (149 %) (E) over budget (total), CG 0.8 mm off thrust axis (E) [-0.6, +0.6, -61.4], 30.1 mm without swap parts (reported, L exception), probes 2.8 mm (E) edge at (-0.70, -0.06, -0.71) (down camera)

## Layout (C on E sizes; `checks.txt`)
- Kept from v2: L2 at the nose, axis 15° below +x, base (302.4, 0, 51); front camera horizon, portrait, 74 up the L2 back plane; hub, plates, arms shape; catch 30 aft; belly sled, pods, TPU feet; balance by moving the motor square onto the CG (now 41.7 ahead: front arms r 229, rear r 170).
- Changed: the v2 cage (guard rings, spokes, V, wheel ring, corner nodes, struts, wheels) is replaced by the shell; the belly comes up under the plates (pack top z -7 as v0; v2 hung it at -33 for the funnel's wheel height).
- Shell: frequency 2 class-I geodesic, symmetric about xz, laid on an ellipsoid of the margin hull and pushed out node by node until its convex hull holds every part with margin; start scale scanned (0.7 / 0.85 / 1.0 / cast onto the hull), smallest valid shell kept (1.0: 64.2 L; hull-cast 49.4 L is invalid, its nose camera peak tents the top with 564 tubes). 47 nodes, 130 tubes 54 .. 253 (mean 167), 84 triangles; 715 x 703 x 253 node extents, h/w 0.35; largest cell Ø125. Tubes 19.2 off the swept disks, the L2 + camera 7.0 inside, the belly 3.0 (C).
- Opening: 8 nodes on an ellipse 352 x 209 round the swap zone 240 x 136 (receiver 179.6 x 76.2 M + 30 a side), plane z -57 (cartridge bottom - 3); swap column 0 mm³ (M). TPU feet (z -61) the floor contact.
- Load paths: 12 spokes (3 per arm tip), 8 core ties (bottom plate edges to the shell, clear of the cartridge), 4 L2 ties behind its field + the v2 keels, 8 belly ties to the ring; links 23.2 off the disks.
- L2 field: 20.3 % crossed by shell tubes and nodes (E); the front camera sees no prop, arm or L2.

## Alternatives weighed (C on E)
- **Pack on the thrust axis (`alt_pack_axis/`):** pack out balanced (2.0 mm), but the motor square goes 161.9 ahead: front arms r 332 (1P band), rear r 141; 3434 g (+154). Not taken: Spark's v2 format; the pack-out offset (30.1 mm) is an L Balance exception (DEFAULT).
- **Frequency 3 (`alt_f3/`):** smaller cells, 3697 g (+417), shell 62.5 L. Not taken: Spark chose the larger cells.
- **v1 sphere (`runs/overseer_layout/v3/`):** 747 x 711 x 710, 3361 g; superseded (Spark: not a sphere).

## Carbon
- Shell: 130 tubes 6 x 4 (Spark 26 Oct; 6/4 DEFAULT wall), 21.74 m, 54 .. 253; socket bore 6.3.
- Links: 32 rods 3.0 solid, 3.94 m, shortest 63 (bay tie): all over the 33 break-even; printed struts: none. Sizes by the frame model next.

## Mass (E)
3280 g (latch set 338.5 M counted): shell 741 (tubes 529 E, nodes 212 E), links 43 (C). Budget 2200 (soft). T/W 2.01 at 1650 gf a motor (C on E). v1 sphere 3361, v0 ring cage layout 2330 (cartridge at 4 % fill). Lighter levers: 6/5 tubes, lighter nodes after the frame model.

## Next
Frame model on the shell (crash 35 g in 14 directions, floor landing on the feet, handling), then nodes and arms; new funnel round the shell (box catch session).
