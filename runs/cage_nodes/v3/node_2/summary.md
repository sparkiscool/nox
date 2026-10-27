# Node 2 (v3, main 730510c, h 1.5, direct, PETG, bores 6.2 / 3.2)
Mass 11.27 g (M), worst index 0.048 (tube to 1 pull). 6 x 6 mm, 0 x 3 mm.
nox/solid_check.py on cage_node_2.stl: **FAIL**. 1 body, but at 2.0 mm erosion 3 bodies (eroded volumes 3138.2 / 327.2 / 327.0 mm3): two sockets (about 327 mm3 each, the 6.2 sockets with their 2.5 walls) separate from the main body when the 2.0 erosion thins the Ø7 arm. Engine result.json agrees (solid_ok false, drawn_not_in_main: 0.0 at erode 2.0). Not patched; no print_orient, preview or save as a part.
