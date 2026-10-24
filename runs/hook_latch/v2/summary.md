hook_latch v2: ready to print
Weight: housing A / B 23.7 g each, plunger 3.9 g (M); set of 4 units 110.3 g (M), was 110.1 g (v1, M)
Strength: worst index 0.51 (hand calc, E loads) in misuse 100 N on one plunger at the highest held wheel centre 9.0 (C); functional 30 N: plunger 0.24, guide wall 0.14 (hand calc, in-layer). Fits (C): 0 mm³ moving parts over the stroke, bolt access, panel v7 bosses + gussets, v7 collar at all 4 units (PR 37 review), extended plungers vs the catch-interface drone; ring 9.2 and side sheets 23.1 clear
System: catch (drone + 8 wheels + collar + 4 latches + cartridge push, `nox/box/catch_system.py`): worst plunger 28.3 N in the XT60 insert, 85.8 N in a 100 N hand lift at a corner (C, E loads); assembled worst index 0.44 (hand lift); latch unit 57.5 x 49.8 x 44.4 (C), 28.7 cm³ solid (C); cage first: only wheels / nodes touch, node cheeks 1.0 off the faces (G (1), C)
Solid: solid_check PASS on all 3 files (1 body, 1 eroded body, 0 necks, 0 thin walls)
Print: housing on its valley-side face (440 mm², the largest, M), overhang 795 mm² (M), tree supports ~4 g (E); plunger on its downhill contact face (370 mm², M; the uphill face is larger but would put the rough side on the contact; brim 5 mm, docs rule, C), overhang 54 mm² (M); files print/box_hook_latch_housing_A_v2.stl (x2), print/box_hook_latch_housing_B_v2.stl (x2), print/box_hook_latch_plunger_v2.stl (x4)

Design: `nox/box/hook_latch.py` VER v2 (drawn, manifold3d); doc `nox/box/box-hook-latch.md`; numbers `nox/dimensions.md` G "Hook latch".
Change from v1: panel skin 1.6 -> 2.0 (C): guide top w -2.3, flange w -9.0..-13.0, plunger 32.95 (C); search, tolerance band, hall sensing, bolts unchanged.
Open: funnel panel v7 (PR 37) merges first; first fit with the real drone; pogo travel; servo and hall wiring (box electronics v1).
Previews: preview.png (housing A), preview_housing_B.png, preview_plunger.png; motion.html / .mp4 / .gif / .png.
