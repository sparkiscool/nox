hook_latch v1: ready to print
Weight: housing A / B 17.3 g each, plunger 2.2 g (M); set of 4 units 77.9 g (M), was none (first version; replaces the electromagnet + steel belly plate)
Strength: worst index 0.47 (hand calc, E loads) in misuse 100 N on one plunger, plunger bending at the guide top; functional 30 N: 0.22 plunger, 0.15 guide wall (in-layer); no voxel FE (drawn part). Fits: 0 mm³ plunger / horn / pin / servo vs housing over the stroke, 0 mm³ extended plungers vs the catch-interface drone (R 5 wheels, cheeks, rings), gap to the wheel 1.0 (R 5) / 2.0 (R 4) (C); housing vs collar v6 433 mm³ = the v6 ribs inside the panel v7 window (spec in the part doc)
Solid: solid_check PASS on all 3 files (1 body, 1 eroded body, 0 necks, 0 thin walls)
Print: housing on its guide-side face (511 mm², the largest of the 6, M), overhang 515 mm² (M), tree supports ~3 g (E); plunger on its downhill face (220 mm², M, brim 5), overhang 15 mm²; files print/box_hook_latch_housing_A_v1.stl (x2), print/box_hook_latch_housing_B_v1.stl (x2), print/box_hook_latch_plunger_v1.stl (x4)

Design: `nox/box/hook_latch.py` (drawn, manifold3d); doc `nox/box/box-hook-latch.md`; numbers `nox/dimensions.md` G "Hook latch".
Placement: plunger axis from the wheel interface (53.6 up the slope at x ±100, C); servo pose from a 20-case search (horn r 7..13, shaft up / downhill, body direction): lightest housings tie at r 8.5 / 10 / 11.5, r 10 taken (DEFAULT).
DEFAULT: gap 1.0 to the R 5 wheel; stroke 7; MG90S dims E (D18); loads E (30 N functional per plunger, 100 N misuse).
Open: funnel panel v7 (skin windows + bosses on the 4 seated-face panels, funnel chat); first fit with the real drone; servo wiring.
Previews: preview.png (housing A), preview_housing_B.png, preview_plunger.png; motion.html / .mp4 / .gif / .png (latch close-up view).
