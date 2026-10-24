hook_latch v1: ready to print
Weight: housing A / B 23.1 g each, plunger 3.8 g (M); set of 4 units 107.3 g (M), was none (first version; replaces the electromagnet + steel belly plate)
Strength: worst index 0.45 (hand calc, E loads) in misuse 100 N on one plunger at the highest held wheel centre 8.0 (C), plunger bending at the guide top; functional 30 N: plunger 0.21, guide wall 0.14 (in-layer); no voxel FE (drawn part). Fits: 0 mm³ plunger / horn / pin / servo vs housing over the stroke, 0 mm³ bolt head + key access at all 4 bolts, 0 mm³ extended plungers vs the catch-interface drone (C); housing vs collar v6 879 mm³ (M) = the v6 ribs inside the panel v7 window. Tolerance (C, swept): wheel −2.0 .. +1.0 along the slope held, +1.0 .. +3.5 cammed by the lead-in, ±4.5 sideways
Solid: solid_check PASS on all 3 files (1 body, 1 eroded body, 0 necks, 0 thin walls)
Print: housing on its valley-side face (570 mm², the largest, M), overhang 637 mm² (M), tree supports ~4 g (E); plunger on its downhill contact face (354 mm², M; the uphill face is 368 mm², M, but would put the rough side on the contact; brim 5 mm, docs rule), overhang 47 mm² (M); files print/box_hook_latch_housing_A_v1.stl (x2), print/box_hook_latch_housing_B_v1.stl (x2), print/box_hook_latch_plunger_v1.stl (x4)

Design: `nox/box/hook_latch.py` (drawn, manifold3d); doc `nox/box/box-hook-latch.md`; numbers `nox/dimensions.md` G "Hook latch".
Placement: plunger axis from the wheel interface (53.6 C up the slope at x ±100 C); servo pose from a 20-case search (horn r 10 .. 16 C, shaft up / downhill, body direction): r 11.5 (C) the lightest, taken.
DEFAULT: gap 1.0 (C) to the R 5 wheel; stroke 11 (C); lead-in 2.0 (C); plunger 8 x 12 (C); MG90S dims E (D18); loads E (30 N functional per plunger, 100 N misuse).
Open: funnel panel v7 (skin windows + bosses on the 4 seated-face panels, funnel chat); first fit with the real drone; pogo travel; servo wiring.
Review round 1 (FAIL) fixed: bolt access, ear-post webs, plunger pocket walls, held wheel height vs plunger top, panel boss pads.
Previews: preview.png (housing A), preview_housing_B.png, preview_plunger.png; motion.html / .mp4 / .gif / .png (latch close-up view).
