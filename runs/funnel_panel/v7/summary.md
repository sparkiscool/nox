funnel_panel v7 + v7 plain: ready to print
Weight: v7 A 80.8 g, B 80.7 g; v7 plain A 75.4 g, B 75.3 g (M, solid), was 68.0 / 67.8 (v6); collar 4 x v7 + 4 x v7 plain 624.3 g (M), was 543
Strength: worst index 0.893 (v7) / 0.91 (v7 plain) of the 1.0 mm limit in press u 60 v 90 (as-built plate FE, C; was 0.98 v6); v7 plain 6.9 MPa max; 6.6 MPa max (limit 19.5); latch bosses 0.32 / 0.72 mm under 30 N x 2 / 100 N (FE), boss root index 0.15 misuse (hand calc, across layers); weakest spot the skin bay above the latch footprint by the rim beam. Fits: collar 0.000 mm³ between panels, 1 body; hook latch housings and plungers (in and out) 0 mm³ at the 2.0 skin; window 0.5 a side round the plunger; wheel track 0 steps over 0.2 (window edges excepted). No supports
Solid: solid_check PASS (A and B v7, A and B v7 plain, on the print/ files)
Print: contact face down, bed contact 12 935 mm² v7 / 13 052 mm² v7 plain (M), tree supports 0 g (14 mm² over 45°: key undersides); files print/box_funnel_panel_A_v7.stl (x2), print/box_funnel_panel_B_v7.stl (x2), print/box_funnel_panel_A_v7_plain.stl (x2), print/box_funnel_panel_B_v7_plain.stl (x2)

Design: `nox/box/catch_v4/funnel_panel_v7.py` (drawn on `funnel_panel.py` v6) + `panel_opt_v7.py` (Mindlin plate rib optimizer, `rib_mask_v7.json`); doc `nox/box/box-funnel.md` "Part: funnel panel A / B v7"; numbers `nox/dimensions.md` G "Funnel panel v7".
DEFAULT: skin 2.0 (v6 1.6 fails solid_check thin walls); low ribs 6.7 under the housing flange, none over the guide block (spec said rib-free: leaves skin islands); drawn 4.4 tie ribs (bosses, rim, strut, seat block); rib grow 1.25 (least that passes as built).
Kept for the hook latch: `hook_latch.py` SKIN 1.6 -> 2.0 (guide top -2.3, flange -9.0); the merged v1 housing overlaps v7 242 mm³ per unit.
v7 plain (unlatched faces): `funnel_panel_v7.py --plain`, ribs from `NOX_FUNNEL_PLAIN=1 panel_opt_v7.py`; the 3 printed v6 panels fail the gate and are reprinted as v7 plain (DEFAULT).
Open: first article: glue one v7 face, 2 kg press at u 60 v 90, insert the latch housing.
Files here: STLs, result.json (latched FE + every check) and result_plain.json (plain FE), panel_opt_v7(_plain).json + logs (rib runs), solid_*.txt, preview.png (A v7), preview_B.png, preview_plain.png, motion.html / .mp4 / .gif / .png (collar with the 4 latches, plungers out and in), collar_v7_seat_frame.stl.
