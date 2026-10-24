holder organic-v12: ready to print

Mass: 12.5 g (M)
Strength: worst index 0.48 in 'misuse 10' at [28.5, 7.5, 21.5] (M/F); weakest spot the camera-side arm near the battery channel
Solid: solid_check PASS (1 body, 1 eroded body, 0 necks, 0 thin walls, 0 acute edges)
Function: function check PASS (skins filled, nothing in the cuts, nothing outside the envelope)
Run: wall time 2490 s (41 min), peak 3.54 GB (M), stopped at max_stage it 96 beta 32
Look: thin round struts with fillets and open space between them, like a generative-design part

Compare organic-v10 (gen 6.0, sharp battery-channel edge): 12.50 g, worst 0.504, PASS.
v12 rounds the battery channel top edges r 3 mm (commit 611f4e8). Worst index improves 0.504 to 0.481 (stress concentration at the channel relieved); mass unchanged at 12.5 g.

Targets: solid PASS met; worst <= 0.50 met (0.481); mass about 10.3 g +-15% (8.8 to 11.8 g) NOT met, 12.5 g is 0.7 g over the upper bound, same as v10. The rounded edge buys strength, not mass.
