# Box catch assembly v0 (seed, 24 Oct)

`python3 nox/assembly_seed.py box_catch`: funnel panels v6 from `nox/box/catch_v4/funnel_panel.py` (seat-frame meshes here), hook latch v1 print files posed by the inverse of `hook_latch.main`'s print poses (checked equal to the script's own seat-frame placement, 0.0 mm), seated drone wheels and ring from `hook_latch.drone_parts` (context, E). File `nox/box/box_catch.assembly.json`.

`check.txt` = `python3 nox/assembly.py nox/box/box_catch.assembly.json --check --list`:
System: Box catch, cage first n/a (no protected part), fit 961.4 mm³ (E) (funnel panel A +y/latch housing +y_x-) FAIL, bbox 8.89 L (E), extents 337 x 337 x 78 mm (E), mass 673 g (E), CG [0.0, 0.0, 42.2] (E, no cg_ref)
(961.4 per housing = hook_latch result.json housing_vs_collar_v6 961.5: the panel v7 windows and bosses are not drawn yet.)
