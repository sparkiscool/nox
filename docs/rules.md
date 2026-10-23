# Rules (Spark)

The short list is in `CLAUDE.md`. This page holds the detail.

## Speed first
No blocking gates except one: the solid check. Print, test, iterate. Every other check is reported, never blocks.

## Solid parts, no disconnected structures, ever
No part is saved, published, called BUILT or READY, or shown as a result unless `python3 nox/solid_check.py <mesh>` reads **PASS** on the exact file:
- (a) one body;
- (b) still one body after a 0.9 mm erosion, so nothing hangs on an edge, a corner, a sliver or a neck under ~1.8 mm;
- (c) every drawn feature (pads, cheeks, plates, slot walls, lugs) inside that eroded body, passed in as `drawn=`, and each drawn body still joined when eroded to 0.4 x its own thickness;
- (d) no wall under 1.8 mm anywhere, holes and nut pockets included (a hole needs ≥ 2.0 of material all round);
- (e) neck test: eroded again at 1.2 and 1.5 mm, no piece may split off that is ≥ 1.6 mm thicker than the neck it hung on and joins the rest over less than 4 x its own thickness² (a 5 mm pad on a 2..3 mm neck fails; a thin-walled box joined round its whole perimeter does not).

A plain body count is not enough: bracket_e v8 was "1 body" while each cheek hung on two 0.85 mm ligaments beside its M5 hole, and its topology rerun passed (a)..(d) while both bolt pads hung on 2..3 mm necks. A FAIL is fixed in the design (thicker feature, moved hole, generated material allowed to reach the feature), never patched on the mesh. Generative runs carry the check built in (`arm_topo.run_part` result `solid_ok`; `stand_v7_topo_build.py`); drawn parts call it at export (`stand_v7_parts.py`).

Fastener access: a generative setup cuts an access cylinder for every bolt head, nut and tool, **never wider than the drawn pad it serves**. Draw a boss wall ≥ 3 round any counterbore or access cut, so material can reach the pad from behind.

## Attachments are drawn (Problem 46)
Anything that ties a printed part to the carbon plate, a bolt, a rod or a mating part is drawn, never left to the generator: bolt pads ≥ Ø12 with ≥ 2.0 web round the hole, posts ≥ 6 x 6, a gusset or a 2.0 skirt from every post into the body, index ≤ 0.5 on those members until a pull coupon calibrates `printopt` (open test 20). Two parts snapped at generated plate attachments the index had passed. A bolt pad includes its boss: the wall round the bolt head's counterbore is drawn, not left to the generator.

## Design
- **Simplest design wins.** Fewer parts, features and steps beat clever ones; justify any added complexity.
- **Every design decision comes from an optimization.** Nothing goes in a spot just because it has to be somewhere.
- **Tag every number:** P measured on the part, M mesh, C CAD, E estimate, F frame model.
- **Ask Spark** only on costly or hard-to-undo choices, or before changing a shared number. Otherwise decide, mark DEFAULT, continue. Every question goes in `nox/open.md` with its best answer, one short line.
- **Replies short.** Each problem plainly, with the best fix. No em dashes.
- Everything we make is beautiful: the part, the preview, the summary, the doc.
- Drone parts: lightest that meets the loads. Box parts: strong and fits, mass not a target.
- Crash behaviour is proven by physical test, not simulation. Buy and modify over building from scratch. Goal: investor-demo prototype.

## Materials and printer
- PETG for rigid parts, TPU for nodes, feet and bumpers. No PLA.
- Nothing printed thinner than 2.0 mm; flexures excepted.
- Carbon rods: one stock, 3.0 solid, for every rod and pin in the project (`nox/dimensions.md` G). Rods epoxied, never press fit.
- Printer: Bambu A1 mini (180 cube). Every part prints on its largest flat face with tree supports (`docs/printing.md`). The bed face is picked before the run and the STL saved in that orientation; the 45° overhang filter in `noxopt` is optional (a support costs grams, not a failed print).

## No old designs
Scout v1 and every retired box and swap mechanism are gone. Never reintroduce a slide swap, slider, bridges / pusher, turret or ferris head, turntable, flat carousel, groove seat, Z stage or top-mounted battery unless Spark asks for it fresh. Scout v2 (`archive/scout-v2/`) is history: never a current design, never a source for numbers (its numbers that still hold are in `nox/dimensions.md`).
