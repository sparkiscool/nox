# overseer_hub_deck v8: FAILED (the build drawn-body test)

Run 27 Oct 10:21-11:26 PDT (0.99 h; smoke 0.64..1.57 h), peak 5.7 GB (M). Low brace roots (z 12 / 18), 24 cm3, h 1.25.

- Mass 64.5 g (M); **worst index 0.34** (crash 0 at the toe (104, 19, 4)): strength passes (v6 2.72 on the high roots).
- CLI `nox/solid_check.py`: PASS (`solid_check.txt`). The build drawn-body test FAILS: the +y tower cluster (body 1) is off the main body at the 2.0 erosion; its only thick joint ran through the centreline (x 86..97, y -4..2), where the cage tie tubes and two wire routes cut.
- Fix in v9 (setup): drawn bars from each tower core back to its mid feet beside the bays.
