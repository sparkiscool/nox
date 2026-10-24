test_layer_pull v1: ready to print
System: test coupon, no assembly (it calibrates the index every system pass uses; protocol `docs/tests.md`)
Weight: 28.57 g (M), was none
Strength: predicted break (nox/topo as-built FE, across layers): 326.8 N at h 1.0 (index 0.09 at 10 N, worst at the gauge / flare junction [43.5, 16.5, 17.5]); h 0.5 258.0 N; hand F = Zt A 330.5 N (E)
Solid: solid_check PASS
Print: plate bottom down (as saved), no supports needed (flares and teardrops 45 deg), 100 % infill, bed contact 2605 mm2 (M); file print/test_layer_pull_v1.stl

## Details

- Script: `nox/tests/coupons.py` (drawn, manifold3d). Extents [90.0, 30.0, 38.2] mm (M), watertight True.
- solid_check: `print/test_layer_pull_v1.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [14759.1], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}`
- Three necks 3.6 x 3.6 x 8.0 gauge, printed upright: a straight pull is pure across-layer tension, so measured / A = Zt (P). 45 deg flare into the plate, eye 10 x 16 with a Ø8 teardrop hole along X; the hook seats in the apex.
- 3.6 and not 3.0: the solid gate's 1.5 neck erosion must keep a core in the gauge, else the eye reads as a blob on a neck (FAIL).
- Needs a 50 kg luggage scale: 33 kgf predicted.

## print_orient --all

```
print/test_layer_pull_v1.stl extents [90.  30.  38.2] g PETG 28.57 watertight True
   saved: bed   2605 mm2  overhang    695 mm2  height  38.2  first layer 1 islands [2605.2] (total 2605.2)  thin-island run 0 mm
 +x down: bed    180 mm2  overhang    987 mm2  height  90.0  first layer 1 islands [180.0] (total 180.0)  thin-island run 0 mm
 -x down: bed    180 mm2  overhang    987 mm2  height  90.0  first layer 1 islands [180.0] (total 180.0)  thin-island run 0 mm
 +y down: bed    540 mm2  overhang   1300 mm2  height  30.0  first layer 1 islands [540.0] (total 540.0)  thin-island run 0 mm
 -y down: bed    540 mm2  overhang   1300 mm2  height  30.0  first layer 1 islands [540.0] (total 540.0)  thin-island run 0 mm
 +z down: bed     47 mm2  overhang   3395 mm2  height  38.2  first layer 3 islands [42.9, 42.9, 42.9] (total 128.7)  thin-island run 0 mm
```
