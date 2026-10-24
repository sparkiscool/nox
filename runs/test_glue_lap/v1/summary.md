test_glue_lap v1: ready to print
System: test coupon, no assembly (it calibrates the index every system pass uses; protocol `docs/tests.md`)
Weight: 3.93 g (M), was none
Strength: predicted break (hand calc, E): bond 2.0 MPa (design bond, box-funnel.md 7) x 60 mm2 = 120.0 N; the pads' own net section 1020 N in-layer
Solid: solid_check PASS
Print: flat face down (as saved), no supports; 12 halves (3 joints per adhesive x 2 adhesives), 100 % infill, bed contact 804 mm2 (M); file print/test_glue_lap_v1.stl

## Details

- Script: `nox/tests/coupons.py` (drawn, manifold3d). Extents [70.0, 20.0, 4.0] mm (M), watertight True.
- solid_check: `print/test_glue_lap_v1.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [1549.9], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}`
- One half of a single-lap joint: strip 10 wide x 4 thick, the last 6 at half thickness (bond face on top). Turn the second half over and butt each pad against the other's step: the overlap is exactly 10 x 6 and the joint is flush, so the pull line runs through the bond (little peel). Ø8 eye at each end.

## print_orient --all

```
print/test_glue_lap_v1.stl extents [70. 20.  4.] g PETG 3.93 watertight True
   saved: bed    804 mm2  overhang      0 mm2  height   4.0  first layer 1 islands [803.8] (total 803.8)  thin-island run 0 mm
 +x down: bed     20 mm2  overhang     53 mm2  height  70.0  first layer 1 islands [20.0] (total 20.0)  thin-island run 0 mm
 -x down: bed     14 mm2  overhang    103 mm2  height  70.0  first layer 1 islands [19.2] (total 19.2)  thin-island run 0 mm
 +y down: bed     14 mm2  overhang    292 mm2  height  20.0  first layer 1 islands [19.2] (total 19.2)  thin-island run 0 mm
 -y down: bed     14 mm2  overhang    292 mm2  height  20.0  first layer 1 islands [19.2] (total 19.2)  thin-island run 0 mm
 +z down: bed    744 mm2  overhang     60 mm2  height   4.0  first layer 1 islands [743.8] (total 743.8)  thin-island run 0 mm
```
