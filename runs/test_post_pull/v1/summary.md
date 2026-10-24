test_post_pull v1: ready to print
System: test coupon, no assembly (it calibrates the index every system pass uses; protocol `docs/tests.md`)
Weight: 34.56 g (M), was none
Strength: predicted break (nox/topo as-built FE, across layers at the post root, misuse index x 0.85 / 2.5 = 1): 6x6 37.7 N (E), 4x3 13.3 N (E) (h 1.0, the parts' grid); h 0.5: 26.4 / 8.4 N (E); beam hand calc 36.7 / 8.5 N (E: Zt 25.5)
Solid: solid_check PASS
Print: plate bottom down (as saved), no supports needed (eye flanks and teardrop holes 45 deg); copy A 100 % infill, copy B 4 walls 15 % gyroid, bed contact 3782 mm2 (M); file print/test_post_pull_v1.stl

## Details

- Script: `nox/tests/coupons.py` (drawn, manifold3d). Extents [130.0, 30.0, 39.0] mm (M), watertight True.
- solid_check: `print/test_post_pull_v1.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [17778.6], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}`
- Posts printed upright like real attachment posts: 6x6 (X 6 x Y 6, hook line 25 above the root) and 4x3 (X 4 x Y 3, pulled across the 3, hook line 18), two of each. Eye on each top: Ø8 hole along X (45 deg teardrop), the eye as wide as its post (the solid gate's neck test). Plate 130 x 30 x 6, 5 x Ø5.5 through holes between the posts.
- Pull: horizontal, along Y (either way, the eye is symmetric), luggage-scale hook through the eye, the plate screwed or clamped down.
- FE: `topo.build.strength` on a sub-model per post (plate patch 30 x 30 fixed on its bottom face, 10 N on the eye's hole wall on the pull side), PETG transversely isotropic (Zt 25.5 E, Xt 51 E), misuse class; break = the load where index x 0.85 / 2.5 = 1 (linear).
  - 6x6: h 1.0 37.7 N (E) (index 0.781 at 10 N), h 0.5 26.4 N (E), hand 36.7 N (E); worst at [22.5, 12.5, 6.5] (root layer, tension face).
  - 4x3: h 1.0 13.3 N (E) (index 2.209 at 10 N), h 0.5 8.4 N (E), hand 8.5 N (E); worst at [81.5, 13.5, 6.5].
  - At h 1.0 the 4x3 post is 4 x 3 cells and the root corner is not resolved: the method reads 1.56 x the beam value; the 6x6 reads 1.03 x. The two sizes split material error from grid error.

## print_orient --all

```
print/test_post_pull_v1.stl extents [130.  30.  39.] g PETG 34.56 watertight True
   saved: bed   3782 mm2  overhang    500 mm2  height  39.0  first layer 1 islands [3781.5] (total 3781.5)  thin-island run 0 mm
 +x down: bed    180 mm2  overhang   1050 mm2  height 130.0  first layer 1 islands [180.0] (total 180.0)  thin-island run 0 mm
 -x down: bed    180 mm2  overhang   1050 mm2  height 130.0  first layer 1 islands [180.0] (total 180.0)  thin-island run 0 mm
 +y down: bed    780 mm2  overhang   1055 mm2  height  30.0  first layer 1 islands [780.0] (total 780.0)  thin-island run 0 mm
 -y down: bed    780 mm2  overhang   1055 mm2  height  30.0  first layer 1 islands [780.0] (total 780.0)  thin-island run 0 mm
 +z down: bed     19 mm2  overhang   4185 mm2  height  39.0  first layer 2 islands [25.7, 25.7] (total 51.5)  thin-island run 0 mm
```
