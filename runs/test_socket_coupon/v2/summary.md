test_socket_coupon v2: ready to print
System: test coupon, no assembly (it calibrates the index every system pass uses; protocol `docs/tests.md`)
Weight: 7.88 g (M), was none
Strength: predicted epoxy pull-out (hand calc, E): 2.0 MPa x pi x 3.0 x 9.4 = 177 N per rod (bond on the rod face)
Solid: solid_check PASS
Print: plate bottom down (as saved; copied unchanged from archive/scout-v2/print/s2_socket_v2_coupon_v2.stl), 100 % infill, bed contact 1969 mm2 (M); file print/test_socket_coupon_v2.stl

## Details

- Script: `nox/tests/coupons.py` (drawn, manifold3d). Extents [66.0, 30.0, 12.0] mm (M), watertight True.
- solid_check: `print/test_socket_coupon_v2.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [1558.6], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 4, 'ok': True}`
- Copy of the Scout v2 socket coupon (`archive/scout-v2/lantern/socket_coupon.py`): bores 3.5 / 3.6 / 3.7 (1 / 2 / 3 notches), each standing and lying, 4 ribs 0.3 under the bore, 2 lock rings, bleed slot, 10 deep. Rechecked with the current solid_check: PASS.

## print_orient --all

```
print/test_socket_coupon_v2.stl extents [66. 30. 12.] g PETG 7.88 watertight True
   saved: bed   1969 mm2  overhang    288 mm2  height  12.0  first layer 1 islands [1968.6] (total 1968.6)  thin-island run 0 mm
 +x down: bed     60 mm2  overhang    766 mm2  height  66.0  first layer 1 islands [60.0] (total 60.0)  thin-island run 0 mm
 -x down: bed     60 mm2  overhang    767 mm2  height  66.0  first layer 1 islands [60.0] (total 60.0)  thin-island run 0 mm
 +y down: bed    132 mm2  overhang    500 mm2  height  30.0  first layer 1 islands [132.0] (total 132.0)  thin-island run 0 mm
 -y down: bed    118 mm2  overhang    525 mm2  height  30.0  first layer 7 islands [1.6, 1.6, 1.6, 1.6, 32.0, 37.6, 41.6] (total 117.6)  thin-island run 1 mm
 +z down: bed     93 mm2  overhang   2175 mm2  height  12.0  first layer 3 islands [34.8, 35.1, 35.5] (total 105.4)  thin-island run 0 mm
```
