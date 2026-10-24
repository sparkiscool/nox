# scan_stick_thread_test v2: the stick fit coupon (24 Oct)

Drawn (OpenSCAD, `nox/overseer/scan_stick/scan_stick.scad` part `thread_test`), not topology: a test piece, not structure.
The base v22 socket exactly: Ø28.3 x 2 shank counterbore (Ø27.5 shank P Spark + 0.4 radial C), 3/4-5 ACME thread cut 26 long at z 1.5..27.5 (0.40 radial clearance C, tip thread 25 P Spark), 1 mm entry chamfer, open top. 27 engaged when the shank shoulder seats; the tip then sits 0.5 under the top face (C).

System: the pole interface of the scan stick (`dimensions.md` S, Pole / Thread rows); same cut as base v22.
Weight: 11.8 g (M), was 8.3 g (v1, 12 mm ring only)
Strength: not loaded (fit test)
Solid: solid_check PASS
Print: thread end down (276 mm² bed, M), not the larger 333 mm² counterbore face: that pose puts the shoulder floor over air and supports in the seat being tested. Supports off (closed-ring thread flanks and the 45° step print unsupported), brim 5 mm; file `print/scan_stick_thread_test_v2.stl` (sha256 57ceee0d2049), 35 x 35 x 27.5 mm (M)

Test: screw the pole in by hand. Pass: it turns in without forcing, the shank shoulder seats on the counterbore floor, the tip shows just under the top face, little wobble. Tight: `acme_clear` 0.45. Loose: 0.35. Report which.
