# scan_stick_thread_test v3: the stick fit coupon (24 Oct)

Drawn (OpenSCAD, `nox/overseer/scan_stick/scan_stick.scad` part `thread_test`), not topology: a test piece, not structure.
Spark 24 Oct: v2 had no section for the thicker cylinder. v3 is the base v23 socket exactly: a Ø28.3 sleeve 27 deep over the Ø27.5 cylinder (P Spark, + 0.4 radial C), the shoulder seats on its floor; 3/4-5 ACME thread cut 26 long above it (0.40 radial clearance C, tip thread 25 P Spark), 1 mm entry chamfer, open top: the tip sits 0.5 under the top face (C). Section: `socket_view.png` (`nox/overseer/scan_stick/socket_view.py`).

System: the pole interface of the scan stick (`dimensions.md` S, Pole / Thread rows); same cut as base v23.
Weight: 22.3 g (M), was 11.8 g (v2, no sleeve)
Strength: not loaded (fit test)
Solid: solid_check PASS
Print: thread end down (270 mm² bed, M), not the larger 333 mm² sleeve-mouth face: that pose puts the shoulder floor over air and supports in the seat being tested. Supports off (closed-ring thread flanks and the 45° step print unsupported), brim 5 mm; file `print/scan_stick_thread_test_v3.stl` (sha256 97156bb47eed), 35 x 35 x 52.5 mm (M)

Test: screw the pole in by hand. Pass: the cylinder slides into the sleeve with little play, the thread turns in without forcing, the shoulder seats on the floor, the tip shows just under the top face. Tight thread: `acme_clear` 0.45; loose: 0.35. Tight or sloppy sleeve: say which, the bore is Ø28.3.
