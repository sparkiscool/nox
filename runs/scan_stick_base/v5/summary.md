scan_stick_base v5: FAILED (solid_check FAIL, worst 2.3)
Weight: 188.5 g (M), was 123.4 g (v4 FAILED, M)
Strength: worst index 2.35 (F) in head impact 50 g (+x) at the flange-to-column joint (-4, -21, 109) and 2.16 under the flange (16, 3, 157); pack impact 2.08 at the ledge root (M): the generated column does not carry the 16 Nm bending (E) from the head loads
Solid: solid_check FAIL: 6 plain bodies, one strap bar detached (joint 0, 1752 mm3 at (-25, -28, 113)), 1.0 mm slivers where the lip overlaps the pack keep-out (±25.5, 13) (M)
Print: not saved
Fix in the design (v6, `base_topo_v6.py`): the column is a drawn Ø36 x 4 tube (bore Ø28 for the wires) from the thread boss to the flange, so bending is carried by a known section (~6 MPa at 16 Nm, E); strap slots through 10 mm rails instead of separate bars; lip outside the keep-out; 30 cm3 generated (E) to brace the plates to the tube. Preview: preview.png.
