# overseer_hub_deck v1 (nox/topo, h 1.0, 10 cm3 generated, PETG)
Resumed from ck.npz after a first run was killed at it 89 (sled spike beside it); stopped at max_stage, it 131, beta 32.
- Mass 20.56 g (M) vs budget 24 (soft), +4.6 g vs the 16.0 g coupon (size reference only). 16.19 cm3, bbox 119.5 x 85.0 x 8.0, 1 body, watertight.
- Worst index 0.159 (F/E) in "press 50 N on the Pi (assembly)" at (-56.5, 19.5, 1.5); crash worst 0.051 (35 g 4/5), flight worst 0.021 (4 g), landing 0.032. All under targets (crash <= 1.0, flight/landing <= 0.5). Pi mount >= 100 Hz not computed by the run (CHECK).
- Optimiser: J rose to ~157 (beta 16) and ~195 (beta 32), worst term "held b5 mz" (a held body's tie moment about z, 3.4e4). The log gives the body only by index; b5 is one of the 12 drawn bodies (4 foot pads + 4 Pi posts + 4 more); its position is not logged. The as-built index is far under limits, so the J term is a penalty on the tie, not a strength problem.
- solid_check (exact file): runs/overseer_hub_deck/v1/overseer_hub_deck.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [9635.6], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
- The in-run gate flagged drawn_not_in_main (0.97-0.998 at erode 2.0) in result.json; the standalone check on the saved STL reads PASS. Reported, not patched.
- print_orient (saved, plate face down): runs/overseer_hub_deck/v1/overseer_hub_deck.stl extents [119.5  85.    8. ] g PETG 20.56 watertight True
   saved: bed    665 mm2  overhang   2552 mm2  height   8.0  first layer 8 islands [56.3, 56.3, 64.0, 64.1, 121.3, 121.4, 125.4, 125.5] (total 734.2)  thin-island run 0 mm
- Preview: preview.png (bed contact 705 mm2, fits A1 mini).
- Fit check (manifold3d intersection vs v5 cage, arms and core parts): bottom plate, Pi, Pi posts, arms, cage, wires touch only (0.00 mm3). FINDING: ubec_5_v 254.8 mm3 and buck_12_v 176.2 mm3 overlap the deck (their bodies start z 4.0, the deck bay floor reaches z 6.5): the bays carry no cut for the UBEC/buck bodies. Fix in v2: cut the two body envelopes (+0.3) from the bay.
- No DEFAULT taken.
