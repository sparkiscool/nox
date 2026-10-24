scan_stick_unit v3: system pass (coarse), no print file
System: scan stick, coarse system solve at h 4.0 (C) over the joined envelope with every system case (42 cases, E); cut = the bolted joint at z 168 (C); requirements in dimensions.md "System scan stick"; joint reactions by statics in joint_reactions.txt (F)
Weight: none (coarse layout only)
Strength: none at h 2.0: the fine stage (860,160 cells, 650,337 dofs, multigrid, M) was killed twice for memory on the 15 GB box (P); the piece pass runs each piece at h 2.0 instead
Solid: not run
Print: not saved

Run: unit_topo_v3.py (v2 + guards: L2 U frame rails, Pi guard, UBEC guard, pack posts and ring; drop-in pack pocket with a 3 mm pin; L2 and camera 8 forward; stepped camera frustum; default engine Settings(), organic, 6.25 mm members, C). Coarse stage: 33 iterations, peak 2.05 GB (M). TPU cage node v1: scan_stick_cage_node_v1.stl, 1.95 g (E), solid_check PASS.
Next: scan_stick_pieces_v1.py head (this box) and base (sibling session), then the assembled recheck (assembly_view.py with SCAN_UNIT=v3 and the piece files).
