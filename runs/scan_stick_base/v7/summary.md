scan_stick_base v7: FAILED (solid_check FAIL on the plain body count only)
Weight: 290.7 g (M), was 234.8 g (v6 FAILED, M)
Strength: worst index 0.58 (F) in head impact 50 g (-x) at the tube / Pi-pad web (13.5, 11, 45) (M); pack side impact 0.51: every case under 0.6, sturdy
Solid: solid_check: 1 eroded body, 0 necks, 0 thin walls, every drawn body in the main body, but 5 plain bodies (M): the STL is one body (trimesh) and a 0.25 mm voxelisation finds 1 body + one stray voxel, so the extra bodies are voxel specks from cut faces flush with drawn faces (bore top on the flange bottom, keep-outs on the plate faces)
Print: not saved
Fix in the design (v8, `base_topo_v8.py`): the flush cut faces moved 0.5-2 mm off the drawn faces; rebuilt from this layout (`topo.run.rebuild`, no re-optimisation). Preview: preview.png.
