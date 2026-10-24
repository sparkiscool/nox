# Overseer assembly v1 (system v2, 24 Oct)

Written by `SYS_WRITE_ASM=1 python3 nox/overseer/overseer_system.py runs/overseer_system/v2A` (the file's only writer): 13 grouped parts with their meshes here, openings (L2 field, pack swap path, belly sensors), the pack swap access path, and the designed joints as contact pairs (rod ends in the arm, hub and apex sockets, pods on the belly bars, hub blocks on the plates and the front arm roots, camera plates on their arms). `check.txt` = `python3 nox/assembly.py nox/overseer/overseer.assembly.json --check --list`:

System: Overseer, cage first 9.5 mm (E) at +X (L2), fit 0.0 mm³ (E), bbox 101 L (E), extents 690 x 597 x 245 mm (E), mass 3516 g (E), CG 2.0 mm off thrust axis (E), 2.4 mm without swap parts, probes 2.9 mm (E), access 1/1 clear: PASS.

Left out (no pose yet): ESC bodies (in the arm masses), wheel pins, pogo pads, receiver and latch, scan stick mount.
