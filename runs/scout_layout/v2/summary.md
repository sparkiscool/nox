# Scout v3 system pass v2 (24 Oct)

Script `nox/scout/v3/layout_v2.py` (~6 min, P on this box). Doc `nox/scout/v3/layout.md`; numbers `nox/dimensions.md` "System Scout v3".
Status: DESIGN, system pass 1; nothing saved or printed.

```
System: Scout v3 assembled (frame, props, electronics, PR 44 receiver + cartridge v4, cage, box keep-outs, funnel, latch, belly post)
Cage: 116 rods 6.63 m, 67 printed nodes, ~268 g (E); worst index 0.98 in top corner 135 (F); max deflection 1.96 mm (F)
Contact-first: FAIL at the tail-bottom, receiver ahead by 16.4 (C): needs receiver / cartridge >= 7.0 inside the funnel planes
Drone: ~744 g (E), CG (-1.8, -1.8, 8.9)
```

- System line (`python3 nox/assembly.py nox/scout/v3/scout.assembly.json --check`, file written by `nox/scout/v3/write_assembly.py`): System: Scout v3, cage first -13.4 mm (E) at (-0.66, +0.06, -0.75) (receiver v4 interface) FAIL, fit 0.0 mm³ (E), bbox 6.57 L (E), extents 233 x 233 x 122 mm (E), mass 725 g (E), CG 0.9 mm off thrust axis (E) [-0.9, -0.1, -38.2], 0.5 mm without swap parts
- `checks.txt`: candidates, dropped rods by reason, every case (index, deflection), core joint reactions, nodes.
- `result.json`, `motion.html` (artifact), `motion.mp4` / `.gif` / `.png`.
- `pr44/`: PR 44 meshes used as the latch-standard bottom (DEFAULT).

Next: fewer joints (coarser ground structure, node merging), CG onto the thrust centre, rerun after the receiver change.
