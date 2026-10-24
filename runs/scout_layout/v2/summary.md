# Scout v3 system pass v2, pass 3 (24 Oct)

Script `nox/scout/v3/layout_v2.py` (~3 min with the video, P on this box). Doc `nox/scout/v3/layout.md`; numbers `nox/dimensions.md` "System Scout v3".
Status: DESIGN; nothing saved or printed.

```
System: Scout v3, cage first -18.3 mm (E) at (-0.43, +0.27, -0.86) (receiver v4 interface) FAIL, fit 0.0 mm³ (E), bbox 6.9 L (E), extents 233 x 233 x 128 mm (E), mass 748 g (E), CG 0.9 mm off thrust axis (E) [-0.9, -0.1, -36.9], 0.6 mm without swap parts
Cage: 124 x 3.0 rods 7.53 m in PETG sockets, 66 nodes, ~290 g (E), on a preloaded TPU suspension (gives at 120 N)
Strength: worst index 0.97 (latch misuse 175 N), crash 1.5 m/s peak 120 N, stroke <= 7.4 mm (F)
```

- `checks.txt`: candidates, dropped rods by reason, stroke limits, every case, core joint reactions, nodes.
- `result.json`, `motion.html` (artifact), `motion.mp4` / `.gif` / `.png`, `assembly/` (meshes for `scout.assembly.json`), `pr44/`.
