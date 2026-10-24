# Scout v3 system pass v2, latch refit (24 Oct)

Script `nox/scout/v3/layout_v2.py` (~3 min with the video, P on this box). Doc `nox/scout/v3/layout.md`; numbers `nox/dimensions.md` "System Scout v3".
Status: DESIGN; nothing saved or printed.

```
System: Scout v3, cage first 4.5 mm (E) at (+0.00, +0.66, -0.75) (flow MTF-02P), fit 0.4 mm³ (E) (Radxa Zero 3W/cage (rods + nodes)), bbox 6.95 L (E), extents 234 x 233 x 128 mm (E), mass 744 g (E), CG 1.4 mm off thrust axis (E) [+1.4, +0.0, -36.4], 2.0 mm without swap parts, probes -29.7 mm (E) edge at (-0.35, -0.64, -0.68) (receiver v1 (latch)) FAIL
Cage: 125 x 3.0 rods 6.82 m in PETG sockets, 68 nodes, ~297 g (E), preloaded TPU suspension (gives at 120 N); keep-alive pads (G (8)); latch 258becf
Strength: every case <= 1.00 (F), crash 1.5 m/s peak 120 N, stroke <= 7.8 mm
```

- `checks.txt`: candidates, dropped rods by reason, stroke limits, every case, core joint reactions, nodes.
- `result.json`, `motion.html` (artifact), `motion.mp4` / `.gif` / `.png`, `assembly/` (meshes for `scout.assembly.json`), `latch/` (the latch standard's meshes, b59b2dc), `pr44/` (superseded).
