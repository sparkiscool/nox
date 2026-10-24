# Scout v3 system pass v2, latch refit (24 Oct)

Script `nox/scout/v3/layout_v2.py` (~3 min with the video, P on this box). Doc `nox/scout/v3/layout.md`; numbers `nox/dimensions.md` "System Scout v3".
Status: DESIGN; nothing saved or printed.

```
System: Scout v3, cage first 4.5 mm (E) at (+0.00, +0.66, -0.75) (flow MTF-02P), fit 0.0 mm³ (E) (receiver v1 (latch)/cage (rods + nodes)), bbox 6.9 L (E), extents 233 x 233 x 128 mm (E), mass 734 g (E), CG 1.3 mm off thrust axis (E) [+1.3, +0.0, -36.2], 2.0 mm without swap parts, probes -77.8 mm (E) edge at (+0.21, -0.69, -0.69) (cartridge v5 (latch)) FAIL
Cage: 130 x 3.0 rods 7.16 m in PETG sockets, 65 nodes, ~294 g (E), on a preloaded TPU suspension (gives at 120 N); no belly post
Strength: every case <= 0.98 (F), crash 1.5 m/s peak 120 N, stroke <= 7.6 mm
```

- `checks.txt`: candidates, dropped rods by reason, stroke limits, every case, core joint reactions, nodes.
- `result.json`, `motion.html` (artifact), `motion.mp4` / `.gif` / `.png`, `assembly/` (meshes for `scout.assembly.json`), `latch/` (the latch standard's meshes, b59b2dc), `pr44/` (superseded).
