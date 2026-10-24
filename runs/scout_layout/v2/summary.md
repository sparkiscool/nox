# Scout v3 system pass v2, latch refit (24 Oct)

Script `nox/scout/v3/layout_v2.py` (~3 min with the video, P on this box). Doc `nox/scout/v3/layout.md`; numbers `nox/dimensions.md` "System Scout v3".
Status: DESIGN; nothing saved or printed.

```
System: Scout v3, cage first 4.5 mm (E) at (+0.00, +0.66, -0.75) (flow MTF-02P), fit 0.0 mm³ (E) (receiver v1 (latch)/cage (rods + nodes)), bbox 6.95 L (E), extents 234 x 233 x 128 mm (E), mass 761 g (E), CG 1.3 mm off thrust axis (E) [+1.3, +0.0, -35.9], 1.9 mm without swap parts, probes -76.4 mm (E) edge at (+0.24, +0.68, -0.69) (cartridge v5 (latch)) FAIL
Cage: 141 x 3.0 rods 8.00 m in PETG sockets, 65 nodes, ~319 g (E), preloaded TPU suspension (gives at 120 N); keep-alive pads on the lower ring (G (8))
Strength: every case <= 1.00 (F), crash 1.5 m/s peak 120 N, stroke <= 7.4 mm
```

- `checks.txt`: candidates, dropped rods by reason, stroke limits, every case, core joint reactions, nodes.
- `result.json`, `motion.html` (artifact), `motion.mp4` / `.gif` / `.png`, `assembly/` (meshes for `scout.assembly.json`), `latch/` (the latch standard's meshes, b59b2dc), `pr44/` (superseded).
