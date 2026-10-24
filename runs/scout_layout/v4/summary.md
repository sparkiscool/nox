# Scout v3 system pass v4: layout change after the review (24 Oct)

`WARM=runs/scout_layout/v3/result.json OBJ=count ESO_SEED=1 python3 nox/scout/v3/layout_v2.py` (~25 min with the video, P on this box), then `python3 nox/scout/v3/write_assembly.py && python3 nox/system_check.py nox/scout/v3/scout.assembly.json`. Doc `nox/scout/v3/layout.md` (v4 section); numbers `nox/dimensions.md` "System Scout v3". Status: DESIGN; nothing saved or printed.

```
System: see system_line.txt: cage first 5.5 mm (E), fit 0.0, CG 1.8 / 2.5 mm, access 8 / 8 clear, mass 855 g (E); probes FAIL 6 rows, worst receiver -26.9
Cage: 164 members: 98 x 3.0 carbon rods (shortest 34.3 mm, every rod >= 33) + 32 printed 6 x 6 PETG struts in 37 printed parts; 74 node sites, 11 core joints; ~374 g (E)
Strength: every case <= 1.00 (F); misuse peaks at the printed wheel corner strut (the fuse)
```

Alternatives: v3 (62 rods + 23 parts, ~318 g; 12 probe rows, bolt keys blocked); cold ESO on the v4 layout (126 + 44, ~434 g, 7 rows).
Probe rows left: receiver lower nose corners -26.9 (edge) / -4.0 (post), cartridge bottom edge -11.7, sliders -2.1 / -1.4 (latch receiver v2 / cartridge v6 TPU skids, asked 24 Oct), flow -1.4.

- `checks.txt`, `run.log`, `result.json`, `system_line.txt`, `motion.html` (https://claude.ai/artifact/2QDBKs1a2Y3az9nheVgCvP), `motion.mp4` / `.gif` / `.png`, `compare.html` / `.png` (v3 / v4, https://claude.ai/artifact/HThTP3yY7WfWDKbUqgdmz1), `assembly/`.
