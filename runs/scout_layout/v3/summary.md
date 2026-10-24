# Scout v3 system pass v3: carbon where its length earns it, fewest pieces (24 Oct)

Script `nox/scout/v3/layout_v2.py` (OBJ count, ESO_SEED 1, L_CARBON 33; ~12 min with the video, P on this box). Doc `nox/scout/v3/layout.md` (v3 section); numbers `nox/dimensions.md` "System Scout v3". Status: DESIGN; nothing saved or printed.

```
System: Scout v3, cage first 4.5 mm (E), fit 0.0 mm³ (E), bbox 6.95 L (E), extents 234 x 233 x 128 mm (E), mass 765 g (E), CG 1.3 mm off thrust axis (E), 1.9 mm without swap parts, probes -27.8 mm (E) edge (receiver v1 (latch)) FAIL
Cage: 139 members: 90 x 3.0 carbon (shortest 35.5 mm) = 62 pieces; 49 printed (30 x 6 x 6 struts, 19 x 3 x 12 webs) in 23 printed parts; 70 node sites, 12 core joints; ~318 g (E)
Before (v2): 125 carbon (shortest 16.3 mm) = 99 pieces, 57 printed nodes, ~297 g
Strength: every case <= 1.00 (F), crash 1.5 m/s peak <= 120 N, stroke <= 7.4 mm
```

Seeds (pieces + printed parts, g): count 1: 62 + 23, 318 (kept); count 2: 62 + 26, 316; mass 2: 81 + 34, 301; mass 1: 90 + 33, 312.
Probes: 21 failing part lines (v2 23), worst -27.8 (v2 -29.7); see `system_line.txt`, `checks.txt`.

- `checks.txt`, `run.log`, `result.json`, `motion.html` (artifact https://claude.ai/artifact/2QDBKs1a2Y3az9nheVgCvP), `motion.mp4` / `.gif` / `.png`, `compare.html` / `.png` (before / after, https://claude.ai/artifact/96yG1XcgYt5DdxEBX9PdAA), `assembly/` (meshes for `scout.assembly.json`).
