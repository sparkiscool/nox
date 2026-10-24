# Scout v3 layout v1 (24 Oct)

Script `nox/scout/v3/layout_v1.py` (~3 min with the video, P on this box). Doc `nox/scout/v3/layout.md`; numbers `nox/dimensions.md` V.
Status: DESIGN, rough shapes; nothing saved or printed. Next: topology runs (receiver v4, cartridge v4, cage parts, camera mount).

```
Scout v3 layout v1: whole drone, before the topology runs
Weight: ~517 g (E), CG (0.04, -0.53, 9.7)
Checks: props 12.6 rods / 8.4 parts, funnel faces 2.6, opening 70.1 of 84, plungers 1.9, arm 1.0, head 2.0, path 2.0, 0 overlaps, flow and camera views clear (C)
Chosen: Radxa on the plate top, UBEC over it, pogo pads (+-10, -63); cartridge v4 92 long, XT60 at the lead end
```

- `checks.txt`: the placement search table (24 cases) and every check of the chosen layout.
- `result.json`: the numbers above and the search.
- `motion.html` (artifact), `motion.mp4` / `.gif` / `.png`: lands in the funnel, latch out, the pack drops out through the XT60 and back in, the push lifts the drone onto the plungers, latch in, take off.
- `receiver_envelope.stl`, `cartridge_v4_envelope.stl`: the design spaces handed to the topology runs.

Open (layout.md Problems): cartridge +13 for the bay modules (box), FC lowest point, plate tail and its XT60 cut-out, box numbers taken as E, tail hub on the plate, Overseer sled windows for the pogo pads, 27° nose / tail tip-over.
