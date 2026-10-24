# Overseer front cameras v1 (24 Oct)

Spark 24 Oct: "two cameras on the overseer that are both pointing forward angled so that they capture as much of the lidar's view as possible".

`python3 nox/overseer/cam_cover.py runs/overseer_cams/v1/cam_cover.json` (~1 min). Coverage = share of the L2 field (the hemisphere ahead of its base plane, axis 15° below +x, uniform in solid angle, 30k directions) inside the union of the two camera frusta (pinhole, DECXIN 2633V2 89 x 73° landscape, E 4:3 from the 102° diagonal). Nelder-Mead from the best 4 grid starts per family.

| Pair | Yaw | Pitch | Roll | Share of the L2 field (C) |
|---|---|---|---|---|
| one camera, best pitch (landscape) | 0 | -30 | 0 | 27.6 % |
| **mirror, landscape (chosen, DEFAULT)** | **±46.8** | **-13.6** | **0** | **54.7 %** |
| mirror, portrait | ±49.1 | -10.7 | ±20.3 | 55.6 % (sampling noise over the ceiling) |
| stacked, landscape | 0 / 0 | 34.2 / -56.1 | 0 | 54.7 % |
| stacked, portrait | 0 / 0 | 28.8 / -60.4 | ∓0.3 | 54.4 % |
| ceiling: two frusta, no overlap, fully inside the field | | | | 54.8 % |

All pair families reach the ceiling: the two frusta just must not overlap and must stay inside the field. The mirror landscape pair is chosen because it matches the brief (both forward, angled) and needs no roll; it spans 178° across and 73° up/down round 13.6° below the horizon. Alternative: the stacked pair (one up, one down) for more vertical reach when colouring by yaw.

Placement (system model, `nox/overseer/overseer_system.py`, `runs/overseer_system/v2A`): beside the L2 at its mid height (centres y ±66), each set back behind the L2 base plane until it is out of the L2 field (17.4 mm), on a printed arm from its apex socket lug. Rods block 0.6 % of each image (C); props, arms, motors and the L2 are out of both views.
