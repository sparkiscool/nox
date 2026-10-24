# Box structure system pass v1 (`nox/box/box_system.py`)

System FE h 4 (F): 119049 cells, 520446 dofs; pieces at their own grids (box-stand.md FE grids). Index 1.0 = allowable at the case class (sustained SF 10, flight = functional SF 4, misuse SF 2.5).

## Fit (C / M)

| Pair | mm3 |
|---|---|
| cradle_keel ^ baymod@-90 | 0.15 |
| frame_endR@30 ^ catch: funnel panel A -y | 319.63 |
| frame_endR@30 ^ catch: funnel panel B -y | 319.63 |
| frame_endL@-210 ^ catch: funnel panel A +y | 319.63 |
| frame_endL@-210 ^ catch: funnel panel B +y | 319.63 |
| sheetF ^ catch: funnel panel A +x | 4794.23 |
| sheetF ^ catch: funnel panel B +x | 4794.23 |
| sheetF ^ catch: funnel panel B +y | 89.37 |
| sheetF ^ catch: funnel panel A -y | 89.37 |

## Mass and CG (M pieces, E bought / lumps)

Box 10.31 kg, CG [19.5, 82.4, -6.7]; with the Scout 10.83 kg, CG [18.6, 86.9, 2.5], tip {'+X': 39.9, '-X': 45.6, '+Y': 51.2, '-Y': 37.5}; with the Overseer 12.64 kg, CG [15.9, 99.8, 28.6], tip {'+X': 37.0, '-X': 41.8, '+Y': 46.3, '-Y': 36.3} (deg).

## Assembled worst index per piece (piece FE with the system's joint reactions)

| Instance | Piece | Own FE worst | Assembled worst | Case | At (W) |
|---|---|---|---|---|---|
| kb | kb | 0.09 | **3.77** | misuse: knocked over onto the +X side, 5 g lateral | [-17.0, -57.0, -181.0] |
| frame_endL@-210 | frame_endL | 0.9 | **1.65** | Overseer catch, 30 off toward -X -Y (237 N) | [-116.2, 150.8, 98.2] |
| frame_mid@0 | frame_mid | 0.59 | **1.60** | Overseer catch, 30 off toward +X +Y at 0.15 m/s (237 N) | [150.8, 203.2, -32.2] |
| frame_endR@30 | frame_endR | 0.86 | **1.59** | Overseer catch, 30 off toward +X +Y at 0.15 m/s (237 N) | [132.8, 156.8, 114.8] |
| frame_mid@-180 | frame_mid | 0.59 | **1.49** | Overseer catch, 30 off toward -X -Y (237 N) | [-150.8, 150.8, -29.2] |
| frame_mid@-150 | frame_mid | 0.59 | **0.80** | Overseer catch, 30 off toward -X -Y (237 N) | [-138.8, 203.2, -111.8] |
| frame_mid@-120 | frame_mid | 0.59 | **0.71** | Overseer catch, 30 off toward -X -Y (237 N) | [-89.2, 203.2, -150.8] |
| bracket_jA@165 | bracket_jA | 0.05 | **0.62** | Overseer catch, 30 off toward -X -Y (237 N) | [-168.5, 213.5, 64.5] |
| cradle_keel | cradle_keel | 0.18 | **0.60** | XT60 pull at the -90 bay 50 N | [-19.0, 139.0, -183.0] |
| mast | mast | 0.08 | **0.59** | XT60 insert at the drone 60 N | [17.0, -59.0, -43.0] |
| baymod@-60 | baymod | 0.05 | **0.55** | Overseer catch, 30 off toward +X +Y at 0.15 m/s (237 N) | [103.5, 202.5, -116.5] |
| baymod@-90 | baymod | 0.05 | **0.49** | Overseer catch, 30 off toward +X +Y at 0.15 m/s (237 N) | [32.5, 151.5, -151.5] |
| frame_mid@-30 | frame_mid | 0.59 | **0.44** | XT60 insert at the -60 bay 60 N | [114.8, 203.2, -102.8] |
| bracket_jA@-15 | bracket_jA | 0.05 | **0.37** | Overseer catch, 30 off toward +X +Y at 0.15 m/s (237 N) | [168.5, 213.5, -64.5] |
| barA | bar_half | 0.38 | **0.35** | XT60 insert at the drone 60 N | [14.2, 48.8, -198.8] |
| bracket_jB@195 | bracket_jB | 0.09 | **0.33** | Overseer catch, 30 off toward -X -Y (237 N) | [-173.5, 208.5, -53.5] |
| bracket_e@45 | bracket_e | 0.14 | **0.24** | Overseer catch, 30 off toward +X +Y at 0.15 m/s (237 N) | [136.5, 127.5, 133.5] |
| barB | bar_half | 0.38 | **0.20** | XT60 insert at the drone 60 N | [-23.2, 57.8, -177.8] |
| bracket_e@135 | bracket_e | 0.14 | **0.20** | Overseer catch, 30 off toward -X -Y (237 N) | [-136.5, 236.5, 133.5] |
| post-F0 | post_seg | 0.16 | **0.16** | XT60 pull at the -90 bay 50 N | [-180.5, 341.5, -199.5] |
| bracket_jB@15 | bracket_jB | 0.09 | **0.14** | Overseer catch, 30 off toward +X +Y at 0.15 m/s (237 N) | [173.5, 146.5, 53.5] |
| post+R2 | post_seg | 0.16 | **0.11** | Overseer catch, 30 off toward +X +Y at 0.15 m/s (237 N) | [184.5, -70.5, 94.5] |
| post+F1 | post_seg | 0.16 | **0.10** | Overseer catch, 30 off toward +X +Y at 0.15 m/s (237 N) | [187.5, 341.5, 91.5] |
| post-F1 | post_seg | 0.16 | **0.10** | Overseer catch, 30 off toward -X -Y (237 N) | [-187.5, 341.5, 91.5] |
| toe | toe | 0.19 | **0.08** | XT60 insert at the -60 bay 60 N | [9.8, 330.8, -191.2] |
| post-F2 | post_seg | 0.16 | **0.07** | Overseer catch, 30 off toward -X -Y (237 N) | [-188.5, 335.5, 94.5] |
| post+F2 | post_seg | 0.16 | **0.07** | Overseer catch, 30 off toward +X +Y at 0.15 m/s (237 N) | [188.5, 335.5, 94.5] |
| post-R2 | post_seg | 0.16 | **0.06** | Overseer catch, 30 off toward -X -Y (237 N) | [-190.5, -64.5, 238.5] |
| post+F0 | post_seg | 0.16 | **0.06** | XT60 pull at the -90 bay 50 N | [191.5, 342.5, -196.5] |
| post-R1 | post_seg | 0.16 | **0.06** | Overseer catch, 30 off toward -X -Y (237 N) | [-190.5, -64.5, 91.5] |
| post+R0 | post_seg | 0.16 | **0.05** | Overseer catch, 30 off toward +X +Y at 0.15 m/s (237 N) | [188.5, -71.5, -102.5] |
| post+R1 | post_seg | 0.16 | **0.05** | Overseer catch, 30 off toward +X +Y at 0.15 m/s (237 N) | [184.5, -70.5, -52.5] |
| post-R0 | post_seg | 0.16 | **0.02** | Overseer catch, 30 off toward -X -Y (237 N) | [-188.5, -70.5, -55.5] |

## Per case: worst piece, interface deflections (F)

| Case | Class | Worst piece | Index | Stub face mm | Bay -90 minus stub mm | Funnel pads minus stub mm |
|---|---|---|---|---|---|---|
| standing, Scout seated | sustained | kb | 0.32 | [0.013, 0.134, -0.034] | [-0.012, -0.134, 0.034] | [-0.014, -0.13, 0.006] |
| standing, Overseer seated | sustained | kb | 0.32 | [0.013, 0.134, -0.034] | [-0.012, -0.134, 0.034] | [-0.012, -0.124, -0.03] |
| uneven floor (front-right corner off), Overseer seated | sustained | kb | 0.32 | [0.013, 0.135, -0.034] | [-0.007, -0.134, 0.034] | [-0.009, -0.123, -0.031] |
| Scout catch, 30 off toward +X +Y at 0.15 m/s (53 N) | flight | frame_endR@30 | 0.38 | [0.013, 0.134, -0.034] | [0.001, -0.134, 0.038] | [-0.186, -0.136, -0.092] |
| Overseer catch, 30 off toward +X +Y at 0.15 m/s (237 N) | flight | frame_mid@0 | 1.60 | [0.013, 0.132, -0.034] | [0.055, -0.135, 0.059] | [-0.794, -0.155, -0.468] |
| Overseer catch, 30 off toward -X -Y (237 N) | flight | frame_endL@-210 | 1.65 | [0.017, 0.163, -0.034] | [-0.042, -0.145, 0.04] | [0.822, 0.04, -0.543] |
| misuse: Scout free fall 0.15 m (255 N) | misuse | frame_mid@0 | 1.03 | [0.013, 0.131, -0.034] | [0.062, -0.135, 0.061] | [-0.854, -0.163, -0.465] |
| misuse: Overseer motors cut at the gate (314 N) | misuse | frame_mid@0 | 1.30 | [0.013, 0.131, -0.034] | [0.079, -0.136, 0.067] | [-1.045, -0.165, -0.609] |
| XT60 insert at the drone 60 N | flight | kb | 0.76 | [0.202, 2.079, -0.089] | [-0.182, -2.053, 0.159] | [-0.116, -1.628, 0.397] |
| XT60 pull at the drone 50 N | flight | kb | 0.77 | [-0.047, -1.229, 0.383] | [0.048, 1.244, -0.295] | [0.049, 1.255, -0.506] |
| XT60 insert at the -90 bay 60 N | flight | kb | 1.04 | [-0.057, -1.425, 0.439] | [0.055, 1.449, -0.43] | [0.065, 1.446, -0.467] |
| XT60 insert at the -60 bay 60 N | flight | kb | 1.08 | [-1.467, -1.321, 0.403] | [1.639, 1.36, -0.318] | [1.374, 1.352, -0.456] |
| XT60 pull at the -90 bay 50 N | flight | kb | 0.64 | [0.169, 1.5, -0.117] | [-0.157, -1.492, 0.322] | [-0.107, -1.232, 0.236] |
| axle motor stall 1.8 N.m | flight | kb | 0.34 | [0.29, 0.158, -0.028] | [-0.289, -0.158, 0.028] | [-0.288, -0.151, 0.001] |
| misuse: 50 N sideways on the funnel rim | misuse | baymod@-90 | 0.17 | [0.156, 0.514, 0.002] | [0.004, -0.437, 0.194] | [0.699, -0.638, 0.269] |
| lifted by the side sheets, 2 g | flight | frame_mid@-120 | 0.56 | [0.022, 1.013, -0.344] | [-0.011, -1.018, 0.264] | [-0.022, -1.013, 0.339] |
| misuse: set down hard, 5 g | misuse | kb | 0.40 | [0.063, 0.672, -0.17] | [-0.059, -0.672, 0.169] | [-0.068, -0.66, 0.081] |
| misuse: knocked over onto the +X side, 5 g lateral | misuse | kb | 3.77 | [6.089, 0.335, 0.005] | [-5.929, -0.307, -0.163] | [-5.75, -0.24, -0.068] |

## Concept check (docs/generative.md rule 0)

- System line (`python3 nox/assembly.py nox/box/box_structure.assembly.json --check`): System: Box structure, cage first 3.0 mm (E) at -Y (PSU LRS-350-24), fit 0.2 mm³ (E) (cradle_keel/barB), bbox 70.4 L (E), extents 384 x 415 x 441 mm (E), mass 5080 g (E), CG [15.2, 107.8, -10.6] (E, no cg_ref), access 2/3 clear (bay swap -90: module out along +Y 100 blocked by cradle_keel 75 mm³) FAIL
- Alternative A (DEFAULT): box structure v2 on the 9-bay ring at R 113 (arm v6 ask): Overseer bays fit the 30 deg sector, every ring and base piece is redrawn once from v2 requirements; costs +13 mm on every radius (sheets ~+26 wide, floor -13).
- Alternative B: keep R 100, the Overseer cartridge lowered 13 (latch / Overseer sessions): keeps the stand envelopes, but box-case Problem 1 (Overseer cartridge at r ~107 in a 30 deg sector at r 100) is still open and arm v6 reads FAIL.
- Index targets: 0.5 functional / 1.0 misuse (as System Box catch, DEFAULT) vs 1.0 everywhere: the strict set fails mast, cradle_keel, bracket_jA and the bay module in addition to kb and the frames.
