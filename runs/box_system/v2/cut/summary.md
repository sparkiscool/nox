# Box structure v2: the cut of the round 2 field and the system FE on the cut assembly (`nox/box/box_system_v2_cut.py`)

Status: RUN RECORD of candidates for the piece topology runs, not parts. Nothing goes to print/. Tags: C drawn / layout, M mesh, E estimate, F this FE model (voxel h 4, piece-labelled).

System: Box structure v2 cut, cage first 3.0 mm (E) at -Y (PSU LRS-350-24), fit 0.5 mm³ (E) (topP_2/topN_2), bbox 80 L (E), extents 418 x 407 x 470 mm (E), mass 13890 g (E), CG [8.1, 122.6, -18.7] (E, no cg_ref), access 9/9 clear

Weight: structure 5144 g (M, 38 pieces PETG 1.27 g/cm3 + 8 TPU feet 1.22), was 4919 g (E) for the round 2 field; whole box with the cartridges, arm, funnel and electronics lumps 14731 g (E), CG [16.2, 95.0, -22.2] (E, W); structure CG [7.5, 104.2, -20.1] (M).
Stiffness (F, h 4): stub tilt 0.321 mrad at 57 N + 5.9 N.m (target <= 10, k 18.37 N.m/mrad against >= 0.59); worst bay rotation about the axle relative to the stub 2.58 mrad in `misuse: Overseer motors cut` (target <= 2.6). Lower bound with only the bolt areas bonded: stub tilt 0.699 mrad, bay rotation 2.603 mrad.
Bay rotation about the axle relative to the stub by case (mrad, F; all 9 bays): standing, Overseer seated: 0.23, Overseer catch +X+Y diag: 1.98, Overseer catch -X-Y diag: 2.49, misuse: Overseer motors cut: 2.58, XT60 insert at the -90 bay 60 N: 0.20, centring at the -90 bay 7.6 N + 1.05 N.m: 0.42, XT60 insert at the -120 bay 60 N: 0.38, centring at the -120 bay 7.6 N + 1.05 N.m: 0.16, all bays: 60 N insert: 0.65, all bays: 43.5 N across: 1.64.
Strength: worst piece index 0.34 (rearP_up, all bays: 43.5 N across); targets 0.5 functional / 1.0 misuse and sustained, aim 0.8: 0 of 38 pieces over target.
Solid: solid_check PASS on 38 of 38 pieces and 8 of 8 TPU feet, run on the exact files in `pieces/` (`solid/<piece>.txt`); a FAIL is a cut that is not a candidate.

## Pieces

| Piece | Qty | Envelope X / Y / Z (M) | Mass M | Worst index (case) | Bed face, contact | Joints | solid_check |
|---|---|---|---|---|---|---|---|
| epost | 1 | 54 x 20 x 62 | 32.2 g | 0.04 (misuse: knocked over onto +X 5 g) | -Y face down, 1111 mm2 | - | PASS |
| rearP_lug | 1 | 52 x 40 x 42 | 16.1 g | 0.04 (all bays: 43.5 N across) | -X face down, 47 mm2 | link_4 1xM3; mast_leg 1xM3 | PASS |
| floorbar | 1 | 140 x 28 x 26 | 41.4 g | 0.04 (misuse: set down hard 5 g) | -Y face down, 1315 mm2 | mast_low 1xM3 | PASS |
| bridge | 1 | 134 x 69 x 68 | 129.9 g | 0.09 (Overseer catch +X+Y diag) | -Y face down, 5295 mm2 | topP_2 2xM3; topN_2 2xM3 | PASS |
| bay30 | 1 | 108 x 175 x 108 | 227.4 g | 0.24 (Overseer catch -X-Y diag) | +X face down, 101 mm2 | bay0_a 1xM3; bay0_b 4xM3; topP_1 1xM3; topP_2 2xM3; link_1 1xM3 | PASS |
| bay-30 | 1 | 95 x 145 x 174 | 364.9 g | 0.11 (all bays: 43.5 N across) | +X face down, 3415 mm2 | bay0_a 4xM3; bay0_b 4xM3; foot_leg_1 2xM3; foot_leg_2 2xM3; bay-60_a 2xM3; bay-60_b 2xM3 | PASS |
| foot_leg_1 | 1 | 28 x 32 x 45 | 19.2 g | 0.13 (Overseer catch -X-Y diag) | +X face down, 344 mm2 | bay-30 2xM3 | PASS |
| foot_leg_2 | 1 | 24 x 32 x 45 | 17.8 g | 0.16 (Overseer catch -X-Y diag) | +X face down, 325 mm2 | bay-30 2xM3 | PASS |
| bay-90 | 1 | 130 x 139 x 72 | 182.5 g | 0.16 (Overseer catch +X+Y diag) | -Z face down, 108 mm2 | bay-60_a 1xM3; bay-60_b 2xM3; bay-120_a 2xM3; bay-120_b 1xM3 | PASS |
| foot_leg_3 | 1 | 27 x 32 x 45 | 18.9 g | 0.20 (Overseer catch +X+Y diag) | -X face down, 335 mm2 | bay-150 2xM3 | PASS |
| bay-150 | 1 | 95 x 146 x 172 | 275.0 g | 0.16 (Overseer catch -X-Y diag) | -X face down, 1738 mm2 | bay-180_a 2xM3; bay-180_b 4xM3; foot_leg_3 2xM3; bay-120_a 2xM3; bay-120_b 2xM3 | PASS |
| bay-210 | 1 | 106 x 170 x 93 | 189.9 g | 0.25 (Overseer catch -X-Y diag) | +Y face down, 234 mm2 | bay-180_b 3xM3; topN_1 2xM3; topN_2 2xM3; link_3 2xM3 | PASS |
| link_1 | 1 | 62 x 68 x 87 | 80.3 g | 0.10 (all bays: 43.5 N across) | +X face down, 149 mm2 | rearP_low 3xM3; rearP_up 1xM3; bay30 1xM3; bay0_a 2xM3 | PASS |
| link_2 | 1 | 73 x 68 x 95 | 65.4 g | 0.16 (all bays: 43.5 N across) | -Y face down, 89 mm2 | mast_low 2xM3; rearN_low 3xM3; bay-120_a 3xM3 | PASS |
| link_3 | 1 | 69 x 54 x 118 | 49.0 g | 0.09 (all bays: 43.5 N across) | -Y face down, 526 mm2 | rearN_up 2xM3; bay-210 2xM3; topN_1 1xM3 | PASS |
| link_4 | 1 | 42 x 54 x 59 | 36.3 g | 0.08 (all bays: 43.5 N across) | -Y face down, 1074 mm2 | rearP_lug 1xM3; mast_leg 2xM3; bay-60_a 2xM3 | PASS |
| link_5 | 1 | 35 x 15 x 32 | 11.5 g | 0.04 (Overseer catch +X+Y diag) | -Y face down, 569 mm2 | mast_top 2xM3; rearP_up 1xM3 | PASS |
| mast_top | 1 | 169 x 100 x 75 | 141.6 g | 0.12 (all bays: 43.5 N across) | -X face down, 246 mm2 | link_5 2xM3; topP_1 1xM3; topN_1 2xM3; rearP_up 1xM3; mast_mid 3xM3 | PASS |
| mast_low | 1 | 121 x 100 x 157 | 244.6 g | 0.22 (all bays: 43.5 N across) | -Y face down, 1220 mm2 | link_2 2xM3; floorbar 1xM3; mast_mid 2xM5 | PASS |
| mast_mid | 1 | 168 x 85 x 170 | 626.4 g | 0.21 (all bays: 43.5 N across) | +Z face down, 2734 mm2 | rearP_low 1xM3; rearN_up 1xM3; mast_top 3xM3; mast_low 2xM5; mast_leg 1xM3 | PASS |
| mast_leg | 1 | 33 x 49 x 108 | 44.5 g | 0.16 (all bays: 43.5 N across) | +X face down, 238 mm2 | link_4 2xM3; rearP_lug 1xM3; mast_mid 1xM3 | PASS |
| rearP_low | 1 | 129 x 57 x 102 | 100.5 g | 0.17 (misuse: knocked over onto +X 5 g) | +Z face down, 93 mm2 | link_1 3xM3; mast_mid 1xM3; rearP_up 1xM3; rearP_up_b 2xM3 | PASS |
| rearP_up | 1 | 78 x 50 x 116 | 52.8 g | 0.34 (all bays: 43.5 N across) | -Z face down, 221 mm2 | link_1 1xM3; link_5 1xM3; topP_1 1xM3; mast_top 1xM3; rearP_low 1xM3 | PASS |
| rearP_up_b | 1 | 47 x 76 x 62 | 45.3 g | 0.06 (misuse: set down hard 5 g) | -Y face down, 464 mm2 | rearP_low 2xM3 | PASS |
| rearN_low | 1 | 46 x 40 x 113 | 44.8 g | 0.14 (all bays: 43.5 N across) | +Y face down, 93 mm2 | link_2 3xM3; rearN_up 1xM3 | PASS |
| rearN_up | 1 | 140 x 100 x 126 | 153.4 g | 0.11 (all bays: 43.5 N across) | -Z face down, 208 mm2 | link_3 2xM3; mast_mid 1xM3; rearN_low 1xM3 | PASS |
| topP_1 | 1 | 139 x 160 x 71 | 157.6 g | 0.10 (Overseer catch +X+Y diag) | -Y face down, 87 mm2 | mast_top 1xM3; rearP_up 1xM3; bay30 1xM3; topN_1 1xM3; topP_2 2xM3 | PASS |
| topP_2 | 1 | 136 x 128 x 58 | 218.5 g | 0.10 (Overseer catch +X+Y diag) | -X face down, 502 mm2 | bridge 2xM3; bay30 2xM3; topP_1 2xM3 | PASS |
| topN_1 | 1 | 136 x 157 x 76 | 193.2 g | 0.09 (Overseer catch -X-Y diag) | +X face down, 131 mm2 | mast_top 2xM3; bay-210 2xM3; link_3 1xM3; topP_1 1xM3; topN_2 2xM3 | PASS |
| topN_2 | 1 | 142 x 128 x 65 | 229.0 g | 0.10 (Overseer catch -X-Y diag) | +X face down, 502 mm2 | bridge 2xM3; bay-210 2xM3; topN_1 2xM3 | PASS |
| bay0_a | 1 | 60 x 102 x 125 | 126.1 g | 0.12 (all bays: 43.5 N across) | +Y face down, 519 mm2 | bay30 1xM3; bay-30 4xM3; link_1 2xM3; bay0_b 2xM3 | PASS |
| bay0_b | 1 | 46 x 125 x 128 | 178.8 g | 0.11 (Overseer catch +X+Y diag) | +X face down, 615 mm2 | bay30 4xM3; bay-30 4xM3; bay0_a 2xM3 | PASS |
| bay-60_a | 1 | 101 x 92 x 69 | 86.7 g | 0.18 (all bays: 43.5 N across) | +Y face down, 480 mm2 | bay-30 2xM3; bay-90 1xM3; link_4 2xM3; bay-60_b 2xM3 | PASS |
| bay-60_b | 1 | 108 x 135 x 88 | 161.0 g | 0.09 (Overseer catch -X-Y diag) | +Y face down, 115 mm2 | bay-30 2xM3; bay-90 2xM3; bay-60_a 2xM3 | PASS |
| bay-120_a | 1 | 114 x 90 x 116 | 133.9 g | 0.32 (all bays: 43.5 N across) | -Z face down, 108 mm2 | bay-90 2xM3; bay-150 2xM3; link_2 3xM3; bay-120_b 2xM3 | PASS |
| bay-120_b | 1 | 105 x 151 x 115 | 184.9 g | 0.21 (Overseer catch +X+Y diag) | -Z face down, 107 mm2 | bay-90 1xM3; bay-150 2xM3; bay-120_a 2xM3 | PASS |
| bay-180_a | 1 | 28 x 71 x 90 | 63.1 g | 0.09 (Overseer catch -X-Y diag) | -X face down, 139 mm2 | bay-150 2xM3; bay-180_b 2xM3 | PASS |
| bay-180_b | 1 | 41 x 143 x 128 | 183.9 g | 0.20 (all bays: 60 N insert) | +Y face down, 53 mm2 | bay-150 4xM3; bay-210 3xM3; bay-180_a 2xM3 | PASS |

TPU feet: 8 x O20 x 5 (E) under the floor patches of the field (M areas 2.4, 2.0, 1.8, 1.7, 1.7, 1.6, 0.9, 1.0 cm2) on O16 x 6 pads with an M3 insert.

## Cut and joints

- Source: `runs/box_system/v2/topo_r2/density.npz` (g3000), cubic x4 to a 2 mm grid, 0.5 surface (M); the drawn bodies (pads, seat posts, stub, electronics bosses) exact; keep-outs of round 2 (grown 1) cut exact; features thinner than 2.3 mm removed (voxel opening, M); each piece finished by a 0.5 mm voxel opening (r 1.2) and a smooth 0.5 iso-surface (+-0.25 mm).
- Regions (DEFAULT, planes and wedges): mast |X| < ~85, Y < 20; rear frames; seat-post bridge Y > 266; 9 bay bodies (30 deg wedges, r > 140, Y > 60); top frame Z > 110 split at X 0; links = the rest. Size splits (180 cube): mast on Z mast 2 at Z 78 (35.1 cm2); mast_12 on Z mast 1 at Z -92 (22.8 cm2); rearP on Z rearP at Z 14 (11.6 cm2); rearN on Z rearN at Z -62 (3.3 cm2); topP on Y topP at Y 152 (4.7 cm2); topN on Y topN at Y 152 (6.0 cm2); bay0 on Y bay0 at Y 148 (8.3 cm2); bay-60 on Y bay-60 at Y 138 (7.4 cm2); bay-120 on Y bay-120 at Y 122 (7.4 cm2); bay-180 on Y bay-180 at Y 130 (8.4 cm2); the Overseer bay bodies are cut between their bolt rows (DEFAULT 158 put the lap zone Y 150..166 on the middle bolt row at Y 165, so the planes sit between the rows).
- Joints: 68 bolted tab laps (111 x M3 + 2 x M5 heat-set inserts); each lap = a lug of one piece on a recess of the other (overlap 14 for M3 / 18 for M5, lug 16 / 20 wide with O12 / O16 pad and >= 2.0 web, 6 / 8 thick, insert boss wall >= 4), screw access O10 x 10 clear. Interfaces left without a bolt (contact only): 22: link_2 | rearN_low (4.1 cm2), link_5 | mast_top (4.2 cm2), topP_1 | rearP_up (1.0 cm2), mast_mid | rearP_up (8.5 cm2), rearN_low | mast_low (1.4 cm2), topN_1 | mast_top (3.1 cm2), bridge | topP_2 (1.7 cm2), bridge | topN_2 (7.5 cm2), bay-210 | link_3 (0.7 cm2), bay30 | bay0_a (4.8 cm2), bay30 | link_1 (1.1 cm2), bay-210 | link_3 (5.5 cm2) ....
- Bolt count and size iterated once from the joint wrenches of the first FE (M3 up to 6, else M5, index aim 0.8; capacities E: heat-set pull-out 300 / 700 N, lug bearing 600 / 1300 N at SF 1):

| Joint | Plane | Worst bolt index (first FE) | Was | Now |
|---|---|---|---|---|
| mast_low / link_2 | Y rear | 0.50 | 3 x M3 | 2 x M3 |
| mast_mid / rearP_low | X mast + | 1.00 | 1 x M3 | 2 x M3 |
| topP_2 / bridge | Y bridge | 2.07 | 2 x M3 | 6 x M3 |
| topN_2 / bridge | Y bridge | 2.08 | 2 x M3 | 6 x M3 |
| bay-30 / bay0_a | R-15 | 0.29 | 3 x M3 | 2 x M3 |
| bay-30 / bay0_b | R-15 | 0.25 | 3 x M3 | 2 x M3 |
| bay-210 / topN_2 | R-45 | 0.82 | 2 x M3 | 3 x M3 |
| bay-90 / bay-60_b | R-75 | 0.44 | 3 x M3 | 2 x M3 |
| bay-120_b / bay-90 | R-105 | 0.92 | 1 x M3 | 2 x M3 |
| topP_2 / bay30 | R-135 | 1.25 | 2 x M3 | 4 x M3 |
| bay-180_b / bay-150 | R-165 | 0.27 | 3 x M3 | 2 x M3 |
| bay0_a / bay-30 | R-195 | 0.24 | 3 x M3 | 2 x M3 |
| link_1 / bay0_a | Y bay | 0.52 | 3 x M3 | 2 x M3 |
| link_2 / bay-120_a | Y bay | 1.25 | 3 x M3 | 5 x M3 |
| link_3 / bay-210 | Y bay | 0.17 | 3 x M3 | 2 x M3 |
| mast_mid / mast_top | Z mast 2 | 1.24 | 3 x M3 | 5 x M3 |
| mast_low / mast_mid | Z mast 1 | 3.90 | 2 x M3 | 5 x M5 |
| bay0_a / bay0_b | Y bay0 | 0.21 | 3 x M3 | 2 x M3 |
| bay-180_a / bay-180_b | Y bay-180 | 0.15 | 3 x M3 | 2 x M3 |

19 of 68 joints changed.

## Joint wrenches (F, largest per joint over the 18 cases; every joint and case in `req/<piece>.json`)

| Joint | Max F N (case) | Max M N.m (case) | Worst bolt index (case) |
|---|---|---|---|
| mast_mid / rearP_low | 244 (misuse: knocked over onto +X) | 1.8 (misuse: knocked over onto +X) | 0.95 (misuse: knocked over onto +X) |
| mast_low / mast_mid | 172 (misuse: set down hard 5 g) | 12.8 (all bays: 43.5 N across) | 1.75 (all bays: 43.5 N across) |
| bay-210 / topN_2 | 167 (misuse: Overseer motors cut) | 3.8 (misuse: Overseer motors cut) | 0.82 (Overseer catch -X-Y diag) |
| bay-150 / foot_leg_3 | 157 (misuse: Overseer motors cut) | 0.6 (Overseer catch -X-Y diag) | 0.46 (Overseer catch -X-Y diag) |
| bay-60_b / bay-30 | 154 (misuse: knocked over onto +X) | 2.8 (misuse: knocked over onto +X) | 0.54 (misuse: knocked over onto +X) |
| topP_2 / bay30 | 147 (misuse: Overseer motors cut) | 3.2 (misuse: Overseer motors cut) | 1.27 (Overseer catch +X+Y diag) |
| mast_low / link_2 | 130 (all bays: 43.5 N across) | 2.0 (all bays: 43.5 N across) | 0.49 (Overseer catch -X-Y diag) |
| link_2 / bay-120_a | 130 (all bays: 43.5 N across) | 1.9 (misuse: knocked over onto +X) | 1.26 (all bays: 43.5 N across) |
| foot_leg_1 / bay-30 | 129 (misuse: set down hard 5 g) | 0.4 (misuse: Overseer motors cut) | 0.31 (Overseer catch -X-Y diag) |
| mast_mid / mast_top | 125 (all bays: 60 N insert) | 11.9 (all bays: 43.5 N across) | 1.25 (all bays: 43.5 N across) |
| bay-150 / bay-120_b | 124 (all bays: 60 N insert) | 1.2 (Overseer catch -X-Y diag) | 0.57 (all bays: 60 N insert) |
| bay-120_b / bay-90 | 118 (all bays: 60 N insert) | 0.9 (misuse: Overseer motors cut) | 0.92 (all bays: 60 N insert) |
| foot_leg_2 / bay-30 | 116 (Overseer catch -X-Y diag) | 0.4 (misuse: Overseer motors cut) | 0.39 (Overseer catch -X-Y diag) |
| bay-90 / bay-60_b | 112 (misuse: knocked over onto +X) | 2.5 (misuse: knocked over onto +X) | 0.57 (all bays: 60 N insert) |
| link_1 / bay0_a | 103 (all bays: 43.5 N across) | 2.8 (all bays: 43.5 N across) | 0.52 (all bays: 43.5 N across) |
| bay-120_a / bay-90 | 103 (all bays: 43.5 N across) | 0.6 (all bays: 43.5 N across) | 0.44 (all bays: 43.5 N across) |
| topN_2 / bridge | 102 (misuse: Overseer motors cut) | 5.5 (misuse: Overseer motors cut) | 2.08 (Overseer catch -X-Y diag) |
| bay-210 / topN_1 | 95 (all bays: 60 N insert) | 1.5 (misuse: Overseer motors cut) | 0.54 (all bays: 60 N insert) |

Joints with a bolt index over the 0.8 aim after the iteration: 9.

## Floor (TPU feet, unilateral, active set)

| Foot | On piece | x, y (M) | Area cm2 (M) | Standing N | Max N (case) | Cases lifted |
|---|---|---|---|---|---|---|
| foot_1 | mast_low | -40, -56 | 2.4 | 37.4 | 181 (misuse: set down hard 5 g) | 0 |
| foot_2 | foot_leg_1 | 196, 115 | 2.0 | 30.7 | 130 (misuse: set down hard 5 g) | 0 |
| foot_3 | foot_leg_3 | -196, 194 | 1.8 | 36.7 | 157 (misuse: Overseer motors cu) | 0 |
| foot_4 | bay-90 | 40, 159 | 1.7 | 13.0 | 69 (misuse: set down hard 5 g) | 1 |
| foot_5 | foot_leg_2 | 196, 202 | 1.7 | 28.3 | 116 (Overseer catch -X-Y diag) | 1 |
| foot_6 | bay-120_a | -68, 83 | 1.6 | 22.3 | 110 (misuse: set down hard 5 g) | 0 |
| foot_7 | epost | 164, -56 | 0.9 | 4.5 | 23 (misuse: set down hard 5 g) | 0 |
| foot_8 | bay-120_b | -68, 239 | 1.0 | 2.2 | 55 (Overseer catch -X-Y diag) | 6 |

Uplift per case (a foot with under 0.5 N in the unilateral solve; the two all-bays cases hold the feet, then it is a foot pulled down by more than 0.5 N): Overseer catch +X+Y diag: foot_8; misuse: Overseer motors cut: foot_8; XT60 insert at the -90 bay 60 N: foot_8; XT60 pull at the -90 bay 50 N: foot_4; misuse: head stall at the -90 bay 74 N: foot_5, foot_8; centring at the -90 bay 7.6 N + 1.05 N.m: foot_8; all bays: 60 N insert: foot_8.
At most 2 of 8 feet lift in any case (the head stall at the -90 bay); the box stays on the rest, no hold-down is needed in these cases (F). Standing, the load is uneven on the 8 soft feet: 2.2 N on the least loaded, 37.4 N on the most (the stiff structure rides on three or four of them).

## Pieces and joints over a target

Pieces: none. Every piece is at or under 0.5 functional / 1.0 misuse / 1.0 sustained in the system FE (worst 0.34); the aim for the piece runs is 0.8 of those, so the pieces have slack to lose material (the topology runs set the mass).

Joints over the 0.8 aim (bolt index = SF x (tension / pull-out + shear / bearing), worst bolt, worst case); the lug sites the cut could place are fewer than wanted there:

- mast_mid / rearP_low: index 0.95 with 1 x M3 placed (2 wanted): needs 2 x M3 or 1 x M5 on a wider section, or a longer lap region in the piece run (the cut found no more free sites inside the 180 cube and the keep-outs).
- mast_low / mast_mid: index 1.75 with 2 x M5 placed (5 wanted): needs 10 x M3 or 5 x M5 on a wider section, or a longer lap region in the piece run (the cut found no more free sites inside the 180 cube and the keep-outs).
- bay-210 / topN_2: index 0.82 with 2 x M3 placed (3 wanted): needs 3 x M3 or 1 x M5 on a wider section, or a longer lap region in the piece run (the cut found no more free sites inside the 180 cube and the keep-outs).
- topP_2 / bay30: index 1.27 with 2 x M3 placed (4 wanted): needs 4 x M3 or 2 x M5 on a wider section, or a longer lap region in the piece run (the cut found no more free sites inside the 180 cube and the keep-outs).
- link_2 / bay-120_a: index 1.26 with 3 x M3 placed (5 wanted): needs 5 x M3 or 3 x M5 on a wider section, or a longer lap region in the piece run (the cut found no more free sites inside the 180 cube and the keep-outs).
- mast_mid / mast_top: index 1.25 with 3 x M3 placed (5 wanted): needs 5 x M3 or 3 x M5 on a wider section, or a longer lap region in the piece run (the cut found no more free sites inside the 180 cube and the keep-outs).
- bay-120_b / bay-90: index 0.92 with 1 x M3 placed (2 wanted): needs 2 x M3 or 1 x M5 on a wider section, or a longer lap region in the piece run (the cut found no more free sites inside the 180 cube and the keep-outs).
- topN_2 / bridge: index 2.08 with 2 x M3 placed (6 wanted): needs 6 x M3 or 3 x M5 on a wider section, or a longer lap region in the piece run (the cut found no more free sites inside the 180 cube and the keep-outs).
- topP_2 / bridge: index 2.11 with 2 x M3 placed (6 wanted): needs 6 x M3 or 3 x M5 on a wider section, or a longer lap region in the piece run (the cut found no more free sites inside the 180 cube and the keep-outs).


## Clearance (every piece against every keep-out)

- System line (true envelopes: receivers, cartridges and their paths, arm hub / motors / parked heads, both drone-insert lanes, rail disc, drone keep-out, PSU, v9 catch, sheets): 0.5 mm³ (E) (topP_2/topN_2); access 9 / 9 (cartridge out to park), the bay bodies' +Y service withdrawal is not checked (bodies are in halves).
- Round 2's keep-outs grown 1 mm (the finish and the lap lugs may use up to 0.25 of that margin; the true surfaces stay >= 0.75 off): receivers 639 mm3; cartridge_paths 0 mm3; drone 15 mm3; catch 114 mm3; hub_27.stl 7 mm3; motors_28.stl 72 mm3; parked_29.stl 10 mm3; drone_30.stl 6 mm3; drone_31.stl 0 mm3; rail disc 84 mm3; funnel inside 218 mm3. The catch / funnel-inside / receiver overlaps are the drawn seat posts and receiver bosses (drawn wins, M).
- Fixed electronics: PSU X 42..157 Y -66..-36 Z -170..45: 0 mm3 (clear); charger X -162..-44 Y -66..-7 Z -200..-29: 0 mm3 (clear); sheet face strip Y -66..-50: 114221 mm3 (epost 23949, floorbar 25096, mast_top 1205, mast_low 38228, mast_mid 8396, rearP_up_b 9114, rearN_up 8233). **The sheet face strip Y -66..-50 is not clear**: 7 pieces carry material there (the floor bar, the e-boss post, the mast foot, three rear bosses): they are the frame members the boards sit round, listed below; if the strip is a hard keep-out these need a re-cut (alt: boards on the strip's free area, DEFAULT).

## Rear zone: frame members the boards sit round (M, sections of the cut at Y; PSU X 42..157 Z -170..45 and charger X -162..-44 Z -200..-29 are fixed; the boards are movable)

| Y | Piece | X range | Z range | Section mm2 |
|---|---|---|---|---|
| Y -62 | mast_low | -52..40 | -231..-169 | 2061 |
| Y -62 | epost | 153..206 | -231..-170 | 1927 |
| Y -62 | floorbar | -206..-106 | -226..-201 | 1630 |
| Y -62 | rearP_up_b | 159..206 | 39..61 | 655 |
| Y -62 | rearN_up | -185..-160 | -31..-12 | 313 |
| Y -62 | mast_mid | 24..43 | 44..61 | 234 |
| Y -62 | mast_mid | -43..-28 | -28..-13 | 168 |
| Y -62 | mast_top | -58..-47 | 116..126 | 77 |
| Y -50 | mast_low | -80..18 | -231..-160 | 2618 |
| Y -50 | floorbar | -162..-66 | -225..-201 | 1349 |
| Y -50 | rearN_up | -206..-155 | -35..-9 | 648 |
| Y -50 | rearP_up_b | 160..184 | 28..55 | 575 |
| Y -50 | epost | 159..193 | -220..-176 | 504 |
| Y -50 | mast_mid | -47..-24 | -34..-9 | 389 |
| Y -50 | mast_mid | 24..47 | 39..57 | 278 |
| Y -50 | mast_top | -58..-47 | 116..126 | 77 |
| Y -50 | epost | 158..170 | -231..-225 | 68 |
| Y -30 | mast_mid | -41..-4 | -92..-4 | 1841 |
| Y -30 | mast_low | -41..-8 | -176..-124 | 1201 |
| Y -30 | mast_mid | 11..52 | 35..57 | 694 |
| Y -30 | rearP_up_b | 160..185 | 15..44 | 465 |
| Y -30 | rearN_up | -185..-152 | -33..-9 | 394 |
| Y -30 | mast_top | -58..-47 | 116..126 | 77 |
| Y -30 | mast_low | -27..-22 | -118..-110 | 28 |
| Y -30 | mast_low | -26..-17 | -96..-92 | 27 |
| Y -30 | mast_low | -26..-22 | -105..-98 | 21 |
| Y -10 | mast_mid | -80..88 | -92..78 | 19501 |
| Y -10 | mast_top | -64..88 | 64..126 | 3200 |
| Y -10 | mast_low | -40..23 | -146..-92 | 2172 |
| Y -10 | rearN_up | -122..-66 | 16..64 | 1071 |
| Y -10 | rearN_up | -185..-137 | -33..-8 | 735 |
| Y -10 | rearP_up | 88..112 | 31..72 | 598 |
| Y -10 | rearP_low | 74..111 | -67..-33 | 561 |
| Y -10 | rearP_up_b | 162..192 | 0..32 | 545 |
| Y -10 | mast_leg | 58..74 | -99..-78 | 335 |
| Y -10 | rearP_low | 163..184 | -5..14 | 288 |

Full lists: `rear_zone.json`.

## Sheets (cladding clipped to the frame; clips are drawn tabs, not structure)

- sheet+X: 5 clip tabs (O10 stalk to 0.2 off the sheet, M3 insert): rearP_up_b at (173, -62, 56) stalk 33.0 mm; bay30 at (146, 208, 126) stalk 59.5 mm; epost at (170, -60, -226) stalk 35.3 mm; bay0_a at (192, 107, -38) stalk 13.5 mm; bay30 at (146, 77, 108) stalk 59.5 mm
- sheet-X: 5 clip tabs (O10 stalk to 0.2 off the sheet, M3 insert): bay-210 at (-146, 75, 109) stalk 59.5 mm; floorbar at (-168, -65, -222) stalk 38.1 mm; rearN_up at (-175, -54, -31) stalk 30.7 mm; bay-210 at (-160, 180, 92) stalk 45.8 mm; bay-150 at (-199, 207, -90) stalk 6.4 mm
- sheetR: 5 clip tabs (O10 stalk to 0.2 off the sheet, M3 insert): epost at (163, -65, -226) stalk 0.4 mm; mast_top at (-53, -7, 121) stalk 59.2 mm; floorbar at (-162, -64, -206) stalk 1.4 mm; rearP_up_b at (180, -62, 46) stalk 3.5 mm; rearN_up at (-179, -14, -26) stalk 51.6 mm
- sheetF: 5 clip tabs (O10 stalk to 0.2 off the sheet, M3 insert): bridge at (37, 290, 162) stalk 44.8 mm; bridge at (-52, 280, 138) stalk 54.8 mm; bridge at (-5, 275, 124) stalk 59.7 mm; bridge at (37, 275, 124) stalk 59.5 mm; bridge at (-1, 283, 154) stalk 52.1 mm

## Findings for the system session

- **Round 2 drawn the receiver bosses at the wrong place.** `box_system_topo_v2r2.bosses()` translated them to z = z_c + Z_T, but z_c (0.85 / 85.8 Scout, 0.85 / 85.5 / 163.8 Overseer) is already the cartridge z from the closed end: the bolt rows sat Z_T (42.55 / 85.5) off, e.g. an Overseer row at Y 1.2, outside the receiver (Y 75..255). The receiver meshes show the bolt holes at Y_C - (z_c - Z_T) (Ø3.4: the points sit 1.70 from the hole wall, M). This cut draws its own O12 x 11 bosses at those points (C) and the FE joins every receiver to its bay body there. The round 2 field is not wrong where the pads carry the load (the pads cover the whole receiver), but the loads of round 2 went through the wrong bosses: its bay rotations (0.4 / 1.8 mrad) are not the rotations of a bolted receiver.
- The field's own receiver-to-mast paths are real (the bay bodies join the mast through the links); the cut keeps them as laps.
- Seat posts: the cut keeps the 8 O16 x 8 drawn discs with an M3 insert hole in each (the catch's M3 is tight into it, C); the landing overlap with the v9 catch is reported in the System line (drawn wins, M).
- One floor patch (the e-boss island, 22 cm3) is not joined to the frame: its piece (`epost`) stands on its own TPU foot and carries 1/8 of the electronics lump straight to the floor.

## DEFAULTs taken (alternative in brackets)

- Overseer bay bodies split between the bolt rows, planes Y 122..148, not at Y 158 (lap zone on the middle row); the mast split at Z 78 and -92 so the stub plate Z +-45 is one piece; the top frame at Y 152 (task: Y 138).
- Lap = lug on recess with 2 .. 6 bolts per interface by area, M3 first, M5 where the wrench needs it (alt: plain butt flanges); bolt capacities E (heat-set pull-out 300 / 700 N, lug bearing 600 / 1300 N), index = SF x (T / T_u + V / V_u) on a rigid pin group.
- FE joints: every cut and lap face bonded (contact and clamp), bolt demand from the joint wrench; the lower bound bonds only an O12 disc round each bolt (stub tilt and bay rotation reported for both). Receivers PETG, cartridges stiff (E 10 GPa, density from the swap mass), TPU feet E 9.8 MPa, the catch rigid on its 8 seat discs (its own FE gives its compliance), the stub plate loaded as in round 2.
- The two all-bays envelope cases hold the feet (the net 164 N + 27 N.m of the arm's reaction would tip the box otherwise): foot tension is reported as the hold-down needed. All other cases are unilateral.
- Build axis for the strength allowables = the shortest extent of each piece (largest flat face down, E).

## Hardware (E)

- Laps: 111 x M3 x 10 screws into 111 M3 x 5.7 heat-set inserts, 2 x M5 x 16 into M5 x 9.5; receivers 44 bolts into 44 M3 inserts (Scout 4 x 5, Overseer 6 x 4, bosses O12 x 11); seat posts 8 inserts; feet 8 M3 inserts + 8 screws; sheet clips 20 M3 inserts.
- Pieces with a bed contact under 400 mm2 on their best face: 22 of 38 (the field is organic: tree supports and a brim, as the print rules say; the piece runs choose the pose).

## Run record

- Scripts: `nox/box/box_system_v2_cut.py` stages structure, export, fe (--mode all / bolts), sizes, solid, orient, report; PYPARDISO_MKL_RT=/usr/local/lib/libmkl_rt.so.3, 1e-4 diagonal regularisation as v1. Branch `run/box_system-v2-cut`.
- Peak memory (M, `resource.getrusage` ru_maxrss of the process, logged at exit in `log.txt`): FE h 4 4.04 GB; cut export 2.34 GB; solid_check at pitch 0.25 up to 11.61 GB for the largest piece, one at a time (an earlier run was killed by the 12 GB memory cgroup when an FE ran beside it: never two together). Largest total 11.6 GB, under the 12 GB stop.
- Wall (this box, 4 cores): about 5.0 h from the first run to the PR, of it: one cut export 7 min alone (laps 2.5 min, finish 4 min), FE h 4 18 min for the 18 cases alone (60 s each, 117696 cells, 500982 dofs), solid_check 64 min for the 46 files.
- h 4 used (aim 4; no coarser fallback needed). An h 8 debug run (22k cells) read the same stub tilt (0.31 against 0.321 mrad).
- Iterations: FE 1 on the first cut (bolt counts by area) -> bolt sizing -> cut 2 -> FE 2 (all numbers above); after FE 2 only the finish was touched (clips and feet kept inside the sheets, pieces clipped to the sheet planes, 0.3 mm), the FE ran again on that last cut.

## Appendix: exact solid_check and print_orient output per piece

### epost
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/epost.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [20309.4], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/epost.stl extents [53.9 20.4 62.5] g PETG 32.15 watertight True
   saved: bed    112 mm2  overhang   1246 mm2  height  62.5  first layer 1 islands [154.9] (total 154.9)  thin-island run 0 mm
 +x down: bed     55 mm2  overhang   1279 mm2  height  53.9  first layer 2 islands [8.6, 61.6] (total 70.2)  thin-island run 0 mm
 -x down: bed     40 mm2  overhang   1266 mm2  height  53.9  first layer 1 islands [66.7] (total 66.7)  thin-island run 0 mm
 +y down: bed     27 mm2  overhang   2503 mm2  height  20.4  first layer 2 islands [14.8, 38.2] (total 53.1)  thin-island run 2 mm
 -y down: bed   1111 mm2  overhang   1158 mm2  height  20.4  first layer 1 islands [1183.2] (total 1183.2)  thin-island run 0 mm
 +z down: bed      1 mm2  overhang   1453 mm2  height  62.5  first layer 1 islands [6.5] (total 6.5)  thin-island run 2 mm
```
### rearP_lug
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/rearP_lug.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [9558.8], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/rearP_lug.stl extents [51.9 40.  42. ] g PETG 16.13 watertight True
   saved: bed      4 mm2  overhang   1073 mm2  height  42.0  first layer 1 islands [9.9] (total 9.9)  thin-island run 0 mm
 +x down: bed      0 mm2  overhang    916 mm2  height  51.9  first layer 1 islands [2.3] (total 2.3)  thin-island run 0 mm
 -x down: bed     47 mm2  overhang    844 mm2  height  51.9  first layer 1 islands [71.6] (total 71.6)  thin-island run 0 mm
 +y down: bed     46 mm2  overhang   1046 mm2  height  40.0  first layer 1 islands [68.4] (total 68.4)  thin-island run 0 mm
 -y down: bed     13 mm2  overhang   1086 mm2  height  40.0  first layer 1 islands [25.4] (total 25.4)  thin-island run 0 mm
 +z down: bed      0 mm2  overhang   1071 mm2  height  42.0  first layer 1 islands [4.8] (total 4.8)  thin-island run 1 mm
```
### floorbar
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/floorbar.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [26333.1], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/floorbar.stl extents [139.9  28.4  25.5] g PETG 41.39 watertight True
   saved: bed    117 mm2  overhang   2573 mm2  height  25.5  first layer 1 islands [155.5] (total 155.5)  thin-island run 0 mm
 +x down: bed     44 mm2  overhang    475 mm2  height 139.9  first layer 1 islands [66.3] (total 66.3)  thin-island run 0 mm
 -x down: bed     34 mm2  overhang    588 mm2  height 139.9  first layer 1 islands [39.4] (total 39.4)  thin-island run 0 mm
 +y down: bed     29 mm2  overhang   3234 mm2  height  28.4  first layer 1 islands [62.6] (total 62.6)  thin-island run 0 mm
 -y down: bed   1315 mm2  overhang   1617 mm2  height  28.4  first layer 1 islands [1369.6] (total 1369.6)  thin-island run 0 mm
 +z down: bed    317 mm2  overhang   2345 mm2  height  25.5  first layer 1 islands [464.0] (total 464.0)  thin-island run 0 mm
```
### bridge
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bridge.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [83190.7], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bridge.stl extents [134.5  68.9  68.5] g PETG 129.85 watertight True
   saved: bed      3 mm2  overhang   5839 mm2  height  68.5  first layer 1 islands [15.0] (total 15.0)  thin-island run 0 mm
 +x down: bed      1 mm2  overhang   4828 mm2  height 134.5  first layer 1 islands [4.8] (total 4.8)  thin-island run 0 mm
 -x down: bed      6 mm2  overhang   4711 mm2  height 134.5  first layer 1 islands [26.1] (total 26.1)  thin-island run 0 mm
 +y down: bed    255 mm2  overhang   7131 mm2  height  68.9  first layer 5 islands [61.1, 61.6, 61.6, 61.8, 62.1] (total 308.3)  thin-island run 0 mm
 -y down: bed   5295 mm2  overhang   1465 mm2  height  68.9  first layer 1 islands [5538.1] (total 5538.1)  thin-island run 0 mm
 +z down: bed      1 mm2  overhang   6147 mm2  height  68.5  first layer 2 islands [5.1, 5.1] (total 10.2)  thin-island run 0 mm
```
### bay30
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bay30.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [149239.8], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bay30.stl extents [108.2 175.  108.3] g PETG 227.44 watertight True
   saved: bed      0 mm2  overhang  11584 mm2  height 108.3  first layer 1 islands [1.8] (total 1.8)  thin-island run 0 mm
 +x down: bed    101 mm2  overhang  12053 mm2  height 108.2  first layer 2 islands [61.3, 61.4] (total 122.7)  thin-island run 0 mm
 -x down: bed      0 mm2  overhang  12559 mm2  height 108.2  first layer 1 islands [1.6] (total 1.6)  thin-island run 1 mm
 +y down: bed      0 mm2  overhang   7704 mm2  height 175.0  first layer 1 islands [5.1] (total 5.1)  thin-island run 0 mm
 -y down: bed     44 mm2  overhang   7957 mm2  height 175.0  first layer 1 islands [65.7] (total 65.7)  thin-island run 0 mm
 +z down: bed      0 mm2  overhang  12097 mm2  height 108.3  first layer 1 islands [3.9] (total 3.9)  thin-island run 1 mm
```
### bay-30
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bay-30.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [240045.1], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 2, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bay-30.stl extents [ 94.9 145.5 174. ] g PETG 364.92 watertight True
   saved: bed      0 mm2  overhang  13192 mm2  height 174.0  first layer 1 islands [2.6] (total 2.6)  thin-island run 3 mm
 +x down: bed   3415 mm2  overhang  17116 mm2  height  94.9  first layer 1 islands [3635.3] (total 3635.3)  thin-island run 0 mm
 -x down: bed      0 mm2  overhang  21701 mm2  height  94.9  first layer 1 islands [2.2] (total 2.2)  thin-island run 2 mm
 +y down: bed     56 mm2  overhang  14328 mm2  height 145.5  first layer 2 islands [43.8, 44.1] (total 87.9)  thin-island run 0 mm
 -y down: bed      3 mm2  overhang  14658 mm2  height 145.5  first layer 1 islands [10.2] (total 10.2)  thin-island run 0 mm
 +z down: bed     11 mm2  overhang  12639 mm2  height 174.0  first layer 1 islands [21.4] (total 21.4)  thin-island run 0 mm
```
### foot_leg_1
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/foot_leg_1.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [12218.0], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 1, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/foot_leg_1.stl extents [27.8 31.5 45. ] g PETG 19.23 watertight True
   saved: bed    106 mm2  overhang    515 mm2  height  45.0  first layer 1 islands [148.7] (total 148.7)  thin-island run 0 mm
 +x down: bed    344 mm2  overhang   1113 mm2  height  27.8  first layer 2 islands [46.6, 320.7] (total 367.2)  thin-island run 0 mm
 -x down: bed      0 mm2  overhang   1434 mm2  height  27.8  first layer 1 islands [1.5] (total 1.5)  thin-island run 1 mm
 +y down: bed      2 mm2  overhang   1148 mm2  height  31.5  first layer 1 islands [10.4] (total 10.4)  thin-island run 0 mm
 -y down: bed     41 mm2  overhang   1133 mm2  height  31.5  first layer 1 islands [78.2] (total 78.2)  thin-island run 0 mm
 +z down: bed      0 mm2  overhang    680 mm2  height  45.0  first layer 1 islands [4.0] (total 4.0)  thin-island run 4 mm
```
### foot_leg_2
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/foot_leg_2.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [11290.5], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 1, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/foot_leg_2.stl extents [24.4 32.  45. ] g PETG 17.8 watertight True
   saved: bed    106 mm2  overhang    418 mm2  height  45.0  first layer 1 islands [148.9] (total 148.9)  thin-island run 0 mm
 +x down: bed    325 mm2  overhang    932 mm2  height  24.4  first layer 2 islands [42.9, 304.5] (total 347.4)  thin-island run 0 mm
 -x down: bed      0 mm2  overhang   1262 mm2  height  24.4  first layer 1 islands [3.8] (total 3.8)  thin-island run 1 mm
 +y down: bed     12 mm2  overhang   1044 mm2  height  32.0  first layer 1 islands [23.9] (total 23.9)  thin-island run 0 mm
 -y down: bed      4 mm2  overhang   1088 mm2  height  32.0  first layer 1 islands [26.7] (total 26.7)  thin-island run 0 mm
 +z down: bed      0 mm2  overhang    666 mm2  height  45.0  first layer 1 islands [2.7] (total 2.7)  thin-island run 5 mm
```
### bay-90
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bay-90.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [118609.2], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bay-90.stl extents [130.4 139.5  71.9] g PETG 182.51 watertight True
   saved: bed    108 mm2  overhang  11842 mm2  height  71.9  first layer 1 islands [148.9] (total 148.9)  thin-island run 0 mm
 +x down: bed      1 mm2  overhang   5149 mm2  height 130.4  first layer 1 islands [5.8] (total 5.8)  thin-island run 0 mm
 -x down: bed      0 mm2  overhang   5187 mm2  height 130.4  first layer 1 islands [2.1] (total 2.1)  thin-island run 0 mm
 +y down: bed     49 mm2  overhang   6002 mm2  height 139.5  first layer 2 islands [45.1, 45.2] (total 90.3)  thin-island run 0 mm
 -y down: bed      0 mm2  overhang   6169 mm2  height 139.5  first layer 2 islands [20.8, 20.9] (total 41.7)  thin-island run 0 mm
 +z down: bed      0 mm2  overhang  11737 mm2  height  71.9  first layer 1 islands [2.3] (total 2.3)  thin-island run 2 mm
```
### foot_leg_3
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/foot_leg_3.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [12003.0], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 1, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/foot_leg_3.stl extents [27.3 32.  45.5] g PETG 18.91 watertight True
   saved: bed    106 mm2  overhang    480 mm2  height  45.5  first layer 1 islands [148.7] (total 148.7)  thin-island run 0 mm
 +x down: bed      0 mm2  overhang   1398 mm2  height  27.3  first layer 1 islands [1.5] (total 1.5)  thin-island run 1 mm
 -x down: bed    335 mm2  overhang   1089 mm2  height  27.3  first layer 2 islands [45.5, 312.6] (total 358.2)  thin-island run 0 mm
 +y down: bed      5 mm2  overhang   1125 mm2  height  32.0  first layer 1 islands [14.6] (total 14.6)  thin-island run 0 mm
 -y down: bed      4 mm2  overhang   1172 mm2  height  32.0  first layer 1 islands [13.9] (total 13.9)  thin-island run 0 mm
 +z down: bed      0 mm2  overhang    703 mm2  height  45.5  first layer 1 islands [6.4] (total 6.4)  thin-island run 5 mm
```
### bay-150
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bay-150.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [181179.5], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 1, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bay-150.stl extents [ 94.7 145.5 172.5] g PETG 274.99 watertight True
   saved: bed      1 mm2  overhang  10868 mm2  height 172.5  first layer 1 islands [3.9] (total 3.9)  thin-island run 1 mm
 +x down: bed      0 mm2  overhang  16016 mm2  height  94.7  first layer 1 islands [1.5] (total 1.5)  thin-island run 1 mm
 -x down: bed   1738 mm2  overhang  13626 mm2  height  94.7  first layer 2 islands [61.3, 1801.8] (total 1863.1)  thin-island run 0 mm
 +y down: bed      4 mm2  overhang  10093 mm2  height 145.5  first layer 1 islands [10.4] (total 10.4)  thin-island run 0 mm
 -y down: bed      5 mm2  overhang   9982 mm2  height 145.5  first layer 2 islands [20.0, 20.3] (total 40.3)  thin-island run 0 mm
 +z down: bed     13 mm2  overhang  10672 mm2  height 172.5  first layer 1 islands [25.7] (total 25.7)  thin-island run 0 mm
```
### bay-210
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bay-210.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [120528.0], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bay-210.stl extents [105.9 169.5  93.4] g PETG 189.91 watertight True
   saved: bed      1 mm2  overhang  11134 mm2  height  93.4  first layer 1 islands [5.8] (total 5.8)  thin-island run 1 mm
 +x down: bed      0 mm2  overhang  12343 mm2  height 105.9  first layer 2 islands [2.1, 2.9] (total 5.1)  thin-island run 1 mm
 -x down: bed    101 mm2  overhang  11742 mm2  height 105.9  first layer 2 islands [61.1, 61.4] (total 122.5)  thin-island run 0 mm
 +y down: bed    234 mm2  overhang   6655 mm2  height 169.5  first layer 1 islands [283.8] (total 283.8)  thin-island run 0 mm
 -y down: bed     48 mm2  overhang   6839 mm2  height 169.5  first layer 1 islands [68.0] (total 68.0)  thin-island run 0 mm
 +z down: bed      0 mm2  overhang  11499 mm2  height  93.4  first layer 1 islands [1.8] (total 1.8)  thin-island run 2 mm
```
### link_1
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/link_1.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [52544.7], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/link_1.stl extents [62.5 68.  87. ] g PETG 80.29 watertight True
   saved: bed      0 mm2  overhang   2531 mm2  height  87.0  first layer 1 islands [2.8] (total 2.8)  thin-island run 1 mm
 +x down: bed    149 mm2  overhang   3773 mm2  height  62.5  first layer 4 islands [30.9, 33.9, 48.5, 117.8] (total 231.1)  thin-island run 0 mm
 -x down: bed      0 mm2  overhang   3646 mm2  height  62.5  first layer 1 islands [4.0] (total 4.0)  thin-island run 1 mm
 +y down: bed     44 mm2  overhang   3741 mm2  height  68.0  first layer 1 islands [66.9] (total 66.9)  thin-island run 0 mm
 -y down: bed     45 mm2  overhang   3676 mm2  height  68.0  first layer 1 islands [67.6] (total 67.6)  thin-island run 0 mm
 +z down: bed      6 mm2  overhang   2581 mm2  height  87.0  first layer 1 islands [25.5] (total 25.5)  thin-island run 0 mm
```
### link_2
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/link_2.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [41857.0], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/link_2.stl extents [73.5 68.  95. ] g PETG 65.43 watertight True
   saved: bed      0 mm2  overhang   3384 mm2  height  95.0  first layer 1 islands [2.4] (total 2.4)  thin-island run 0 mm
 +x down: bed      1 mm2  overhang   2974 mm2  height  73.5  first layer 1 islands [6.9] (total 6.9)  thin-island run 0 mm
 -x down: bed      0 mm2  overhang   2994 mm2  height  73.5  first layer 1 islands [20.0] (total 20.0)  thin-island run 0 mm
 +y down: bed     47 mm2  overhang   3435 mm2  height  68.0  first layer 1 islands [73.4] (total 73.4)  thin-island run 0 mm
 -y down: bed     89 mm2  overhang   3354 mm2  height  68.0  first layer 2 islands [66.8, 72.5] (total 139.3)  thin-island run 0 mm
 +z down: bed      0 mm2  overhang   3137 mm2  height  95.0  first layer 1 islands [4.9] (total 4.9)  thin-island run 1 mm
```
### link_3
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/link_3.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [30543.5], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/link_3.stl extents [ 69.5  54.  117.5] g PETG 49.0 watertight True
   saved: bed     15 mm2  overhang   2362 mm2  height 117.5  first layer 1 islands [49.0] (total 49.0)  thin-island run 0 mm
 +x down: bed      0 mm2  overhang   2885 mm2  height  69.5  first layer 1 islands [15.1] (total 15.1)  thin-island run 0 mm
 -x down: bed      0 mm2  overhang   2863 mm2  height  69.5  first layer 1 islands [4.8] (total 4.8)  thin-island run 1 mm
 +y down: bed     44 mm2  overhang   2764 mm2  height  54.0  first layer 1 islands [67.2] (total 67.2)  thin-island run 0 mm
 -y down: bed    526 mm2  overhang   2471 mm2  height  54.0  first layer 1 islands [602.2] (total 602.2)  thin-island run 0 mm
 +z down: bed     47 mm2  overhang   2175 mm2  height 117.5  first layer 1 islands [72.6] (total 72.6)  thin-island run 0 mm
```
### link_4
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/link_4.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [22770.1], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/link_4.stl extents [42.  54.  59.3] g PETG 36.26 watertight True
   saved: bed      0 mm2  overhang   1924 mm2  height  59.3  first layer 1 islands [2.0] (total 2.0)  thin-island run 2 mm
 +x down: bed      0 mm2  overhang   2037 mm2  height  42.0  first layer 1 islands [20.0] (total 20.0)  thin-island run 0 mm
 -x down: bed      0 mm2  overhang   1998 mm2  height  42.0  first layer 1 islands [6.9] (total 6.9)  thin-island run 0 mm
 +y down: bed     45 mm2  overhang   1687 mm2  height  54.0  first layer 1 islands [71.8] (total 71.8)  thin-island run 0 mm
 -y down: bed   1074 mm2  overhang    632 mm2  height  54.0  first layer 1 islands [1190.2] (total 1190.2)  thin-island run 0 mm
 +z down: bed      0 mm2  overhang   1921 mm2  height  59.3  first layer 1 islands [1.1] (total 1.1)  thin-island run 1 mm
```
### link_5
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/link_5.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [6675.7], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/link_5.stl extents [35.  15.  31.9] g PETG 11.51 watertight True
   saved: bed      0 mm2  overhang    605 mm2  height  31.9  first layer 1 islands [1.8] (total 1.8)  thin-island run 0 mm
 +x down: bed     54 mm2  overhang    491 mm2  height  35.0  first layer 1 islands [80.7] (total 80.7)  thin-island run 0 mm
 -x down: bed      0 mm2  overhang    549 mm2  height  35.0  first layer 1 islands [2.5] (total 2.5)  thin-island run 2 mm
 +y down: bed     67 mm2  overhang    809 mm2  height  15.0  first layer 2 islands [11.8, 100.5] (total 112.3)  thin-island run 0 mm
 -y down: bed    569 mm2  overhang    277 mm2  height  15.0  first layer 1 islands [664.8] (total 664.8)  thin-island run 0 mm
 +z down: bed    289 mm2  overhang    294 mm2  height  31.9  first layer 1 islands [354.4] (total 354.4)  thin-island run 0 mm
```
### mast_top
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/mast_top.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [92672.1], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 1, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/mast_top.stl extents [168.5  99.9  75. ] g PETG 141.63 watertight True
   saved: bed    138 mm2  overhang   6389 mm2  height  75.0  first layer 3 islands [66.9, 74.4, 74.4] (total 215.7)  thin-island run 0 mm
 +x down: bed      6 mm2  overhang   5015 mm2  height 168.5  first layer 1 islands [26.6] (total 26.6)  thin-island run 0 mm
 -x down: bed    246 mm2  overhang   4826 mm2  height 168.5  first layer 1 islands [297.3] (total 297.3)  thin-island run 0 mm
 +y down: bed    182 mm2  overhang   5679 mm2  height  99.9  first layer 4 islands [67.0, 72.4, 72.7, 74.5] (total 286.6)  thin-island run 0 mm
 -y down: bed     50 mm2  overhang   6277 mm2  height  99.9  first layer 1 islands [61.1] (total 61.1)  thin-island run 0 mm
 +z down: bed      1 mm2  overhang   7129 mm2  height  75.0  first layer 1 islands [3.4] (total 3.4)  thin-island run 1 mm
```
### mast_low
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/mast_low.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [167260.0], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/mast_low.stl extents [120.5  99.9 157. ] g PETG 244.63 watertight True
   saved: bed    106 mm2  overhang   7464 mm2  height 157.0  first layer 1 islands [149.6] (total 149.6)  thin-island run 0 mm
 +x down: bed      4 mm2  overhang   7705 mm2  height 120.5  first layer 1 islands [13.4] (total 13.4)  thin-island run 0 mm
 -x down: bed    263 mm2  overhang   6922 mm2  height 120.5  first layer 3 islands [43.8, 112.4, 188.0] (total 344.2)  thin-island run 0 mm
 +y down: bed     45 mm2  overhang  10648 mm2  height  99.9  first layer 1 islands [65.8] (total 65.8)  thin-island run 0 mm
 -y down: bed   1220 mm2  overhang   9459 mm2  height  99.9  first layer 1 islands [1322.2] (total 1322.2)  thin-island run 0 mm
 +z down: bed    189 mm2  overhang   7043 mm2  height 157.0  first layer 2 islands [131.5, 131.5] (total 263.0)  thin-island run 0 mm
```
### mast_mid
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/mast_mid.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [445348.0], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/mast_mid.stl extents [168.   85.4 170. ] g PETG 626.41 watertight True
   saved: bed   1584 mm2  overhang   9040 mm2  height 170.0  first layer 2 islands [444.3, 1334.3] (total 1778.6)  thin-island run 0 mm
 +x down: bed   1154 mm2  overhang   9014 mm2  height 168.0  first layer 2 islands [562.3, 758.5] (total 1320.8)  thin-island run 0 mm
 -x down: bed    208 mm2  overhang   9872 mm2  height 168.0  first layer 1 islands [257.2] (total 257.2)  thin-island run 0 mm
 +y down: bed      0 mm2  overhang  21721 mm2  height  85.4  first layer 1 islands [2.7] (total 2.7)  thin-island run 1 mm
 -y down: bed    259 mm2  overhang  23622 mm2  height  85.4  first layer 2 islands [130.4, 163.2] (total 293.6)  thin-island run 0 mm
 +z down: bed   2734 mm2  overhang   7248 mm2  height 170.0  first layer 1 islands [3026.6] (total 3026.6)  thin-island run 0 mm
```
### mast_leg
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/mast_leg.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [27790.6], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/mast_leg.stl extents [ 33.   48.5 108. ] g PETG 44.52 watertight True
   saved: bed      4 mm2  overhang   1617 mm2  height 108.0  first layer 1 islands [12.7] (total 12.7)  thin-island run 0 mm
 +x down: bed    238 mm2  overhang   2360 mm2  height  33.0  first layer 1 islands [302.8] (total 302.8)  thin-island run 0 mm
 -x down: bed      7 mm2  overhang   2584 mm2  height  33.0  first layer 1 islands [16.1] (total 16.1)  thin-island run 0 mm
 +y down: bed     89 mm2  overhang   2581 mm2  height  48.5  first layer 2 islands [66.7, 71.5] (total 138.2)  thin-island run 0 mm
 -y down: bed     37 mm2  overhang   2772 mm2  height  48.5  first layer 1 islands [70.4] (total 70.4)  thin-island run 0 mm
 +z down: bed     46 mm2  overhang   1535 mm2  height 108.0  first layer 1 islands [72.3] (total 72.3)  thin-island run 0 mm
```
### rearP_low
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/rearP_low.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [65403.3], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/rearP_low.stl extents [128.5  57.  101.9] g PETG 100.53 watertight True
   saved: bed      0 mm2  overhang   4578 mm2  height 101.9  first layer 1 islands [1.7] (total 1.7)  thin-island run 1 mm
 +x down: bed      2 mm2  overhang   3831 mm2  height 128.5  first layer 1 islands [7.5] (total 7.5)  thin-island run 0 mm
 -x down: bed     45 mm2  overhang   3665 mm2  height 128.5  first layer 1 islands [72.5] (total 72.5)  thin-island run 0 mm
 +y down: bed     92 mm2  overhang   5301 mm2  height  57.0  first layer 2 islands [66.2, 73.4] (total 139.6)  thin-island run 0 mm
 -y down: bed      0 mm2  overhang   5693 mm2  height  57.0  first layer 1 islands [8.9] (total 8.9)  thin-island run 0 mm
 +z down: bed     93 mm2  overhang   4256 mm2  height 101.9  first layer 2 islands [67.2, 70.0] (total 137.2)  thin-island run 0 mm
```
### rearP_up
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/rearP_up.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [32757.5], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/rearP_up.stl extents [ 77.9  50.  116.5] g PETG 52.77 watertight True
   saved: bed    221 mm2  overhang   1887 mm2  height 116.5  first layer 1 islands [277.8] (total 277.8)  thin-island run 0 mm
 +x down: bed      0 mm2  overhang   3538 mm2  height  77.9  first layer 1 islands [1.2] (total 1.2)  thin-island run 4 mm
 -x down: bed     46 mm2  overhang   3172 mm2  height  77.9  first layer 1 islands [72.0] (total 72.0)  thin-island run 0 mm
 +y down: bed     93 mm2  overhang   2749 mm2  height  50.0  first layer 2 islands [70.8, 73.3] (total 144.1)  thin-island run 0 mm
 -y down: bed      0 mm2  overhang   3030 mm2  height  50.0  first layer 1 islands [7.4] (total 7.4)  thin-island run 0 mm
 +z down: bed      0 mm2  overhang   2311 mm2  height 116.5  first layer 1 islands [10.4] (total 10.4)  thin-island run 0 mm
```
### rearP_up_b
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/rearP_up_b.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [29241.7], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/rearP_up_b.stl extents [46.9 75.9 62. ] g PETG 45.28 watertight True
   saved: bed     43 mm2  overhang   2357 mm2  height  62.0  first layer 1 islands [66.9] (total 66.9)  thin-island run 0 mm
 +x down: bed     49 mm2  overhang   2167 mm2  height  46.9  first layer 1 islands [57.5] (total 57.5)  thin-island run 0 mm
 -x down: bed     15 mm2  overhang   2216 mm2  height  46.9  first layer 2 islands [4.8, 27.5] (total 32.3)  thin-island run 0 mm
 +y down: bed      7 mm2  overhang   1847 mm2  height  75.9  first layer 1 islands [14.3] (total 14.3)  thin-island run 0 mm
 -y down: bed    464 mm2  overhang   1346 mm2  height  75.9  first layer 1 islands [508.2] (total 508.2)  thin-island run 0 mm
 +z down: bed      1 mm2  overhang   2405 mm2  height  62.0  first layer 1 islands [6.6] (total 6.6)  thin-island run 0 mm
```
### rearN_low
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/rearN_low.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [28117.9], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/rearN_low.stl extents [ 45.9  40.  112.5] g PETG 44.82 watertight True
   saved: bed      7 mm2  overhang   1441 mm2  height 112.5  first layer 1 islands [14.9] (total 14.9)  thin-island run 0 mm
 +x down: bed     79 mm2  overhang   2615 mm2  height  45.9  first layer 1 islands [109.5] (total 109.5)  thin-island run 0 mm
 -x down: bed      0 mm2  overhang   2714 mm2  height  45.9  first layer 1 islands [2.3] (total 2.3)  thin-island run 0 mm
 +y down: bed     93 mm2  overhang   2683 mm2  height  40.0  first layer 2 islands [69.7, 73.9] (total 143.6)  thin-island run 0 mm
 -y down: bed     40 mm2  overhang   2906 mm2  height  40.0  first layer 2 islands [12.2, 55.4] (total 67.6)  thin-island run 2 mm
 +z down: bed     47 mm2  overhang   1320 mm2  height 112.5  first layer 1 islands [73.9] (total 73.9)  thin-island run 0 mm
```
### rearN_up
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/rearN_up.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [100658.9], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/rearN_up.stl extents [139.9  99.9 125.5] g PETG 153.36 watertight True
   saved: bed    208 mm2  overhang   7272 mm2  height 125.5  first layer 1 islands [247.6] (total 247.6)  thin-island run 0 mm
 +x down: bed     43 mm2  overhang   6305 mm2  height 139.9  first layer 1 islands [66.3] (total 66.3)  thin-island run 0 mm
 -x down: bed     50 mm2  overhang   6265 mm2  height 139.9  first layer 1 islands [61.2] (total 61.2)  thin-island run 0 mm
 +y down: bed     88 mm2  overhang   7282 mm2  height  99.9  first layer 2 islands [66.8, 72.8] (total 139.6)  thin-island run 0 mm
 -y down: bed    199 mm2  overhang   7284 mm2  height  99.9  first layer 1 islands [224.8] (total 224.8)  thin-island run 0 mm
 +z down: bed      2 mm2  overhang   7466 mm2  height 125.5  first layer 1 islands [7.5] (total 7.5)  thin-island run 0 mm
```
### topP_1
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/topP_1.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [105263.9], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/topP_1.stl extents [139.5 160.   71.3] g PETG 157.61 watertight True
   saved: bed      0 mm2  overhang   8304 mm2  height  71.3  first layer 1 islands [1.3] (total 1.3)  thin-island run 1 mm
 +x down: bed      0 mm2  overhang   7033 mm2  height 139.5  first layer 1 islands [9.9] (total 9.9)  thin-island run 0 mm
 -x down: bed     46 mm2  overhang   7087 mm2  height 139.5  first layer 1 islands [74.0] (total 74.0)  thin-island run 0 mm
 +y down: bed     44 mm2  overhang   4662 mm2  height 160.0  first layer 1 islands [65.7] (total 65.7)  thin-island run 0 mm
 -y down: bed     87 mm2  overhang   4575 mm2  height 160.0  first layer 2 islands [65.5, 66.0] (total 131.5)  thin-island run 0 mm
 +z down: bed      1 mm2  overhang   8419 mm2  height  71.3  first layer 1 islands [5.1] (total 5.1)  thin-island run 0 mm
```
### topP_2
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/topP_2.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [152148.1], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/topP_2.stl extents [136.5 128.   57.5] g PETG 218.48 watertight True
   saved: bed      4 mm2  overhang   7719 mm2  height  57.5  first layer 2 islands [0.8, 10.7] (total 11.5)  thin-island run 0 mm
 +x down: bed      0 mm2  overhang   6465 mm2  height 136.5  first layer 1 islands [3.5] (total 3.5)  thin-island run 1 mm
 -x down: bed    502 mm2  overhang   5668 mm2  height 136.5  first layer 1 islands [585.4] (total 585.4)  thin-island run 0 mm
 +y down: bed     89 mm2  overhang   6594 mm2  height 128.0  first layer 2 islands [66.1, 67.7] (total 133.7)  thin-island run 0 mm
 -y down: bed    257 mm2  overhang   6716 mm2  height 128.0  first layer 1 islands [303.1] (total 303.1)  thin-island run 0 mm
 +z down: bed      4 mm2  overhang   7465 mm2  height  57.5  first layer 1 islands [12.6] (total 12.6)  thin-island run 0 mm
```
### topN_1
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/topN_1.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [131373.9], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/topN_1.stl extents [136.5 157.   75.9] g PETG 193.25 watertight True
   saved: bed      0 mm2  overhang   9153 mm2  height  75.9  first layer 1 islands [2.3] (total 2.3)  thin-island run 4 mm
 +x down: bed    131 mm2  overhang   7774 mm2  height 136.5  first layer 1 islands [171.0] (total 171.0)  thin-island run 0 mm
 -x down: bed      1 mm2  overhang   7964 mm2  height 136.5  first layer 1 islands [5.8] (total 5.8)  thin-island run 0 mm
 +y down: bed     46 mm2  overhang   5088 mm2  height 157.0  first layer 1 islands [66.7] (total 66.7)  thin-island run 0 mm
 -y down: bed      0 mm2  overhang   5073 mm2  height 157.0  first layer 1 islands [9.0] (total 9.0)  thin-island run 0 mm
 +z down: bed      1 mm2  overhang   9332 mm2  height  75.9  first layer 1 islands [5.1] (total 5.1)  thin-island run 0 mm
```
### topN_2
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/topN_2.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [159434.8], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/topN_2.stl extents [142.4 128.   65.5] g PETG 229.01 watertight True
   saved: bed      0 mm2  overhang   7634 mm2  height  65.5  first layer 1 islands [2.9] (total 2.9)  thin-island run 0 mm
 +x down: bed    502 mm2  overhang   6801 mm2  height 142.4  first layer 1 islands [585.4] (total 585.4)  thin-island run 0 mm
 -x down: bed      0 mm2  overhang   7498 mm2  height 142.4  first layer 1 islands [1.5] (total 1.5)  thin-island run 1 mm
 +y down: bed     89 mm2  overhang   6871 mm2  height 128.0  first layer 2 islands [67.0, 70.4] (total 137.4)  thin-island run 0 mm
 -y down: bed    368 mm2  overhang   6879 mm2  height 128.0  first layer 1 islands [427.5] (total 427.5)  thin-island run 0 mm
 +z down: bed      9 mm2  overhang   7355 mm2  height  65.5  first layer 1 islands [20.7] (total 20.7)  thin-island run 0 mm
```
### bay0_a
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bay0_a.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [79049.2], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bay0_a.stl extents [ 59.9 102.  124.5] g PETG 126.06 watertight True
   saved: bed      0 mm2  overhang   3985 mm2  height 124.5  first layer 1 islands [4.3] (total 4.3)  thin-island run 0 mm
 +x down: bed     51 mm2  overhang   9055 mm2  height  59.9  first layer 1 islands [61.8] (total 61.8)  thin-island run 0 mm
 -x down: bed      0 mm2  overhang   8844 mm2  height  59.9  first layer 1 islands [2.5] (total 2.5)  thin-island run 0 mm
 +y down: bed    519 mm2  overhang   3952 mm2  height 102.0  first layer 1 islands [671.6] (total 671.6)  thin-island run 0 mm
 -y down: bed     45 mm2  overhang   4241 mm2  height 102.0  first layer 1 islands [71.5] (total 71.5)  thin-island run 0 mm
 +z down: bed     21 mm2  overhang   3978 mm2  height 124.5  first layer 1 islands [44.9] (total 44.9)  thin-island run 0 mm
```
### bay0_b
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bay0_b.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [118172.6], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bay0_b.stl extents [ 45.9 124.5 128. ] g PETG 178.83 watertight True
   saved: bed      7 mm2  overhang   3925 mm2  height 128.0  first layer 2 islands [9.0, 13.7] (total 22.7)  thin-island run 0 mm
 +x down: bed    615 mm2  overhang  10968 mm2  height  45.9  first layer 1 islands [653.8] (total 653.8)  thin-island run 0 mm
 -x down: bed     33 mm2  overhang  11138 mm2  height  45.9  first layer 3 islands [0.9, 0.9, 58.0] (total 59.8)  thin-island run 0 mm
 +y down: bed     53 mm2  overhang   3883 mm2  height 124.5  first layer 3 islands [30.4, 30.7, 93.4] (total 154.5)  thin-island run 0 mm
 -y down: bed     95 mm2  overhang   3882 mm2  height 124.5  first layer 2 islands [74.2, 74.5] (total 148.7)  thin-island run 0 mm
 +z down: bed      3 mm2  overhang   3842 mm2  height 128.0  first layer 1 islands [11.5] (total 11.5)  thin-island run 1 mm
```
### bay-60_a
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bay-60_a.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [53704.5], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bay-60_a.stl extents [101.4  92.   69.4] g PETG 86.67 watertight True
   saved: bed     21 mm2  overhang   6802 mm2  height  69.4  first layer 2 islands [0.9, 44.2] (total 45.1)  thin-island run 0 mm
 +x down: bed      0 mm2  overhang   5481 mm2  height 101.4  first layer 1 islands [2.1] (total 2.1)  thin-island run 1 mm
 -x down: bed      0 mm2  overhang   5317 mm2  height 101.4  first layer 1 islands [3.5] (total 3.5)  thin-island run 1 mm
 +y down: bed    480 mm2  overhang   2280 mm2  height  92.0  first layer 2 islands [185.6, 414.4] (total 600.0)  thin-island run 0 mm
 -y down: bed     43 mm2  overhang   2638 mm2  height  92.0  first layer 1 islands [67.5] (total 67.5)  thin-island run 0 mm
 +z down: bed      0 mm2  overhang   6790 mm2  height  69.4  first layer 1 islands [1.5] (total 1.5)  thin-island run 1 mm
```
### bay-60_b
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bay-60_b.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [103930.9], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bay-60_b.stl extents [107.5 134.5  87.9] g PETG 161.02 watertight True
   saved: bed      0 mm2  overhang  11612 mm2  height  87.9  first layer 1 islands [1.3] (total 1.3)  thin-island run 1 mm
 +x down: bed      0 mm2  overhang   8731 mm2  height 107.5  first layer 1 islands [3.2] (total 3.2)  thin-island run 2 mm
 -x down: bed      3 mm2  overhang   8687 mm2  height 107.5  first layer 1 islands [12.7] (total 12.7)  thin-island run 0 mm
 +y down: bed    115 mm2  overhang   3400 mm2  height 134.5  first layer 3 islands [33.4, 35.1, 103.7] (total 172.3)  thin-island run 0 mm
 -y down: bed     90 mm2  overhang   3342 mm2  height 134.5  first layer 2 islands [67.8, 71.4] (total 139.2)  thin-island run 0 mm
 +z down: bed      1 mm2  overhang  11446 mm2  height  87.9  first layer 1 islands [10.6] (total 10.6)  thin-island run 0 mm
```
### bay-120_a
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bay-120_a.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [86332.0], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bay-120_a.stl extents [114.4  90.  115.9] g PETG 133.93 watertight True
   saved: bed    108 mm2  overhang   6963 mm2  height 115.9  first layer 1 islands [150.6] (total 150.6)  thin-island run 0 mm
 +x down: bed      2 mm2  overhang   6751 mm2  height 114.4  first layer 1 islands [8.6] (total 8.6)  thin-island run 0 mm
 -x down: bed      0 mm2  overhang   6916 mm2  height 114.4  first layer 1 islands [1.4] (total 1.4)  thin-island run 1 mm
 +y down: bed     46 mm2  overhang   5685 mm2  height  90.0  first layer 1 islands [72.4] (total 72.4)  thin-island run 0 mm
 -y down: bed     87 mm2  overhang   5701 mm2  height  90.0  first layer 2 islands [66.1, 66.9] (total 133.0)  thin-island run 0 mm
 +z down: bed      0 mm2  overhang   7077 mm2  height 115.9  first layer 1 islands [1.5] (total 1.5)  thin-island run 1 mm
```
### bay-120_b
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bay-120_b.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [118858.9], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bay-120_b.stl extents [105.4 150.5 115. ] g PETG 184.89 watertight True
   saved: bed    107 mm2  overhang  12394 mm2  height 115.0  first layer 1 islands [149.9] (total 149.9)  thin-island run 0 mm
 +x down: bed      6 mm2  overhang  10692 mm2  height 105.4  first layer 1 islands [20.1] (total 20.1)  thin-island run 0 mm
 -x down: bed      0 mm2  overhang  10993 mm2  height 105.4  first layer 1 islands [1.5] (total 1.5)  thin-island run 1 mm
 +y down: bed    105 mm2  overhang   4391 mm2  height 150.5  first layer 3 islands [32.2, 33.4, 101.7] (total 167.3)  thin-island run 0 mm
 -y down: bed     45 mm2  overhang   4386 mm2  height 150.5  first layer 1 islands [67.5] (total 67.5)  thin-island run 0 mm
 +z down: bed      1 mm2  overhang  12476 mm2  height 115.0  first layer 1 islands [4.3] (total 4.3)  thin-island run 1 mm
```
### bay-180_a
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bay-180_a.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [39839.0], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bay-180_a.stl extents [27.5 70.5 90. ] g PETG 63.06 watertight True
   saved: bed      0 mm2  overhang   1335 mm2  height  90.0  first layer 1 islands [5.9] (total 5.9)  thin-island run 0 mm
 +x down: bed      2 mm2  overhang   5002 mm2  height  27.5  first layer 1 islands [10.0] (total 10.0)  thin-island run 0 mm
 -x down: bed    139 mm2  overhang   4897 mm2  height  27.5  first layer 2 islands [90.6, 90.6] (total 181.2)  thin-island run 0 mm
 +y down: bed     47 mm2  overhang   1451 mm2  height  70.5  first layer 1 islands [72.9] (total 72.9)  thin-island run 0 mm
 -y down: bed     47 mm2  overhang   1422 mm2  height  70.5  first layer 2 islands [46.0, 46.0] (total 92.0)  thin-island run 0 mm
 +z down: bed     34 mm2  overhang   1265 mm2  height  90.0  first layer 2 islands [30.7, 78.2] (total 108.9)  thin-island run 0 mm
```
### bay-180_b
```
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bay-180_b.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [119809.6], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
/home/user/nox/.claude/worktrees/agent-a24c66574aeffb552/runs/box_system/v2/cut/pieces/bay-180_b.stl extents [ 40.8 142.5 128. ] g PETG 183.91 watertight True
   saved: bed      3 mm2  overhang   4137 mm2  height 128.0  first layer 1 islands [10.0] (total 10.0)  thin-island run 0 mm
 +x down: bed      0 mm2  overhang  12940 mm2  height  40.8  first layer 1 islands [3.1] (total 3.1)  thin-island run 1 mm
 -x down: bed      0 mm2  overhang  12999 mm2  height  40.8  first layer 1 islands [1.9] (total 1.9)  thin-island run 0 mm
 +y down: bed     53 mm2  overhang   3353 mm2  height 142.5  first layer 3 islands [30.4, 30.7, 93.4] (total 154.5)  thin-island run 0 mm
 -y down: bed     47 mm2  overhang   3223 mm2  height 142.5  first layer 1 islands [74.5] (total 74.5)  thin-island run 0 mm
 +z down: bed      0 mm2  overhang   4264 mm2  height 128.0  first layer 1 islands [22.4] (total 22.4)  thin-island run 0 mm
```
