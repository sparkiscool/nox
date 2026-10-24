# Box structure v2: coarse system topology, round 2 (`nox/box/box_system_topo_v2r2*.py`)

Status: RUN RECORD, not a part. Nothing goes to print/. `iso05.stl` is a density surface (viewing mesh); solid_check is not run on it (round 1: 29 GB out of memory), so nothing here is called BUILT or READY.
Visual: https://claude.ai/artifact/NsZR5e2h5UQPktp3rb7wRH (`topo_views.html`, `nox/box/box_system_topo_v2r2_page.py`). Tags as CLAUDE.md: C CAD (layout and drawn inputs), M measured on the mesh or field (h 8 model), E estimate. Every number below carries one.

## Setup (what changed from round 1)
- Sheets are cladding: not drawn, not held. Support = the floor layer Z -236..-228 anywhere in the footprint (fixed, bilateral); the knocked-over case lies on the +X face layer. The structure stands on its own. The 3 mm acrylic comparison (modulus 3.2 GPa, E) was not run (open item 3).
- Final layout from main (front Y 335, enclosure X +-206, Z -236..234, C). Catch = `catch_W.stl` as it is (funnel v8 plain, no +-11.5 copies).
- Keep-outs (grown 1 mm): receivers, cartridge paths, drone, catch; arm hub / motors / parked heads and both drone insert lanes (`drone_30`, `drone_31`) from the layout meshes; rail disc; each bay envelope swept +Y (service door); electronics: 12 boards (`runs/box_electronics/v1/result.json` placement as of PR 95, C on E sizes; Y -66 + 10 standoff + depth + 6, C), PSU X 42..157 Y -66..-36 Z -170..45 (C), charger X -162..-44 Y -66..-7 Z -200..-29 (C), IEC inlet and E-stop (C).
- **Lanes used: the layout files on main when the run started (carriage Y 53.75..73.75, grabber r >= 111.5).** The heads v7 change (carriage Y to 75.35, Z >= 89; head + grabber r 95..159.6) came after. Checked on the finished fields (`box_system_topo_v2r2_newlanes.py`): material inside the new carriage / head lanes (above the axle, Z > 0) 0.0 / 0.0 cm3 (M) in all four runs (cell centres also checked against main's `drone_30.stl` / `drone_31.stl`). The no-ring result stands against the new lanes without a rerun. The catch is funnel v8 plain; main now has panel v9 plain: not rerun (open item 2).
- 18 cases: round 1's 14 (lifted case now supported on the floor at 2 g: the sheets no longer carry it) + head stall 74 N (E) at the drone insert and at the -90 bay, centring 7.6 N + 1.05 N.m (E) at the -90 and -120 bays (tangential force, moment about the axle, DEFAULT reading), reacted at the stub face. Receiver loads and cartridge weights go to the M3 bolt bosses (Scout 4 at z_c 0.85 / 85.8, x +-25.6; Overseer 6 at z_c 0.85 / 85.5 / 163.8, x +-26.8): drawn bosses O12, wall 3.8 (C). Stub case 57 N + 5.9 N.m (E, arm) kept.
- Electronics weight (2000 g, E) goes to 8 drawn O14 bosses on the rear face just outside the PSU and charger corners (DEFAULT).
- Seat landings: the 8 O16 x 8 drawn discs at the `layout.json` landings. Datum ring variant: r 115..200, Y 74..98, -210..30 deg (C), minus the keep-outs, 953 cm3 drawn (M; the drawn total grows 860 cm3, M, = 1.09 kg at 1.27 g/cm3, E).
- h 8 (C), 2h coarse stage, min member 18 (C), `finish="drawn"`, ties off. Four runs: 2000, 3000, 3000 + ring, 4000 cm3 generated. Peak RSS 4.7..4.9 GB each (M, `ru_maxrss`), two at a time; wall 45..50 min each.

## Result (3000 cm3 chosen, no ring)
- **Generated material only: 3059 cm3 (M) = 3886 g PETG at 1.27 g/cm3 (E).** Drawn pads, bosses, posts, stub, rear bosses 814 cm3 (M); total incl. drawn 4919 g (E).
- Connectivity (generated from `connectivity.json`): 9 of 9 bays, 8 of 8 seat posts, stub joined, 8 of 8 electronics bosses joined to the floor-supported body (M; boss 1 is a separate 25 cm3 island that also touches the floor); floating none. All 9 bays also join the stub body without passing the floor layer: the field builds its own bay-to-mast paths.
- **Contact areas (M, main-body cells in the support layer x h² = 0.64 cm2 each): floor 9.0 cm2 in the Z < -228 layer (14 cells, 7 patches of 1..2 cm2; 9.6 cm2 counting any material); +X face 57.6 cm2 (knocked-over support only).** The floor patches belong to the bay -60 / -120 bodies (X ±192..200), the bay -120 body (X -72..-64, Y 74..90), the bay -90 body (X 32..48, Y 154..162) and the mast (X -48..-32, Y -62..-46) (M, `analysis.txt`).
- Compliance / full solid (M, h 8): standing 2.9; Overseer catch 22.8 / 24.1; motors cut 27.0 (worst); XT60 at the drone 8.3; stall 74 N at the drone 9.5, at the -90 bay 12.0; XT60 at -90 / -120 9.9 / 10.7; pull 7.3; centring 3.6 / 2.4; stub case 5.9; all bays insert 18.9 / across 14.5; lifted 2.1; set down 2.1; knocked over 13.1.
- **Stub tilt 0.41 mrad at 57 N + 5.9 N.m (M on the h 8 model)** against <= 10 mrad (C, arm): k about 14.5 N.m/mrad against the 0.59 target (C, arm). Largest stub rotation in any case 2.28 mrad (all bays across).
- **Bay rotation about the axle, relative to the stub (M on the h 8 model): all bays 60 N insert max 1.8 mrad (0.10 deg), 43.5 N across 1.8 mrad.** The +-0.15 deg datum is 2.6 mrad (C). With the ring: 0.76 / 1.30. At 4000: 0.69 / 0.88.
- Load path (M): top frame Z > 110 holds 799 cm3; bay-to-bay links r 150..200 (44..128 cm3 per 15 deg wedge); bay bodies 130..280 cm3 each; a rear frame Y < 8 (about 1075 cm3): the mast X -72..72 rides over the board stack at Y -25..10 (area up to 100 cm2 at X -32), 664 cm3 at Y < -6 goes round the boards to the PSU / charger corner bosses. It reaches the floor through 7 small patches (X -200..-192 / Y 186..202; -72..-64 / 74..90; -48..-32 / -62..-46; 32..48 / 154..162; 192..200 / 106..122 and 194..210): TPU feet in the cut.

## Volume sweep
| Run | Generated cm3 (M) | Generated g (E) | Joined bays / posts / e-bosses | Floor / +X cm2 (M) | Worst compliance / solid (M) | Stub tilt mrad (M) | Worst bay rot. rel. stub, all-bays (M) |
|---|---|---|---|---|---|---|---|
| 2000 | 2048 | 2601 | 9/9, 8/8, 4/8 | 5.1 / 6.4 | 1180 | 0.71 | 169 / 1676 mrad |
| **3000** | 3059 | 3886 | 9/9, 8/8, 8/8 | 9.0 / 57.6 | 27.0 | 0.41 | 1.8 / 1.8 mrad |
| 3000 + ring | 3017 | 3831 | 9/9, 8/8, 8/8 | 8.3 / 10.9 | 42.7 | 0.62 | 0.76 / 1.30 mrad |
| 4000 | 4005 | 5086 | 9/9, 8/8, 8/8 | 9.6 / 89.6 | 14.7 | 0.22 | 0.69 / 0.88 mrad |

2000 reads joined at density 0.5 but its compliance (150..1180 x solid, M) says the links are grey and unusable. 3000 chosen: meets every target with margin; 4000 is about 2 x stiffer for 1.2 kg (E) more.

## Datum ring
The field builds its own bay-to-mast paths in both runs. The drawn ring (953 cm3 drawn, the drawn total +860 cm3 = +1.09 kg, M / E) gives 1.3 vs 1.8 mrad worst bay rotation and raises most compliances (worst 42.7 vs 27.0, M). Not added (DEFAULT).

## Cut proposal (rule-based reading of the 3000 field, `cuts.txt`; envelopes M on the h 8 field; mass E; 180 cube)
| Piece | Qty | Envelope X / Y / Z (W mm) | Joint | Mass E |
|---|---|---|---|---|
| Mast + stub face | 3 (split in Z) | -72..72 / -70..10 / -233..127 | laps at Z cuts | 0.93 kg total |
| Rear frame +X, -X | 4 | 72..208, -192..-72 / -70..10 / -233..127 | laps; 8 electronics bosses | 0.43 kg total |
| Top frame halves | 4 (split Y 138) | 0..136, -128..0 / 10..266 / 111..175 | laps; 8 seat posts | 0.84 kg total |
| Seat-post bridge | 1 | -56..72 / 266..290 / 111..175 | bolted to the top frame | 0.10 kg |
| Scout bay bodies (30, -30, -90, -150, -210) | 5 | longest 136..160 | 4 M3 inserts | 0.17..0.36 kg each |
| Overseer bay bodies (0, -60, -120, -180) | 4 (8 halves) | Y 58..258 = 200 long, over the 180 cube (-180: 184) | 6 M3 inserts; split Y 158 or diagonal | 0.21..0.31 kg each |
| Hub links | 6 | -184..200 / 10..218 / -153..111 | to mast and bay bodies | 0.37 kg total |
| Base feet | 3 | -96..96 / 10..58 / -177..-153 | a low tie; the floor contact itself is on the bay -60 / -120 / -90 bodies and the mast (above), TPU pads there | 0.02 kg |
Total about 4.9 kg (E).

## Verdict
The structure stands on its own: 18 cases carried, 9/9 bays and 8/8 posts joined, stub tilt 0.41 mrad (M). Not cut from yet: bilateral floor (uplift not checked), lanes pre-heads v7 (0 overlap after), v9 catch and acrylic comparison open.

## Open
1. Overseer bay bodies 200 long: split or diagonal.
2. Rerun on funnel panel v9 plain and heads v7 lanes before the cut requirements are written.
3. Acrylic-sheet comparison (3 mm, modulus 3.2 GPa, E) not run.
4. Floor contact is bilateral; a unilateral check of the feet belongs in the cut.
5. Seat landings overlap the grown funnel keep-out (439 / 151 mm3, M; drawn wins); clamp not resolved at h 8.
6. The rear frame (1.4 kg E) is tied to the electronics layout.
