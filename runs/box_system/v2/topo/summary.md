# Box structure v2: coarse system topology, round 1 (`nox/box/box_system_topo_v2*.py`)

Status: RUN RECORD, not a part. `result.json` "note" (run output) says four bays at 1500; `gen1500/analysis.txt` and this summary are the record (7 of 9 not joined). Nothing goes to print/. `iso05.stl` is a density surface; solid_check runs out of memory on it (29 GB, `solid_check_output.txt`).

Visual: https://claude.ai/artifact/1JYZSHTs8E6Y2jHTnyAZAs (source `topo_views.html`, `nox/box/box_system_topo_v2_page.py`).

## Setup
- h 8 (h 6 stalled on its first solve), 2h coarse stage first, min member 18, `finish="drawn"`, body ties off. 78 iterations, peak RSS 5.2 GB, wall 47 min (M).
- Layout: main at the Y_C 165 merge. Enclosure X ±206, Y −66..346, Z −236..234 (C); catch v7 plus copies at ±11.5 Y (C); stub plate Ø90 at Y −8..0 (C); pads 8 thick (C); **sheets drawn as 8 mm PETG (T = max(3, h), C) on a support that is the whole floor layer, which takes in their bottom edges**.
- 14 cases (loads from `dimensions.md` System box structure, E): standing, the Overseer catch on both diagonals, motors cut, XT60 60 N insert at the drone and at bays −90 / −120, 50 N pull, stub 57 N + 5.9 N·m, all bays at 60 N insert and at 43.5 N across, lifted 2 g, set down 5 g, knocked over 5 g.
- Not in the run: the final front at Y 335, the PR 86 carriage trim, the receiver bolts, the 74 N stall and the centring cases.

## Result (3000 cm³ target, 3025.7 cm³ generated, M; 3.84 kg PETG for the generated material alone, E; 10.4 kg with the 5196 cm³ drawn incl. the 8 mm sheets, E)
- 1500 cm³ (`gen1500/`, M): 22 components; only bays 30 and −210 join the main body (7 of 9 pads not in it), 2 of 8 seat posts (post4, post5) and the stub are off it; compliance 296..13526 x the full solid. 3000 cm³ (M): main body 8194.6 cm³ plus a 47.1 cm³ fragment (pad −60 touches both); the eroded design is 6 bodies, the main one 99.3 %. Every pad, post and the stub are on the main body. 1500..3000 is untested.
- Compliance / full solid (M on the h 8 model): standing 2.4, catch 6.5 / 16.3, motors cut 7.3, XT60 at the drone 10.0, bays −90 / −120 8.5 / 14.6, pull 9.7, all bays insert 18.0 / across 16.6, lift 1.8, set down 1.9, knocked over 14.3.
- Stub tilt 0.37 mrad at 57 N + 5.9 N·m (about 16 N·m/mrad against the 0.59 target); 0.71 at the drone insert; 1.4 / 2.8 mrad with all nine bays loaded (M on the h 8 model).
- Load path (M, `contacts.txt`, `profiles.txt`): a top frame under the funnel (Z 115..179) holds 1710 cm³, 57 % of the 3000 target, 45 % of the 3787 cm³ structure incl. drawn pads (front beam + 2 side rails joining the 8 posts; the +X rail 2.3 x the −X); a mast column (X −64..44, Y −56..−4, Z −113..163) flat on the rear sheet, up to the top frame, with a hub arch to the +X rail; deep end walls on the Overseer bays 0 and −180 onto the side sheets; a ring band tying bays only bay to bay, thinnest at −90. Contact: floor 12 cm² without the sheets (two feet near bay −90), against 221 / 191 / 164 cm² on the +X / −X / rear sheets; 208 cm² of floor if the sheets' edges count (`analysis.txt`).

## Verdict
**Not cut from yet.** The sheets carry the box in this model (rigid 8 mm PETG; real 3 mm acrylic is 16..19 x less stiff in bending, E: (8/3)³ less PETG 2.78 vs acrylic ~3 GPa), and the field leans on them. The cut proposal (`topo_views.html`, 13 piece types incl. an added datum ring for the ±0.15° bay datum) is kept as the round-1 reading. Round 2 reruns on the final layout (front Y 335, funnel v8 file, the PR 86 lanes, receiver bolts, 74 N stall, centring) with the sheets as clip-on cladding (non-structural, DEFAULT; alt 3 mm acrylic at its real stiffness) and the floor as the only support, before the cut requirements are written.

## Open
1. The sheets carry the box (round 2).
2. Overseer bay bodies 188 long (Y 71..259, M), over the 180 cube: diagonal print or split.
3. The ±Y seat landings overlap the grown funnel keep-out by 467 / 1365 mm³ each (M; posts drawn, they win); at h 8 the posts are 1..4 cells (M), so the clamp is not resolved.
