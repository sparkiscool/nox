# Scout camera mount v1 (27 Oct 2026, drawn; `nox/scout/v3/camera_mount.py`)

Piece of **System Scout v3** (`dimensions.md` "System Scout v3": Camera (one), Strength, Rods; system model `nox/scout/v3/scout.assembly.json`, layout run `runs/scout_layout/v12/`, frame model `runs/scout_frame/v2/alt_sized/`).

The brief:
- Spark 27 Oct: the camera snap-clipped to the cage after it is built, no rod through it, the clips are the fuse.
- Spark 27 Oct, added: "crash protection for the lense sticking out, ... cool and spacex like cyberware, and construction".
- DEFAULT 27 Oct: both clip rods are 6/4 tube (rod 7-8 and rod 14-15 in the sized cage).

Visuals:
- Concept sheet (to-scale section, the two alternatives, breaking order, stills): https://claude.ai/artifact/FsimytGGoVftHsaL2bhAPM. Source `concept.html`, built by `nox/scout/v3/camera_mount_sheet.py`.
- Motion, the mount pushed onto the cage: https://claude.ai/artifact/82LPH8p6h9bXxAq6h3kYPE. Source `motion.html`, built by `nox/scout/v3/camera_mount_view.py`; also `motion.mp4` and `motion.gif`.
- Stills: `still_closeup.png`, `still_side.png`, `still_inside.png`, `motion.png`.

Previews: `preview_scout_camera_mount_v1.png` (print/scout_camera_mount_v1.stl), `preview_scout_camera_mount_collar_v1.png` (print/scout_camera_mount_collar_v1.stl).

## What it is
**One PETG print** (21.8 g, M), plus 2 stop collars (0.25 g each, M). Everything is drawn: planar facets and chamfers, with trusses where the load goes. Why drawn, not generated: Spark 27 Oct asked for a "spacex like cyberware, and construction" look (planar facets, sharp chamfers, visible trusses). The brief allowed topology only for the hidden web. A `topo` pass on that web is the v2 option.

- **Cradle:** a box frame round the DECXIN 2633V2, octagon-chamfered by 3 (C).
  - Envelope y ±22.5, z 17..67, x 128..153 (C).
  - 0.5 clear at the sides, lips 6 over and under; walls 3.0 (C).
  - The board sits on 4 M2.5 heat-set bosses on 28 x 28 (P). Each boss body is Ø9.2 with a wall of 3 round a Ø3.2 hole (C). Each carries a **Ø12 pad with a 2.0 web** at the seat face, x 132..134 (C, pad rule).
    - The pads overhang the 38 board's edge by 1.0 at most (C).
    - The board's back is not measured (Q3). A part there under 3.0 tall clears the X frame windows (C).
    - The board sits at 135 on 4 bought M2.5 nylon (or silicone) washers 1.0 thick (E). They are not printed: no TPU part under 2.0.
  - The back is an X frame through the bosses (border 4, bars 5, C); the cable leaves through its windows.
  - Each face is a chevron truss (ribs 3.5, longerons 4, C).
- **Lens hood:** the frame runs on to x 153, **4.0 ahead of the lens front** (C; gate 3, DEFAULT).
  - Front land 1.0, outer chamfer 1.2, inner flare 0.8 (C).
  - A flat frontal hit lands on the hood and goes through the cradle, trusses and clips into the two tubes. The lens never takes it.
- **Arms and knees:** flanged web trusses in the cheek plane (y ±19.5..22.5).
  - Web 3, window rib 3.5, chord flanges 8 x 3 inboard (C).
  - Top: from the clip's back flat to the back frame (x 131, z 44..66).
  - Bottom: a knee from the clip into the bottom wall (x 128..140).
- **Clips (4):** octagon C round the tube, axis along y. Each is 9 long, bore **6.2** (the coupon pick), finger wall 2.0, mouth 5.6 (snap 0.20 a side), lead-in 30° (C).
  - All 4 mouths face **into the cage (−x)**: the whole mount is pushed on from the front along −x after the cage is built, and all 4 snap at once.
  - Positions are the layout's: top (110.5, ±18.0, 61.1) on rod 7-8, bottom (122.6, ±18.0, 11.5) on rod 14-15 (C).
  - 3 grip ribs, 0.05 radial interference on the tube, at 0° and ±95° (E; set on a coupon). They stop rattle and slide.
  - The two rods together stop it turning.
  - The bore is a parameter: `SCM_ROD=3` gives 3.2 for a 3.0 rod if Spark reverts.
- **Stop collars (2):** the clip profile, 5 long, snapped onto rod 14-15 outboard of the bottom clips (0.3 gap, C), then a drop of CA (DEFAULT). They are the y end stop. They do not block the pop (a pop is radial).
- **Hardware (bought):** 4 heat-set inserts M2.5 (OD 3.5 x 5.7, E), 4 M2.5 x 8 screws through the board (C), 4 nylon or silicone M2.5 washers 1.0 thick (E).
- **Assembly:**
  1. Inserts into the bosses.
  2. Camera onto the bosses (screws from the front, through the open hood).
  3. Push the mount onto the two tubes from the front.
  4. Collars on, CA.

## Concept: two alternatives (weighed with the hood included)
- **A, one piece (picked, DEFAULT):**
  - 1 print and 2 collars, 22.3 g (M), no hardware beyond the camera screws; the fewest parts.
  - A centred frontal hit is carried in full: at 250 N, PETG 13.1 MPa and tubes at 0.30 of allowable (F).
  - Its fuse is soft. The clips let go at about the push-on force: 23 N a clip (E), 86 N pulled straight forward (F on E).
  - An off-centre bump on a hood corner pops the far-side clips at about 87 N (F on E).
  - Cage build tolerance: about ±0.3 at the snap (E).
- **B, cradle + 4 separate clips bolted on with M2.5:**
  - The mouths face up and down, so frontal hits never pull a clip; it pops only on vertical hits.
  - Clips snap on one at a time (a stiffer snap is possible), and a broken clip is a 1 g reprint.
  - Costs: 5 prints, 4 screws and 4 inserts, about +6 g (E), and about 6 mm of lug height above the cage roof (E).
- **Picked A:** simplest that meets every gate. Spark may ask for B if the camera popping off on bumps is a nuisance in flight tests. B is drawn only on the concept sheet.
- **Ruled out:**
  - One piece with up or down mouths: the bottom clips' push path hits rod 14-16 (C).
  - Rotate-on install: a frontal hit would then pop the bottom clips (C).

## Checks
- **Solid:** `python3 nox/solid_check.py print/scout_camera_mount_v1.stl`: **solid_check PASS** (1 body eroded, no necks). `print/scout_camera_mount_collar_v1.stl`: **solid_check PASS**. The assembled-pose meshes in this folder pass too.
- **View:** the layout's point test (`in_cam_view`, 89 x 73 + 2°):
  - 0 of 80k surface points in view (C).
  - Closest part 34.4° outside the field (C).
  - With the lens as a disk of r 6 (E, Q3): 0 in view, 29.9° clear (C).
  - Hood lead 4.0 (C).
- **Fit:**
  - Nothing inside the camera envelope (C).
  - Closest other rod (sized tubes): rod 14-16 at 6.58 (C).
  - Closest node: 16 at 5.45 (C).
  - Props 21.3 off (C).
  - Push-on stroke (40 mm along −x, C): every other rod at least 7.72 clear; the clip rods meet only the snap (0.23 of 0.25, C).
- **Snap** (curved-beam fingers, Castigliano, contact sweep; PETG E 2000, mu 0.25, E):
  - Finger stiffness 168.6 N/mm; push-on 23.4 N a clip, pop 23.4 N (E).
  - Finger 22.4 MPa at full opening (limit 30, E).
  - Grip 11.1 N slide a clip (E).
  - Picked from 81 rows: fingers ≥ 2.0 (no flexure exception), snap ≥ 0.20, push-on ≤ 25 N, strongest pop (`result.json` snap_scan).
- **Frame model** (`frame_model.py`):
  - Model: the v12 cage with the sized tubes (6/4 on the 39, real tube mass 0.024 g/mm, E). The mount's clip necks and truss chords are PETG beams. The cradle and camera are one rigid body: 31.8 g, CG (136.0, 0, 41.7), real inertia (M mesh + E camera).
  - **Lowest mode 109.3 Hz (F)**, gate 100: a cage pitch mode (tail point and nose); the camera has 23 % of its energy (F).
    - Stiffness changes barely move it: clips at E/10 give 109.2 Hz (F), chords at their out-of-plane I give 109.5 Hz (F).
    - The first mode the camera dominates is 426 Hz (F).
  - Frontal hit 250 N on the hood (35 g on 735 g, E):
    - Centred: PETG 13.1 MPa, clip pull 0, tubes at 0.30 of allowable (F).
    - On a hood corner: the far clips are pulled 67 N (F on E), so they pop (below).
  - 35 g inertial on the mount: clip pull at most 3.6 N against pop 23.4 (F on E).
  - Flight 4 g: 0.14 N (F).
- **Breaking order** (a load on the hood centre, the first thing to let go; F on E):

  | Load | First | Then |
  |---|---|---|
  | Pull forward | clips pop at 86 N | saddle ties at 843 N |
  | Push back (frontal) | saddle ties at 843 N (the cage's fuse); the clips bear on their closed backs and the mount holds | rod 7-8 at 907 N |
  | Side | a clip pops at 40 N | clips slide at 44 N, then the stop collars |
  | Up / down | clips pop at 92 N | saddle ties at 298 N |

  The clips are the fuse wherever a load pulls them out, and the cradle and hood never yield first (PETG yield 50 MPa, E).
- **Print:** `print_orient.py`, saved **+y face down** (the cheek plane: cheek, arm and knee webs, clip ends).
  - Bed 811 mm² (M), the largest of the 6; overhang 1467 mm² (M), the least.
  - Clip bores vertical, so the fingers bend within the layers.
  - 45.8 x 60.6 x 45.0 (M).
  - Collar: 39 mm² bed (M), so a brim.
- **System:** the v12 manifest with this mount in place of the layout's cradle, arms and clips (`scout_with_mount.assembly.json`, `system_check.txt`).
  - Limit: this check runs on the layout's cage as drawn, every rod 3.0 (rod r 1.5, shell tubes 67.5 g, E). The clips need the sized 6/4 tubes.
    - Those tubes add +74 g (E, frame model v2). That is not in the 735 g or the CG below.
    - The clips and collars carry no contact pair: the drawn 3.0 rods sit inside the 6.2 bores, and the fit check reads 0.0 mm³ for them.
  - Assembled result:
  - Cage first 6.2 mm (E) at +Z (camera); was 4.2.
  - Field cones open: camera 55°, **out 0 mm** (was 2: the lens no longer stands past the cage).
  - Shell 47 rods closed; fit 0.0 mm³ (E).
  - bbox 13.8 L (E), extents 390 x 278 x 127 mm (E); +4 in x for the hood.
  - Mass 735 of 500 g (E).
  - CG 1.9 mm off the thrust axis pack in (E, gate 3; was 0.9), 0.8 pack out (E).
  - Probes 7.2 mm (E); access 8/8 clear. PASS.

## Mass
- Mount 21.8 g + collars 0.5 g = 22.3 g (M). The layout's cradle, arms and clips were 16.5 g (E), so +5.8 g.
- The 5.8 g buy the lens hood (frontal hits off the lens), the truss arms (the solid gate's neck test fails Ø9 bars on 3 mm walls) and the Ø12 board pads.
- Camera + mount 32.3 g (M + E) against the layout's 26.5 slot (E). Weight is soft; nothing dropped below a target.

## Carbon
None added. The mount clips onto 2 cage members:
- rod 7-8, 6/4 tube, 193 long (C);
- rod 14-15, 6/4 tube, 124 long (C).

Both are in the frame model's sized 39 (`runs/scout_frame/v2/alt_sized/up6.json`). No printed struts beyond the mount itself.

## Requirements for others (done by the Scout v3 cage session, 27 Oct, in this PR)
- **Cage / frame model:** `cage_frame.py` now holds these clip points, the 32.3 g mount + camera and the real 6/4 tube mass: the sized cage's lowest mode is 111 Hz (F; `runs/scout_frame/v3/alt_sized/`), every rod 3.0 still fails (`runs/scout_frame/v3/`).
- **Node 16:** keep its sockets for 14-16 / 15-16 / 7-16 / 8-16 inside 5 of the node centre toward +x; the mount's back frame is 5.45 from a Ø12 node there (C).
- **Layout:** the camera slot is 32.3 g (M + E) in `dimensions.md` Mass; `nox/scout/v3/scout.assembly.json` carries this mount in place of the layout's cradle (CG 1.9 pack in, gate 3, E; no trim needed).
- **Cage sync (CLAUDE.md, review 15):** saved part reported to the Scout v3 cage session (session_01EtHMZNhc68Ghbc2KCNMUzU), file, commit and mass (21.8 g M). That session put it in `nox/scout/v3/scout.assembly.json` and republished the Scout artifact https://claude.ai/artifact/WHdjYkzKPifXuHT1dWeqoR with it (version 21, 27 Oct).

## Open
- **Snap fit (E):** print one mount and try it on a 6/4 offcut before the drone: snap 0.20 and grip 0.05 are estimates. Too tight: `SCM_SNAP=0.15`. Loose: raise GRIP.
- **Q3 (unmeasured):** the camera lens, board back and mass. The view check holds for a front lens element up to r 15 (C, 6.4° clear); at r 18 (C) the hood enters the field.
- **Stiffness design:** topology is not run. The arms are drawn trusses on the frame model's load path, drawn for the look Spark asked for (above). A `topo` pass on the hidden web is v2 if Spark wants it lighter.
- **Rods:** the 6.2 bore holds only while the 6/4 DEFAULT for these two tubes stands. If Spark reverts to 3.0, rerun with `SCM_ROD=3` (3.2 bore).
