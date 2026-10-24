# Strength tests: turn the E numbers into P numbers

Most loads and allowables in `nox/dimensions.md` are E, and the as-built index that `nox/topo` reports has never been checked against a real print. These tests fix that. Spark only prints and pulls; everything else is ready.

- **Coupons:** `nox/tests/coupons.py` (drawn, manifold3d; `python3 nox/tests/coupons.py` rebuilds the files, the checks and the predictions). Runs: `runs/test_<name>/v1/` (socket: `v2`), each with `summary.md`, `result.json`, `preview.png`.
- **Predicted:** the post and layer coupons go through nox/topo's own as-built strength report (`topo.build.strength`, the voxel FE every part uses, grid h 1.0, PETG Xt 51 / Zt 25.5 / S 25.5 MPa E). Break load = the load at which the method's stress reaches the material strength (misuse index x 0.85 / 2.5 = 1). Glue, socket and XT60 are hand calcs or E numbers. All predictions live in `nox/dimensions.md` T.
- **Readings:** `python3 tools/calibrate.py <key> <N> <N> ...` (`--kg` if the scale reads kg). It prints mean, min, the factor = measured / predicted, the derived MPa, the row to paste over the `dimensions.md` T row and the `log.md` line. Use the min factor: a part's real index = its reported index / factor.
- **Where results go:** the T row (P), the P number in its own `dimensions.md` row (named per test below), one `log.md` line, the part doc if a part depends on it. When every strength test is in, the Tests row in `nox/current-parts.md` goes to TESTED.

## Order (by the E numbers each test replaces)
| # | Test | Replaces | Print |
|---|---|---|---|
| 1 | Post pull | the method factor on every as-built index (18 indices on READY rows in `current-parts.md`) and the attachment rule (index ≤ 0.5 at attachments, log 17 Oct) | plate 1 |
| 2 | XT60 mate and pull | insert 60 N, pull 50 N, release ~45 N and what follows from them in 9 `dimensions.md` rows (D17, G seated hold, I XT60 mate, bay module, J release and insert, L swap case, the arm v5 couple 6.39 N·m, gripper spring ≥ 15 N) | none |
| 3 | Layer pull | PETG Zt 25.5 and S 25.5 (both 0.5 Xt, E) in `nox/topo/material.py`, behind every across-layer index; splits the post-pull factor into material and grid error | plate 1 |
| 4 | Glue lap | the 2.0 MPa design bond (funnel glue lines read 5 to 7 on it, `box/box-funnel.md` 7) and the adhesive choice | plate 2 |
| 5 | Socket fit + epoxy pull | the socket bore (3.6 C, or 3.3 in the older row) and the rod pull-out (2.0 MPa bond E) | plate 1 |

**Print plates** (Bambu Studio, A1 mini, Bambu PETG Basic, 0.20 mm Standard @BBL A1M, textured PEI, every file imported as saved, largest flat face already down; tree(auto) 30° global finds nothing to support, all overhangs are 45°; bed contact is over 400 mm² on every file, so no brim):
- Plate 1: `print/test_post_pull_v1.stl` x 2 (copy A: 100 % rectilinear infill; copy B: object settings 4 walls, 15 % gyroid, the box-part default; write B on it after the print), `print/test_layer_pull_v1.stl` (100 %), `print/test_socket_coupon_v2.stl` (100 %).
- Plate 2: `print/test_glue_lap_v1.stl` x 12 (100 %).
- Record for every test: the profile as sliced (layer height, walls, infill), the filament and colour, the print date and the test date (PETG takes up water: test within a week, or say how long it sat).

**Buy / use:** a digital luggage scale with peak hold, 50 kg, 10 g steps (most read kg; use `--kg`); a scrap board ≥ 150 x 60 x 18 and 5 wood screws 4 to 5 mm x 25 with washers (or two C-clamps); locking pliers; a kitchen scale (XT60 insert); 120 grit paper and isopropyl alcohol; 2-part epoxy (the rod epoxy on hand; note brand and cure time); a methacrylate (MMA) structural adhesive, e.g. Loctite AA H8000, 3M Scotch-Weld DP8010 or Plexus MA300 (to buy); 3.0 carbon rod, 6 pieces 60 long; one new XT60 pair and a pack with its own plug.

## 1. Post pull (calibrates the as-built index for attachments)
**Print:** `print/test_post_pull_v1.stl`, 34.6 g PETG (M), 130 x 30 x 39, plate bottom down, two copies (A 100 %, B 4 walls 15 % gyroid). Two 6 x 6 posts and two 4 x 3 posts, printed upright like real attachment posts, an eye with a Ø8 hole on each top. The side pull bends the post, so it breaks across layers at the root.
**Predicted** (`dimensions.md` T): 6 x 6: 37.7 N (hook line 25 above the root); 4 x 3: 13.3 N (pulled across the 3, hook line 18). The beam hand calc gives 36.7 / 8.5 N; at h 1.0 the 4 x 3 post is 4 x 3 cells and its root corner is not resolved, so the two sizes split grid error from material error.
**Steps:**
1. Screw copy A to the board through the 5 holes (washers under the heads), board clamped to the bench, posts up.
2. Scale on peak hold, zeroed while hanging free. Hook through the eye of a 6 x 6 post.
3. Pull horizontally across the plate (along the 30 side, either way), scale level with the eye and square to the post.
4. Pull steadily, about 10 s from zero to the break, no jerk. Read the peak.
5. Note where it broke: the root layer (expected), higher up the post, the eye, or the plate.
6. Repeat on the second 6 x 6 and both 4 x 3 posts, then on copy B.
7. Photograph the fracture faces next to a ruler.
8. `python3 tools/calibrate.py post_pull 6x6 <N> <N> 4x3 <N> <N>`, then `... post_pull 6x6_B <N> <N> 4x3_B <N> <N>`.

**Record:** the peak per post (N or kg), where it broke, copy A or B, the profile. **Goes to:** T rows `post_pull_*` (factor P). If the min factor is under 0.8, the next design session brings it into nox/topo (material.py or the attachment rule) and says so in `log.md`.

## 2. XT60 mate and pull (D17)
**Print:** nothing. **Predicted:** insert 60 N, pull 50 N (E, `dimensions.md` I "XT60 mate"; the XT60L sheet says 1.8 to 4.5 kgf).
**Steps:**
1. Pull: hold the socket half by its body in a vise (soft jaws or a rag), or tie its leads to a fixed hook.
2. Tie a loop of strong string round the pin half's leads right behind the body; hook the scale in it.
3. Pull straight along the axis, steadily, until it comes apart. Read the peak. 5 times on the new pair.
4. Insert: stand the socket half on a kitchen scale (body upright, leads to the side), push the pin half straight in by hand until it seats. Film the display with a phone; the highest reading is the insert force (g x 0.00981 = N). 5 times.
5. Repeat 1 to 4 with a pack's own plug and its mating half.
6. Note which half (pins or sockets) the packs carry.

**Record:** 5 pull and 5 insert peaks per pair (new / pack). **Goes to:** `calibrate.py xt60_pull ...` and `xt60_insert ...` (T rows); the P values in `dimensions.md` I "XT60 mate" and D17 (the box swap session updates the insert / pull cases, the gripper spring and the pinion choice from them).

## 3. Layer pull (PETG across-layer strength Zt)
**Print:** `print/test_layer_pull_v1.stl`, 28.6 g PETG (M), 90 x 30 x 38, plate bottom down, 100 %. Three 3.6 x 3.6 x 8 necks printed upright with 45° flares and an eye on top: a straight pull is pure across-layer tension.
**Predicted:** 326.8 N (nox/topo FE h 1.0, peak at the gauge / flare junction; Zt x A = 330.5 N, A 12.96 mm²), about 33 kgf: use the 50 kg scale.
**Steps:**
1. Screw the plate to the board through the 4 holes; board on the floor.
2. Stand on the board either side of the coupon; hook through the eye of one neck.
3. Pull straight up, the scale vertical over the neck, steadily (about 10 s to the break). Read the peak.
4. Note where it broke: in the gauge (expected), at a flare, or in the eye.
5. Repeat on the other two necks.
6. `python3 tools/calibrate.py layer_pull <N> <N> <N>`: it also prints Zt = F / 12.96 MPa.

**Record:** 3 peaks, break place, profile (layer height matters most). **Goes to:** T row `layer_pull` (the measured Zt in MPa is in its Measured cell), and the nox/topo engine session sets `material.py` PETG Zt (and S, until a shear test) from it in its own PR, then reruns `python3 -m topo.test_topo`.

## 4. Glue lap (PETG bond: epoxy vs methacrylate)
**Print:** `print/test_glue_lap_v1.stl` x 12, 3.9 g each (M), 70 x 20 x 4, flat face down, 100 %. Each half ends in a 6 long half-thickness pad; turn one half over and butt each pad against the other's step: the overlap is exactly 10 x 6 = 60 mm² and the joint is flush, so the pull runs through the bond.
**Predicted:** 120 N per joint (2.0 MPa design bond x 60 mm², E).
**Steps:**
1. Sand both pad faces with 120 grit until matte, wipe with isopropyl alcohol, let dry 5 min.
2. Epoxy: mix, a thin film on both pads, mate (pad against step at both ends, edges in line), one binder clip, wipe the squeeze-out off the edges. 3 joints, mark E1 to E3.
3. Methacrylate: the same, 3 joints, mark M1 to M3. Both cure 24 h at room temperature.
4. Hook one eye on a fixed hook (a screw in the board, a door handle), the scale in the other eye.
5. Pull in line with the strip, steadily, about 10 s to the break. Read the peak.
6. Note the failure: adhesive (a clean PETG face), cohesive (glue left on both faces) or substrate (PETG tore).
7. `python3 tools/calibrate.py glue_epoxy <N> <N> <N>` and `glue_methacrylate <N> <N> <N>` (MPa printed).

**Record:** peak and failure mode per joint, glue brands and cure. **Goes to:** T rows `glue_*`; the winning bond (min MPa) replaces the 2.0 MPa design bond, and the funnel session re-reads its glue lines (`box/box-funnel.md` 7).

## 5. Socket fit and epoxy pull (3.0 rod)
**Print:** `print/test_socket_coupon_v2.stl` (copied unchanged from the Scout v2 file, solid_check PASS 24 Oct), 7.9 g PETG (M), 66 x 30 x 12, plate down, 100 %. Bores 3.5 / 3.6 / 3.7 (1 / 2 / 3 notches on the edge), each standing and lying, 10 deep.
**Predicted:** 177 N per rod (2.0 MPa x π x 3.0 x 9.4 on the rod face, E).
**Steps:**
1. Dry fit: push a 3.0 rod into each of the 6 bores by hand. Record which slide in fully without forcing and without wobble.
2. Cut 6 rods 60 long, sand the first 12 of each, wipe with alcohol.
3. Epoxy one rod into every bore (fill the bore, push the rod to the floor, wipe). Cure 24 h.
4. Clamp the plate flat to the bench (clamp on the plate, clear of the sockets).
5. Wrap the rod's free end in tape, grip it in locking pliers, hook the scale on the pliers.
6. Pull along the rod axis (up for standing, along the plate for lying), steadily. Read the peak; note rod pulled out clean, socket split, or socket torn off the plate.
7. `python3 tools/calibrate.py socket_pull <N> <N>` for the two sockets of the chosen bore; write the other four in the T notes line.

**Record:** the dry-fit result per bore, 6 peaks and failure modes. **Goes to:** T row `socket_pull` and the socket bore fit line; the bore (P) into `dimensions.md` G "Carbon rods" through the Scout v3 session.

## Not strength tests (stay in `current-parts.md` Spark, Tests)
Hook latch first fit (real drone seated, push up by hand); latch 50 cycles on the real cartridge; hover ulog over the box, vibration log, flight current log.
