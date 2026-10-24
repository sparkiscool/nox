# Scan stick (23 Oct 2026): the Overseer's sensors on a hand-held pole

The Overseer's lidar, camera, computer and flight pack on a paint-roller pole, so a facility can be scanned on foot
the way the drone will scan it: walk, press the button, the Pi records lidar + IMU + video, press again to stop.
Bought parts: `nox/parts-list.md` "Scan stick". Numbers: `nox/dimensions.md` section S (own geometry), M (L2,
camera), O (pack). State: `nox/current-parts.md` "Overseer". Scripts here: one `*_topo_*.py` per printed part (nox/topo
runs), `overnight.sh` (the run queue), `assembly_view.py` (the motion artifact), `scan_stick.scad` (the 3/4-5 ACME
thread: `thread_cut.scad` exports the thread body every run cuts; `thread_test` is the fit ring; `render.sh` exports both).

```
        [Unitree L2]          seat z 202 above the pole tip: hemisphere FOV up, nothing above it
       ┌───────────┐
       │  head     │  4 L2 pads + camera block, bolted to the base flange with 4x M4
       └─┬───────┬─┘
     cam │       │   camera bracket on the head's block, lens forward
         │ base  │   generated column, flange at the top
  [Pi 5] │       │ [4S 6000]   Pi tray on +X, pack on -X, UBEC / buck shelf on -Y, all near the hand
         └──┬─┬──┘
            │ │   3/4-5 ACME socket: the whole unit screws onto the pole tip (Spark 23 Oct: one bolted unit)
          ──┴─┴──  pole
```

## Parts (printed, PETG; every one a nox/topo part except the thread ring)
| Part | File (`print/`) | What |
|---|---|---|
| base | `scan_stick_base_v17.stl` | 3/4-5 ACME socket in a Ø40 (C) boss, drawn Ø36 (C) tube column (the cables run up its bore), top flange with 4x M4 heat-set inserts; Pi tray plate with Ø8 (C) standoffs + switch strip on +X (rocker, button and LED beside the tube), pack ledge / lip / rails / 5 mm (C) back plate with strap slots on -X, UBEC / buck shelf on -Y; the webs between them generated (`base_topo_v7.py` layout, `base_topo_v17.py` rebuild with the exact cuts) |
| head | `scan_stick_head_v13.stl` | bolt flange with a Ø10 (C) cable hole, 4 L2 pads with drawn gussets, 10.5 thick (C) camera block with its bracket holes at z 6 (C); four generated arms between (`head_topo_v8.py` layout, `head_topo_v13.py` rebuild) |
| cam | `scan_stick_cam_v7.stl` | 6 thick (C) plate layer, 2 bolt pads to the head's block, 4 Ø9 x 10 (C) standoffs with M2.5 inserts on the DECXIN 28 x 28 (P) pattern, drawn gussets; generated web in the plate layer (`cam_topo_v7.py`), 13.35 g (M) |
| thread test | `scan_stick_thread_test_v1.stl` | ring with the base's thread: print first, screw onto the pole, tune `acme_clear` |

Results, strength and print data per part: `runs/scan_stick_base/v17/summary.md`, `runs/scan_stick/v13/summary.md`,
`runs/scan_stick_cam/v7/summary.md`. Retired: head v1 (drawn), v2 (noxopt, no gussets), v3 (stopped at the layout change),
v4-v7 (gate FAILs), v8-v12 (passed; rebuilt after review: v10 a 3.0 mm (C) pad wall round the counterbores, v11 a 10 thick (C) camera
block for 7 deep (C) insert holes, v12 a 12 thick (C) block so the holes clear the M4 pockets plus Ø5 (C) key holes, v13 the holes
down at z 6 (C) because v12's -X hole ran into the 112.5° pad's driver clearance) (`runs/scan_stick/`),
the clamp cradle v1, cam v1-v6 (v4 had 4 mm posts, Spark 23 Oct: 10 mm; v5 web wrapped the posts; v6 stopped, its 4 mm plate layer under the robust plate), base v2-v16 (`runs/scan_stick_base/`; v8-v11 feather FAILs at the boss / tube step, fixed by
the Ø40 boss; v12 passed but the layout envelope clipped a Pi standoff and the UBEC keep-out cut the -Y rail, and the
standoffs had no drawn tray; v13 / v14 lost drawn bodies to the envelope at the build, lesson in `base_topo_v17.py`; v15
passed but its rocker and button cuts ran into the tube wall behind the strip, 0.7 mm (M) left; v16 passed but its button and LED
holes left 1.25 mm (M) of strip).
Every FAILED run keeps its folder and summary; only the READY files are in `print/`.

## Decisions (DEFAULT unless Spark says otherwise)
- **One unit on the pole tip** (Spark 23 Oct): the base's 3/4-5 ACME female socket is the only pole interface; no clamp
  on the tube, so the pole's lower tube OD no longer matters. Head and base bolt together (4x M4 x 12 (C) into heat-set
  inserts), the camera bracket bolts to the head (2x M3 x 12 (C) into inserts).
- **Pack and Pi low, lidar and camera high** (Spark 23 Oct): pack and Pi right above the thread, lidar seat 202 mm
  above the pole tip (C), camera on the head's block. The head bolts on turned -90° about z (the flange pattern is a square):
  the camera block and its bracket hang over the Pi tray side (+X), where nothing of the base rises above z 123 (C); over the
  pack side (±Y) the bracket would hit the 5 mm (C) back plate. The L2 cable notch then faces +Y (the rail side). `assembly_view.py`
  prints the overlap of every pair (0 mm³ = clear, M).
- **Pole:** EXTEND-A-REACH 1.5-3 ft telescoping extension pole, Amazon B08LL28XZW, universal 3/4"-5 ACME tip.
- **Lidar up-facing**, matching a top-mounted L2 on the Overseer (section O). The hemisphere FOV must see the same
  side of the room as the drone's, or the nightly diff flags occlusion differences as changes.
- **Loads (E):** 25 g knocks in 14 directions on every carried mass, 50 g impact in 6 directions (a 1 m drop onto the
  head, E), 100 N side hits and a 150 N drop on the lidar pads / the flange, hand grabs 20-50 N, strap tension 120 N.
  Alternative not taken: 25 g only (the drone's misuse set), which would be lighter.
- **Power:** 4S 6000 pack through a rocker switch to a 12 V 2 A buck (L2: 10 W, 13 W warm-up, spec) and a 5 V 5 A
  UBEC (Pi 5 + camera; section M). ~19 W total, ~3 h on the 6000 (E).
- **Data:** the L2 is Ethernet (lidar 192.168.1.62, host 192.168.1.2, UDP 6101 → 6201); the Pi records a ROS 2 bag
  of `unilidar/cloud` + `unilidar/imu` (or a raw pcap without ROS) and video from the USB camera; one folder per
  session; button on GPIO17, LED on GPIO27. Files in `pi/` (`record.py`, `install.sh`, the systemd unit). Map with
  `point_lio_unilidar` or FAST-LIO2 (settings in `claude/research/2026-10-11-pi5-l2-lidar-slam.md`).

## Print
PETG, 0.2 layers (C), 4 walls, 30 % gyroid (DEFAULT, E), tree supports, every file saved on its largest flat face: head on
its flange, cam on its plate; the base on its bottom (thread boss ring + ledge foot, 97 x 88 footprint, 168 tall, M: fits the
A1 mini 180, spec) although the flange top reads 8 % more contact (M): a 168 tall column standing on a Ø40 (C) flange would tip, and
the insert holes print cleaner facing up. Support grams are
in each summary.

## Assembly
1. Thread test ring on the pole tip. Light drag by hand = right. Tight: `acme_clear` + 0.1, re-export the thread
   (`thread_cut.scad`) and rerun the base.
2. Heat-set inserts: 4x M4 (Ruthex 8.1 long, spec) in the base flange, 2x M3 (Ruthex 5.7, spec) in the head's camera block, 4x M2.5 (E) in the bracket's standoffs from the top.
3. Base onto the pole. Pack on the ledge (lip at the bottom, rails each side), two straps through the slots in the rails and the back plate. Pi on its
   4 standoffs (M2.5 x 6 self-tap, C), ports up. UBEC + 12 V buck taped to the shelf side (Spark 23 Oct). Rocker at the +Y end of the strip, button and LED at the -Y end (C).
4. Head onto the flange turned so its camera block sits over the Pi (+X) and the cable notch faces +Y; 4x M4 x 12 (C): each screw goes sideways into its pocket between the arms (18 mm of room, M, before the L2 goes on) and is tightened with a 3 mm (C) key through the Ø5 (C) hole above the pocket. L2 on the pads, cable
   outlet over the notch (+Y once the head is turned), 4x M3 x 8 (C) from below (driver clearance is cut under each pad). Camera on its 4 standoffs
   (M2.5 x 6 into the inserts, C; the plug and parts on the board's back sit in the 10 mm, C), bracket on the block with its tab holes at the block's bottom (z 6, C), the plate hanging down beside the base (2x M3 x 12, C).
5. Ethernet L2 → Pi, L2 barrel → 12 V buck, camera USB → Pi: cables through the head's flange hole, down the tube bore and out of the 10 x 30 (C) window at z 40-70 (C) on -Y (the shelf side).
6. `pi/install.sh` on the Pi. Wait ~10 s (E) after power-up before recording (the L2 sends no points until warm).

## Open (measure or ask Spark)
- Camera lens barrel Ø and board thickness (Q3) → post height if the lens needs more than 3 mm (C).
- UBEC and 12 V buck: taped to the shelf side (Spark 23 Oct, Q4 closed); the bay on the shelf's -Y face is 28 wide (x ±14, C) x 30 deep x 64 tall if ties are wanted.
- Pack mass (Q9): 0.59 kg (E) in every load case.
