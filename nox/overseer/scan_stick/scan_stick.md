# Scan stick (23 Oct 2026): the Overseer's sensors on a hand-held pole

The Overseer's lidar, camera, computer and flight pack on a paint-roller pole, so a facility can be scanned on foot
the way the drone will scan it: walk, press the button, the Pi records lidar + IMU + video, press again to stop.
Bought parts: `nox/parts-list.md` "Scan stick". Numbers: `nox/dimensions.md` section S (own geometry), M (L2,
camera), O (pack). State: `nox/current-parts.md` "Overseer". Source: `scan_stick.scad` (OpenSCAD, parametric; the
engine's Python/manifold parts live beside it: `head_topo_run.py` is the v2 topology run of the head).

```
   [Unitree L2]           lidar on the pole top, hemisphere FOV up, nothing above it
   ┌───────────┐
   │ head      │  plate on 4 bolt pads + 3/4-5 ACME female thread screwed onto the pole tip; camera block in front
   └──┬─────┬──┘
      │ cam │     camera bracket (DECXIN 2633V2 on 4 standoffs) hangs on the block, lens forward
      │pole │
   ┌──┴─────┴──┐
   │ cradle    │  split clamp on the pole's lower tube: Pi 5 tray (front), 4S 6000 pack on a ledge (back),
   │[Pi] (o) [B]│  UBEC + 12 V buck shelf (side), rocker switch, record button, LED
   └───────────┘
      hands
```

## Parts (printed, PETG)
| Part | File (`print/`) | What |
|---|---|---|
| head | `scan_stick_head_v2.stl` | topology part (`head_topo_run.py`, `runs/scan_stick/v2/summary.md`): generated body joining the 3/4-5 ACME pole boss, the 4 L2 bolt pads and the camera block (drawn features: section S) |
| thread test | `scan_stick_thread_test_v1.stl` | ring with the same thread: print first, screw onto the pole, tune `acme_clear` |
| cam | `scan_stick_cam_v1.stl` | plate with the camera's 28 x 28 M2 pattern on standoffs, 2 x M3 to the head's block |
| cradle | `scan_stick_cradle_v1.stl` | split clamp on the pole, Pi 5 tray, pack plate with ledge and straps, UBEC / buck shelf, switch / button / LED strip |

Why the non-mounting features exist (simplest design rule): the head has none left, v2 is generated between its
mounting features (the drawn v1 plate with centre hole and gussets is history, 86.8 g M); cradle ribs stiffen the 3.5 plates
that carry the ~590 g pack (Q9) and the Pi; ledge lip and side rails keep the pack from sliding out of the straps;
vent slots behind the Pi 5 are for its cooler; lightening slots on the pack plate are mass only (M); zip slots hold the
UBEC and buck, which have no holes (section M, Q4).

Alternative head, same script: `lidar_facing = "down"` hangs the L2 under a 115 mm arm, dome down (a belly-mounted
drone layout), camera flat under the arm looking down. Not saved and not gated (solid_check runs only on the up head;
the down head gets its own pass if it is ever printed). The Overseer's L2 mount is open (section O "at the nose, above
the collar faces", i.e. top-mounted), so the stick is up-facing, DEFAULT. `mount_style` "quarter20" / "clamp" are the
same script on a camera-stud or plain-tube pole; unused with this pole.

## Decisions (DEFAULT unless Spark says otherwise)
- **Pole:** EXTEND-A-REACH 1.5-3 ft telescoping extension pole, Amazon B08LL28XZW, universal 3/4"-5 ACME tip (the
  brand's whole line). The head screws on; no clamp at the top. Lower tube OD not published: `rod_d` 25.4 (E) until
  measured (P), then re-render the cradle (`render.sh <rod_d>`).
- **Lidar up-facing**, matching a top-mounted L2 on the Overseer. The hemisphere FOV must see the same side of the
  room as the drone's, otherwise the nightly diff flags occlusion differences as changes. Switch to `"down"` if the
  Overseer's mount goes under the belly.
- **Power:** 4S 6000 pack through a rocker switch to a 12 V 2 A buck (L2: 10 W, 13 W warm-up, spec) and a 5 V 5 A
  UBEC (Pi 5 + camera; section M). ~19 W total, ~3 h on the 6000 (E). Pole mass with everything: lidar 230 (spec) +
  pack ~590 (Q9) + Pi ~80 (E) + prints 298 (M) + pole (CHECK) ≈ 1.4 kg (E).
- **Data:** the L2 is Ethernet (lidar 192.168.1.62, host 192.168.1.2, UDP 6101 → 6201); the Pi records a ROS 2 bag
  of `unilidar/cloud` + `unilidar/imu` (or a raw pcap without ROS) and `rpicam-vid`-style video from the USB camera;
  one folder per session; button on GPIO17, LED on GPIO27. Files in `pi/` (`record.py`, `install.sh`, the systemd unit).
  Map with `point_lio_unilidar` or FAST-LIO2 (settings in `claude/research/2026-10-11-pi5-l2-lidar-slam.md`).
- **Screws:** M3 x 12 into the L2 (6 deep), M4 x 30 + nut clamp bolts, M3 heat-set + M3 x 6 for the camera bracket,
  M2 x 5 self-tap camera, M2.5 x 6 self-tap Pi.

## Print
PETG, 0.2 layers, 4 walls, 40 % gyroid (head) / 30 % (cradle, cam) (DEFAULT, E). Every file is saved on its largest
flat face: head = the 4 lidar pads down (1037 mm² bed, M), boss up, tree supports ~6 g (E) under the generated body
and the camera block (no thread entry chamfer: it left a 1.1 mm sliver on the first turn, solid_check thin wall; the
pole tip is chamfered); cam flat (2418 mm², M);
cradle on its bottom (4257 mm², M: the ledge + hub + plate feet; the two bigger faces carry standoffs and the ledge,
so they cannot be the bed face), tree supports under the horizontal Pi standoffs and the clamp bolt holes.

## Assembly
1. Thread test ring on the pole tip. Light drag by hand = right. Tight: `acme_clear` + 0.1 and reprint.
2. Head on the pole. Two heat-set inserts in the camera block. L2 on the plate, cable outlet into the -X notch,
   4 x M3 x 12 from below. Camera on its standoffs (lens forward), bracket on the block.
3. Cradle on the lower tube below the grip: Pi tray forward, clamp ears to the rear, 2 x M4. Pack on the ledge, two
   straps. UBEC + 12 V buck zip-tied to the shelf. Switch / button / LED in the strip above the Pi.
4. Ethernet L2 → Pi, L2 barrel → 12 V buck, camera USB → Pi, zip-tie the cable run along the pole.
5. `pi/install.sh` on the Pi (static 192.168.1.2 on eth0, recorder service). Wait ~10 s after power-up before
   recording (the L2 sends no points until warm, IMU init needs it still).

## Open (measure or ask Spark)
- Lower tube OD of the pole (P) → `rod_d`.
- Camera lens barrel Ø and board thickness (Q3) → standoff height if the lens needs more than 3 mm behind the plate.
- UBEC and 12 V buck bodies (Q4) → shelf slots are 52 x 60 with zip ties, fits 43-50 x 17-25 (E).
- Which way the Overseer's L2 faces, once its mount is designed → `lidar_facing`.
