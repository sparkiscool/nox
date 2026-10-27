# Scout v3 electronics in the pointed tail: concept (27 Oct 2026)

> **Status 27 Oct: concept v1 FAILED review 3 times in a row. It goes back to the system pass (v2, three strikes rule); do not build from this run.**
> Round 3 found:
> - The root joint does not fit: the Ø10 bosses reach |y| 8.5 and the plate, and the low bolt has no body behind it.
> - The CG here is ~1.5 off: `mass_cent` used bounding-box centres for 4 non-watertight meshes. With area centroids, A reads 4.65 / 2.35 pack in / out and needs the catch set 7.4 forward; B 5.35 / 3.34, 9.2; the cage's own tail bay −2.00 / +1.71.
> - Every placement here was hand-placed, not optimised.
> What stands: the tail volume, wash and cage-first findings, and the access method.

Spark, 27 Oct: "check the new scout cage design it should have a pointy tail and you need to fit the stuff there". Before that, the same day: the electronics list is confirmed, no lights, and the battery side is left as it is (`nox/scout/v3/README.md` Electronics mount).

**Scripts**
- `nox/scout/v3/electronics_tail.py`: layout, checks, `result.json`, `preview.png`, `run.log`.
- `nox/scout/v3/electronics_tail_syscheck.py`: `system_check` cage first, written to `system_check.txt`.

**Numbers:** `nox/dimensions.md` System Scout v3, row "Electronics tail bay".

**Context:** the race-car cage (Scout v3 design session, artifact WHdjYkzKPifXuHT1dWeqoR v5). Its meshes and rods are saved in `ctx/`. Everything here is on that geometry, so even C numbers carry its E until its run is on main.

**Visual:** https://claude.ai/artifact/EiZ5Z7WktXkSQePPHYEPiU (source `preview.png`): top row A, bottom row B, side and top views each.

**Tags:** C = computed by the scripts on the ctx geometry; E = estimate; M = bought-part data (`dimensions.md` M); P = measured. Board data for the Radxa is C from Radxa's STEP (`dimensions.md` H).

Review rounds: 27 Oct round 1 (cantilever, access, compactness, margins, tags) and round 2 (the fin swung about the tie line, the root tab under the attachment rule, crash order, access to outside the shell, stale numbers). Both are answered below.

## What goes in the tail
| Moves aft | Stays |
|---|---|
| Radxa Zero 3W, 5 V UBEC, XR2 | FC on the plate centre (IMU, motor leads), flow under the belly (its own view, PX4 offset), camera at the nose, the whole battery side |

## Concept A, picked: tail fin bolted to the belly mount, clipped to the tail core tie
**Fin:** one PETG fin on the plane y +5, web 2.4 (C). Front edge x −62, back to x −140 (C).
- Bottom edge z −26 (C).
- Top edge rises from z 18.5 at x −66 under the tie to the clip, then falls to z 34 at x −140 (C).
- Fin + clip span z −26..54.3 (C).

**Root (round 2):** a flange across the fin's front, x −65..−62, y −6.5..6.5, z −4..18.5 (C).
- It bolts with 2 M3, one high and one low and across: (y −3.5, z 14) and (y +3.5, z −1) (C).
- They go into heat-set inserts in Ø10 bosses on a tail block printed on the belly mount (C):
  - x −62..−56, z 10.6..18.5 over the receiver;
  - x −62..−59.9, z −4..10.6 behind it.
- Roll, pitch and yaw all go into the belly mount, which is the piece this session owns.
- |y| ≤ 6.5 keeps the block and flange 3.2 inside the rear disks' wash edge (C).
- Load (E, 35 g sideways on 33 g of fin and electronics): 11.3 N, about 0.43 N·m about z at the flange. That is about 61 N tension in one bolt, against an M3 insert pull-out of several hundred N (E). The frame model sizes it.

**Clip, the stiffener and the fuse:** a C-clip round the 3.0 tail core tie at x −100..−112 (C).
- 3.3 bore and a 3.0 wall (boss rule); its mouth is 2.6 wide, so it snaps on with 0.4 interference.
- It stiffens the fin's aft end in normal flight.
- In a tail hit the tie flexes first and the clip pops off at about 20 N (E), before the root or the fin yields. The root alone carries the 35 g case.
- Order (rule 7): cage node / tie, then the clip lets go (it snaps back on), and nothing else.

**What the fin carries**
- **Radxa:** on edge on the −y face, on 3.0 standoffs (clear of the SD slot, 2.86 under the PCB, C from the STEP).
  - Component side out, ports edge up, SD end aft.
  - x −132..−67, z −24..6 (C).
  - Standing vertical, it cools in the forward-flight air. It is out of the pack's shadow, so the onboard wifi radiates sideways.
- **UBEC:** worst-size envelope 50 x 25 x 10 (M) on the +y face, x −138..−88, z −12..13 (C).
- **XR2:** on the +y face at the top aft corner, x −132..−120, z 16..32 (C). Antenna up to z 46 (E), into free air behind the props.

**Carbon:** no new rods. The fin, flange, clip and tail block are printed PETG. The clip holds the existing tail core tie (the cage session's rod).

## Checks (C on the ctx geometry)
| Check | Radxa | UBEC | XR2 | Fin + clip | Tail block |
|---|---|---|---|---|---|
| Rod surface (≥ 3) | 9.3 | 12.1 | 10.9 | 3.3 (the tie it clips excepted) | 38.4 |
| Inside the shell | yes | yes | yes | yes | yes |
| `system_check` flat walls, worst direction (≥ 3) | 15.4 | 13.9 | 19.3 | | |
| Out of the rear prop wash (disk R 44.5, ≥ 3) | 7.3 | 6.4 | 33.6 | 3.8 | 3.2 |
| Nearest drone part | 7.0 receiver end pieces | 28.0 | 62.5 | 2.0 receiver end pieces | 0.2 (it is the belly mount's own block) |

- `system_check` edge probes: worst 6.7 (UBEC) (C).
- **Access** (C; plug, tool and pull sizes E):
  - FC USB-C plug + cable (12 x 7, 250 long): clear by 6.9, from the FC tail edge down and out the right side, the same path as without the fin.
  - Radxa port plugs (90° USB-C, 14 over the port edge): 3.0 clear.
  - A Ø10 tool or plug from there reaches outside the shell, sideways to the right through a cell: 6.2 clear.
  - microSD (15 pull out of the aft end): 0.9 clear. A Ø10 tool from the slot reaches outside the shell sideways: 6.1 clear.
  - Nothing has to come off.
- **Compact** (C):
  - The tail bay fits a 84 x 22.7 x 80.3 box: 0.153 L for 0.024 L of electronics.
  - The drone stays 378.9 x 278.3 x 125.1 (largest 378.9, box 13.19 L), the cage's own extents. Nothing sticks out.
- **Balance** (C on E masses): CG x −3.11 pack in, −0.16 pack out (3.12 / 0.47 off the thrust axis).
  - Pack in is 0.12 over the 3 gate before the catch-set scan.
  - The cage session's scan (catch set slid fore-aft, range 0..24, its standing lever) needs 3.5 forward: worst 1.12.
  - Reference: the cage session's own tail bay, as drawn, reads −0.43 / +3.92 in this model.
- **Mass** (E): fin + flange + clip 7 g and tail block 2 g before topology, longer leads 4 g. All-up in this model 647 g (C on E).

## Margins and the plan if they don't hold
- **Balance:** each gram at the fin moves the CG 0.16 aft (C). The catch-set slide of 3.5 (C) is the lever.
  1. If it cannot be had, the fin's topology target is ≤ 5 g (E): −0.3.
  2. The UBEC drops to the measured body. The 50 x 25 x 10 envelope is the worst of M; a 15 g body is E.
  3. Last resort: the Radxa turns end for end (SD forward), with access rechecked.
- **Wash 3.2 / 3.8 (C):** the root sits at the narrowest strip between the rear disks. Topology keeps inside this outline, so it can only grow the margins.
- **Fin to rods 3.3 (C):** this is the clip near the roof centre rod (z 61.5). The clip may move forward along the tie, never aft.
- **SD 0.9 (C):** the pull box against the fin's flange side. If the card catches, the fin gets a 3 deep notch behind the slot (E).

## Alternative B, not picked: crosswise tray behind the disks
- A Radxa laid lengthwise does not fit the pointed cone: rods at x −150..−169 (C).
- Crosswise behind the disks it fits: rods 12.5 off; `system_check` flat walls 17.1 (C).
- But it fails on three counts:
  - CG x −3.81 pack in needs a 5.3 slide (C).
  - The tray's spine from the plate crosses the rear wash (−24.5, C).
  - The tray blocks its plugs and SD (C).
- It is less compact: 0.206 L for 0.031 L (C).

## Open (DEFAULT, next)
1. Rerun on the race-car layout once it is on main. Hand the tail bay masses and CG to the cage session's scan (needs 3.5 forward).
2. System pass:
   - 35 g crash (E) and 50 N handling (E) into the root bolts, flange and clip;
   - the fin's first mode with the Radxa and UBEC;
   - cable runs: camera USB about 300 (E) from the nose (cable length CHECK), FC UART4 and 5 V to the Radxa, XR2 to the FC GPS port.
3. Then the fin's and the belly mount's topology.
