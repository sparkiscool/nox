# Scout v3 electronics in the pointed tail: concept (27 Oct 2026)

Spark, 27 Oct: "check the new scout cage design it should have a pointy tail and you need to fit the stuff there". Before that, the same day: the electronics list is confirmed, no lights, and the battery side is left as it is (`nox/scout/v3/README.md` Electronics mount).

- Script: `nox/scout/v3/electronics_tail.py`.
- Numbers: `nox/dimensions.md` System Scout v3, row "Electronics tail bay".
- Context: the race-car cage (Scout v3 design session, artifact WHdjYkzKPifXuHT1dWeqoR v5). Its meshes and rods are saved in `ctx/`; everything here is E on that geometry until its run is on main.
- **Visual:** https://claude.ai/artifact/EiZ5Z7WktXkSQePPHYEPiU (source `preview.png`): top row A, bottom row B, side and top views each.

Tags: C = computed by the script on the ctx geometry; E = estimate; H / M = `dimensions.md` sections.

## What goes in the tail
| Moves aft | Stays |
|---|---|
| Radxa Zero 3W, 5 V UBEC, XR2 | FC on the plate centre (IMU, motor leads), flow under the belly (its own view, PX4 offset), camera at the nose, the whole battery side |

## Concept A, picked: tail fin clipped to the tail core tie
- **Mount:** one PETG fin on the plane y +5, web 2.4 (C).
  - Root lug over the receiver top, onto the belly mount's tail face: x −56, z 11.6..18.5 (C).
  - Front edge x −62 (C), 2 behind the receiver end pieces. Back to x −140, z −26..34 (C).
- **Closed load path (review 27 Oct):** the fin rises to a split clip round the tail core tie (3.0 carbon) at x −100..−112 (C). The clip has a 3.3 bore and a 3.0 wall (boss rule).
  - The path plate tail → tie → clip → fin → root lug → belly mount → plate is a triangle, not a stalk.
  - Away from the clip, the fin is ≥ 3.3 off every other rod (C).
- **Radxa:** on edge on the −y face, on 3.0 standoffs (clear of the SD slot, H), component side out, ports edge up, SD end aft.
  - x −130..−65, z −24..6 (C).
  - Standing vertical, it cools in the forward-flight air. It is out of the pack's shadow, so the onboard wifi radiates sideways.
- **UBEC:** worst-size envelope 50 x 25 x 10 (M Q4) on the +y face, x −138..−88, z −12..13 (C).
- **XR2:** on the +y face at the top aft corner, x −132..−120, z 16..32 (C). Antenna up to z 46 (E), into free air behind the props.
- **Carbon:** no new rods. The fin, clip and lug are printed PETG. The clip holds the existing tail core tie (the cage session's rod).

## Checks (C on the ctx geometry, E)
| Check | Radxa | UBEC | XR2 | Fin + clip |
|---|---|---|---|---|
| Rod surface (cage first, ≥ 3) | 11.0 | 12.1 | 10.9 | 3.3 (the tie it clips excepted) |
| Inside the shell | yes | yes | yes | yes |
| Out of the rear prop wash (disk R 44.5, ≥ 3) | 6.9 | 6.4 | 33.6 | 3.5 |
| Nearest drone part | 5.0 receiver end pieces | 28.0 | 62.5 | 0.2 belly mount (its joint) |

- **Access** (C, plug sizes E):
  - FC USB-C plug + cable (12 x 7, 250 long): best path from the FC tail edge, down and out the right side, clear by 6.9. That is the same as with the cage session's tail bay, so the fin does not touch it.
  - Radxa port plugs (90° USB-C, 14 over the port edge): 2.6 clear.
  - microSD out of the aft short end (15 pull): 2.5 clear.
- **Compact** (C):
  - The tail bay fits a 84 x 20.8 x 80.3 box: 0.141 L for 0.024 L of electronics.
  - The drone stays 378.9 x 278.3 x 125.1, the cage's own extents. Nothing sticks out.
- **Balance** (C on E masses):
  - CG x −2.77 pack in, +0.36 pack out: 2.79 / 0.56 off the thrust axis, under the 3 gate.
  - Reference: the cage session's own tail bay, as drawn, reads −0.43 / +3.92 in this model.
  - The cage session's balance scan should move the catch set 2.6 forward: worst 1.31.
- **Mass** (E): fin 6 g before topology, longer leads 4 g. All-up in this model 644 g.

## Margins and the plan if they don't hold
- **Pack-in CG 2.79 against the gate 3:** each gram added at the fin moves the CG 0.16 aft (C).
  1. The catch-set slide of 2.6 is the lever. It sits inside the cage session's scan range 0..24, and the summary hands it to them.
  2. If the slide does not land, the fin's topology target is ≤ 6 g, so no added mass.
  3. The XR2 moves forward to the fin root (x −66): 0.1 mm.
  4. Last resort: the Radxa turns end for end, SD end forward, with ports and SD reachable again checked.
- **Fin wash 3.5** is the fin's lower front corner beside the rear disks. Topology keeps the fin inside this outline, so it can only grow the margin.
- **Fin to rods 3.3** is the clip zone near the roof rod (z 61.5). The clip may move forward along the tie, never aft.

## Alternative B, not picked: crosswise tray behind the disks
- A Radxa laid lengthwise does not fit the pointed cone: rods at x −150..−169.
- Crosswise behind the disks it fits: rods 12.5 off (C).
- But it fails on three counts:
  - CG x −3.81 pack in (C) needs a 5.3 re-slide.
  - The tray's spine from the plate crosses the rear wash (−24.5, C).
  - Its plugs and SD are blocked by the tray (C).
- It is less compact: 0.206 L for 0.031 L.

## Open (DEFAULT, next)
1. Rerun on the race-car layout once it is on main. Hand the tail bay masses and CG to the cage session's scan.
2. System pass:
   - 35 g crash and 50 N handling loads into the fin root and clip;
   - the fin's first mode with the Radxa and UBEC;
   - cable runs: camera USB ~300 from the nose (cable length CHECK), FC UART4 and 5 V to the Radxa, XR2 to the FC GPS port.
3. Then the fin's topology, with the belly mount round the FC and flow.
