# Scout v3 electronics in the pointed tail: concept (27 Oct 2026)

Spark, 27 Oct: "check the new scout cage design it should have a pointy tail and you need to fit the stuff there". Before that, the same day: the electronics list is confirmed, no lights, and the battery side is left as it is (`nox/scout/v3/README.md` Electronics mount).

- Script: `nox/scout/v3/electronics_tail.py`.
- Numbers: `nox/dimensions.md` System Scout v3, row "Electronics tail bay".
- Context: the race-car cage (Scout v3 design session, artifact WHdjYkzKPifXuHT1dWeqoR v5). Its meshes and rods are saved in `ctx/` (E until that layout run is on main; this concept is rerun on it then).
- Picture: `preview.png`.

## What goes in the tail
| Moves aft | Stays |
|---|---|
| Radxa Zero 3W, 5 V UBEC, XR2 | FC on the plate centre (IMU, motor leads), flow under the belly (its own view, PX4 offset), camera at the nose, the whole battery side |

## Concept A, picked: tail fin
- **Mount:** one PETG fin on the plane y +5, 2.4 web.
  - Root lug over the receiver top, onto the belly mount's tail face (x −56, z 11.6..18.5, under the plate).
  - Front edge x −62, 2 behind the receiver end pieces. Runs back to x −133, z −26..34.
  - Its faces are 2.3 clear of the tail core tie, which runs on the centre plane.
- **Radxa:** on edge on the −y face, on 3.0 standoffs (clear of the SD slot), component side out.
  - Ports edge up: camera USB-C and OTG reachable from above. SD end aft.
  - x −130..−65, z −24..6.
  - Standing vertical, it cools in the forward-flight air. It is out of the pack's shadow, so the onboard wifi radiates sideways.
- **UBEC:** worst-size envelope 50 x 25 x 10 (M Q4) on the +y face, x −138..−88, z −12..13.
- **XR2:** on the +y face at the top aft corner, x −132..−120, z 16..32. Antenna up to z 46, into free air behind the props.

## Checks (on the artifact geometry, E)
| Check | Radxa | UBEC | XR2 | Fin |
|---|---|---|---|---|
| Rod surface (cage first, ≥ 3) | 11.0 | 12.1 | 10.9 | 2.3 (tail core tie, mount only) |
| Inside the shell | yes | yes | yes | yes |
| Out of the rear prop wash (disk R 44.5, ≥ 3) | 6.9 | 6.4 | 33.6 | 3.5 |
| Nearest drone part | 5.0 receiver end pieces | 28.0 | 62.5 | 0.2 belly mount (its joint) |

- **Balance:** CG x −2.73 pack in, +0.42 pack out (2.75 / 0.61 off the thrust axis). Both are under the 3 gate, but pack in is tight.
  - Reference: the cage session's own tail bay, as drawn, reads −0.43 / +3.92 in this model.
  - The cage session's balance scan should move the catch set ~2.5 forward (E): worst 1.34.
- **Mass:** fin 6 g (E, before topology), longer leads 4 g (E). All-up in this model 644 g.

## Alternative B, not picked: crosswise tray behind the disks
- A Radxa laid lengthwise does not fit the pointed cone: rods at x −150..−169.
- Crosswise behind the disks it fits: rods 12.5 off.
- But CG x −3.81 pack in needs a 5.3 re-slide. The tray's spine from the plate crosses the rear wash (−24.5). It is a longer cantilever, and the Radxa lies flat under the roof rods.

## Open (DEFAULT, next)
1. Rerun on the race-car layout once it is on main. Hand the tail bay masses and CG to the cage session's scan.
2. System pass:
   - 35 g crash and 50 N handling loads into the fin root;
   - the fin's first mode with the Radxa and UBEC;
   - cable runs: camera USB ~300 from the nose (cable length CHECK), FC UART4 and 5 V to the Radxa, XR2 to the FC GPS port;
   - the FC USB-C access path, which the cage session routes out the tail-left through a cell and the UBEC now sits beside.
3. Then the fin's topology, with the belly mount round the FC and flow.
