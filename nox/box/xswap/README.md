# X-axis flip swap v2 (Alex, 25 Oct 2026)

Battery shell, battery holder (drone and charging bay, one print) and pincer for a Zeee 4S 5200 mAh 120C hard-case pack
(138 x 47 x 48 listing, ±2, E; 470 g listing) on XT60 only. v2 adds guided capture for an imprecise rail and a latch
that holds through a 5 to 10 mph crash. v1 (flexure clips) is in git history and `runs/xswap/v1/`.

## How it works
1. The drone sits on top with its pack hanging below in the **holder** (opens down). The pack lives in a **shell**; its
   own female XT60 sits in the shell's lead-end block, face up, and mates the holder's XT60E1-M panel male (6.0 engaged).
2. **Latch:** a rigid PETG **rocker** on each long side of the shell (at the swap CG, y 3.9) turns on an M4 pin in two
   ears. Its hook drops into a pocket in a solid **lug** on the holder and sits 2.5 over a ledge. The pin is inboard of the
   hook contact, so any pull on the pack turns the hook further in: pulling cannot open it, only pressing the pads can.
   A 5 x 13 spring holds each rocker closed; a rest stop and a hard stop on the shell limit it (0 / 15 deg).
3. **Guidance:** two bullet-nosed guide posts on the holder (r 5, tip r 1.5) find two sockets on the shell (entry r 7.2)
   18 to 25 mm before seat, then a 30 deg funnel (6 per side, 3.5 at the ends) and the skirt lead-ins, then the XT60
   mates on the last 6. Checked: offsets to ±5 in x, y and diagonals, and tilts to 3 deg, always touch a post first.
4. **Crash:** the floor is solid 3.0; two keys under the holder deck drop into the shell's deck windows and take side
   and end hits; the lugs carry pull-out straight up into 4 extra M3 inserts in the drone mount face.
5. **Pincer:** rides the vertical rail. Each finger has a round nose bar (r 3) that lands in the pad's round slot; the
   finger pushes the pad, the rocker turns 13 deg and both hooks clear the ledges (0.44 clear). The round nose lets the
   rocker turn round it; the slot lets the nose ride 2.6 up the pad as it turns.
6. **Sequence:** rise, pinch, rail down 65 (XT60 apart, clear), flip 180 deg about the finger line (hubs on both sides,
   8 mm D stubs in a carriage fork), rail down 65 into the **bay holder** (the same print upside down), open: the springs
   swing the hooks over the bay ledges.

## Checks (v2, `runs/xswap/v2/check.txt`, `result.json`)
All 0 mm³ (C): seat; rockers over 0..13 deg; insert / pull sweep pinched (0..55 every 0.5); pincer approach (0..90);
finger nose tracking the rocker 0..13 deg; fingers on the pinched shell; pinion and racks over the stroke; flip vs drone
and bay holders (shell nominal and hanging 2.6); bay insert and seat. A latched shell pulled down 0.6 collides (locked).
Connectivity PASS on all 8 print files (`xswap_solid.py`).

Crash (E): 150 g on the 637 g swap mass = 937 N pull-out (10 mph = 4.47 m/s stopped in ~7 mm), 469 N per rocker:
hook root 10.4 MPa, hook bearing 6.7, pin boss 4.2, M4 pin 18.6 (steel), ears 9.8 / 5.8, lug spine 4.6 across layers,
ledge 8.8, 234 N per lug insert, floor 20.0 MPa (all in-layer except the lug spine; PETG ~45 in-layer, ~15-20 across).
Closing torque from the hook load 1476 N·mm vs 86 opening from the rocker's own inertia. Side / end hit: keys 12.5 MPa.

## Print (PETG, Bambu A1 mini, tree supports)
| File | Count | Orientation | g solid (C) |
|---|---|---|---|
| xswap_shell_v2 | 1 per pack | tail end down | 129 |
| xswap_rocker_v2 | 2 per pack | on its side | 9 |
| xswap_xt60_keeper_v2 | 1 per pack | flat | 2 |
| xswap_holder_v2 | 1 drone + 1 per bay | mount face (rib tops) down, 178.6 long | 163 |
| xswap_xt60_cap_v2 | 1 per holder | plate down | 4 |
| xswap_finger_v2 | 2 | rack bottom down | 11 |
| xswap_pinion_v2 | 1 | tooth face down | 8 |
| xswap_jaw_v2 | 1 | bottom down, 172 wide | 331 |

## Hardware
- Per shell: 2 x M4 x 45 + nyloc (rocker pins), 2 x 5 x 13 compression springs (~0.5 N/mm, E), 2 x M3 x 60 + nyloc
  (tail), 2 x M3 inserts + 2 x M3 x 6 (keeper), 1 mm foam tape. Re-solder the pack's XT60 through the keeper (epoxy it).
- Per holder: 1 x XT60E1-M, 2 x M3 inserts + 2 x M3 x 8 (cap), 8 x M3 inserts in the mount face (4 corners + 4 over the
  lugs), screws from the drone / bay side. The cap stands 3 proud: cut the drone plate for it and the leads.
- Pincer: as v1 (MG996R / DS3218 class servo, 8 mm D stubs, 608s in your carriage fork, now outside x ±85.5). Rail travel
  ≥ 130, ≥ 60 N each way.

## Open (measure before printing, E)
1. Caliper the pack, servo, horn and XT60E1-M flange; set the numbers at the top of `xswap.py`; rerun check, export, solid.
2. XT60-only charging means no cell balancing: balance-charge on the bench, or add a balance connector later.
3. The 60 N insert push lifts the drone: hold it down on the box during the insert.
4. First print: one shell, two rockers, one holder. Check the springs return the rockers, the hooks drop over the ledges
   by themselves, a hard pull on the pack does nothing, and a 5 mm misaligned push still seats.
