# X-axis flip swap v1 (Alex, 25 Oct 2026)

Battery shell, battery holder (drone and charging bay, one print) and pincer for a Zeee 4S 5200 mAh 120C hard-case pack
(138 x 47 x 48 listing, ±2, E; 470 g listing) on XT60 only.

## How it works
1. The drone sits on top with its pack hanging below in the **holder** (opens down). The pack lives in a **shell**; its
   own female XT60 sits in the shell's lead-end block, face up, and mates the holder's XT60E1-M panel male (6.0 engaged, C).
2. Two **clip beams** on the shell's long sides (at the swap CG, y 3.9) hang from the top deck. Each has a latch nose that
   rests on a tooth at the bottom of a holder arm (1.0 engaged, 0.2 drop, C), and a pad with a **finger pocket** lower down.
3. The **pincer** rides the vertical rail (the swap's X axis) up under the pack. Its two fingers close along x with one
   servo (rack and pinion, m 1.5, 18 T). Each finger's **shaped nose** (a 30 deg / 20 deg frustum on a 3 mm neck) seats in its
   pad pocket and pushes the pad in 3.1 mm. That one squeeze pulls both noses 1.48 in (0.48 clear of the teeth, C) and
   grips the shell through the pockets.
4. The rail moves down 80 (C): the XT60 pulls apart (~50 N, E) and the shell clears the holder.
5. The pincer flips the shell 180 deg about the pinch line (x axis through the pads). Hubs on both sides of the jaw
   take 8 mm D stub shafts in a fork on the rail carriage. Sweep radius 91.6 about the axis, x ±77 (C).
6. The rail moves down 85 more into the **bay holder**: the same print mounted upside down (opens up), 243 below the drone
   holder's mount face. The fingers open, the noses spring out under the bay teeth, and the pincer leaves.

Run back the other way for a charged pack. A bay is the drone holder turned 180 deg about the flip axis, so the lead
end swaps sides: mount each bay with its XT60 end opposite the drone holder's.

## Files
- `xswap.py` all geometry (one frame: x across the pack, y along it, +y XT60 end, z up, z 0 = shell top at seat).
- `xswap_check.py` seat, latch, pinched insert / pull sweep, relaxed-pull lock, pincer approach and grip, rack mesh over
  the stroke, flip vs both holders, bay seat, masses, hand calcs -> `runs/xswap/v1/result.json`, `check.txt`.
- `xswap_export.py` print files in print orientation -> `runs/xswap/v1/stl/`; `xswap_solid.py` connectivity check
  (0.3 mm voxels, 0.9 mm erosion, one body); `xswap_render.py` + `zrender.py` figures -> `runs/xswap/v1/figs/`.

## Checks (v1)
All overlaps 0 mm³ (C): seat, insert / pull sweep pinched (0..45 every 0.5), pincer approach (0..90), fingers shut on the
squeezed shell, pinion vs racks over the stroke, flip vs drone and bay holders, bay insert and seat. A relaxed shell
pulled down 0.3 collides (locked). Connectivity PASS on all 7 print files.

Hand calcs (E, PETG E 2000 MPa): beam 8.4 N per pad, root strain 1.03 % at full pinch (keep the servo stroke at 3.1);
nose 29 N each at a 10 g drop (0.6 MPa); pinch servo 0.52 N·m to hold the 60 N push (MG996R / DS3218 class is 1+ N·m);
flip 0.24 N·m static; 30 N per stub shaft at the push.

## Print (PETG, Bambu A1 mini, tree supports)
| File | Count | Orientation | g solid (C) |
|---|---|---|---|
| xswap_shell_v1 | 1 per pack | tail end down | 102 |
| xswap_xt60_keeper_v1 | 1 per pack | flat | 2 |
| xswap_holder_v1 | 1 drone + 1 per bay | mount face (rib tops) down | 106 |
| xswap_xt60_cap_v1 | 1 per holder | plate down | 4 |
| xswap_finger_v1 | 2 | rack bottom down | 13 |
| xswap_pinion_v1 | 1 | tooth face down | 8 |
| xswap_jaw_v1 | 1 | bottom down | 325 |

## Hardware
- Per shell: 2 x M3 x 60 + nyloc (tail cross bolts), 2 x M3 heat-set inserts + 2 x M3 x 6 (keeper), 1 mm foam tape.
  The pack's XT60 is re-soldered: thread the leads through the keeper's slot, solder, push the housing up into the
  pocket (face 0.5 proud), screw the keeper on; a drop of epoxy round the housing takes the 50 N pull.
- Per holder: 1 x XT60E1-M, 2 x M3 inserts + 2 x M3 x 8 (cap), 4 x M3 inserts in the mount face (screws from the drone /
  bay side). The cap stands 3 proud of the mount face: the drone plate needs a cut-out for it and the XT60 leads.
- Pincer: 1 x MG996R / DS3218 class servo with its round horn, 4 x M3 inserts + M3 x 8 (servo ears), horn screws
  (pinion), 2 x 8 mm D stub shafts + 4 x M3 inserts + 4 x M3 set screws (hubs), a flip actuator ≥ 0.5 N·m on one stub, a
  rail carriage fork with 2 x 608 bearings outside x ±77. Rail travel ≥ 165, force ≥ 60 N each way plus weights.

## Open (measure before printing, E)
1. Caliper the pack (PACK_* in `xswap.py`), the servo (case, ears, shaft offset, horn Ø21.5 and its screw circle) and the
   XT60E1-M flange; rerun the three scripts.
2. XT60-only charging means **no cell balancing**: balance-charge each pack on the bench now and then, or add a balance
   connector to the shell and bay.
3. The 60 N insert push lifts the drone: it must be held down on the box (a hook or latch on its frame) during insert.
4. First print: a shell + holder coupon. Check the pinch force, that the noses snap under the teeth when released, and
   50 insert / pull cycles; Amass rates most XT60s ~100 mating cycles, so plan the holder XT60 as a wear part.
