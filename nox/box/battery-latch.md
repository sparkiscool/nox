# Battery latch v1: one latch for every pack (24 Oct 2026)

Spark 24 Oct: "design the new latch system for all batteries so the grabber can follow it". One latch + cartridge standard for every pack (Scout 4S 1500, Overseer 4S 6000), in the drone receiver and in every box bay, worked by the swap arm's grabber. Script: `nox/box/battery_latch.py` (parts, sweeps, hand calc, reactions, system checks, `latch_assembly(pack, state)` for `swap_system.py`). Run: `runs/battery_latch/v1/` (`result.json` holds every number below). Numbers shared with other systems: `nox/dimensions.md` "System box swap: latch". Owner: battery latch worker (control session's box swap system).

## Frames (C)
- **Cartridge frame c** (as the Scout bottom layout, PR 44): x across the pack, y up the install axis (0 = flat bottom = ring r 100 in a bay), z along the pack (0 = closed end). The tab line z_T is the swap assembly's CG (pack + plug + cartridge + sliders), so the gripper holds the pack at its CG and the drone puts it on the thrust axis.
- **Scout drone frame d** = (z_T − z, −x, y − 44.5): tab line at drone x 0, flat bottom at drone z −44.5 (ring r 100, as PR 44).
- **Wheel W** (dimensions I): X = −x, Y = Y_C + (z − z_T), Z radial = r 100 + y.

## Concept check (24 Oct, before the piece runs)
Requirements (the brief): hold every L case for both packs; XT60 mates on insert (60 N) and unmates on pull (50 N) on the install axis, XT60E1-M float carrier on the receiver; latched state readable; a half-in pack never reads or stays latched; engages by the insert stroke alone, the grip releases it, releasing the grip locks it; the same grip interface on both packs; no more complex than the v2 squeeze latch; Scout receiver + cartridge ≥ 7.0 inside the collar planes below z 20 with the keel nodes free; pack CG on the thrust axis; breaks after the cage.

| | A: v2 / v4 squeeze (PR 44, baseline) | **B: squeeze v5, one latch module (chosen)** | C: snap hooks on the receiver | D: one bolt + fixed lug | E: push-push (heart cam) |
|---|---|---|---|---|---|
| How | 2 sliders + 1 pen spring in the cartridge, passive pockets in the receiver, tabs in a bottom recess | A's mechanism; one tab / recess / bolt / pocket module (same numbers on every cartridge), bolt overlap 2.5, tab line on the swap CG, receiver as a frame (top band + 3 legs a side), ends clipped to the collar planes, magnet in each tab | 2 PETG cantilever hooks on the receiver snap over cartridge ledges; fingers spread them from outside the cartridge | 1 slider at the lead end, a fixed lug at the closed end; squeeze one tab against a fixed face | insert latches, a second push over-travel releases |
| Printed / moving parts | 7 / 2 | 7 / 2 | 5 / 0 (2 flexures) | 6 / 1 | 6 / 2 |
| Hold, 35 g crash (worst index, E) | Scout 0.07 (not sized for the Overseer's 257 N) | Scout 0.07, Overseer 0.25; worst case anywhere 0.39 (grip pull) | > 1 (E): a hook the 15 N finger can spread holds ~5x that, not 257 N | 2x B per bolt, pack pivots on the lug in a side hit (E 0.5) | cam pin in a printed track, ~1 (E) |
| Grabber stroke / force | 4.0 per finger, spring ≥ 15 N, release ~2.5 N | **4.0 per finger, tabs x ±4..10 → 0..6, ≥ 15 N spring, release 2.5 (Scout) / 3.8 N (Overseer)** | new fingers outside the cartridge, ~3 stroke, ~20 N | 4.0, one finger braces | none (stroke only), but the grip cannot release it (brief fails) |
| ±1.0 head offset | **FAIL**: 3.0 overlap needs 3.3 of retract, a 1.0 offset leaves one bolt 3.0 | PASS: 2.5 overlap needs 2.8, worst bolt retracts 3.0 (0.2 spare); blades enter at 0 mm³ at ±1 | fingers must find the hooks behind the cage | as B | n/a |
| Scout collar planes (need 7.0) | 1.01 / 1.50 FAIL (led the cage by 16.4) | **7.00** (C) | ~6 (wider receiver for the hook arms) | as B | as B + 4 (cam track) |
| Print risk | low (PR 44 coupon) | low (same channel, 0.35 clearance) | high: PETG flexure creep under the 12 N preload, fatigue | low | high: 0.5 cam features |
| Mass, Scout cartridge + receiver | 122.3 g (M) | 85.2 g (M, before topology) | ~70 g (E) | ~80 g (E) | ~95 g (E) |

**Chosen: B (DEFAULT).** It is A's mechanism (equal parts count, Spark locked the squeeze latch 15 Oct) with the four things A lacks: the ±1.0 misalignment (overlap 2.5), the collar planes (7.0), the Overseer's 257 N, and one grip interface on both packs. Alternatives weighed: C wins 2 parts and ~15 g but cannot hold 35 g with a finger-releasable hook and puts the fingers outside the cartridge, behind cage members; D saves a slider but halves the hold and lets the pack pivot; E fails "the grip releases it". A is B's baseline and fails the misalignment and the collar planes.

Rule checks on the layout (C, `result.json`; the Scout run uses the Scout v3 system pass 1 assembly on branch `claude/gracious-darwin-ogidvp`, its PR 44 receiver, cartridge, pack and XT60 swapped for this set):
- **Scout:** collar margin 7.00 (±5° yaw, ±2° tilt, below z 20); keel nodes 8.8 / 10.0 / 8.8 clear; CG 0.8 off the thrust axis, 0.9 without the swap parts (pass); cage first −2.9 at the tail-left bottom (the cage's 4th keel node (−28, 28.5, −47.5) is missing, Scout v3 cage pass), fit: the PR 44 pogo carrier and the cage's PR 44 receiver joints clash with the new block (Scout v3 session redraws them on the mount interface below). Was −16.4.
- **Overseer** (`overseer.assembly.json`, its cartridge and pack replaced, tab line 30 behind the thrust axis, system DEFAULT B; layout A puts it on the axis): fit vs the belly sled 10537 mm³ (the sled is cut round the receiver envelope below, Overseer session); balance is the layout's.
- Bounding box: Scout set 61 x 54 x 102 mm (0.34 L), Overseer set 76 x 67 x 180 mm (0.92 L).
