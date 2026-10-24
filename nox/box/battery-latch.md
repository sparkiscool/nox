# Battery latch v1: one latch for every pack (24 Oct 2026)

Spark 24 Oct: "design the new latch system for all batteries so the grabber can follow it". One latch + cartridge standard for every pack (Scout 4S 1500, Overseer 4S 6000), in the drone receiver and in every box bay, worked by the swap arm's grabber. Script: `nox/box/battery_latch.py` (parts, sweeps, hand calc, reactions, system checks, `latch_assembly(pack, state)` for `swap_system.py`); `nox/box/battery_latch_topo.py` (receiver FE on the L cases, `--fe-v1`, and the receiver topology setup); `nox/box/battery_latch_figure.py` (figures). Run: `runs/battery_latch/v1/` (`result.json` holds every number below). Shared numbers: `nox/dimensions.md` "System box swap: latch". Layout artifact: https://claude.ai/artifact/MX2dpRcjHegntjHvqp6U8x. Owner: battery latch worker (control session's box swap system).

## Frames (C)
- **Cartridge frame c** (as the Scout bottom layout, PR 44): x across the pack, y up the install axis (0 = flat bottom = ring r 100 in a bay), z along the pack (0 = closed end). The tab line z_T is the swap assembly's CG (pack + plug + cartridge + sliders), so the gripper holds the pack at its CG and the drone puts it on the thrust axis.
- **Scout drone frame d** = (z_T − z, −x, y − 44.5): tab line at drone x 0, flat bottom at drone z −44.5 (ring r 100, as PR 44).
- **Wheel W** (dimensions I): X = −x, Y = Y_C + (z − z_T), Z radial = r 100 + y.

## Concept check (24 Oct, before the piece runs)
Requirements (the brief): hold every L case for both packs; XT60 mates on insert (60 N) and unmates on pull (50 N) on the install axis, XT60E1-M float carrier on the receiver; latched state readable; a half-in pack never reads or stays latched; engages by the insert stroke alone, the grip releases it, releasing the grip locks it; the same grip interface on both packs; no more complex than the v2 squeeze latch; Scout receiver + cartridge ≥ 7.0 inside the collar planes below z 20 with the keel nodes free; pack CG on the thrust axis; breaks after the cage.

| | A: v2 / v4 squeeze (PR 44, baseline) | **B: squeeze v5, one latch module (chosen)** | C: snap hooks on the receiver | D: one bolt + fixed lug | E: push-push (heart cam) |
|---|---|---|---|---|---|
| How | 2 sliders + 1 pen spring in the cartridge, passive pockets in the receiver, tabs in a bottom recess | A's mechanism; one tab / recess / bolt / pocket module (same numbers on every cartridge), bolt overlap 2.3, undercut 15°, tab line on the swap CG, receiver as a frame (top band + 3 legs a side), ends clipped to the collar planes, magnet in each tab | 2 PETG cantilever hooks on the receiver snap over cartridge ledges; fingers spread them from outside the cartridge | 1 slider at the lead end, a fixed lug at the closed end; squeeze one tab against a fixed face | insert latches, a second push over-travel releases |
| Printed / moving parts | 7 / 2 | 7 / 2 | 5 / 0 (2 flexures) | 6 / 1 | 6 / 2 |
| Hold, 35 g crash | Scout 0.07 (hand calc; not sized for the Overseer's 257 N) | FE crash worst Scout 0.36, Overseer 0.44 | > 1 (E): a hook the 15 N finger can spread holds ~5x that, not 257 N | 2x B per bolt, pack pivots on the lug in a side hit (E 0.5) | cam pin in a printed track, ~1 (E) |
| Grabber stroke / force | 4.0 per finger, spring ≥ 15 N, release ~2.5 N | **4.0 per finger, tabs x ±4..10 → 0..6, ≥ 15 N spring, release 3.0 (Scout) / 4.5 N (Overseer)** | new fingers outside the cartridge, ~3 stroke, ~20 N | 4.0, one finger braces | none (stroke only), but the grip cannot release it (brief fails) |
| ±1.0 head offset | **FAIL**: 3.0 overlap needs 3.3 of retract, a 1.0 offset leaves one bolt 3.0 | PASS: the short bolt retracts 3.0, its tip 0.7 inside the face (swept, 0 mm³) | fingers must find the hooks behind the cage | as B | n/a |
| Scout collar planes (need 7.0) | 1.01 / 1.50 FAIL (led the cage by 16.4) | **7.00** (C) | ~6 (wider receiver for the hook arms) | as B | as B + 4 (cam track) |
| Print risk | low (PR 44 coupon) | low (same channel, 0.35 clearance) | high: PETG flexure creep under the 12 N preload, fatigue | low | high: 0.5 cam features |
| Mass, Scout cartridge + receiver | 122.3 g (M) | 87.9 g (M) | ~70 g (E) | ~80 g (E) | ~95 g (E) |

**Chosen: B (DEFAULT).** It is A's mechanism (equal parts count; Spark locked the squeeze latch 15 Oct) with the four things A lacks: the ±1.0 misalignment, the collar planes (7.0), the Overseer's 257 N, and one grip interface on both packs. C wins 2 parts and ~15 g but cannot hold 35 g with a finger-releasable hook and puts the fingers outside the cartridge, behind cage members; D saves a slider but halves the hold and lets the pack pivot; E fails "the grip releases it". A fails the misalignment and the collar planes.

**Review (concept mode, 24 Oct): FAIL, 6 findings, all taken:**
1. Spring rest force (0.68 N) under a slider's 35 g inertia (1.1 / 1.4 N) in a side hit. A stiffer o 3 spring goes solid in the 8.0 squeezed length (checked: k 0.5 needs ~29 coils of 0.4 wire, 11.6 solid), so the undercut went 10° → 15°: the 1 g hang's wedge + friction now adds to the spring, hold / inertia x1.27 (Scout) / x2.07 (Overseer); the outboard bolt is pushed further in, so a side hit can only ever move one bolt for the pulse. Spring now k 0.2, rest 0.8 N.
2. Overseer pull-out along a 163 span: the Overseer receiver gets 2 more bosses at the tab line (6 total); the receiver is now checked by voxel FE on the L cases (`--fe-v1`), which found and fixed two more weak spots: the end plates under ±z crash (end faces now relieved 0.5 in the middle so the cartridge bears at its corners, into the side frames; Overseer +z 1.56 → 0.44) and the end legs under a 50 N handling push (legs 16 / 24 long).
3. Half-in: the hall model gives 96.7 mT latched / 7.2 half in / 1.0 squeezed; the box seat check gates the drone's release (dimensions "Latched state").
4. Overlap 2.5 → 2.3; asymmetric squeeze swept at ±1.0: 0 mm³, pull-out clear, tip 0.7 inside the face.
5. "System box swap: latch" written, with reactions and `budget_g`; v3 baseline 72.1 g noted.
6. Overseer posed at A (on the thrust axis, the Overseer DEFAULT) and B; the balance FAIL in `overseer.assembly.json` is that layout's, not the latch's.

## Pieces
| Piece | Material, count | Scout g (M) | Overseer g (M) | Print |
|---|---|---|---|---|
| `box_battery_cartridge_<pack>_v5` | PETG x 1 | 33.1 | 88.9 | closed end down, tree supports (174 tall for the Overseer) |
| `box_battery_slider_<pack>_v5` | PETG x 2 (one part, both sides) | 3.2 | 4.0 | inner tab face down |
| `box_battery_receiver_<pack>_v1` (latch block, drone and bays) | PETG x 1 | 41.2 | 79.5 | closed-end face down, tree supports |
| `box_battery_receiver_lid_<pack>_v1` | PETG x 1 | 5.3 | 6.9 | top face down |
| `box_battery_receiver_strip_<pack>_v1` | TPU 95A x 2 | 1.0 | 2.1 | flat |
Bought per set: XT60E1-M, M3 x 5.7 heat-set x 4 (Overseer 6) + M3 bolts, o 3 x 20 compression spring (0.3 wire, k ~0.2), 2 x o 2 x 2 N52 magnets, 2 x M3 x 5 cup-point set screws. Assembly: sliders into the cartridge channels from inside, spring into its U-slots, pack in (it closes the slots), plug into its pocket, set screws; XT60E1-M into the receiver pocket, lid on with the lead-end mount bolts.

## Checks (C unless tagged, `result.json`)
- **solid_check PASS** on all 10 print files (1 body, 1 eroded body, 0 necks, 0 thin walls); print_orient: one first-layer island each, no thin island run.
- **Insertion:** sliders free, from 50 below seat every 1.0, and 0.3 off in x: 0 mm³ cartridge vs receiver, no stuck step (the feet cam the bolts in). Gripped pull-out (sliders squeezed) 0 mm³. Squeeze 0..4.0: 0 mm³ against cartridge, receiver and each other. Bolts bear after a 0.8 drop.
- **FE (receiver, h 1.0):** Scout crash worst 0.36, handling 0.67; Overseer crash worst 0.44, handling 0.73; flight and swap ≤ 0.2.
- **Scout v3 system** (pass 1 snapshot, `runs/battery_latch/v1/scout_v3_pass1/`): System: Scout v3, cage first −3.1 mm (E) at (−0.28, +0.24, −0.93) (cartridge) FAIL, fit 2193.6 mm³ (E) (pogo pads + carrier / receiver) FAIL, bbox 6.57 L (E), extents 233 x 233 x 122 mm (E), mass 689 g (E), CG 0.8 mm off thrust axis (E), 0.9 mm without swap parts, probes −34.0 mm (camera, unshielded) FAIL. Collar planes 7.00, keel nodes 5.3 / 10.0 / 5.3 clear. The FAILs are the Scout's: its cage lacks the 4th keel node at the tail-left, its PR 44 pogo carrier and cage joints sat on the old receiver, the camera probe is the nose.
- **Overseer** (`overseer.assembly.json` on main, cartridge and pack replaced): sled vs receiver 12544 mm³ (A) / 11300 (B), the sled's to cut.

## Handoffs (other sessions own these)
- **Swap arm v6** (`swap_system.py`): grabber interface and forces in dimensions "System box swap: latch"; the head hall at (x 7.0, y −1.5, z z_T + 8) under each latched tab magnet; `latch_assembly(pack, "latched" | "gripped" | "free")` gives the meshes in cartridge coords; the y stroke is the same for both packs (flat bottom at r 100).
- **Scout v3:** a belly frame on the 4 mount bolts (drone (41.85, ±25.6), (−43.25, ±25.6), mount plane z 9.6) carrying the plate posts, pogo carrier, Radxa, flow and the cage's core joints; the 4th keel node at (−28, 28.5, −47.5) for cage first.
- **Overseer:** the sled cut round the receiver box + 1.0 with the 6-bolt mount plane.
- **Box structure:** bay bodies give each receiver its mount plane (Scout r 154.1, Overseer r 167.1 at the bay) and the strip side; the bay receiver is the same print file.

## Open (E)
- Spring rate and the XT60 forces are E; the first latch print checks release force per finger (≤ 15 N), the snap-in at the seat, and that the pack drops 0.8 onto the bolts when let go.
- Hall part not chosen (threshold ~50 mT; a switch with B_op 15..60 mT or an analog hall).
