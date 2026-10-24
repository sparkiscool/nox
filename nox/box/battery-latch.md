# Battery latch v1: one latch for every pack (24 Oct 2026)

Spark 24 Oct: "design the new latch system for all batteries so the grabber can follow it". One latch + cartridge standard for every pack (Scout 4S 1500, Overseer 4S 6000), in the drone receiver and in every box bay, worked by the swap arm's grabber. Script: `nox/box/battery_latch.py` (parts, sweeps, hand calc, reactions, system checks, `latch_assembly(pack, state)` for `swap_system.py`); `nox/box/battery_latch_topo.py` (receiver FE on the L cases, `--fe-v1`, and the receiver topology setup); `nox/box/battery_latch_figure.py` (figures). Run: `runs/battery_latch/v1/` (`result.json` holds every number below). Shared numbers: `nox/dimensions.md` "System box swap: latch". Layout artifact: https://claude.ai/artifact/MX2dpRcjHegntjHvqp6U8x. Owner: battery latch worker (control session's box swap system).

## Frames (C)
- **Cartridge frame c** (as the Scout bottom layout, PR 44): x across the pack, y up the install axis (0 = flat bottom = ring R 113 in a bay, arm v6), z along the pack (0 = closed end). The tab line z_T is the swap assembly's CG (pack + plug + cartridge + sliders), so the gripper holds the pack at its CG and the drone puts it on the thrust axis.
- **XT60 end = arm side (hub 24 Oct, from the arm v6 system pass, PR 71, DEFAULT):** the receiver gives no pitch guidance before its deck at the seat, so the XT60 mates on the arm's own pitch stiffness. Free pitch at the mate vs the float carrier's ±1.06°: XT60 at the far end 2.56 / 1.71 / 3.47° (Scout at Y_C 175 / Scout 129.1 / Overseer, R 113), at the arm-side end with carbon ties on the arm 1.03 / 0.52 / 0.70° (arm v6 numbers). So the whole latch set is turned 180° about the install axis: the cartridge's lead (XT60) end faces the arm (−Y); no geometry changed. Alt (not taken): receiver pitch guidance before the mate, which needs z-locating faces engaged over ~25 before the seat at both ends, still ~0.7° on 0.3 gaps.
- **Scout drone frame d** = (z − z_T, x, y − 44.5): tab line at drone x 0, XT60 end at the nose (+x), flat bottom at drone z −44.5.
- **Tab line:** the heads grip every cartridge at one Y_C on the drone's thrust axis (arm v6). Scout: z_T = its swap CG (42.55). Overseer: z_T 110.76 = swap CG 80.76 + 30 toward the XT60 end, because its pack sits 30 behind its thrust axis (Spark: B); the grip then carries 0.22 N·m at 1 g / 0.66 at 3 g, rib-pocket index 0.21 / 0.64 (E).
- **Wheel W** (dimensions I, arm v6): X = x, Y = Y_C − (z − z_T), Z radial = R + y, R 113 for every seated face (arm v6; was r 100); common grip at Y_C 175 for both packs.

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
1. Spring rest force (0.68 N) under a slider's 35 g inertia (1.1 / 1.4 N) in a side hit. A stiffer o 3 spring goes solid in the 8.0 squeezed length (checked: k 0.5 needs ~29 coils of 0.4 wire, 11.6 solid), so the undercut went 10° → 15°: the 1 g hang's wedge + friction now adds to the spring, hold / inertia x1.27 (Scout) / x2.07 (Overseer); the outboard bolt is pushed further in, so a side hit can only ever move one bolt for the pulse; with no hang (inverted, free fall) the spring alone is 0.8 N (x0.73 / x0.58 of the inertia), but then nothing pulls the pack out, and one bolt alone carries the corner case at index 0.08 / 0.28. Spring now k 0.2, rest 0.8 N.
2. Overseer pull-out along a 163 span: the Overseer receiver gets 2 more bosses at the tab line (6 total); the receiver is now checked by voxel FE on the L cases (`--fe-v1`), which found and fixed two more weak spots: the end plates under ±z crash (end faces now relieved 0.5 in the middle so the cartridge bears at its corners, into the side frames; Overseer +z 1.56 → 0.44) and the end legs under a 50 N handling push (legs 16 / 24 long).
3. Half-in: one hall per tab (x ±7.5, 1.0 under the flat bottom), searched and swept: latched ≥ 109 mT vs half in ≤ 16 at head offset ±0.5 (≥ 76 vs ≤ 38 at ±1.0); the box seat check (both halls + XT60 current) gates the drone's release (dimensions "Latched state"). Re-review (concept mode): PASS, follow-ups taken (keel node 4 stand-in, hall sweep, free-fall side hit, Overseer budget slots, Overseer clash owners).
4. Overlap 2.5 → 2.3; asymmetric squeeze swept at ±1.0: 0 mm³, pull-out clear, tip 0.7 inside the face.
5. "System box swap: latch" written, with reactions and `budget_g`; v3 baseline 72.1 g noted.
6. Overseer posed at B (pack 30 behind the thrust axis, Spark 24 Oct decision) and A; the balance numbers in `overseer.assembly.json` are that layout's, not the latch's.

## Pieces
| Piece | Material, count | Scout g (M) | Overseer g (M) | Print |
|---|---|---|---|---|
| `box_battery_cartridge_<pack>_v5` | PETG x 1 | 33.1 | 89.7 | closed end down, tree supports (174 tall for the Overseer) |
| `box_battery_slider_<pack>_v5` | PETG x 2 (one part, both sides) | 3.2 | 4.0 | bar top down (largest face, layers along the bar) |
| `box_battery_receiver_<pack>_v1` (latch block, drone and bays) | PETG x 1 | 41.2 | 79.5 | closed-end face down, tree supports |
| `box_battery_receiver_lid_<pack>_v1` | PETG x 1 | 5.3 | 6.9 | top face down |
| `box_battery_receiver_strip_<pack>_v1` | TPU 95A x 2 | 1.0 | 2.1 | flat |
Bought per set: XT60E1-M, M3 x 5.7 heat-set x 4 (Overseer 6) + M3 bolts, o 3 x 20 compression spring (0.3 wire, k ~0.2), 2 x o 2 x 2 N52 magnets, 2 x M3 x 5 cup-point set screws. Assembly: sliders into the cartridge channels from inside, spring into its U-slots, pack in (it closes the slots), plug into its pocket, set screws; XT60E1-M into the receiver pocket, lid on with the lead-end mount bolts.

## Checks (C unless tagged, `result.json`)
- **solid_check PASS** on all 10 print files (1 body, 1 eroded body, 0 necks, 0 thin walls); print_orient: one first-layer island each, no thin island run.
- **Insertion:** sliders free, from 50 below seat every 1.0, and 0.3 off in x: 0 mm³ cartridge vs receiver, no stuck step (the feet cam the bolts in). Gripped pull-out (sliders squeezed) 0 mm³. Squeeze 0..4.0: 0 mm³ against cartridge, receiver and each other. Bolts bear after a 0.8 drop.
- **FE (receiver, h 1.0):** Scout crash worst 0.36, handling 0.67; Overseer crash worst 0.44, handling 0.73; flight and swap ≤ 0.2.
- **Scout v3 system** (pass 1 snapshot, `runs/battery_latch/v1/scout_v3_pass1/`, XT60 end at the nose): System: Scout v3, cage first −4.2 mm (E) at (−0.28, +0.24, −0.93) (receiver) FAIL, fit 2193.6 mm³ (E) (pogo pads + carrier / receiver) FAIL, bbox 6.57 L (E), extents 233 x 233 x 122 mm (E), mass 689 g (E), CG 2.0 mm off thrust axis (E), 3.0 mm without swap parts FAIL (tol 3), probes −34.0 mm (camera, unshielded) FAIL. Collar planes 7.00, keel nodes 10.0 / 5.3 / 10.0 clear; with the 4th keel node stand-in the latch set clears cage first by 5.0. The FAILs are the Scout's: the missing keel node, its PR 44 pogo carrier and cage joints on the old receiver, its dry-mass balance, the camera probe at the nose.
- **Overseer** (`overseer.assembly.json` on main, cartridge and pack replaced): sled vs receiver 12544 mm³ (A) / 11300 (B), the sled's to cut.

## Handoffs (other sessions own these)
- **Swap arm v6** (`swap_system.py`): XT60 at the arm-side end (its pitch numbers above, carbon ties on the arm); grabber interface and forces in dimensions "System box swap: latch"; two analog halls on the head at (x ±7.5, y −1.0, z z_T + 8), one under each tab magnet, head x offset ±0.5 at the read; `latch_assembly(pack, "latched" | "gripped" | "free")` gives the meshes in cartridge coords; the y stroke is the same for both packs (flat bottom at r 100).
- **Scout v3:** rebalance the dry mass ~2 mm aft (the receiver's lid + XT60E1-M now at the nose: dry CG 3.0 off, the limit); a belly frame on the 4 mount bolts (drone (−41.85, ±25.6), (43.25, ±25.6), mount plane z 9.6) carrying the plate posts, pogo carrier, Radxa, flow and the cage's core joints; the 4th keel node at (−28, 28.5, −47.5): with a stand-in there the latch set clears cage first by 5.0 (`result.json` scout_system_keel4).
- **Overseer** (pack 30 behind the thrust axis, Spark: B): the sled cut round the receiver box + 1.0 with the 6-bolt mount plane; cage lower vs pack (324 mm³), feet and down camera vs receiver (~87 / ~81 mm³); the CG is the layout's (B's pack-out shift is Spark's accepted exception).
- **Box structure:** bay bodies give each receiver its mount plane (Scout r 167.1, Overseer r 180.1 at the bay, R 113 + Y_M) and the strip side; the bay receiver is the same print file.

## Open (E)
- Spring rate and the XT60 forces are E; the first latch print checks release force per finger (≤ 15 N), the snap-in at the seat, and that the pack drops 0.8 onto the bolts when let go.
- Hall part not chosen (threshold ~50 mT; a switch with B_op 15..60 mT or an analog hall).
