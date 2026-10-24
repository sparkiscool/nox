# Battery latch v1 + receiver v2: one latch for every pack (24 Oct 2026)

Spark 24 Oct: "design the new latch system for all batteries so the grabber can follow it". One latch + cartridge standard for every pack (Scout 4S 1500, Overseer 4S 6000), in the drone receiver and in every box bay, worked by the swap arm's grabber. Script: `nox/box/battery_latch.py` (parts, sweeps, hand calc, reactions, system checks, `latch_assembly(pack, state)` for `swap_system.py`); `nox/box/battery_latch_topo.py` (receiver FE on the L cases and the swap load set: `--swap-v1`, `--swap-v2`, the receiver topology setup); `nox/box/battery_latch_figure.py` (figures). Run: `runs/battery_latch/v2/` (receiver v2; `result.json` and `fe/` hold every number below; v1 in `runs/battery_latch/v1/`, superseded). Shared numbers: `nox/dimensions.md` "System box swap: latch". Layout artifact: https://claude.ai/artifact/MX2dpRcjHegntjHvqp6U8x. Owner: battery latch worker (control session's box swap system).

## Frames (C)
- **Cartridge frame c** (as the Scout bottom layout, PR 44): x across the pack, y up the install axis (0 = flat bottom = ring R 113 in a bay, arm v6), z along the pack (0 = closed end). The tab line z_T is the swap assembly's CG (pack + plug + cartridge + sliders), so the gripper holds the pack at its CG and the drone puts it on the thrust axis.
- **XT60 end = arm side (hub 24 Oct, from the arm v6 system pass, PR 71, DEFAULT):** the receiver gives no pitch guidance before its deck at the seat, so the XT60 mates on the arm's own pitch stiffness. Free pitch at the mate vs the float carrier's ±1.06°: XT60 at the far end 2.56 / 1.71 / 3.47° (Scout at Y_C 175 / Scout 129.1 / Overseer, R 113), at the arm-side end with carbon ties on the arm 1.03 / 0.52 / 0.70° (arm v6 numbers). So the whole latch set is turned 180° about the install axis: the cartridge's lead (XT60) end faces the arm (−Y); no geometry changed. Alt (not taken): receiver pitch guidance before the mate, which needs z-locating faces engaged over ~25 before the seat at both ends, still ~0.7° on 0.3 gaps.
- **Scout drone frame d** = (z − z_T, x, y − 44.5): tab line at drone x 0, XT60 end at the nose (+x), flat bottom at drone z −44.5.
- **Tab line:** z_T = swap CG + TAB_OFF toward the XT60 end. Overseer 5.18 (arm v7 PR 79: tab line and catch centre on the latch set's centre; grip moment 0.04 / 0.11 N·m at 1 / 3 g, index 0.04 / 0.11). Scout 0 (DEFAULT; the arm's 3.71 puts the pack's tail-bottom corner 6.4 inside the collar planes, where the cartridge can't wrap it at 7.0; the Scout set, 101.3 long, sits 3.7 off its catch centre inside the 191 opening).
- **Wheel W** (dimensions I, arm v7 PR 79): X = x, Y = Y_C − (z − z_T), Z radial = R + y, R 113 for every seated face, common grip Y_C 165. Arm-side ends at Y_C 165: Scout Y 111.6 (set-screw boss) .. 207.6, Overseer Y 76.5..250.5.
- **XT60 pitch (hub ask, arm PR 79; receiver v2):**
  - The XT60E1-M floats ±2.37° in pitch, about the cartridge's x (the arm's slide-tangential axis): 0.5 axial play over its 12.1 flange depth. The PR 44 ±1.06° is roll, over the 27.1 width.
  - **Overseer:** corner guide posts on both end walls, end gap 0.15. Pitch at the XT60's first contact is ±0.36° (C).
  - **Scout:** no posts (DEFAULT). The collar planes clip its posts to a 13 mm lever (~265 N a post at 3.5 N·m), and the float takes the arm's 1.33° free mate (C).

## Receiver v2: durability (24 Oct)
Spark (24 Oct, via the arm and the hub): "since now we need to unplug and plug in the battery we need to redesign the arm and the latches so they can handle the extra load" and "the latch system for the charging bays ... dosent seem to hold in repetitoon and seems to bend since its just thin plastic". This is receiver v2. The cartridge v5 and sliders v5 are unchanged.

**Requirements (hub DEFAULT from the arm, E):**
1. Loads on every swap: insert 60 N, ending at up to 74 N at head stall; pull 50 N (57 N with the grabber holding); pitch guide ≤ 3.5 N·m (6.1 N·m misuse); 43.5 N across; 7.6 N / 1.05 N·m centring.
2. Life ≥ 5,000 cycles per bay and ≥ 10,000 at the drone.
3. Cyclic stress ≤ ~10 MPa in every printed member at 74 N + 3.5 N·m. Printed walls ≥ 3 at loaded faces.
4. Hold and spring from bought steel. Wear faces replaceable or metal-backed.
5. Deflection ≤ 0.1 mm at 74 N.
6. XT60 carrier replaceable if the XT60's life sets the service interval.
7. A section of v2 beside v1 under the 74 N case.

**Load set in the FE** (`battery_latch_topo.SWAP`, `add_swap_cases`). Every swap case is class flight, SF 4: allowables 10.8 MPa in-layer and 5.4 MPa across layers, which is the ~10 MPa fatigue line. The 6.1 N·m case is misuse.

| Case | Load |
|---|---|
| 74 N insert | +y on the XT60 seat, or on the TPU stops |
| 57 N pull | −y on the XT60 lips, or on the pocket floors (bolts in, a fault) |
| Guide 3.5 / 6.1 N·m | z couple on the end faces: the top band of one end against the bottom band of the other, from each face's real extent |
| 74 N + 3.5 N·m | both together |
| 43.5 N across | x on one side's feet, both senses |
| Centring | 7.6 N on one side's lead-ins + a 1.05 N·m yaw couple |

Supports: the boss seats. For the +y cases (v2) the mount face bears on the partner.

**v1 on that set (FE h 1.0, `--swap-v1`, M mesh, E loads):** guide 3.5 N·m index 3.59 (Scout) / 1.72 (Overseer); 43.5 N across 2.02 / 1.53, the feet move 5.3 / 4.4 mm; centring 1.09 / 0.81. Everything else ≤ 0.45. Spark's "bends" is the open frame: windowed 2.5 / 3.0 plates on 4.0 feet, and no guide faces below the rails.

**Concepts weighed:**
| | **A: closed channel + replaceable XT60 carrier (chosen)** | B: v1 frame stiffened | C: steel strike pins in the pockets | D: steel leaf / ball plungers in the receiver |
|---|---|---|---|---|
| How | Side walls 5.0 from the feet to 1.0 under the mount plane (Overseer: truss windows between the stations); full deck; end walls 5.0 (Overseer: closed end full width down to its guide posts' bottom, lead end posts + 6 gussets round the XT60 window; Scout: no posts); cross ribs at the tab line and the carrier. The XT60E1-M float pocket leaves the block: an XT60 carrier base (pull lips + ears) + cap (the v1 lid), clamped by the two lead-end M3 bolts. Printed deck down. | Plates 4.0 with truss windows, guide posts, pockets and XT60 pocket in the block, lid | A, plus a Ø3 dowel pin as each pocket floor | Hold in the receiver by bought springs, the cartridge sliders gone |
| Steel in the hold | the pen spring (steel); bolts from below; XT60 pull goes lips → carrier → body boss (compression) → bolt head → partner insert | as v1 | pins on the bolt | leaf / plungers |
| Wear parts | sliders (3 to 4 g), XT60 carrier + XT60E1-M + pigtail, TPU strips; the body (bolts from below) | the whole receiver | pins | springs |
| Fails | heavier (Scout set 140 g vs 91, Overseer 289 vs 186, M) | 43.5 N across at the feet (open bays); the XT60 not replaceable without the block | **no room**: the 30° lead-in sits 0.55 under the pocket floor at the mouth, so a pin breaks through the cam face unless the slider / grip numbers change (Spark's squeeze latch, locked 15 Oct) | changes the grabber interface; a ball plunger is a detent, not a lock, so it fails the crash hold |

**Chosen: A (DEFAULT).** B is lighter but leaves the open bays that bend. C and D do not fit the locked latch.

Wear on the printed pockets:
- 1.4 MPa plane contact (18 x 2.3) at the 57 N fault pull.
- At each swap a bolt cams in on the 30° foot at the pen spring's force (~0.7 N rest, ~15 N squeezed, E).
- PETG on PETG wear over 10,000 cycles at that force is small (E). The first coupon checks it.
- Sliders and the body are the replaceable wear parts (alternative: replaceable strike blocks at the tab line, +2 parts and +4 mm per side there, past the Overseer's 66.8).

**XT60 life (requirement 6):**
- Most Amass XT60 listings give 100 mating cycles (GB/T 5095.1). One Amass sheet gives 1,000, unconfirmed (E, CHECK the datasheet for the exact part).
- Either way the XT60 sets the service interval, far under 5,000 / 10,000.
- So the XT60 carrier with its XT60E1-M and a short pigtail (inline connector to the bay or drone harness) is a replaceable part in every bay and the drone. Replacing it: 6 M3 from below (cartridge out), receiver down, carrier swapped.
- Alternative: a carrier fed from below (2 screws, the receiver stays up), which needs counterbored heads under the plug zone. Next version if the swap count calls for it.
- The pack's own female XT60 sits in the cartridge (set screws) and ages with the pack.
- A higher-cycle connector is the hub's / Spark's call.

**Concept review (concept mode, 24 Oct): FAIL, 6 findings, all taken or answered:**
1. The v2 mesh had lost its cuts: pockets, lead-ins, bolt holes, carrier notch. That was an edit slip of mine. Restored. The FE, figures and files were rerun from the corrected mesh.
2. XT60 carrier:
   - The 0.2 gap under the ears is closed: the lead-end boss tops are at Y_DK0.
   - The 74 N insert now loads the cap's ceiling (Y_PCEIL), not the lips.
   - A carrier fed from below (2 screws, the receiver stays up) does not fit. The cartridge's plug zone sits 1.3 under the lips (Y_ZONE 41.8 vs LIP_Y0 43.1, Scout), so a screw head or ear has no room.
   - Service stays DEFAULT: cartridge out, 4 to 6 M3 from below, receiver down, carrier swapped. Retorque the clamp then (printed stack ~12.5).
3. Guide posts per pack. **Scout: none (DEFAULT).**
   - The collar planes clip the Scout's posts to a 13 mm lever, so each post takes ~265 N at 3.5 N·m on every swap.
   - Its XT60 float (±2.37°) takes the arm's free mate (1.33°).
   - The seat's couple goes to one end's stops against the bolts (lever 23 mm, ~152 N on the bolts, FE case).
   - END_GAP stays 0.3.
   - **Overseer: posts** (lever 47 mm, END_GAP 0.15), pitch at the XT60's first contact ±0.36° (C).
4. Requirement 4 (steel in the hold, metal-backed wear faces), **relaxed for the latch pockets, DEFAULT.**
   - The pocket floors see ~0.1 MPa in flight and 1.4 MPa at the 57 N fault pull (E).
   - At each swap a bolt cams in at the pen spring's 0.7 to 2.4 N (E). Steel pins have no room (concept C).
   - Steel in the latch: the pen spring and the M3 bolts. The XT60 pull goes lips → carrier → body boss → bolt head → partner insert.
   - Wear parts: sliders (3 to 4 g), TPU strips, XT60 carrier; the body comes off with its bolts from below.
   - Alternative: separate strike blocks at the tab line (the pocket block, rf..rf + 5.6, keyed into a wall notch, one M3 from below), if the first coupon shows wear.
5. Print orientation: deck down (mount plane on the bed) kept.
   - print_orient, Scout body: bed 1418 mm² deck down vs 257 feet down (M).
   - The pocket floors and the deck top between ribs print on tree supports.
   - A lower mount plane (Y_M = deck top) would make the deck the bed face, but it moves the interface every partner is built to (Scout mount, Overseer sled, bay bodies). Not taken; the owners can ask.
6. Lighter: the Overseer's side walls got truss windows between the stations.
   - A thinner wall fails the foot tip: the 3.0 lead-in capture leaves 1.0 on a 4.0 wall, a solid_check FAIL, so the walls are 5.0.
   - A topology pass on the v2 envelope is the follow-up.

**More found while fixing:**
- With v1's thin plates the Scout's bolt heads came up outside the plate. On the 5.0 wall an access channel would cut the closed-end corner (across 0.96). Each Scout bolt now runs in a full-height column (r 5) beside the wall. The head sits on a flat at y 14.0, above the collar clip, in a counterbore with 3.0 round it: M3 x 45 (E). The bolt pattern (±25.6) is unchanged.
- The body (not the bosses or the carrier cap) stops 1.0 under the mount plane: the Scout's FC sits 0.1 under it.
- **Overseer mount bolts (open, v1 too):** at x ±26.8 they come up from below through the cartridge's rail zone (rails x 26.5..27.5 to y 58.6), and their heads land inside the cartridge path.
  - Options, for the Overseer and box sessions: (a) bolts from above into heat-set inserts in deck bosses (needs 8.7 above the deck's underside, which it has); (b) outboard bosses, 76.2 wide (no longer 66.8).
  - The Overseer set stays DESIGN.

**v2 on the swap set (FE h 1.0, deck-down print, body + carrier + cap clamped, `--swap-v2`, M mesh, E loads):**
- Scout (walls 5.0, ends 5.0, no posts): every swap case ≤ 0.61. 43.5 N across 0.61, guide 3.5 N·m 0.56 (6.1 misuse 0.61), 74 N + 3.5 N·m 0.56, 57 N on the bolts 0.21. Every other swap case ≤ 0.07, crash ≤ 0.07 (15.3 g).
- In MPa: index ≤ 0.61 is ≤ 6.6 in-layer and ≤ 3.3 across layers (flight allowables 10.8 / 5.4).
- Scout deflection: 74 N insert 0.001 (XT60 seat) / 0.003 mm (stops); 57 N pull 0.015; guide 0.27 mm at the bolts; across 0.40 mm (was 5.3 on v1).
- Overseer (walls 5.0 solid, ends 5.0, posts; h 1.5: at 1.0 the iterative solve runs > 2 h):
  - Every swap case ≤ 0.46. 43.5 N across 0.46, guide 3.5 N·m 0.10 (6.1 misuse 0.11), 74 N + 3.5 N·m 0.10, centring 0.20; crash ≤ 0.39 (35 g).
  - Deflection: 74 N insert 0.001 mm, guide 0.015 mm, across 0.77 mm.
  - Truss windows (34 g lighter) moved 1.15 mm across, past the float; not taken.
  - A window between the lead-end posts first put the across peak on its corner (1.14). The lead end now runs full width with a slot for the set-screw boss.
- Pitch at the XT60's first contact: Scout > 4° (no posts; the float's ±2.37° takes the 1.33° mate); Overseer ±0.36° (C).
- Print orientation: closed-end face down (+z, as v1) put the posts' z pull across layers (guide 1.11), so deck down.

## Concept check (24 Oct, before the piece runs)
Requirements (the brief): hold every L case for both packs; XT60 mates on insert (60 N) and unmates on pull (50 N) on the install axis, XT60E1-M float carrier on the receiver; latched state readable; a half-in pack never reads or stays latched; engages by the insert stroke alone, the grip releases it, releasing the grip locks it; the same grip interface on both packs; no more complex than the v2 squeeze latch; Scout receiver + cartridge ≥ 7.0 inside the collar planes below z 20 with the keel nodes free; pack CG on the thrust axis; breaks after the cage.

| | A: v2 / v4 squeeze (PR 44, baseline) | **B: squeeze v5, one latch module (chosen)** | C: snap hooks on the receiver | D: one bolt + fixed lug | E: push-push (heart cam) |
|---|---|---|---|---|---|
| How | 2 sliders + 1 pen spring in the cartridge, passive pockets in the receiver, tabs in a bottom recess | A's mechanism; one tab / recess / bolt / pocket module (same numbers on every cartridge), bolt overlap 2.3, undercut 15°, tab line on the swap CG, receiver as a frame (top band + 3 legs a side), ends clipped to the collar planes, magnet in each tab | 2 PETG cantilever hooks on the receiver snap over cartridge ledges; fingers spread them from outside the cartridge | 1 slider at the lead end, a fixed lug at the closed end; squeeze one tab against a fixed face | insert latches, a second push over-travel releases |
| Printed / moving parts | 7 / 2 | 7 / 2 | 5 / 0 (2 flexures) | 6 / 1 | 6 / 2 |
| Hold, 35 g crash | Scout 0.07 (E, hand calc; not sized for the Overseer's 257 N, E) | FE crash worst Scout 0.36 (M mesh, E loads, at 35 g; 0.16 at the suspension's 15.3 g), Overseer 0.56 (M / E) | > 1 (E): a hook the 15 N finger can spread holds ~5x that, not 257 N | 2x B per bolt (E), pack pivots on the lug in a side hit (E 0.5) | cam pin in a printed track, ~1 (E) |
| Grabber stroke / force | 4.0 per finger (C), spring ≥ 15 N, release ~2.5 N (E) | **4.0 per finger, tabs x ±4..10 → 0..6 (C), ≥ 15 N spring, release 3.0 (Scout) / 4.5 N (Overseer) (E)** | new fingers outside the cartridge, ~3 stroke, ~20 N (E) | 4.0 (C), one finger braces | none (stroke only), but the grip cannot release it (brief fails) |
| ±1.0 head offset | **FAIL**: 3.0 overlap needs 3.3 of retract, a 1.0 offset leaves one bolt 3.0 (C) | PASS: the short bolt retracts 3.0, its tip 0.7 inside the face (C, swept, 0 mm³) | fingers must find the hooks behind the cage | as B | n/a |
| Scout collar planes (need 7.0) | 1.01 / 1.50 FAIL (C; led the cage by 16.4) | **7.00** (C) | ~6 (E; wider receiver for the hook arms) | as B | as B + 4 (E, cam track) |
| Print risk | low (PR 44 coupon) | low (same channel, 0.35 clearance) | high: PETG flexure creep under the 12 N preload, fatigue | low | high: 0.5 cam features |
| Mass, Scout cartridge + receiver | 122.3 g (M) | 91.0 g (M) | ~70 g (E) | ~80 g (E) | ~95 g (E) |

**Chosen: B (DEFAULT).** It is A's mechanism (equal parts count; Spark locked the squeeze latch 15 Oct) with the four things A lacks: the ±1.0 misalignment, the collar planes (7.0), the Overseer's 257 N, and one grip interface on both packs. C wins 2 parts and ~15 g but cannot hold 35 g with a finger-releasable hook and puts the fingers outside the cartridge, behind cage members; D saves a slider but halves the hold and lets the pack pivot; E fails "the grip releases it". A fails the misalignment and the collar planes.

**Review (concept mode, 24 Oct): FAIL, 6 findings, all taken:**
1. Spring rest force (0.68 N, E) under a slider's 35 g inertia (1.1 / 1.4 N, E) in a side hit. A stiffer o 3 spring goes solid in the 8.0 squeezed length (checked: k 0.5 needs ~29 coils of 0.4 wire, 11.6 solid), so the undercut went 10° → 15°: the 1 g hang's wedge + friction now adds to the spring, hold / inertia x1.27 (Scout) / x2.07 (Overseer) (E); the outboard bolt is pushed further in, so a side hit can only ever move one bolt for the pulse; with no hang (inverted, free fall) the spring alone is 0.8 N (E; x0.73 / x0.58 of the inertia), but then nothing pulls the pack out, and one bolt alone carries the corner case at index 0.08 / 0.28 (E). Spring now k 0.2, rest 0.8 N.
2. Overseer pull-out along a 163 span: the Overseer receiver gets 2 more bosses at the tab line (6 total); the receiver is now checked by voxel FE on the L cases (`--fe-v1`), which found and fixed two more weak spots: the end plates under ±z crash (end faces now relieved 0.5 in the middle so the cartridge bears at its corners, into the side frames; Overseer +z 1.56 → 0.44, M mesh, E loads) and the end legs under a 50 N handling push (legs 16 / 24 long).
3. Half-in: one hall per tab (x ±7.5, 1.0 under the flat bottom), searched and swept: latched ≥ 109 mT vs half in ≤ 16 at head offset ±0.5 (≥ 76 vs ≤ 38 at ±1.0) (C, E magnet model); the box seat check (both halls + XT60 current) gates the drone's release (dimensions "Latched state"). Re-review (concept mode): PASS, follow-ups taken (keel node 4 stand-in, hall sweep, free-fall side hit, Overseer budget slots, Overseer clash owners).
4. Overlap 2.5 → 2.3 (C); asymmetric squeeze swept at ±1.0: 0 mm³, pull-out clear, tip 0.7 inside the face (C).
5. "System box swap: latch" written, with reactions (E) and `budget_g`; v3 baseline 72.1 g (M) noted.
6. Overseer posed at B (pack 30 behind the thrust axis, Spark 24 Oct decision) and A; the balance numbers in `overseer.assembly.json` are that layout's, not the latch's.

## Pieces
| Piece | Material, count | Scout g (M) | Overseer g (M) | Print |
|---|---|---|---|---|
| `box_battery_cartridge_<pack>_v5` | PETG x 1 | 31.9 | 89.7 | closed end down, tree supports (174 tall for the Overseer) |
| `box_battery_skid_{closed,lead}_scout_v1` (Scout only) | TPU 95A, 1 each | 1.4 / 3.1 | – | outer (chamfer) face down; bonded on the cartridge's end chamfers (flexible contact adhesive), replaceable |
| `box_battery_slider_<pack>_v5` | PETG x 2 (one part, both sides) | 3.2 | 4.0 | bar top down (largest face, layers along the bar) |
| `box_battery_receiver_<pack>_v2` (latch block, drone and bays) | PETG x 1 | 86.9 | 218.2 (66.8 wide) | mount plane down (deck down), tree supports; the Overseer's 181.3 goes on the bed's diagonal (needs 175.5 of 180) |
| `box_battery_xt60_carrier_<pack>_v1` (base: pull lips + ears) | PETG x 1 | 4.6 | 5.0 | ear tops down |
| `box_battery_receiver_lid_<pack>_v1` (the carrier's cap) | PETG x 1 | 5.3 | 8.1 | top face down |
| `box_battery_receiver_strip_<pack>_v1` | TPU 95A x 2 | 1.0 | 2.1 | flat |

**Both sets: HOLD / DESIGN, do not print** (files only in `runs/battery_latch/v2/stl/`, previews beside them).
- Scout set: HOLD (PR 69 review). Its assembled probe check fails at the Scout's side windows, and its receiver is new.
- Overseer set: DESIGN (its mount bolts and system, below).

Per set: 141.6 g Scout, 333.2 g Overseer (M); v1 was 91.0 / 185.9. The grams buy the swap load set: v1's open frame moved 5.3 / 4.4 mm under 43.5 N across and reached index 3.6 / 1.7 in the guide case. v2 moves 0.40 / 0.77 mm, swap worst 0.61 / 0.46.

Bought per set:
- XT60E1-M with a short pigtail and an inline connector (the replaceable XT60 carrier).
- M3 bolts up from below into M3 x 5.7 heat-set inserts in the partner: Scout 4 x M3 x 45, head on its flat at y 14.0, lead-end pair through ear + cap. Overseer 6, open, see Handoffs.
- o 3 x 20 compression spring (0.3 wire, k ~0.2), 2 x o 2 x 2 N52 magnets, 2 x M3 x 5 cup-point set screws.

Assembly: sliders into the cartridge channels from inside, spring into its U-slots, pack in (it closes the slots), plug into its pocket, set screws; XT60E1-M into the carrier base, cap on, carrier into the receiver's notch, receiver up on its bolts.

## Checks (C unless tagged, `runs/battery_latch/v2/result.json`)
- **solid_check PASS on all 14 files** (Scout 8, Overseer 6; one line each in `summary.md`). print_orient: one first-layer island each.
- **Insertion:**
  - Sliders free, from 50 below seat every 1.0: 0 mm³ cartridge vs receiver, no stuck step.
  - 0.3 off in x: receiver 0 mm³. The rigid-modelled XT60 male meets the plug 10.6 mm³ in the last 4; its float takes ±1.0.
  - Gripped pull-out 0 mm³. Squeeze 0..4.0: 0 mm³. Asymmetric squeeze ±1.0: 0 mm³, pull-out clear.
- **FE, receiver v2** (body + carrier + cap clamped, the L cases and the swap set, M mesh, E loads), worst index per case group:

  | Pack | Swap | Crash | Handling 50 N | Flight |
  |---|---|---|---|---|
  | Scout | 0.61 | 0.07 (15.3 g) | 0.10 | 0.03 |
  | Overseer | 0.46 | 0.39 (35 g) | 0.07 | 0.07 |

  Every case and deflection is in `runs/battery_latch/v2/fe/` and the dimensions row.
- **First mode of the seated pack: 559 Hz (Scout) / 461 Hz (Overseer)** (E: rigid swap mass, partner rigid; ≥ 100 Hz target).
- **Sliders at the 74 N stall** (a bolt jammed on its lead-in, fault): 2.4 MPa, index 0.22. Grip pull 57 N + 1 g: tab bar 4.3 / 4.7 MPa, index 0.40 / 0.44 (E, hand calc).
- **Scout v3 system** (the refit snapshot `runs/battery_latch/v1/scout_v3_refit/`, current latch parts, skids as cage):
  - `System: Scout v3, cage first 4.5 mm (E) at (+0.00, +0.66, −0.75) (flow MTF-02P), fit 0.0 mm³ (E), bbox 6.9 L (E), extents 233 x 233 x 128 mm (E), mass 788 g (E), CG 1.5 mm off thrust axis (E), 2.1 mm without swap parts, probes −46.1 mm (E) edge at (−0.78, −0.39, +0.49) (receiver v1 (latch)) FAIL, access 7/7 clear`. The "receiver v1 (latch)" label is the snapshot's part name; the mesh is v2.
  - **The probe FAIL is the receiver reached from the side through the Scout cage's side windows.** The cartridge set and the receiver round the swap opening are exempt from the probe to 35° (DEFAULT; alternative: no exemption, which needs the receiver recessed ≥ 3 inside the cage's bottom plane). The exemption does not cover this edge, so the Scout set is HOLD until the Scout chat closes its side windows and a rerun reads probes PASS.
  - Collar planes 7.08 (clip inset 11.0; v2's 5.0 walls needed 0.5 more inset). Keel nodes 4.8 clear.
- **Assembled worst index per case (Scout, one set of numbers everywhere):**
  - The Scout frame model gives every flight / crash / landing case ≤ 0.98 (F).
  - The latch parts under them, receiver v2 FE: crash ≤ 0.07 at 15.3 g (0.08 at the suspension's ~16.7 g, scaled), flight 0.03, landing (catch 6 g) 0.05, handling 0.10.
  - The swap set ≤ 0.61. Sliders ≤ 0.40, grip pull. Cartridge floor 0.14 at ~16.7 g (E, hand calc scaled).
- **Overseer** (`overseer.assembly.json` on main, pose B, cartridge, pack and receiver replaced): `System: Overseer, cage first −59.0 mm (E) at (+0.91, −0.01, −0.41) (Unitree L2) FAIL, fit 27630.3 mm³ (E) (belly sled + pods / receiver) FAIL, bbox 85.7 L (E), extents 634 x 570 x 237 mm (E), mass 2638 g (E), CG 10.6 mm off thrust axis (E), 26.7 mm without swap parts FAIL (tol 3), probes −0.0 mm (E) edge at (+0.71, −0.06, +0.70) (Unitree L2, unshielded) FAIL`.
  - The sled was cut for the v1 receiver. The Overseer set is 66.8 x 68.8 x 181.3 (0.83 L, M).

## Handoffs (other sessions own these)
- **Swap arm** (`swap_system.py`):
  - XT60 at the arm-side end. Scout: no guide posts, the float's ±2.37° takes the mate, and the seat's couple goes to the stops and bolts. Overseer: posts, ±0.36° at first contact.
  - Scout TAB_OFF 0 (not 3.71, above). Grabber interface and forces in dimensions "System box swap: latch".
  - Two analog halls on the head at (x ±7.5, y −1.0, z z_T + 8), head x offset ±0.5 at the read.
  - `latch_assembly(pack, "latched" | "gripped" | "free")` now also returns `xt60_carrier`.
  - The swap load set the arm gave (74 / 57 N, 3.5 / 6.1 N·m, 43.5 N, 7.6 N / 1.05 N·m) is what v2 is checked on.
- **Every receiver partner (Scout mount, Overseer sled, bay bodies):**
  - M3 x 5.7 heat-set inserts at the boss positions (unchanged), bolts up from below.
  - The mount plane is unchanged (Scout 54.1, Overseer 68.8). The body's top bears on it except the Scout's 1.0 FC relief.
  - XT60 service (requirement 6): the Amass XT60 lists ~100 mating cycles (one sheet 1,000, E). Each bay and the drone replace the XT60 carrier with its XT60E1-M and pigtail on that interval: cartridge out, receiver bolts out, carrier swapped.
  - A higher-cycle connector is the hub's / Spark's call.
- **Scout v3:**
  - The mount takes 4 x M3 x 45 from below (heads on the receiver's flats at y 14.0, reached with the cartridge out).
  - Close the side windows so the edge probe reads PASS (the set stays HOLD until then).
  - The skids are swap parts (cage role) in its assembly.
- **Overseer (open, v1 too):** the receiver's mount bolts at x ±26.8 come up from below through the cartridge's rail zone (rails x 26.5..27.5 to y 58.6), so their heads would sit in the cartridge path. Options:
  - (a) Bolts from above (through the sled) into heat-set inserts in the receiver's deck bosses: 8.7 above the deck's underside, room for a 6.7 insert and a 2.0 floor.
  - (b) Outboard bosses from below, 76.2 wide.
  - Plus: the sled cut round the v2 receiver (fit 27630 mm³ now), and its 6-bolt pattern.
- **Catch:** funnel v8 opening ≥ 190.7 along the drone fore-aft for the Overseer set (dimensions, row "Overseer vs the funnel opening"). The set is 1.7 longer (181.3).
- **Box structure:** bay bodies give each receiver its mount plane (Scout r 167.1, Overseer r 181.8 at the bay, R 113 + Y_M) and the strip side. The bay receiver is the same print file. The Scout receiver is 63.6 wide (was 61.2).

## Receiver topology attempt (24 Oct)
`battery_latch_topo.py` keeps the receiver's must-haves drawn (locating pads, feet, pocket blocks, end pads, stop seats, XT60 pocket, bosses) and generates the rest on the L cases (Scout crash 15.3 g). Scout run (branch `run/battery_latch_receiver-scout-v1`, h 1.0, 7 cm³, 6.1 GB peak, M): 34.8 g (M) (v1 41.2, M) but **solid_check FAIL** (eroded necks 1.8 / 2.4, a 1.5 wall at (−0.1, 45.2, 94.3), M), worst index 25.3 (M mesh, E loads; 50 N on a foot), crash 7.4; the beta 32 stage ended on a tie-case violation: the 4.0 end pads (the 180 bed holds the Overseer to 4.0) and the small stations split once eroded. It does not beat v1 (rule 6), so **the drawn, FE-checked v1 receivers stay**; the Overseer run on the same setup was stood down (memory: 6.3 GB at h 1.4, 1.65x the engine's estimate, M). Next setup (follow-up): fewer, thicker drawn stations (≥ 6 everywhere, end pads merged into the side legs), generated volume ≥ 12 cm³ (Scout), tie weights checked on a coarse run first.

## Open (E)
- Spring rate and the XT60 forces are E; the first latch print checks release force per finger (≤ 15 N), the snap-in at the seat, and that the pack drops 0.8 onto the bolts when let go.
- Hall part not chosen: an analog hall (DRV5055A4-class, ±169 mT, E): 157 mT latched, 1.7 half in, 3.1 squeezed at nominal; threshold ~62 mT at ±0.5 head offset.
- The swap load set is E (the arm's). The first receiver coupon checks the stall, the pocket wear over cycles (strike blocks if it wears), and the across stiffness (0.40 / 0.77 mm at 43.5 N).
- Receiver v2 topology (lighter at the same cases) is a follow-up. The FE peaks sit at the end walls' corners; the side walls carry little except across.
