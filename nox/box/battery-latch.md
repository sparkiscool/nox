# Battery latch v1 + receiver v2: one latch for every pack (24 Oct 2026)

Spark 24 Oct: "design the new latch system for all batteries so the grabber can follow it". One latch + cartridge standard for every pack (Scout 4S 1500, Overseer 4S 6000), in the drone receiver and in every box bay, worked by the swap arm's grabber. Script: `nox/box/battery_latch.py` (parts, sweeps, hand calc, reactions, system checks, `latch_assembly(pack, state)` for `swap_system.py`); `nox/box/battery_latch_topo.py` (receiver FE on the L cases and the swap load set: `--swap-v1`, `--swap-v2`, the receiver topology setup); `nox/box/battery_latch_figure.py` (figures). Run: `runs/battery_latch/v3/` (cartridge v6 + skids; `result.json` and `fe/` hold every number below; receiver v2 run `runs/battery_latch/v2/`, v1 `runs/battery_latch/v1/`, superseded). Shared numbers: `nox/dimensions.md` "System box swap: latch". Layout artifact: https://claude.ai/artifact/MX2dpRcjHegntjHvqp6U8x. Owner: battery latch worker (control session's box swap system).

## Frames (C)
- **Cartridge frame c** (as the Scout bottom layout, PR 44): x across the pack, y up the install axis (0 = flat bottom = ring R 113 in a bay, arm v6), z along the pack (0 = closed end). The tab line z_T is the swap assembly's CG (pack + plug + cartridge + sliders), so the gripper holds the pack at its CG and the drone puts it on the thrust axis.
- **XT60 end = arm side (hub 24 Oct, from the arm v6 system pass, PR 71, DEFAULT):** the receiver gives no pitch guidance before its deck at the seat, so the XT60 mates on the arm's own pitch stiffness. Free pitch at the mate vs the float carrier's ±1.06°: XT60 at the far end 2.56 / 1.71 / 3.47° (Scout at Y_C 175 / Scout 129.1 / Overseer, R 113), at the arm-side end with carbon ties on the arm 1.03 / 0.52 / 0.70° (arm v6 numbers). So the whole latch set is turned 180° about the install axis: the cartridge's lead (XT60) end faces the arm (−Y); no geometry changed. Alt (not taken): receiver pitch guidance before the mate, which needs z-locating faces engaged over ~25 before the seat at both ends, still ~0.7° on 0.3 gaps.
- **Scout drone frame d** = (z − z_T, x, y − 44.5): tab line at drone x 0, XT60 end at the nose (+x), flat bottom at drone z −44.5.
- **Tab line:** z_T = swap CG + TAB_OFF toward the XT60 end. Overseer 5.18 (arm v7 PR 79: tab line and catch centre on the latch set's centre; grip moment 0.04 / 0.11 N·m at 1 / 3 g, index 0.04 / 0.11). Scout 0 (DEFAULT; the arm's 3.71 puts the pack's tail-bottom corner 6.4 inside the collar planes, where the cartridge can't wrap it at 7.0; the Scout set, 101.3 long, sits 3.7 off its catch centre inside the 191 opening).
- **Wheel W** (dimensions I, arm v7 PR 79): X = x, Y = Y_C − (z − z_T), Z radial = R + y, R 113 for every seated face, common grip Y_C 165. Arm-side ends at Y_C 165: Scout Y 111.6 (set-screw boss) .. 207.6, Overseer Y 76.5..250.5.
- **XT60 pitch (hub ask, arm PR 79; receiver v2):** the XT60E1-M floats ±2.37° in pitch (about the cartridge's x: 0.5 axial play over its 12.1 flange; ±1.06° is roll, over the 27.1 width). The arm's free mate about x is 1.23° (Scout) / 1.03° (Overseer), inside it, so **no guide posts on either pack** (arm, 24 Oct). Pitch at the XT60's first contact > 4° (C). The x side faces (gap 0.3, the cartridge's full length) engage before the XT60's first contact and hold its roll to 0.40° (arm).

## Receiver v2: durability (24 Oct)
Spark (24 Oct, via the arm): "since now we need to unplug and plug in the battery we need to redesign the arm and the latches so they can handle the extra load", and "the latch system for the charging bays ... dosent seem to hold in repetitoon and seems to bend since its just thin plastic". This is receiver v2. The cartridge v5 and sliders v5 are unchanged.

**Requirements (hub DEFAULT from the arm, E):**
1. Loads on every swap: insert 60 N, ending at up to 74 N at head stall; pull 50 N (57 N with the grabber holding); 43.5 N across; 7.6 N / 1.05 N·m centring.
   - The arm later dropped the 3.5 / 6.1 N·m pitch guide: about the cartridge x its free mate is 1.23 / 1.03°, inside the float's ±2.37°. So no guide posts and no guide couple.
   - It added: the x side faces engage before the XT60's first contact, gap ≤ 0.3. Here 0.3, engaged over the whole cartridge length.
2. Life ≥ 5,000 insert / pull cycles per bay and ≥ 10,000 at the drone.
3. Cyclic stress ≤ ~10 MPa in printed members. Printed walls ≥ 3 at loaded faces.
4. Hold and spring from bought steel. Wear faces replaceable or metal-backed.
5. Deflection ≤ 0.1 mm at 74 N.
6. XT60 carrier replaceable if the XT60's life sets the service interval.
7. A section of v2 beside v1 under the 74 N case.

**FE load set** (`battery_latch_topo.SWAP`, `add_swap_cases`). Every swap case is class flight, SF 4: 10.8 MPa in-layer, 5.4 MPa across layers, which is the fatigue line.

| Case | Load |
|---|---|
| 74 N insert | +y on the cap's float ceiling, or on the TPU stops |
| 57 N pull | −y on the carrier's lips, or on the pocket floors (bolts in, a fault) |
| 74 N + 43.5 N | both together |
| 43.5 N across | x on one side's feet, both senses |
| Centring | 7.6 N on the lead-ins + a 1.05 N·m yaw couple |

Supports: the bolt seats, and for +y the mount face bearing on the partner. The guide cases stay in the script behind `SWAP_GUIDE`.

**v1 on that set** (FE h 1.0, `--swap-v1`, M mesh, E loads):
- 43.5 N across: index 2.08 (Scout) / 1.53 (Overseer). The feet move 5.4 / 4.4 mm.
- 74 N + across: 2.05 / 1.53. 74 N insert alone: 0.30 / 0.32.
- Spark's "bends" is the open frame: windowed 2.5 / 3.0 plates on 4.0 feet.

**Concepts weighed:**
| | **A: closed channel + replaceable XT60 carrier (chosen)** | B: v1 frame stiffened | C: steel strike pins in the pockets | D: steel leaf / ball plungers in the receiver |
|---|---|---|---|---|
| How | Side walls 5.0 from the feet to the mount plane, no windows; full deck; end walls 6.0 to the mount plane, below the rails set back 3.3 from the cartridge end down to the feet; cross ribs at the tab line and the carrier. The XT60E1-M float pocket leaves the block: an XT60 carrier base (pull lips + ears) + cap (the v1 lid), clamped by the two lead-end M3 bolts. Printed deck down. | Plates 4.0 with truss windows, the XT60 pocket in the block, lid | A, plus a Ø3 dowel pin as each pocket floor | Hold in the receiver by bought springs, the cartridge sliders gone |
| Fails | heavier: set 147.7 / 331.3 g vs 91.0 / 185.9 (M) | 43.5 N across at the feet (open bays); the XT60 not replaceable without the block | **no room**: the 30° lead-in sits 0.55 under the pocket floor at the mouth, so a pin breaks the cam face unless the slider / grip numbers change (Spark's squeeze latch, locked 15 Oct) | changes the grabber interface; a ball plunger is a detent, not a lock, so it fails the crash hold |

**Chosen: A (DEFAULT).**

**Concept review (concept mode, 24 Oct): FAIL, 6 findings, all taken or answered:**
1. The v2 mesh had lost its cuts (pockets, lead-ins, bolt holes, carrier notch). That was an edit slip of mine. Restored; FE, figures and files rerun from the corrected mesh.
2. XT60 carrier:
   - The 0.2 gap under the ears is closed: the lead-end boss tops are at Y_DK0.
   - The 74 N insert loads the cap's ceiling, not the lips.
   - A carrier fed from below (receiver stays up) has no room: the cartridge's plug zone sits 1.3 under the lips (Scout).
   - Service DEFAULT: cartridge out, receiver bolts out, carrier swapped. Retorque the printed clamp stack (~12.5) then.
3. Guide posts: dropped on both packs (the arm's answer above). END_GAP 0.3.
4. Requirement 4 is **relaxed for the latch pockets (DEFAULT)**:
   - The floors see ~0.1 MPa in flight and 1.4 MPa at the 57 N fault pull (E). At each swap a bolt cams in at the pen spring's 0.7 to 2.4 N (E).
   - Steel pins have no room (C).
   - Steel in the latch: the pen spring and the M3 bolts. The XT60 pull goes lips → carrier → body boss → bolt head → partner insert.
   - Wear parts: sliders (3 to 4 g), TPU strips, XT60 carrier; the body comes off with its bolts.
   - Alternative: separate strike blocks at the tab line (the pocket block, keyed into a wall notch, one M3 from below), if the first coupon shows wear.
5. Print orientation: deck down (mount plane on the bed) kept.
   - print_orient `--all`, Scout body: deck down 1318 mm², end down 1823 mm² (105.6 tall), feet down 257 (M). Deck down is kept, though it is not the largest face: it is the orientation the FE checked (layers across the install axis). End down is the closed-end-down (+z) layer set that put a z pull across layers in the earlier guide case (1.11, M mesh, E loads); without the guide cases it is unchecked.
   - The pocket floors and the deck top between ribs print on tree supports.
   - Lowering the mount plane to the deck top would move the interface every partner is built to. Not taken.
6. Lighter: truss windows in the Overseer's walls save 34 g but move 1.15 mm across, past the float. Not taken; a v2 topology pass is the follow-up.
   - The walls are 5.0 because the 3.0 lead-in capture leaves 1.0 of foot tip on a 4.0 wall (solid_check FAIL).

**More found while fixing:**
- **Scout bolts.** On the 5.0 wall a head access channel cut the closed-end corner (across 0.96, M mesh, E loads). Each Scout bolt now runs in a full-height column (r 5) beside the wall, the head on a flat at y 14.0 above the collar clip, with 3.0 round its counterbore: M3 x 45 (E). The bolt pattern ±25.6 (C) is unchanged. Widening to ±28.6 (C) instead hit the cage rods (29 mm³, M).
- **Scout FC.** The FC sits 0.1 under the mount plane, so the body has a 1.0 relief there (tab line −5..+9). The rest of the top bears on the partner and is the bed face.
- **End walls.** Ending at the rails without posts left the Overseer's 43.5 N across case at 1.20 mm, index 0.92 (M mesh, E loads), at that corner. They now run to the feet, set back 3.3 below the rails (6.0 thick), with a slot for the cartridge's set-screw boss at the lead end. Pitch at the XT60's first contact stays > 4° (C).
- **Overseer mount bolts (open, v1 too).** At x ±26.8 / 27.8 they come up from below through the cartridge's rail zone (rails x 26.5..27.5 to y 58.6), so their heads would sit in the cartridge path. Options for the Overseer and box sessions:
  - (a) Bolts from above into heat-set inserts in deck bosses: 8.7 above the deck's underside fits a 6.7 insert and a 2.0 floor.
  - (b) Outboard bosses, 76.2 wide.
  - The Overseer set stays DESIGN.
- **Probe exemption** for the swap set round the opening: 40°, matching the Scout's system_check (DEFAULT; was 35. Alternative: none, the receiver ≥ 3 inside the cage's bottom plane).

**v2 on the swap set** (FE, deck-down print, body + carrier + cap clamped, `--swap-v2`, M mesh, E loads):

| | Scout (h 1.0) | Overseer (h 1.5; h 1.0 runs > 2 h) |
|---|---|---|
| Worst swap index | 0.51, 43.5 N across: 4.6 MPa in-layer, 2.8 across layers | 0.55, 43.5 N across: 2.7 / 3.0 MPa |
| 74 N + 43.5 N | 0.41 | 0.45 |
| Centring | 0.24 | 0.25 |
| 57 N on the bolts | 0.21 | 0.12 |
| Every other swap case | ≤ 0.07 | ≤ 0.06 |
| Crash | ≤ 0.07 (15.3 g) | ≤ 0.39 (35 g) |
| Handling | 0.12 | 0.07 |
| Deflection, 74 N insert | 0.001 mm (seat) / 0.003 (stops) | 0.001 / 0.004 mm |
| Deflection, 57 N pull | 0.014 mm | 0.016 mm |
| Deflection, 43.5 N across | 0.35 mm (v1 5.4) | 0.78 mm (v1 4.4) |

All inside the XT60 float's ±1.0.

**XT60 life (requirement 6):**
- Most Amass XT60 listings give 100 mating cycles (GB/T 5095.1). One Amass sheet gives 1,000, unconfirmed (E, CHECK the datasheet).
- The XT60 sets the service interval, far under 5,000 / 10,000.
- So the XT60 carrier with its XT60E1-M and a short pigtail (inline connector to the bay or drone harness) is a replaceable part in every bay and the drone.
- The pack's own female XT60 (cartridge, set screws) ages with the pack.
- A higher-cycle connector is the hub's / Spark's call.

Visual: runs/battery_latch/v2/figs/compare_v1_v2.png and fe_swap_insert_74_N_+_across_43.5_N.png (v1 beside v2 under 74 N + 43.5 N, one colour scale), in the layout artifact.

## Cartridge v6 and the Scout skids (run v3, 24 Oct)
Follow-up to receiver v2. The receiver body is unchanged. Run `runs/battery_latch/v3/` (superseded by v4 below: its receiver skids and strips are replaced; the cartridge v6 parts carry on).

**Cartridge v6 + slider v6: a centre rib** (heads v7 ask, DEFAULT):
- Why: gripped, the v5 tabs met each other, so the cartridge could slide ±4 on the slider bars against the head, with only friction holding it.
- Rib: x ±2.0 (4.0; 3.0 necked in the gate's erosion), y 0..11.5 along the recess, notched to y 7.9 over z_T ± 2.5 for the pen spring (C).
- It is fixed between two 6.0 cross walls past the recess (z_T ± 11.5..17.5), frame to frame under the pack (C).
- Each tab stops on it. The tabs sit at rest at ±5.0 / ±10.0 and squeeze to ±2.0 / ±7.0: stroke 3.0 (C).
- The spring slots are 3.0 deep, so the gap is still 16 at rest and 10 squeezed: 0.8 / 2.0 N (E).
- Both bolts retract 3.0 at any head offset (need 2.6, C). The squeezed tabs clamp the cartridge to the head.
- The rib now carries the head's x load. Hand calc (E): index 0.25 at the 43.5 N across (2.7 MPa), 0.04 / 0.13 at a 3 g carry (Scout / Overseer), 0.05 / 0.16 at 6 g misuse.
- Release 2.6 / 4.1 N per finger; grip pull 57 N + 1 g index 0.37 / 0.40; side-hit slider hold ×1.42 / ×2.27 (E). Hall squeezed 2.9 mT (E model).
- Heads v7 builds to the squeezed ±7.0; its blades only touch the outer faces.

**Scout TPU edge skids** (Scout v3 ask, drawn in its system pass as the requirement, cage role, bonded on). Drawn to the Scout's envelope:
- Cartridge strips (2): foot |x| 19.2..23.2, y −2.0..0, outboard of the wall bottom (X_OUT 19.0), held by a leg on the wall to y 3.0. Bond 192 mm² each, peel-loaded (E, DEFAULT; alternative: a dovetail rail).
- Receiver nose plate: ±27, 2.2 on the lead end wall.
- Receiver side skids (2): Ø3.5 round bar on the Scout's path + the strip |x| 24.6..27 under the feet's outer edge, one print, ending on the plate.
- The receiver skids are clipped at funnel inset 5.25, so they hold ≥ 1.0 inside the funnel planes at ±5° yaw / ±2° tilt: plate 1.16, side skids 1.86, cartridge strips 3.15 (C). Bolt key lines 13.5 / 5.58 / 10.1 clear (C). Every number is in `result.json` "skids".
- The receiver skids go on the drone receiver only; bay receivers have none (DEFAULT, the skids are cage). The cartridge strips go everywhere: heads v7 cut its deck and pads to |X| ≤ 18.2 with a keep-out over them.
- Insertion with every strip and skid: 0 mm³, no stuck step (C).
- My refit-snapshot run (3595381) reads fit 44.5 mm³: that old cage's rods against a side skid. That snapshot predates both the skids and the Scout's new layout (its own `scout.assembly.json` on its branch is still v4, no skids).
- So the Scout set stays HOLD until the Scout's own pass loads these exact meshes and reads fit 0 and probes PASS. The Scout manifest (`runs/battery_latch/v3/manifest_scout/`) carries every strip and skid, the cartridge end skids included (cage, swap); on its own collar-band stand-in it reads `fit 0.0 mm³ (E), mass 511 g (E), CG 1.4 / 2.6 mm (E), probes 2.9 mm (E)` (the stand-in band is relieved 0.5 round the skids, which are the cage there). The meshes went to the Scout session 24 Oct.

Sets: Scout 159.6 g (M), Overseer 338.5 g (M). The rib and its cross walls are +5.7 / +7.2 g on the cartridge. solid_check PASS on all 18 files (`runs/battery_latch/v3/summary.md`). No carbon rods: every piece is printed.

## Receiver v3 (Scout) and the end pieces (run v4, 24 Oct)
The Scout's system pass, with its keel back, asked for this (Scout v3 session); the PR 109 review added the head seats. Run `runs/battery_latch/v4/` supersedes v3.

Why the keel is back. I estimated a 0.81 J hit through 2 mm of TPU:
- about 4 kN peak on a flat floor (E: TPU 95A ~26 MPa linear, rigid floor and drone);
- about 0.7 to 1.0 kN from an edge on a Ø3.5 bar.

Receiver v2 at index 1.0 (misuse SF 2.5, scaled linearly from its FE rows, E) holds about:
- 3.2 kN up into the partner;
- 2.5 kN on an end;
- 1.0 kN from the side;
- 0.42 kN on one foot.

So 2 mm of TPU is a scrape layer, not a crash stroke. Holding the hit to the Scout suspension's 120 N plateau needs ~13.5 mm of linear travel (E). The Scout withdrew its no-keel DEFAULT: the hit goes through its suspension.

**Receiver v3** (Scout only; the Overseer stays v2):
- **Head seats:** v2's flats at y 14.0 sat outside the collar clip, so an M3 head bore on only 44–52 % of its annulus and the 3.0 ring round each counterbore was half gone (PR 109 review). v3 moves:
  - the flats to y 21.5 (drone z −23.0);
  - the Scout bolts to x ±26.55 (the ring clear of the cartridge path at 20.35);
  - the tail pair to z −0.1 (the ring inside the tail end wall).
  - Every seat now reads bearing 1.0 and ring 1.0 (`result.json` head_seats, C).
- **Bolts:** M3 x 40 from below, 7.4 past the mount plane (DEFAULT, E). The partner needs a hole ≥ 8 deep under each M3 x 5.7 insert.
- **Partner inserts** (drone x, y): (−42.65, ±26.55) and (43.25, ±26.55), was (−43.85 / 43.25, ±25.6). This is a Scout frame change, sent to the Scout.
- **Clip inset:** 11.5 (search: collar 7.45, need 7.0); 11.0 before the 26.55 bolts.
- **Under the keel:** |x| ≤ 25.5 below y 6.5 (drone z −38). The collar clip already held the columns inside 25.9 there; this shaves the pocket blocks' outer 0.4 for y 5.75..6.5 (wall 2.6 over that 0.75 band, C).
  - The keel's printed nodes (r 4 at drone (x, ±29.6, −47.5)) clear the receiver by 1.2, the end pieces by 5.6 and the cartridge strips by 2.5, nodes and rail line alike (need 1.0; `result.json` keel, C).
- **FE** on the same swap set (`fe/fe_scout_receiver_v2.json`, run v4; M mesh, E loads):
  - worst 0.415 (v2 0.51);
  - 43.5 N across: 0.34 mm;
  - 74 N: 0.001 mm;
  - crash ≤ 0.075, handling 0.123;
  - first mode 546 Hz.

**Receiver TPU end pieces** (drone receiver only, DEFAULT; one print each, bonded on). They replace the v3 nose plate and side skids.
- The receiver bottom strips are dropped (the keel is there).
- **Nose:** the 2.2 plate on the lead end wall face, plus 2 Ø3.5 bars along the 45° chamfer at drone y ±34, from (59.2, ±34, −16) to (35.2, ±34, −40). Ears at |y| 26..34.5 join plate and bars. Bonded 369 mm² (M).
- **Tail:** 2.2 on the tail end wall face and its chamfer, |y| ≤ 27, z −31.5..−6, with a Ø9 hole round each tail bolt key. Bars on the same line from the wall (−49.95, ±34, −25.25) to (−35.2, ±34, −40). Bonded 1197 mm² (M).
- **Bar placement:**
  - Each bar's axis sits 1.0 off the chamfer, so the bar has a flat where it crosses a bolt column.
  - On their own the bars touched the receiver only there (~6 mm), so they are joined to the plates.
  - y 34, not the Scout's 31: at 31 / 31.5 the bolt rings (r 6.2 at y 26.55) cut the bars to 0.5 skins.
  - The bars stop at z −40 (keel).
  - Below the tail plate's −31.5, the swap path and the key holes leave only slivers; the bars cover there.
- **Clearances:** clipped at funnel inset 6.25. Margins at ±5° yaw / ±2° tilt: nose 1.21, tail 2.24, cartridge strips 3.15 (need 1.0). Bolt key lines: nose 9.3, tail 4.5 (the Ø9 hole), cartridge strips 11.2 (C).
- Insertion with every piece: 0 mm³, no stuck step (C).
- The cartridge strips and end skids are unchanged (the cartridge ends follow the 11.5 inset).

**Sets:** Scout 165.3 g per drone set, 157.1 per bay set (M); v3 159.6. The grams buy the full head seats, the tail end piece and the 1.0 wider bolt pattern. Overseer 338.5 g (M). solid_check PASS on all 17 files (`runs/battery_latch/v4/summary.md`). No carbon rods: every piece is printed.

**HOLD:** the Scout set stays HOLD until the Scout's own pass (its cage, keel back, the new bolt pattern) reads fit 0 and probes PASS on these meshes.

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
| `box_battery_cartridge_<pack>_v6` (centre rib) | PETG x 1 | 37.9 | 97.6 | closed end down, tree supports (174 tall for the Overseer) |
| `box_battery_cart_strip_scout_v1` (Scout only) | TPU 95A x 2 (one part) | 1.1 | – | foot down; bonded under the cartridge's bottom long edges |
| `box_battery_rskid_nose_scout_v2`, `box_battery_rskid_tail_scout_v1` (Scout drone receiver only: plate + 2 bars each) | TPU 95A, 1 each | 3.2 / 5.0 | – | plate's outer face down, bars on tree supports; bonded on the receiver's end wall and chamfer |
| `box_battery_skid_{closed,lead}_scout_v1` (Scout only) | TPU 95A, 1 each | 1.4 / 3.1 | – | outer (chamfer) face down; bonded on the cartridge's end chamfers (flexible contact adhesive), replaceable |
| `box_battery_slider_<pack>_v6` (tabs at rest ±5.0, 3.0 spring slots) | PETG x 2 (one part, both sides) | 2.9 | 3.7 | bar top down (largest face, layers along the bar) |
| `box_battery_receiver_scout_v3` / `_overseer_v2` (latch block, drone and bays) | PETG x 1 | 94.4 | 216.2 (66.8 wide) | mount plane down (deck down), tree supports; the Overseer's 183.6 goes on the bed's diagonal (needs 177.1 of 180) |
| `box_battery_xt60_carrier_<pack>_v1` (base: pull lips + ears) | PETG x 1 | 4.6 | 5.0 | ear tops down |
| `box_battery_receiver_lid_<pack>_v1` (the carrier's cap) | PETG x 1 | 5.3 | 8.1 | top face down |
| `box_battery_receiver_strip_<pack>_v1` | TPU 95A x 2 | 1.0 | 2.1 | flat |

**Both sets: HOLD / DESIGN, do not print** (files only in `runs/battery_latch/v4/stl/`, previews beside them).
- Scout set: HOLD until the Scout's own pass (its cage, keel back) reads fit 0 and probes PASS on run v4.
- Overseer set: DESIGN (its mount bolts and system, below).

Per set (v4): 165.3 g Scout drone set (157.1 bay), 338.5 g Overseer (M); v3 159.6; receiver v2 run 147.7 / 331.3; v1 91.0 / 185.9 (M). The grams buy the swap load set: v1's open frame moved 5.4 / 4.4 mm under 43.5 N across, index 2.08 / 1.53. v2 moves 0.35 / 0.78 mm, swap worst 0.51 / 0.55 (all M mesh, E loads).

Bought per set:
- XT60E1-M with a short pigtail and an inline connector (the replaceable XT60 carrier).
- M3 bolts up from below into M3 x 5.7 heat-set inserts in the partner: Scout 4 x M3 x 40 (DEFAULT, E), head on its flat at y 21.5, lead-end pair through ear + cap. Overseer 6, open, see Handoffs.
- o 3 x 20 compression spring (0.3 wire, k ~0.2), 2 x o 2 x 2 N52 magnets, 2 x M3 x 5 cup-point set screws.

Assembly: sliders into the cartridge channels from inside, spring into its U-slots, pack in (it closes the slots), plug into its pocket, set screws; XT60E1-M into the carrier base, cap on, carrier into the receiver's notch, receiver up on its bolts.

## Checks (C unless tagged, `runs/battery_latch/v4/result.json`)
- **solid_check PASS on all 17 files** (run v4: Scout 11, Overseer 6; one line each in `summary.md`). print_orient: first-layer islands all over 20 mm² (receivers 3 Scout / 2 Overseer, the rest 1).
- **Insertion:**
  - Sliders free, from 50 below seat every 1.0: 0 mm³ cartridge vs receiver, no stuck step.
  - 0.3 off in x: receiver 0 mm³. The rigid-modelled XT60 male meets the plug 10.6 mm³ in the last 4; its float takes ±1.0.
  - Gripped pull-out 0 mm³. Squeeze 0..4.0: 0 mm³. Asymmetric squeeze ±1.0: 0 mm³, pull-out clear.
- **FE, receiver v2 / Scout v3** (body + carrier + cap clamped, the L cases and the swap set, M mesh, E loads), worst index per case group:

  | Pack | Swap | Crash | Handling 50 N | Flight |
  |---|---|---|---|---|
  | Scout (v3) | 0.415 | 0.075 (15.3 g) | 0.123 | 0.03 |
  | Overseer | 0.55 | 0.39 (35 g) | 0.07 | 0.07 |

  Every case and deflection is in `runs/battery_latch/v4/fe/` (Scout v3; the Overseer v2 file carried over) and the dimensions row.
- **First mode of the seated pack: 546 Hz (Scout) / 458 Hz (Overseer)** (E: rigid swap mass, partner rigid; ≥ 100 Hz target).
- **Sliders at the 74 N stall** (a bolt jammed on its lead-in, fault): 2.4 MPa, index 0.22. Grip pull 57 N + 1 g: tab bar 3.9 / 4.3 MPa, index 0.37 / 0.40 (E, hand calc, slider v6).
- **Scout v3 system:** the Scout's own pass decides (its cage, keel back, run v4 meshes). Two reference lines, neither the judge:
  - The latch manifest on its own collar-band stand-in (`runs/battery_latch/v4/manifest_scout/`): `System: battery_latch_scout, cage first 5.8 mm (E), fit 0.0 mm³ (E), bbox 3.75 L (E), extents 208 x 208 x 87 mm (E), mass 515 g (E), CG 0.9 mm off thrust axis (E), 1.8 mm without swap parts, probes 2.9 mm (E) edge at (−0.18, −0.20, −0.96) (cartridge)`.
  - My old refit snapshot (`runs/battery_latch/v1/scout_v3_refit/`; its cage predates the skids, the keel and the new bolt pattern): `System: Scout v3, cage first 4.5 mm (E), fit 2.6 mm³ (E) (mount + nose legs (Scout)/receiver skid nose (latch)) FAIL, mass 812 g (E), CG 1.3 / 1.8 mm (E), probes −27.9 mm (E) edge (receiver) FAIL, access 7/7 clear`.
  - The swap set round the swap opening is exempt to 40° (DEFAULT, as the Scout's own check; alternative: no exemption, which needs the receiver recessed ≥ 3 inside the cage's bottom plane).
  - Collar planes 7.45 (clip inset 11.5). Keel: receiver 1.2, end pieces 5.6, cartridge strips 2.5 (C).
  - **Scout set HOLD** until the Scout's pass reads fit 0 and probes PASS.
- **Assembled worst index per case (Scout, one set of numbers everywhere):**
  - The Scout frame model gives every flight / crash / landing case ≤ 0.98 (F).
  - The latch parts under them, receiver v3 FE: crash ≤ 0.07 at 15.3 g (0.08 at the suspension's ~16.7 g, scaled), flight 0.03, landing (catch 6 g) 0.05, handling 0.12.
  - The swap set ≤ 0.415. Sliders ≤ 0.37, grip pull; cartridge v6 rib 0.25. Cartridge floor 0.14 at ~16.7 g (E, hand calc scaled).
- **Overseer** (`overseer.assembly.json` on main, pose B, cartridge, pack and receiver replaced): `System: Overseer, cage first −59.0 mm (E) at (+0.91, −0.01, −0.41) (Unitree L2) FAIL, fit 27792.2 mm³ (E) (belly sled + pods / receiver) FAIL, bbox 85.7 L (E), extents 634 x 570 x 237 mm (E), mass 2636 g (E), CG 10.7 mm off thrust axis (E), 26.8 mm without swap parts FAIL (tol 3), probes −0.0 mm (E) edge at (+0.71, −0.06, +0.70) (Unitree L2, unshielded) FAIL`.
  - The sled was cut for the v1 receiver. The Overseer set is 66.8 x 68.8 x 183.6 (0.84 L, M).

## Handoffs (other sessions own these)
- **Swap arm** (`swap_system.py`):
  - XT60 at the arm-side end. No guide posts on either pack (your per-axis answer). The x side faces (gap 0.3, full length) engage before the XT60's first contact.
  - Scout TAB_OFF 0 (not 3.71, above). Grabber interface and forces in dimensions "System box swap: latch".
  - Two analog halls on the head at (x ±7.5, y −1.0, z z_T + 8), head x offset ±0.5 at the read.
  - `latch_assembly(pack, "latched" | "gripped" | "free")` now also returns `xt60_carrier`.
  - The swap load set the arm gave (74 / 57 N, 43.5 N, 7.6 N / 1.05 N·m, no guide couple) is what v2 is checked on.
  - Heads v7: cartridge v6 has the centre rib (x ±2.0). Tabs at rest ±5.0 / ±10.0, squeezed ±2.0 / ±7.0, stroke 3.0 (outer faces as agreed). Pocket, flat bottom and hall / magnet spots unchanged.
- **Every receiver partner (Scout mount, Overseer sled, bay bodies):**
  - M3 x 5.7 heat-set inserts at the boss positions (unchanged), bolts up from below.
  - The mount plane is unchanged (Scout 54.1, Overseer 68.8). The body's top bears on it except the Scout's 1.0 FC relief.
  - XT60 service (requirement 6): the Amass XT60 lists ~100 mating cycles (one sheet 1,000, E). Each bay and the drone replace the XT60 carrier with its XT60E1-M and pigtail on that interval: cartridge out, receiver bolts out, carrier swapped.
  - A higher-cycle connector is the hub's / Spark's call.
- **Scout v3:**
  - The mount takes 4 x M3 x 40 from below (heads on the receiver's flats at y 21.5, reached with the cartridge out) into inserts at drone (−42.65, ±26.55) and (43.25, ±26.55), each with a hole ≥ 8 deep (receiver v3; was (−43.85 / 43.25, ±25.6)).
  - Its tail-edge and keel asks (24 Oct) are drawn in run v4: receiver v3 narrowed under the keel, nose and tail end pieces with the bars at y ±31 above z −40, the bottom strips dropped. Its pass on the v4 meshes decides fit and probes; the set stays HOLD until then.
  - The skids are swap parts (cage role) in its assembly.
- **Overseer (open, v1 too):** the receiver's mount bolts at x ±26.8 (v2 ±27.8) come up from below through the cartridge's rail zone (rails x 26.5..27.5 to y 58.6), so their heads would sit in the cartridge path. Options:
  - (a) Bolts from above (through the sled) into heat-set inserts in the receiver's deck bosses: 8.7 above the deck's underside, room for a 6.7 insert and a 2.0 floor.
  - (b) Outboard bosses from below, 76.2 wide.
  - Plus: the sled cut round the v2 receiver (fit 27792 mm³ now), and its 6-bolt pattern.
- **Catch:** funnel v8 opening ≥ 190.7 along the drone fore-aft for the Overseer set (dimensions, row "Overseer vs the funnel opening"). The set is 4.0 longer (183.6).
- **Box structure:** bay bodies give each receiver its mount plane (Scout r 167.1, Overseer r 181.8 at the bay, R 113 + Y_M) and the strip side. The bay receiver is the same print file. The Scout receiver is 63.6 wide (was 61.2).

## Receiver topology attempt (24 Oct)
`battery_latch_topo.py` keeps the receiver's must-haves drawn (locating pads, feet, pocket blocks, end pads, stop seats, XT60 pocket, bosses) and generates the rest on the L cases (Scout crash 15.3 g). Scout run (branch `run/battery_latch_receiver-scout-v1`, h 1.0, 7 cm³, 6.1 GB peak, M): 34.8 g (M) (v1 41.2, M) but **solid_check FAIL** (eroded necks 1.8 / 2.4, a 1.5 wall at (−0.1, 45.2, 94.3), M), worst index 25.3 (M mesh, E loads; 50 N on a foot), crash 7.4; the beta 32 stage ended on a tie-case violation: the 4.0 end pads (the 180 bed holds the Overseer to 4.0) and the small stations split once eroded. It does not beat v1 (rule 6), so **the drawn, FE-checked v1 receivers stay**; the Overseer run on the same setup was stood down (memory: 6.3 GB at h 1.4, 1.65x the engine's estimate, M). Next setup (follow-up): fewer, thicker drawn stations (≥ 6 everywhere, end pads merged into the side legs), generated volume ≥ 12 cm³ (Scout), tie weights checked on a coarse run first.

## Open (E)
- Spring rate and the XT60 forces are E; the first latch print checks release force per finger (≤ 15 N), the snap-in at the seat, and that the pack drops 0.8 onto the bolts when let go.
- Hall part not chosen: an analog hall (DRV5055A4-class, ±169 mT, E): 157 mT latched, 1.7 half in, 2.9 squeezed at nominal; threshold ~62 mT at ±0.5 head offset.
- The swap load set is E (the arm's). The first receiver coupon checks the stall, the pocket wear over cycles (strike blocks if it wears), and the across stiffness (0.35 / 0.78 mm at 43.5 N).
- Cartridge v6 and the Scout skids: done in run v3 (above). The skids' crash loads into the receiver come from the Scout's run (E); handling 50 N at any skid point is the receiver's handling case (0.12). The Scout confirms fit and probes with these exact meshes.
- Receiver v2 topology (lighter at the same cases) is a follow-up. The FE peaks sit at the end walls' corners; the side walls carry little except across.
