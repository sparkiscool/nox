# S2 slider: concept sweep (13 Oct 2026, slider chat)

**Read section 4 first: the simplify pass (Spark 17:54) replaced the dog, the keep-alive pads and the diodes of sections 1 to 3 with break-before-make strips; the built design is S2 slider v3 / cartridge v3 (`nox/v2/s2-slider.md`).** Full research reports beside this file: `2026-10-13-patents-swap.md`, `2026-10-13-patents-latch-contacts.md`, `2026-10-13-swap-mechanisms.md`.

Goal (Spark 13 Oct 14:48): the best-functioning slider that reads on no active US patent; every feature's position has
a stated reason (Spark 14:59). Research: `patents_swap.md`, `patents_latch_contacts.md`, `mechanisms.md`.
Lay patent screen, not legal advice. Numbers E unless tagged.

## 1. Architecture: every combination, and why each was kept or dropped

### A. How the old cartridge leaves
| Option | Patent (E) | Function | Verdict |
|---|---|---|---|
| A1 fresh cartridge pushes the old one out (v2 as built) | **Sony US12049333 claim 1, HIGH** (fresh battery directly impacts the old one, impulse releases it out the opposite opening) | one stroke, simplest | **dropped: patent** |
| A1b same with a spacer block between the packs | Sony by equivalents; "directly" was added in prosecution, so probably clear, but the box loses or recirculates a spacer | adds a loose part per swap | dropped |
| A2 **dog on the pusher sled**: the box lowers a dog into the gap behind the seated cartridge, the dog pushes the old cartridge out, the fresh one follows 5 behind and never touches it | none found (fork / rod ejector screened: no active claim) | one actuator, one stroke; push applied low on the old tail corners (no pitch-up, no binding); the dog's skids lift the tail lock first, so the release does not depend on cartridge tolerances (mechanisms §2) | **kept** |
| A3 puller on the nose side hooks the old nose while the pusher feeds the fresh one | Coretronic US11749860 (latch-releasing carrier + catching hook) LOW if no hook; steel plate on every cartridge for an electromagnet | pulling self-aligns, but a second carriage across the seat, a magnet per cartridge, and hardware in the old-pack chute | dropped: more box parts for no drone-side gain |
| A4 two-phase (eject, then insert) | clear | ~2x stroke time, same hardware as A3 | dropped: slower |
| A5 spring ejector on the drone | DJI US11296384 (sprung ejector) LOW | +mass on the drone, the spring is loaded by every insertion | dropped: mass on the flying side |
| A6 old cartridge exits down or sideways | Sony claim 17 still reachable; through the plate is impossible (FC, Radxa) | needs a hole in the frame | dropped |

### B. Keeping the drone powered during the stroke
| Option | Patent (E) | Function | Verdict |
|---|---|---|---|
| B1 two pin groups, packs make-before-break (v2) | DJI US10611252 claim 1 MEDIUM ("another battery onboard the UAV" powers it while the first is decoupled) | 16 pins, 2 diode channels, overlap window only a few mm | dropped: patent, more pins |
| B2 **box power through 2 pads on the slider**, a box head docks on them before the stroke; packs break-before-make; one pin group | LOW: box power is not an onboard battery; no claim found on pads reached by a docking head (Workhorse needs nest sidewalls, Modular Flight a switched multi-region pad, Brookhurst a two-flap gripper) | 8 pins instead of 16, shorter strips (~1.3 g less PCB per cartridge), the drone also stays powered and charged while parked; seat check for free (the pack conducts only when seated) | **kept** |
| B3 onboard supercap + boost | LOW-MEDIUM (a supercap may be argued equivalent to a "battery" until 2035) | +8 to 12 g on the drone (E) | dropped: mass and patent |
| B4 let it reboot | clear | 35 s reboot (FC + Radxa) | dropped: function |

### C. What lifts the tail lock (blocks +x)
| Option | Verdict |
|---|---|
| C1 fresh cartridge nose (v2) | dropped with A1 |
| C2 **dog skids** ride the T bumps from behind before the dog reaches the old tail; the fresh nose (5 behind) takes over the bumps as the dog moves on | **kept**: box-timed, works with A2 |
| C3 docking head holds T open for the whole stroke | needs a lever on the leaf reaching the head; the seat end stop then has to be on the sled. Dropped: more drone parts |
| C4 box servo finger | extra actuator. Dropped |

### D. Contacts (patent: pins on the drone, strips on the pack, no contacts on latches, pack stays dumb)
| Option | Verdict |
|---|---|
| D1 **Mill-Max 0854 lying in the slider walls, a row of 4 per pole at z 31.5** | **kept**: lowest wall (the lips already need a wall to z 28.5), pins in the plane of the cartridge flanges so the two sides' preloads cancel through the cartridge |
| D2 a column of 4 per pole | dropped with B1 (it only helped the make-before-break window); needs a wall to z ~46 |
| D3 vertical pins under the cartridge | dropped: through the plate into the FC / Radxa (Problem 13) |
| D4 leaf springs / blade and fork | bought parts are 0854s (Spark 12 Oct); fallback if the pins wear (Makita blade, lapsed) |
| D5 Omniball side-load pins | 3.5 A each: 12 pins per pole. Fallback only |

### E. Rails
| Option | Verdict |
|---|---|
| E1 **flange in short C-groove segments** | **kept**: 0.25 clearance, friction 0.5 design; segment positions swept (section 2) |
| E2 full-length C rails | heavier for the same hold; covered by the sweep (long segments) |
| E3 steel / carbon rods with half bushings in the cartridge | low friction, but +1 to 2 g and two more bought parts per drone; push force is not a problem (pusher 40 N with a 15 N limit) |
| E4 dovetail / V | friction x1.4. Dropped |
Steals: two-stage clearance (B&D US7005831, lapsed): 0.35 at the first segment the cartridge meets, 0.25 at the seat segment; 1.5 x 45 lead-ins on every groove mouth (mechanisms §5).

### F. Retention elements
Flexure tooth in a notch, leaf in TENSION under the crash load (a leaf in compression buckles at ~6 N, mechanisms §2): T tail lock blocks +x, N nose ratchet blocks -x. One printed part each, no separate spring (Autel US10224526 family needs a separate elastic piece), all latches on the slider and the cartridge passive (Autel). Magnets: 14 g for 60 N, dropped. Ball detent: 250 MPa on PETG, dropped. Bayonet / friction: dropped.

### G. Pack-side rules kept from the screen
No ID resistor / chip / thermistor through any contact (TECO US11787566). Flat cartridge end faces, no tapered nesting (Sony claim 17). Strips glued in, not slid in and snapped (Panasonic US12368207). No electronics in the pack (ETI US11411257).

## 2. Placement sweep (coarse topology runs, `s2v2/sweep.py`)
Each feature that can move gets swept; the reason recorded is the measured winner. Fixed features and why:
- Plate bolts: plate holes (Spark 12 Oct).
- Guard sockets: guard frame model + above the plate face (dimensions E, 13 Oct).
- T tail lock at the cartridge tail: the dog enters behind the seated tail, so the lock must sit where the dog skids reach it first.
- Keep-alive pads on top of the T root blocks: reached from above (the only side free of the tunnel, the dog path and the tail struts), the dock load goes straight down the block into the plate, poles 47 apart.
Swept: groove segments (position and length), pin group x, N ratchet x.

## 3. Box interface this slider needs (DEFAULT, for the box chat)
- **Dog**: a plastic block 8 long (x) with the cartridge's lower cross-section (flanges included, so it rides the grooves
  like a cartridge), relieved to |y| <= 17 over the pin band z 29 to 34 so it never touches the pins. It hangs from a tine
  through the open top of the tunnel. The box lowers it into the tunnel behind the seated cartridge with its front at
  x <= -50 (the T bumps are at -50 to -46), then the sled moves +x: the block rides the T bumps first (tail lock lifted),
  meets the old cartridge's tail corner pads at -42 and pushes it out through the nose into a fixed chute.
- **Fresh cartridge**: on the same sled, nose 2 behind the dog's back face (hard stop on the sled, so it never touches the
  old cartridge). Its nose takes over the T bumps 2 mm after the dog leaves them, so the tail lock stays lifted while the
  old cartridge's notch passes. It stops when its own notch meets the T tooth (pusher force limit 15 N); the dog is then
  ahead of it inside the nose of the tunnel and lifts straight out through the open top.
- **Keep-alive head**: comes down from above onto the two pads on top of the T root blocks, (-65, +-26.4, 31), B+ left,
  B- right, before the stroke and lifts after the fresh cartridge is seated. Box supply through a drone-side ideal diode
  (pads are dead in flight). Never through the seat, legs or guard.
- Old pack into a fixed chute; magazine still during the stroke; no carousel; no pack ID through contacts.

## 4. Simplification pass (Spark 13 Oct 17:54: delete everything that is not needed to work)
| Deleted | Why it was there | Why it can go (check) |
|---|---|---|
| Keep-alive pads, keep-alive head, box dog | Sony design-around + powering the drone through a break-before-make swap | B+ strips end 5 from the tail and 13.2 from the nose: the old and fresh B+ strips are 18.2 apart with the cartridges touching, more than the 16.2 pin span, so the packs never connect; the 1.9 mm of travel with no B+ contact is 3.8 ms at 0.5 m/s, 3.8 V of sag at ~1 A on the 1000 uF bus cap (fit.py). B- runs nearly full length: ground never breaks. |
| Diode board (2 x LM74700 + MOSFET), TVS | stop the packs paralleling | the packs can no longer touch the pins together, so nothing to block |
| Nose ratchet N (leaf, root block, tooth, notch) | block backward motion with the leaf in tension | the tail-lock tooth is square: backward load pulls its leaf in tension; forward load pushes the leaf end 0.3 onto the rear groove segment's tail face, a rigid stop |
Kept, each for a measured reason: 2 groove segments per side (widest base the leaf allows, sweep), 4 pins per pole (40 A bursts stay at <= 13 A per pin with one pin lost), 2 pawls (one per side: 60 N on one 0.8 x 2 tooth face would be 37 MPa in bearing, at the PETG crash limit; two halve it), 1 bus cap (rides the 3.8 ms gap).
Patents after the pass (lay, E): TECO now out on two elements (no ID; the two pawls sit at the same x, not staggered). DJI US10611252 out (no second battery, a bus capacitor is not a battery). Sony US12049333 claim 1 needs the battery held "away from a body of the UAV" (added in prosecution; the spec puts the bay on a column under the body); ours sits on top of the body: LOW-MEDIUM, attorney to confirm. If not cleared: a box-side dog between the packs carrying its own gold strips fed by the box through a box-side diode (the 8 pins then step old pack, dog, fresh pack); the drone does not change.
