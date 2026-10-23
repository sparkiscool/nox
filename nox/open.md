# Open problems and questions

Numbers are permanent (other docs cite them). A closed item leaves this page: its number goes on the Closed line at the bottom, its outcome in `nox/log.md`. Problems are session-owned: a session resolves them, no input from Spark. Questions are Spark's: each carries its best answer in brackets, and a session that cannot wait takes that answer as DEFAULT.

## Problems (session-owned)
5. **Holder group budget unknown** (cartridge + latch + camera + receiver). Fix: rewrite the budget once the Scout v3 bottom layout is drawn and weighed (P).
6. **Exposed live pads on spare packs.** Fix: snap-on printed cover (not drawn).
30. **The pack path through the funnel is blocked.** The cartridge comes straight down through the hub and the foot frame to the head gripper; the hub pocket (56.3 square) and the foot-frame window (37) are smaller than the cartridge (~80 x 45 E) plus its tabs (20 below). Fix: hub v4 opening and foot frame v3 window sized to the cartridge + 2 per side (the open fingers stay inside that outline), after the cartridge is drawn.
31. **Bottom layout** (pack + latch pockets + contacts + belly plate + pogo pads under the plate, flow sensor lowest). The Scout v2 receiver v3.4 drew it (`archive/scout-v2/s2-receiver.md`); Scout v3 redraws it against the same catch interface. The contacts chat places the shoes against it.
33. **Props on top (prop plane z ≈ +41.2, `dimensions.md` C "Motor / prop layout").** Rows in `dimensions.md` H that still say "below the props" are pusher-era until their owner updates them. Each chat rechecks its part against 41.2. The seated drone sits 0.7 lower in the collar (collar bottom edge z −0.1 in drone z); the catch and feet drop sims rerun in their chats.
41. **Catch v4 (`nox/box/catch_v4/`, collar funnel) has scripts but no part doc;** the funnel rows in `current-parts.md` may be superseded by it. Fix: the catch chat writes `nox/box/box-funnel.md` up to date and its rows.
43. **Built parts with no saved file:** heads A / B v5 (waiting on question 28; Spark 19 Oct: the heads will be regenerated). Fix: the owner chat saves to `print/` and sets READY; until then the print chat cannot reach them.
44. **Docs that still describe superseded versions:** `nox/box/box-arm.md` clamp section (loop-and-pin, now the fallback behind the bought connector, decision 27); `box-arm.md` Stub v3 section and Problem 23 (2040 post and M5 x 8: now the printed mast and M5 x 12, `box-stand.md`); `nox/box/box-case.md` Ring frames (end frames are v7b with the bracket holes; `box.md` item 4 says v6). Fix: each owner chat updates its doc on its next turn.
45. **Design scripts that still assume no supports** (`noxopt` overhang filter on). Supports are allowed everywhere and the bed face is picked by `print_orient.py`. Fix: each owner turns the filter off on its next run where it costs mass or strength and adds the `print_orient.py` pass to its Done line; nothing reruns just for this.
46. **Printed attachments snapped at members the index passed** (receiver v3.3 plate attachment, a cartridge plate). The layer-aware index over-rates small cross-sections on this printer. Rule now in `docs/rules.md` Attachments (pads ≥ Ø12, posts ≥ 6 x 6, gussets, index ≤ 0.5 there) and caught by `solid_check.py`. Stays open until the pull coupon (test 20) calibrates `printopt`; owner chats apply it on their next version, nothing already printed and holding is reprinted for it.
49. **Every carbon rod is 3.0; anything drawn for 2.5 or 2.0 rods is superseded for those features.** Scout v2 sockets were fixed in v3.2 / v4 / the receiver v3.4 (archive). Still open: test 8's coupon rod, and any Scout v3 socket (bore from test 21).
50. **Bay module dovetail flanks have 0 clearance** (`box-case.md` Problem 7): `arm_cad._dovetail(-1, 0.2)` grows width and depth together, so the groove flank lies on the tongue flank. Fix: sand the printed modules' tongue flanks; the next module version grows the groove width only. Ring frames already carry the fix.
51. **Solid check backlog:** every READY part made before 20 Oct (arm hub set, bay module v4i; in the archive: receiver v3.4, cartridge tray / block, camera mount, cage v3.2 / v4 nodes) has not been run through `solid_check.py` with the neck test. Fix: each owner runs it on its saved file (1 to 3 min) and reissues a version on FAIL.
52. **`print/latest/` is empty** although every box part is READY in `print/`: `LATEST.md` has no rows and no `print/latest/<part>.<ext>` exists, so a print session has nothing to slice from. Fix: each owner copies its newest file to `print/latest/<part>.<ext>` and writes its `LATEST.md` line (version, date, g, sha256 first 12), or Spark decides that `print/` + the File cell in `current-parts.md` replaces the latest folder and the rule in `docs/printing.md` is cut.

## Questions for Spark
Measure (caliper, one at a time; each goes in `nox/dimensions.md`):
2. D13 rest: idler bore, slip-ring flange OD + holes, MG90S ear spacing, 80T body width. [Before the arm head prints.]
28. **Belt connector (decision 27):** length along the belt, width, height, bolt size and spacing, which face the teeth sit on. [Gates heads A / B v5b.]

On hand? (yes / no / how many):
4. NEMA17 x3, MG90S x2 (D9), electromagnet + steel plate (D11), DC load or power resistor for the 10 A pad drop test (Problem 9), 1000 µF cap, INA226.
23. 3.0 solid carbon rod: how much is on hand? The v2 cage needed 2.48 m (C); v3 will set its own cut list. Decides whether buy item 13 is needed at all.

Decide:
5. Funnel size (Problem 36): levers (a) pyramid base inboard of the ring, (b) measure hover scatter D7, (c) pack on end under the plate. [Best: b now, a in the funnel chat, c in the bottom layout.]
7. OK to drill 3 holes in the 80T pulley's web. [Best: yes.]
19. Problem 46 sizes (pads Ø12, posts 6 x 6, index 0.5) are a DEFAULT; veto or change them. [Best: keep until the pull coupon is measured.]
29. **Hub v5 supports: 37.6 g = 33 % of the model (the ~25 % line).** Print the slice as is, or send the hub back for a lighter motor plate? [Best: print it; the supports are under the motor plate where they come off clean, and a regen costs a day.]
32. **Enclosure sheet material:** 3.0 acrylic is the DEFAULT (cut list in the enclosure row); polycarbonate 3.0 is the tougher choice at the same slot. [Best: acrylic if on hand or cheaper; polycarbonate if a drop test is planned.]
34. **Skid strips 3.0 proud instead of 2.5** (shared catch interface): gives the cage's skid-ring sockets 1.0 to the funnel face instead of 0.39; the seat rises 0.7. [Best: yes, before the first catch; the settle sim reruns anyway.]
Scout v2 leftovers, moot unless v3 reuses the part: 26 (cage mass vs the 55 budget), 30 (v4 node set vs v3.2), 33 (cage v12 drawn skeleton vs a neck-test exemption; applied as DEFAULT in v13).

Physical tests (after the prints; results to the part doc, state to `current-parts.md`):
8. Funnel friction coupon: 3.0 rod on PTFE tape on a printed 35° face, clean and dusty; needs μ ≤ 0.58 (Problem 21). Only if the catch still slides a rod on a printed face.
9. Glue coupon: PETG, epoxy vs methacrylate, scuffed (Problem 27). Decides the funnel and bay glue.
10. Latch on the real cartridge + receiver (`archive/scout-v2/s2-bottom-latch.md` tests 1 to 5); bay lip + gripper finger: hold in every orientation, 50 cycles, force both ways (Problem 35).
11. Magnet hold through the hub floor skin; must beat preload + 2 pogo springs (Problem 34).
12. Hover ulog over the box (D7, question 5 b); vibration log on the first hover with the top case (Problem 23); flight current log, then bus scope in a bench swap (Problem 1). Standard props and a PX4 Actuators spin check before the first hover after the flip.
20. Post pull coupon (Problem 46): two 6 x 6 and two 4 x 3 PETG posts on a plate, pulled to break by hand scale or hanging weights; numbers (P) to `nox/dimensions.md` C. Calibrates the index for every attachment.
21. **Socket v2 coupon** (`archive/scout-v2/print/s2_socket_v2_coupon_v2.stl`): which bore (3.5 / 3.6 / 3.7) a 3.0 rod slides into by hand without wobble, standing and lying; then the 24 h epoxy pull on the standing 3.6 (P). Sets every 3.0 socket bore from here on.

Buy (when the part is ready for it):
13. Carbon rod 3.0 solid, one size for everything; no 2.0, no 2.5, no 4x3 tube. Only if item 23 says there is not enough on hand; length from the Scout v3 cut list.
14. **4 x M5 x 12 ISO 7380 button head** for the stub on the printed mast (`nox/box/box-stand.md`).
15. **Enclosure (`nox/box/box-stand.md` Hardware):** 4 clear sheets 3.0 (2 x 397 x 441, 2 x 366 x 441); 24 x M3 x 16 + 12 x M3 x 10 + 36 M3 nuts; 8 x M5 x 25, 4 x M5 x 35 (if not on hand).

Later: patent attorney on the swap, the pack-side squeeze latch and the one-squeeze grip-and-release before selling (`claude/research/2026-10-13-patents-swap.md`, `archive/scout-v2/s2-bottom-latch.md` Patents).

## Closed
Problems: 2, 3, 18, 22, 28 (supports allowed on every part), 38, 39, 40, 42, 47. Questions: 14, 15, 16, 17, 18, 21 (print the flipped v3.3), 22 (every rod 3.0), 24 (camera mount stays separate), 25, 27 (bought belt connectors replace the loop-and-pin), 31 (no ballast). Outcomes: `nox/log.md`.
