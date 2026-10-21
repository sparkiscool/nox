# S2 receiver (Scout side of the swap, merged with the underside), v3.4 (18 Oct 2026 00:20)

One generated PETG part on the three carbon-plate bolts. It takes the cartridge like a bay does (30° lead-ins, locating faces, two latch pockets, two copper shoe modules), carries the flow sensor under the camera, and holds the four cage skirt rods. State: `v2/current-parts.md` (only place). Numbers: `v2/dimensions.md` E / H. Script: `v2/receiver_v34.py` (Nox Source); skirt interface `v2/cage_v2/receiver_skirt.py` + `.json` (cage chat owns them). Coordinates in the script are cartridge coords: x_c = drone y, y_c = drone z + 44.5, z_c = 37.5 − drone x; the saved STL is in print pose (pads on the bed, z up = −y_c).

## v3.4 (what changed from v3.3 and why)
| Change | Detail | Why |
|---|---|---|
| Plate attachment drawn, not generated (Problem 46) | Pads: sides r 8.6, nose r 7.5 (all ≥ Ø12, ≥ 2.0 web round the Ø3.3 bolt hole), 2.5 thick on the plate. Nut cups: full 2.0-wall rings r 5.5..7.5 (82 mm² section, > 6 x 6) from the pad down to the deck underside (y 49.5); the wrench slot (8 wide, nut height) is an exact cut through them. Collars r 9.5 (wall 4.0) at nut height beside the slot. Nose cup = tail half only (the camera mount sits on the nose half): it runs on as a 2.0 skirt 11.5 below the deck (to y 38) and fuses into the end tab's nose face, on a deck tongue 15 wide under the whole half ring. Side cups: deck tongues the width of the cup (15) + a 2.0 gusset from the cup to the deck edge, foot at y 35. Pocket-zone locating posts 3.0 thick (were 2.0). | v3.3 snapped at the nose: its nose cup was a half ring drawn only to the deck underside and crossed the deck's end bar in two 3.2 mm² necks (M); the sides were drawn through to the deck, the nose was not (C, from the v3.3 drawn set). Passes 1 and 2 moved the hot spots: side collar 0.65 → r 9.5 collar; pocket post 1.08 across layers → 3.0 posts + deeper gussets; nose slot edge 0.60 → nose collar. |
| Cage skirt sockets for 3.0 rods (Problems 48 / 49) | Bore 3.3 (rod + 0.3), 10 deep, 0.6 lead-in, closed end 1.0, sleeve wall 2.0 (r 3.65). Roots at drone (±30.5, ±26.5, 5), each aimed at its side node's own socket (±3.2, ±112, 33.2), root to target 94.1 (C). Loads from the frame model with every rod 3.0 (F, `receiver_skirt.json` 22:20): rods at index 1 at 0.89 m/s (was 0.71), peak skirt root force 183 N (was 132). | Spark 17 Oct 21:33 / 21:45: every rod 3.0. v3.3 sockets were 2.8 bore and 1.85° off (cage print chat 21:21). |
| Shoe-lead channel rerouted | v3.3 cut the leads' channel (z ZM+4..ZM+9, up to the plate) straight through the tail skirt sleeves, so those two sockets were only ~6.5 deep (C). v3.4: leads leave the back-wall hole (z ZM+7.6), run 5 nose-ward under the sleeve (y 39..46), climb to the plate at z ZM−2..ZM+3, 0.9 clear of the sleeve end (C). DEFAULT. | Found while checking the drawn set. |
| Print pose | Pads on the bed, build −y, overhang filter off (supports allowed, Problem 45). The two v3.3 bed columns under the lead-end tab (mouth-down fix, 1.3 g) are removed. | README Printing, Orientation and supports (17 Oct 18:30). |
| Camera keep-out | Mount v2's real envelope: drone x 52..70, |y| ≤ 21, z −13.5..26, plus the nose pad hull (x 45..51.5, z 6..16). | Mount v2 left the v3.3 keep-out by up to 2.0 at x 52..54 (camera chat). Spark 23:06: the mount stays a separate part. |
| Grid | x ±41 (was ±39): the side pads (x to 39.6) no longer clip at the grid edge (12 mm³ missing drawn in pass 0). | function check 0.1 / 0 / 0. |

## Numbers (18 Oct 00:20)
| Item | Value | Tag |
|---|---|---|
| Mass | 45.7 g (v3.3 43.7; drawn attachment +2.3, bed columns −1.3, pocket posts +0.7, generated −1.3) | M |
| Drawn / generated | 27.4 cm³ drawn exact (34.8 g; attachment 10.6 g, skirt sleeves 2.3 g); 8.0 cm³ generated budget; grid h 2.0, 41 x 31 x 60 cells, 27 cases | C / M |
| Layout | 137 iterations, 0.6 % grey, 784 s; prune 0 thin members; one watertight body (8 smoothing slivers under 2 mm³ at the sleeve caps dropped, 5.5 mm³) | M |
| Function check | drawn missing 0.1 mm³, in cuts 0, in keep-outs 0; thin check 0; wrinkle 0.29 / 0.48 | M |
| Strength, as built (layer-aware, bosses down) | worst non-misuse 0.62 in the skirt pole hit (generated member at x −22, y 39, z 43); attachment members ≤ 0.43 on every crash / flight case (0.67 in the pinch 20 N misuse); worst overall 1.25 pinch 20 N on the posts (misuse; v3.3 1.36). v3.3 was ≤ 0.76 and snapped: the index is uncalibrated until the pull coupon (Spark item 20). | M |
| Print | pads on the bed, 599 mm² (M) bed contact, 3 first-layer islands (155 / 222 / 222), no thin islands, best of the 6 faces (`print_orient.py`); 4453 mm² overhang → tree(auto) supports (~9 g E, like v3.3's 9.34 slicer); 81 x 117 x 61 | M |
| Flow sensor | board centred under the camera, lens at drone (63, −2, −24), tape land + 1.2 rim, plug pocket out the tail, 23° view cone clear of the cartridge (0 mm³ overlap) | C |
| Skirt rods | 4 x 3.0 solid, root to side-node socket 94.1 (C); cut length from the cage cut list (90.1 in `v2/s2-cage-v2.md`), epoxied both ends | C |

## Assembly and test (Spark, after the print)
1. Three M3 nyloc nuts into the cups (wrench from outboard on the sides, from the nose on the nose), bolt to the plate. Hand pull + twist at each boss before anything else.
2. Skirt rods: dry-fit all four (3.3 bores, no drilling), check the side nodes land at their sockets, then epoxy (rod sockets are never press fit).
3. Latch tests 1 to 5 on the real cartridge (`v2/s2-bottom-latch.md`); the shoes (`v2/s2-contacts.md`) land on the copper strips.
4. Weigh (P) → current-parts g cell; any break → `v2/log.md` failure log.

## Open
- The index is uncalibrated (Problem 46): until the pull coupon (item 20), the attachment numbers above are relative, not safety factors.
- The v3.3 fit print (its own row in `v2/current-parts.md`) is the DubHacks fallback: necks at the nose, tail sockets ~6.5 deep, sockets to be drilled 3.3.
- Contacts chat: shoe placement against this body (Problem 31 is otherwise closed by v3.4).
- A v3.5 would only be needed if the hand-pull test or a flight breaks something; a generated-member miss (0.62) is not a reason.
