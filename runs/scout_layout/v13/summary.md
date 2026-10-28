# Scout v3 layout v3 / cage v2, race-car shape: v13, the catch re-trimmed for the real node mass (28 Oct 2026, concept)

**Why:** the saved cage nodes v2 (PR 166, `runs/scout_nodes/v2/`) weigh 192.2 g (M) where v12 carried 39 g (E). On v12 they put the CG 4.65 mm forward (system_check, 899 g, E), over the 3.0 gate (DEFAULT). The layout's own mass model was the error, so the system pass reruns on it (CLAUDE.md "System first").

**What changed from `v12/`:**
- **Node mass model** (`layout_v3.py` `node_g`, `NODE_FIT`): g = 2.34 per shell socket + 3.49 per saddle-tie socket − 0.57 on a ring node − 3.14, fitted by least squares to the 18 saved v2 nodes (188.8 g M, largest error 1.9 g, E), + 0.86 g (M) TPU foot cap per ring node. Both the shell scan and the balance scan use it.
- **Camera mount at its saved mass:** 22.3 g (M: v1 21.8 + 2 collars 0.5) in the layout's balance (`SCOUT_CAM_MOUNT_G`).
- **Electronics:** the electronics session's round 5 envelope (`electronics_envelope.json`, SOURCE.txt) in place of round 3.
- **Catch:** the balance scan (tail bay 25 (C), catch −16..+6 about the thrust centre (C)) picks **2.5 aft** (tab line x −2.50, C; v12: 5.0 forward, so 7.5 aft). The bottom loop moves with it.
- **Cage:** 19 nodes (4 on the ring), 50 shell rods (61..239, 6.34 m, C) + 8 saddle ties = **58 rods, every one 3.0 solid** (Spark 27 Oct); shortest shell 61, shortest tie 82 (C), all over the 33 break-even. The nose rods 7-8 and 14-15 and nodes 7, 8, 12-15, 17 sit where v12 had them (0.0 moved, C); the ring, waist, tail and side nodes (12 of 19, C) moved 7.3..7.7 aft (C); the nose low node 18 moved 3.8 (C); one new top-centre node 16 at (−26.3, 0, 61.3) (C). Offset scan (C): 16 → 50 rods, 20 → 62, 24 → 50 (picked: least shadow), 28 → 56, 32 → 53.
- Props: shell tubes 13.9 off the swept disks (≥ 12), nodes 17.0, links 14.8 (C). Largest cell Ø129 (C). Prop shadow 3.0 % above, 8.2 % below (E). Funnel band: 0 links in it (C). Tail nodes past x −130 about the catch ≥ 91.1 above the plane (≥ 45, C).

System (`system_check_mount.json`): Scout v3, cage first 6.2 mm (E) (camera), shell 50 rods closed, fit 0.0 mm³ (E), bbox 14.1 L (E), extents 398 x 278 x 127 mm (E), **mass 912 g (E)** of 500 (soft budget), **CG 0.7 mm off the thrust axis pack in, 0.7 pack out (E)**, access 8/8 clear; assembled worst index: shell rod buckling 9.13 x Pcr in the 35 g crash (F, frame v4, below). PASS (the system_check gates). The drawn 3.0 rods throughout.

**Camera mount v1 in the assembly for its mass and pose only:** its clip rods did not move, but its clips are bored 6.2 for the 6/4 tube and do not grip the 3.0 rods (Spark 27 Oct, review 28 Oct). The camera mount v2 (3.2 clips, topology, worker) replaces it.

## Stale on v13 (their sessions redraw them; the layout placeholders stand in)
- **Cage nodes v2** (PR 166): drawn on v12's cage; 12 of 19 nodes moved and one is new (C). The node script reruns on `v13/cage_shell.json` as v3.
- **Electronics round 5 and the belly mount v1:** drawn at the v12 catch; the catch moved 7.5 aft. The electronics session redraws on v13 (with the FC on top of the plate, Spark 27 Oct). Wash on the round 5 envelopes as moved with the bay (plan distance outside the disks, need ≥ 3, C): mount spine −3.4, tail block −11.3 (bounding boxes); Radxa +12.4, UBEC +22.3, XR2 +3.8.

## Strength (frame model v4 on v13, `runs/scout_frame/v4/`, every rod 3.0, nodes lumped by the socket fit)
31 of 58 buckle (worst 9.13 x Pcr, F), 12 crash and 22 handling stresses over 600 MPa (F), first mode **34 Hz with the camera moving** (F; 48 Hz with the old 1.5 g nodes: the heavier nodes lower it). Spark 27 Oct: 3.0 everywhere, the rods are the fuse, accepted. The camera at 100 Hz (CLAUDE.md drone list 6) is the camera mount v2's job: on 3.0 rods it must find its stiffness near the nodes or from the core.

## Mass (E)
912 g (system_check): cage 281 (shell rods 70, nodes 203 by the fit incl. caps, saddle ties 8), the layout's tie saddle 17, camera + mount 32.3 (M + E), electronics round 5 (mount 49.2, cables 6.3), bought, latch set and wires ~526. vs v12 746: +166, nearly all nodes (+164). What the grams buy: sockets that outlast their rods' push and pull (nodes v2 FE ≤ 0.94). Nodes at 2.9 times the rod mass (203 vs 70, E) is the target for the node rerun.

Script `nox/scout/v3/layout_v3.py` (v13 default), view `layout_v3_view.py`, assembly updates `scout_assembly_updates.py` (each part's drawn-on run; stale parts stay out). Visual: https://claude.ai/artifact/WHdjYkzKPifXuHT1dWeqoR (version 26, 28 Oct; source `motion.html`, still `preview.png`). `runs/scout_layout/v12/parts/` stays: the camera mount and belly mount scripts read it.
