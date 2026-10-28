# Scout v3 layout v3 / cage v2, race-car shape: v13, the catch re-trimmed for the real node mass (28 Oct 2026, concept)

**Why:** the saved cage nodes v2 (PR 166, `runs/scout_nodes/v2/`) weigh 192.2 g (M) where v12 carried 39 g (E). On v12 they put the CG 4.65 mm forward (system_check, 899 g E), over the 3.0 gate (DEFAULT). The layout's own mass model was the error, so the system pass reruns on it (CLAUDE.md "System first").

**What changed from `v12/`:**
- **Node mass model** (`layout_v3.py` `node_g`, `NODE_FIT`): g = 2.34 per shell socket + 3.49 per saddle-tie socket − 0.57 on a ring node − 3.14, fitted to the 18 saved v2 nodes (188.8 g M, max residual 1.9 g, E). The shell scan and the balance scan both use it.
- **Camera mount at its saved mass:** 22.3 g (M, v1 21.8 + 2 collars 0.5) in the layout's balance (`SCOUT_CAM_MOUNT_G`).
- **Electronics:** the electronics session's round 5 envelope (`electronics_envelope.json`, SOURCE.txt) in place of round 3.
- **Catch:** the scan (TB 25, DC −16..+6) picks **2.2 aft of the thrust centre** (v12: 5.0 forward, so 7.2 aft, C). The bottom loop moves with it.
- **Cage:** 19 nodes (4 on the ring), 50 shell rods (61..239, 6.34 m) + 8 saddle ties = **58 rods, every one 3.0 solid** (Spark 27 Oct), shortest 61 shell / 82 tie (all over the 33 break-even). The nose rods 7-8 and 14-15 and nodes 7, 8, 12-15, 17 sit where v12 had them (0.0 moved, C); the ring, waist and tail nodes moved 7.1..7.7 aft; one new top-centre node 16 at (−26.3, 0, 61.3); node 18 (nose low) moved 3.6. Offset scan: 16 → 50 rods, 20 → 62, 24 → 50 (picked: least shadow), 28 → 56, 32 → 53.
- Props: shell tubes 13.9 off the swept disks (≥ 12), nodes 17.0, links 14.8 (C). Largest cell Ø129 (C). Prop shadow 3.0 % above, 8.0 % below (E). Funnel band: 0 links in it (C). Tail nodes past x −130 about the catch ≥ 91.1 above the plane (≥ 45, C).

System (`system_check_mount.json`, the assembly with the saved camera mount v1, which still fits: its clip rods did not move): Scout v3, cage first 6.2 mm (E) (camera), shell 50 rods closed, fit 0.0 mm³ (E), extents 398 x 278 x 127 mm (E), **mass 909 g (E)** of 500 (soft budget), **CG 0.6 mm off the thrust axis pack in, 0.7 pack out (E)**, access 8/8 clear. PASS.

## Stale on v13 (their sessions redraw them; the layout placeholders stand in)
- **Cage nodes v2** (PR 166): drawn on v12's cage; 12 of 19 nodes moved and one is new. The node script reruns on `v13/cage_shell.json`.
- **Electronics round 5 and the belly mount v1:** drawn at the v12 catch; the catch moved 7.2 aft. The electronics session redraws (with the FC on top of the plate, Spark 27 Oct). Wash on the round 5 envelopes as moved with the bay: mount spine −3.4, tail block −11.3 (bounding boxes, C); Radxa +12.3, UBEC +22.0, XR2 +3.8.

## Strength (frame model v4 on v13, `runs/scout_frame/v4/`, every rod 3.0)
30 of 58 buckle, 12 crash and 22 handling stresses over 600 MPa, first mode 48 Hz with the camera moving (F on E; node masses lumped at the old 1.5 g, E). Spark 27 Oct: 3.0 everywhere, the rods are the fuse, accepted. The camera mount v2 (worker) must reach 100 Hz on these rods.

## Mass (E)
909 g (system_check): cage 278 (shell rods 70, nodes 200 by the fit model, saddle ties 8), the layout's tie saddle 17, camera + mount 32.3 (M + E), electronics round 5 (mount 49.2, cables 6.3), bought, latch set and wires ~526. The nodes are the new weight: 200 g against v12's 39.

Script `nox/scout/v3/layout_v3.py` (v13 default), view `layout_v3_view.py`, assembly updates `scout_assembly_updates.py` (each part's drawn-on run; stale parts stay out). Visual: https://claude.ai/artifact/WHdjYkzKPifXuHT1dWeqoR (version 25, 28 Oct; source `motion.html`, still `preview.png`).
