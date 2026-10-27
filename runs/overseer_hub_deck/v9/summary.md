# overseer_hub_deck v9: saved (core v3: the tower for the L2 braces)

Part: the Overseer hub deck, one PETG print on the bottom plate (`nox/overseer/core_topo_v3.py hub_deck 1.25 24 --ver=v9`, nox/topo, organic finish, members 6.25): Pi 5 posts on root webs, UBEC / buck seat rails (VHB), 2 keel sockets and 4 L2 brace sockets (6.2 bores) on a front tower with drawn cores and bars back to the mid feet, 8 feet Ø14 with M3 x 5.7 inserts (M3 up from under the plate). Requirements: `nox/dimensions.md` System Overseer core (core v3, low brace roots), per-case root forces `runs/overseer_core/v3/core_root_forces_low.json` (cage frame model, F on E).

Waited for: hub deck v8's result (its build drawn-body failure is what v9 fixes; v4 -> v6 -> v8 -> v9 each needed the one before).

Smoke (`estimate.json`): fine 34.0 s/it, 50 cases, 5.48 GB, wall 0.59..1.44 h (E). Run 11:31-12:30 PDT, 0.90 h, peak 5.3 GB (M).

```
overseer_hub_deck v9: ready to print (after the insert pull test on the keel hub coupon)
System: Overseer core v3 assembled (v9 mesh, `runs/overseer_core/v3/checks_run.txt`): fit 0 mm³ (C on M) for the deck (the 28.0 FAIL is the flow clip vs the cage's parked tie node), cage first 7.7 (E), probes -210 at -X FAIL (slot unshielded, cage's), bbox 124 L, extents 771 x 675 x 239 (E), access 37/37, mass 3255 g (E), CG 2.0 off the thrust axis pack in / 29.7 pack out (L exception) (E); FE bolt reactions <= 343 N a foot (front hit, SF 1.1), insert pull <= 70 N (C on M); head mode 153.3 Hz with these socket springs (cage, F on E)
Weight: 68.0 g (M), was 38.9 (v4, keels only)
Strength: worst index 0.30 (M) in crash 0 (front hit) at the toe (104, -19, 4) (v8 0.34, v6 2.72 on the high roots, M); every crash case with its own keel + brace forces
Solid: solid_check PASS (print/overseer_hub_deck_v9.stl); the build's drawn-body test PASS
Print: plate face down, bed contact 3005 mm² (M, print_orient), tree supports (organic underside); file print/overseer_hub_deck_v9.stl
```

- `solid_check.txt` PASS; `print_orient.txt`: plate face down, 4 first-layer islands 153..2436 mm² (M), 25.9 tall, 153.7 x 87 x 25.9 (M) (fits 180).
- Sockets: a 6 mm tube on each of the 6 lines from its tube end clears the deck (0.00..0.01 mm³); walls 3.1..5.6 99..100 % filled (C on M).
- Modes (`modes.json`, `nox/overseer/hub_deck_modes.py`, voxel FE at 1.25, feet fixed, Pi / UBEC / buck lumped): first 659 Hz (Pi mount >= 100, L). Socket axial stiffness (N/mm): braces lo 4611 / 5342, hi 3236 / 3116; keels 9649 / 9117 (C on M). Head mode with these springs in series at the core ends: **153.3 Hz** (cage frame model, `runs/overseer_head_cage/concept/result_A1_low_roots_deck_v9.json` on the cage branch; 155.4 rigid core; F on E) >= 100: no stiffer deck needed.
- FE bolt reactions (SF 1.1, C on M): front hit max 343 N a foot (shear), pull <= 11 N; top hit max 216 N, pull <= 70 N (the mid feet). The rigid-body bounds (412 / 240 N pull, E) were conservative.
- Assembled recheck (`runs/overseer_core/v3/checks_run.txt`): 0 mm³ with arms, Pi, UBEC, buck, both plates, cage tubes and nodes, props, wires; the 2 heads at (60, ±17) in reserved slot-top pockets (latch ask); fit FAIL 28.0 mm³ is the flow clip vs the cage's parked front tie node.
- What the grams buy (68.0 vs v4's 38.9 g): the 4 brace sockets and the tower that closes the head's keel + brace triangle on the core (the braces left the front arms), at worst 0.30.
- History: v4 keels only (superseded); v5 stopped; v6 (high roots) FAILED 2.72; v7 stopped (roots lowered instead); v8 build drawn test FAIL (+y tower joint through the centreline); v9 drawn bars tower -> mid feet.
- Carbon: none in this piece (2 keels + 4 braces are the cage's 6/4 tubes, epoxied into the 6.2 bores).
- DEFAULTs: low brace roots z 12 / 18 (alt z 22 / 32: 2.72); UBEC / buck on VHB; the front Ø3.5 holes' 0.25 play accepted.
- Gate before printing the deck: the insert pull test on `print/overseer_core_coupon_keel_hub_v2.stl` (13.0 g, M): an M3 in an insert, pulled by weights or a spring scale, >= 200 N (DEFAULT: 3x the FE's 70 N pull, C on M). The head-mode gate (>= 100 Hz with the springs) is met: 153.3 Hz (F on E).

Previews: `preview_overseer_hub_deck_v9.png` (the print file), `preview.png` (the run).

Cage sync (CLAUDE.md, review item 15): the cage session told 27 Oct 12:45 PDT (file, commit 26bf1475, 68.0 g M); its artifact https://claude.ai/artifact/VhDfnp75uoVsj8SnyrW2Qg republished with hub deck v9 13:07 PDT 27 Oct (version 14; cage branch claude/gracious-mccarthy-5rctc9 @ ee944ce8, its system_check PASS).
Open (Spark's scope): the cage now roots its front core ties on this tower's front face (114, 0, 20), its rear / side ties on the bottom plate edges and its bay ties on a sled; Spark parked the body nodes ("ignore all nodes on the body for now") and the drone side is an empty slot, so v9 carries the keels and braces only; asked of Spark (a v10 adds the 2 front tie sockets if he says yes).
