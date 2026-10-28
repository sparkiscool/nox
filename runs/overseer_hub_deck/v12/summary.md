# overseer_hub_deck v12: saved (core v4: no head load)

Part: the Overseer hub deck, one PETG print on the bottom plate (`nox/overseer/core_topo_v4.py hub_deck 1.0 11 --ver=v12`, nox/topo, organic finish, members 6.25 (C)). Drawn (C): 6 feet Ø14 x 6.5 with M3 x 5.7 inserts (M3 up from under the plate), 4 Pi posts Ø7 x 8 on Ø12 root webs (M2.5 self-tapping), 4 pads Ø12 a bay under the UBEC and the buck (VHB); the topology joins them. Requirements: `nox/dimensions.md` System Overseer core (core v4, `runs/overseer_core/v4/`). The cage hangs the L2 head on 3 rods to its own nodes (PR 165), so nothing of the head lands on the deck: v9's tower and its 6 sockets are gone.

Waited for: the cage's head-rod change on main (PR 165, merged 27 Oct 17:41 PDT); the run used the cage branch's arms (`CORE_ASSEMBLY` = core v4 on that branch), the assembled recheck below is on main after the merge.

Smoke (`estimate.json`): drawn first FAIL (the drawn set alone is 13 bodies, solid FAIL; worst 0.18), room audit PASS (min 0.47), fine 20.2 s/it, 86 cases, 5.96 GB, wall 0.38..0.95 h (E). Run 17:30-18:12 PDT, 0.70 h, peak 5.5 GB (M).

```
overseer_hub_deck v12: ready to print
System: Overseer core v4 assembled on main (v12 mesh, `runs/overseer_core/v4/checks.txt`): fit 0 mm³ with arms, Pi, UBEC, buck, both plates, cage tubes and nodes, links, wires (C on M); cage first 9.6 (E); access 39/39 clear; mass 3235 g (E), CG 0.3 off the thrust axis pack in, 28.2 pack out (L exception) (E); system_check PASS
Weight: 37.1 g (M), was 68.0 (v9, tower); budget 30 (DEFAULT, E)
Strength: worst index 0.17 (M) in "press 50 N on the Pi (assembly)" at (57, 18.5, 0.5); crash cases 35 g x 14 (E) with the payload (Pi 55, UBEC 15 + 10 wires, buck 6 + 10; g, E)
Solid: solid_check PASS (print/overseer_hub_deck_v12.stl), 1 body; the build's drawn-body test PASS
Print: plate face down, bed contact 2245 mm² (M, print_orient), 139.1 x 87.0 x 8.0, tree supports under the frame; file print/overseer_hub_deck_v12.stl
```

- `solid_check.txt` PASS; `print_orient.txt`: plate face down, 10 first-layer islands 147..291 mm² (M), no thin island, 8.0 tall (fits 180).
- Modes (`modes.json`, `nox/overseer/hub_deck_modes.py` with core v4, voxel FE at 1.0, feet fixed, Pi / UBEC / buck lumped): first **432 Hz** (Pi mount >= 100, L; C on M). FE bolt reactions at 35 g, SF 1.1: <= 11.4 N a foot, pull <= 0.5 N (C on M); rigid-body bound 14.8 N (E).
- What the grams buy (37.1 vs the ~29 g estimate, E): the organic finish's skins and fillets round the 6.25 members (layout 22.8 cm³ -> 29.2 cm³ built). Option B (Pi on standoffs, ~4 g, E) would save ~33 g but drills 4 holes in the carbon plate and puts 4 heads in the receiver's zone (`runs/overseer_core/v4/alternatives.png`).
- Carbon: none in this piece.
- DEFAULTs: option A (one deck) over B and C; UBEC / buck on VHB on 4 pads a bay; the deck wires' 20 g on the bays.
- The insert pull test on the keel hub coupon is dropped with the keels (the v1 hub deck coupon checks the plate holes, posts and inserts; 11 N a foot).

Previews: `preview_overseer_hub_deck_v12.png` (the print file), `preview.png` (the run).
Supersedes: hub deck v9 (HOLD, the tower), v10 (room audit FAIL), v11 (lost to a box restart), v11b (stopped); `print/overseer_hub_deck_v9.stl` removed (in git).
Open (Spark's scope): body tie roots (front, rear, side, bay) stay parked with the body nodes; the cage's front core ties at (114, 0, 20) root on nothing now (the cage's to re-root).

Artifact: https://claude.ai/artifact/JSamk2HosoRiVZgo3KHBH3 (core v4 + hub deck v12, republished 27 Oct 18:40 PDT).
System sync: the core manifest `nox/overseer/overseer_core.assembly.json` carries the v12 mesh (core v4 on main); the cage session told on save (file, commit, 37.1 g M).
