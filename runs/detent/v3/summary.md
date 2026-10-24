# Detent v5: ring v3 (24 Oct 2026, swap arm system worker)
Ring v1 failed solid_check; ring v2 passed it but broke the 2.0 rule (1.54 from the button-head spot faces to the notches, 1.05 from the Ø3.4 holes to the bore on the 80T boss, M): strike 2. **Ring v3** (`detent.ring_v3`): no bolt through it; three Ø6.2 x 1.9 pockets in its underside key it on the 80T's own M3 ISO 7380 heads (Ø5.7 x 1.65, E: CHECK on arrival), epoxied on the Ø30 boss (DEFAULT); the pockets open into the bore, 2.4 inside the rim and 2.05 (3D) from the nearest notch flank (C). Same 12 notches (roller at −135°, `CHOSEN_V6`), same magnet. Detent arm v2 unchanged (2 x DIN 988 3 x 5 x 0.5 shims). Tower in hub v6 at (21.2, −30.3): every gap ≥ 1.0 on the v6 stack (C; `detent.tower_gap`).
```
detent_ring_v3: DESIGN (v6 concept check FAIL x2; READY in v7 after its concept check)
System: box swap arm v6 (`nox/box/swap_system.py`, `nox/box/box_swap_arm.assembly.json`), assembled worst index 0.18 in the detent's holding torque 0.55 N·m (keyed bearing ~1.8 MPa on three pockets, E; was n/a), fit 0.0 mm³ at 5 poses (M), mass 3232 g (E, the assembly with its context), CG 7.0 mm off the axle (E; the turning parts' unbalance 22.5 g·m at park, M / E)
Weight: 6.7 g (M), was 5.0 (v1): the 1.7 g buy a full-height ring without the v1 lower step and bolt holes, the three key pockets need the 5.9 height to keep the 2.0 walls
Strength: keyed torque 0.55 N·m at r 18 over three pockets ~10 N each (E, bearing ~2 MPa); fits
Solid: solid_check PASS (print/box_detent_ring_v3.stl)
Print: bottom (pocket side) down, bed contact 916 mm² (M), pockets bridged 6.2, no supports; file print/box_detent_ring_v3.stl
```

Previews (rendered from the files this PR first saved in print/, now removed until v7): `preview_box_detent_ring_v3.png`
