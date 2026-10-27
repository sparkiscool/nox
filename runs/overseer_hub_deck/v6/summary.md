# overseer_hub_deck v6: FAILED (strength)

Run 27 Oct 08:12-10:12 PDT (2.00 h; smoke 1.0..2.4 h at h 1.25, h 1.0 ran out of memory), peak 8.0 GB (M). `nox/overseer/core_topo_v3.py hub_deck 1.25 18` (core v3: the tower for the L2 braces, the cage frame model's per-case root forces).

- Mass 53.9 g (M); **worst index 2.72** in crash 4 (top hit: braces pull 309 / 403 N a side, keels push 213, + 35 g) at the +y keel boss / front foot joint (94.1, 15.6, 8.1); crash 0 1.42 at the toe (104, 17, 4).
- solid_check (CLI): FAIL, 2 voxel bodies (a fragment); the build check also FAIL (a drawn body 99.8 %).
- The deep side web room was barely used: 18 cm3 generated over 165 cm3 of design space (volfrac 0.11).
- Fix in v7 (setup): a drawn solid core round the keel bosses and front feet (x 88..104, |y| 8..24, z 0..14), 30 cm3 generated.
