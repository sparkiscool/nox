# overseer_hub_deck v3: FAILED (the build's drawn-body test; `nox/solid_check.py` on the file alone reads PASS)

Run 27 Oct 01:46-03:51 PDT (2.08 h; smoke estimate 1.51..3.59 h, `estimate.json`), peak 9.4 GB (M). `nox/overseer/core_topo_v2.py hub_deck 1.0 12`, core v2 requirements.

- Mass 34.5 g (M), worst index 0.35 in "keels pull 274 N each (F) + 35 g up" at (94.5, 18.5, 7.3), function check OK, 1 body.
- `python3 nox/solid_check.py overseer_hub_deck.stl`: PASS (`solid_check.txt`). **The build's check with the drawn bodies FAILS** (log.txt): drawn bodies not in the main body at the 2.0 erosion: the 2 keel boss + front foot bodies (share 0.0), the deck's bay body (0.43), the rear feet (0.99), a 3.5 sliver of a mid foot.
- Cause (setup, not the mesh): the layout warned "the solid gate wants 6.25 mm members ... the design space routes at most 4.44 mm to every body". The limit was the mid feet (65, ±22.5), cut to a 3.5 sliver by the conservative arm box (|y| > 20 ahead of x 56); every member then ran at 4.44, under the gate's 2.0 erosion. Feet Ø12 round Ø4.0 left 4.0 of wall.
- Fix in v4: the arm session's real root envelope (u >= 58, |v| <= 21 along each arm), feet Ø14, the keel tube cut 0.5 into the socket mouth (no end slivers). Not fixed on the mesh.
- Preview `preview.png`. Superseded by v4.
