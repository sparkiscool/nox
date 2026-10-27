# Rod socket coupons v2 (27 Oct 2026, Spark: "the coupons are too big the smallest one is still loose print the smaller series, also are we making sure we are accounting for ones bring printed sideways being the same")

Script `nox/overseer/rod_coupons.py v2`. Pictures: `preview_6mm.png`, `preview_3mm.png`, `section.png` (the smallest bore cut through, standing and sideways).

| File | Stock | Bores (1 / 2 / 3 dots) | Rows | Depth | Vent | Mass (M, PETG) | Bed | Solid |
|---|---|---|---|---|---|---|---|---|
| `print/rod_coupon_6mm_v2.stl` | 6 mm carbon tube | 6.00 / 6.10 / 6.20 (C) | standing + sideways | 15 (C) | Ø1.6 (C) | 16.0 g (M) | base, 2047 mm² (M) | PASS |
| `print/rod_coupon_3mm_v2.stl` | 3 mm carbon rod | 3.00 / 3.10 / 3.20 (C) | standing + sideways | 10 (C) | Ø1.2 (C) | 7.8 g (M) | base, 1170 mm² (M) | PASS |

- `print/rod_coupon_6mm_v2.stl`: solid_check PASS (one body, no thin walls, no necks), picture `preview_6mm.png`.
- `print/rod_coupon_3mm_v2.stl`: solid_check PASS (one body, no thin walls, no necks), picture `preview_3mm.png`.

What changed from v1:
- **Smaller bores.** In PETG even v1's smallest bore (6.30, 3.30) was loose (Spark, P). v2 steps 0.10 down from the rod's own OD: 6.00 / 6.10 / 6.20 and 3.00 / 3.10 / 3.20.
- **Sideways row.** Each bore is printed twice. The front row stands up (axis = build z, a round hole). The back row lies along the bed (axis y, mouth on the back face). A sideways hole prints differently: layer steps on its sides and a sagging roof make it tighter top to bottom (E). The node and head-mount sockets point every way, so Spark picks a bore for each row, and a socket in between takes its bore by its angle to the bed (DEFAULT, linear in the angle).
- Unchanged: two glue rings per socket (0.5 deep, 2 tall, 45° roof), a floor vent, a 1 x 45° entry chamfer, 2.5 walls (2.6 round the sideways holes).

Test: dry-fit a rod in each of the 6 sockets per coupon, epoxy the snug ones, pull and bend. Tell me the best mark in each row (standing and sideways) for each stock.
