# Rod socket coupons v1 (27 Oct 2026, Spark: "make coupons for the 6mm and the 3mm rods, make it so it lets resin make the best hold on the rods and print 3 per with small difference in size so i can tell you what we want")

Script `nox/overseer/rod_coupons.py`. Pictures: `preview_6mm.png`, `preview_3mm.png`, `section.png` (cut through the sockets).

| File | Stock | Bores (1 / 2 / 3 dots) | Depth | Vent | Mass (M, TPU) | Bed | Solid |
|---|---|---|---|---|---|---|---|
| `print/rod_coupon_6mm_v1.stl` | 6 mm carbon tube | 6.30 / 6.45 / 6.60 | 15 | Ø1.6 | 7.7 g | base, 1084 mm² | PASS |
| `print/rod_coupon_3mm_v1.stl` | 3 mm carbon rod | 3.30 / 3.45 / 3.60 | 10 | Ø1.2 | 4.1 g | base, 671 mm² | PASS |

- `print/rod_coupon_6mm_v1.stl`: solid_check PASS (one body, no thin walls, no necks), picture `preview_6mm.png`.
- `print/rod_coupon_3mm_v1.stl`: solid_check PASS (one body, no thin walls, no necks), picture `preview_3mm.png`.

For the epoxy hold (DEFAULT 27 Oct, E):
- **Glue rings:** two in each socket, 0.5 deep and 2 tall, with a 45° roof so they print without support. The cured epoxy keys into them, so the joint holds on shape, not only on adhesion. Epoxy grips TPU poorly.
- **Floor vent:** air and extra epoxy leave through it, so the socket fills to the bottom. The vent is narrower than the rod, so the rod stops on the floor.
- **Entry chamfer:** 1 x 45°. A small resin reservoir and an easy start for the rod.
- **Walls:** 2.5 round every socket, the same as the nodes and the head mount.

Bores step 0.15 up from the rule's OD + 0.3, because an FDM hole prints about 0.1 to 0.2 under. Spark picks the fit on the printed part. That bore, and the epoxy he likes, go to `dimensions.md` and into the node and head-mount sockets.

Test:
1. Print both coupons in TPU (the nodes' material). Print another pair in PETG to test the head-mount sockets.
2. Dry-fit a rod in each socket.
3. Epoxy one rod per socket and let it cure.
4. Pull by hand, then bend each rod sideways.
5. Tell me which mark holds best.
