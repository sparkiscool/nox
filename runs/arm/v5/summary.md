arm v5: swap cycle sweep and motion (no new mesh here; parts in runs/arm_rail/v1, runs/detent/v2)
Sweep: `python3 nox/box/arm_kinematics.py 1.0 v5`: 298 states (every 1 s, every keyframe, every 2 deg of turn), 0 overlaps (C)
Closest (C): fingers to servo 0.30, packs in the demo bay 0.30, detent spacer over the notch 0.40, fingers 0.50 (v4 parts / designed); hub proxy to 80T and post 0.99 (1.0 exact); carriage plate to rail 1.0, wheels to plate 1.0, axle stacks to rail 1.4, detent ring to tower 1.07; every pair: sweep_pairs.json
Parts: rails A / B v1, carriages (drawn from arm_v5_layout) with V-wheels v1, M5 x 25 stacks, bushes v1, 16T pinions v1 rolling, detent CHOSEN_V5 (arm v2, ring v1), hub v6 drawn proxy, v5 motors, axle loop at -39 deg, v4 grippers and servos, cartridge v3, bay modules v4i (M) at 9 angles
Cycle: KEYS_V5 (22 s, E): drone insert to 95.6 (+6 E), last 12 of each insert at 20 mm/s (E)
Not swept: the stand and funnel (sweep_box built config stale), hub v6 / heads v6 meshes (rerun when they exist)
Motion: motion.html / .mp4 / .gif / .png (one cycle, readouts head A / B z0)
