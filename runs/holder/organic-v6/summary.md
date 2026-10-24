holder organic-v6: demo run, not saved to print/

| Item | Value |
|---|---|
| Settings | organic finish, `--gen 5.0`, min member 6.25 mm, held ties |
| Setup | bosses 12 mm round cut by the envelope; keep-out under the pack from z 29 (7 mm layer) |
| Mass | 11.26 g (M) |
| Worst index (as built) | 0.582 in 'misuse 10' / 'misuse 12' at (26.5, 7.5 / 32.5, 21.5), the rails along the battery channel's top edges |
| Solid check | FAIL: 1 plain body, 1 eroded body at 0.9 mm; at 2.0 mm each boss keeps a 64-voxel core lobe toward the envelope corner, apart from its main core (share 0.96 to 0.98) |
| Function check | ok |
| Wall time | about 4200 s, 124 iterations |

Look: right. One frame: four legs from the bosses up to the pad, a ring round the pad, rails along the channel, a
fork to the camera pad; round members, open space, smooth fillets; only holes, flat contact faces and the
envelope-cut boss faces are geometric.

What is wrong: (1) the boss lobe (setup): a 12 mm boss on the bolt, clipped by the envelope, is 4.4 mm thick toward
the envelope corner but 3.4 mm toward the faces, so its core splits. Fix: 11.5 mm boss centred 0.75 mm toward the
part (one core, inner side). (2) Worst 0.582 on the channel rails: more material (gen 5.4).
