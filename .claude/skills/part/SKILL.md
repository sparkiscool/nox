---
name: part
description: One prompt from a design brief to a saved Nox part with its motion artifact. Use when Spark names a part and what to change ("part hub v6: ...", "iterate the detent arm", "build rev N of ...").
---

# /part <part> <version>: <brief>

One session, one part, start to finish, following CLAUDE.md and `docs/generative.md`. Research and decisions are already made: the brief
carries them. Everything below runs in this cloud box; Spark's machine only opens the results.

## 1. Load (read only what the row names)
- `git pull` on `main`; `git checkout -b part/<part>` (or `run/<part>-<version>` if the brief needs a generative run).
- The part's row in `nox/current-parts.md`, its part doc, the `nox/dimensions.md` sections it names. Numbers in the brief
  that change a shared number: DEFAULT, its `dimensions.md` row + one `log.md` line, and continue (never wait on Spark).

## 2. Design (system first, then cut)
- Load or build the part's system model (`docs/generative.md` The rule, 0): the assembled machine or sub-assembly, every
  neighbour and bought part in place, the system's load cases. Design and check the part there, never alone; its loads
  are the joint reactions from the system solve. A brief that touches several pieces of one system designs them as one and
  cuts them after.
- Drawn part: edit `nox/<machine>/<part>*.py` (manifold3d / cadquery bodies). Generated part: copy the pattern in
  `docs/generative.md` How-to (`nox/topo`, organic finish) or the part's existing `*_topo_*.py`; never `noxopt`. Heavy = hand it to the `run-part` agent (background, own worktree)
  and carry on with anything that does not need its mesh; it checkpoints, checks, and opens the PR itself. Several variants or
  parts in one brief = one worker each, started together.
- Every number in the script is tagged in the doc; nothing is copied from an older version.

## 3. Gate and checks (speed first: one blocking gate)
```
python3 nox/solid_check.py <mesh.stl>            # the only gate; FAIL = fix the design, never the mesh
python3 nox/print_orient.py <mesh.stl> --all     # saved face must be the largest bed contact, or say why
python3 nox/render_preview.py <mesh.stl> --mat PETG --prev <last g> --out runs/<part>/<version>/preview.png
```
The three commands, the strength recheck, the assembled-system check (fit, worst index, mass, CG) and the motion artifact only need the meshes: run them in parallel.
Strength recheck when the part carries a load case (pattern: `nox/box/arm_recheck.py`). Report, do not block.

## 4. Motion artifact (every part that moves or sits in a mechanism)
Use `nox/motion.py`: build a `Scene` from the part's meshes and its kinematics (`*_kinematics.py`, `*_kin.py`, or a
pose function written in the part script), one cycle of the mechanism, mating parts as context (`printed=False` for
bought parts). Write `runs/<part>/<version>/motion.html` (`Scene.write`) and `motion.mp4` (`Scene.video`). Publish the html with the
Artifact tool (icon "gear") and send the mp4 with SendUserFile. A static part gets `preview.png` only.

## 5. Done = saved (same turn, automatic)
Follow `docs/generative.md` "Save and print" exactly: `print/<part>_<version>.stl`, State READY and the File cell in
`current-parts.md`, one `log.md` line, `summary.md` in the fixed form beside `preview.png` and `motion.*`,
`python3 tools/runs_index.py`, commit (`<part> <version>: <what>`), push, `python3 tools/pr.py open "<part> <version>: <what>"`.

## 6. Review and merge (same turn)
Run the `review` agent on the PR. PASS: fix any findings worth fixing, push, then `python3 tools/pr.py merge <n>` (REST; `gh pr merge` fails in cloud boxes)
and `git checkout main && git pull`. FAIL: fix the design, re-review; never merge a FAIL. If the merge is refused by
permissions, say so with the PR link and stop; do not work around it.

## Reply (short)
The summary block, the artifact link, the PR link, any DEFAULT taken. Problems first, each with its fix.
