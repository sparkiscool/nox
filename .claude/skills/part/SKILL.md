---
name: part
description: One prompt from a design brief to a saved Nox part with its motion artifact. Use when Spark names a part and what to change ("part hub v6: ...", "iterate the detent arm", "build rev N of ...").
---

# /part <part> <version>: <brief>

One session, one part, start to finish, following CLAUDE.md. Research and decisions are already made: the brief
carries them. Everything below runs in this cloud box; Spark's machine only opens the results.

## 1. Load (read only what the row names)
- `git pull` on `main`; `git checkout -b part/<part>` (or `run/<part>-<version>` if the brief needs a generative run).
- The part's row in `nox/current-parts.md`, its part doc, the `nox/dimensions.md` sections it names. Numbers in the brief
  that change a shared number: ask Spark in one line with the best answer, otherwise DEFAULT and continue.

## 2. Design (the part's own script, nothing else)
- Drawn part: edit `nox/<machine>/<part>*.py` (manifold3d / cadquery bodies). Generated part: copy the pattern in
  `nox/noxopt_demo.py` or the part's existing `*_run_*.py`; heavy = hand it to the `run-part` agent and stop here
  (it checkpoints, checks, and opens the PR itself).
- Every number in the script is tagged in the doc; nothing is copied from an older version.

## 3. Gate and checks (speed first: one blocking gate)
```
python3 nox/solid_check.py <mesh.stl>            # the only gate; FAIL = fix the design, never the mesh
python3 nox/print_orient.py <mesh.stl> --all     # saved face must be the largest bed contact, or say why
python3 nox/render_preview.py <mesh.stl> --mat PETG --prev <last g> --out runs/<part>/<version>/preview.png
```
Strength recheck when the part carries a load case (pattern: `nox/box/arm_recheck.py`). Report, do not block.

## 4. Motion artifact (every part that moves or sits in a mechanism)
Use `nox/motion.py`: build a `Scene` from the part's meshes and its kinematics (`*_kinematics.py`, `*_kin.py`, or a
pose function written in the part script), one cycle of the mechanism, mating parts as context (`printed=False` for
bought parts). Write `runs/<part>/<version>/motion.html` (`Scene.write`) and `motion.mp4` (`Scene.video`). Publish the html with the
Artifact tool (icon "gear") and send the mp4 with SendUserFile. A static part gets `preview.png` only.

## 5. Done = saved (same turn, automatic)
Follow CLAUDE.md "Done = saved" exactly: `print/<part>_<version>.stl` + `print/latest/`, `LATEST.md`, State READY in
`current-parts.md`, one `log.md` line, `summary.md` in the fixed form beside `preview.png` and `motion.*`,
`python3 tools/runs_index.py`, commit (`<part> <version>: <what>`), push, `gh pr create --fill --base main`.

## 6. Review and merge (same turn)
Run the `review` agent on the PR. PASS: fix any findings worth fixing, push, then `gh pr merge <n> --merge --delete-branch`
and `git checkout main && git pull`. FAIL: fix the design, re-review; never merge a FAIL. If the merge is refused by
permissions, say so with the PR link and stop; do not work around it.

## Reply (short)
The summary block, the artifact link, the PR link, any DEFAULT taken. Problems first, each with its fix.
