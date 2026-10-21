# Nox: doc map and rule book (rewritten 13 Oct 2026; cleanup 14 Oct 20:29; simplified 16 Oct 17:40; cloud sessions 16 Oct 18:00; split work 16 Oct 21:11; done = saved 17 Oct 15:10; one live status 17 Oct 15:40; latest folder 17 Oct 15:45; every session 17 Oct 15:55; orientation + supports 17 Oct 18:30; code moved to Nox Source 18 Oct 04:30; no disconnected structures, `solid_check.py` a hard gate, 20 Oct 01:30; neck test added 20 Oct 14:45; **everything moved into the git repo `sparkiscool/nox`, Claude Code cloud sessions, 21 Oct**)

Spark's drone project. Current work: **Scout v2** (lightest caged quad, battery under the drone in a cartridge held by the S2 bottom latch, contact pads) and the **box** (`box.md`: static printed funnel on top, electromagnet hold, and under it a vertical swap wheel of charging bays with a two-head radial arm). **Overseer** (2.0 kg S500, lidar) is paused; its hardware is in `v2/dimensions.md` section O. DubHacks (17 Oct): `dubhacks-queue.md`.

**No old designs exist in this project.** Scout v1 and every retired box and swap mechanism were deleted 14 Oct; there is no archive. Never reintroduce a slide swap, slider, bridges / pusher, turret or ferris head, turntable, flat carousel, groove seat, Z stage or top-mounted battery unless Spark asks for it fresh.

The short version of this page is `CLAUDE.md` at the repo root, loaded in every session. This page holds the detail.

## Every session (Spark 17 Oct 15:37: every session works on the latest designs, and every design change reaches every session)
- **Start:** `git pull`. Read your part's row and the rows of the parts it touches in `v2/current-parts.md` (State, Version, Saved file). That is the newest design; a version in your own memory, in a chat attachment or in a local file is older unless that table says so. Before any fit check, pull `main` so the mating parts' current scripts are the ones on disk.
- **Every turn that changes a design:** same turn, no exceptions, even when the file is also on the laptop or the SD card: the changed scripts, the print file in `print/` (Printing, Done = saved), your State / Version / Saved file cells, one line in `v2/log.md`, all committed and pushed. Local copies (the laptop's clone, SD card, downloads) are mirrors; `main` on GitHub is the design.
- **Someone else's part:** never edit its file or row; put what it must change in your PR description (and, when Spark runs chats by hand, in a paste-ready prompt for him; Split the work).
- A session that ends without pushing left work nowhere; the next session cannot see it.

## Where files live (18 Oct: the claude.ai Project filled its 2 MB cap; 21 Oct: one git repo)
- **Everything is in the repo `sparkiscool/nox`, branch `main`**: docs (`.md`), scripts, configs, research notes, params, engine, run results (`runs/`), print files (`print/`). Paths are the ones the Project and the old "Nox Source" artifact used (`nox/v2/receiver_v33.py`, `nox/box/arm_cad.py`, `claude/overseer_params_2026-09-26.params`, ...).
- **Fetch:** the session's clone. `git pull` at the start of every turn that reads a shared file.
- **Save (same turn as the change):** `git add`, `git commit -m "<part> <version>: <what>"`, `git push`. A rejected push means another session pushed first: `git pull --rebase`, resolve, push again. Then one line in `v2/log.md`.
- **Big files:** no LFS (blocked from cloud boxes). Meshes, previews and print files go in plain git; any single file stays under 50 MB (decimate viewing meshes, zip a big 3mf).
- **Laptop mirror:** the CAD laptop holds a clone at `Documents\Nox`. A linked chat runs `git pull` there before slicing; nothing is edited on the laptop alone.
- The claude.ai Project "Nox" keeps only this README as a pointer; no doc there is current.

## Reading (Spark 14 Oct: usage runs out too fast)
- **Nothing is a must-read, this page included,** except the Every session start line above and `CLAUDE.md` (auto-loaded). Read only what the session's request needs: a CAD session reads its part's row in `v2/current-parts.md`, its part doc and the `v2/dimensions.md` section it uses; a research session reads no CAD or part docs.
- No research passes or subagents unless Spark asks. One session per part; Spark starts a fresh one when it gets long.

## Split the work (Spark 16 Oct 21:11; Claude Code Projects 21 Oct)
- **At the start of every task, and whenever a run would wait on another, split the work into independent pieces.** Each piece is its own cloud session on the repo, so pieces run at the same time on their own machines.
- **How pieces are started:** the Nox control conversation (claude.ai/code → Projects → Nox) starts one thread per piece and watches them on its Overview tab. Without that conversation, Spark pastes a prompt into a new session himself (`claude --cloud "<prompt>"` from any clone, or a new session at claude.ai/code on the repo).
- One piece = one part, or one heavy run (one generative run per machine).
- Pieces that write the same doc rows or the same shared number stay in one session.
- A prompt holds: the goal, the files to read, the exact commands, what to report back, and the numbers it needs (tagged). Short, in a code block, ready to paste.
- The session that splits says which piece it kept and doesn't touch the others. Each piece ends in its own PR; the control conversation merges them.

## Live status (17 Oct 15:40)
- **A part's state is written in exactly one cell: its State cell in `v2/current-parts.md`.** No other doc restates it: the queue holds print order, part docs hold design and test results, the print log and `v2/log.md` are history. A doc that disagrees with that cell is stale.
- State words, nothing else in the cell: DESIGN · BUILT (passes the Done checks, file not saved) · READY (file committed in `print/`, the only proof) · PRINTING · PRINTED (not tested) · TESTED · FAILED · HOLD · ON HAND.
- "Ready to print" without a file in `print/` on `main` is BUILT, whatever any doc says.

## Docs
| Doc | Holds |
|---|---|
| `CLAUDE.md` (repo root) | the short rule book every session loads; this page is the long one |
| `box.md` | **the box design**: funnel, magnet hold, keep-alive, swap wheel, sequence, buy list |
| `v2/current-parts.md` | every current part (Scout v2 and box): version, **live state (the only place)**, saved file, next; open problems; waiting on Spark |
| `v2/dimensions.md` | the only source for numbers, by section (A bought hardware, C shared, E cartridge / contacts, G guard / funnel, H underside, I box swap wheel, J bottom latch, O Overseer) |
| `v2/README.md` | Scout v2 scope, loads, materials, optimizer settings, mass budget |
| `v2/log.md` | change and failure log, one line each |
| `runs/<part>/<version>/` + `runs/index.md` | every heavy run's result: `result.json`, `summary.md`, `preview.png`, `log.txt`, mesh; the board is rebuilt by `tools/runs_index.py` |
| `v2/box-funnel.md`, `v2/s2-bottom-latch.md`, `v2/s2-cartridge.md`, `v2/s2-contacts.md`, `v2/s2-cage-v2.md`, `v2/s2-top-case.md`, `v2/s2-receiver.md`, `v2/s2-layout.md` | one doc per part; scripts beside them (`s2_cartridge.py`, `s2_blade.py`, `tray_to9.py`, `cage_v2/`, `s2_top_case/`, `s2_camera/`, `catch_v4/`, `lantern/`, `receiver_v34.py`). The camera mount has no part doc yet. The latch coupon is dropped (17 Oct 15:50): the latch is tested on the real cartridge |
| `box-arm.md`, `box-case.md`, `box-stand.md`, `box-buy-list.md`, `box-wiring.md` | box swap arm, bay modules, stand, enclosure; box buy list; wiring. Scripts in `box/` (`arm_layout.py`, `arm_cad.py`, `arm_topo.py`, `arm_run_*.py`, `arm_recheck.py`, `belt_clamp.py`, `detent.py`, `stand/`) |
| `nox-print-log.md` | every print job, newest first (history) |
| `dubhacks-queue.md` | DubHacks print order, fast profile, print notes |
| `print/<part>_<version>.<ext>` | **every print-ready file** |
| `print/latest/<part>.<ext>` + `print/latest/LATEST.md` | **the newest file of every part, one per part, fixed name.** The only folder anyone slices from |
| `nox-generative.md` | how to run a generative part |
| `noxopt.py` | optimizer. `noxopt_demo.py` = reference pipeline, every part script starts as a copy of it (it imports `noxopt_bench.py`). `test_noxopt.py` = self-test. `printopt.py` = FE kernel and strength check. `noxopt_local.py` = multigrid / pardiso backend for big fine runs. `interfaces.py` = drawn-feature library (cadquery), `reconstruct.py` = density to one exported body (older pipeline) |
| **`solid_check.py`** | **strict connectivity + thin-wall + neck check of a mesh (20 Oct; neck test 14:45): one body after a 0.9 mm erosion, every drawn feature in it, no wall under 1.8, no piece hanging on a neck under 3 mm. Hard gate on every part (Rules, Solid parts)** |
| `render_preview.py` | print preview image, required before "ready to print" |
| `print_orient.py` | print-orientation check of a mesh (bed contact, overhang, free-standing islands per face); run before every save and every slice (Printing, Orientation) |
| `scout.md` | Scout firmware, radio, wiring (hardware reference, not a design doc) |
| `claude/research-queue.md`, `claude/research/*.md`, `claude/*-research.md` | background research, not maintained; may describe mechanisms since dropped. Read only when the topic comes up; never treat one as a current design |
| `claude/overseer_params_2026-09-26.params` | Overseer tune backup. Restore after any airframe reset. Never delete |
| `claude/scout_B9_B10_2026-10-04.params` + `.txt` | Scout B9 flow / range offsets + B10 speed caps (QGC format) and its notes; not loaded yet |
| `cloud/setup.sh`, `cloud/session_start.sh`, `.claude/` | the cloud environment setup, the per-session hook, the agents (`run-part`, `review`) and settings |

## Cloud sessions (Spark 16 Oct: the whole project runs in the cloud, from any device; 21 Oct: Claude Code cloud on the repo)
- **The repo is the only source.** Nothing depends on a file that exists only on a PC, in a chat or in an artifact. A new or changed script is committed and pushed the same turn (Where files live).
- **Environment:** the Claude Code cloud environment "Nox" (claude.ai/code → Environments) runs `cloud/setup.sh` once and snapshots the box, so the engine's packages are already installed; `cloud/session_start.sh` reinstalls them if a box lacks them and prints the box's cores and RAM. Nothing to unpack. The old "Nox Engine Bundle" is retired; the engine files in `nox/` are the current ones (newer than the 16 Oct bundle).
- **Engine change:** edit, run `python3 nox/test_noxopt.py`, commit, one line in `v2/log.md`.
- **Files:** a run's meshes, checkpoints and logs go in `runs/<part>/<version>/` and are committed with every checkpoint, so a reclaimed box loses nothing. A print-ready file goes into `print/` the same turn (Printing, Done = saved); a cloud box and the chat itself are not storage.
- **Limits:** a Claude Code cloud box is listed at ~4 vCPU, 16 GB RAM, 30 GB disk (docs 20 Oct; the chat boxes were 2 cores / 7 GB, P). Measure on the first run and tag it; keep peak under 75 % of RAM and run nothing else heavy beside a generative run. A run lives only while a command runs (start it in the background, checkpoint every ~10 iterations, commit each checkpoint); hard cap 6 h. One generative run per box (split the rest, section above). Generative runs above ~60k design cells: `noxopt_local` with `backend="gmg"` (pardiso died at 6 GB in the 7 GB boxes, 20 Oct).
- **Control:** one conversation at claude.ai/code (Projects → Nox) starts and watches every thread. A heavy run uses the `run-part` agent; a PR gets the `review` agent before merge. Scheduled or on-demand reruns are routines (claude.ai/code → Routines).

## Rules (Spark)
- **Speed first.** No blocking gates, **except one: the solid check (next rule).** Print, test, iterate. Every other check is reported, never blocks.
- **Solid parts, no disconnected structures, ever (Spark 20 Oct 01:08: "make sure that is always true moving forward").** No part is saved, published, called BUILT or READY, or shown as a result unless `python3 nox/solid_check.py <mesh>` reads **PASS** on the exact file: (a) one body; (b) still one body after a 0.9 mm erosion, so nothing hangs on an edge, a corner, a sliver or a neck under ~1.8 mm; (c) every drawn feature (pads, cheeks, plates, slot walls, lugs) inside that eroded body, passed in as `drawn=`, and each drawn body still joined when eroded to 0.4 x its own thickness; (d) no wall under 1.8 mm anywhere (holes and nut pockets included: a hole needs ≥ 2.0 of material all round); **(e) neck test (20 Oct 14:45): eroded again at 1.2 and 1.5 mm, no piece may split off that is ≥ 1.6 mm thicker than the neck it hung on and joins the rest over less than 4 x its own thickness² (so a 5 mm pad on a 2..3 mm neck fails; a thin-walled box joined round its whole perimeter does not).** A plain body count is not enough: bracket_e v8 (20 Oct) was "1 body" while each cheek hung on two 0.85 mm ligaments beside its M5 hole, and its topology rerun passed (a)..(d) while both bolt pads hung on 2..3 mm necks (Spark 14:19 "why is the bracket disconnected"). A FAIL is fixed in the design (thicker feature, moved hole, generated material allowed to reach the feature), never patched on the mesh. Generative runs carry it built in (`arm_topo.run_part` result `solid_ok`; `stand_v7_topo_build.py`); drawn parts call it at export (`stand_v7_parts.py`). Bolt heads, nuts and their tools also need clear access; a generative setup cuts an access cylinder for every fastener, **never wider than the drawn pad it serves: draw a boss wall ≥ 3 round any counterbore or access cut, so material can reach the pad from behind.**
- **Simplest design wins.** Fewer parts, features and steps beat clever ones; justify any added complexity.
- **Every design decision comes from an optimization.** Nothing goes in a spot just because it has to be somewhere.
- **Tag every number:** P measured on the part, M mesh, C CAD, E estimate, F frame model.
- **Ask Spark** only on costly or hard-to-undo choices, or before changing a shared number. Otherwise decide, mark DEFAULT, continue. Every question comes with its best answer, one short line.
- **Replies short.** Each problem plainly, with the best fix.
- Drone parts: lightest that meets the loads. Box parts: strong and fits, mass not a target.
- Materials: PETG for rigid parts, TPU for nodes, feet, bumpers, PLA out (v2 detail in `v2/README.md`). Rods epoxied, never press fit.
- Nothing printed thinner than 2.0 mm; flexures excepted.
- Printer: Bambu A1 mini (180 cube). **Every part prints on its largest flat face with tree supports (Spark 17 Oct 16:36 and 18:05; Printing, Orientation and supports).** The bed face is picked before the run and the STL is saved in that orientation; the 45° overhang filter in `noxopt` is optional (a support costs grams, not a failed print).
- Crash behaviour is proven by physical test, not simulation. Buy and modify over building from scratch. Goal: investor-demo prototype.

## Generative parts
How-to and defaults: `nox-generative.md`. Every part runs the whole `noxopt_demo.py` pipeline, in order, nothing skipped. A part script is a copy of the demo with the part's own loads, drawn features, cuts and envelope.
1. **Drawn = mounting + must-haves only:** fastener seats, mating and contact faces, pin bores, rod sockets, connector seats; flexures, hooks, lens and sensor windows, TPU feet. Each at its smallest volume. Everything else is generated. Remove any other drawn geometry before the run and say what was removed. Attachments to the plate, a bolt, a rod or a mating part follow `v2/current-parts.md` Problem 46 (pads ≥ Ø12, posts ≥ 6 x 6, gussets). **Every drawn body lies inside the envelope (the build clips to it), and the design space must be able to reach each drawn body over its whole length, not only at a tip.** A bolt pad includes its boss: the wall round the bolt head's counterbore is drawn, not left to the generator.
2. **Layout:** stiffness run to a mass target, every load case plus a tie load on every drawn feature. Members come out at the filter radius (at least 2.0 mm and 2 cells). Symmetric mounting gives a symmetric part: centre the grid on the mirror plane, build one half and mirror. Pick the bed face first (Printing, Orientation and supports): the drawn mounting pads are usually it.
3. **Stop** at ≤ 2 % grey and settled. Hard cap 6 h, then build from the last checkpoint (every ~10 iterations, each committed). All runs in the cloud, one per session (Cloud sessions, Limits); too big means coarser or in pieces. Print run progress to `runs/<part>/<version>/log.txt` and report only the final numbers.
4. **No thin rods:** `prune_thin` after the layout; `thin_check` reads 0 after the build. A thin member goes back to `prune_thin`, never thickened by hand.
5. **Build:** `smooth_surface` polish (fillets where members meet, holes stay holes, nothing under 2.0, off every keep-out), then exact drawn bodies, envelope trim, exact cuts. Mounting features are never smoothed. `function_check` reads 0 / 0 / 0. Report `wrinkle`; look at close-ups; lumpy means rebuild.
6. **No patching:** no grey fill, dilation, member growing or "keep the largest body". One body holding every drawn feature; a detached feature means rerun. **`solid_check` PASS (Rules, Solid parts) or the part does not exist.**
7. **Strength:** as-built recheck of every case (`printopt.py` layer-aware index). A must-hold case over 1.0 gets local reinforcement, at most 3 passes; a remaining miss is noted. **A topology part replaces a drawn one only if it beats the drawn part's worst index on the same FE (same grid) and passes the solid check; the FE grid must be at most a third of the smallest neck it has to judge.**

Done when: **`solid_check` PASS**, one watertight body, interfaces exact, function check 0 / 0 / 0, thin check 0, fit vs mating parts, wall check, smooth, mass vs last version, `print_orient.py` pass (below).

## Printing
- **Orientation and supports (Spark 17 Oct 16:36 supports everywhere, 18:05 well-supported parts are the default; rule written 18:30).** Two parts failed on 17 Oct for the same reason: the slicer was handed a part standing on thin posts (receiver v3.3 mouth down: 8 islands on the bed, two of 10.5 mm², 94 mm² total, M). Tree supports hold overhangs; they cannot hold a post that *is* the part. Rule, every part, every slice:
  1. **Largest flat face down.** Drawn mounting pads (plate bosses, bolt pads, contact faces) are that face on almost every drone part. A sticker like "mouth down" or "right side down" is not a reason; `python print_orient.py <mesh>` is: it prints bed contact (mm², M), overhang area, first-layer islands and the thin-island run for the saved face and the 5 others. Pass = the saved face has the largest bed contact or the designer says why not, no first-layer island under 20 mm² that lasts more than 10 mm, and no free-standing post taller than 10 mm anywhere.
  2. **Save the STL in that orientation** (the design session), and save the Bambu project (`print/latest/<part>.3mf`) with it, so a print chat never re-orients.
  3. **Tree(auto) supports on, threshold 30°, global and per object; brim 5 mm outer + inner on anything under 400 mm² of bed contact.** Support g > 0 on any part with an overhang, or the slice goes back.
  4. **Report with every slice:** bed face, bed contact (M), support g, time. A part that needs more than ~25 % of its mass in supports goes back to the designer for a better face or a split, not to the printer.
  5. A print that fails for orientation is re-sliced, not redesigned; a print that snaps at a member is a design failure (`v2/log.md` failure log, Problem 46).
- **Done = saved (Spark 17 Oct 14:59; latest folder 15:34; git 21 Oct). Automatic: no one asks for it.** A model is not finished, and "ready to print" is not said, until the same turn has done all of these: (0) **`solid_check` PASS on the exact file being saved**; (1) the print file at `print/<part>_<version>.<ext>` (zip if over 50 MB); (2) a copy at `print/latest/<part>.<ext>` (fixed name, no version, overwrites the old one) and that part's line in `print/latest/LATEST.md` rewritten (version, date, g, sha256 first 12); (3) its State in `v2/current-parts.md` set to READY with the file name in its Saved file cell; (4) its row in `dubhacks-queue.md` names the file (order and print notes only, no state); (5) one line at the top of `v2/log.md`; (6) `runs/<part>/<version>/summary.md` + `preview.png`; (7) **committed and pushed, PR open.** A file that exists only in a chat, a cloud box or a SendUserFile card is not saved.
- **Latest folder (Spark 17 Oct 15:34): print the latest from any session.** `print/latest/` holds exactly one file per part under a fixed name (`s2_tray.stl`, `s2_cartridge_block.stl`, `s2_cartridge_plate.gcode.3mf`, `s2_camera_mount.stl`, ...) and `LATEST.md` (part, version, date, g, sha per file). A print chat, before slicing: (a) `git pull` on the laptop clone, read `LATEST.md` and the Version cells in `v2/current-parts.md`; (b) slices only files in `print/latest/`, never a chat attachment; (c) runs `print_orient.py` on the file and re-orients in the project file (not the STL) if the saved face fails. Bambu Studio: never launch it while it is running (a launch makes a second copy); click the open window and close other project windows first. Type file paths, never paste them (other chats share the clipboard). Keyboard works when a window covers the buttons: Ctrl+R slice, Ctrl+G export sliced file, Ctrl+I import, Ctrl+O open. The rotate panel's typed fields don't take input from a remote session: set orientation by editing the item transform in the `.3mf` (`3D/3dmodel.model` and `Metadata/model_settings.config`, row-vector 3MF convention) and re-open it.
- **Log every print** the moment it is sent: one row at the top of `nox-print-log.md` (time, part + version, file, material, slicer time + g + support g, printer, result). Settings only if they differ from the profile. Same turn, the part's `v2/current-parts.md` State becomes PRINTING. When Spark reports: PRINTED, then TESTED or FAILED (any chat he tells writes it). A failed or cancelled print gets a new log row.
- **A printed version is frozen.** Any change is a new version.
- **Ready to print:** never said until Spark has seen the preview image (`python render_preview.py <mesh> --mat PETG|TPU --prev <last version's g>`, committed as `runs/<part>/<version>/preview.png` and sent with SendUserFile when he is in the chat) and this update:
```
<part> <version>: ready to print
Weight: <g> (M), was <g> (<+/- g>)
Strength: worst index <x> in <case> (was <y>); weakest spot <where>   [drone parts; box parts: "fits" + support g]
Solid: solid_check PASS (1 body, 1 eroded body, 0 necks, 0 thin walls)
Print: <bed face>, bed contact <mm²> (M), tree supports <g>, <walls / infill>; file print/<name>
```
- After printing: Spark tests and weighs it (P); any break goes in the failure log in `v2/log.md`.

## Editing docs (several sessions write at once)
1. `git pull` right before editing; change only your own rows; keep the edit small; commit and push at once.
2. Changing a number: its row in `v2/dimensions.md` plus one line at the top of `v2/log.md`. Superseded values leave the docs.
3. One rule, one place; one doc per part. Live docs hold current state only; a retired design is deleted, not described.
4. One fact, one cell: a part's state only in `v2/current-parts.md`; a number only in `v2/dimensions.md`. Other docs link, never copy.
