# Printing: orientation, saving, slicing, logging

## Orientation and supports (every part, every slice)
Two parts failed on 17 Oct for the same reason: the slicer was handed a part standing on thin posts (receiver v3.3 mouth down: 8 islands on the bed, two of 10.5 mm², 94 mm² total, M). Tree supports hold overhangs; they cannot hold a post that *is* the part.
1. **Largest flat face down.** Drawn mounting pads (plate bosses, bolt pads, contact faces) are that face on almost every drone part. A sticker like "mouth down" is not a reason; `python3 nox/print_orient.py <mesh>` is: it prints bed contact (mm², M), overhang area, first-layer islands and the thin-island run for the saved face and the 5 others. Pass = the saved face has the largest bed contact or the designer says why not, no first-layer island under 20 mm² that lasts more than 10 mm, and no free-standing post taller than 10 mm anywhere.
2. **Save the STL in that orientation** (the design session), and the Bambu project (`print/latest/<part>.3mf`) with it, so a print session never re-orients.
3. **Tree(auto) supports on, threshold 30°, global and per object; brim 5 mm outer + inner on anything under 400 mm² of bed contact.** Support g > 0 on any part with an overhang, or the slice goes back.
4. **Report with every slice:** bed face, bed contact (M), support g, time. A part that needs more than ~25 % of its mass in supports goes back to the designer for a better face or a split, not to the printer.
5. A print that fails for orientation is re-sliced, not redesigned; a print that snaps at a member is a design failure (`nox/log.md` failure log, `docs/rules.md` Attachments).

## Done = saved (automatic, no one asks for it)
A model is not finished, and "ready to print" is not said, until the same turn has done all of these:
0. `python3 nox/solid_check.py` PASS on the exact file being saved;
1. `print_orient.py` pass, STL saved on its largest flat face;
2. the print file at `print/<part>_<version>.<ext>` (zip if over 50 MB);
3. a copy at `print/latest/<part>.<ext>` (fixed name, no version, overwrites the old one) and that part's line in `print/latest/LATEST.md` rewritten (version, date, g, sha256 first 12);
4. its State in `nox/current-parts.md` set to READY with the file name in its File cell;
5. one line at the top of `nox/log.md`;
6. `runs/<part>/<version>/summary.md` in the form below, with `preview.png` beside it (`python3 nox/render_preview.py <mesh> --mat PETG|TPU --prev <last version's g>`);
7. committed and pushed, PR open.

A file that exists only in a chat, a cloud box or a SendUserFile card is not saved. Spark sees the preview image and this update before "ready to print" is said:
```
<part> <version>: ready to print
Weight: <g> (M), was <g> (+/- g)
Strength: worst index <x> in <case> (was <y>); weakest spot <where>   [drone parts; box parts: "fits" + support g]
Solid: solid_check PASS (1 body, 1 eroded body, 0 necks, 0 thin walls)
Print: <bed face>, bed contact <mm²> (M), tree supports <g>, <walls / infill>; file print/<name>
```

## Latest folder: print the latest from any session
`print/latest/` holds exactly one file per part under a fixed name (`box_hub.stl`, `box_baymod.stl`, ...) and `LATEST.md` (part, version, date, g, sha per file). A print session, before slicing: (a) `git pull` on the laptop clone, read `LATEST.md` and the Version cells in `nox/current-parts.md`; (b) slices only files in `print/latest/`, never a chat attachment; (c) runs `print_orient.py` on the file and re-orients in the project file (not the STL) if the saved face fails.

Bambu Studio on the laptop: never launch it while it is running (a launch makes a second copy); click the open window and close other project windows first. Type file paths, never paste them (other chats share the clipboard). Keyboard works when a window covers the buttons: Ctrl+R slice, Ctrl+G export sliced file, Ctrl+I import, Ctrl+O open. The rotate panel's typed fields do not take input from a remote session: set orientation by editing the item transform in the `.3mf` (`3D/3dmodel.model` and `Metadata/model_settings.config`, row-vector 3MF convention) and re-open it.

## Log every print
The moment a print is sent: one row at the top of `nox/print-log.md` (time, part + version, file, material, slicer time + g + support g, printer, result). Settings only if they differ from the profile. Same turn, the part's State in `nox/current-parts.md` becomes PRINTING. When Spark reports: PRINTED, then TESTED or FAILED (any session he tells writes it). A failed or cancelled print gets a new log row.

## After printing
- A printed version is frozen. Any change is a new version.
- Spark tests and weighs it (P); the P numbers go in the part doc, any break in the failure log in `nox/log.md`.
