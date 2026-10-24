# One assembly file per system

Every tool reads the same part poses from one file per system: `nox/<machine>/<system>.assembly.json`, loaded with `nox/assembly.py`. No session builds a private model of a system any more (that is how a detent silently stopped clearing after a rack change).

| System | File | Owner (only writer) |
|---|---|---|
| Overseer | `nox/overseer/overseer.assembly.json` | Overseer session |
| Box catch | `nox/box/box_catch.assembly.json` | box catch session (funnel, hook latch) |
| Scout v3 | `nox/scout/v3/scout.assembly.json` (skeleton until the Scout v3 layout lands) | Scout v3 session |

A new system (box arm, stand, swap) starts its file the same way the first time its system pass runs.

## Format
The `nox/system_check.py` manifest (its docstring is the reference): `parts` with `name`, one geometry (`mesh` path relative to the file, the cwd or the repo root, or `box` / `cylinder` / `sphere`), `pose` (4x4 or `{"t", "rpy_deg"}`), `role` (cage / protected / structure / bought), `protect`, `material`, `mass_g` (bought parts must give it), `cg`, `tag`, `swap`; top level `name`, `cg_ref`, `thrust_axis`, `cg_tol_mm`, `cage_margin_mm`, `openings`, `contact_pairs`, `clearance_pairs`, `guard_offsets`. Added by `assembly.py`:
- part: `owner` (session / system), `version`, `source` (script that made the part or its pose), `moves` (`"module:function"` or `"path/file.py:function"`; `function(s, part)` returns the part's 4x4 world pose at state s, s = 0 is the file's pose), `note`.
- top: `machine` (box / scout / overseer), `note` (what is in, what is left out and why), and the reserved `access` and `probes` (system_check checks; kept untouched).
Unknown keys are kept on save. The file is written one part per line so a diff reads part by part.

## How a system session updates its file
1. Build or change the parts in their scripts as usual (meshes to `print/` or `runs/<part>/<version>/`).
2. Put each part's mesh path, pose, role, mass and tag in the file from the script that knows them (import the layout script, never retype its numbers): `a = assembly.load(f); a.set_pose(name, T); a.save()`, or rewrite the file from the system's layout script. A part whose pose is not derivable stays out and is named in `note`.
3. `python3 nox/assembly.py <file> --check --list`. The last line is the System line: `summary.md` System lines come from `assembly.py --check`, never from a hand-built manifest. Kinematic states: `a.at(**{name: s}).check()`.
4. Commit the file with the change in the same turn.

## Rules
- The system's owner session is the only writer of its file. Another session that needs a part moved puts it in its PR description, as for any part it does not own.
- Bought parts carry `mass_g` and `protect` where CLAUDE.md lists them (pack, cartridge, receiver, electronics, props, motors, camera).
- `contact_pairs` hold only designed joints (sockets, seats, press fits, a part inside its envelope), each one the owner can name; a real interference stays a FAIL.
- Every tool that needs the assembled system (system_check, the motion artifact via `.scene()`, renders via `.export_glb()` / `.export_stl()`, FE setups) loads this file.

`python3 nox/assembly_seed.py <system>` wrote the first files (24 Oct) from the layout scripts on main; it is history, not a second writer.
