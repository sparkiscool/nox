# Retired: Scout cage nodes v2

Superseded by `runs/scout_nodes/v3/` (28 Oct): v2 was drawn on layout v12; its 192 g of nodes moved the CG 4.7 forward, so the layout was re-trimmed to v13 (PR 169) and the nodes were redrawn on it, with the bell mouths and collars only where the FE asks for them (the review's mass target). Never printed, never on main.

Kept: summary.md, result.json, previews, sections, logs, nodes/*.json, envelopes.json, print_checks.json, offsets.json. Meshes, placed meshes and the page source were removed; recover them with
`git checkout 34997971 -- runs/scout_nodes/v2 print/` (the commit that saved v2).
