# Overseer layout v5 / cage v3: sensor head outside the disk cage, mirror-exact, camera pair beside the L2 (26-27 Oct 2026, concept)

Spark 26 Oct: "The lidar needs to be outside the cage and not be blocked at all, try to give it enough support and guard is okay as long as it's outside the view of the lidar, also keep the two camera layout, also needs to not have its view obstructed, also why are there three orientations, I just need the one with perfect cg". Supersedes layout v4 / cage v2 (L2 inside the disk; the page showed the chosen layout with two alternatives beside it, which read as three orientations: this page shows one drone).
Spark 27 Oct (sic): "There is no limit for the tube length do t worry about that delete it / There needs to be perfect left and right symmetry the nose and tail can be different / Don't worry about tube length just create the structure don't worry about materials just stay to using 6mm and 3mm tubes". The concept review (FAIL) asked for: a camera guard, the L2 exemption cut to its field cone, the L2 mount braced into the core, a fuse in the mount, the rule text, the pack-on-axis numbers. All done below.
Spark 27 Oct, later: "There is not perfect symmetry in the cage right now, don't change the structure just make it symmetrical by adding rods, also you didn't keep the same two camera format we had before, make minimal changes but apply those". Structure kept: the same 48 nodes and 131 tubes (nodes moved ≤ 1.7 by the balance trim), the scan pinned to its pick. Symmetry: system_check finds every node and every tube with its mirror (nodes within 0.0 mm, 0 tubes without a mirror; links too), so no rod was missing to add; the page's motion yaws the drone, the top view at frame 0 (`preview_top.png`) shows the mirror. Cameras: both back exactly as v2 (front portrait on the horizon 74 up the L2 back plane, as it was; the down camera and flow pods back from v4's 35 forward to v2's place, lenses at v2's height).
Script `nox/overseer/layout_v5.py` (~10 min: balance x start / dome / prop-snap scan), artifact `layout_v5_view.py`. Rule: CLAUDE.md "Cages are geodesic shells formed to the drone" (field sensors outside), `docs/generative.md` rule 0 Geodesic cage, check `system_check` shell (closed + mirror) and field openings.
Spark 27 Oct, last: "get rid of the down camera we don't need it put the two cameras on either side the lidar pointing so their field of view is maximized". Done: no down camera (no pod), the two cameras beside the L2; below.

Visual: https://claude.ai/artifact/VhDfnp75uoVsj8SnyrW2Qg (source `runs/overseer_layout/v5/motion.html`, stills `preview.png` (iso) and `preview_top.png` (top, frame 0: the mirror); the L2 field (blue half-sphere outline) and the two camera views (green frustums) drawn)

System: Overseer, cage first 4.1 mm (E) at (+0.81, -0.26, +0.53) (camera right) (field cones open: Unitree L2 75 deg, out 66.63 mm, camera left 55 deg, out 7.21 mm, camera right 55 deg, out 7.69 mm), shell 133 rods closed, h/w 0.31, fit 0.2 mm³ (E) (spokes, ties, keels, L2 guard ring/cartridge), bbox 121 L (E), extents 761 x 694 x 230 mm (E), mass 3368 of 2200 g (153 %) (E) over budget (total), CG 0.2 mm off thrust axis (E) [-0.0, -0.2, -62.8], 32.0 mm without swap parts (reported, L exception), probes 7.1 mm (E) edge at (+0.68, +0.18, -0.71) (flow MTF-02P), access 19/19 clear

## Camera pair (Spark 27 Oct)
- **Down camera removed** with its pod; the flow sensor stays in its pod in the belly, 15 ahead of v2's (Spark 27 Oct).
- **Method:** every orientation of the mirrored pair (portrait / landscape, yaw 0..70 out, pitch -30..+30) ranked by the union of the two fields (the ask), then by the share of the L2 field they see (they colour its cloud); the first with a mount beside the L2 (lens 75..135 off the centre plane, -30..90 up the L2 base plane, the body 2 behind that plane so the L2 field stays clear, clear of the guard ring) where nothing of the drone is in view (L2, props, arms, apex, the other camera) and the shell target clears one of its view faces by NODE_R + 3 + 5 wins; that face then limits the shell like the L2 plane does.
- **Chosen:** portrait (73 across x 89 up), yawed 50.0° out, pitched 15° down; lenses (405, ±105, 64), 13 behind the L2 base plane. Combined field 3.45 sr, 27.5 % of the sphere: the two fields do not overlap, so this is the most a pair of these cameras gives; 55 % of the L2 field (C). Nothing of the drone in either view (C).
- **Alternative weighed:** landscape, yaw 47.5°, pitch -10: 3.44 sr, 54 % of the L2 field (orientation only); portrait wins by a hair on both.
- **Mount:** each camera on a printed plate behind its board, 2 rods to the low and high lug of its side (part of "apex + cam bracket"); rigid to the L2 for the camera-lidar calibration. The single front camera's guard bar is gone with it.
- **Cost:** the head is wider, so the L2 base moves 31 forward (x 414.8, was 384.0) and the motor square follows (72.4 ahead: front arms r 254, rear r 155); mass 3368 g (12 g less: the down camera and its pod out).
- **Cage first:** cameras 4.1 outside their 55° field cones (worst, up and forward of the right camera; 3 needed); each stands 7.2..7.7 out of the cage inside its cone (reported); probes pass.

## Layout (C on E sizes; `checks.txt`)
- As v4 (the v2 drone: L2 at the nose axis 15° below +x, motors forward, catch 30 aft, pack top z -7), but the L2, the two cameras and their mounts are left out of the shell.
- **L2 field clear:** every shell node centre 25.1 behind the L2 base plane (its 90° field); 0.0 % of the field crossed; the most forward cage member surface 3.0 behind the plane (guard ring nodes) (C).
- **L2 support and guard (outside its view):** v2 seat, back block, side lugs, 2 keels to the front hub, 4 ties into the shell, 4 braces from the lugs to the front arm beams 30 out of the root (a braced pyramid on the core). Guard ring r 62 round its axis, 8 behind its base plane, 8 rods + 16 struts into the shell (C). **Break order:** the apex lugs are the printed fuse, sized to let go before the L2 base (requirement, E).
- **Field cones:** in `system_check` the L2 is a field opening 75° on its axis (37.5 outside the cone; it stands 66.6 out along its axis) and each camera 55° on its own axis (diagonal half-angle 51 + 4); cage first gated everywhere else; probes skip field parts.
- **Shell, mirror-exact:** frequency 2 class-I geodesic, every node moved with its mirror partner, projected behind the view limits (the L2 plane, one face of each camera, the head plane; the worst limit per node each pass, the pair averaged, so the two camera limits keep the mirror); flat facets triangulated in mirror pairs; links mirrored. No tube-length limit. Start 0.85, dome 60, prop snap (the 27 Oct pick, pinned). 48 nodes, 133 tubes 43 .. 437, 641 x 673 x 209 (h/w 0.31), 52.2 L; largest cell Ø141; tubes 13.3 off the swept disks. system_check: 0 tubes without a mirror.
- **Balance:** motor square onto the CG with the shell rebuilt each time: CG 0.2 off (E).
- Props: shadow 7.8 % above, 14.4 % below (E).
- Opening, swap column: 0 mm³. Access 19 / 19 (FC and Pi USB-C, every arm M3; links kept off the screw key lines): the flow pod 15 ahead of v2's (Spark 27 Oct), so every rear bottom screw comes through a Ø9 hole in the sled, nothing unplugged.

## One layout ("the one with perfect CG")
CG 0.2 mm off the thrust centre with the pack in (system_check, E). With the pack + cartridge out it is 32.0 mm off: accepted by Spark 27 Oct ("as long as with battery the drone is centered we are okay"), the L Balance exception. The alternative (`V5_PACK=axis`, 27 Oct, before the camera pair): pack on the thrust axis, 0.6 mm pack out, front arms r 373, +263 g, T/W 1.81.

## Carbon rods (6 mm tube, 3 mm rod)
- Shell: 133 carbon tubes 6 mm, 22.8 m, 43 .. 437 (no length limit, Spark 27 Oct).
- Links: 63 carbon rods 3 mm, 8.2 m (spokes, core ties, L2 ties and braces, keels, bay ties, guard ring, camera plates to the lugs), shortest 33.

## Mass (E)
3368 g: shell 771 (tubes 555 + nodes 216), links 90. T/W 1.96 at 1650 gf a motor (C on E).

## Next
Frame model on the shell and the sensor head (crash 35 g in 14 directions incl. a nose hit on the L2 through its mount, a side hit on a camera, floor landing on the feet, handling), then nodes, arms, belly sled; new funnel round the shell (box catch session).
