Scout v3 electronics mount system on layout v13: stopped by Spark (28 Oct), checkpoint state. Not reviewed. Numbers are C/E from one dev run.

Done
- Round 3 FC decision coded (in belly_mount_v3.py, carried over from the v12 dev run): H-high DEFAULT. The board is hard on 3 M3 male-female aluminium standoffs, 12 mm stock length, over the front and side AIO holes (1.9, -18.03), (19.93, 0), (1.9, 18.03). The rear AIO hole stays empty. PCB z 34.0..35.6, board z 30.8..38.8. On v12 the standoffs cleared the tail block foot by 2.02.
  - Drawn: hex bodies, M3 nuts bonded under the plate, M3 x 6 button heads on top. The FC model has a keep-out round each hole, r 4.95 on top and 5.8 below.
  - The belly mount carries a nut relief (r 5.28) at the 3 holes. With the relief, VB's crash index rose to 0.742 (over the 0.67 limit), so on v12 the pick became "VB, rails 11.6": 30.2 g, hung set 151 Hz, crash 0.392.
  - Modes (E): rigid 3625+ Hz, PCB tail corner about 821 Hz. The 14.5 kHz figure from round 2 is dropped.
  - Isolator B is the alternative, as a layout sketch: D8xH8 40A at 45 deg, modes 114..177, margin 1.03, +24 g.
  - FILTERING DEFAULT: DSHOT_BIDIR_EN, IMU_GYRO_DNF_EN, IMU_GYRO_FFT_EN. Gate: the first-hover vibration log.
- Electronics tail mount v3: nox/scout/v3/electronics_v3.py, a driver over electronics_system.py. One run, 1518 s, output in runs/scout_electronics/v3/.
  - Model: catch +2.5, which is 7.5 aft of v12 (the data says 7.5, not 7.2). The FC is on top. Every piece is cut out of the wash except under the plate.
  - Picked V5 (round 5's variant: block walls 2.4/20 %, foot to z 34). Mount 51.6 g: tail block 17.4 g at (-49.1, 0.6, 13.9), carrier 34.2 g at (-109.6, 0.6, 9.6). Cables 6.5 g. CG 0.66 / 0.72 at +2.5. Modes 207 / 336 Hz.
  - L loads: crash 0.30, flight 0.01, landing 0.02, all pass. The yoke/block-before-carrier order misses in 2 crash x cases. The E 50 N case is 1.07.
  - The breaking-order line now uses v13's cage (rods 9.13 x Pcr, frame v4). Round 5's 0.39 line is reported, and V5 holds it.
  - Fit table for A: 33/33 pass.
  - Wash: Radxa +8.3, UBEC +22.9, XR2 +3.7, carrier +3.0, tail block -6.2 (still in the wash: the cutter applied, the checker disagrees; open). Funnel margins all >= 3.7.
  - Its system_check FAILs only on the old belly-mount placeholder against the FC nuts: 188.6 mm3. The placeholder needs the nut relief, or the real belly v3 replaces it.
  - The plate-bolt keys at (-42, +-16) are -1.26 from the rods/props. This is open.
  - Concept B's strut clashes the cage nodes. B is rejected anyway.

Unfinished
- belly_mount_v3.py was built by patch (it parses) but has never run. Its first run needs the electronics v3 envelope, which now carries a files map.
- Remaining work:
  - wash for the tail block and for the belly ears / rear receiver bosses (at x -50.15 and |y| 31.55 they are past the plate's shadow);
  - the electronics placeholder fit (nut relief);
  - the plate-bolt keys;
  - re-derive the standoff length against the v13 tail block foot;
  - fit, system_check and balance with all pieces;
  - the reproducibility pair of runs;
  - rows (dimensions Belly mount and FC rows, current-parts), a single log.md line, check_state, review_precheck;
  - the artifact update;
  - the summary.md files for runs/scout_belly_mount/v3 and runs/scout_electronics/v3.
- Plate holes, proposed single list:
  - existing: 3 AIO holes for the FC; the rear AIO hole, empty; the side holes (3.9, +-31) for the belly; the nose hole (45, 0), inside the new notch;
  - new: (-25, +-13) for belly VB; (-42, +-16) for the tail block; the layout's 4 tie-saddle holes (+-20, +-39.3);
  - the battery-lead notch is now x 38.15..51, |y| <= 6.2, r 1.5, because the XT60 cups moved to x 40.75.
- log.md: the merge dropped my 27 and 28 Oct lines (round 3 wants one line), and no new line was written yet.
