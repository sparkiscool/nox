# Crash-flip (turtle) recovery for Scout (11 Oct 2026)

**Bottom line**
1. PX4 has no turtle mode for DShot ESCs: not in Scout's v1.18.0-rc1 (still the newest tag), release/1.18 or main (source read 11 Oct). Only ModalAI's voxl_esc driver has one. Forum requests (2023, 2025) got no maintainer answer; no PR found.
2. The parts exist. Bluejay and AM32 take DShot commands 20/21, a temporary reverse with no flash write; ArduPilot and Betaflight turtle use exactly these. PX4 sends only the saved commands 7/8.
3. Path (E): stay on PX4, add 20/21, flip with short disarmed motor-test bursts. The cage keeps props off the floor, but righting needs ~141 gf reversed per far motor and a two-stage roll.

## Findings
**PX4.** Set Spin Direction sends 7/8 plus a save, rewriting ESC flash each time [DShot.cpp]. CA_R_REV makes a motor reversible full time, for bidirectional flight. Main (post 1.18) adds hexarotor failure recovery that reverses a motor, assuming reverse thrust is 0.4 of forward [CA_REV_THR_FRAC]. rc1 sends commands with the telemetry bit clear, which Bluejay ignores; PR #28826 fixed this on release/1.18 (23 Sep), untagged. Commands go out only with all motors stopped. voxl_esc turtle copies Betaflight: AUX switch, 90 % cap, 15 % stick deadband [voxl_esc params]; users zeroed FD_FAIL_R/P (default 60°) to arm inverted [ModalAI forum]. Motor test runs only disarmed, yields to the kill switch, stops within 3 s [Commander.cpp].

**ArduPilot.** Turtle mode since Copter 4.2, May 2022 [release]. Needs DShot. Enter disarmed at zero throttle; throttle up arms and sends 21, throttle down disarms, exit sends 20; stick picks the motor pair. Throttle and GCS failsafes are off inside the mode [mode_turtle.cpp]. MicoAir743 board files exist [hwdef].

**Betaflight.** The crash-flip switch at arming skips the upright check and sends 21; any disarm sends 20 [core.c]. An axis stick spins two adjacent motors, a diagonal one. With crashflip_rate set (default 0), power fades after ~90° of attitude change [mixer.c].

**ESCs.** Bluejay acts after 6 identical frames, motor stopped. 21 flips the programmed direction; 20 or a power cycle restores it. Only 12 writes flash, and a 12 sent after 21 would store the reverse [Bluejay DShot.asm]. Temperature protection ships off [Bluejay settings]. AM32 treats 20/21 the same [AM32 dshot.c]. Stalled motors draw high current [Oscar Liang].

**Caged Scout (E).** Inverted, Scout sits on the 140 top ring, props ~85 mm up (C): no prop strike on a flat floor. But the flat top gives a 70 mm gravity arm, with the far motors 124 mm from the pivot edge (C). Each needs 0.28 mg reversed: 141 gf at 500 g, or ~350 gf forward-rated at a 0.4 reverse fraction (thrust-to-weight 2.8+). Scout first rolls onto its side, a stable rest for CG heights 25 to 45 mm. From there each far motor needs ~0.25 mg, pushing mostly sideways, so Scout slides below a floor friction of ~0.6; Skydio warns of sliding too. Thrust must hold to ~50° roll, past Betaflight's 90° fade. Corner pivots are worse (99 mm, C). A domed top rights itself passively [US9725158] but would block the top opening.

**Flyability, Skydio.** Flyability's US12134488B2 (granted Nov 2024) claims reversing the propellers far from an obstacle contact, in flight, to stop tipping; it never describes ground righting [Google Patents]. So the Elios flip recovery is most likely in-flight anti-tip (E). Skydio R10 ships ground turtle: a prompt at 15° tilt or more, the pilot picks a suggested direction, motors disarm once upright [Skydio].

## Steal for Nox
- Firmware: port PX4's own voxl_esc turtle mixer into the DShot driver, sending 20/21, never a save after 21.
- Firmware: flip with disarmed motor-test bursts (3 s cap), so Scout never arms inverted and FD_FAIL stays at 60°.
- Comms: the Radxa sends the bursts as ACTUATOR_TEST commands over uXRCE-DDS and suggests a direction from attitude; the pilot confirms (E).
- Scout: push over a side, never a corner; hold power to ~50° roll (E).
- Cage: draw the missing top bumpers as rounded TPU rockers to cut the 70 mm arm; grippy TPU on equator nodes (E).
- Scout v2 cage: make the inverted and side rest poses design cases; a smaller or rounded top cuts flip thrust (E).

## For Spark
- `scout.md` Firmware ("PX4 1.18.0-rc1"; "fixed upstream PR #28826"): the fix is on release/1.18, in no tag. Turtle needs a release/1.18 build, matched on the Overseer. Your call (E).
- `scout.md` As built ("FC + Bluejay ESC"): ArduPilot lists AM32 on the MicoAir743-AIO. Check which is flashed; both take 20/21 (E).
- `scout.md` ("KV not recorded") and `nox-dimensions.md` §1 ("500 g or a little more"): turtle hinges on ~141 gf reversed per motor (E). Measure it.
- `scout-cage.md` v10 Problem 1 ("Top bumpers still not drawn"): rocker shapes would also help the flip (E).
- Elios report ("flip it back upright"): probably in-flight anti-tip, per the patent (E).

## Recommended path for Scout (E)
- Firmware: PX4 release/1.18 plus a ~200-line DShot patch. Motors stopped: send 21 to the far pair, burst 40 to 70 % for 3 s max, send 20. Send 20 at boot; block arming until sent. Gates: disarmed, kill off, tilt over 60°, RC link up. Skip the no-code hack (Set Spin Direction plus motor test): it writes flash, and a motor left reversed flips the next takeoff. ArduPilot needs no code but breaks the Overseer match.
- ESC: keep the programmed direction and the motor 3/4 wire swap; no 3D mode; temperature protection on. With bidirectional DShot, abort a burst if eRPM stays near 0 for 0.3 s.
- Wiring: nothing new; one free switch (ch 6 or 7, CHECK) enables the flip.
- Tests: 1 props off: 21 reverses, 20 and a power cycle restore. 2 props on, strapped to a scale: reverse thrust at 40/55/70 %. 3 interlocks: kill, RC loss, 3 s cap, arm attempt. 4 cage on, inverted and side, carpet then smooth floor, 10 tries each. 5 motor and ESC temperature after 5 bursts.

## Sources
- [PX4 DShot driver (main)](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/dshot)
- [PX4 v1.18.0-rc1 tag](https://github.com/PX4/PX4-Autopilot/tree/v1.18.0-rc1)
- [PX4 release/1.18 branch](https://github.com/PX4/PX4-Autopilot/tree/release/1.18)
- [PX4 PR #28826](https://github.com/PX4/PX4-Autopilot/pull/28826)
- [PX4 voxl_esc turtle parameters](https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/actuators/voxl_esc/voxl_esc_params.yaml)
- [PX4 control allocator parameters](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/control_allocator/module.yaml)
- [PX4 main release notes](https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/releases/main.md)
- [PX4 Commander.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/Commander.cpp)
- [PX4 forum: turtle mode feature request (2025)](https://discuss.px4.io/t/flip-after-crash-turtle-mode-feature-request/48112)
- [PX4 forum: direct motor control for turtle mode (2023)](https://discuss.px4.io/t/direct-motor-control-for-turtle-mode/30559)
- [ModalAI forum: how to enable Turtle Mode](https://forum.modalai.com/topic/3937/how-to-enable-turtle-mode)
- [ArduPilot Turtle Mode](https://ardupilot.org/copter/docs/turtle-mode.html)
- [ArduPilot mode_turtle.cpp](https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/mode_turtle.cpp)
- [Copter 4.2.0 release](https://discuss.ardupilot.org/t/copter-4-2-0-has-been-released/85859)
- [ArduPilot MicoAir743-AIO hwdef](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL_ChibiOS/hwdef/MicoAir743-AIO)
- [Betaflight core.c](https://github.com/betaflight/betaflight/blob/master/src/main/fc/core.c)
- [Betaflight mixer.c](https://github.com/betaflight/betaflight/blob/master/src/main/flight/mixer.c)
- [Bluejay DShot.asm](https://github.com/bird-sanctuary/bluejay/blob/main/src/Modules/DShot.asm)
- [Bluejay settings](https://github.com/bird-sanctuary/bluejay/blob/main/src/Settings/BluejaySettings.asm)
- [AM32 dshot.c](https://github.com/am32-firmware/AM32/blob/main/Src/dshot.c)
- [Turtle mode setup (Oscar Liang)](https://oscarliang.com/setup-turtle-mode-flip-over-after-crash/)
- [US12134488B2, Flyability (Google Patents)](https://patents.google.com/patent/US12134488)
- [US9725158 self-righting frame (Justia)](https://patents.justia.com/patent/9725158)
- [Skydio R10 Turtle Mode](https://support.skydio.com/hc/en-us/articles/50098097408155-How-to-use-Turtle-Mode-with-Skydio-R10)
