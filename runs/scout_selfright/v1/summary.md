# Scout self-righting (27 Oct 2026, research)

Spark: "based on the drone designs is there a way for us to flip it over with the props when its landed backwards? the motors cannot spin backwards".

**Answer: not with one-way props.** Their thrust always points out of the drone's top.
- Upside down on the flat roof, it presses the drone into the floor.
- On its side, it is a sideways push above the floor contact, which rolls the drone further over, onto its back.

**The current cage (v11) rests stably in 6 poses (C):** upright, upside down (CG 63 above the roof), on either side (CG ~119 from the side faces) and on the two rear-quarter faces. Check: each hull face, cage nodes plus the camera cradle, whether the CG's foot falls inside it; CG from system_check.

Ways out:
1. **Turtle mode (reverse spin):** no added weight or parts.
   - The ESC must reverse in firmware. dimensions.md lists an AM32 ESC on the Overseer; the Scout's MicoAir743v2 AIO needs a CHECK.
   - PX4 (the Scout's firmware) has no turtle mode; ArduPilot does.
2. **Flip arm:** a servo lever on the roof, ~15 to 20 g E, plus one servo channel.
3. **Self-righting cage:** the only stable pose upright.
   - Needs a roof peak plus sides that lean out at the top: about ±170 at the top against ±120 now (E, 2D: the CG foot must fall below the side face's bottom edge).
   - A wider, heavier cage, and the race-car shape goes.
   - A roof peak alone only moves the stable pose to the side.

Picture: https://claude.ai/artifact/VfUix53fmkUNJBcrhR1SD5 (`selfright.html`).

## Firmware check (27 Oct, Spark: "when i tried in the app they couldent spin backwards so i had to resolder wires")
- **The ESC can reverse.** The Scout's MicoAir743 AIO 35A runs AM32 (vendor build AM32_MICOAIR_743_AIO_F421 2.17; one listing says 2.12, so check what is flashed). It takes DShot300/600, bidirectional DShot (telemetry only) and the DShot direction commands.
- **Why the app could not reverse it:** MicoAir's AM32 guide sets the ESC through the am32.ca configurator over the FC's USB. It notes that PX4 has no ESC passthrough, so with PX4 on the FC the configurator cannot reach the ESC.
- **Without resoldering on PX4:** the MAVLink console's `dshot reverse -m <motor>` then `dshot save -m <motor>` sets a motor's direction for good. This is PX4's dshot driver (from memory, E; check on the bench).
- **Turtle mode (reversing in flight to flip):**
  - PX4 has none. A 2025 feature request on the PX4 forum went unanswered. PX4's own reversible output (DSHOT_3D_ENABLE with the ESC in 3D mode) changes the throttle range and is not a flip routine.
  - ArduPilot Copter has Turtle mode, which needs DShot ESCs that take the reverse command; AM32 does.
  - ArduPilot also runs on the MicoAir743 AIO and has optical-flow position hold.
- **Choice for Spark:** move the Scout to ArduPilot for turtle mode (a firmware change and retune; PX4 flies it well now), or stay on PX4 and add the flip arm (~15 to 20 g). A custom PX4 turtle module is possible, but it is code nobody maintains.
- Sources: micoair.com/docs/how-to-configure-and-update-am32-esc-on-micoair743-aio/, micoair.com/flightcontroller_micoair743_aio_35a/, ardupilot.org/copter/docs/turtle-mode.html, ardupilot.org/copter/docs/common-MicoAir743-AIO.html, discuss.px4.io/t/flip-after-crash-turtle-mode-feature-request/48112, docs.px4.io/main/en/config/motor_failure_recovery.
