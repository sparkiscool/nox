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
