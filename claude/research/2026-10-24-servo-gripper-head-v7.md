# Servo gripper and head research for Nox head v7

Search was a web search tool only. Many sources are seller listings, patents or forums. Where no number was found it says so. "ENG" marks my own engineering reasoning, not a sourced fact.

## 1. Servo gripper mechanisms that keep the 50-74 N axial load off the servo

Note: the 50-74 N is along the head axis (pull and push through the cartridge). The servo only squeezes ~3-4.5 N per finger over 4 mm. So the real design rule is that the axial load must go cartridge -> tabs/recess walls -> head frame, with the servo only moving the fingers sideways.

- **Over-centre toggle lock on the finger linkage.** Fingers are held closed (or open) by a link past dead point against a rigid stop. Holding load goes into the stop, not the horn. Source gives the principle only, no number. Caveat from the patent: the stop sees large reaction, so it must be a printed wall, not the horn. Fit: the squeeze position is a fixed 4.0 mm stroke, so a toggle at end of stroke is easy. https://www.firgelliauto.com/blogs/mechanisms/vise-grip-pliers and https://patents.google.com/patent/US5884952A
- **Double-toggle gripper.** Two toggles give a large force multiplication near dead point with the actuator seeing little. No number in the result. https://patents.google.com/patent/US4716811
- **Geometric backstop behind the fingers (connector block).** In the pneumatic gripper patent the fingers cannot open until the block retracts. ENG: for Nox, the finger sits in a printed slot whose end wall takes the pull; the tabs' own latch faces (x +/-4 and +/-10) carry the pull and the servo never feels it. https://patents.google.com/patent/US5884952A
- **Parallel jaw, linear slides (rack-and-pinion or crank-slider).** Jaws on two printed slides (or 3 mm carbon rod guides) so pull is taken by the slide, not the pinion. No sourced number; ENG: a 4 mm stroke needs only ~+/-25 deg of a 8 mm horn arm, so a short crank-slider with a near-dead-centre crank at the end of travel is also nearly self-locking.
- **Pull-through-frame by shape, not force.** Since the tabs latch into the receiver, hold the cartridge with a fixed hook or pocket in the head (8x4x3, 10 deg grip pocket) and use the fingers only as the release squeeze. ENG, no source.
- **Compliant (flexure) fingers.** Printed PETG flexures give the sprung squeeze with no pivot play and no backlash; limit: flexure strength in PETG, and it does not lock. Use only for the finger tip, not for the load path. No source number found.

Self-locking summary (ENG): toggle past dead point and a worm are self-locking; parallel racks, four-bars and cams are not unless they sit on a stop or cam with friction angle below the lock angle (cam lead under ~ atan(mu), PETG mu ~0.3-0.4 so under ~17 deg).

## 2. Hobby servo practice

- **No published axial or side-load limit for DS3218 or MG995 was found.** A RobotShop thread says the user never saw such a chart and that the two bearings sit on a stubby shaft, bad for side load. https://community.robotshop.com/forum/t/maximum-radial-load-to-a-servo-rules-of-thumb-specs/16175 So treat as "zero": carry radial and axial load on a separate pivot.
- **Support the output with an outboard bearing / second pivot.** A patent cites lateral loads breaking the shaft even when torque is within rating, and fixes it with a support panel and auxiliary in-line shaft. Fit: use a 625ZZ or a 3 mm brass bush plus a horn-side pin on the opposite side of the finger. https://patents.google.com/patent/US7501731
- **Torque numbers (seller listings, vary by clone).** DS3218: 19-21 kg-cm at 5 V, 21.5-23.5 kg-cm at 6.8 V; no stall current found for the DS3218 (1.9 A at 5 V belongs to the DS3235, do not use it). MG995: 9.0-9.4 kg-cm at 4.8 V, ~11 at 6 V; stall 1.2-1.5 A; sellers warn against sustained stall. Fit: 3-4.5 N x ~10 mm lever is ~0.05 N-m (0.5 kg-cm), so either servo has >10x margin. Size supply for stall. https://www.flux.ai/lwcassid/high-speed-servo-ds3218-pro.md , https://protosupplies.com/product/servo-motor-mg995/ , https://thinkrobotics.com/es/products/extra-large-metal-servo-mg995
- **Metal horn and spline in printed parts.** No source number. ENG: do not rely on a printed 25T spline; use the bought metal horn and screw it with 4 M2 into a printed part with a 2.0 mm minimum web, or heat-set inserts. MG995 and DS3218 splines are metal gears with a plastic or aluminium horn; a metal horn avoids the stripped-horn failure.
- **Holding when unpowered.** Geared servos back-drive when unpowered; sellers give no holding number. Hence the toggle or stop idea in section 1. Do not count on the servo holding.
- **Position feedback.** Solder a wire to the pot wiper and read it with an ADC; maps roughly linearly to pulse width, but readings drift under load and many servos need opening; pot is rough and external sensor is better. Fit: use a 2-position check (open / squeezed) with the wiper tap, or a small hall sensor and magnet on the finger. https://forums.parallax.com/discussion/comment/1328903 , https://community.robotshop.com/forum/t/servo-position-feedback/21456

## 3. Battery-swap stations and connectors

- **Compliance by alignment features, not force control.** US 11981226 uses a conical male connector guiding the battery female connector, plus magnets for positioning and grip. Fit: a lead-in cone on the XT60 housing in the cartridge cuts the alignment tolerance the head needs. https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11981226
- **Centre-and-lock the drone first.** Inverted Docking Station (Drones 6(3):56) centres and locks the UAV with W-shaped plates before the gripper acts; sizes the plates from landing error. Fit: the funnel/hook latch already does this; v7 should state the residual error the head must accept. https://0-www-mdpi-com.brum.beds.ac.uk/2504-446X/6/3/56
- **Low-cost exchange mechanism with force and voltage sensing.** Drones 7(4):270 uses a gripper with force and voltage sensing and a supercapacitor to keep the UAV alive during swap. The page returned 503 on fetch so no numbers were read from it. Fit: sensing the push force (current of the NEMA17) to stop at mate. https://www2.mdpi.com/2504-446X/7/4/270
- **Camera guidance then impedance control.** CN122501567A: coarse global camera, fine end camera, impedance control to avoid rigid impacts. Fit: the Nox head is open loop, so compliance must be mechanical. https://eureka.patsnap.com/patent/CN122501567A
- **Sliding pushers square the drone and a gripper holds the body (US 12286252).** Fit: confirms the usual split of "hold the drone fixed, move only the battery". https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12286252
- **XT60 force: no measured number found.** Only generic test method (EIA-364 TP-13 style) and a 75 N automotive limit for large connectors. Your 50-57 N unmate stands as the project's own measurement; keep it as P-tagged. Test method: https://www.electronics.org/sites/default/files/test_methods_docs/3-18.pdf . ENG: a floating head (~1-2 mm XY play on a spring or on PETG flexures) is the standard fix when lead-in is not enough; a source for remote-centre compliance on a drone swap was not found.

## 4. NEMA17 + printed rack and pinion and V-wheel carriage

- **Module 1 is at the printable limit.** A tooth is ~1.4 mm thick, barely 3 passes at 0.4 mm nozzle; guide says keep module at ~4x nozzle diameter or use a 0.25 mm nozzle. Fit: on the A1 mini with 0.4 nozzle, module 1 is marginal; consider module 1.5 or a wider face, or 0.2-0.25 mm nozzle. https://simplethimble.readthedocs.io/en/latest/chapters/3D-Printing/rack-pinion/rack-pinion.html (the fetched page itself gave no numbers; the figures above come from the search summary of the guide text).
- **PETG is the right plastic for torque and impact**, tougher but less stiff than PLA. Same source.
- **Backlash.** Absolute 0.1-0.3 mm in FDM, 0.15 mm first guess; herringbone anecdotally reduces backlash. Fit: backlash matters little for a push-to-stop axis; use herringbone only if the rack is not printed in sections. https://forum.v1e.com/t/printed-r-p-x-y-z-platform-from-mpcnc-junkbox/48648/6
- **Tooth stress check (ENG, not sourced).** Tangential force = 0.59 N-m / 8 mm pitch radius = 74 N, equal to the stall push. Lewis stress with face width 10 mm, Y ~0.30 for 16T: 74 / (10 x 1 x 0.30) = ~25 MPa; PETG yield ~45-50 MPa so static margin ~2x, little fatigue margin. Widen the face to 12-15 mm or cap motor current so stall is not reached.
- **Preload via eccentric spacers.** OpenBuilds eccentric spacer gives up to 1.5 mm off-centre preload on V-wheels. Fit: use eccentric on 1 of 3 wheels, or a spring-loaded third wheel on the printed rail. A forum thread shows preload cannot fix an out-of-tolerance rail (60 mm rail measured 59.75). https://us.openbuilds.com/eccentric-spacer
- **Wheel spacing versus moment.** No source found. ENG: reaction on the wheels = M / wheel span, so a longer wheel span (the 2-wheel side) cuts load; put the pair that takes the head's pull-off moment as far apart as the head allows, and keep the rack force line close to the wheel plane to reduce moment.

## Recommend for the Nox head v7 (ranked)

1. Load path through the frame: the 50-74 N goes cartridge tabs/recess walls -> head frame. Servo only moves fingers 4 mm. Put a printed stop wall at each finger's end of stroke. (sections 1, 2)
2. Toggle or near-dead-centre crank on each finger with a hard stop so the grip holds with servo unpowered. (1)
3. Add an outboard second pivot / bearing (625ZZ or bush) opposite the servo horn; assume the servo spline takes no side load. Use a bought metal horn. (2)
4. Give the cartridge XT60 a lead-in cone and let the head float ~1-2 mm in XY (flexure or spring) so the 60-74 N push is not also an alignment force. (3)
5. Widen the pinion/rack face to 12-15 mm (or move to module 1.5) and cap motor current below stall; Lewis stress at 74 N is ~25 MPa on a 10 mm face. (4)
6. Preload V-wheels with one eccentric or spring wheel, put the two-wheel pair as far apart as the head allows. (4)
7. Sense grip with the servo pot wiper (2 states) and the push with motor current; do not depend on pot accuracy under load. (2, 3)
8. Measure XT60 mate/unmate on the real part with a force gauge and keep it as P-tagged; no published figure found. (3)
