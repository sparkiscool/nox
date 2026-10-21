# Skydio X10, R10 and Dock for X10: autonomy, docking, scan (11 Oct 2026)

**Bottom line**
1. The Dock for X10 does not need a precise landing. Fences push the legs to center, then a hub rises from below with horns, pins, floating contacts and a sprung lock (E, from matching Skydio patents). Dock landings succeeded 99.91% in Q2 2026 [Reliability].
2. X10 navigates on six 200° cameras, three above and three below, on a Jetson Orin plus a QRB5165 [X10 manual; X10 maintenance manual]. NightSense is lamps above and below, capped at 8 m/s [X10 manual].
3. Top steals: the horned rising hub for the Overseer swapper, a torque self-test as the seated check, and Overseer coverage gaps as Scout tasks.

## Findings

**X10 autonomy.** Six 32 MP trinocular navigation cameras see 20 m over 360°, up to 16 m/s, keeping prop tips 60 cm off by default [X10 specs]. The manual adds a ToF sensor, no avoidance below 3 m at launch and landing, and a 20 m ceiling on weak GPS unless vision-only is chosen [X10 manual]. Hover holds ±10 cm on VIO, ±1 m on GNSS [X10 specs]. Nidec names the Orin as main processor [Nidec]; the dock blog's 100 TOPS Qualcomm claim conflicts [Dock blog], and 100 TOPS fits an Orin NX (E). X10D adds a multi-band radio [Wikipedia]. No FCC internal photos or teardown were found.

**Night.** NightSense is one lamp lighting above and one below, visible or IR [X10 manual], with matched cameras [NightSense blog]. If vision and GPS both degrade, the drone drops to attitude mode; headlights on the landing zone can trigger that [Night guide].

**R10.** 785 g, 10.4 x 10.3 x 2.4 in, guarded props, up to 20 min, vision navigation indoors, turtle-mode self-righting, and up to 3 h of perched streaming [R10 FAQ]. Two stereo navigation cameras give 180° [R10 page]. No R10 dock was found.

**Dock for X10 mechanism.** Skydio's Aug 2024 patent family [US12673792B2] matches the manual's two alignment fences and retracting charger [Dock manual], so it likely shows this dock (E). Legs land in depressions that widen the tolerance. Two members slide in on the legs and park the battery over a hidden hub. A lead screw lifts the hub on a rail, with Hall sensors for height. Angled horns funnel the drone into a cradle, and two pins enter holes in the battery. The contact head floats against magnets to line up power, data and ground contacts, and they re-center it after undocking. A sprung bail swings about 30 to 60° onto a shelf on the drone [US12649593; US12673792B2]. Magnets hold the 685 to 708 g pack [X10 manual; X10 specs]. E-stop cuts motors and HVAC but keeps compute on; the roof settles fully open or closed [Dock manual].

**Landing into it.** AprilTags sit on the platform [Dock manual]; the drone must be within 3.7 m above the dock to detect it [Site manual]. The 2019 patent adds to the gate in `hover-catch-mechanism-survey.md`: GPS to 3 to 5 m, an outer tag about 25x the pad tag's area, a hover about 5 cm up, then a faster drop. Touchdown is confirmed by matching IMU data to expected contact forces, then by applying torque and checking the drone cannot rotate. A failure climbs out to a fallback landing point [US11897632; Dock manual]. Accuracy is not published.

**2D and 3D Scan.** The operator boxes the asset with a floor (default 0.5 m up), a ceiling and three or more wall pillars. A short exploration flight (0.5 to 5 m/s) builds a model and predicts time, batteries and photos. The scan defaults to 80/70 overlap and a 45° gimbal [3D Capture]. A coverage heatmap shows gaps before leaving site [Onboard Modeling]. 2D maps fly a terrain-following grid [3D Scan].

## Steal for Nox
- Swapper (Overseer): rising pusher with angled horns, two cartridge pins, a magnet-centered floating connector, then a sprung bail.
- Box catcher: after the jaws close, the drone commands a small yaw torque; no IMU rotation means seated.
- Box catcher: under each drone, a big tag for approach and a small one for the last centimeters.
- Box catcher: E-stop cuts motors but keeps logic and camera alive, plus a motion warning light [Dock manual].
- Firmware (Overseer): if lidar collision prevention is added, keep it off over the box, as X10 does below 3 m.
- Scout: a downward lamp, like NightSense's lower one, so flow works in the dark (E); and perch mode, land and keep streaming.
- Overseer and comms: explore, map, show a coverage heatmap; the gaps become Scout tasks.
- Cartridge (Scout v2): magnet retention as on X10; check it holds the 25 g misuse load, ~40 N for a 165 g pack (E).

## For Spark
- `nox-dimensions.md` §4 ("long V-jaws clamp the ring rods at mid-span, both ways") and `box.md` ("the box camera confirms the drone is seated"): add the torque self-test before disarm; US11897632 claims it with a funnel dock (E, not a legal read).
- `box.md` ("e-stop cuts motor power") and §4 ("StallGuard on the TMC2209"): Overseer weight on the 45° lower flanks pushes each jaw out about 10 N (E), and an unpowered belt drive can back-drive. Decide the jaws' E-stop state (E).
- `box.md` X9 ("Overseer pack measure, cartridge, pocket, 3 bays") and `nox-dimensions.md` §2 ("Bays: float ±0.5"): draw the Overseer cartridge with two pin holes and a floating connector seat, and let magnets re-center the bay float (E).
- `box.md` ("landing camera in the box looking up, white LEDs"): they shine into both drones' flow lenses before the trigger, the case Skydio warns about. Aim them off the lens or log flow quality with them on (E).
- `scout.md` Lights ("1-3 W white headlight"): add a downward lamp (E).
- `box.md` ("Done: not set"): make it a counted run, say 20 clean catches in a row (E).

## Sources
- [X10 specs: Skydio X10 technical specs](https://skydio.com/x10/technical-specs)
- [X10 manual: Skydio X10 operator manual (PDF)](https://www.adorama.com/col/productManuals/SKYD4EVH1VL.pdf)
- [X10 maintenance manual: Skydio X10 Maintenance Manual rev 2](https://support.skydio.com/hc/article_attachments/49394235475227)
- [Night guide: How to fly Skydio X10 at night](https://support.skydio.com/hc/en-us/articles/21479288051867-How-to-fly-Skydio-X10-at-night)
- [NightSense blog: Unlocking the night](https://skydio.com/blog/unlocking-the-night-skydio-x10-s-nightsense-technology)
- [Nidec: motors for Skydio X10](https://www.nidec.com/en/product/news/2024/news0722-01/)
- [Wikipedia: Skydio X10](https://en.wikipedia.org/wiki/Skydio_X10)
- [Dock blog: Dock for X10 launch](https://skydio.com/blog/dock-for-x10-future-scalable-autonomous-flight-data-capture)
- [Dock manual: Dock for X10 Flight System operator manual rev 2.3](https://support.skydio.com/hc/article_attachments/50296199666459)
- [Site manual: Dock for X10 Site Readiness Manual rev 3.1](https://support.skydio.com/hc/article_attachments/50099725285275)
- [Reliability: Skydio reliability dashboard](https://www.skydio.com/platform/reliability-dashboard)
- [US12673792B2: Methods of docking UAVs with base stations (Skydio)](https://patents.google.com/patent/US12673792B2/en)
- [US12649593: Charging hubs with fixed and movable brackets](https://patents.justia.com/patent/12649593)
- [US11897632: Automated docking of UAV (Skydio)](https://patents.justia.com/patent/11897632)
- [R10 FAQ: Skydio R10 FAQs](https://www.skydio.com/r10/faq)
- [R10 page: Skydio R10](https://www.skydio.com/r10)
- [3D Capture: How to use 3D Capture with Skydio X10](https://support.skydio.com/hc/en-us/articles/30063556135195-How-to-use-3D-Capture-with-Skydio-X10)
- [Onboard Modeling: A new dimension for 3D scanning](https://www.skydio.com/blog/a-new-dimension-for-3d-scanning-with-onboard-modeling)
- [3D Scan: Skydio 3D Scan](https://skydio.com/software/3d-scan)
