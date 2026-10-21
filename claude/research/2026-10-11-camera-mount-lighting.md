# Impact-tolerant camera mounts and inspection lighting (11 Oct 2026)

**Bottom line**
1. Elios 2's camera retracts in frontal hits. In Flyability's patent a latch holds the pod rigid up to a set force; past it the pod slides back on soft springs and returns [US12325542B2]. No hobby camera mount found publishes impact data [GetFPV].
2. Dust is beaten by geometry, not power: lights wide of the lens, toed out so only beam edges reach the subject, at short range [Jaffe; UWPG]. Dim LEDs by current, never by switching at 50 to 400 Hz [ENTTEC].
3. For Scout (E): two off-axis heads, 240 lm at 2.2 W (boost 450 lm, 4.5 W), ~11 g; on Scout v2, a spring-preloaded camera sled with 15 mm of travel.

## Findings

**Retract mounts.** The Elios 2 brochure puts the payload on a retractable, load-limited structure [Elios 2 brochure]; Janes describes a retracting cage instead [Janes]. In US12325542B2 (Flyability, granted 2025) springs push the pod forward through the cage, and magnets at 2 to 4 points, or a preloaded spring on a stop, hold it rigid. Past a threshold force it slides or pivots back to an end stop; the springs return it and the magnets re-latch. Examples: ~1 N/mm, ~20 mm stroke, 60 to 150 g pod, 3 to 5 m/s; no release force is given. Mounted in parallel with the cage, the pod meets about a tenth of the craft's impact energy [US12325542B2]. The springs store only ~0.2 J (E): the pod gets out of the way and the cage absorbs the crash. EPFL's frame on magnetic joints with elastic-band return survived 50+ drops from 2 m (~6.3 m/s, E) [NCCR].

**Lights.** Elios 3 holds 20 W above 50 °C but boosts to 40 W only up to 30 °C [E3 spec]. It rates 20 W as Elios 2's maximum: ~3,200 lm at E3's 160 lm/W peak (E), a third of Elios 2's claimed 10,000 lm [Elios 2 brochure]. The specs disagree. IBIS2 Assist dims its two lights in five steps [DRONE.jp].

**Dust geometry.** Backscatter comes from air both lit and seen. Separating lights from the camera shrinks that volume and stretches usable range from ~1 to 2 or 3 attenuation lengths [Jaffe]. Divers mount lights wide of the port, angled slightly out, so only beam edges light the subject; backscatter grows with distance [UWPG]. Rule of thumb (E): toe each light out until its inner beam edge crosses the camera axis just short of the subject. In dust, go closer, not brighter.

**LEDs.** Luminus SST-20 (3.45 mm): CRI 95 up to 4000K, typ 130 lm/W (3000K, 350 mA, 85 °C), 120° beam, ~1.84x flux at 700 mA [Luminus]. Per lumen, a small head costs ~10 mW from the pack, ~60% of it heat, and 25 to 45 mg (E).

**PWM vs shutter.** WS2812B dims at ~400 Hz, SK6812 at ~1.2 kHz. Rolling shutters show bands, global shutters pulse whole frames; below ~1 kHz it shows, worst at 5 to 20% brightness [ENTTEC]. 25 kHz and up is safe and silent [Waveform].

**Low-light cameras.** IBIS2: 1/2.8 in STARVIS, 1080p [DRONE.jp]. Elios 3: 1/2.3 in, 2.71 mm lens, 0.18 mm/px at 30 cm [E3 spec]; neither publishes an F-number. IMX462: 1/2.8 in, 2.9 µm, rolling shutter [Commonlands]; STARVIS 2's IMX662 adds over 8 dB of dynamic range [Framos]. Global-shutter pixels lose fill factor to a storage node and add read noise [Tucsen]. The DECXIN's sensor was not found; same-size 38 x 38 colour global-shutter UVC boards use the OV9782: 1/4 in, 3 µm, 1280 x 800 [Arducam; Newegg].

**Exposure (E).** 18% grey at F2 and ISO 800 needs ~110 lux at 1/100 s and ~440 lux at 1/400 s; F2.8 doubles both. 1/400 s holds a 0.2 m/s drift to ~1 px at ~0.5 mm/px (Scout at 0.3 m).

## Steal for Nox
- Scout v2 holder: Elios-style camera sled; rigid in flight, gives way past a set push, resets itself.
- Scout: two light heads wide of the lens, with Normal, Boost and Oblique (one side) modes, as Elios 3.
- Firmware: Elios-style heat limit; Boost only when armed, 20% once disarmed, set by the Radxa from the arm state (E).
- Box catcher: constant-current landing LEDs, so no camera or flow sensor sees flicker (E).
- Data product: quote GSD at a fixed standoff, as Elios does.

## For Spark
- **v1 lens (E).** `nox-dimensions.md` §2: lens front "2.0 behind the plane of the front mid-V rod surfaces", "lens to nearest rod 23.2". A wall meets the front equator rod ~24 ahead of the lens first. A thin edge at lens height meets only the V rods, so ~2 mm of rod give puts it on the glass; a stub narrower than the ~45 mm V gap needs none. Pocket v19 is frozen: accept it for the demo at the planned 1.0 m/s cap (`scout.md` B10).
- **v2 camera sled (E).** `v2/README.md` lets the cage deflect to "clearance ≥ 0". Before the S2 holder run, change `v2/dimensions.md` Camera ("4 posts OD 8.14, 13 long"): post tips ~20 behind the board, plug room 15 deeper, a USB loop. The board slides on four long plain-shank M2 screws in the inserts; springs press it onto the screw heads at 7.5 N, twice the "camera 3.7 N" case in `v2/s2-holder.md`, rising to 20 N at 15 mm. That stores ~0.2 J, enough to stop a 0.4 kg Scout at 1 m/s on the lens alone. It stays seated under inertia, and the screws keep the first mode near 400 Hz or above. ~+2 g. The patent is in force; Scout's camera stays inside the cage, while claim 1's pod projects through it (not a legal read).
- **Lights (E)**, against "1-3 W white headlight" in `scout.md`. Two heads at the NL / NR nodes (drone (64, ±31, 40); v2: at the front sockets (55, ±31, 27)), 29 behind and 31 beside the lens, pitched with it. Each: one SST-20 4000K CRI 95 on a 20 mm star, 60° optic toed out 20 to 25°, so the inner edges cross the axis 0.15 to 0.33 m ahead. One constant-current buck per side (for Oblique), dimmed by a DC level: an RC filter on a PX4 servo output works; switching the LED at 50 to 400 Hz does not. Normal 350 mA: 240 lm, 2.2 W, ~1,000 lux mid-frame at 0.3 m, ~250 at 0.7 m. Boost 700 mA: 450 lm, 4.5 W (3 to 5% of hover), ~1,900 and ~450 lux. ~11 g, not in the v2 mass budget. Each head shades ~5% of a front prop disk; that inflow holds a star to ~+20 °C at 350 mA and ~+40 at 700. In still air it reaches ~+60 at 350 mA, against "PETG softens ~80 °C" (`v2/s2-contacts.md`). The ~90° view still sees lit dust at its edges 2 to 3 cm out. CHECK rod shadows; keep the WS2812B status LEDs out of frame.
- **Flow light.** MTF-02P "60 lux minimum" (`claude/overseer-scout-hardware-reference.md`): a bare downward LED needs ~50 lm at 0.5 m, ~190 lm at 1 m (E).
- **Field of view.** `scout-camera-tilt.md`: "Vertical half field ~36.5° (102° diagonal, 4:3)". On a 16:10 sensor like the OV9782 it is ~33° (E): CHECK the stream size.

## Sources
- [Elios 2 brochure (xboom)](https://www.xboom.in/wp-content/uploads/2025/02/Copy-of-Eliso-2_compressed-1.pdf)
- [Janes: Flyability launches Elios 2 caged UAS](https://www.janes.com/osint-insights/defence-news/xponential-2019-flyability-launches-elios-2-caged-uas)
- [US12325542B2, UAV with protective outer cage, Flyability (Google Patents)](https://patents.google.com/patent/US12325542)
- [NCCR: Insect-inspired mechanical resilience for multicopters](https://nccr-robotics.ch/wp-content/uploads/2020/11/32-Insect-Inspired-Mechanical-resilience-for-multicopters.pdf)
- [GetFPV: Brain3D TPU camera protective case](https://www.getfpv.com/brain3d-stock-tab-style-adjustable-camera-protective-mounting-case.html)
- [E3 spec: Elios 3 tech spec PDF (Nexxis)](https://nexxis.com/wp-content/uploads/sites/3/2022/06/20220519_E3_Tech_Spec_web_version.pdf)
- [DRONE.jp: IBIS2 Assist](https://drone.jp/?p=110006)
- [Jaffe: Underwater optical imaging, Oceanography](https://tos.org/oceanography/assets/docs/1-2_jaffe.pdf)
- [UWPG: Backscatter in underwater photography](https://www.uwphotographyguide.com/?p=17292)
- [Luminus: SST-20-WxH datasheet](https://download.luminus.com/datasheets/Luminus_SST-20-WxH_Datasheet.pdf)
- [ENTTEC: Why do LEDs flicker on camera](https://support.enttec.com/pixel/pixel-general-knowledge/why-do-leds-flicker-on-camera)
- [Waveform: Flicker-free LED strip dimming](https://www.waveformlighting.com/film-photography/an-introduction-to-flicker-free-led-strip-dimming)
- [Commonlands: Sony IMX462](https://commonlands.com/pages/image-sensors/imx462)
- [Framos: IMX662-AAMR flyer](https://www.framos.com/wp-content/uploads/IMX662-AAMR_Flyer.pdf)
- [Tucsen: Global vs rolling shutter](https://www.tucsen.com/learning/cmos-electronic-shutters-global-vs-rolling)
- [Arducam: B0385 OV9782 USB camera manual (Core Electronics)](https://core-electronics.com.au/attachments/localcontent/B0385_Manual_218018a0904.pdf)
- [Newegg: OV9782 global-shutter USB camera listing](https://www.neweggbusiness.com/p/9B-1EF-024X-00018)
