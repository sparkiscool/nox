# Indoor drone noise reduction, and whether anti-noise (ANC) works (11 Oct 2026)

Asked by Spark in chat (not a queue item). Every number is E unless it carries a source tag. Nox values quoted from `nox/nox-dimensions.md`, `nox/scout.md`, `nox/v2/README.md`.

**Bottom line**
1. **Playing anti-noise from the drone is not worth building for Nox.** The best 2026 study (RWTH Aachen, two drones, hemi-anechoic room, speakers too heavy to fly) got a 9.4 dB median cut on the tones below the drone, but perceived loudness dropped only 6.5 to 8.5% [Steiner 2026]. NASA's single-speaker rig got 30 dB+ in some directions and made it louder in others [NASA 2018]. Indoors the louder directions bounce off the walls back to the listener (E).
2. **The physics is worst on the Scout.** Its 3.5 in props hover at roughly 20,000 to 25,000 rpm (E), and the props are 3 blade (P), so the main tone sits near 1.0 to 1.2 kHz (E), wavelength ~28 to 34 cm. A speaker has to sit within a few cm of each prop to cancel it in all directions (E). Broadband noise (the "whoosh") cannot be cancelled at all.
3. **The version of the idea that does work uses the rotors themselves: synchrophasing.** Lock the four props' blade angles so their tones cancel. Penn State got 10 dB over a 20° arc and ~6 dB in nearly all directions on a hexacopter, but needs phase held within ~5° [Valente 2025]. Research-grade firmware, not a demo item.
4. **What actually makes Nox quieter, in order of payoff per effort:** lighter Scout (v2 already does this), ESC PWM 48 kHz, smooth tune and RPM filter, fresh balanced props, keep off the ceiling and away from people, then hardware: tractor props (props above the arms), bigger slower props, more blades. Ducts and toroidal props: no.

## Nox noise sources (E)
| | Scout | Overseer |
|---|---|---|
| Props | Gemfan 3520, 3.5 in, 3 blade (P, Spark 11 Oct) [`scout.md`] | 1045, 2 blade [`nox-dimensions.md` §4] |
| Thrust per motor at hover | ~125 g at 500 g all-up; ~100 g at v2's ~400 g | ~500 g at 2.0 kg |
| Hover rpm | ~20,000 to 25,000 | ~6,000 |
| Tip speed | ~90 to 115 m/s | ~80 m/s |
| Blade-pass tone (BPF) | ~1.0 to 1.2 kHz | ~200 Hz |
| Wavelength at BPF | ~0.28 to 0.34 m | ~1.7 m |

The Scout's tone lands in the 1 to 5 kHz band where hearing is most sensitive, and its tip speed is higher than the Overseer's. Expect the Scout to sound more annoying than the 4x heavier Overseer at the same distance (E, not measured).

## 1. Anti-noise from on-board speakers (ANC)
- **How it works:** mic near the prop, speaker plays the inverted wave. Works well in small volumes (headphones, ducts, car cabins) and on low tones.
- **Best results so far:**
  - Steiner et al. 2026 (Acta Acustica): DJI Mini 2 SE (246 g, BPF 179 to 260 Hz) and ModalAI Sentinel (1.35 kg, BPF 56 to 124 Hz), drones on a rod, 5 studio monitors under them. Simulated median 9.4 dB, peak 15 dB+ on tones in a 1 m disk 3.4 m below. Wind noise from the props set the floor; a 15.6 kHz peak from motor control could not be cancelled. Loudness down 8.5% (Mini) and 6.5% (Sentinel). The speaker array made the drone unflyable. [Steiner 2026]
  - Bi et al. 2025 (cited in Steiner): 4.78 dB in 1500 to 2400 Hz, peak 10 dB+.
  - NASA 2018: 7.6 cm speaker, 23.9 cm rotor, anechoic: 30 dB+ tonal cut in chosen directions, amplification in others; pitched as a steerable "cone of silence". [NASA 2018]
  - NASA 2020 virtual-sensor ANC: 6 to 13 dB at far-field points, one 2-blade rotor. [NASA 2020]
- **Why it fails for Nox indoors:**
  1. Directional only. Indoors, sound reaches a person off walls, floor and racks from every direction, so a quiet cone in one direction is filled in by reflections (E).
  2. Tones only. The broadband part is uncorrelated and stays.
  3. Scout tones are at ~1.0 to 1.2 kHz (E): global cancellation needs a speaker within roughly a tenth of a wavelength of each prop, ~2 to 3 cm (E), four times over, plus harmonics.
  4. Mass and power. Each speaker that can match a prop at 1 m is tens of grams plus an amp and a DSP (E). v2's whole battery holder budget is 21 g.
  5. Mics in prop wash: wind noise corrupts the reference signal [Steiner 2026].
- **Where ANC does work:** at the listener. ANC earmuffs or headsets for the operator and nearby staff. Off the shelf, no drone change.

## 2. Synchrophasing (anti-noise using the props)
- Fix the relative blade angle between rotors so their tones interfere destructively. Free in mass; needs phase-locked motor control.
- Penn State hexacopter, anechoic: 10 dB harmonic cut over a 20° arc, ~6 dB global with other offsets; phase error must stay under ~5° [Valente 2025]. Follow-up 2026 paper extends it to changing rotor speeds [Valente 2026].
- Dortmund 2026 two-prop rig: ~15 dB at BPF in the prop plane at 4000 rpm, but only ~3 dB at BPF and 0.35 dB A-weighted averaged over all mics [Lübbert 2026].
- Glasgow CFD: up to 7 dBA between two tandem rotors at the best phase [Qiao 2025].
- Unsynchronised rotors also make the broadband noise pulse randomly, which matters for annoyance [Gan 2024].
- For Nox: the MicoAir AIO runs Bluejay. Bidirectional DShot gives rpm, not blade angle, so this needs custom ESC firmware (AM32 is open source) and a hover-only mode. A quad has to vary rpm to fly, so it only holds in steady hover. Park it as a later R&D item.

## 3. Propellers
- **Bigger, slower props** are the biggest single lever: lower tip speed at the same thrust cuts noise and raises efficiency, confirmed on quadcopter props [Brungart 2019]. Crude tip-speed^5-6 scaling (E): 3.5 to 4 in ~-3 dB, to 5 in ~-8 to -9 dB at the same thrust. Costs a bigger frame and cage.
- **Lighter drone** does the same thing for free: v2 at ~400 g vs ~500 g is ~11% lower tip speed, ~-2.5 to -3 dB (E).
- **More blades:** 4 to 4.5 dB reported [Steiner 2026, citing Metzger]; pushes the tone higher in pitch.
- **Toroidal props: skip.** A 2026 test of a commercial 5 in toroidal prop found it the loudest at every hover thrust and angle, with no efficiency gain [Acta Acustica 2026]. Dortmund found toroidal props cut the tones but raised broadband and overall SPL, at 0.046 to 0.050 N/W vs 0.080 for a normal prop [Lübbert 2026]. MIT's claims are mostly anecdotal [New Atlas].
- **Uneven blade spacing:** moves energy to half the BPF, lowering perceived loudness; one 90° two-blade prop kept efficiency parity [Usov 2025]. Needs custom props; low priority.
- **Damaged or unbalanced props** after cage hits add vibration and tones. Swap them (E).

## 4. Airframe and layout
- **Scout is a pusher** (motors inverted, props under the arms) [`scout.md`]. Props running in a strut or arm wake get a pusher penalty, up to 20 dB toward the axis in NASA's (large prop) tests [NASA 1986]; on a small quad, rotor-airframe interaction can dominate the BPF tones [Ma 2024]. Flipping to tractor (props on top) is the cheapest hardware fix to test (E: worth 2 to 5 dB, not measured). It moves the prop plane, so it is a cage-geometry decision.
- **Ducts / shrouds:** a rigid shroud cuts tones below the rotor plane [Go, Bristol]; a poorly shaped one adds broadband [NASA 2022]; printed ducts measured louder at high frequencies [UFU thesis]. Lined ducts gave up to 4 dB broadband in the lab [Palleja-Cabre 2025]. With v2's mass target and prior duct findings (`2026-10-11-small-caged-indoor-drones.md`), keep ducts off.
- **Cage rods near the props:** the equator ring sits 12 mm off the prop tips in the prop plane [`nox-dimensions.md` §3]. Likely a small extra tone source (E, untested).

## 5. Motors and ESCs
- Motor noise sits near 1.2 kHz and ~5 kHz (rotor vibration modes) [NASA Huff 2018].
- Bluejay supports 24, 48, 96 kHz PWM; 48 kHz is the usual 3 to 5 in setting and runs quieter and cooler at part throttle [UAVModel]. Check the Scout's setting: set 48 kHz (DEFAULT proposal) if it is on 24.
- RPM filtering via bidirectional DShot on PX4 smooths motor commands; rpm hunting is what people notice [Steiner 2026; Ma 2024].

## 6. Indoors specifically
- **Recirculation:** in a closed room the drone re-ingests its own wake after ~30 s. On a 1.4 kg Phantom the tones barely changed but broadband rose ~1 to 2 dB out of the rotor plane, more when hovering high (near the ceiling) and more for lighter drones [Ma 2024]. On fixed small rigs in small rooms, BPF rose 15 dB+ [Stephenson, cited in Ma 2024]. The Scout is light and flies in tight spaces, so expect it to be on the bad side (E).
- **Reverb:** in a hard-walled room, distance helps less than outdoors, since the reflected field doesn't fall off (E, standard room acoustics).
- **Operations:** avoid hovering near the ceiling and in corners, keep transits brisk, don't hover over people, give the operator ANC hearing protection.

## Steal for Nox
- **Scout v2:** keep it light; every 100 g is ~2.5 to 3 dB (E).
- **Scout firmware:** Bluejay 48 kHz PWM; bidirectional DShot rpm filter verified on PX4; soft hover tune.
- **Scout v2 frame:** test tractor vs pusher on one motor before the cage is drawn.
- **Scout props:** compare the 3520 (3 blade) vs a 4 blade 3.5 in on a phone spectrum app at 1 m.
- **Box / ops:** ANC earmuffs for the operator; no anti-noise speakers on any drone.
- **Later R&D:** synchrophasing in hover via modified AM32 firmware.

## Cheap measurement plan (one evening, E)
Phone with a spectrum app (calibrated SPL meter if available), drone tethered or hovering at 1.5 m, mic at 1 m and 3 m, same room. Record A-weighted level and spectrum for: baseline; PWM 24 vs 48 kHz; props flipped to tractor on a single motor stand; cage on vs off; 30 s vs 90 s hover (recirculation). That gives the Scout's real BPF too.

## For Spark
- Blade count confirmed 3 (Spark 11 Oct); recorded in `nox/scout.md`, `nox/nox-dimensions.md` §1 and `nox/nox-history.md`.
- `nox/nox-dimensions.md` §1 "Pusher, ducts off": pusher layout is a likely tone source (E, NASA 1986 / Ma 2024). Worth a 10 min bench test before v2's cage is drawn, since flipping moves the prop plane (z 0 datum).
- `nox/v2/README.md` all-up target "~400": also the cheapest noise fix, ~2.5 to 3 dB vs v1 (E).

## Sources
- [Steiner, Hilgemann, Jax, Investigations on drone-mounted ANC, Acta Acustica 2026](https://acta-acustica.edpsciences.org/articles/aacus/pdf/2026/01/aacus250210.pdf)
- [NASA, Initial developments toward an ANC system for small UAS (Inter-Noise 2018)](https://ntrs.nasa.gov/citations/20200002566)
- [NASA, ANC of a small UAS rotor using virtual error sensors (2020)](https://ntrs.nasa.gov/citations/20205004166)
- [Valente, Greenwood, Johnson, Electronic rotor phase synchronization, JAHS 2025](https://rex.libraries.wsu.edu/esploro/outputs/journalArticle/An-Experimental-Evaluation-of-an-Electronic/99901313917401842)
- [Valente et al., Synchrophasing under nonstationary conditions, JSV 2026](https://ascent.aero/publication/multirotor-noise-control-using-synchrophasing-under-nonstationary-conditions)
- [Gan et al., Time-varying broadband noise of multirotor aircraft (2024)](https://rosap.ntl.bts.gov/view/dot/79609)
- [Lübbert et al., Design optimization vs active phase control, Forum Acusticum 2026](https://dael.euracoustics.org/confs/fa2026/data/articles/000489.pdf)
- [Qiao, San Martin, Barakos, Synchrophasing of rotors in close proximity, AST 2025](https://eprints.gla.ac.uk/342775)
- [Joseph et al., Propeller synchrophasing mechanisms, JSV 2025](https://eprints.soton.ac.uk/502892/)
- [Commercial toroidal propeller vs conventional, Acta Acustica 2026](https://acta-acustica.edpsciences.org/articles/aacus/abs/2026/01/aacus260084/aacus260084.html)
- [MIT Lincoln Lab toroidal propeller (New Atlas)](https://newatlas.com/aircraft/toroidal-quiet-propellers/)
- [Brungart et al., The reduction of quadcopter propeller noise, NCEJ 2019](https://pure.psu.edu/en/publications/the-reduction-of-quadcopter-propeller-noise/)
- [Usov et al., Unequal blade spacing, ERF 2025](https://dspace-erf.nlr.nl/items/78ca54a9-68d2-4de6-a156-cff56d4004cf)
- [NASA, Directivity and trends of noise from a propeller in a wake (1986)](https://ntrs.nasa.gov/citations/19860021869)
- [Ma et al., Flow recirculation effect on free-flying UAS noise, Acoustics Australia 2024](https://ira.lib.polyu.edu.hk/bitstream/10397/108873/1/s40857-024-00327-x.pdf)
- [Go et al., Shrouded UAV propeller sound field (Bristol)](https://research-information.bris.ac.uk/en/publications/experimental-and-numerical-investigation-of-the-sound-field-produ/)
- [NASA, Fundamental noise characterization of a ducted propeller in hover (2022)](https://ntrs.nasa.gov/api/citations/20220006288/downloads/simon_v4.pdf)
- [Bessa, Noise reduction devices for UAVs (UFU)](https://repositorio.ufu.br/handle/123456789/22187)
- [Palleja-Cabre et al., Over-tip-rotor liners, Forum Acusticum 2025](https://eprints.soton.ac.uk/504967/1/ForumAcusticum2025_OTR_Liners_ducted_rotor.pdf)
- [Huff and Henderson, Electric motor noise from small quadcopters (NASA 2018)](https://ntrs.nasa.gov/citations/20180005580)
- [Bluejay PWM settings (UAVModel)](https://blog.uavmodel.com/?p=3429)
