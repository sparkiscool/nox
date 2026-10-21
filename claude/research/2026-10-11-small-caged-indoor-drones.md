# Small caged and ducted indoor drones under 500 g (11 Oct 2026)

**Bottom line**
1. Under 500 g the guard is part of the airframe: Avata 2 (377 g), IBIS2 (243 g) and Dronut (420 to 440 g) enclose their props within 165 to 212 mm [DJI; DRONE.jp; UST049; xboom]. None publishes a crash rating.
2. Flybotix ASIO's cage is just under 100 g on a 1 kg craft (~10%, E): ten composite spars that crumple inward without bouncing, released by two hooks [UST036].
3. Ducts add thrust only with tip gaps under ~3% of radius [Drones25], 1.3 mm on Scout (E), and guard only the sides. Keep "ducts off".

## Findings

**DJI Avata 2.** 377 g, 185 x 212 x 64 mm, 21 min hover; 31.7 Wh Li-ion pack, ~145 g [DJI]. Guards are part of a flatter polycarbonate frame [DJI; Wiki], 16 mm lower and 33 g lighter than Avata 1 [DrDrone]. The pack clicks in; side buckles release it. The camera gets only a storage cover, and the manual warns impacts can damage its gimbal [Manual]. Turtle mode rights it on command [Manual]; on Avata 1 the rear motors run flat out to roll it [Droneblog]. An owner says the ducts absorb most crashes, but DJI sells no frame spares [DroneXL].

**Liberaware IBIS2.** 243 g = 150 g airframe + 93 g pack; 194 x 198.5 x 58 mm; 11 min; IP51; two headlights, 380 lm total; 1/2.8 in STARVIS camera [DRONE.jp, Assist version]. The maker cites ducted fans with in-house props, IP5X-class dust-proof Nidec motors, a crash-resistant body sized by structural analysis, and flight in 500 mm pipes [JPX]. Assist mode holds height by ToF off floor or ceiling within 4 m [DRONE.jp]. Guard material, battery access, camera protection and crash data: not found.

**Cleo Dronut X1.** 420 to 440 g; 165 to 177 mm across, ~100 mm tall, though sources disagree [UST049; xboom; NewAtlas]. 12 min (17 on X1 Pro), swappable pack, 4 m/s cap, carbon composite shell [UST049; xboom; RobotReport]; the prototype carried camera and battery inside it [IEEE]. Two stacked counter-rotating props run in the duct [NewAtlas]. In Cleo's patent, two or more flaps slide from slots into the duct exit at a fixed 20 to 70°: protrusion sets pitch and roll; the rotor speed split sets yaw [US11542000B2].

**Flybotix ASIO.** 1 kg; cage just under 100 g, 395 x 295 mm: ten composite spars joined at two pole fasteners, built to crumple slightly inward without bouncing. Two hooks release it; it folds and is replaced in minutes. A 1.5 m drop (~5.4 m/s, E) left craft and cage undamaged [UST036]. Spars sit ~124 mm apart at the equator (E); UST036 names no mesh, while the patent laces string between flat ribs 0.5 to 5 mm thick and 2 to 20 mm wide, arched concave-in [US12275542B2]. Solvay supplies thermoplastic carbon composite [Solvay]. ASIO X: two coaxial 13 in props, no swashplate or servo, 74.1 Wh smart pack, 1.15 kg, up to 20 min [Frontier]. Camera protection: not found.

**Ducts, measured.**
- UMD, 16 cm rotor: up to 94% more thrust at equal power with a 13% of D lip radius, 10° diffuser and 0.1% of D gap. Edgewise flow adds strong nose-up pitch and drag; at fixed overall size a larger open rotor likely wins [UMD; Chopra].
- TiShrov, 257 g: a ~45 g shroud at a ~1.5% R gap gave ~40% better power loading [JAHS].
- Lip suction lets the duct carry up to half the thrust; output falls as the gap opens [TUDelft]. CFD on a 0.8 m rotor puts break-even at 3% R [Drones25].
- 3 in whoops run 1 to 2 mm gaps (2.6 to 5.2% R, E) and claim +10 to 15% for bell-mouth inlets [UAVModel].

## Steal for Nox
- **Cage (Scout v2):** ASIO-style ribs from top ring to bottom ring, soft inward, nets for fingers. Bent to Scout's ~84 mm rib radius, 1 mm carbon strip stands at ~0.67 GPa, a third of a 3 mm rod; ten 1 x 6 mm ribs weigh ~14 to 17 g (E).
- **Cage (Scout v2):** hooked joint at the frame plate (ASIO), so a cage broken in v2's one allowed crash swaps in minutes.
- **Cage (current):** film a drop test and quote the height as Scout's crash rating, as ASIO does.
- **Firmware:** autotune (B5, `nox/scout.md`) before bump demos, since IBIS2 users report it flies on through wall contact [Liberaware]; an upward ToF for ceiling-referenced height hold (IBIS2 Assist, E).
- **Scout lights and camera:** two headlights at ~380 lm total, ~3 to 4 W of LEDs (E) against the "1-3 W white headlight" in `nox/scout.md`, and a STARVIS-class sensor [DRONE.jp]. Test the global-shutter DECXIN at demo light (E).

## For Spark
- **Cage budget.** `nox/v2/README.md` sets the cage at "55" g. At ASIO's ~240 g/m² of hull (E, 0.41 m² spheroid), Scout's 0.16 to 0.28 m² hull (E) needs ~40 to 70 g: 55 is in reach, though ASIO carries no catch loads. Cage v10's "101 g without bumpers" (`nox/nox-dimensions.md` §3) is 1.5 to 2.6x denser (E).
- **Soft ribs.** ASIO X leaves ~38 to 64 mm from prop tips to its outside (E, [Frontier]); v2 keeps "12 from the prop disks" (`nox/v2/dimensions.md`). Check inward-crumpling ribs in the frame model first. Fixed, non-folding ribs likely fall outside Flybotix claim 1 (E, not a legal read); UST036 cites a second cage patent, unread (queue item 7).
- **Ducts.** Keep "ducts off" (`nox/v2/dimensions.md` §A): at 400 g a 10% hover-power saving pays for only ~30 g of duct (E, momentum theory). Two fixes to `claude/cage-clean-sheet-research.md`: Scout's props are "3.5 in (r 44.45)" (`nox/nox-dimensions.md` §1), not 5 in, and UST036 and the ASIO X flyer show no duct on ASIO (E).
- **Battery.** v2's "165" g on "~400" (`nox/v2/README.md`) is 41%, near Avata 2 and IBIS2 at 38% (E). Avata 2's Li-ion stores ~219 Wh/kg vs ~135 for Scout's LiPo; stock cells meeting "bursts 30 A+" (`nox/v2/dimensions.md`) likely buy flight time, not grams (E).

## Sources
- [DJI Avata 2 specs (DJI)](https://www.dji.com/avata-2/specs)
- [DJI Avata 2 user manual, Kamera Express mirror (Manual)](https://www.kamera-express.lu/media/116dd224-2363-459a-93cf-581f9bdf4f6e/dji-fpv-avata-2.pdf)
- [DJI Avata, Wikipedia (Wiki)](https://en.wikipedia.org/wiki/DJI_Avata)
- [DJI Avata 2 vs DJI Avata, Dr Drone (DrDrone)](https://drdrone.ca/blogs/drone-news-drone-help-blog/dji-avata-2-vs-dji-avata)
- [Avata turtle mode (Droneblog)](https://www.droneblog.com/?p=1044916)
- [DJI Avata 2 long-term review (DroneXL)](https://dronexl.co/zh/2024/12/14/dji-avata-2-long-term-review-after-6-months/)
- [IBIS2 Assist (DRONE.jp)](https://drone.jp/?p=110006)
- [Liberaware investor disclosure (JPX)](https://www2.jpx.co.jp/disc/218A0/140120250314594402.pdf)
- [IBIS customer interview (Liberaware)](https://liberaware.co.jp/ibis/)
- [Cleo Dronut dossier, Uncrewed Systems Technology 049 (UST049)](https://www.uncrewed-systems.com/content/uploads/magazines/Issue%20049/97/)
- [Cleo Dronut X1 Pro listing (xboom)](https://www.xboom.in/shop/xboom/cleo-dronut-x1-pro/)
- [Dronut X1 (NewAtlas)](https://newatlas.com/drones/dronut-x1-drone/)
- [Cleo Dronut X1 (RobotReport)](https://www.therobotreport.com/cleo-dronut-x1-drone-designed-for-gps-denied-environments/)
- [Cleo Robotics ducted fan drone, IEEE Spectrum (IEEE)](https://spectrum.ieee.org/cleo-robotics-demonstrates-uniquely-clever-ducted-fan-drone)
- [Flight control for an unmanned aerial vehicle, Cleo Robotics (US11542000B2)](https://patents.google.com/patent/US11542000)
- [Flybotix ASIO dossier, Unmanned Systems Technology 036 (UST036)](https://www.ust-media.com/ust-magazine/UST036/61/)
- [Solvay and Flybotix collaborate on drone (Solvay)](https://www.solvay.com/en/press-release/solvay-and-flybotix-collaborate-on-drone)
- [Foldable and removable protective cage, Flybotix (US12275542B2)](https://patents.google.com/patent/US12275542)
- [Flybotix ASIO X specification flyer, Frontier Precision (Frontier)](https://shop.frontierprecision.com/wp-content/uploads/2026/06/Frontier-Precision-Flybotix-ASIO-X-NDAA-Specification-Flyer.pdf)
- [Pereira, hover and wind-tunnel testing of shrouded rotors, UMD thesis (UMD)](https://drum.lib.umd.edu/handle/1903/8752)
- [MAV shrouded rotor concept, Chopra lab (Chopra)](https://inderjitchopra.umd.edu/?p=127)
- [TiShrov shrouded rotor MAV, J. American Helicopter Society (JAHS)](https://sites.utexas.edu/sirohi/files/2017/07/015_hrishikeshavan11_jahs.pdf)
- [Effects of the rotor tip gap on a ducted rotor in hover (TUDelft)](https://research.tudelft.nl/en/publications/effects-of-the-rotor-tip-gap-on-the-aerodynamic-and-aeroacoustic-/)
- [Tip gap and rotor position in a ducted rotor, Drones 2025 (Drones25)](https://www.mdpi.com/2504-446X/9/1/45)
- [Cinewhoop ducts and props (UAVModel)](https://blog.uavmodel.com/6062-2/)
