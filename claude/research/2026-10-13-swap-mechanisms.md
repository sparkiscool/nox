# Nox Scout v2 battery slider: mechanism research (13 Oct 2026)

Scope: push-through hot-swap of a 165 g 4S pack in a printed PETG cartridge (80 mm long), slider on the drone,
crash load 60 N (35 g x 0.174 kg) in either x direction, thousands of swap cycles, Bambu A1 mini.

Source tags: [S#] refers to the Sources list at the end. "Calc" = my own arithmetic from the cited inputs.
"Inference" = engineering judgement, not a published number.

---

## 0. Headline findings

1. **No published push-through design uses a positive lock.** Both known push-through swaps (Berkeley
   Midair Hotswap [S1][S2], Skyyfish patent US11807130B2 [S9]) hold the pack with an interference joint or
   spring contacts snapping into recesses, i.e. a detent that the pusher simply overpowers. That makes
   retention force = push-out force. Nox's tooth-lock released by the next pack is ahead of that prior art,
   but it is also untested territory: no failure data exist for it.
2. **Patent flag (box side, not the slider):** Skyyfish US11807130B2 (priority 2020-06-03, granted
   2023-11-07, active, est. expiry 2042-08-02) claims a station with two battery holders that have movable
   docking bays with rails, aligned on one axis with the landed drone, and a piston that moves a fresh module
   into the drone while simultaneously pushing the depleted module into an aligned bay of the second holder
   [S9]. A magazine-in / chute-out Nox box is close to this. Needs a claim read before the box design freezes.
   The drone-side slider alone is not what it claims.
3. **Mill-Max says do not side-load spring pins** ("Lateral engagement or side-loading of the piston should be
   avoided since this may cause damage"; use Omniball for side load) [S12]. Nox's horizontal 0854 pins wiping
   along strips are exactly a side-loaded use. Workable if ramps are shallow and steps are eliminated, but this
   is the highest-wear item in the design and needs a cycle test.
4. **A flexure pawl must not carry the crash load through the leaf in compression.** A 0.8 x 4 x 10 mm PETG
   leaf buckles at about 6 N (Calc, Euler fixed-free, E 1.5 GPa) vs 30 N per side needed. The tooth must bear on
   a rigid shoulder of the slider; the leaf only positions the tooth.
5. **Make-before-break overlap is a geometry formula, and it may be very small.** See section 4.5. With 50 mm
   strips, 40 mm group spacing, 80 mm cartridge pitch and a group span g along x, the overlap window where one
   full group is on each pack is 10 - g mm; for the 46 mm strip it is 6 - g mm. If the 4 pins of a group are laid
   out along x (g about 6 to 8 mm) the overlap is near zero, and the 1000 uF cap bridges only about 100 us at
   10 A. Lay each group out across the strip height (g about 2 mm) instead.

---

## 1. Push-through and magazine battery swap systems

### 1.1 UC Berkeley "Midair Hotswap" (UAV@Berkeley student org, Spring 2017) [S1][S2]
- Battery in a case with power leads on **flat copper plates**; quad side has **four 10-prong spring-loaded
  contact pins** that "adjust to the position and height of the copper plates" [S1][S2].
- **Retention: an "interference joint"** on the 3D printed ABS case (Stratasys Fortus, chosen for dimensional
  accuracy). Chosen explicitly to avoid a locking motor; "the main challenge was balancing mechanical simplicity
  against in-flight lock security" [S2].
- **Swap: linear slide rig** (chrome rods, linear bearings, one stepper, two limit switches, Arduino Uno)
  "pushes out the old case while sliding in a new one" [S2].
- Not published: push force, timings, power-continuity circuit, cycle counts. Reported: the quick-release
  electrical connection "did not fail in final tests"; desktop FDM printers failed frequently; only one swap
  per flight; landing was manual on a grounded hexacopter [S2].
- Lesson for Nox: the only public push-through precedent is a short-lived proof of concept with friction
  retention. Nothing to copy for retention numbers.

### 1.2 Berkeley HiPeRLab "Flying Batteries" (Jain & Mueller, arXiv:1909.10091) [S3]
- Not push-through (a battery-carrying drone docks on top), but it is the best documented hot-swap power path:
  - Spring-loaded connectors on the main drone, copper plates on the flying battery's legs.
  - **"Smart bypass diodes" in series with each battery, < 0.1 V drop at about 18 A**; a normally closed relay
    in series with the primary battery; relay coil powered from the secondary input so it cannot open without a
    secondary present [S3].
  - Paper states parallel LiPo connection is only safe within about 0.2 V/cell, hence diodes [S3].
  - Retention purely passive (weight + guide geometry); no contact loss at 12 m/s^2 lateral oscillation;
    failure mode seen: failed electrical contact after docking (drone stayed on primary, battery re-docked) [S3].
  - 18 A operating, 55 A peak current through the contacts [S3].
- Lesson: diode-OR per source is the proven approach; Nox's LM74700 per group is the same idea with lower drop.

### 1.3 Skyyfish US11807130B2 "Battery exchange and charging system for drones" [S9]
- Fresh module pushed by a piston from a holder bay into the drone; as it advances it **pushes the depleted
  module out into an empty bay of a second holder, one motion** [S9].
- Modules ride on rails suspended by wheels; **"spring-loaded contacts snap into recesses, which provides
  mechanical retention as well as electrical connection"** (detent retention) [S9].
- No power-continuity method described [S9]. Status and claim summary in section 0.

### 1.4 UCSB "Eternal Flight" capstone (Toyon) [S4]
- Hexacopter "tanker" swaps a quad's battery: a linear actuator (Actuonix) "pushes in the replacement battery
  and simultaneously pushes out the drained battery" [S4].
- **Spring compression contacts**; continuity achieved by letting **"both batteries temporarily in parallel"**
  [S4]. No diodes. That is exactly what Nox avoids; at 4S with packs at different SOC it causes large
  inter-pack currents (see 0.2 V/cell rule in [S3]).

### 1.5 Other academic swap stations (none publish latch forces)
| System | Retention | Contacts | Time | Hot swap | Result / failures |
|---|---|---|---|---|---|
| MIT ACL (Toksoz 2011, Ure 2015) [S5][S6] | not described in accessible text | not described | not given | yes, "without shutting down" | >90 autonomous swaps in 3 h missions, >90% vehicle utilization [S6] |
| Oulu jig + robot arm (Vilkki 2024, Tello) [S7] | jig funnels; pack pulled from rear | not given | 118 s cycle | no (powered off) | **195/200 OK**; 3 failures = pickup misalignment, 2 = charger LED misread; chamfered dock corrected misalignment 6 times [S7] |
| IPN Mexico (MDPI Drones 2023) [S8] | servo rack gripper | retractable connectors | target <10 s | no | >10 s swap loses ESC calibration [S8] |
| ICAS 2024 aerostat station [S10] | MC-37F push latches + drone-side PQ12 actuators | pogo pins on copper plate | n/a | no | flexing docking board at altitude [S10] |
| SDSU Recon Roost 2025 [S11] | claw pulls cartridge, actuator pushes fresh in | n/a | n/a | n/a | PETG legs, PET-CF frame [S11] |
| Twente "wasp"/SBox [S13] | carriage "locks into place", no detail | n/a | n/a | no | "close mechanical fit" difficult with landing error |
| Boeing EP13382159 / US9481458 [S14] | permanent magnets on UAV, electromagnet on pack cancels them to release | V+, V-, 2 signal | n/a | no | drop-down + lift-up, not push-through |
| Home-robot station (Wu 2012) [S15] | locking device | n/a | 84.2 s avg | "on-line" | deflection of case and docking studied |
| IBM library robot US6909940 [S16] | grippers | n/a | n/a | yes, via back-up battery or capacitor | |
| Ocado bots [S17] | UR10e arm | n/a | < 60 s | "hot swap" claimed | 100 robots / station |

Hot-swap approaches found: diode-OR per source [S3]; P-FET ideal-diode OR proposed for ArduPilot hot swap
[S18]; temporary parallel (risky) [S4]; back-up battery or capacitor during the gap [S16]; DJI Matrice 300 dual
TB60 packs, swap one while powered [S19].

### 1.6 Takeaways for retention in push-through systems
- Every documented push-through relies on detent/friction retention, which ties crash retention to pusher force.
- Every documented automated swap failure is **misalignment at insertion** [S7][S13], not latch failure. Lead-ins
  and passive centering matter more than raw retention.

---

## 2. Slide-in retention options

Requirement: hold 60 N in +x and -x, 35 g impact, release on demand during a push stroke, thousands of cycles,
PETG, low mass.

### 2.1 Flexure tooth lock (positive, released by cam) - Nox T pawl
- Holding: set by tooth shear/bearing and the stop it bears on, not by spring force. Example (Calc): tooth 3 mm
  wide x 1.5 mm engaged, 30 N per side: 6.7 MPa bearing, about 5 MPa root shear (6 mm^2). Printed PETG UTS
  29-52 MPa [S20][S21]. Large margin.
- **Load path rule (Calc):** a cantilever leaf loaded axially in compression buckles early. 0.8 x 4 x 10 mm leaf,
  E 1.5 GPa: P_cr = pi^2 E I / (4 L^2) = 6.3 N, far below 30 N. Put the crash load into a rigid slider shoulder
  behind the tooth (or load the leaf in tension), so the leaf only positions the tooth.
- **Self-locking face:** retention face at 90 deg or slightly undercut (5-10 deg hook) so the load pulls the
  tooth deeper. Snap-fit guideline: 80-90 deg return angles are "permanent"; about 35-45 deg are releasable
  [S22]. Friction angle for PETG about 14-17 deg (mu 0.25-0.3) [S22][S23].
- Insertion/lift force small (Calc below, 2-3 N per pawl). Wear is low because the tooth only slides over the
  cartridge during the stroke.
- Creep: fine if the engaged state is the as-printed (zero-strain) state. Creep only matters if a pawl rests
  deflected on a cartridge flank for hours. PETG clips "lose some springiness" over time (forum, anecdotal) [S24].
- Failure modes: tooth not fully dropping (debris, elephant foot), leaf fatigue, softening above about 60 C
  (Eastar 6763 HDT 64 C at 1.8 MPa [S21]; a LiPo after a hard flight can sit near this; inference).
- Mass: about 0.5-1 g per pawl (inference).

### 2.2 Ratchet pawl (one-way) - Nox N pawl
- Same as 2.1 in the blocking direction; sawtooth lets +x pass. Same buckling and load-path rule.
- Risk: during push-through, the old cartridge must pass N in +x while T must already be lifted. Every
  pawl adds one more "must be in the right state at the right x" condition.

### 2.3 Ball / spring detent
- Holding (Calc, frictionless form F_spring x tan(beta) [S25] plus friction angle): KIPP M4 "strong" plunger
  12-22 N [S26]; on a 45 deg flank with mu 0.2: 22 x tan(56 deg) = 33 N each, two give about 66 N.
- **Symmetric: the pusher must overpower the same 60+ N on every swap**, and the drone seat reacts it.
- **PETG cannot be the detent track (Calc):** 2.5 mm steel ball at 22 N on PETG (E* about 2.4 GPa): Hertz
  contact radius 0.21 mm, peak pressure about 250 MPa vs PETG yield 50 MPa [S21]. The ball will plough a groove;
  holding force decays with cycles. Needs a metal insert.
- Mass: M4 plunger about 1-2 g each plus inserts (inference). Verdict: no.

### 2.4 Magnets (N52)
- K&J D42-N52 (6.35 x 3.2 mm, 0.75 g): 12.7 N to thick steel at contact; D64-N52 (9.5 x 6.35 mm, 3.4 g):
  37.8 N [S27][S28].
- Gap kills force: about 0.76 mm of paper cut a D82 from 6.44 lb to 3 lb (-53%) [S29]. A printed skin plus
  clearance is that kind of gap.
- Shear (sliding) is much weaker: magnet-to-magnet shear about 50% of pull (measured 47-70%) [S30]; magnet on
  steel relying on friction only 10-25% of pull [S29].
- For 60 N along x with about 0.5-1 mm gap: about 4 x D64 (14 g) plus steel targets (Calc). Pusher must also
  overcome it every swap.
- Field use: **Skydio 2 and X2 batteries are held by magnets** [S31][S32]. Skydio warns magnets "may attract
  metallic debris" and that rotating or flipping the drone during hand landing "may cause the battery to become
  dislodged" [S31][S32].
- Verdict: only as a low-force "pull to home" helper (a few N), never as crash retention.

### 2.5 Friction / interference (Berkeley)
- Holding = push-out force by definition; PETG wears and creeps, so both drift over cycles (inference).
- Verdict: no for 60 N.

### 2.6 Over-centre latch
- Very high holding (toggle locks) but needs actuation both ways. Could be tripped by a station finger.
  Adds a pivot or bistable flexure; printed bistable flexures carry constant strain in one state, so creep
  matters (inference). Verdict: viable but heavier and more parts than 2.1.

### 2.7 Latch released by a station finger vs by the next pack
- Power tools: spring-biased latch with an angled flange that snaps in on slide-in; release by pushing a
  lever, pivot placed for about 2:1 leverage (Bosch/SB Power Tool US6168881, expired 2018) [S33]. The release
  is decoupled from the next pack.
- **Station finger (pusher-carriage cam that lifts T before the fresh nose arrives)**: release timing is set by
  the box, not by cartridge-to-cartridge tolerances; failure is visible (finger did not lift = no push).
  Next-pack release: no extra station part, but if the bump is not fully lifted when the fresh nose meets the
  old tail, the stroke stalls against the tooth (inference).

### 2.8 Bayonet
- Needs rotation; incompatible with a single linear push-through stroke. Verdict: no.

### 2.9 Consumer drone crash retention
- DJI Mini 3: slide in, "push until you hear a click"; release by pressing buckles on both sides [S34].
  DJI Avata: side buckles, pull backward [S35].
- Field failures are anecdotal, no rates published:
  - Mavic: a battery that latched on one side only; "one click instead of two" warning sign [S36].
  - Mavic Pro / Mavic 2: mid-air battery fall-outs; suspected partial latching or swelling; users add straps [S37].
  - Spark: small clips wear, design "still allows insertion without the clips fully engaged"; DJI later shipped
    "beefier clips" [S38].
- Lesson: the dominant failure is **a pack that looks seated but is not latched**. Design so a partly latched
  state is impossible or detectable. Nox can detect it electrically: only the fully seated position puts both
  pin groups on one strip, so require both ideal-diode channels to conduct before arming (inference).

---

## 3. PETG / printed flexure and snap-fit rules

- **Material**: Eastar 6763 (moulded PETG): yield 50 MPa, strain at yield 4.0%, E 2.1 GPa, HDT 64-70 C [S21].
  Printed PETG: UTS 29-34 MPa, E 1.04-1.14 GPa (one study) [S20]; 45-52 MPa, E 1.33-1.56 GPa (another) [S39].
- **Allowable strain**: Bayer: single-use 4% for PC; **frequent assembly use about 60% of the single value**
  [S22]. Applying the same logic to PETG (yield 4%): single about 2.5-3%, repeated about 1.5-2% (inference).
  For printed parts use **1.0-1.5%** for >10^3 cycles.
- **Fatigue (printed PETG, flat, longitudinal raster, R = 0.4, 5 Hz)**: 30 MPa -> about 1.0e5 cycles;
  23 MPa -> 1.6e5; 20 MPa -> 1.9e5 [S39]. At E about 1.5 GPa, 1.5% strain is about 22 MPa (Calc), i.e. inside the
  10^5 band. Only the best orientation was tested.
- **Orientation**: build the beam flat in X-Y so bending stress runs along the extrusion lines; if the beam must
  run along Z, cut allowable stress/strain by 50% [S40]. Layer lines must never cross the root of a leaf.
- **Formulas** (Bayer [S22]): tapered beam (h -> h/2) deflection y = 1.09 e l^2 / h; force P = b h^2 E e / (6 l);
  mating force W = P (mu + tan a) / (1 - mu tan a); factor 1.8 at mu 0.6, a 30 deg.
  - Example (Calc): l 12 mm, h 1.2 mm, b 4 mm, e 1.5%, E 1.5 GPa -> y = 1.96 mm lift, P = 1.8 N, W = 2.7 N at
    30 deg ramp, mu 0.5.
- **Root radius**: add radii on the tensile side of the root [S40].
- **Creep**: avoid any leaf held deflected at rest [S40]; PETG holds tension better than PLA but still relaxes
  [S24].
- **Evidence of printed latches lasting 1000+ cycles**: none found with data. Treat cycle life as unproven and
  run a 2000-cycle bench test of the pawl on the actual print settings.

---

## 4. Sliding power contacts and hot-swap continuity

### 4.1 Mill-Max 0854 (current part)
- 9 A at 10 C rise; spring 25 g preload, **120 g (1.18 N) at mid stroke 1.14 mm**, max 2.29 mm; **20 mOhm max**;
  1,000,000 cycles (axial); gold 20 uin plunger/sleeve [S41][S42]. Mill-Max family max 13 A, derated 10.4 A [S43].
- Ratings assume mid stroke; operating near full compression reduces life [S12].
- Side loading should be avoided [S12]. Wiping life of the plunger tip is not rated.

### 4.2 Alternatives
| Contact | Current | Force | Life | Notes |
|---|---|---|---|---|
| Mill-Max Omniball 0945 [S44] | 3.5 A cont / 5 A peak | 55 g mid | 1e6 compression **and rolling** | Ball tip made for side/rolling actuation; stroke only 0.76 mm max; 40 A burst needs 8+ per pole per group |
| Smiths spring probes [S45] | n/a | n/a | "thousands of insertions in a wiping fashion" | engage "from a wide range of angles" |
| Harwin pogo [S46] | 1-2 A | 0.39-1.32 N | 10,000 | too small |
| Anderson Powerpole PP30 (flat wiping leaf) [S47] | 30 A | 3-5 lbf mate | 10,000 no-load; 250 hot-plug at 120 V 30 A | 600 uOhm |
| Makita blade-in-fork clamp terminal (US8715018, lapsed 2018) [S48] | tool-pack class | self-sprung clamp pieces | | clamps flex so sliding friction "doesn't rise much, which reduces wear" |
| TE low-profile battery connector [S49] | 5 A | | 5,000 | |

### 4.3 Gold strips (cartridge side)
- **Hard gold 30 uin over >= 50 uin Ni: "expected to survive 1,000 cycles before wear through"** (PCBWay) [S50].
  Vendor table: 15 uin about 100 cycles, 30 uin 500+, 50 uin 1000+ [S51]. Samtec tests 30 uin to 100 cycles;
  10 uin fails porosity tests [S52]. ENIG (2-5 uin) is not a wiping finish.
- Minimum end-of-life normal force for gold about 30-40 g; "the more points of contact the better" [S52].
  0854 at 120 g is comfortably above.
- **Specify 50 uin hard gold over 100-150 uin Ni for thousands of wipes** (inference from [S50][S51]).

### 4.4 Ramps, steps and debris
- Edge-connector practice: bevel the leading edge **20-45 deg, 30 deg standard** [S53].
- Lateral force on a pin climbing a ramp of angle t: F_lat = F_spring tan(t + phi) (Calc). With 1.18 N and
  phi about 17 deg: 30 deg ramp -> 1.3 N side load; 20 deg -> 0.9 N; a vertical step catches.
- Rules (inference): no steps anywhere in the pin path; make each cartridge side one PCB with gold pads and
  solder mask elsewhere so the pin rides on a flush surface (mask step about 25 um); put the only ramps at the
  cartridge nose/tail at <= 20-25 deg so pins are pre-compressed to mid stroke before reaching gold; break the
  cartridge-to-cartridge joint so the pin drop into the valley is < 0.3 mm.
- Debris: wiping contacts self-clean; magnets collect metal debris [S31][S32]. Avoid the pin riding on raw PETG
  (it shaves plastic onto the strips) and on bare FR4 (glass is abrasive) (inference).

### 4.5 Current and continuity numbers (Calc)
- Per group 4 x 9 A = 36 A at 10 C rise; 10 A continuous = 2.5 A/pin; 40 A burst = 10 A/pin (slightly over the
  9 A continuous rating, acceptable for seconds).
- Group resistance (max) 20/4 = 5 mOhm; loop through + and - groups 10 mOhm; 0.4 V and 16 W at 40 A for one
  group pair (worst case max-resistance pins), 1 W at 10 A.
- **1000 uF hold-up: t = C dV / I = 1 mF x 1 V / 10 A = 100 us.** It covers pin bounce only, not a gap.
- **Overlap window** (cartridge pitch P, strip length L, group spacing D, group x-span g, strips centred):
  - one full group on each pack: window = L + D - P - g
  - no group straddling old and fresh strips: P - L > g
  - both groups on the strip when seated: D + g <= L
  - Nox numbers: P 80, D 40: L = 50 -> 10 - g; L = 46 -> 6 - g. With g = 7.5 (4 pins along x at 2.5 mm):
    2.5 mm and -1.5 mm. At about 230 mm/s (80 mm in 0.35 s), 2.5 mm is about 11 ms.
  - Fix: put each group's 4 pins in a column across the strip height (g about 2 mm), giving about 8 mm overlap
    on the 50 mm strip (about 35 ms), and check the 46 mm side the same way.
- Ideal diode: LM74700-Q1 reverse-current block < 0.75 us, 20 mV forward regulation, 3.2-65 V [S54]. With OR
  diodes, the fresher (higher voltage) pack takes the load as soon as it connects, so the old group breaks at
  near-zero current (inference); the make event still sees inrush into the cap and load.

---

## 5. Rails for printed slides

- **Friction**: PETG on steel static about 0.27 (Calc from 8 N at 30 N) [S23]; Bayer like-on-like factor 1.2 x
  plastic-on-steel [S22]; PETG-PETG estimate 0.3-0.4. Design value **0.5** (2 x expected, per PBC) [S55].
  Orientation affects wear more than friction; best wear with layers orthogonal to sliding [S56].
- **Jamming (sticky drawer)**: binds when moment-arm / bearing-length > 1/(2 mu): D1 < L1 / (2 mu) [S55]; same as
  h/w > 1/(2 mu) in the Black & Decker non-binding-slide patent [S57]; Kalsi: threshold mu = L/D [S58].
  At mu 0.5: keep the pusher offset (from the bearing line) smaller than the engaged length.
  - Worst case is the entry: when the fresh cartridge has only a few mm in the first groove segment, L1 is tiny.
    Fix: box rails continue the slider rails so the cartridge is supported over >= 2 segments before the
    pusher loads it, and push at rail height on the centre line.
- **Clearances (FDM)**: PLA/PETG sliding 0.2-0.3 mm per side, 0.25 mm typical [S59]; 0.4 mm for slide fits
  [S40]; about 2 x layer height as a start [S60]. XY dimension governs.
- **Profiles (inference + friction math)**:
  - Flange in C-groove / T-slot (current): lowest normal-force amplification, tolerates FDM error; needs lead-ins.
  - Dovetail / V-groove: self-centring but wedge amplifies friction by 1/sin(half angle), 1.41 x at 90 deg; more
    jam-prone with printed surfaces.
  - Round rods (2-3 mm steel or carbon on the slider, open half-bushing grooves in the cartridge): lowest
    friction and wear, open both ends so push-through still works; about 1-2 g added.
- **Lead-ins for a 0.5-1 mm misaligned pusher**: entry chamfers >= 1.5 x misalignment (1.5 mm x 45 deg on
  groove mouths and flange corners), a floating or rounded pusher head on the centre line, short bearing pads
  rather than long continuous contact. Chamfered docks corrected misalignment in the Oulu tests [S7].

---

## 6. Recommended numbers to design to

| Item | Value | Basis |
|---|---|---|
| Crash hold per direction | >= 60 N, design 90 N (1.5 x) | brief |
| Tooth load path | into rigid shoulder; leaf carries no axial load | Calc, buckling 6 N |
| Leaf strain | <= 1.5% (target 1.0%) | [S22][S39] |
| Leaf build | flat in XY, no layer line at root, root radius >= 0.5 mm | [S40] |
| Pawl lift force | 2-3 N each | Calc |
| Total push force | expect 15-25 N; actuator >= 40 N with stall detect | Calc |
| Strip plating | hard gold 50 uin over >= 100 uin Ni | [S50][S51] |
| Pin ramps | <= 20-25 deg, no steps, pins pre-compressed before gold | [S53], Calc |
| Pin working stroke | mid stroke 1.1 mm (0854) | [S41] |
| Make-before-break overlap | >= 5 mm at stroke speed | Calc |
| Rail clearance | 0.25 mm per side (tune 0.2-0.4) | [S59] |
| Design friction | 0.5 | [S55] |
| Entry chamfer | 1.5 mm x 45 deg | inference |
| Seat check | both diode channels conducting before arming | inference |

---

## Sources

- [S1] UAV@Berkeley, Midair Hotswap (2017): https://uav.studentorg.berkeley.edu/2017/05/01/midair-hotswap/
- [S2] UAV@Berkeley project page: https://uav.studentorg.berkeley.edu/projects/midair-hotswap
- [S3] Jain & Mueller, Flying Batteries, arXiv:1909.10091: https://arxiv.org/abs/1909.10091 ; IEEE Spectrum: https://spectrum.ieee.org/swappable-flying-batteries-keep-drones-aloft-almost-forever
- [S4] UCSB Eternal Flight: https://engineering.ucsb.edu/sites/default/files/images/virtualOpenHouse/CE%20eternal_flight.pdf
- [S5] MIT DSpace, Automated Battery Swap and Recharge: https://dspace.mit.edu/handle/1721.1/81474
- [S6] MIT ACL project: https://acl.mit.edu/projects/automated-battery-management-system
- [S7] Vilkki et al., Machines 2024, 12, 818: https://www.mdpi.com/2075-1702/12/11/818/htm
- [S8] MDPI Drones 7(4) 270: https://www.mdpi.com/2504-446X/7/4/270
- [S9] Skyyfish US11807130B2: https://patents.google.com/patent/US11807130B2/en
- [S10] ICAS 2024 paper 1116: https://www.icas.org/icas_archive/icas2024/data/papers/icas2024_1116_paper.pdf
- [S11] SDSU Recon Roost poster: https://aerospace.sdsu.edu/design-day/files/2025/recon_roost_poster.pdf
- [S12] Mill-Max general application notes: https://www.mill-max.com/engineering-notebooks/general-application-notes
- [S13] U. Twente wasp/SBox: https://ris.utwente.nl/ws/files/103952838/autonomous.pdf
- [S14] US9481458 (EP13382159 priority): https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9481458
- [S15] Wu et al. 2012, Int J Adv Robot Syst: https://doaj.org/article/1c76a639414f4ea1a86b91e3f286d712
- [S16] IBM US6909940: https://patents.justia.com/patent/6909940
- [S17] Ocado battery swap: https://ocadointelligentautomation.com/systems/battery-swap
- [S18] ArduPilot forum, hot swapping: https://discuss.ardupilot.org/t/hot-swapping-batteries-on-drone/108622
- [S19] Heliguy TB60: https://heliguy.com/products/matrice-300-tb60-battery
- [S20] PETG annealing / layer height study: https://pmc.ncbi.nlm.nih.gov/articles/PMC10342851
- [S21] Eastar 6763 datasheet: https://protolabs.com/media/pzub5tfo/eastar-6763.pdf
- [S22] Bayer snap-fit guide: https://cdn.sparkfun.com/assets/home_page_posts/1/4/1/0/Plastic_Snap_fit_design.pdf ; JMES cantilever hook study: https://journal.ump.edu.my/jmes/article/view/8758
- [S23] Tribology in Industry 2024 (PETG vs steel): https://tribology.rs/journals/2024/2024-1/9-1546.pdf
- [S24] Bambu forum, repeated flexing: https://forum.bambulab.com/t/pla-tough-vs-repeated-flexing/231087
- [S25] Detent mechanics: https://www.firgelliauto.com/blogs/mechanisms/detent
- [S26] KIPP K0322 spring plungers: https://www.marek.eu/data/katalogove-listy/kipp/K0322.pdf
- [S27] K&J D42-N52: https://www.kjmagnetics.com/proddetail.asp?prod=D42-N52
- [S28] K&J D64-N52: http://www.kjmagnetics.com/d64-n52-neodymium-disc-magnet
- [S29] K&J, How much will a magnet hold: http://www.kjmagnetics.com/blog/how-much-will-a-magnet-hold
- [S30] K&J, Magnet shear force: http://www.kjmagnetics.com/blog/magnet-shear-force
- [S31] Skydio 2 user guide: https://support.skydio.com/hc/article_attachments/360096640034/Skydio2_UserGuide_CU1_compressed_v3.pdf
- [S32] Skydio X2 battery support article: https://support.skydio.com/hc/articles/1260804644430 ; Droneblog: https://www.droneblog.com/skydio-battery/
- [S33] US6168881 (SB Power Tool): https://patents.google.com/patent/US6168881
- [S34] Heliguy, DJI Mini 3 battery: https://www.heliguy.com/blogs/knowledge-base/how-do-i-insert-and-remove-the-intelligent-flight-battery-for-the-dji-mini-3/
- [S35] Droneblog, Avata battery: https://www.droneblog.com/dji-avata-remove-battery/
- [S36] MavicPilots, battery latches: https://mavicpilots.com/threads/battery-latches.58083
- [S37] MavicPilots, mid-air fall out: https://mavicpilots.com/threads/mid-air-battery-fall-out-mavic-pro.94342/post-1061573 ; https://mavicpilots.com/threads/my-mavic-2-zoom-fell.116897/post-1318287
- [S38] SparkPilots, battery eject: https://sparkpilots.com/threads/a-very-quick-and-simple-solution-to-the-battery-eject-problem.10885/latest
- [S39] PETG fatigue, J. Compos. Sci. 5(3) 76: https://www.mdpi.com/2504-477X/5/3/76
- [S40] Core77 / Fictiv snap-fit guide: https://www.core77.com/posts/65318/how-to-design-snap-fit-components
- [S41] Farnell, 0854-0-15-20-82-14-11-0: https://it.farnell.com/en-IT/mill-max/0854-0-15-20-82-14-11-0/spring-loaded-pin-9a-15-21mm/dp/2751174
- [S42] Mill-Max catalog page 028.1: https://www.mill-max.com/sites/default/files/external/catalog/2020-03/028.1.pdf
- [S43] Mill-Max FAQ: https://www.mill-max.com/engineering-notebooks/spring-loaded-pogo-pins-connectors/faqs
- [S44] Mill-Max Omniball PR697: https://www.mill-max.com/sites/default/files/external/assets/2020-05/pr697_-_omniball_spring-loaded_contacts_ms_0.pdf
- [S45] Smiths Interconnect dovetail white paper: https://www.smithsinterconnect.com:443/getattachment/f800082f-c95f-456f-8256-38209bd3af30/Dovetail-Connector-Series-White-Paper_1.pdf
- [S46] Harwin pogo pins: https://cdn.harwin.com/pdfs/Harwin_Datasheet-Pogo-Pins-Contact-Pads.pdf
- [S47] Anderson PP30: https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/7185/AndersonPower_09092019_1330G5.pdf
- [S48] Makita US8715018: https://patents.google.com/patent/US8715018
- [S49] TE mobile battery connectors: https://www.farnell.com/datasheets/2060511.pdf
- [S50] PCBWay hard gold: https://www.pcbway.com/blog/Engineering_Technical/Understanding_PCB_Manufacturing__Hard_Gold_Plating.html
- [S51] Telewire plating guide: https://www.telewiretech.com/blogs/technical-resources/the-definitive-guide-to-connector-plating-thickness-15μ-gold-30μ-gold-and-tin-compared
- [S52] Samtec plating FAQ: https://blog.samtec.com/post/connector-plating-faqs-normal-force-cycles-temperature-and-other-essentials/
- [S53] Eurocircuits edge connectors: https://www.eurocircuits.com/pcb-design-guidelines/edge-connectors/
- [S54] TI LM74700-Q1: https://www.ti.com/product/LM74700-Q1
- [S55] PBC Linear 2:1 white paper: https://media.pbclinear.com/pdfs/pbc-linear-white-papers/white-paper-demystifying-the-2-1-ratio-and-stick-slip-phenomenon.pdf
- [S56] Mahmood et al., Tribology Online 2023: https://www.jstage.jst.go.jp/article/trol/18/6/18_302/_article
- [S57] US9884416 non-binding linear slides: https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9884416
- [S58] Kalsi handbook D21, sticky drawer: https://www.kalsi.com/handbook/D21_Sticky_drawer_effect.pdf
- [S59] QIDI snap-fit clearance guide: https://qidi3d.com/blogs/print-lab/3d-printed-snap-fit-joints-clearance-guide
- [S60] GrabCAD sliding clearance: https://grabcad.com/tutorials/how-to-fdm-print-for-part-sliding-clearance
