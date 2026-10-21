# Drone docks: alignment, contacts, swap mechanisms (11 Oct 2026, run 2)

**Bottom line**
1. Every dock lands first, then centers passively (DJI ramps, ASU V-grooves) or with leg pushers (Brookhurst patent). Landing errors are ±15 to 17 cm (Hextronics); DJI publishes none.
2. The two charge-in-place market leaders (DJI, Skydio) charge through contacts and take 27 to 35 min. Swappers (Hextronics, RAPID) take 60 to 80 s. Heisha tested swap arms and chose contact charging as cheaper and more stable.
3. Best steals for the box: polarity-free V-groove contacts with a diode bridge for keep-alive power, contact sensors in place of tug checks, and pushing on legs or hard points only, then clamping.

Not repeated here: market swap-time table (`catcher-speed-research.md`) and landing-accuracy survey (`hover-catch-mechanism-survey.md`).

## Findings

**DJI Dock 2 / 3 (charge only, no swap).** Dock 2 centers the aircraft with a "slide-ramp centering design" after RTK + marker landing. Charge is 20 to 90% in ~32 min, and the dock does not support battery replacement (DJI Dock 2 FAQ). Install manual: keep the pad clear of metal objects, aircraft nose to the arrow, no light-colored objects within 5 m (marker false detection). Dock 2 weighs ~34 kg (IT Home). Dock 3: 55 kg, charge 15 to 95% in 27 min, landing in wind up to 12 m/s, aircraft RTK hover ±0.1 m. Its aircraft battery is 149.9 Wh, 640 g, 1.8C max charge, 400 cycles (Dock 3 spec). On the original Dock, the M30 Dock version adds "a charging interface and charging management module at the landing gear" (Heliguy). So contacts are on the legs and the charge controller rides on the aircraft.

**Skydio Dock for X10.** Charge 15 to 95% in ~35 min (up to 73 min in extreme temperatures), launch ~20 s, 229 lb, internal HVAC, IP56 (Skydio FAQ). Contact method not published.

**Heisha (D50, D135).** D50: 3 cm landing claim, 40 min charge, 50 cm landing board, 55 kg (DroneDJ). D135 for M300 "still adopts the classic contact method." Heisha "tested mechanical arms but found them costlier and less stable than the charging pad" (sUAS News).

**Hextronics.** Universal: robotic swap in as little as 80 s, ±15 cm landing accuracy, 31.75 kg (Advexure). Atlas 300: "17 cm of radial landing error recovery," 8 batteries, under 5 min downtime (Drone Nerds). Swap hardware is not published.

**Percepto Base.** Markets a "patented docking & charging mechanism"; no specs public (Percepto).

**Fraunhofer RAPID.** Platform carries the landed drone into the station, swaps both packs, and keeps the drone powered throughout. ~60 s total (RAPID).

**Patents worth reading (mechanisms).**
- **US12286252B2 (Brookhurst Garage, granted Apr 2025, active).** Pushers slide on the parking plate and push the drone's **legs** to a reference spot. Two flaps stand up to grip the body, and a sensor checks a marker on the drone. A shutter opens so a carrier reaches the pack **from below**. The pack slides along a guide channel into a port, and two sensors confirm it: one for touching the guide, one for being fully seated.
- **US11465776B2 (Arizona State, granted 2022, active).** Two pads with brass **V-grooves** and tapered non-conductive flanks funnel conductive strips on the landing skids into the groove. The gear is mirror-symmetric and a diode array on the drone makes polarity irrelevant. ~300 W at ~90% efficiency.
- **US11267583B2 (Fehler, individual, granted 2022).** One rack-and-pinion pushes two contacts outward into tapered receivers. The same motion makes power and **locks the pack**, and retracting it releases the pack.

## Steal for Nox
- **Box catcher:** passive lead-in first (DJI ramps, ASU tapered flanks), then a clamp. Matches the iris; the Brookhurst order is push legs, verify, then grip.
- **Box swapper (Overseer):** bottom access through an opening while the drone is clamped (Brookhurst) is the same architecture as the Overseer's bottom swap. Two contact sensors (guide touch, fully seated) are a cheap check that doesn't depend on Z current.
- **Keep-alive contact:** brass V-groove pads under the catch frame plus a diode bridge on the drone give polarity-free power at any landing orientation (ASU). A candidate for the powered swap in `catcher-speed-research.md`.
- **Scout cartridge / Scout v2 contacts:** contacts that also latch (US11267583B2) could merge the latch and the pads into one moving part. Patent active to ~2038 (E).
- **Charge controller on the drone (DJI M30 Dock version):** keeps the box side a dumb DC supply.
- **Fallback:** if the swap proves unreliable, charge in place is what DJI, Skydio and Heisha ship (E: ~30 min at ~1.8C).

## For Spark
- `box.md` (swappers, X9): consider adding a guide-touch and a fully-seated sensor to the swapper, alongside the hall lift trip in `scout-swap-arm.md`. Brookhurst uses both (E).
- `box.md` / `overseer.md` (catch frame): a V-groove keep-alive contact on the iris ledges, with strips on the catch-frame corner soles, would power the drone during the swap without an extra connector move (E, ASU patent). Touches the catch frame: Spark's call.
- `v2/dimensions.md` (Contacts, 3 spring pins per pole): look at US11267583B2's extend-to-latch contacts before choosing pins (E).
- IP: US12286252B2 claims a parking plate + body gripper + chargers + carrier swap station, and the box has the same four elements. Get a claims read before selling (E). Fine for a demo.

## Sources
- [DJI Dock 2 FAQ (DJI, CN)](https://enterprise.dji.com/cn/dock-2/faq)
- [DJI Dock 2 Installation and Setup Manual](https://xcopter.com/download/DJI_Dock_2_Installation_And_Setup_Manual_v1.0_ENII.pdf)
- [DJI Dock 2 launch (IT Home)](https://www.ithome.com/0/731/071.htm)
- [DJI Dock 3 specs (Korec)](https://www.korecgroup.com/wp-content/uploads/2025/09/DJI-Dock-3-Specs-DJI.pdf)
- [M30 vs M30 Dock version (Heliguy)](https://www.heliguy.com/blogs/knowledge-base/what-s-the-difference-between-the-dji-m30-series-and-the-m30-series-dock-version/)
- [Skydio Dock for X10 FAQ](https://www.skydio.com/dock/faqs)
- [Heisha D50 (DroneDJ)](https://dronedj.com/2022/05/27/heisha-d50-drone-dock-dji-autel/)
- [Heisha D135 insight (sUAS News)](https://www.suasnews.com/2021/02/insight-into-heisha-d135-drone-charging-station-for-dji-matrice300/)
- [Hextronics Universal (Advexure)](https://advexure.com/products/hextronics-universal)
- [Introducing Hextronics (Drone Nerds)](https://enterprise.dronenerds.com/?p=4822)
- [Percepto Base](https://percepto.com/drone-in-a-box/percepto-base/)
- [RAPID system components (Fraunhofer CML)](https://rapid2020.eu/system-components/)
- [US12286252B2, Brookhurst Garage](https://patents.google.com/patent/US12286252)
- [US11465776B2, Arizona State](https://patents.google.com/patent/US11465776)
- [US11267583B2, Fehler](https://patents.google.com/patent/US11267583B2/en)
