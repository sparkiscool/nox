# Mounting dimensions research (22 Oct 2026)

Background for `nox/dimensions.md` section M (the numbers in use live there). Three web passes, 22 Oct 2026: every source below was opened (pages read, PDFs and drawings rendered, CAD parsed); search snippets were not used. Tags: spec = maker page, drawing or CAD; listing = store page; typical = one maker of a generic part; derived = measured from a maker's drawing or CAD; unverified = sources disagree or no primary source. Nothing here is measured on our parts (P) yet.

Sections: 1 drone electronics, 2 airframe, propulsion and lidar, 3 box mechatronics.

## 1. Drone electronics

Tags: **spec** = manufacturer page/drawing/file; **listing** = store page; **unverified** = sources disagree, or no primary source found.
Every URL below was opened (page text read, and the drawing images or files downloaded and checked) unless marked "search only".

---

### 1.1 MicoAir743v2-AIO-35A (H743 v2 AIO, 35 A 4-in-1 ESC)

| Item | Value | Tag | Source |
|---|---|---|---|
| Board L x W x H | 36 x 36 x 8 mm | spec | https://micoair.com/flightcontroller_micoair743v2_aio_35a/ (text + drawing `wp-content/uploads/2026/04/MicoAir743v2-AIO-35A_size.webp`) |
| Board L x W x H | 36 x 36 x 8 mm (agrees) | listing | https://www.rotorama.com/product/micoair-h743-v2-aio |
| Mass | 10 g | spec | MicoAir page/drawing above; Rotorama and Pyrodrone also give 10 g |
| Mounting pattern (c-c) | 25.5 x 25.5 mm square, 4 holes | spec | MicoAir drawing (25.5 mm marked on both axes) |
| Hole diameter / screw | Ø3 mm, labelled "(M3)" on drawing | spec | MicoAir drawing |
| Mounting pattern | 25.5 x 25.5 mm, Φ3 mm (agrees) | listing | https://pyrodrone.com/products/micoair-h743-v2-aio-flight-controller-35a-2-6s-4in1-esc-25-25mm (title says "25x25mm", which is the frame-class name) |
| Grommet size | not found. The MicoAir page lists no package contents. Rotorama lists "1x Set of soft mounts" but gives no size. | not found | Rotorama page above |
| Connector positions (from top-view drawing) | USB-C on the left edge, lower half. Battery −/+ pads on the bottom edge, left of centre (R001 shunt next to them). GPS SH connector on the top edge, right of centre. DJI/HD-VTX SH1.0-6P connector on the right edge, upper half. BOOT button left of centre. Motor pads along all four edges between the holes. Height is 8 mm including connectors. | spec | MicoAir drawing |
| Other connectors | USB-C, SH1.0-6P DJI (pin 1 = 12 V), microSD slot, I2C for compass | spec | MicoAir page |

Older Rotorama listing for the earlier AM32 version (https://www.rotorama.com/product/micoair-743-35a-aio, discontinued): 36x36x8 mm, 25.5x25.5 mm, 10 g. Same footprint.

### 1.2 MicoAir743-Lite flight controller

| Item | Value | Tag | Source |
|---|---|---|---|
| Board L x W x H | 36 x 36 x 8 mm | spec | https://micoair.com/flightcontroller_micoair743lite/ (text + drawing `wp-content/uploads/2025/11/MicoAir743-Lite_size.webp`) |
| Board L x W x H | 36 x 36 x 8 mm (agrees) | spec (PX4 docs copy) | https://docs.px4.io/main/en/flight_controller/micoair743-lite |
| Mass | 10 g (board). The drawing also shows the "MicoAir743-Lite Flight Stack" (FC + MicoAir ESC) at 17.5 mm tall and 24 g. | spec | MicoAir drawing |
| Mounting pattern (c-c) | 30.5 x 30.5 mm | spec | MicoAir drawing, PX4 docs |
| Hole diameter | Ø4 mm | spec | MicoAir drawing ("Φ4mm"), PX4 docs |
| Screw size | not stated by the manufacturer. Ø4 mm holes normally take M3 screws through rubber grommets, but that is not confirmed. | not found | – |
| Connector positions | All connectors are SH1.0, mounted at the board edges. Top edge: 2 connectors, one is the ESC connector at top right. Left edge: 3 (UART, UART, LED/BUZZER). Bottom edge: 3 (GPS, UART1, SBUS/CRSF). Right edge: 2 upper connectors with an SWD label below them, and the BOOT button near the lower right edge. The drawing does not clearly show where the USB-C port is (the page lists USB Type-C), so check it on the board. MicroSD slot is left of centre. | spec | MicoAir drawing, PX4 interface diagrams |
| Store page | Shipping box only (15x10x4 cm, 0.2 kg), not useful | listing | https://store.micoair.com/product/micoair743-lite/ |

### 1.3 MicoAir MTF-02P optical flow + ToF

| Item | Value | Tag | Source |
|---|---|---|---|
| Body L x W x H | 21.6 x 16 x 6.5 mm | spec | https://micoair.com/?p=4526 (text + drawing `wp-content/uploads/2024/10/MTF-02P_size-jpg.webp`) |
| Mass | 1.5 g | spec | MicoAir page |
| Mass | 1.8 g | listing (conflicts) | https://rotorvillage.ca/micoair-mtf-02p-optical-flow-6m-range-lidar/ |
| Mounting holes | **None.** The top-view drawing shows no holes, and neither the MicoAir page nor the Rotorvillage page lists any. The mount must clamp the PCB or be a pocket/tape fit. | spec | MicoAir drawing |
| Lens / emitter positions (top view, SH connector on the right) | ToF emitter/receiver window (twin black lenses) at the upper left. Optical-flow camera lens below it, left of centre. The ToF block is the tallest part (6.5 mm total including the PCB). | spec | MicoAir drawing |
| Connector | SH1.0-4P (Tx, Rx, 5V, GND), on the right short edge, facing sideways | spec | MicoAir connector image `MTF-02P_connecotr-jpg.webp` |
| Other | The protocol-select jumper is on the back of the board, so keep it reachable or leave it set. Optical-flow FOV is 42°; the ToF emitting angle is 2°. Keep both windows clear and facing down. | spec | MicoAir page |

### 1.4 DECXIN 2633V2 USB global-shutter camera, 102°

| Item | Value | Tag | Source |
|---|---|---|---|
| Board size | not found. No page names a "2633V2". Several searches (DECXIN/Dechuangxin, "2633V2", "DECXIN-2633", Amazon search) found only other DECXIN models. Those are AR0234 2MP 38x38 mm boards, 38x38x23.9 mm with lens, on Alibaba listings that returned empty content when opened. | not found | Amazon search page opened (no DECXIN results); Alibaba listing https://www.alibaba.com/product-detail/DECXIN-1080P-120fps-2MP-HD-Global_1601125785036.html returned empty |
| Mounting holes (notes: M2 on 28 x 28) | not found | unverified | – |
| Lens barrel diameter / height | not found | not found | – |
| Board thickness | not found | not found | – |
| Mass | not found | not found | – |

38 x 38 mm is a common board size for this class of USB global-shutter modules (ELP and others), but nothing ties it to the 2633V2. **Measure the actual part or get the seller's drawing.**

### 1.5 RadioMaster XR2 (Nano) ELRS 2.4 GHz receiver

| Item | Value | Tag | Source |
|---|---|---|---|
| Product name | "XR2 Nano 2.4GHz ExpressLRS Receiver" | spec | https://www.radiomasterrc.com/products/xr2-nano-2-4ghz-expresslrs-receiver |
| Board L x W x H | 16 x 12 x 6 mm | spec | RadioMaster page |
| Mass | 0.8 g | spec | RadioMaster page |
| Antenna | Integrated 2.4 GHz tower antenna (on-board, no external whip or u.FL) | spec | RadioMaster page |
| Mounting holes | None. Castellated solder pads; no mounting holes listed. | spec | RadioMaster page |
| Connections | CRSF bus + second UART bus. Ships with a CRSF wire, 3 heat-shrink tubes and a manual card. | spec | RadioMaster page |

### 1.6 Radxa Zero 3W (confirming against Radxa's 2D drawing)

| Item | Value | Tag | Source |
|---|---|---|---|
| Board L x W | 65.0 x 30.0 mm (layer BOARD_GEOMETRY_OUTLINE, extents 65.0016 x 30.0055) | spec | Radxa 2D DXF V1.11 `radxa_zero_3w_v1110_top.dxf` in https://dl.radxa.com/zero3/docs/hw/3w/radxa_zero_3w_2d_dxf.zip (linked from https://docs.radxa.com/en/zero/zero3/download) |
| Hole centres (origin at board corner) | (3.55, 3.60), (61.40, 3.60), (3.60, 26.45), (61.40, 26.50) | spec | DXF CIRCLE entities, layer PIN_TOP |
| Hole spacing (c-c) | X: 57.85 (bottom pair) / 57.80 (top pair). Y: 22.85 (left pair) / 22.90 (right pair). **Matches your 57.85 x 22.85 to within 0.05 mm.** The DXF is slightly non-square, probably drafting rounding. | spec | DXF |
| Hole diameter | Ø2.82 mm (r = 1.4097), for M2.5. **Matches your Ø2.8.** | spec | DXF |
| 40-pin header | 2 rows at y = 25.405 / 27.945, x from 8.32 to 56.58 (pitch 2.54) | spec | DXF |
| Mass / height | not checked (only confirmation of holes requested) | – | – |

### 1.7 Matek FCHUB-12S V2 PDB

| Item | Value | Tag | Source |
|---|---|---|---|
| Board L x W x H | 55 x 50 x 6 mm | spec | https://www.mateksys.com/?portfolio=fchub-12s (V2 section) |
| Mass | 21 g | spec | Matek page |
| Mounting pattern (c-c) | 30.5 x 30.5 mm | spec | Matek page |
| Hole diameter | Φ3 mm | spec | Matek page ("Mounting: 30.5 x 30.5mm, Φ3mm") |
| Screw size | M3 | listing | https://www.rotorama.com/product/matek-xclass-pdb-fchub-12s ("30.5x30.5mm M3"; also 55x50x6 mm, 21 g) |
| Pad / connector layout | Main battery +/− pads on a tab at the top edge. 4 ESC pad groups (+, −, S, TLM, G) at the four corners. JST-SH 8-pin FC connector at the bottom centre, just above the bottom edge. 5V, 12V, 12VSW, VBat and PIO pads along the bottom edge and sides. The selectable "V" (12V/VBat) solder jumper is near the JST connector. | spec | Matek V2 quick-start PDF https://www.mateksys.com/downloads/Manual/FCHUB-12S-V2_Manual.pdf (layout figure) |
| Package | FCHUB-12S V2, SH1.0 8-pin cable 5 cm, 2 SH1.0 8-pin connectors. Rotorama also lists a 390 µF 63 V capacitor. | spec / listing | Matek page / Rotorama |
| Note | The 30.5 mm pattern matches the MicoAir743-Lite (30.5 mm) but not the 743v2-AIO (25.5 mm). | – | – |

The Cirkit Designer page from search results lists 36 x 36 mm and 7 g. That page was not opened, and it disagrees with Matek, so it is ignored.

### 1.8 Generic 5 V 5 A UBEC (typical values only)

| Example | L x W x H | Mass | Tag | Source |
|---|---|---|---|---|
| Hobbywing UBEC 5A (2-8S, 5/6/7.4 V, 5 A cont., 15 A peak) | 50 x 17 x 10 mm | 21 g | spec | https://www.hobbywing.com/en/products/ubec-5a93 |
| Hobbywing UBEC 3A (for size comparison, 3 A) | 43 x 17 x 7 mm | 11 g | spec | https://www.hobbywing.com/en/products/ubec-3a-2-6s99.html |
| Lumenier UBEC 5A (2-6S) | 46 x 25 x 7 mm | 12 g | unverified (search only; GetFPV returned 403) | https://www.getfpv.com/lumenier-ubec-5v-5-5v-6v-5a-2-6s-input.html |
| Turnigy 6A UBEC (2-6S) | 45 x 23 x 10 mm | 18 g | unverified (search only; HobbyKing returned 403) | https://hobbyking.com/en_us/turnigy-6a-6v-25v-switch-mode-ubec-w-selectable-voltages.html |

**Typical:** about 43–50 x 17–25 x 7–10 mm, 11–21 g, usually heat-shrink wrapped with no mounting holes (zip-tie or tape mount). Your note of 44 x 20 is within this range. **Measure the actual unit:** there is no single "generic" size. Leave room for the input/output leads at both short ends.

## 2. Airframe, propulsion and sensors

Researched 2026-10-22. All sources listed were opened (web pages fetched; PDFs, drawings and CAD downloaded and read).
Tags: **spec** = manufacturer page / manual / drawing / CAD; **listing** = store page; **unverified** = sources disagree, the meaning is inferred, or there is no primary source.
"Derived" means measured or calculated by me from a manufacturer drawing or CAD file, not printed as a number by the maker.

---

### 2.1 SpeedyBee Bee35 3.5" cinewhoop frame

| Item | Value | Tag | Source |
|---|---|---|---|
| Wheelbase | 153 mm | spec | [S1], [S3] |
| Overall L x W x H | not found (no frame outline dims published) | – | – |
| Frame mass | 138 g (CNC heat sink 15 g, camera gimbal 28 g, listed separately) | spec | [S1] |
| Versions | Bee35 has a carbon bottom plate. Bee35 Pro has an aluminium-alloy heat-sink bottom plate. There is also a Pro LED variant. | spec | [S1], [S2] |
| Top plate thickness | 3.5 mm | spec | [S1], listing [S3] |
| Bottom plate thickness | 2 mm | spec | [S1], listing [S3] |
| FC stack patterns | 20 x 20, 25.5 x 25.5 (AIO), 30.5 x 30.5 mm. SpeedyBee says 30.5 is "not recommended" and recommends the 20 mm mini stack or 25.5 AIO. | spec | [S1], [S2] p.3 |
| FC hole diameter / screw size | not found (the manual and product page do not state it) | – | – |
| VTX patterns | 20 x 20 and 25.5 x 25.5 mm (45/90 deg orientations) | spec | [S1] |
| Motor mount pattern | 12 x 12 mm, M2 (manual: "M2 (6mm) screws are recommended") | spec | [S2] p.3 |
| Recommended motors / props | 1806 / 2004 / 2006 motors, 90 mm props | spec | [S1] |
| ESC/motor mounting height | 24 mm (the product page does not explain the datum) | spec | [S1] |
| Camera mount width | "Camera mounting distance: 19/20 mm" (fits 19 mm micro or 20 mm cameras) | spec | [S1], listing [S3] |
| Battery strap | strap supports horizontal or vertical battery mounting. Slot size: not found. Recommended battery: 900-1500 mAh, 4S/6S. | spec | [S1] |
| Accessory mounts | adjustable GPS mount 12 x 16 mm; multi-function antenna mount (SMA position); TYPE-C module mount; optional anti-vibration camera base (sold separately) | spec | [S2] p.5, p.12 |
| Frame hardware (from the manual) | M2.5 x 20 mm aluminium standoffs with M2.5 x 8 button screws; M3 x 15 mm aluminium standoffs; M3 screws 7/8/9/10/14/20 mm; M2 x 3/6/7 mm (O3 / RunCam Link and antenna) | spec | [S2] p.2, 5, 8-12 |
| ESC motor-lead length | 52 mm for a 25.5 AIO; 62 mm for a 20 mm mini stack | spec | [S2] p.4 |

Sources:
- [S1] https://www.speedybee.com/speedybee-bee35-3-5-inch-frame/
- [S2] https://www.speedybee.com/product_images/img_SpeedyBee_Bee35_Frame/SpeedyBee-Bee35-Manual-EN.pdf (2023-10 manual, 12 pp.)
- [S3] https://www.racedayquads.com/products/speedybee-bee35-cinewhoop-3-5-micro-frame-kit-choose-version

---

### 2.2 EMAX ECO II 2004 motor

| Item | Value | Tag | Source |
|---|---|---|---|
| KV variants | 1600, 2000, 2400, 3000 KV | spec | [E1] |
| Overall size | 24.7 x 19.2 mm (listings do not say which figure is which; very likely diameter x height) | listing / unverified orientation | [E2], [E3], [E4] |
| Stator | 20 x 4 mm, 12N14P | listing | [E2] |
| Base mounting pattern | 12 x 12 mm | spec | [E1] (also matches Bee35 manual [S2]) |
| Base screw size | M2 | listing | [E3], [E4]; consistent with [S2] |
| Base screw depth / max screw length | not found | – | – |
| Prop mount | T-mount, 1.5 mm locating shaft, prop held by 2 screws | listing | [E3], [E4], [E5] |
| Prop screw size and spacing | not found (no bell drawing found) | – | – |
| Bearing/inner shaft | 3 mm steel. The 1.5 mm figure is the T-mount prop shaft. | listing | [E3], [E4] |
| Mass | 16.4 g without cable [E3]/[E4]; 16.6 g [E5]; 18 g with wires [E2] | listing (sources differ by wire inclusion) | [E2]-[E5] |
| Wires | 120 mm, 24 AWG | listing | [E2] |
| Input | 4-6S | listing | [E2], [E5] |

Sources:
- [E1] https://shop.emax-usa.com/products/eco-ii-2004-brushless-motor-choose-kv
- [E2] https://www.racedayquads.com/products/emax-eco-ii-2004-2400kv-motor
- [E3] https://www.rotorama.com/product/emax-eco-ii-2004-3000kv
- [E4] https://www.rotorama.com/product/emax-emax-eco-ii-2004-2400kv
- [E5] https://pyrodrone.com/collections/emax-eco-ii-series/products/emax-eco-ii-2004-brushless-motor-2400kv

---

### 2.3 Gemfan 3.5" 3-blade prop: "3520" and "D90" are two different props

| Item | Hurricane 3520 (3.5 x 2.0, open prop) | D90 / D90S (90 mm ducted, 3.0" pitch) | Tag | Source |
|---|---|---|---|---|
| Diameter | 89.5 mm | 90 mm / 3.5" | listing | [G1]; [G2], [G3] |
| Pitch | 2.0" | 3.0" | listing | [G1]; [G2], [G3] |
| Hub type | T-mount, 1.5 mm | D90S: T-mount 1.5 mm. D90 hybrid: native 5 mm (M5) bore plus T-mount adapters for 1.5 / 2 mm. | listing | [G1]; [G2]; [G3] |
| Hub height (centre thickness) | 5.5 mm | 5.5 mm | listing | [G1]; [G2], [G3] |
| Hub outer diameter | not found | not found | – | – |
| T-mount screw count / spacing | not found on opened pages | not found | – | – |
| Mass | 1.8 g | 2.3 g | listing | [G1]; [G2], [G3] |
| Blade max chord | 11.7 mm | not found | listing | [G1] |
| Material | PC | PC | listing | [G1]; [G2] |

Note: the Gemfan official pages (gemfanhobby.com) returned 403 / bot-check and could not be opened. No spec-tagged numbers.

Sources:
- [G1] https://www.racedayquads.com/products/gemfan-hurricane-3520-durable-3-blade-propeller-set-of-4-1-5mm
- [G2] https://pyrodrone.com/collections/ducted-props/products/gemfan-d90-ducted-durable-3-blade-2cw-2ccw (D90S T-mount)
- [G3] https://www.myfpvstore.com/6rq1 (D90 with M5 bore + T-mount adapters)

---

### 2.4 S500 quad frame: Holybro S500 V2 (CAD "S500-C1_ASM") primarily, plus a generic S500 for comparison

**Variant used:** Holybro S500 V2 kit (product page and assembly manual). Hole geometry comes from Holybro's own "S500 Frame Kit" STEP file (`S500-C1_ASM.step`, exported 2025-01-15) on docs.holybro.com. I parsed the hole circles and part placements from the STEP myself, so those values are **derived**. Plate X/Y coordinates are relative to the plate centre.

| Item | Value | Tag | Source |
|---|---|---|---|
| Wheelbase | 480 mm (not 500) | spec | [H1], [H2] p.1 |
| Overall L x W x H | 383 x 385 x 240 mm | spec | [H2] p.1 |
| Mass | ~915 g complete kit incl. electronics, no battery | spec | [H1] |
| Plate thickness | product page: "1.5 mm top and bottom plates". CAD: top 1.5 mm, bottom 2.0 mm. | spec vs derived: **conflict** | [H1] vs [H3] |
| Top plate outline | 217.9 mm across (arm-to-arm tips) | derived | [H3] |
| Bottom plate outline | 196 x 118 mm | derived | [H3] |
| Plate spacing | top plate underside 18.04 mm above the bottom plate underside, so **~16.0 mm clear gap** above the 2.0 mm bottom plate | derived | [H3] |
| Top plate: central M3 patterns | 4 x Ø3.1 on a **49.0 x 49.0 mm** square; 8 x Ø3.0 on the two centre axes at ±14.5 and ±35.5 mm (**29 mm and 71 mm** spacing on both axes) | derived | [H3] |
| Bottom plate: hanger / rail mount grid | Ø3.1 holes at X = ±24 and ±65 mm, Y = ±22.5 and ±36.5 mm (pairs 14 mm apart), with a Ø5.0 hole at the centre of each pair (Y = ±29.5). The CAD places the four rail hangers at X = ±65, Y = ±29.5; X = ±24 is an alternate position. | derived | [H3] |
| Bottom plate: other patterns | 4 x Ø2.8 at **24.5 x 18.0 mm** (plus 2 at ±17.25 on the long axis, 34.5 mm); 4 x Ø3.5 at **36 x 34 mm** (offset to one end, X = -60/-96); Ø3.0 at X = -15/+12 (27 mm), Y = ±31 / ±56 | derived | [H3] |
| FC mounting | the Pixhawk sits on its own plate fixed with M2.5 x 6 screws. FC plate hole pattern: not found | spec (screw) | [H2] step 9 |
| PDB | "built into the bottom plate" on V2 [H1]. The CAD also contains a separate 68 x 50 x 1.6 mm power PCB with 4 x Ø3.0 at 45 x 45 mm, on 6 mm nylon standoffs. | spec / derived | [H1], [H3] |
| Motor mount on arm | listings only: generic S500 16 x 19 / 19 x 25 mm [H4]; S500/S550 spare arm 16 / 25 mm [H5]. The Holybro CAD shows a universal motor pad: 4 x Ø3.2 holes on a 17.9 mm circle, 8 x 4 mm-wide slots whose ends lie on 17 mm and 30 mm diameters, 4 x Ø5.6 on a 31.3 mm circle, and a Ø10.5-13 mm centre bore. It should take 16 x 19 mm M3 2216 motors (check on the part). | listing / derived, unverified | [H4], [H5], [H3] |
| Motor screws | M3 x 7 (16 pcs) in the manual text; hardware list says M3 x 8 | spec (internal inconsistency) | [H2] p.3, step 5 |
| Arm-to-plate screws | M2.5 x 6 (2 per arm per plate); pre-threaded sleeves, no nuts | spec | [H1], [H2] step 3 |
| Landing gear | 16 mm and 10 mm carbon tubes with plastic tee connectors [H1]. CAD: vertical poles Ø16 OD / Ø14 ID x 150 mm; 2 x Ø10 OD / Ø8 ID x 248 mm rails under the bottom plate, **59 mm apart** centre-to-centre, axes 15 mm below the bottom-plate datum. The manual fixes the gear to the board through 4 holes with M3 x 8. | spec / derived | [H1], [H2] step 2, [H3] |
| Battery mount board | 100 x 40.3 x 2.0 mm; 4 x Ø2.6 at 25 x 70 mm; hung on pylons ~46.5 mm below the bottom-plate datum | derived | [H3] |
| Motors in kit | **conflict:** 2216 KV880 (manual [H2], PX4 guide) vs Holybro 2216 KV920 (current product page [H1]) | spec vs spec | [H1], [H2] |
| Generic "S500" (non-Holybro, for comparison) | 500 mm diagonal, PCB top/bottom plates, 19 mm stack height, motor holes 16 x 19 / 19 x 25 mm, no 30.5 x 30.5 FC holes | listing | [H4] |

Sources:
- [H1] https://holybro.com/products/s500-v2-development-kit
- [H2] https://cdn.shopify.com/s/files/1/0604/5905/7341/files/S500_V2_Kit_AssemblyManual.pdf?v=1646987800 (Pixhawk 4 S500 V2 kit assembly manual)
- [H3] https://docs.holybro.com/drone-development-kit/s500-frame-kit, which links the STEP file `S500-C1_ASM.step` (gitbook-hosted download)
- [H4] https://www.rotorama.com/product/s500-ram
- [H5] https://www.unmannedtechshop.co.uk/products/s500-s550-arm

---

### 2.5 SunnySky X2216 880 KV (the "II" and the current V3 share the same drawing)

| Item | Value | Tag | Source |
|---|---|---|---|
| Base mounting pattern | **4 x M3 x 0.5, depth 2.8 mm**, as two hole pairs at **16 mm and 19 mm** centre-to-centre (perpendicular pairs), at 45 deg to the lead exit | spec (drawing) | [U1] drawing, [U2] drawing |
| Shaft | Ø3.175 mm (0/-0.02). Short shaft: 36.5 mm overall body+shaft (multirotor version). Long shaft: 50-52 mm overall. | spec | [U1], [U2], [U3] |
| Bell / body | rotor Ø27.7 mm (27.5 mm on [U3]); 32 mm can length; 34 mm body length without shaft; stator 22 x 16 mm, 12N14P | spec | [U1]-[U3] |
| Prop adapter | bolt-on, **M5 x 0.8** thread (CW/CCW versions), Ø5 mm, 22.5 mm long (13 mm threaded), Ø17 mm flange, fixed by 3 x M2 x 0.4 (depth 2.8) on a Ø12 mm circle in the bell | spec (drawing) | [U1], [U2] |
| Included X-mount | 34 x 34 mm cross, 3 mm thick, Ø10.2 centre hole; inner 4 x Ø3.2 (Ø5.7 c'bore) at 16 / 19 mm; outer 4 x Ø3.1 (spacing not dimensioned); 41 mm overall incl. section | spec (drawing) | [U1], [U2] |
| Mass | 68 g motor + 13 g accessories (X2216 KV880 II [U4]); 72 g (original X2216 880 [U3]); 67.5 g (V3 880 [U5]) | spec / listing; differs by version | [U3], [U4], [U5] |
| Lead length | 100 mm | spec (drawing) | [U1] |

Sources:
- [U1] https://cdn.shopify.com/s/files/1/2024/0305/files/X2216_ef645552-9c5f-4bf9-a167-6a401ccc1f41.jpg (dimension drawing embedded in [U3])
- [U2] https://cdn.shopify.com/s/files/1/2024/0305/files/X2216.jpg (drawing embedded in the SunnySky USA X2216 V3 long-shaft page https://sunnyskyusa.com/products/sunnysky-x2216-v3-brushless-motors-long-shaft-version)
- [U3] https://sunnyskyusa.com/products/sunnysky-x2216-brushless-motors (official SunnySky USA store)
- [U4] https://www.buddyrc.com/products/sunnysky-x2216-kv880-ii-brushless-motor-long-shaft-on-sale
- [U5] https://www.buddyrc.com/products/sunnysky-x2216 (V3 table)

---

### 2.6 1045 propeller (10 x 4.5), generic

| Item | Value | Tag | Source |
|---|---|---|---|
| Diameter / pitch | 254 mm / 4.5" | listing | [P1], [P2] |
| Hub thickness | 9.5-10 mm [P1]; 9.7 mm [P2] | listing | [P1], [P2] |
| Hub outer diameter | not found | – | – |
| Bore | 6.0 mm | listing | [P1], [P2] |
| Adapter rings | 3, 3.17, 4, 5, 6 mm [P1]; 3, 4, 5, 6, 7.8 mm [P2] | listing (sets vary) | [P1], [P2] |
| Mass | ~12 g [P1]; ~7 g [P2] (large spread, weigh your own) | unverified | [P1], [P2] |
| Fit to X2216 | the SunnySky adapter is M5 x 0.8 with a Ø5 pilot (section 5), so use the 5 mm ring | spec (adapter) | [U1] |

Sources:
- [P1] https://probots.co.in/black-1045-blade-propeller-10-inch-10x4-5-pair.html
- [P2] https://thinkrobotics.com/collections/drone-accessories-online/products/10x4-5-1045-1045r-propeller-1-cw-1-ccw-online

---

### 2.7 Unitree 4D LiDAR L2

| Item | Value | Tag | Source |
|---|---|---|---|
| Body | 75 (W) x 75 (D) x 65 (H) mm; base flange Ø75 | spec | [L1] p.8, p.15 |
| Heights | lower housing step at 24 mm; a reference line in the window at 44.5 mm (the drawing does not label it; probably the optical/scan centre); window recess 32 mm wide | spec / meaning of 44.5 unverified | [L1] p.8 drawing |
| Mass | 230 g | spec | [L1] |
| Mounting holes | **4 x M3, 6 mm deep**, in the bottom, on a **Ø51 mm** circle at 90 deg spacing, rotated 22.5 deg from the cable-outlet axis (equivalent square ≈ **36.06 x 36.06 mm**, derived) | spec (square derived) | [L1] p.8 drawing |
| Positioning slots | 4 slots in the base rim, 3.5 mm wide, each on the same radial line as an M3 hole; meant for locating features on a bracket | spec | [L1] p.4, p.8 |
| Other circle | Ø60 mm circle through 4 housing screws (not for mounting) | spec (drawing) | [L1] p.8 |
| Connector / outlet | a single cable leaves through a notch at the bottom edge of the base, on the -X side ("+X is the opposite direction of the outlet"). The cable splits into DC3.5-1.35 power, RJ45 Ethernet and a GH1.25-4Y UART plug. | spec | [L1] p.4-5, p.10 |
| FOV | 360 deg x 90 deg hemisphere above the base. Negative-angle mode gives 360 x 96 deg (6 deg below horizontal). Tiny blind cone straight up. Near blind zone 0.05 m. | spec | [L1] p.6-7, p.15 |
| Keep-out rules | do not block the FOV, not even with clear glass; leave space on all four sides for cooling; the body must not carry extra loads; bottom cover must stay at or below 85 °C; can be mounted in any orientation via the bottom holes; needs a water-protection cover if exposed. The measurement core rotates 360 deg internally (clear the head). | spec | [L1] p.6-7, p.16 |
| Electrical | 12 V DC, 10 W typical, 13 W peak while self-heating; IP54; -10 to 50 °C | spec | [L1] p.15-16 |

Source:
- [L1] https://oss-global-cdn.unitree.com/static/Unitree%204D%20LiDAR%20L2%20User%20Manual.pdf (English user manual, PDF dated 2024-12)

---

### 2.8 4S LiPo packs (TYPICAL only; varies by brand)

| Pack | L x W x H (mm) | Mass (g) | Tag | Source |
|---|---|---|---|---|
| 4S 1500 (CNHL Black Series V2, 130C) | 76 x 36 x 33 | ~174 | listing, typical | [B1] |
| 4S 1550 (Tattu Funfly, 100C) | 71 x 35 x 35 | 180 | listing, typical | [B2] |
| **Our note 1500** | 70 x 32 x 33 | 165 | (narrower and lighter than both listings) | – |
| 4S 6000 (Gens Ace, 35C) | 144 x 45 x 40 | 593.5 | listing, typical | [B3] |
| 4S 6000 (Gens Ace Redline HV 15.2 V hard case, 130C; car pack) | 139 x 47 x 37 | 447 | listing, atypical (hard case) | [B4] |
| **Our note 6000** | 148 x 48 x 45 | (none) | (a few mm larger than the listings; fine as an envelope) | – |

Sources:
- [B1] https://www.racedayquads.com/products/cnhl-black-series-14-8v-4s-1500mah-100c-lipo-battery-xt60
- [B2] https://www.rotorama.com/product/tattu-funfly-1550-4s-100c
- [B3] https://www.onedrone.com/store/gensace-6000mah-14-8v-35c-4s.html?tag=14.8v
- [B4] https://www.buddyrc.com/products/gens-ace-redline-6000mah-15-2v-130c-4s1p-hard-case-hv-lipo-battery

---

### 2.x Pages that could not be opened (so their numbers are not used)
- gemfanhobby.com product pages (403 / bot check)
- getfpv.com pages (403), including Tattu 6000 4S (search snippet said 147 x 45 x 43.3 mm, 590 g; not verified)
- hobbyking.com pages (403)
- EMAX's own product page has no dimensions or weight; no EMAX bell/T-mount drawing was found

## 3. Box mechatronics

Researched 2026-10-22. All values in mm unless noted. Every value below comes from a page or PDF that was actually opened (PDF drawings were rendered and read).

**Tags**
- **spec**: from the manufacturer's datasheet or drawing.
- **listing**: from a store page or a reseller-hosted file.
- **typical**: from one representative maker of a generic part that many companies make.
- **unverified**: sources disagree, or the value was scaled or derived by me.
- **not found**: no opened source gave the value.

---

### 3.1 NEMA 17 stepper (reference part: StepperOnline 17HS19-2004S1, 42x48)

| Item | Value | Tag | Source |
|---|---|---|---|
| Body square | 42.3 max | spec | [S1] |
| Face hole pattern (c-c) | 31 ±0.2, square, 4 holes | spec | [S1] |
| Hole thread / depth | 4x M3, 4.5 min deep | spec | [S1] |
| Pilot boss diameter | Ø22 (0 / -0.05) | spec | [S1] |
| Pilot boss height | 2 | spec | [S1] |
| Shaft diameter | Ø5 (0 / -0.012) | spec | [S1] |
| Shaft length from face | 24 ±1 | spec | [S1] |
| D-flat | 15 ±0.25 long, flat at 4.5 ±0.1 (0.5 deep) | spec | [S1] |
| Body length (this model) | 48 max | spec | [S1] |
| Lead | 1000 ±15, 4-pin 2.54 Harwin-type connector | spec | [S1] |
| Shaft load note | "Length 20 mm: 21 N max" (radial) | spec | [S1] |
| Mass, 17HS19 | "--" (not given on drawing) | not found | [S1] |
| Common body lengths / mass | 42x38: 285 g; 42x48: 350 g; pancakes 42x11.6 (70 g), 42x18.6 (140 g) | listing | [S2] |

- [S1] https://static.maritex.eu/file/display/5sXxpHH1SP-JwnhJEZ0lhfX-xdYKZRi3/17HS19-2004S1_Full_Datasheet.pdf (StepperOnline drawing A0842, hosted by a distributor)
- [S2] https://www.pololu.com/category/87/stepper-motors

*Mount note:* the 31 mm pattern and the Ø22 boss are NEMA-standard, but M3 depth varies by maker; this drawing gives 4.5 min.

---

### 3.2 BIGTREETECH SKR Mini E3 V3.0

| Item | Value | Tag | Source |
|---|---|---|---|
| Board outline | 103.75 x 70.25 | spec | [B1], [B2] |
| Hole H1 | 23.39 from left edge, 3.22 from top edge | spec | [B1] |
| Hole H2 | 18.20 from right edge (= 85.55 from left), 3.02 from top edge | spec | [B1] |
| Hole H3 | 5.53 from left edge, 32.05 from bottom edge | spec | [B1] |
| Hole H4 | 37.34 from left edge, 29.41 from bottom edge | spec | [B1] |
| Hole H5 | 2.53 from right edge (= 101.22 from left), 2.56 from bottom edge | spec | [B1] |
| Other features | micro-SD protrudes about 2.79 past the right edge; micro-USB is on the right edge (16.49 / 21.88 / 30.26 dimension lines) | spec | [B1] |
| Hole diameter / screw | not stated in BTT drawing or manual (Ender-3 mounting, M3 screws in practice) | not found | [B1], [B2] |

- [B1] https://raw.githubusercontent.com/bigtreetech/BIGTREETECH-SKR-mini-E3/master/hardware/BTT%20SKR%20MINI%20E3%20V3.0/Hardware/BTT%20E3%20SKR%20MINI%20V3.0_SIZE_20220301.pdf
- [B2] https://raw.githubusercontent.com/bigtreetech/BIGTREETECH-SKR-mini-E3/master/hardware/BTT%20SKR%20MINI%20E3%20V3.0/Hardware/BTT%20SKR%20MINI%20E3%20V3.0%20user%20manual.pdf
- A retailer page in search results gave 100.75 x 70.25. This conflicts with the BTT drawing. I did not open that page, so I give no value from it.
- A STEP model is in the same repo: `hardware/BTT SKR MINI E3 V3.0/3D/BTT E3 SKR MINI V3.0.step`

*Mount note:* the holes are not on a rectangle. They follow the Ender-3 board pattern of 5 holes. Use the edge offsets above.

---

### 3.3 Mean Well LRS-350-24 (case No. 207A)

| Item | Value | Tag | Source |
|---|---|---|---|
| Case L x W x H | 215 x 115 x 30 (tolerance ±1) | spec | [M1] |
| Packed weight | 0.76 kg | spec | [M1] |
| Bottom-face holes | 4x M4, max screw penetration L = 3 | spec | [M1] |
| Bottom pattern | 150 (along length) x 50 (across width) | spec | [M1] |
| Bottom hole position | first row 32.5 from terminal-block end; rows 32.5 from each long side (32.5 + 50 + 32.5 = 115) | spec | [M1] |
| Side-face holes | 4x M4, both long sides, max penetration L = 5 | spec | [M1] |
| Side pattern | 150 apart along length, 32.5 from terminal end, 12.5 above bottom face | spec | [M1] |
| Terminal block | 9 screw terminals at 9.5 pitch, on the 115 end face (1 AC/L, 2 AC/N, 3 FG, 4–6 −V, 7–9 +V) | spec | [M1] |
| Fan | on top face, 47.45 from the far end, 36.7 from one side; airflow exits the end opposite the terminals | spec | [M1] |
| Optional brackets (dashed on drawing) | 135 across, tabs 15 wide, 16 overhang | spec | [M1] |

- [M1] https://www.meanwell.com/Upload/PDF/LRS-350/LRS-350-SPEC.PDF (rev 2025-09-12, page 4)

*Mount note:* keep the screw engagement at 3 or less on the bottom and 5 or less on the sides, or the screws can hit the PCB.

---

### 3.4a TowerPro MG995

| Item | Value | Tag | Source |
|---|---|---|---|
| Body (spec text) | 40.7 x 19.7 x 42.9 | spec | [T1] |
| Drawing table, A (height incl. output) | 42.7 | spec | [T1] |
| B (body length) | 40.9 | spec | [T1] |
| C (bottom to case top) | 37 | spec | [T1] |
| D (width) | 20 | spec | [T1] |
| E (length over ears) | 54 | spec | [T1] |
| F (bottom to underside of ears) | 26.8 | spec | [T1] |
| Mass | 55 g | spec | [T1] |
| Alt. body / ear figures | 40 between ears(body), 54 over ears, 38 case height, 28 to ears, 47.2 over horn | listing (unverified vs T1) | [T2] |
| Ear hole spacing / hole Ø | not found on any opened MG995 page | not found | — |
| Output spline | 25T (aftermarket horn listed for MG995/MG996R) | listing | [T3] |
| Shaft offset from body | not found | not found | — |

- [T1] https://www.towerpro.com.tw/product/mg995-robot-servo-180-rotation/ plus its drawing http://torqpro.com/wp-content/uploads/2014/07/小馬達尺寸標示圖B.jpg. The page's review box names the "MG996R", so the same table may be shared between the two models.
- [T2] https://components101.com/motors/mg995-servo-motor (drawing MG995-Dimensions.png)
- [T3] https://probots.co.in/25t-circular-metal-servo-horn-mg996r-mg995.html
- Other opened sources repeat 40.7 x 19.7 x 42.9 and 55 g: https://servodatabase.com/servo/towerpro/mg995r and https://iarduino.ru/lib/cd4c7a894318c9658717c2409e0e3ddd.pdf

### 3.4b DSServo DS3218(MG) 20 kg

| Item | Value | Tag | Source |
|---|---|---|---|
| Body | 40 x 20, case height 40.4 (bottom to case top, excl. spline) | spec | [D1] |
| Listed size / mass | 40 x 20 x 40.5, 60 g | listing | [D2], [D3] |
| Length over ears | 54.5 | spec | [D1] |
| Ear hole pattern | 49.5 (along length) x 10 (across), 4 holes | spec | [D1] |
| Ear hole Ø | 4.5. These are r = 2.25 cylinders at (±24.75, ±5) in the maker's STEP file. | spec (from STEP) | [D4] |
| Bottom to underside of ears | 27.7 | spec | [D1] |
| Output spline | 25T. The listing names only the included horn ("25T metal arm"); the sibling DS3235 datasheet gives "Spline Specification – 25T". | listing / spec (sibling) | [D3], [D5] |
| Shaft offset | output axis about 10 from the output end of the 40 body, so about 10 off the body centre. Scaled from drawing [D1]. | unverified | [D1] |
| "24" on drawing | output axis to horn hole, not a body dimension | spec | [D1] |

- [D1] https://cdn.shopify.com/s/files/1/0673/6848/5000/files/DS3218MG-Drawing.pdf?v=1768562471 (linked from the DSServo store page in [D2])
- [D2] https://aifitlab.com/products/dsservo-ds3218-mg-servo-motor
- [D3] https://thinkrobotics.com/products/ds3218-servo
- [D4] https://cdn.shopify.com/s/files/1/0673/6848/5000/files/DS3218MG.stp?v=1768562476
- [D5] https://hajim.rochester.edu/me/sites/kelley/me240/DS3235-270_datasheet.pdf (DS3235: same 49.5 x 10 / 54.5 / 27.7 / 40.4 drawing)

*Mount note:* the DS3218 drawing (49.5 x 10, Ø4.5) is the only opened source for a standard-servo ear pattern. Do not reuse it for the MG995 without measuring, because the MG995's 54 over-ears and 26.8 ear height differ from the DS3218's 54.5 and 27.7.

---

### 3.5 6010-2RS deep-groove ball bearing

| Item | Value | Tag | Source |
|---|---|---|---|
| Bore d | 50 (−0.01/0) | spec | [K1] |
| OD D | 80 (−0.011/0) | spec | [K1] |
| Width B | 16 (−0.06/0) | spec | [K1] |
| Chamfer r1,2 min | 1 | spec | [K1] |
| Shaft shoulder da | 54.6 min – 59.7 max | spec | [K1] |
| Housing shoulder Da max | 75.4 | spec | [K1] |
| Fillet ra max | 1 | spec | [K1] |
| d1 / D2 | ≈59.75 / ≈72.8 | spec | [K1] |
| Loads | C 22.9 kN, C0 15.6 kN, limiting speed 5000 r/min | spec | [K1] |
| Mass | 0.261 kg (NSK 6010ZZ shielded; same boundary dims) | spec (other maker / variant) | [K2] |

- [K1] https://www.skf.com/group/products/rolling-bearings/ball-bearings/deep-groove-ball-bearings/productid-6010-2RS1 (Technical specification tab)
- [K2] https://docs.rs-online.com/357c/A700000014623553.pdf (NSK 6010ZZ datasheet). NSK gives a different shaft abutment (da 55–58.5, Da 75).

---

### 3.6 20-series 6 mm-slot aluminium extrusion (2020 and 2040)

**T-slot, MISUMI HFS5 ("5 series, slot 6")**

| Item | 2020 | 2040 | Tag | Source |
|---|---|---|---|---|
| Outer profile | 20 x 20, corners 4-R1 | 20 x 40, corners 4-R1 | spec | [E1] |
| Slot opening | 6 | 6 | spec | [E1] |
| Slot inner width | 12 | 12 | spec | [E1] |
| Lip thickness | 2 | 2 | spec | [E1] |
| Cavity below lip | 4, so slot depth from face is about 6 | same | spec | [E1] |
| Core width (between slots) | 8 | 8 | spec | [E1] |
| Centre hole | Ø4.2 (for M5 tap) | 2x Ø4.2, at 10 from ends, 20 apart | spec | [E1] |
| Mass | 0.5 kg/m | 0.88 kg/m | spec | [E1] |
| Section area | 183 mm² | 324 mm² | spec | [E1] |
| I (x / y) | 0.742 / 0.742 x10⁴ mm⁴ | 1.358 / 5.13 x10⁴ mm⁴ | spec | [E1] |

**V-slot type (Ooznest "20 Series T/V-Slot")**

| Item | 2020 | 2040 | Tag | Source |
|---|---|---|---|---|
| Slot opening | 6.13 | 6.13 | listing | [E2], [E3] |
| Slot depth from face | 6.1 | 6.1 | listing | [E2], [E3] |
| Slot inner width | — | 11 | listing | [E3] drawing |
| Slot bottom to lip underside | — | 4.3, so the lip incl. V is about 1.8 at the opening (my reading of the drawing) | unverified | [E3] drawing |
| Centre / end hole | Ø4.2 | Ø4.2, at 10 from ends, 20 apart | listing | [E2], [E3] |
| End tap | M5x0.8, 10 deep (optional service) | same | listing | [E2], [E3] |
| Mass | "0.46" (unit given as "g"; probably kg/m) | "0.81" (same unit problem) | unverified | [E2], [E3] |
| I (x / y) | 0.703 / 0.703 x10⁻⁸ m⁴ | 4.818 / 1.239 x10⁻⁸ m⁴ | listing | [E2], [E3] |

- [E1] https://stg0-my.misumi-ec.com/pdf/fa/2014/p2_529.pdf (MISUMI FA catalog p.2-529/530)
- [E2] https://ooznest.co.uk/product/v-slot-linear-rail-20x20mm-cut-to-size/
- [E3] https://ooznest.co.uk/product/v-slot-linear-rail-20x40mm-cut-to-size/ and its drawing https://ooznest.co.uk/wp-content/uploads/2025/12/20x40mm-20-Series-T-V-Slot-Aluminium-Extrusion-Profile-Cut-To-Size_drawing.png
- RatRig lists V-slot 2040 with slot width 6 and slot depth 6 (https://ratrig.com/vslot-2040.html) (listing).
- The official OpenBuilds drawing could not be reached (openbuildspartstore.com redirects to openbuilds.com).

---

### 3.7 GT2 pulleys / idlers (6 mm belt)

Pitch diameter from first principles: PD = z x 2 / π. For 80T that is 50.93; for 20T it is 12.73. These are my calculations, not from a source.

**80T, 8 mm bore (Phidgets TRM4106_0, made by Chengdu Leno)**

| Item | Value | Tag | Source |
|---|---|---|---|
| Tooth OD (De) | Ø50.42 (+0.05/0) | spec | [G1] |
| Tooth width | 11 (+0.2/0) | spec | [G1] |
| Hub | Ø36 x 8 (19 − 11) | spec | [G1] |
| Overall length | 19 | spec | [G1] |
| Bore | Ø8 H7 | spec | [G1] |
| Set screws | 2x M4 at 90° (2 mm hex key per listing) | spec | [G1], [G2] |
| Flanges | **none** on this part | spec | [G1] |
| Material | LY12 (2024-T6) | spec | [G1] |

- [G1] https://www.phidgets.com/productfiles/TRM4106/TRM4106_0/Documentation/TRM4106_0Mechanical.pdf
- [G2] https://www.phidgets.com/?prodid=755
- A flanged generic 80T (a Manomano listing claiming 55 OD, Ø30 x 9 hub, 20 tall, M5 screw) appeared in search, but the page blocked access, so it is not verified.

**20T, 5 mm bore**

| Item | LDO-2GT-20T-ID5-H16 | Adafruit #1251 | Tag | Source |
|---|---|---|---|---|
| Tooth OD | Ø12.22 (+0.05/0) | 12.2 | spec / listing | [G3], [G4] |
| Flange Ø | ≈15 (flange is drawn the same width as the hub) | 16 | spec / listing | [G3], [G4] |
| Belt (tooth) width | 7.0 between flanges | — | spec | [G3] |
| Top flange thickness | 1.2 | — | spec | [G3] |
| Hub | Ø15.0 x 7.8 | — | spec | [G3] |
| Overall length | 16.0 | 16 | spec / listing | [G3], [G4] |
| Bore | Ø5 H7 | 5 | spec / listing | [G3], [G4] |
| Set screws | 2x M4, axis 3.9 from hub end | 2 (size not stated) | spec / listing | [G3], [G4] |
| Mass | — | 6 g | listing | [G4] |

- [G3] https://ooznest.co.uk/wp-content/uploads/2018/10/2GT-Pulley-20-Tooth-5mm-Bore-6mm-Belt-Drawing.pdf (LDO drawing)
- [G4] https://www.adafruit.com/product/1251
- Generic 20T pulleys often use M3 grub screws. I could not open a listing that confirms this (robu.in and makerstore returned 403 or a challenge page), so the screw size is **unverified** for non-LDO parts.

**Toothless (smooth) idlers with bearing, 6 mm belt (Tinytronics)**

| Item | GT2316S (16T-size, bore 3) | GT2320S (20T-size, bore 3) | GT2520S (20T-size, bore 5) | Tag | Source |
|---|---|---|---|---|---|
| Flange OD | 13 | 18 | 18 | listing | [G5], [G6] |
| Belt-running Ø | 9.5 (drawing) | not stated | not stated | listing | [G5] |
| Overall width | 8.5 | 8.5 | 8.5 | listing | [G5], [G6] |
| Belt channel width | 6.5 (drawing) | not stated | not stated | listing | [G5] |
| Bore | 3 | 3 | 5 | listing | [G5], [G6] |

- [G5] https://www.tinytronics.nl/en/mechanics-and-actuators/parts/pulleys/gt2-16-pulley-toothless-3mm-axis-with-ball-bearing (drawing image `dimensions gt216.png`)
- [G6] https://www.tinytronics.nl/en/mechanics-and-actuators/parts/pulleys/gt2-20-pulley-toothless-5mm-axis-with-ball-bearing and https://www.tinytronics.nl/en/mechanics-and-actuators/parts/pulleys
- **A 16T-size toothless idler with a 5 mm bore was not found** among opened listings. If "Ø16" means a flange of 16, no opened listing has exactly that; the closest have 13 or 18 flanges.

---

### 3.8 KW11-3Z microswitch (Omron SS-5GL equivalent)

| Item | Value | Tag | Source |
|---|---|---|---|
| Body L x H x W | 19.8 x 10.2 x 6.4 | spec (Omron SS-5GL) | [W1] |
| Mounting holes | 2x Ø2.35 (+0.075/−0.05), c-c 9.5 ±0.1 | spec | [W1] |
| Hole position | first hole 5.1 from the end nearest the COM terminal; holes 2.9 above terminal-side edge | spec | [W1] |
| Recommended panel holes | 2x Ø2.4 or M2.3 tapped, 9.5 ±0.1 | spec | [W1] |
| Hinge lever (SS-5GL) | lever about 14.5 long, free position 13.6 max, operating position 8.8 ±0.8 (above the hole line) | spec | [W1] |
| Terminals | 3x, Ø1.6 holes, at 1.6 / 8.8 / 7.3 spacing, protruding 6.4 | spec | [W1] |
| Mass | approx. 1.6 g (pin plunger model) | spec | [W1] |
| KW11-3Z (generic) size | 20 x 10 x 6, 2 g | listing | [W2] |
| KW11-3Z hole Ø / spacing | no generic maker drawing found; the Omron SS footprint above is the usual clone footprint | unverified | — |

- [W1] https://omronfs.omron.com/en_US/ecb/products/pdf/en-ss.pdf (pages 4–5)
- [W2] https://domoticx.net/webshop/microswitch-3-polig-5a-250vac-met-lip-14mm-10t85-kw11-3z-2

---

### 3.9 A3144 hall-effect module (red/blue breakout)

| Item | Value | Tag | Source |
|---|---|---|---|
| Board size | 29.2 x 11.2 (3-pin version) | listing | [H1] |
| Mounting hole | 1x, Ø2 | listing | [H1] |
| Mass | 5 g | listing | [H1] |
| Board size (other seller) | page gives both "32x14" and "27.0 x 15.5" | unverified | [H2] |
| Hole (other seller) | page gives both "3 mm screw hole" and "2.0 mm holes" | unverified | [H2] |

- [H1] https://techiesms.com/product/a3144-hall-effect-sensor-module/
- [H2] https://othoba.com/small-pcb-hall-effect-sensor-module-a3144e-49e-3-pin-hall-sensor-hall-effect-detection-magnetic-switches-speed-counting-sensor-module-for-arduino-3-pin-leads-udvabony-bd-475960
- Boards differ by vendor. Measure the actual board, or design a slot plus an M2/M3 clearance hole.

---

### 3.10 Heschen HS-P20/15 24 V electromagnet

**No Heschen datasheet or listing for HS-P20/15 was found.** The values below are for generic 24 V P20/15 units.

| Item | Value | Tag | Source |
|---|---|---|---|
| Body | Ø20 x 15 | typical | [EM1], [EM2] |
| Mounting thread | M3 (rear centre) | typical | [EM1], [EM2] |
| Thread depth | 12 (Tinytronics JF-XP2015) / not stated (First4Magnets) | unverified | [EM1], [EM2] |
| Holding force | 2.5 kg (≈25 N), at zero air gap on flat steel | typical | [EM1], [EM2] |
| Power | 3 W (First4Magnets); about 320 Ω coil, so about 75 mA / 1.8 W (Tinytronics, my calculation) | unverified | [EM1], [EM2] |
| Pole / core Ø | 8 centre pole (5 V Adafruit P20/15) | typical | [EM3] |
| Mass | 22.7 g (5 V Adafruit P20/15) | typical | [EM3] |
| Lead length | 270 (Adafruit 5 V) | typical | [EM3] |

- [EM1] https://www.tinytronics.nl/en/mechanics-and-actuators/electromagnets/electromagnet-2.5kg-24v-dc-jf-xp2015
- [EM2] https://www.first4magnets.com/product/20mm-dia-24v-energise-to-hold-electromagnet-25kg-pull-19831
- [EM3] https://www.adafruit.com/product/3872
- Botland sells a 24 V 20x15 unit with an M4 thread (seen in search results, not opened). Check the thread on the actual part.

---

### 3.11 Brass heat-set inserts (M3, M5)

| Insert | Insert OD (d1) | Centre tip (d2) | Hole Ø (d3) | Length L | Min wall W | Tag | Source |
|---|---|---|---|---|---|---|---|
| Ruthex M3 (RX-M3x5.7) | 4.6 | 3.9 | 4.0 | 5.7 | 1.6 | spec | [I1] |
| Ruthex M3 Short | 4.6 | 3.9 | 4.0 | 4.0 | 1.6 | spec | [I1] |
| Ruthex M3x5x4 Voron | 5.0 | 4.25 | 4.4 | 4.0 | 1.3 | spec | [I1] |
| Ruthex M5 | 7.1 | 6.3 | 6.4 | 9.5 | 2.6 | spec | [I1] |
| Ruthex M5 Short | 7.1 | 6.3 | 6.4 | 5.8 | 2.6 | spec | [I1] |
| CNC Kitchen M3 | 4.6 | 3.9 | 4.0 | 5.7 | 1.6 | listing | [I2] |
| CNC Kitchen M3 Short | 4.6 | 3.9 | 4.0 | **3.0** | 1.6 | listing | [I2] |
| CNC Kitchen M5 | 7.1 | 6.3 | 6.4 | 9.5 | 2.6 | listing | [I2] |
| CNC Kitchen M5 Short | 7.1 | 6.3 | 6.4 | 5.8 | 2.6 | listing | [I2] |

- Ruthex drawing: hole depth at least L + 1.
- Conflict: the "M3 Short" length is 4.0 for Ruthex and 3.0 for CNC Kitchen.
- [I1] https://www.ruthex.de/en/products/ruthex-gewindeeinsatz-m3-100-stuck-rx-m3x5-7-messing-gewindebuchsen (dimension table image `1Tabelle.PT05...jpg`)
- [I2] https://www.3djake.ch/en-CH/cnc-kitchen/threaded-inserts-m5-short (CNC Kitchen table reproduced by the reseller). https://cnckitchen.store/products/heat-set-insert-m3-x-5-7-100-pieces confirms only M3 L = 5.7.

---

### 3.12 ISDT PD60S charger

**No "PD60S" was found on ISDT's site.** ISDT's model comparison lists only the PD60. Values below are for the ISDT **PD60**.

| Item | Value | Tag | Source |
|---|---|---|---|
| Case | 56 x 85 x 21 | spec (PD60) | [C1] |
| Mass | about 87 g | spec (PD60) | [C1] |
| Input | USB-C PD/QC, 5–20 V, 3 A max, 60 W | spec (PD60) | [C1] |
| Mounting holes | none stated | not found | [C1] |

- [C1] https://www.isdt.co/pd60.html?lang=en and https://www.isdt.co/compare?lang=hk

---

### 3.13 4010 24 V fan

| Item | Value | Tag | Source |
|---|---|---|---|
| Frame | 40 x 40 x 10.5 (±0.5) | spec (Orion OD4010) | [F1] |
| Hole spacing | 32 ±0.5 square | spec | [F1] |
| Hole Ø | 4x Ø4.3 ±0.3 | spec (Orion) | [F1] |
| Mass | 0.05 lb (≈23 g) | spec | [F1] |
| Noctua NF-A4x10 24V PWM | 40 x 40 x 10 (11 without pads), holes 32 x 32, 15 g, 4-pin with included 2-pin JST-XHP-2 adaptor | spec | [F2] |
| Generic 4010S 24 V | 40 x 40 x 10, 32 hole spacing, XH2.54-2P | listing | [F3] |
| Hole Ø, generic / Noctua | not stated | not found | [F2], [F3] |

- [F1] https://orionfans.com/productFiles/datasheet/OD4010.pdf
- [F2] https://www.noctua.at/en/products/nf-a4x10-24v-pwm/specifications
- [F3] https://www.bitsandparts.nl/en/fan-axial-4010s-40x40x10mm-24vdc-p1906924
- Some generic listings in search results claimed 35 spacing (not opened). Treat 32 as standard and measure the part.
- Hole Ø differs by maker (Orion 4.3). Use about Ø3.4 for M3 clearance only after checking the actual fan.

---

### 3.14 XT60 and JST-XH connectors

**XT60 (Amass)**

| Item | Value | Tag | Source |
|---|---|---|---|
| XT60 (F) face | 15.50 x 8.10, pin pitch 7.20, body length 15.70 | spec | [X1] |
| XT60H-M (with sheath) face | 16.40 x 9.20, pin pitch 7.20, length 23.80 (20.50 to sheath step) | spec | [X2] |
| XT60E1-M panel mount, flange | 27.10 x 12.10, 3.00 thick | spec | [X3] |
| XT60E1-M screw hole spacing | 20.50 c-c (along the 27.10 side) | spec | [X3] |
| XT60E1-M depth | 16.70 overall; solder cups extend 5.00 past rear body | spec | [X3] |
| XT60E1-M hole Ø / screw | not stated | not found | [X3] |
| XT60E1-M panel cutout | not stated. The body behind the flange is narrower than 12.10 x 27.10 but is not dimensioned. | not found | [X3] |

- [X1] https://probots.co.in/technical_data/connectors/ProbotsXT60EFemalePanelMountConnectorDatasheet.pdf. Despite its file name, this is the Amass spec for the plain XT60-F/XT60-M.
- [X2] https://cdn.eicom.ru/media/PDF/XT60H-M.pdf (Amass spec V1.2)
- [X3] https://datasheet.lcsc.com/datasheet/pdf/64afc687316c2b3fea90e9f41a370702.pdf?productCode=C3040679 (Amass XT60E1-M.G.Y spec, via LCSC)

**JST XH (2.5 mm pitch)**

| Item | Value | Tag | Source |
|---|---|---|---|
| Pitch | 2.5 | spec | [J1] |
| Housing XHP-n length B | 2.5(n−1) + 4.8, e.g. XHP-2 7.3, XHP-3 9.8, XHP-4 12.3 | spec | [J1] |
| Housing depth | 5.7 | spec | [J1] |
| Header B-nB-XH-A length B | 2.5(n−1) + 4.9, e.g. 2p 7.4, 3p 9.9, 4p 12.4 | spec | [J1] |
| Header depth / height | 5.75 / 7.0 above PCB | spec | [J1] |
| Mated height above PCB | 9.8 | spec | [J1] |
| Side-entry header (S-nB-XH-A) | same B; height above PCB 4.5–7, depth C = 9.2 (or 7.6 for -1) | spec | [J1] |

- [J1] https://www.jst-mfg.com/product/pdf/eng/eXH.pdf

---

### 3.15 Pogo pins

**P75-B1 (generic test probe)**

| Item | Value | Tag | Source |
|---|---|---|---|
| Barrel (needle tube) Ø | 1.0 | listing | [P1] |
| Plunger Ø | 0.66 | listing | [P1] |
| Receptacle (socket) Ø | 1.32 (sold separately) | listing | [P1] |
| Total length | about 16.5 | listing | [P1] |
| Full stroke | 2.50 | listing | [P1] |
| Mounting hole | drill about 1.4 (for the socket) | listing | [P1] |
| Rating | 3 A | listing | [P1] |

- Other P75 listings seen in search give a 1.02 barrel with 27.8 or 33.35 length (PM75/PL75), and P75-E2 with 1.3–1.5 barrel. These were not opened and conflict with [P1], so treat P75 sizes as **unverified** until you have the actual part.

**Mill-Max 0854-0-15-20-82-14-11-0 (solder-cup, high-current)**

| Item | Value | Tag | Source |
|---|---|---|---|
| Plunger Ø | 1.270 | spec | [P2] |
| Body Ø | 2.413 | spec | [P2] |
| Body length | 5.563 | spec | [P2] |
| Flange | Ø3.175 x 0.762 | spec | [P2] |
| Solder-cup hole depth | 4.572 (wire up to 16 AWG) | spec | [P2] |
| Overall / initial height | 15.215 | spec | [P2] |
| Max stroke | 2.286 (±0.127) | spec | [P2] |
| Mid stroke (rated) | 1.143 | spec | [P2] |
| Force | 25 g preload, 120 ±20 g at mid stroke | spec | [P2] |
| Current | 12 A @ 30 °C rise (9.6 A derated); contact resistance 20 mΩ max | spec | [P2] |
| Tolerances | lengths ±0.15, diameters ±0.051 | spec | [P2] |

- [P1] https://domoticx.net/webshop/pogo-pin-test-probe-0-75mm-punt-p75-b1
- [P2] https://www.mill-max.com/products/datasheet/0854-0-15-20-82-14-11-0 (via product page https://www.mill-max.com/products/discrete-spring-loaded-pins/spring-loaded-pin-with-solder-cup-termination/0854/0854-0-15-20-82-14-11-0)

---

### 3.x Open gaps (not found)

- SKR Mini E3 V3 mounting-hole diameter.
- MG995 ear-hole spacing, hole Ø and shaft offset.
- DS3218 shaft offset (only scaled from the drawing).
- Heschen HS-P20/15 itself (generic P20/15 data used instead).
- ISDT "PD60S" (PD60 data used instead).
- XT60E1-M screw-hole Ø and panel cutout.
- Generic KW11-3Z drawing (Omron SS-5GL used instead).
- 16T-size toothless idler with a 5 mm bore.
- Generic 4010 hole Ø (only Orion's Ø4.3).
