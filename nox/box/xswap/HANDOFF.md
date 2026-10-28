# xswap handoff (28 Oct 2026)

Alex's drone battery swap station: a flip-swap box for a Zeee 4S 5200 mAh 120C hard-case pack (138 x 47 x 48 mm,
470 to 480 g), XT60 only, PETG on a Bambu A1 mini (180 bed). Reviewer: Spark (sparkiscool).

## Where things are
- Repo: `jy202108g-bot/REPO1` (a full copy of `sparkiscool/nox`), branch **`xswap-v1`**. Local clone: `origin` = REPO1,
  `spark` = sparkiscool/nox (never push there).
- Code: `nox/box/xswap/`. Results: `runs/xswap/v1..v4`, `runs/xswap/v4p` (pincer options).
- Research: `claude/research/2026-10-27-xswap-latch-v4.md`. Design doc: `nox/box/xswap/README_v4.md` (also in the Nox
  project as `claude/xswap-v4-latch.md`).

## Design history
- v1: flexure clips on the shell sides. v2: pinned rocker latch + guide posts + funnel. v3: cradle pincer.
- **v4 (current latch, Spark's review 26 Oct):** latch under the pack. Gate bar per end on a Ø4 steel dowel, steel
  spring, tip 4.2 deep on a 12 mm ledge in holder end legs; lever 1:4.8 (finger 3.8 mm, tip ~14 mm); pincer 44 wide.
  `xswap4.py`, checks `xswap4_check.py` (all pass), print files + strength coupons in `runs/xswap/v4/`.
- **v4 pincer options (27 Oct):** `xswap4p.py`, `xswap4p_check.py`. All four split the jaw into base + lid (v4's jaw
  could not be assembled) and make the arm bear on the lid (v4's arm: 1.7x normal, 0.66x misuse).
  P1 lid + thicker, P2 twin post (+1 keyhole per end in the shell), P3 steel core (Ø3 x 20 pin per post),
  P4 twin + steel (lid 5). Weakest part in all four: the shell's keyhole lips (P1/P3 2.0x / 0.8x, P2/P4 4.1x / 1.6x).
  Steel pin holes open under the arm (press fit 2.95, CA). Flip clear at rail 80 + 80.

## Pages (claude.ai artifacts, private until shared from the Share menu)
- Flip Swap Station (v4 latch, animated): https://claude.ai/artifact/PzbZaPetxmjpgotpwV1Qg3
- Pincer Strength Options (P1-P4 compare): https://claude.ai/artifact/QyoGN8jRpnNzNSA8qBx7ep
- Flip Swap P3 Steel Core: https://claude.ai/artifact/5cYXgTB8jqR86kx3n8Lvda
- Flip Swap P4 Twin Steel: https://claude.ai/artifact/MGernDAnJMtVrSUbJ3QBy8
P3 and P4 were drafted in an email to Yafei.shou@gmail.com; Alex shares them from the Share menu.

## Open / next
1. **Alex picks P3 or P4**, then export its print files (no STLs exist for any pincer option yet; Alex asked to wait).
   Reuse `xswap4_export.py` with `xswap4p` parts; run `xswap_solid.py` on every file.
2. Fusion: the Fusion MCP works through the linked laptop (Windows, `laptop-97hmejge`); an empty design is open. To
   import STLs, Alex must add a folder in the desktop app ("+" > "Add folder"); folder requests are blocked otherwise.
3. Shell keyhole lips are now the weak point (bend across layers); a thicker bottom plate or rotated print would help.
4. Spark said "focus on latching and unlatching"; the flip axis sits low in the jaw (sweep R ~142, rail 80 + 80).
5. Measure the real pack, servo, horn and XT60E1-M flange before printing; first print the v4 coupons.
6. Numbers are hand calculations (E); misuse loads (150 N rail stall, 41 N servo stall) are estimates.
