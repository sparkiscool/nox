"""Belt clamp v2 in the heads (17 Oct 2026, head A v5 chat): the loop-and-pin clamp of belt_clamp_v2.py drawn into each
head, replacing belt_clamp.head_clamp (v1, retired). Same return keys as v1 so arm_cad._glide_clamp swaps in place.

Head frame (arm_cad): X tangential, Y along the axle, u radial from the glide's inner end. The clamped run is the INNER
run at pitch line X = x_pitch (A 27.65, B 48.65 in B's frame), teeth toward +X (the pulley centre), belt Y y_belt..+6.
Both ends back to back on that line, mouths outward (box-arm.md "Head belt clamp v2"): from the zone bottom u0:
  slot 12 (6 meshed teeth) -> throat 3.6 -> pin 1 at u0 + 15.6 -> 8.5 -> pin 2 at u0 + 24.1 -> throat 3.6 -> slot 12 -> mouth at
  u0 + 39.7 (zone 40: CLAMP_DZ / CLAMP_DZ_B unchanged). Pins along Y (across the belt width) through both Y walls: M3 x 12 +
  nut, or a Ø3 dowel (DEFAULT). Sight window on +X over each pocket.
Slot centre X = the meshed pair's centre with the loaded leg on the run line: x_pitch - PLD - BACK + MESHED / 2.
SLOT (the coupon's pick, 1.8 / 2.0 / 2.2) is applied as an exact cut at build time; the generated keep-out is cut at the
widest (2.2), so one layout run serves any pick: rebuild with --slot after the hang test.
Keep-outs (C): 80T flange + 1 (r 28.5, Y 53..65, binding at park z0 32: the body's -X wall is trimmed where it would cross
it); the belt line in and out of both mouths; nut / head + driver along the pins; every void of the clamp.
All numbers C unless tagged. python belt_clamp_v2_head.py -> checks and out/head_clamp_v2_A.stl (drawn body alone).
"""
import math, os
import numpy as np
import manifold3d as mf
import belt_clamp_v2 as B

PLD = 0.254                                   # pitch line to the tooth-side land (GT2 spec, E)
SLOT_DEFAULT = 2.0                            # DEFAULT until the coupon picks (box-arm.md)
SLOT_KEEP = 2.2                               # the generator keeps out of the widest slot
PIN_GAP = 8.5                                 # pin 1 to pin 2 along u (pockets r 3.23 x 2 + 2.04 wall)
ZONE = 2 * (B.SLOT_L + 3.6) + PIN_GAP         # 39.7
NUT_R, NUT_DEPTH = 3.5, 6.0                   # M3 nut / head + driver keep-out along Y on both faces (DEFAULT)
FLANGE_R1, FLANGE_Y = 28.5, (53.0, 65.0)      # 80T flange 55 + CLR 1, Y 54..64 + 1 (arm_layout P80_FL, Y_P80)
Z0_PARK = 32.0                                # the clamp's lowest pass over the flange (arm_layout Z0_A_HOME)
GLIDE_SKIN_X, Y_BAR0 = 20.3, 74.0
try:
    import arm_layout as _L
    _CZ = {"A": tuple(_L.CLAMP_DZ), "B": tuple(_L.CLAMP_DZ_B)}
    Z0_PARK = _L.Z0_A_HOME
    FLANGE_R1 = _L.P80_FL / 2 + _L.CLR_POST
    FLANGE_Y = (_L.Y_P80[0] - _L.CLR_POST, _L.Y_P80[0] + _L.P80_W + _L.CLR_POST)
except ImportError:
    _CZ = {"A": (-18.6, 21.4), "B": (-15.6, 24.4)}
HEADS = {
    "A": dict(x_pitch=27.65, y_belt=66.0, cz=_CZ["A"], bridge_u=(0.0, _CZ["A"][1])),
    "B": dict(x_pitch=48.65, y_belt=56.0, cz=_CZ["B"], bridge_u=(-4.0, _CZ["B"][1])),
}

box = B.box
def hull(pts): return mf.Manifold.hull_points(np.array(pts, dtype=np.float64))
def cylY(x, z, y0, y1, r, n=48):
    return mf.Manifold.cylinder(y1 - y0, r, r, n).rotate([-90, 0, 0]).translate([x, y0, z])


def layout(h):
    """the clamp's frame numbers for head h: slot centre X, channel Y, body X / Y extents, pin u positions, mouths."""
    H = HEADS[h]
    xc = H["x_pitch"] - PLD - B.BACK + B.MESHED / 2            # meshed pair centre (loaded leg on the run line)
    yc0 = H["y_belt"] + B.BELT_W / 2 - B.CH_W / 2              # channel across the belt width (6.4 round the 6 belt)
    yc1 = yc0 + B.CH_W
    u0 = H["cz"][0]
    pins = (u0 + B.SLOT_L + 3.6, u0 + B.SLOT_L + 3.6 + PIN_GAP)
    return dict(xc=xc, x0=xc - B.X_HALF, x1=xc + B.X_HALF, yc=(yc0, yc1), y0=yc0 - B.WALL_Z, y1=yc1 + B.WALL_Z,
                u0=u0, u1=u0 + ZONE, pins=pins)


def voids(h, slot):
    """every cavity of the clamp for head h at slot width `slot`: two slots with lead-ins, throats, pockets, pin holes
    through both Y walls, sight windows on +X."""
    L = layout(h); xc = L["xc"]; yc0, yc1 = L["yc"]; u0, u1 = L["u0"], L["u1"]
    v = None
    for k, (mouth, pin, sgn) in enumerate(((u0, L["pins"][0], +1), (u1, L["pins"][1], -1))):
        s_end = mouth + sgn * B.SLOT_L                                       # slot end (throat start)
        slotb = box(xc - slot / 2, xc + slot / 2, yc0, yc1, min(mouth - sgn * 1.0, s_end), max(mouth - sgn * 1.0, s_end))
        lead = hull([[xc + sx * (slot / 2 + B.LEAD), y, mouth - sgn * 0.01] for sx in (-1, 1) for y in (yc0, yc1)] +
                    [[xc + sx * slot / 2, y, mouth + sgn * B.LEAD] for sx in (-1, 1) for y in (yc0, yc1)])
        throat = hull([[xc + sx * slot / 2, y, s_end] for sx in (-1, 1) for y in (yc0, yc1)] +
                      [[xc + sx * B.POCKET_R, y, pin] for sx in (-1, 1) for y in (yc0, yc1)])
        pocket = cylY(xc, pin, yc0, yc1, B.POCKET_R, 64)
        window = box(xc, L["x1"] + 1.0, yc0, yc1, pin - B.WINDOW_W / 2, pin + B.WINDOW_W / 2)
        hole = cylY(xc, pin, L["y0"] - 1.0, L["y1"] + 1.0, B.PIN_HOLE / 2, 32)
        part = slotb + lead + throat + pocket + window + hole
        v = part if v is None else v + part
    return v


def body(h, slot=SLOT_DEFAULT):
    """the drawn clamp body for head h (its interface): a block over the zone, the voids cut, the 80T flange keep-out
    trimmed off its -X / -Y corner (A only reaches it)."""
    L = layout(h)
    b = box(L["x0"], L["x1"], L["y0"], L["y1"], L["u0"], L["u1"])
    b -= voids(h, slot)
    b -= flange_keep()
    return b


def flange_keep():
    """the fixed 80T flange + 1 in the head frame at park (z0 Z0_PARK): the head frame's Z = W Z - z0. At larger z0 the
    circle sits further down the bar, so park binds for anything at u >= the zone bottom."""
    return cylY(0.0, -Z0_PARK, FLANGE_Y[0], FLANGE_Y[1], FLANGE_R1, 128)


def head_clamp(h, slot=SLOT_DEFAULT):
    """clamp for head h in its head frame. drawn = the clamp body (interface). The joint to the glide's side skin is
    GENERATED inside env (as v1: under the bar band over the whole zone, plus the bridge up beside the glide skin to
    Y 78; B's bridge only at u >= -4). keep = everything never generated into: the voids at the widest slot, the belt
    line in and out of both mouths, the pin hardware + driver on both Y faces, the 80T flange keep-out.
    ends = the two meshed-slot spans (load regions: 93 N along -u on the lower end, +u on the upper, E).
    plate = None (no plate in v2; kept for arm_cad's dict)."""
    L = layout(h); H = HEADS[h]
    drawn = body(h, slot)
    xp = H["x_pitch"]; yb = H["y_belt"]
    keep = voids(h, SLOT_KEEP)
    keep += box(xp - 1.0, xp + 1.6, yb - 0.5, yb + B.BELT_W + 0.5, L["u0"] - 120.0, L["u0"] + 0.01)      # run in (back 0.88 behind the pitch line, teeth 1.0 in front)
    keep += box(xp - 1.0, xp + 1.6, yb - 0.5, yb + B.BELT_W + 0.5, L["u1"] - 0.01, L["u1"] + 120.0)      # run out
    for pin in L["pins"]:
        keep += cylY(L["xc"], pin, L["y0"] - NUT_DEPTH, L["y0"] + 0.01, NUT_R)                           # head / nut + driver
        keep += cylY(L["xc"], pin, L["y1"] - 0.01, L["y1"] + NUT_DEPTH, NUT_R)
    keep += flange_keep()
    bu0, bu1 = H["bridge_u"]
    env = box(GLIDE_SKIN_X, L["x1"] + 0.5, L["y0"] - 0.5, Y_BAR0, L["u0"], L["u1"])
    env += box(GLIDE_SKIN_X, L["x1"] + 0.5, Y_BAR0 - 0.01, Y_BAR0 + 4.0, max(L["u0"], bu0), bu1)
    env -= flange_keep()
    ends = [box(L["xc"] - 1.2, L["xc"] + 1.2, L["yc"][0], L["yc"][1], L["u0"], L["u0"] + B.SLOT_L),
            box(L["xc"] - 1.2, L["xc"] + 1.2, L["yc"][0], L["yc"][1], L["u1"] - B.SLOT_L, L["u1"])]
    ends = [(e + box(L["x0"], L["x1"], L["y0"], L["y1"], e.bounding_box()[2], e.bounding_box()[5])) ^ drawn for e in ends]
    return dict(drawn=drawn, plate=None, keep=keep, env=env, ends=ends, layout=L)


def belt_loops(h, slot=SLOT_DEFAULT):
    """both folded belt ends as seated (checks and pictures), head frame."""
    L = layout(h); out = None
    for mouth, pin, sgn in ((L["u0"], L["pins"][0], +1), (L["u1"], L["pins"][1], -1)):
        bl = B.belt_loop(slot)                                              # coupon frame: x across, y along (+y away from the load), z width
        m = bl.transform([[1, 0, 0, 0], [0, 0, 1, 0], [0, 1, 0, 0]])          # -> (x, u, y): y_c -> u, z -> Y
        m = m.translate([L["xc"], L["y0"], 0.0])                           # coupon z 0 = the body's outer Y face
        if sgn < 0: m = m.mirror([0, 0, 1])
        m = m.translate([0, 0, mouth])
        out = m if out is None else out + m
    return out


def check(h, slot=SLOT_DEFAULT):
    c = head_clamp(h, slot); L = c["layout"]; d = c["drawn"]
    bb = d.bounding_box()
    out = dict(head=h, slot=slot, g=round(d.volume() / 1000 * 1.27, 2), bodies=len(d.decompose()),
               x=(round(L["x0"], 2), round(L["x1"], 2)), y=(round(L["y0"], 2), round(L["y1"], 2)),
               u=(round(L["u0"], 1), round(L["u1"], 1)), pins=tuple(round(p, 1) for p in L["pins"]),
               xc=round(L["xc"], 2))
    bl = belt_loops(h, slot)
    out["belt_vs_body_mm3"] = round((bl ^ d).volume(), 3)
    out["flange_gap"] = round(d.min_gap(flange_keep(), 5.0), 2)
    out["flange_trim_mm3"] = round((box(L["x0"], L["x1"], L["y0"], L["y1"], L["u0"], L["u1"]) ^ flange_keep()).volume(), 1)
    # thinnest walls: slot side wall, pocket side wall, pocket-to-pocket wall, Y walls
    out["walls"] = dict(slot_side=round(B.X_HALF - slot / 2, 2), pocket_side=round(B.X_HALF - B.POCKET_R, 2),
                        between_pockets=round(PIN_GAP - 2 * B.POCKET_R, 2), y_walls=B.WALL_Z)
    return out


if __name__ == "__main__":
    os.makedirs("out", exist_ok=True)
    for h in ("A", "B"):
        r = check(h)
        print(h, r)
        c = head_clamp(h)
        B.export(c["drawn"], f"out/head_clamp_v2_{h}.stl")
        B.export(c["drawn"] + belt_loops(h), f"out/head_clamp_v2_{h}_with_belt.stl")
