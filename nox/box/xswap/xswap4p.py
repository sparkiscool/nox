"""v4 pincer, four stronger versions (Alex 27 Oct: PETG, stronger pincer, artifact only, no STL yet).

Shared fix in all four: the jaw is a base + a screwed lid (v4's one-piece jaw sealed the racks and pinion in, so it
could not be assembled), and each finger's arm runs right under the lid, so a pull on the post pushes the arm up into
the lid instead of bending it as a cantilever (v4's weak link: 25.8 MPa at the 57 N XT60 pull).

  P1 Lid + thicker   : lid 4, arm 10 x 6, flange 3.0
  P2 Twin post       : P1 + a second (anchor) post per finger in its own keyhole: halves post, flange and shell-lip load
  P3 Steel core      : P1 + a 3 mm steel dowel through each post into the arm: steel carries the post's tension and bending
  P4 Twin + steel    : P2 + P3, lid 5

Shell, gate bars, holder are v4's; P2 / P4 add one keyhole per end to the shell bottom.
"""
import math
import numpy as np
import xswap4 as X
from xswap4 import M, box, cyl, U, hull, sym, SEG

VARIANTS = {
    "P1": dict(name="Lid + thicker", lid=4.0, arm_w=10.0, arm_h=6.0, fl_t=2.0, twin=False, steel=False),
    "P2": dict(name="Twin post", lid=4.0, arm_w=10.0, arm_h=6.0, fl_t=2.0, twin=True, steel=False),
    "P3": dict(name="Steel core", lid=4.0, arm_w=10.0, arm_h=6.0, fl_t=2.0, twin=False, steel=True),
    "P4": dict(name="Twin + steel", lid=5.0, arm_w=10.0, arm_h=6.0, fl_t=2.0, twin=True, steel=True),
}
TWIN_DY = 18.0                       # anchor post outboard of the drive post (toward the shell end)
ANC_HY = 6.0                         # anchor flange half length (y): clears the bar's knee at y 70.5
PAD_HX, PAD_HY = 7.0, 7.0            # bearing pad round each post, under the lid
STEEL_R = 1.5                        # 3 mm dowel
PIN_L_DRIVE, PIN_L_ANCHOR = 20.0, 12.0   # standard 3 x 20 and 3 x 12 dowels, inserted from under the arm
PRESS_R = 1.475                      # 2.95 hole: press fit in PETG (+ a drop of CA)
LID_SCREWS = [(sx * 17.0, None) for sx in (-1, 1)]

class G:
    """per-variant z stack (everything below the jaw top moves with the lid and arm sizes)."""
    def __init__(s, c):
        s.c = c
        s.z_lid1 = X.Z_JT                         # -70.6 lid top = jaw top
        s.z_lid0 = X.Z_JT - c["lid"]
        s.z_arm1 = s.z_lid0 - X.G                 # arm top 0.3 under the lid
        s.z_arm0 = s.z_arm1 - c["arm_h"]
        s.rk1 = s.z_arm0; s.rk0 = s.rk1 - 10.0
        s.sv_top = s.rk0 - 2.8 - 3.5
        s.z_ax = s.sv_top - X.SV_H / 2
        s.fl0 = X.Z_FL_0; s.fl1 = X.Z_FL_0 + c["fl_t"]
        s.post_top = X.Z_POST_TOP

def flange_box(x, y, hy, g):
    f = box(x - X.FL_HX, x + X.FL_HX, y - hy, y + hy, g.fl0, g.fl1)
    ch = 0.6
    f = f - U(*[hull([[x + sx * (X.FL_HX + 0.01), yy, z] for yy in (y - hy - 1, y + hy + 1) for z in (g.fl1 - ch, g.fl1 + 0.01)] +
                     [[x + sx * (X.FL_HX - ch), yy, g.fl1 + 0.01] for yy in (y - hy - 1, y + hy + 1)]) for sx in (-1, 1)])
    return f

def finger_A(key, u):
    c = VARIANTS[key]; g = G(c); x = -u
    p = math.pi * X.M_G
    x0 = -X.RK_L / 2
    shift = (-(x0 + p / 2)) % p
    rack = X.V3.rack_2d(X.RK_L, X.M_G, shift=shift).extrude(g.rk1 - g.rk0).translate([x0 + x, X.Y_P + X.R_P, g.rk0])
    y_end = X.FIN_Y + (TWIN_DY if c["twin"] else 0.0) + PAD_HY
    arm = box(x - c["arm_w"] / 2, x + c["arm_w"] / 2, X.Y_P + X.R_P + X.M_G + 0.6, y_end, g.z_arm0, g.z_arm1)
    parts = [rack, arm]
    posts = [(X.FIN_Y, True)] + ([(X.FIN_Y + TWIN_DY, False)] if c["twin"] else [])
    for y, drive in posts:
        parts.append(box(x - PAD_HX, x + PAD_HX, y - PAD_HY, y + PAD_HY, g.z_arm0, g.z_arm1))          # bearing pad
        parts.append(cyl("z", x, y, X.SLOT_R_POST, g.z_arm1 - 0.01, g.fl0 + 0.01))                    # neck through lid + shell
        parts.append(flange_box(x, y, X.FL_HY if drive else ANC_HY, g))
        if drive:
            parts.append(cyl("z", x, y, X.SLOT_R_POST, g.fl1 - 0.01, g.post_top - 1.5))
            parts.append(M.cylinder(1.5, X.SLOT_R_POST, X.SLOT_R_POST - 1.5, SEG).translate([x, y, g.post_top - 1.5]))
    f = U(*parts)
    if c["steel"]:
        for y, drive in posts:
            L = PIN_L_DRIVE if drive else PIN_L_ANCHOR
            f = f - cyl("z", x, y, PRESS_R, g.z_arm0 - 1.0, g.z_arm0 + L)          # open under the arm: press in from below
    return f

def finger_B(key, u): return sym(finger_A(key, u))

def steel_pins(key, u):
    c = VARIANTS[key]; g = G(c)
    if not c["steel"]: return None
    x = -u; out = []
    for y, drive in [(X.FIN_Y, True)] + ([(X.FIN_Y + TWIN_DY, False)] if c["twin"] else []):
        L = PIN_L_DRIVE if drive else PIN_L_ANCHOR
        out.append(cyl("z", x, y, PRESS_R, g.z_arm0, g.z_arm0 + L))
    a = U(*out)
    return a + sym(a)

def _extent(key):
    c = VARIANTS[key]
    sy_c = X.Y_P - (X.SV_L / 2 - X.SV_SHAFT)
    y1 = max(X.FIN_Y + (TWIN_DY if c["twin"] else 0) + PAD_HY + 4.0, sy_c + X.SV_EAR_L / 2 + 4.0)
    y0 = min(2 * X.Y_P - y1, sy_c - X.SV_EAR_L / 2 - 4.0)
    return y0, y1, sy_c

def lid_screws(key):
    y0, y1, _ = _extent(key)
    return [(sx * 17.0, y) for sx in (-1, 1) for y in (y0 + 5.5, y1 - 5.5)]

def lid(key):
    c = VARIANTS[key]; g = G(c); y0, y1, _ = _extent(key)
    L = box(-X.JW, X.JW, y0, y1, g.z_lid0, g.z_lid1)
    def slots_A():
        s = []
        for y in [X.FIN_Y] + ([X.FIN_Y + TWIN_DY] if c["twin"] else []):
            r = X.SLOT_R_POST + 0.6
            s.append(M.batch_hull([cyl("z", xx, y, r, g.z_lid0 - 1, g.z_lid1 + 1) for xx in (0.0, -X.STROKE)]))
        return U(*s)
    sa = slots_A()
    L = L - sa - sym(sa)
    for x, y in lid_screws(key):
        L = L - cyl("z", x, y, 1.7, g.z_lid0 - 1, g.z_lid1 + 1) - cyl("z", x, y, 3.1, g.z_lid1 - 1.8, g.z_lid1 + 1)
    return L

def base(key):
    """v4 jaw body cut at the lid: arm / pad / rack channels open to the top (assembly from above), no lightening cores."""
    c = VARIANTS[key]; g = G(c); y0, y1, sy_c = _extent(key)
    zb = g.sv_top - X.SV_H
    b = box(-X.JW, X.JW, y0, y1, zb, g.z_lid0)
    def cuts_A():
        yend = X.FIN_Y + (TWIN_DY if c["twin"] else 0.0) + PAD_HY
        cc = box(-X.STROKE - PAD_HX - 0.6, PAD_HX + 0.6, X.Y_P + X.R_P - 1.6, yend + 0.6, g.z_arm0 - 0.3, g.z_lid0 + 1)
        cc = cc + box(-X.RK_L / 2 - X.STROKE - 0.6, X.RK_L / 2 + 0.6, X.Y_P + X.R_P - X.M_G - 0.3, X.Y_P + X.R_P + 1.25 * X.M_G + 7.3,
                      g.rk0 - 0.3, g.z_lid0 + 1)
        return cc
    ca = cuts_A()
    b = b - ca - sym(ca)
    b = b - cyl("z", 0, X.Y_P, X.R_P + X.M_G + 0.6, g.rk0 - 0.3, g.z_lid0 + 1) - cyl("z", 0, X.Y_P, 15.1, g.rk0 - 3.3, g.rk0 + 0.01)
    ear_top = g.sv_top - X.SV_EAR_DZ
    b = b - box(-X.SV_W / 2 - 0.25, X.SV_W / 2 + 0.25, sy_c - X.SV_L / 2 - 0.25, sy_c + X.SV_L / 2 + 0.25, zb - 1, g.sv_top + 0.01)
    b = b - cyl("z", 0, X.Y_P, 7.0, g.sv_top - 0.1, g.rk0 - 3.2)
    b = b - box(-X.SV_W / 2 - 0.25, X.SV_W / 2 + 0.25, sy_c - X.SV_EAR_L / 2 - 0.5, sy_c + X.SV_EAR_L / 2 + 0.5, zb - 1, ear_top)
    for sy in (-1, 1):
        for sx in (-1, 1):
            b = b - cyl("z", sx * 5.0, sy_c + sy * X.SV_HOLE, X.V3.INS_D / 2, ear_top - 0.01, ear_top + 6.5)
    for sx in (-1, 1):
        bore = cyl("x", X.Y_P, g.z_ax, 4.1, (X.JW - 9.0) if sx > 0 else -X.JW - 1, (X.JW + 1) if sx > 0 else -(X.JW - 9.0))
        sset = cyl("z", sx * (X.JW - 4.5), X.Y_P, X.V3.INS_D / 2, zb - 1, zb + 6.0) + cyl("z", sx * (X.JW - 4.5), X.Y_P, 1.7, zb - 1, g.z_ax)
        b = b - bore - sset
    for x, y in lid_screws(key):
        b = b - cyl("z", x, y, X.V3.INS_D / 2, g.z_lid0 - 6.5, g.z_lid0 + 0.01)
    return b

def pinion(key):
    g = G(VARIANTS[key])
    p = X.V3.involute_gear_2d(X.M_G, X.Z_G).extrude(g.rk1 - g.rk0 + 0.31).translate([0, X.Y_P, g.rk0 - 0.3])
    p = p + cyl("z", 0, X.Y_P, 14.5, g.rk0 - 2.8, g.rk0 - 0.29)
    p = p - cyl("z", 0, X.Y_P, 10.9, g.rk0 - 3.3, g.rk0 - 0.3) - cyl("z", 0, X.Y_P, 1.7, g.rk0 - 3, g.rk1 + 1)
    return p

def servo(key):
    g = G(VARIANTS[key]); sy_c = X.Y_P - (X.SV_L / 2 - X.SV_SHAFT)
    s = box(-X.SV_W / 2, X.SV_W / 2, sy_c - X.SV_L / 2, sy_c + X.SV_L / 2, g.sv_top - X.SV_H, g.sv_top)
    s = s + box(-X.SV_W / 2, X.SV_W / 2, sy_c - X.SV_EAR_L / 2, sy_c + X.SV_EAR_L / 2, g.sv_top - X.SV_EAR_DZ - X.SV_EAR_T, g.sv_top - X.SV_EAR_DZ)
    return s + cyl("z", 0, X.Y_P, 6.5, g.sv_top, g.rk0 - 2.8) + cyl("z", 0, X.Y_P, 10.75, g.rk0 - 2.8, g.rk0 - 0.3)

def pincer(key, u):
    a = math.degrees(u / X.R_P)
    d = dict(base=base(key), lid=lid(key), finger_a=finger_A(key, u), finger_b=finger_B(key, u),
             pinion=X.rot_z(pinion(key), a, 0.0, X.Y_P), servo=servo(key))
    st = steel_pins(key, u)
    if st is not None: d["steel"] = st
    return d

def shell_body(key):
    s = X.shell_body()
    if VARIANTS[key]["twin"]:
        k = X.tslot_cuts().translate([0, TWIN_DY, 0])
        s = s - k - sym(k)
    return s

# ============================================================================== loads (E)
PULL_N, PULL_MIS = 57.0, 150.0       # XT60 pull + 1 g; rail stall (misuse, E)
PUSH_N, PUSH_MIS = 15.2, 41.0        # drive push at full swing; servo stall (MG996R ~1.1 N m over 2 fingers, E)
ALLOW_IN, ALLOW_ACROSS, SF = 45.0, 15.0, 2.0
E_ST, E_PETG = 200e3, 2.0e3

def stresses(key, pull, push):
    """member stresses MPa for one load pair (per finger: pull / 2, push). in / across = PETG layer direction."""
    c = VARIANTS[key] if key != "v4" else None
    twin = bool(c and c["twin"]); steel = bool(c and c["steel"])
    n = 2 if twin else 1
    out = {}
    fp = pull / 2.0                                              # per finger
    r = X.SLOT_R_POST; A = math.pi * r * r
    if steel:
        As = math.pi * STEEL_R ** 2
        out["post tension (steel)"] = (fp / n / As, 400.0, "steel")
        out["post tension"] = (fp / n * (E_PETG * (A - As)) / (E_PETG * (A - As) + E_ST * As) / (A - As), ALLOW_ACROSS, "across")
    else:
        out["post tension"] = (fp / n / A, ALLOW_ACROSS, "across")
    I_p = math.pi * r ** 4 / 4
    Mb = push * 8.0
    if steel:
        Is = math.pi * STEEL_R ** 4 / 4; Ip = I_p - math.pi * (STEEL_R + 0.05) ** 4 / 4
        share = E_PETG * Ip / (E_PETG * Ip + E_ST * Is)
        out["post bending"] = (Mb * share * r / Ip, ALLOW_ACROSS, "across")
    else:
        out["post bending"] = (Mb * r / I_p, ALLOW_ACROSS, "across")
    flt = c["fl_t"] if c else X.FL_T
    out["flange"] = ((fp / n / 2) * 1.75 / (2 * X.FL_HX * flt ** 2 / 6), ALLOW_IN, "in")
    out["shell lips"] = ((fp / n / 2) * 1.5 / (2.8 * X.BP_T ** 2 / 6), ALLOW_ACROSS, "across")
    if c:                                                         # lid-borne: pad -> lid between screws ~40 apart
        Zl = 20.0 * c["lid"] ** 2 / 6
        out["lid"] = (fp * 40.0 / 8 / Zl, ALLOW_IN, "in")
        out["arm"] = (fp / n * 3.0 / (c["arm_w"] * c["arm_h"] ** 2 / 6), ALLOW_IN, "in")   # pad overhang only
    else:
        L = X.FIN_Y - (X.Y_P + X.R_P + X.M_G + 0.6)
        out["arm"] = (fp * L / (X.ARM_W * X.ARM_H ** 2 / 6), ALLOW_IN, "in")
    out["rack teeth"] = (push / (10.0 * X.M_G * 0.3), ALLOW_IN, "in")
    return out

def safety(key):
    rows = {}
    for case, (pl, ps) in {"normal": (PULL_N, PUSH_N), "misuse": (PULL_MIS, PUSH_MIS)}.items():
        s = stresses(key, pl, ps)
        rows[case] = {k: dict(mpa=round(v[0], 2), allow=v[1], layer=v[2], sf=round(v[1] / max(v[0], 1e-6), 2)) for k, v in s.items()}
    return rows


def steel_pins_A(key, u):
    """steel pins of finger A only (B's are its 180 deg copy), for animation."""
    c = VARIANTS[key]; g = G(c)
    if not c["steel"]: return None
    x = -u
    return U(*[cyl("z", x, y, PRESS_R, g.z_arm0, g.z_arm0 + (PIN_L_DRIVE if d else PIN_L_ANCHOR))
               for y, d in [(X.FIN_Y, True)] + ([(X.FIN_Y + TWIN_DY, False)] if c["twin"] else [])])
