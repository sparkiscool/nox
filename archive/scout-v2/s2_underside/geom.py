"""S2 underside v1 geometry (Scout v2), 13 Oct 2026, underside chat.

Every number of the part lives here. Drone coordinates (x nose +, y left +, z up, prop plane z 0), mm.
Tags: P part, C CAD (Radxa STEP v1.11, MicoAir spec), E estimate, DEFAULT = chosen here (v2/dimensions.md H).

Stack (dimensions H): FC on its own plate standoffs (z 9.5 to 17.5), Radxa below the props (component side up, USB-C tops
z -7), flow sensor below the Radxa (tape face z -15.5, lens face z -24.5). This part hangs from the three plate bolts and
holds the Radxa at its 4 holes (bosses UNDER the board, M2.5 self-tapping screws from above, DEFAULT) and the flow sensor
on a taped land under the Radxa's middle.
"""
import numpy as np
import manifold3d as mf

SEG = 48

# ---------------------------------------------------------------------------------------------------- frame / FC
PLATE_UNDER = 18.5        # plate underside, 3.5 plate (E, DEFAULT; 2.0 plate -> 20.0, only the posts change)
BOLT_NOSE = (45.0, 0.0)   # dimensions E (Spark 21:39: round holes)
BOLT_SIDE = (3.9, 31.0)
BOLT_HOLE_R = 1.65        # M3 clearance 3.3
FLANGE_R, FLANGE_T = 4.75, 2.5            # bolt pads under the plate (DEFAULT)
DRIVER_R = 4.5                            # nyloc room round each bolt (E)
NUT_ZONE = 7.0                            # nyloc 4 + bolt tail 2 + 1 (E)
WRENCH_HALF, WRENCH_T = 7.0, 4.5          # open-end 5.5 mm wrench, head ~14 wide x 3.5 thick (E)
FC_C = (1.9, 0.0)                         # FC centre (v1 position, P)
FC_HALF = 18.0                            # 36 x 36 x 8 (MicoAir)
FC_Z = (9.5, 17.5)                        # lowest point 9 under the plate (Spark, P)
FC_PLUG = 4.0                             # plug / wire room round the FC edges (E)

# -------------------------------------------------------------------------------------------------------- props
MOTORS = [(sx * 54.1, sy * 54.1) for sx in (1, -1) for sy in (1, -1)]      # P
PROP_R = 44.45                            # 3.5 in (P)
SWEEP = 4.0                               # blade tips z +-4 (C)
PROP_KEEP = 12.0                          # README / dimensions C: 12 from the disks and the sweep

# -------------------------------------------------------------------------------------------------------- Radxa
# board coords X along 65 (X 0 = SD end -> tail), Y along 30 (Y 0 = ports edge -> drone right), Z 0 = PCB top
RX0 = -32.5               # drone x = X + RX0
RY0 = -15.0               # drone y = Y + RY0
R_TOP = -7.0              # USB-C tops (DEFAULT: 3 under the tips)
R_PCB_TOP = R_TOP - 3.16  # -10.16
R_PCB_BOT = R_PCB_TOP - 1.6               # -11.76
R_BOT = R_PCB_BOT - 1.55                  # -13.31 bottom parts (C)
R_SD_BOT = R_PCB_BOT - 2.86               # -14.62 microSD slot, X 0 to 4.9, Y 7.8 to 22.3 (C)
R_HOLES_B = [(3.55, 3.60), (3.60, 26.45), (61.40, 3.60), (61.40, 26.50)]   # C
R_HOLES = [(X + RX0, Y + RY0) for X, Y in R_HOLES_B]
BOSS_R = 3.5              # bottom parts are >= 3.6 from every hole centre (STEP, C)
BOSS_LEN = 6.0
PILOT_R = 1.1             # M2.5 self-tapping into PETG (DEFAULT; heat-set would leave a 1.9 wall)
PILOT_DEPTH = 6.5
USB_X = [(37.1, 46.0), (49.6, 58.5)]      # board X of the two USB-C (C); camera on the nearer one, DEFAULT
HDMI_X = (9.2, 15.7)
HDR = dict(X=(7.05, 57.85), Y=(24.1, 29.2))                                # bare header holes (solder pads)

# -------------------------------------------------------------------------------------------------- flow sensor
LENS = (1.9, 0.0)         # DEFAULT (v6)
FB = dict(x=(LENS[0] + 6 - 22, LENS[0] + 6), y=(-6.0, 10.0))     # board 22 x 16, lens 6 from the nose and right edges (P)
TAPE_Z = -15.5            # tape face = seat bottom
SEAT_T = 2.0
SEAT_TOP = TAPE_Z + SEAT_T                # -13.5
LENS_Z = TAPE_Z - 9.0                     # -24.5 (tape 1 + board 8, P)
LAND_W = 3.0              # tape land on the board edge
LAND_OUT = 2.2            # land runs this far past the board edge (tabs stand on it)
TAB_T, TAB_DROP = 2.0, 3.0                # guard tabs, 3 below the tape face (v6)
TAB_GAP = 0.2

# ---------------------------------------------------------------------------------------------------- envelope
ENV = dict(x=(-40.0, 52.0), y=(-37.0, 37.0), z=(-24.0, PLATE_UNDER))   # floor -22 after the skin: 2.5 above the flow lens (DEFAULT 13 Oct, run e)
SKIN = 2.0


def box(x, y, z):
    return mf.Manifold.cube([x[1] - x[0], y[1] - y[0], z[1] - z[0]]).translate([x[0], y[0], z[0]])


def zcyl(x, y, z0, z1, r, seg=SEG):
    return mf.Manifold.cylinder(z1 - z0, r, r, seg).translate([x, y, z0])


def cyl(p0, p1, r):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0; L = float(np.linalg.norm(d)); u = d / L
    c = mf.Manifold.cylinder(L, r, r, SEG)
    ax = np.cross([0, 0, 1.0], u); sa = np.linalg.norm(ax)
    if sa > 1e-9:
        ax = ax / sa; ang = np.arctan2(sa, u[2])
        K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        R = np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K
        c = c.transform(np.hstack([R, np.zeros((3, 1))]).tolist())
    return c.translate(list(p0))


def U(lst):
    out = lst[0]
    for m in lst[1:]: out = out + m
    return out


def to_trimesh(m):
    import trimesh
    me = m.to_mesh()
    return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=True)


def bolts():
    return [BOLT_NOSE, (BOLT_SIDE[0], BOLT_SIDE[1]), (BOLT_SIDE[0], -BOLT_SIDE[1])]


# nose post (Spark 14 Oct 01:25: use the third hole, keep it out of the props): drawn, inside the 12 margin, >= 9.5 from
# the front disks in plan (C, checked by nose_clearance()); in front of the Radxa (x 32.5) by 1.0
NOSE_POST = dict(x=(33.5, 38.3), y=(-2.5, 2.5), z=(-21.0, PLATE_UNDER))
NOSE_MIN_CLEAR = 9.5


def nose_clearance(m=None):
    """smallest plan distance from the nose post's corners to the 4 prop disk edges (blade band z -4 to 4)."""
    xs, ys = NOSE_POST["x"], NOSE_POST["y"]
    return min(np.hypot(x - mx, y - my) - PROP_R for x in xs for y in ys for mx, my in MOTORS)


def pad_bolts_side():
    """bolts this part clamps: the two side bolts (DEFAULT 13 Oct, runs c to e). The nose bolt (45, 0) sits 11 inside the
    prop margin's reach from the Radxa: the only way down to the board passes a 3 wide wedge between the front disks, and
    every layout thinned it under 2 and lost the nose pad. The nose bolt keeps its nyloc room under the plate (slider)."""
    return [(BOLT_SIDE[0], BOLT_SIDE[1]), (BOLT_SIDE[0], -BOLT_SIDE[1])]


def pad_bolts():
    """bolts this part clamps: all three (14 Oct: nose pad back on a drawn post inside the prop margin, Spark)."""
    return [BOLT_NOSE] + pad_bolts_side()


# side posts through the blade band (DEFAULT 13 Oct): drawn, round, in the corridor between the FC box (|y| 22) and the
# disks (|y| <= 31 at |x| 2.5); clear of the nyloc room (7.2 from the bolt) and the camera plug (x >= 3.6)
POST_C = (0.0, 25.0)
POST_R = 2.5
POST_Z = (-9.0, PLATE_UNDER - FLANGE_T)


def blade(s, z0=POST_Z[0], z1=POST_Z[1] + 0.01):
    """side post as a fore-aft blade (13 Oct, run g: the Radxa pitches about the two-bolt line, so the posts need depth
    in x): 9.0 wide at y 22.6 (FC box + 0.6) tapering to 7.6 at y 26.0 (disks at |x| 5.1, nyloc room 5.0 off the bolt)."""
    pts = [(-4.5, 22.6), (4.5, 22.6), (3.8, 26.0), (-3.8, 26.0)]
    pts = [(x, s * y) for x, y in pts]
    if s < 0: pts = pts[::-1]
    m = mf.Manifold.extrude(mf.CrossSection([pts]), z1 - z0).translate([0, 0, z0])
    if s < 0:      # 0.2 off the camera plug room (x >= 3.6, z -13.66 to -5)
        m = m - box((3.4, 20.0), (-40.0, -16.0), (-14.0, -4.8))
    return m


def prop_keepout():
    """points within PROP_KEEP of any prop disk + sweep slab, as a stack of 1 mm cylinder slices (C)."""
    out = []
    for mx, my in MOTORS:
        sl = [zcyl(mx, my, -SWEEP, SWEEP, PROP_R + PROP_KEEP, 96)]
        for k in range(12):
            r = PROP_R + np.sqrt(max(PROP_KEEP ** 2 - (k + 1) ** 2, 0.0))
            sl.append(zcyl(mx, my, SWEEP + k, SWEEP + k + 1, r, 96))
            sl.append(zcyl(mx, my, -SWEEP - k - 1, -SWEEP - k, r, 96))
        out.append(U(sl))
    return U(out)


def radxa_body():
    """Radxa as a keep-out: board + parts (C), port overhang, SD slot."""
    b = [box((RX0, RX0 + 65), (RY0 - 1.75, RY0 + 30), (R_BOT, R_TOP + 0.5)),           # body, ports overhang 1.5 + 0.25
         box((RX0, RX0 + 4.9), (RY0 + 7.8, RY0 + 22.3), (R_SD_BOT, R_PCB_BOT))]          # microSD slot
    return U(b)


def sensor_body():
    return box(FB["x"], FB["y"], (LENS_Z, TAPE_Z))


def parts():
    """dict: drawn, cuts, keep (lists of manifolds), env, and named regions."""
    drawn, cuts, keep = [], [], []
    # bolt pads: top face on the plate underside
    nx0, nx1 = NOSE_POST["x"]; ny0, ny1 = NOSE_POST["y"]
    drawn.append(zcyl(BOLT_NOSE[0], BOLT_NOSE[1], PLATE_UNDER - FLANGE_T, PLATE_UNDER, FLANGE_R))
    drawn.append(box((nx0, BOLT_NOSE[0]), (ny0, ny1), (PLATE_UNDER - FLANGE_T, PLATE_UNDER)))      # link on the plate face
    drawn.append(box((nx0, nx1), (ny0, ny1), NOSE_POST["z"]))                                        # the post
    for bx, by in pad_bolts_side():
        drawn.append(zcyl(bx, by, PLATE_UNDER - FLANGE_T, PLATE_UNDER, FLANGE_R))
        py = POST_C[1] if by > 0 else -POST_C[1]
        drawn.append(blade(1 if by > 0 else -1))
        drawn.append(mf.Manifold.hull(blade(1 if by > 0 else -1, z0=PLATE_UNDER - FLANGE_T, z1=PLATE_UNDER) +
                                      zcyl(bx, by, PLATE_UNDER - FLANGE_T, PLATE_UNDER, FLANGE_R)))
    for bx, by in bolts():
        cuts.append(zcyl(bx, by, ENV["z"][0] - 5, PLATE_UNDER + 5, BOLT_HOLE_R))
        # nyloc + bolt tail under the pad, and an outboard slot for a 5.5 mm open wrench (DEFAULT 13 Oct: a nut-driver
        # column to the bottom split the only corridor past the props, run a)
        zb = PLATE_UNDER - FLANGE_T
        keep.append(zcyl(bx, by, zb - NUT_ZONE, zb, DRIVER_R))
        if by == 0:      # nose: wrench from the nose
            keep.append(box((bx, ENV["x"][1] + 5), (by - WRENCH_HALF, by + WRENCH_HALF), (zb - WRENCH_T, zb)))
        else:            # sides: wrench from outboard
            yo = (by, ENV["y"][1] + 5) if by > 0 else (ENV["y"][0] - 5, by)
            keep.append(box((bx - WRENCH_HALF, bx + WRENCH_HALF), yo, (zb - WRENCH_T, zb)))
    # Radxa bosses under the board, pilot holes from the top
    for hx, hy in R_HOLES:
        drawn.append(zcyl(hx, hy, R_PCB_BOT - BOSS_LEN, R_PCB_BOT, BOSS_R))
        cuts.append(zcyl(hx, hy, R_PCB_BOT - PILOT_DEPTH, R_PCB_BOT + 4.0, PILOT_R))
    # flow sensor land (ring on the board edge, running LAND_OUT past it) + 5 guard tabs
    x0, x1 = FB["x"]; y0, y1 = FB["y"]
    land = box((x0 - LAND_OUT, x1 + LAND_OUT), (y0 - LAND_OUT, y1 + LAND_OUT), (TAPE_Z, SEAT_TOP)) - \
        box((x0 + LAND_W, x1 - LAND_W), (y0 + LAND_W, y1 - LAND_W), (TAPE_Z - 1, SEAT_TOP + 1))
    g = TAB_GAP; t = TAB_T; zt = (TAPE_Z - TAB_DROP, TAPE_Z + 0.01)
    tabs = [box((x1 + g, x1 + g + t), (LENS[1] - 2.5, LENS[1] + 2.5), zt),                       # nose
            box((x0 + 6, x0 + 13), (y1 + g, y1 + g + t), zt),                                       # left
            box((x0 + 6, x0 + 13), (y0 - g - t, y0 - g), zt),                                       # right
            box((x0 - g - t, x0 - g), (y0, y0 + 3.5), zt),                                          # tail corners (plug
            box((x0 - g - t, x0 - g), (y1 - 3.5, y1), zt)]                                          #  exits between them)
    drawn += [land] + tabs
    # keep-outs
    fc = box((FC_C[0] - FC_HALF - FC_PLUG, FC_C[0] + FC_HALF + FC_PLUG), (-FC_HALF - FC_PLUG, FC_HALF + FC_PLUG),
             (R_TOP + 0.5, PLATE_UNDER + 1))                     # FC + plugs, and the wire room under it down to the Radxa
    # FC nose edge: its plugs sit outside |y| < 4 (v1 underside keep-outs, nose-edge plugs x 12-30 outside |y| < 4), so the
    # centre strip keeps only FC + 1 there: a channel for the nose post past the props (13 Oct, run c)
    fc = fc - box((FC_C[0] + FC_HALF + 1.0, FC_C[0] + FC_HALF + FC_PLUG + 1), (-3.5, 3.5), (R_TOP + 0.4, PLATE_UNDER + 2))
    keep.append(fc)
    keep.append(radxa_body())
    keep.append(sensor_body())
    keep.append(box((x0 - 10, x0), (y0 + 3.5 + TAB_GAP, y1 - 3.5 - TAB_GAP), (LENS_Z, TAPE_Z)))     # sensor plug + lead, between the tail tabs
    # camera USB-C plug (nearer USB-C, board X 37.1 to 46.0) out of the drone-right edge (E: plug 12 x 7, cable 25)
    ux0, ux1 = USB_X[0]
    keep.append(box((ux0 + RX0 - 1.0, ux1 + RX0 + 1.0), (RY0 - 30.0, RY0 - 1.75), (R_PCB_TOP - 3.5, R_TOP + 2.0)))
    keep.append(box((RX0 + USB_X[1][0] - 0.5, RX0 + USB_X[1][1] + 0.5), (RY0 - 6.0, RY0 - 1.75), (R_PCB_TOP - 1.0, R_TOP + 0.5)))
    # SD card in / out at the tail end
    keep.append(box((ENV["x"][0] - 5, RX0 + 0.5), (RY0 + 7.0, RY0 + 23.0), (R_SD_BOT - 0.5, R_PCB_TOP + 0.5)))
    # header-pad leads: pads on the top side, wires up to the FC past its tail end (board X 7 to 20 = pins 1 to 10, E)
    keep.append(box((RX0 + HDR["X"][0] - 1, RX0 + HDR["X"][1] + 1), (RY0 + HDR["Y"][0] - 1, RY0 + HDR["Y"][1] + 1),
                    (R_PCB_TOP, R_TOP + 0.5)))
    keep.append(box((RX0 + HDR["X"][0] - 1.5, RX0 + 21.0), (RY0 + HDR["Y"][0] - 1.5, RY0 + HDR["Y"][1] + 2.0),
                    (R_PCB_TOP, FC_Z[0])))
    # screw heads on the board top (M2.5 pan, head 4.5 x 1.8) and a screwdriver from above (board screwed on before the
    # part is bolted to the plate)
    for hx, hy in R_HOLES:
        keep.append(zcyl(hx, hy, R_PCB_TOP, PLATE_UNDER + 1, 3.0))
    # motor / ESC wires along the arm diagonals under the plate (E)
    for sx in (1, -1):
        for sy in (1, -1):
            keep.append(cyl((sx * 18, sy * 18, 14.5), (sx * 60, sy * 60, 14.5), 5.0))
    keep.append(prop_keepout())
    env = box(ENV["x"], ENV["y"], ENV["z"])
    return dict(drawn=drawn, cuts=cuts, keep=keep, env=env)


def skin():
    """empty skin at every envelope face except the top (the plate face is real)."""
    e = ENV; s = SKIN
    inner = box((e["x"][0] + s, e["x"][1] - s), (e["y"][0] + s, e["y"][1] - s), (e["z"][0] + s, e["z"][1] + 10))
    return box(e["x"], e["y"], e["z"]) - inner


# regions for loads (drawn coords)
def boss_tops():
    return [(hx, hy, R_PCB_BOT) for hx, hy in R_HOLES]


if __name__ == "__main__":
    g = parts()
    for k in ("drawn", "cuts", "keep"):
        print(k, len(g[k]), f"{U(g[k]).volume() / 1000:.2f} cm3")
    d = U(g["drawn"]) - U(g["cuts"])
    print(f"drawn net {d.volume()/1000:.3f} cm3 = {d.volume()/1000*1.27:.2f} g PETG")
    print("Radxa holes (drone):", [tuple(round(v, 2) for v in h) for h in R_HOLES])
    print("flow board x, y:", FB)
    allowed = g["env"] - U(g["keep"]) - U(g["cuts"]) - skin()
    print(f"allowed generated region {allowed.volume()/1000:.1f} cm3 of envelope {g['env'].volume()/1000:.1f}")
    # drawn bodies must not sit in a keep-out (except where they are meant to: none)
    clash = (d ^ U(g["keep"])).volume()
    print(f"drawn inside keep-outs: {clash:.2f} mm3")
    print(f"nose post to the prop disks (plan, blade band): {nose_clearance():.2f} mm (need {NOSE_MIN_CLEAR})")
