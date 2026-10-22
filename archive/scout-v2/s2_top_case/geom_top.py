"""S2 top case v1 geometry (Scout v2), 16 Oct 2026, top case chat.

Spark 16 Oct 19:46: one case on top of the plate for every electronic part except the flow sensor (stays at the bottom).
FC on its own grommets (not touched by the case); the case holds the Radxa and the UBEC (tape + zip ties).
Placement from s2t_place.py (prop keep-out 12, nothing inside a disk in plan): FC on the plate, UBEC on edge above it,
XR2 beside the UBEC, Radxa on top. Radxa hung under 4 boss columns, component side DOWN (DEFAULT: the boss tops are the
top contact when the drone lands upside down, so no roof; 1.5 lower than component-up plus a roof).

Drone coordinates (dimensions.md): x nose +, y left +, z up; plate top z 22; prop plane z 40.5 (E, D12), sweep +-4.
Tags: P part, C CAD / STEP, E estimate, DEFAULT chosen here.
"""
import numpy as np
import manifold3d as mf

SEG = 48

# ------------------------------------------------------------------------------------------------- frame / FC
PLATE_TOP = 22.0
BOLT_NOSE = (45.0, 0.0)                    # dimensions E (round holes)
BOLT_SIDE = (3.9, 31.0)
BOLT_HOLE_R = 1.65                         # M3 clearance 3.3
PAD_R, PAD_T = 4.75, 3.0                   # bolt pads on the plate top (DEFAULT; r 4.75 keeps 0.2 off the FC zone corner)
HEAD_R, HEAD_H = 3.0, 3.5                  # M3 socket head 5.5 x 3 + room (E)
# cage v2 side hubs (v2/s2-cage-v2.md): pad r 4.75 x 4 (z 22..26) on each side bolt under this case's pad (bolt 4
# longer), arm on the plate top out to the plate edge. Side pads therefore sit on z 26 (nose pad stays on the plate)
HUB_T = 4.0
PAD_Z0 = {"nose": PLATE_TOP, "side": PLATE_TOP + HUB_T}
KEY_R = 2.0                                # 2.5 hex key from above (E)
FC_C = (1.9, 0.0)                          # FC centre, diamond (P)
FC_L1 = 25.46 + 1.0                        # 36 square turned 45 deg: half diagonal 25.46 (spec) + 1 grommet play (E)
FC_Z = (PLATE_TOP, 37.0)                   # stack 10 tall on its grommets (Spark 17 Oct 11:35, P) + plug room 5 (E)
FC_USB = dict(x=(-50.0, -20.0), y=(-13.0, 3.0), z=(23.0, 33.0))       # USB-C at the tail corner, slightly right (P / E)
BATT_LANE = dict(x=(-54.0, -36.0), y=(-8.0, 8.0), z=(PLATE_TOP, 36.0))  # pack leads up through the XT60 cut-out (E)

# -------------------------------------------------------------------------------------------------------- props
MOTORS = [(sx * 54.1, sy * 54.1) for sx in (1, -1) for sy in (1, -1)]     # P
PROP_R = 44.45                             # P
PROP_Z = 41.2                              # plate top 22 + D12 19.2 (dimensions C, closed 17 Oct 00:52, E)
SWEEP = 4.0                                # C
PROP_KEEP = 12.0                           # dimensions C

# -------------------------------------------------------------------------------------------------------- Radxa
# board coords (Radxa STEP v1.11, C): X along 65, Y along 30, Z 0 = component side. Hung component side DOWN, ports edge
# (Y 0) to the nose, SD end (X 0) to the left: drone x = 15 - Y, y = 32.5 - X
def bxy(X, Y):
    return (15.0 - Y, 32.5 - X)

R_LOW = 59.0                               # USB-C faces (lowest point): UBEC top 56 + 3 heat gap (s2t_place, C)
R_COMP = R_LOW + 3.16                      # component face 62.16
R_BACK = R_COMP + 1.6                      # back face 63.76
R_BACK_PARTS = R_BACK + 1.55               # back-side parts 65.31 (microSD slot 2.86 -> 66.62)
R_HOLES = [bxy(*h) for h in [(3.55, 3.60), (3.60, 26.45), (61.40, 3.60), (61.40, 26.50)]]
BOSS_R = 3.5                               # back parts >= 3.6 from every hole (C); SD slot 0.65 off the +x bosses
BOSS_TOP = R_BACK_PARTS + 0.3 + 2.5       # 68.11: boss tops = top contact; 2.5 over the back parts so members can
                                           # cross the board back (DEFAULT)
PILOT_R, PILOT_DEPTH = 1.1, 3.2            # M2.5 self-tapping from below through the board (DEFAULT, underside v2)
USB_HOST_X, USB_OTG_X, HDMI_X = (37.1, 46.0), (49.6, 58.5), (9.2, 15.7)
SD = dict(X=(0.0, 4.9), Y=(7.8, 22.3))
HDR = dict(X=(7.05, 57.85), Y=(24.1, 29.2))

# ----------------------------------------------------------------------------------------------- UBEC, XR2, land
UBEC = dict(x=(-7.0, 3.0), y=(-22.0, 22.0), z=(37.0, 57.0))           # body 44 x 20 (Spark 17 Oct: 55 with its
                                           # pins, pins clipped, leads soldered, DEFAULT) x 10 (E) on edge; 2 under the Radxa
LAND = dict(x=(3.0, 5.0), y=(-20.0, 20.0), z=(40.0, 54.0))            # tape land, 2.0 (DEFAULT)
XR2 = dict(x=(5.0, 10.0), y=(-11.0, 11.0), z=(40.5, 53.5))            # 22 x 13 x 5 incl. tape, other face of the land (E)
STRAPS = [(40.5, 44.0), (50.5, 54.0)]
# land ears (run d, 17 Oct): run c's only link to the land ran under it and was pruned (UBEC press 3.8e4 x after).
# The straps wrap UBEC + land + XR2, so the land can only be reached between the strap bands: ears out past the UBEC
# ends there, 0.5 clear of both bands (keep-out 12.1 at the tip, C)
EARS = dict(x=(3.0, 5.0), y=(20.0, 26.0), z=(44.5, 50.0))      # two zip ties round UBEC + land + XR2 (z bands, tie <= 3.6 wide, DEFAULT)
STRAP_T = 1.6

# side posts (run b, 17 Oct): run a's legs past the FC corners came out under 2.0 and were pruned, cutting the bosses off.
# The corridor between the FC zone and the 12 keep-out at the side bolts is ~4 x 3 (C), so the posts are drawn (as the
# underside's blades, 13 Oct). Clear of the FC zone (L1 >= 26.6), the keep-out (>= 12.2) and the bolt head (x <= 0.8).
POST = dict(x=(-3.0, 0.8), y=(26.6, 29.6), z=(PLATE_TOP, 40.0))
# nose post (run c, 17 Oct): run b left the nose pad detached (the corridor between the FC nose corner and the keep-out
# is x 26.4..32.4 at |y| 2, C). Drawn as the underside's (Spark 14 Oct: nose pad back on a drawn post): a strip on the
# plate from the pad back to a 3.7 x 4 post, FC zone L1 >= 26.7, keep-out >= 12.03 (C)
NOSE_STRIP = dict(x=(28.6, BOLT_NOSE[0]), y=(-2.0, 2.0), z=(PLATE_TOP, PLATE_TOP + PAD_T))
NOSE_POST = dict(x=(28.6, 32.3), y=(-2.0, 2.0), z=(PLATE_TOP, 40.0))

# ---------------------------------------------------------------------------------------------------- envelope
ENV = dict(x=(-45.0, 50.0), y=(-42.0, 42.0), z=(PLATE_TOP, BOSS_TOP))


def box(x, y, z):
    return mf.Manifold.cube([x[1] - x[0], y[1] - y[0], z[1] - z[0]]).translate([x[0], y[0], z[0]])


def B(d):
    return box(d["x"], d["y"], d["z"])


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


def pad_z0(by):
    return PAD_Z0["nose"] if by == 0 else PAD_Z0["side"]


def bolts():
    return [BOLT_NOSE, BOLT_SIDE, (BOLT_SIDE[0], -BOLT_SIDE[1])]


def bbox_xy(X, Y):
    """board rectangle (board coords) -> drone x, y ranges"""
    xs = [15.0 - y for y in Y]; ys = [32.5 - x for x in X]
    return (min(xs), max(xs)), (min(ys), max(ys))


def fc_zone():
    """FC diamond prism + plug room (L1 ball turned to a square at 45 deg)."""
    s = FC_L1 * np.sqrt(2)
    sq = mf.Manifold.cube([s, s, FC_Z[1] - FC_Z[0]]).translate([-s / 2, -s / 2, FC_Z[0]])
    return sq.rotate([0, 0, 45]).translate([FC_C[0], FC_C[1], 0])


def prop_keepout(zmax=ENV["z"][1] + 5):
    """within PROP_KEEP of a prop disk + sweep slab (1 mm slices), plus the disk itself from the plate up (airflow)."""
    out = []
    z0, z1 = PROP_Z - SWEEP, PROP_Z + SWEEP
    for mx, my in MOTORS:
        sl = [zcyl(mx, my, z0, z1, PROP_R + PROP_KEEP, 96), zcyl(mx, my, PLATE_TOP - 1, zmax, PROP_R, 96)]
        for k in range(12):
            r = PROP_R + np.sqrt(max(PROP_KEEP ** 2 - (k + 1) ** 2, 0.0))
            sl.append(zcyl(mx, my, z1 + k, z1 + k + 1, r, 96))
            sl.append(zcyl(mx, my, z0 - k - 1, z0 - k, r, 96))
        out.append(U(sl))
    return U(out)


def radxa_body():
    """Radxa as a keep-out (C): board + parts, ports and their overhang, SD slot + card."""
    xr, yr = bbox_xy((0, 65), (-1.75, 30))
    b = [box(xr, yr, (R_LOW - 0.3, R_BACK_PARTS + 0.3)) - U([zcyl(hx, hy, R_BACK, BOSS_TOP + 1, BOSS_R) for hx, hy in R_HOLES])]
    xr, yr = bbox_xy(SD["X"], SD["Y"])
    b.append(box(xr, (yr[0], yr[1] + 2.5), (R_BACK, R_BACK + 2.86 + 0.3)))      # slot + card past X 0 (E)
    return U(b)


def strap_lanes():
    core = [B(UBEC), B(LAND), B(XR2)]
    out = []
    for z0, z1 in STRAPS:
        inner = mf.Manifold.hull(U([box(c["x"], c["y"], (z0, z1)) for c in (UBEC, LAND, XR2)]))
        grow = [box((c["x"][0] - STRAP_T, c["x"][1] + STRAP_T), (c["y"][0] - STRAP_T, c["y"][1] + STRAP_T), (z0, z1))
                for c in (UBEC, LAND, XR2)]
        out.append(mf.Manifold.hull(U(grow)) - inner)
    return U(out)


def parts():
    """dict: drawn, cuts, keep (lists of manifolds), env."""
    drawn, cuts, keep = [], [], []
    for bx, by in bolts():
        z0 = pad_z0(by)
        drawn.append(zcyl(bx, by, z0, z0 + PAD_T, PAD_R))
        cuts.append(zcyl(bx, by, PLATE_TOP - 5, ENV["z"][1] + 5, BOLT_HOLE_R))
        keep.append(zcyl(bx, by, z0 + PAD_T, z0 + PAD_T + HEAD_H, HEAD_R))  # bolt head
        keep.append(zcyl(bx, by, z0 + PAD_T, ENV["z"][1] + 5, KEY_R))       # hex key from above
        if by != 0:                                                         # cage side hub: pad + arm to the plate edge
            keep.append(zcyl(bx, by, PLATE_TOP - 1, z0, PAD_R + 0.5))
            yo = (by, 48.0) if by > 0 else (-48.0, by)
            keep.append(box((bx - PAD_R - 0.5, bx + PAD_R + 0.5), yo, (PLATE_TOP - 1, z0)))
    for hx, hy in R_HOLES:
        drawn.append(zcyl(hx, hy, R_BACK, BOSS_TOP, BOSS_R))
        cuts.append(zcyl(hx, hy, R_BACK - 5.0, R_BACK + PILOT_DEPTH, PILOT_R))
        keep.append(zcyl(hx, hy, R_LOW - 6.0, R_BACK, 3.2))                           # screw head + driver from below (E)
    for sy in (1, -1):
        py = tuple(sorted((sy * POST["y"][0], sy * POST["y"][1])))
        z0 = pad_z0(sy)
        drawn.append(box(POST["x"], py, (z0, POST["z"][1])))
        drawn.append(mf.Manifold.hull(box(POST["x"], py, (z0, z0 + PAD_T)) +
                                      zcyl(BOLT_SIDE[0], sy * BOLT_SIDE[1], z0, z0 + PAD_T, PAD_R)))
    drawn += [B(NOSE_STRIP), B(NOSE_POST)]
    drawn.append(B(LAND))
    for sy in (1, -1):
        drawn.append(box(EARS["x"], tuple(sorted((sy * EARS["y"][0], sy * EARS["y"][1]))), EARS["z"]))
    # keep-outs
    keep += [fc_zone(), B(FC_USB), B(BATT_LANE), radxa_body(), B(UBEC), B(XR2), strap_lanes(), prop_keepout()]
    # ports on the nose edge (component side, now facing down): camera 90 deg USB-C plug (E 12) and the OTG plug room
    for X, out in ((USB_HOST_X, 13.0), (USB_OTG_X, 6.0), (HDMI_X, 4.0)):
        xr, yr = bbox_xy(X, (-1.75, 0))
        keep.append(box((xr[1], xr[1] + out), (yr[0] - 1.0, yr[1] + 1.0), (R_LOW - 4.0, R_COMP + 0.5)))
    # header pads (tail edge): leads soldered on the component side, down to the FC and UBEC (pins 1..10, E lane)
    xr, yr = bbox_xy(HDR["X"], HDR["Y"])
    keep.append(box((xr[0] - 1.0, xr[1] + 1.0), (yr[0] - 1.0, yr[1] + 1.0), (R_LOW - 1.0, R_COMP)))
    xr, yr = bbox_xy((HDR["X"][0], 21.0), HDR["Y"])
    keep.append(box((xr[0] - 1.5, xr[1] + 1.5), (yr[0] - 1.5, yr[1] + 1.5), (FC_Z[1] - 4.0, R_COMP)))
    # SD card in / out at the left end
    xr, yr = bbox_xy(SD["X"], SD["Y"])
    keep.append(box((xr[0] - 1.0, xr[1] + 1.0), (yr[1], ENV["y"][1] + 5), (R_BACK - 0.5, R_BACK + 3.5)))
    # motor leads from the FC edges along the arm diagonals, on the plate (E)
    for sx in (1, -1):
        for sy in (1, -1):
            keep.append(cyl((FC_C[0] + sx * 13, sy * 13, 25.0), (sx * 60, sy * 60, 25.0), 3.5))
    env = box(ENV["x"], ENV["y"], ENV["z"])
    return dict(drawn=drawn, cuts=cuts, keep=keep, env=env)


if __name__ == "__main__":
    g = parts()
    d = U(g["drawn"]) - U(g["cuts"])
    print(f"drawn net {d.volume()/1000:.3f} cm3 = {d.volume()/1000*1.27:.2f} g PETG")
    print("Radxa holes (drone):", [tuple(round(v, 2) for v in h) for h in R_HOLES])
    allowed = g["env"] - U(g["keep"]) - U(g["cuts"])
    print(f"allowed generated region {allowed.volume()/1000:.1f} cm3 of envelope {g['env'].volume()/1000:.1f}")
    print(f"drawn inside keep-outs: {(d ^ U(g['keep'])).volume():.2f} mm3")
    # each drawn feature must reach the allowed region (else it can never be tied in)
    for i, m in enumerate(g["drawn"]):
        b = m.bounding_box(); grow = box((b[0] - 1, b[3] + 1), (b[1] - 1, b[4] + 1), (b[2] - 1, b[5] + 1)) - m
        print(f"  drawn {i}: {m.volume():.0f} mm3, free shell 1 mm round it {(grow ^ allowed).volume():.0f} mm3")
