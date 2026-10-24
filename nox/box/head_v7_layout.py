"""Heads A / B v7 layout (24 Oct 2026, heads v7 part session): the system pass's head requirements turned into one head layout,
the concept check's numbers and its picture. Head A frame (as arm_kinematics): X tangential (the blades' travel, + = the rack side),
Y along the axle, u radial from the head's z0 datum; head B is the mirror image in X with the MG995. Y_C 165 = the cartridge's tab line;
the carried cartridge's flat bottom at u 15.4. Cartridge coords map X = x, Y = Y_C + z_T - z, u = 15.4 + y (latch PR 69).

Mechanism (refines the system pass's grabber concept, head_v7_concept.py; the changes are listed in CHANGES):
  - two carriers (PETG), each a blade (3 x 10, Y 160..170, rib 0.8 into the tab's grip pocket) on a boss 12 long sliding on two 3.0
    carbon rods along X (Y 161 / 169, u 6.75) fixed in the head body's walls: the 50..57 N pull goes pocket -> rib -> blade -> boss ->
    rods -> body;
  - ONE steel compression spring crossed between them (the +X carrier's arm seats its -X end, the -X carrier's arm its +X end, on a
    third 3.0 rod at Y 181.25): it closes both blades with the same force (15 N each at squeezed) and its force never reaches the body;
  - two inward stops on the body (the bosses meet them with the blades at inner 4.8): a load across the carried cartridge goes into
    the body after at most 1.2 of travel, never into the cam or the servo spline; the latch's +-1.0 head offset closes inside them;
  - the servo (DS3218 on A, MG995 on B) lies under the grip with its shaft along Y at (X 0, u -2.5); a clamping metal disc horn carries
    a printed cam (base 4.0, lobe 10.5, two-fold) that pushes two 623ZZ roller followers on the carriers apart (open) and stands clear
    of them at rest (gap 1.5; the spring holds, the servo is unloaded, the grip holds unpowered); the two roller forces cancel on the
    spline (pure torque); servo, horn and cam come out together toward the axle (-u);
  - the head body is one print with the carriage (plate top 74.35, wing, the rack web, wheel bores): drawn interfaces (wheel bores,
    deck with push pads, rod walls, spring rod walls, the servo cradle, the stops, hall pockets) joined by its topology run
    (head_v7_topo.py, built on its side, +x); the m1 rack is its own bar, printed flat and epoxied to the web.
python3 head_v7_layout.py -> runs/head/v7/layout/ (layout.json, layout.html: the concept picture, sections to scale) + the checks.
Numbers C geometry, E forces, stiffnesses and bought-part dimensions not measured (tagged in the json)."""
import os, sys, math, json
import numpy as np, trimesh, manifold3d as mf
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(ROOT, "runs", "head", "v7", "layout"); os.makedirs(OUT, exist_ok=True)
from arm_cad import box, cylX, cylY, cylZ, hull, tm, mirror_x

# ============================================================== system inputs (dimensions "System box swap: arm" / "latch"; C unless tagged)
Y_C = 165.0
U_BOT = 15.4                                    # carried cartridge flat bottom (gripper datum)
PLATE = dict(x=(-28.0, 28.0), y=(68.75, float(os.environ.get("NOX_PLATE_TOP", "74.35"))), u=(0.0, 40.0))   # top 74.35 under the cartridge: DEFAULT, asked of the arm system (concept review r1; alt 72.75)
WING = dict(x=(28.0, 46.5), u=(0.0, 30.0))
# the rack tail's backing rib (concept review r3: the pinion's separating force 22.1 N at u -56.3 bends the tail; E): Y 63..68.75 over the
# axle belt (Y <= 62), X out to the arm's free space (space map, >= 1.0): head A clear to X 28.5 the whole tail, head B beside the hub
# and the detent at park; stops at u -11.5, clear of the near wheel
TAIL_RIB = dict(   # (polygon X, u; Y range) per head; free space from the per-head space map (>= 1.0)
    A=[([(31.5, -62.0), (39.6, -62.0), (39.6, -11.5), (29.5, -11.5), (29.5, -59.9)], (63.0, 68.76)),                  # over the belt (X + u >= -30.4)
       ([(29.5, -37.0), (36.5, -40.5), (39.6, -40.5), (39.6, -11.5), (29.5, -11.5)], (68.75, 74.35)),    # up to the plate top (the detent arm at u -46 at park)
       ([(33.5, -31.0), (39.6, -31.0), (39.6, -11.5), (29.5, -11.5), (29.5, -25.0)], (59.25, 63.01))],                 # the root, under Y 63
    B=[([(37.5, -52.0), (39.6, -52.0), (39.6, -11.5), (32.5, -11.5), (32.5, -17.2), (35.5, -17.2), (35.5, -31.5), (37.5, -33.5)], (63.0, 68.76)),
       ([(32.5, -31.0), (39.6, -31.0), (39.6, -11.5), (32.5, -11.5)], (59.25, 63.01))])    # B: the detent arm and roller sit above / beside it at park
# the servo retainer (r4: the ear screws had no driver path on the arm): a printed bar under the servo's bottom face (u -16.3..-12.5,
# flush with the column bottom on the head lane) from (-4, 90) to (27, 148.5) (r8: the -Y end moved off the JR pair's band), Ø12 ends with M3 countersunk screws driven along +u
# through the exit window into heat-set inserts in drawn Ø12 bosses; the ears sit captured in their slots on the pads
RETAINER = dict(ends=((-4.0, 90.0), (27.0, 148.5)), w=10.0, u=(-16.3, -12.5), end_r=6.0, boss_r=6.0, insert=dict(d=4.0, depth=6.7))
FILLET = [(28.0, 30.0), (40.5, 30.0), (40.5, 32.5), (28.0, 40.0)]   # plate / wing inside corner (bound r3: the pull peak sat in the sharp corner)
WEB = dict(x=(40.5, 46.5), body_x=(39.5, 41.5), bar_x=(41.5, 46.5), y=(59.25, 72.75), uA=(-63.0, 30.0), uB=(-54.0, 30.0), teeth_y=(59.25, 73.25), tail_u=-39.0, tail_y0=63.0)
WHEELS = ((22.0, 4.0), (22.0, 32.0), (-22.0, 18.0))      # (X, u); the last is the eccentric
WHEEL = dict(od=24.0, y=(57.75, 67.75), nut_y=(55.05, 57.75), shim_y=(67.75, 68.75), axle_r=2.6, ecc_bore_r=5.6, ecc_boss_r=8.6,
             ecc_flange=(70.25, 72.75), csk_r=5.2)
BUMPER = dict(x=(-10.0, 10.0), u=(0.0, 5.0))
KEEPOUT_DRONE = dict(x=(-47.5, 47.5), y=(75.35, 217.0), Z=(95.0, 159.6))    # head + grabber at the drone (W Z = z0 + u); PR 102 (arm, 24 Oct)
Z0 = dict(park=32.0, bay_cmd=101.8, insert_cmd=111.3)
LATCH = dict(recess_x=15.5, recess_dy=11.5, recess_h=12.0, tab_latched=(4.0, 10.0), tab_squeezed=(1.0, 7.0), tab_dy=11.0, tab_h=11.5,
             pocket=dict(y=4.0, h=4.0, w=8.0, d=3.0, wall_deg=5.0), bar_y0=6.5, release_N=dict(scout=3.0, overseer=4.5),
             hold_N=dict(scout=52.1, overseer=57.3), tab_spring_sq=2.4, head_offset=1.0, hall=dict(x=7.5, dz=8.0, gap=1.0),
             flat_rim=dict(scout=(15.0, 19.0), overseer=(22.5, 26.5)), frames=(15.5, 19.0))
LOADS = dict(insert=60.0, stall=74.0, pull=50.0, pull_unmate=57.3, handling_g=(3.0, 2.0), cart_g=dict(scout=210.8, overseer=748.2),
             cart_cg=dict(scout=(0.0, 26.2, 0.0), overseer=(0.0, 33.1, -5.18)))   # cg: (x, y, z - z_T) cartridge coords
TARGET = dict(swap=0.5, misuse=1.0, handling=1.0, cyclic_MPa=10.0, cycles=dict(drone=10000, bay=5000))

# ============================================================== bought parts (E unless measured)
SERVOS = {   # body L x W, base-to-case-top H, spline boss d x h, shaft from the near end, ears span / thickness / underside from base
    # dimensions.md M (spec): DS3218 40 x 20 x 40.4, ears 4 x Ø4.5 on 49.5 x 10, 54.5 over the ears, ear underside 27.7 above the base;
    # MG995 40.9 x 20, 37 to the case top, 54 over the ears, ear underside 26.8 (its ear holes open, Q12; shaft position open, Q11).
    # The cradle is parametric: the case top stays at Y 138, the ears land where the servo puts them. CHECK the DS3218 on hand (P).
    "DS3218": dict(L=40.0, W=20.0, H=40.4, boss=(13.0, 4.0), shaft=10.0, ear_span=54.5, ear_t=2.9, ear_y=27.7, holes=(49.5, 10.0),
                   kgcm=20.0, g=60.0, lead=dict(y=(1.0, 6.0), w=6.0, t=3.0), tag="spec (dimensions.md M); shaft offset, ear thickness, lead exit E"),
    "MG995": dict(L=40.9, W=20.0, H=37.0, boss=(13.0, 4.0), shaft=10.2, ear_span=54.0, ear_t=2.6, ear_y=26.8, holes=(49.0, 10.0),
                  kgcm=9.0, g=55.0, lead=dict(y=(1.0, 6.0), w=6.0, t=3.0), tag="spec (dimensions.md M); ear holes, shaft offset, lead exit E"),
}
HORN = dict(hub_d=10.0, hub_h=3.5, disc_d=25.0, disc_t=3.0, bolt_circle=16.0, tag="metal 25T disc horn, E (CHECK on hand)")
ROD_D = 3.0
B623 = dict(od=10.0, id=3.0, w=4.0, g=1.6)        # 623ZZ roller followers (C, catalogue)
HALL = dict(pkg=(2.4, 2.9, 1.3), board=(4.2, 3.2, 1.6), tag="DRV5055A4 SOT-23 (long side along Y) on a 4.2 x 3.2 strip at Y 155.6..158.8 (r8: past the cam, 1.0 off the carriers and the blade), E")

# ============================================================== layout (DEFAULT, this session)
Y_SERVO_TOP = 138.0                              # case top; spline boss to 142, horn hub 142..145.5, disc 145.5..148.5, cam 148.5..153.5
U_C = -2.5                                       # servo shaft / cam / roller axis level
CAM = dict(y=(148.5, 154.5), base=4.0, lobe=10.5, rise_deg=100.0, dwell0=10.0, dwell1=10.0, law="modified sine")
ROLLER = dict(x_sq=10.5, y=(149.5, 153.5), pin_y=(149.5, 158.3), collar=(154.5, 155.6))   # the 3.0 carbon pin: flush with the roller (inner race epoxied
                                                  # on it), 3.8 into the lug through a Ø7 collar past the cam's end (r7: it was held 2.0)   # roller centre X at squeezed; open + 5.0
BLADE = dict(t=3.0, y=(160.0, 170.0), inner_sq=6.0, inner_open=11.0, tip_u=U_BOT + 6.0, rib=dict(d=0.8, u=U_BOT + 4.0))
STROKE = BLADE["inner_open"] - BLADE["inner_sq"]  # 5.0 per blade (the mechanism's datum: inner 6.0)
CLAMP = 7.0                                       # blade inner face on the squeezed tabs of cartridge v6 (latch 24 Oct: a 2.0 centre rib, tabs stop
                                                  # at +-1.0, outer faces +-7.0; DEFAULT): the carriers clamp 1.0 out from the datum
DECK = dict(u=(12.9, 14.9), x=18.2, y=(150.0, 180.0), pad=dict(x=(16.0, 18.2), y=(155.0, 175.0), u_top=U_BOT), slot=(3.8, 15.5, 159.0, 171.0))   # the blades' travel + 1.0 (r6); the hall's opening joins the slot; |X| <= 18.2: 1.0 off the Scout v6 TPU strips
RODS = dict(y=(161.0, 169.0), u=6.75, wall_in=29.0, wall_out=32.0)
BOSS = dict(x_sq=(11.0, 23.0), y=(155.85, 174.15), u=(1.6, 11.9))   # 3.2 walls round the rod bores (the gate's 1.5 neck erosion)
SPRING_ROD = dict(y=181.25, u=4.5, wall_in=29.0, wall_out=31.0, arm_w=8.0)
SEAT_SQ = 22.0                                   # spring seats at X -+22 at squeezed: L 44 squeezed, 34 open
STOP = dict(inner=4.8, y=(171.0, 174.5), u=(2.0, 7.5), t=2.0, post_u=-12.0)   # inward stops for the bosses at blade inner 4.8 (concept review r2: the
                                                 # across loads go to the body; the latch's +-1.0 head offset closes inside them)
HALL_POS = [(s * LATCH["hall"]["x"], Y_C - LATCH["hall"]["dz"]) for s in (-1, 1)]
PARK_LINE = -32.0                                # X + u = -32: the bisector between the parked heads (head frame, both heads alike)
CLR = 1.0
MOTION_URL = "https://claude.ai/artifact/F8ukGCJC6oA8hBZ7Tbnr7N"            # motion_A.html, published
PAGE_URL = "https://claude.ai/artifact/GJxsrPPAbaLkysyrSuJDWB"              # layout.html, published

CHANGES = [
    "carriage plate top Y 72.75 -> 74.35 under the cartridge (accepted by the arm system 24 Oct, with M5 x 20 axles): the upper-bound FE of "
    "the whole design space reads 0.73..1.03 at 72.75; the plate and the column's root also grow below u 0 where the arm leaves room "
    "(to u -17.2 on +X, 1.0 over the detent arm; the park bisector on -X)",
    "insert-lane keep-out at the drone, lower bound Z 111.5 -> 95.0 (head and grabber) and Z 100 -> 89 (carriage lane), accepted by the "
    "arm system 24 Oct (servo bottom u -12.5, horn u -15 -> Z 96.3 at the commanded insert): nothing of the catch (Z >= 154) or the "
    "structure is there",
    "open blade position inner faces +-11.0 (concept 10.5): a 0.8 rib into the grip pocket must clear the latched tab's outer face (10) on the "
    "way in (the latch's +-11 open; the tab's 1.0 chamfer cams a 1.0 head offset); stroke 5.0 per blade",
    "one crossed spring (15 N squeezed per blade, L 44 -> 34) instead of one spring per blade: the -X blade has 13 mm to the park bisector "
    "(the two parked heads), where a 15 N spring would need > 950 MPa (alternatives)",
    "two inward stops on the body take a load across the cartridge (43.5 N, receiver gaps closed); the system pass had no X path",
    "blades 10 wide (Y 160..170; the latch's 13): 1.5 clear of the hall package at Y 157 (the tab magnet's spot, z_T + 8); the rib (7.7 wide) "
    "is unchanged",
    "623ZZ roller followers on the carriers (no sliding at the cam): the servo needs about 2.5 kg.cm peak (concept 1.98 average with "
    "mu 0.3 on 19.5 N)",
    "the m1 rack is its own bar (printed flat, teeth in the layers, epoxied to the web): the body prints on its side (+x), where the "
    "column's bending and the plate's lie in the layers; the bar is bonded in the FE and the pinion holds its teeth",
    "r3: the rack tail's backing rib (the pinion's separating force 22.1 N bends the tail: index ~1.5 at the root without it, E); the teeth "
    "hold u only, the separating force is a load per case; the detent keep-out clipped to X < 39.5 (the bar and web keep 1.16)",
    "r3 / r4: the servo leaves along -u through a window under its footprint (commanded OPEN first); a printed retainer bar under it, "
    "2 M3 countersunk along +u into heat-set inserts (no ear screws: no driver path on the arm); the column bottom on the head lane u -16.3; "
    "every service path swept along the cycle against the arm and the head's own parts (head_v7_access.py)",
    "the body's -X flank is cut flat at X -34 with a 2 mm land (the +x build's bed face, 7.4 x 113); a 12.5 x 10 fillet joins the plate and "
    "the wing (the pull's peak sat in that sharp corner)",
]


# ============================================================== cam (synthesis), spring, servo torque
def ms(x):
    """modified sine rise 0..1 over x 0..1"""
    x = min(max(x, 0.0), 1.0)
    if x <= 0.125: return 0.43990085 * (x - math.sin(4 * math.pi * x) / (4 * math.pi))
    if x <= 0.875: return 0.28004957 + 0.43990085 * x - 0.31505577 * math.cos(4 * math.pi * x / 3 - math.pi / 6)
    return 0.56009915 + 0.43990085 * x - 0.0350062 * math.sin(4 * math.pi * x)


def pitch(theta_deg):
    """roller-centre radius (pitch curve) vs the cam turn from rest, two-fold (period 180): base + roller, a dwell, the rise, the open
    dwell to 140 (the roller stops at 120, its disc spans ~18 deg), a fall 140..152 the rollers never touch, base to 180 (at rest the
    roller's disc covers -28..28 deg: the profile there stays at base)"""
    p0 = CAM["base"] + B623["od"] / 2; rise = CAM["lobe"] - CAM["base"]; t = theta_deg % 180.0
    if t <= CAM["dwell0"] + CAM["rise_deg"] + 30.0:
        return p0 + rise * ms((t - CAM["dwell0"]) / CAM["rise_deg"])
    if t <= 152.0:
        return p0 + rise * 0.5 * (1 + math.cos(math.pi * (t - 140.0) / 12.0))
    return p0


def spring_pick():
    """a buyable music-wire spring on the 3.0 rod (ID >= 3.3): F >= 15 N at L 44, solid <= L 34 - 2, tau <= 0.45 UTS at open; least F open"""
    Lsq, Lop = 2 * SEAT_SQ, 2 * SEAT_SQ - 2 * STROKE
    best = None
    for d in (0.6, 0.7, 0.8):
        for OD in (4.6, 4.8, 5.0, 5.2, 5.5):
            D = OD - d
            if D - d < 3.3: continue
            for L0 in (55.0, 60.0, 65.0, 70.0):
                for n in range(15, 60):
                    k = 79000.0 * d ** 4 / (8 * D ** 3 * n)
                    Fsq, Fop = k * (L0 - Lsq), k * (L0 - Lop)
                    solid = (n + 2) * d; pitch_ = (L0 - 2 * d) / n
                    if Fsq < 15.0 or solid > Lop - 2.0 or not (1.4 * d <= pitch_ <= 4.0 * d): continue
                    C = D / d; Kw = (4 * C - 1) / (4 * C - 4) + 0.615 / C
                    tau = 8 * Fop * D * Kw / (math.pi * d ** 3); uts = 2211.0 * d ** -0.145
                    if tau > 0.45 * uts: continue
                    Lcl = Lsq - 2 * (CLAMP - BLADE["inner_sq"])                 # crossed: the carriers apart compress it
                    cand = dict(d=d, OD=OD, ID=round(D - d, 2), L0=L0, n=n, k=round(k, 3), F_sq=round(Fsq, 1), F_open=round(Fop, 1),
                                F_clamp=round(k * (L0 - Lcl), 1), L_clamp=Lcl,
                                solid=round(solid, 1), L_sq=Lsq, L_open=Lop, tau_open=round(tau, 0), tau_ratio=round(tau / (0.45 * uts), 2),
                                C=round(C, 2), note="music wire, closed ends, on the 3.0 carbon rod (guided: L0/D > 4)")
                    if best is None or (cand["F_open"], cand["tau_ratio"]) < (best["F_open"], best["tau_ratio"]): best = cand
    return best


def rack_factor(mu=0.3):
    """the cam force over the spring force needed to slide a carrier against its own guide friction (r5): the cam acts at the roller
    (Y 151.5, u -2.5), the spring at the seat (Y 181.25, u 4.5); the couples go into the guides as cross forces over the guide length,
    the boss on the two rods to the seat's bore on the spring rod (X 17 to -24: 41); yaw and pitch offsets added (conservative)"""
    a = (BOSS["x_sq"][0] + BOSS["x_sq"][1]) / 2; Lg = a + SEAT_SQ + 2.0
    e_yaw = SPRING_ROD["y"] - (ROLLER["y"][0] + ROLLER["y"][1]) / 2; e_pitch = SPRING_ROD["u"] - U_C
    return 1.0 + 2 * mu * (e_yaw + e_pitch) / Lg


def cam_numbers(sp):
    th = np.linspace(0, CAM["dwell0"] + CAM["rise_deg"] + CAM["dwell1"], 601)
    rho = np.array([pitch(t) for t in th]); drho = np.gradient(rho, np.radians(th))
    contact = rho >= ROLLER["x_sq"] - 1e-9                       # the roller is touched once the pitch passes its squeezed position
    s_eff = np.clip(rho - ROLLER["x_sq"], 0, None)
    F = np.where(contact, sp["F_sq"] + sp["k"] * 2 * s_eff, 0.0)  # the crossed spring shortens by both carriers' travel
    T = 2 * F * drho * 1.03 * rack_factor()                      # two rollers; 3 % roller bearings (E); the carrier racking on its guides (r5)
    alpha = np.degrees(np.arctan2(drho, rho))
    i = int(np.argmax(T))
    out = dict(turn_deg=float(th[-1]), rise=CAM["lobe"] - CAM["base"], pitch_rest=round(float(rho[0]), 2), pitch_open=round(float(rho[-1]), 2),
               gap_at_rest=round(ROLLER["x_sq"] - rho[0], 2), max_pressure_deg=round(float(alpha.max()), 1),
               peak_torque_Nmm=round(float(T[i]), 1), at_deg=round(float(th[i]), 1), peak_kgcm=round(float(T[i]) / 98.07, 2),
               margin={s: round(v["kgcm"] / (float(T[i]) / 98.07), 1) for s, v in SERVOS.items()},
               open_force_N=round(float(F[-1]), 1), spline_side_N="0 (two rollers equal and opposite; the stops take loads across)")
    out["curve"] = [(round(float(a), 1), round(float(b), 3), round(float(c), 1)) for a, b, c in zip(th[::20], rho[::20], T[::20])]
    return out


def cam_solid():
    """the cam's profile: the pitch curve (both lobes) offset in by the roller radius, extruded over CAM y (2D in the X-u plane)"""
    pts = []
    for k in range(720):
        a = 2 * math.pi * k / 720
        th = math.degrees(a) % 180.0                                    # two-fold
        # the roller sits at cam angle -theta at turn theta: radius at cam angle a is pitch(theta) with theta = -a (mod 180)
        r = pitch((-math.degrees(a)) % 180.0)
        pts.append((r * math.cos(a), r * math.sin(a)))
    cs = mf.CrossSection([pts]).offset(-B623["od"] / 2, mf.JoinType.Round, 2.0, 64)
    cs = cs - mf.CrossSection.circle(3.2, 32)                          # the horn's centre screw access
    m = cs.extrude(CAM["y"][1] - CAM["y"][0])                          # extrudes along z: (x, u-plane as x, y) -> rotate to X, u, Y
    m = m.rotate([90, 0, 0]).translate([0, CAM["y"][1], U_C])          # (x, y, z) -> (x, -z, y): plane x-y becomes X-u, extrusion -> -Y
    return m


# ============================================================== solids (head A frame)
def servo_solid(name, sign=1.0):
    s = SERVOS[name]; base = Y_SERVO_TOP - s["H"]
    x0, x1 = -s["shaft"], s["L"] - s["shaft"]
    body = box(x0, x1, base, Y_SERVO_TOP, U_C - s["W"] / 2, U_C + s["W"] / 2)
    ec = (x0 + x1) / 2; ear = box(ec - s["ear_span"] / 2, ec + s["ear_span"] / 2, base + s["ear_y"], base + s["ear_y"] + s["ear_t"], U_C - s["W"] / 2, U_C + s["W"] / 2)
    boss = cylY(0.0, U_C, Y_SERVO_TOP, Y_SERVO_TOP + s["boss"][1], s["boss"][0] / 2)
    ld = s["lead"]                                                     # the lead's grommet on the far end face near the base (E, CHECK)
    grom = box(x1, x1 + 3.0, base + ld["y"][0], base + ld["y"][1], U_C - ld["t"] / 2, U_C + ld["t"] / 2)
    return body + ear + boss + grom


def horn_solid():
    y0 = Y_SERVO_TOP + 4.0
    return cylY(0, U_C, y0, y0 + HORN["hub_h"], HORN["hub_d"] / 2) + cylY(0, U_C, y0 + HORN["hub_h"], y0 + HORN["hub_h"] + HORN["disc_t"], HORN["disc_d"] / 2)


def carrier(sign, open_=False):
    """carrier of the +X (sign +1) or -X blade, A frame, squeezed or open: blade + rib, bridge, boss on the rods, rack, roller lug, spring arm."""
    dx = STROKE if open_ else 0.0
    s = sign; xi = BLADE["inner_sq"]
    def bx(a, b, y0, y1, u0, u1):
        a, b = sorted((s * a, s * b)); return box(a + s * dx, b + s * dx, y0, y1, u0, u1)
    blade = bx(xi, xi + BLADE["t"], BLADE["y"][0], BLADE["y"][1], BOSS["u"][1] - 0.01, BLADE["tip_u"])
    w0, h0 = LATCH["pocket"]["w"] - 0.3, LATCH["pocket"]["h"] - 0.3; t5 = math.tan(math.radians(LATCH["pocket"]["wall_deg"])); d = BLADE["rib"]["d"]
    w1, h1 = w0 - 2 * d * t5, h0 - 2 * d * t5; xr = s * (xi + dx)
    rib = hull([[xr, Y_C + a * w0 / 2, BLADE["rib"]["u"] + b * h0 / 2] for a in (-1, 1) for b in (-1, 1)] +
               [[xr - s * d, Y_C + a * w1 / 2, BLADE["rib"]["u"] + b * h1 / 2] for a in (-1, 1) for b in (-1, 1)])
    rb = RODS["u"] + ROD_D / 2 + 0.15                                           # top of the rod bores
    bridge = bx(xi, BOSS["x_sq"][0] + 0.01, BLADE["y"][0], BLADE["y"][1], rb + 0.1, BOSS["u"][1]) + \
        bx(xi, BOSS["x_sq"][0] + 0.01, 163.0, 167.0, 4.5, rb + 0.2)   # a plate over the rods + a web between them
    boss = bx(BOSS["x_sq"][0], BOSS["x_sq"][1], BOSS["y"][0], BOSS["y"][1], BOSS["u"][0], BOSS["u"][1])
    yb = RODS["y"][0] - ROD_D / 2 - 0.15                                        # the lower rod bore's edge
    xc = s * (ROLLER["x_sq"] + dx)
    lug = cylY(xc, U_C, *ROLLER["collar"], 3.5, 32) + bx(ROLLER["x_sq"] - 3.5, ROLLER["x_sq"] + 3.5, CAM["y"][1] + CLR, yb, U_C - 3.5, 5.4) + \
        bx(BOSS["x_sq"][0], ROLLER["x_sq"] + 3.5, CAM["y"][1] + CLR, yb, BOSS["u"][0], BOSS["u"][1])   # under the hall board (u 6.4); the pin cantilevers from it
    # spring arm: the +X carrier's arm runs above the spring (u 8.9..11.4) to a seat at X -26..-22, the -X carrier's below it
    # (u -2.5..0.9) to a seat at X 22..26; each leaves its boss at the boss's outer end, at its own arm level only (they cross)
    us = SPRING_ROD["u"]; ay = (SPRING_ROD["y"] - SPRING_ROD["arm_w"] / 2, SPRING_ROD["y"] + SPRING_ROD["arm_w"] / 2)
    rs = spring_pick()["OD"] / 2
    if s > 0:
        ua = (us + rs + 1.0, BOSS["u"][1]); up = bx(BOSS["x_sq"][1] - 5.0, BOSS["x_sq"][1], BOSS["y"][1] - 0.01, ay[1], *ua)
        seat = bx(-SEAT_SQ - 4.0, -SEAT_SQ, ay[0], ay[1], us - rs, ua[1])
    else:
        ua = (us - rs - 4.25, us - rs - 1.0)
        up = bx(BOSS["x_sq"][1] - 5.0, BOSS["x_sq"][1], BOSS["y"][1] - 5.6, BOSS["y"][1], ua[0], BOSS["u"][0] + 0.01) + \
            bx(BOSS["x_sq"][1] - 5.0, BOSS["x_sq"][1], BOSS["y"][1] - 5.6, ay[1], *ua)
        seat = bx(-SEAT_SQ - 4.0, -SEAT_SQ, ay[0], ay[1], ua[0], us + rs)
    across = bx(-SEAT_SQ - 4.0, BOSS["x_sq"][1], ay[0], ay[1], *ua)
    c = blade + rib + bridge + boss + lug + up + across + seat
    # bores: the two rods, the spring rod through the seat, the roller pin
    for yr in RODS["y"]:
        c -= cylX(yr, RODS["u"], -40, 40, ROD_D / 2 + 0.15, 24)
    c -= cylX(SPRING_ROD["y"], SPRING_ROD["u"], -40, 40, ROD_D / 2 + 0.25, 24)
    c -= cylY(s * (ROLLER["x_sq"] + dx), U_C, ROLLER["y"][0] - 0.6, ROLLER["y"][1] + 0.6, B623["od"] / 2 + 0.6, 32)   # roller pocket
    return c


def parts(servo="DS3218", open_=False):
    """every head part in pose (A frame). Moving parts at squeezed (open_=False) or open."""
    P = {}
    P["carrier+"] = carrier(+1, open_); P["carrier-"] = carrier(-1, open_)
    dx = STROKE if open_ else 0.0
    for nm, sx in (("roller+", 1), ("roller-", -1)):                          # the 623ZZ on its pin (r6: the pin is modelled, in every keep-out)
        xr = sx * (ROLLER["x_sq"] + dx)
        P[nm] = cylY(xr, U_C, *ROLLER["y"], B623["od"] / 2) + cylY(xr, U_C, *ROLLER["pin_y"], ROD_D / 2)
    cam = cam_solid()
    turn = CAM["dwell0"] + CAM["rise_deg"] + CAM["dwell1"]
    P["cam"] = cam if not open_ else cam.translate([0, 0, -U_C]).rotate([0, -turn, 0]).translate([0, 0, U_C])   # +turn from X toward u
    P["horn"] = horn_solid(); P["servo"] = servo_solid(servo)
    P["rods"] = cylX(RODS["y"][0], RODS["u"], -RODS["wall_out"], RODS["wall_out"], ROD_D / 2) + cylX(RODS["y"][1], RODS["u"], -RODS["wall_out"], RODS["wall_out"], ROD_D / 2)
    P["spring rod"] = cylX(SPRING_ROD["y"], SPRING_ROD["u"], -SPRING_ROD["wall_out"], SPRING_ROD["wall_out"], ROD_D / 2)
    sp = spring_pick(); L = 2 * SEAT_SQ - 2 * dx
    P["spring"] = cylX(SPRING_ROD["y"], SPRING_ROD["u"], -L / 2, L / 2, sp["OD"] / 2) - cylX(SPRING_ROD["y"], SPRING_ROD["u"], -L / 2 - 1, L / 2 + 1, sp["ID"] / 2)
    P["body (drawn interfaces)"] = body_drawn(servo)
    return P


def body_drawn(servo="DS3218"):
    """the head body's drawn interfaces (topology joins them): plate + wing + web (the rack bar bonds to it) + the corner fillet, deck + pads, rod walls, spring walls,
    servo cradle, the stops, the bed land, hall pockets."""
    pl = box(*PLATE["x"], *PLATE["y"], *PLATE["u"]) + box(*WING["x"], *PLATE["y"], *WING["u"]) + box(*WEB["body_x"], *WEB["y"], *WEB["uA"])
    pl += cylY(WHEELS[2][0], WHEELS[2][1], *PLATE["y"], WHEEL["ecc_boss_r"], 48)
    pl -= box(WEB["body_x"][0] - 1, WEB["x"][1] + 1, WEB["y"][0] - 1, WEB["tail_y0"], WEB["uA"][0] - 1, WEB["tail_u"])
    pl += prism_xu(FILLET, *PLATE["y"])
    pl += box(35.0, WEB["x"][0] + 0.01, WEB["y"][0], PLATE["y"][0] + 0.01, 0.0, PLATE["u"][1])    # the spine under the wing (X 35..40.5)
    pl -= box(*BUMPER["x"], PLATE["y"][0] - 1, PLATE["y"][1] + 1, BUMPER["u"][0] - 1, BUMPER["u"][1])
    for (x, u) in WHEELS:
        r = WHEEL["ecc_bore_r"] if (x, u) == WHEELS[2] else WHEEL["axle_r"]
        pl -= cylY(x, u, PLATE["y"][0] - 1, PLATE["y"][1] + 1, r, 40)
    deck = box(-DECK["x"], DECK["x"], DECK["y"][0], DECK["y"][1], *DECK["u"])
    for s in (-1, 1):
        a, b = sorted((s * DECK["pad"]["x"][0], s * DECK["pad"]["x"][1]))
        deck += box(a, b, *DECK["pad"]["y"], DECK["u"][1] - 0.01, DECK["pad"]["u_top"])
        a, b = sorted((s * DECK["slot"][0], s * DECK["slot"][1]))
        deck -= box(a, b, DECK["slot"][2], DECK["slot"][3], DECK["u"][0] - 1, DECK["u"][1] + 1)
    for (x, y) in HALL_POS:
        deck -= box(x - 1.6, x + 1.6, y - 1.6, DECK["slot"][2] + 0.05, DECK["u"][0] - 1, U_BOT - LATCH["hall"]["gap"])    # hall face at u 14.4; joined to the slot (r8)
    walls = None
    for s in (-1, 1):
        a, b = sorted((s * RODS["wall_in"], s * RODS["wall_out"]))
        w = box(a, b, BOSS["y"][0], BOSS["y"][1], BOSS["u"][0], BOSS["u"][1])
        a, b = sorted((s * SPRING_ROD["wall_in"], s * SPRING_ROD["wall_out"]))
        w += box(a, b, SPRING_ROD["y"] - 4.5, SPRING_ROD["y"] + 4.5, SPRING_ROD["u"] - 3.5, SPRING_ROD["u"] + 4.0)
        walls = w if walls is None else walls + w
    for yr in RODS["y"]:
        walls -= cylX(yr, RODS["u"], -RODS["wall_out"] + 1.0, RODS["wall_out"] + 1.0, ROD_D / 2 + 0.05, 24)   # -X blind, +X open (in from +X), epoxied
    walls -= cylX(SPRING_ROD["y"], SPRING_ROD["u"], -SPRING_ROD["wall_out"] + 1.0, SPRING_ROD["wall_out"] + 1.0, ROD_D / 2 + 0.05, 24)
    pads, pad_cuts = cradle(servo)
    pads = pads - pad_cuts
    rb, rc = retainer_bosses()
    return pl + deck + walls + pads + stops() + bed_land() + tail_rib(servo) + rb - rc


def rack_bar(servo="DS3218"):
    """the m1 rack as its own printed bar (concept review r2: printed flat, its tooth profile in the layers, while the body prints on its
    side), epoxied along its 93 x 13.5 back face (X 41.5) onto the body web (X 39.5..41.5); teeth tips X 46.5, root
    44.25 (2.75 of bar under the root); phase from u -63 (arm_v5_cad); B's tail to u -54; the tail step under Y 63 for u < -39."""
    wu = WEB["uB"] if servo == "MG995" else WEB["uA"]
    bar = box(*WEB["bar_x"], WEB["y"][0], WEB["teeth_y"][1], *wu) - box(WEB["bar_x"][0] - 1, WEB["bar_x"][1] + 1, WEB["y"][0] - 1, WEB["tail_y0"], wu[0] - 1, WEB["tail_u"])
    p = math.pi; xt, xr = WEB["bar_x"][1], 45.5 - 1.25; g = None; u = WEB["uA"][0] + p / 2
    while u < WEB["uA"][1] - p / 2:
        wt = p / 2 + 0.1 + 2 * 1.0 * math.tan(math.radians(20)); wr = p / 2 + 0.1 - 2 * 0.25 * math.tan(math.radians(20))
        if u > wu[0]:
            k = hull([[xt + 0.01, y, u + s_ * wt / 2] for s_ in (-1, 1) for y in (WEB["y"][0] - 0.01, WEB["teeth_y"][1] + 0.01)] +
                     [[xr, y, u + s_ * wr / 2] for s_ in (-1, 1) for y in (WEB["y"][0] - 0.01, WEB["teeth_y"][1] + 0.01)])
            g = k if g is None else g + k
        u += p
    return bar - g


def tail_rib(servo="DS3218"):
    """the drawn backing rib behind the rack tail (head A or B)"""
    r = None
    for poly, (y0, y1) in TAIL_RIB["B" if servo == "MG995" else "A"]:
        m = prism_xu(poly, y0, y1); r = m if r is None else r + m
    return r


WIRE = dict(pair=dict(x=(9.0, 38.0), y=(86.5, 95.0), u=(-12.3, -6.3)),   # the mated JR pair (29 x 8 x 4.5, E: the longest) along X under the case
            bend=4.0,                                                     # bend room at both ends of the pair (the wires leave its ends, r >= 3)
            hall_x=(8.8, 12.3), hall_u=(9.9, 12.2), y_merge=(92.0, 96.0),   # the runs: 2.2 over the servo pocket, 2.7 under the column top
            board=dict(x=(4.3, 8.7), y=(155.5, 158.9), u0=9.0),           # the board (4.2 x 3.2, package long side along Y) + fit; to the blade sweep's edge
            wire="3 x 30 AWG per board (Ø0.6 each, E)")


def wiring(servo="DS3218"):
    """the reserved wire and connector spaces (r8; CLAUDE.md item 8). Hall boards: each in its pocket past the cam's end (X 4.3..8.7, Y
    155.5..158.9, to the blade's swept keep-out: the board 1.0 off the blade), a riser X 6.7..8.7 and a crossing at Y 152.5..153.8 to its
    run, one closed run a side above the servo (|X| 8.8..12.3, u 9.9..12.2: >= 2.2 to the servo pocket and the horn, >= 1.9 to the cam, 2.7 roof),
    merged at Y 92..96 into the ribbon slot; every section >= 2.0 x 2.0 for 3 x 30 AWG. Servo: the lead from its grommet (far end, near
    the base: E, CHECK) down the end face, over the retainer's tab to the +X end of the mated JR pair (laid along X under the case,
    X 9..38, with 4.0 of bend room at both ends), the socket's wires up from the -X end to the ribbon slot. The retainer's tab closes the
    pair and the lead from below; with the retainer off they drop out along -u with the servo."""
    s = SERVOS[servo]; base = Y_SERVO_TOP - s["H"]; x1 = s["L"] - s["shaft"]; ld = s["lead"]; W_ = []; pr = WIRE["pair"]; hb = WIRE["board"]
    for sg in (-1, 1):
        def bx_(a, b, *r):
            a, b = sorted((sg * a, sg * b)); return box(a, b, *r)
        W_.append(bx_(*WIRE["hall_x"], WIRE["y_merge"][0], 153.8, *WIRE["hall_u"]))                   # the run above the servo
        W_.append(bx_(6.7, WIRE["hall_x"][1], 152.5, 153.8, *WIRE["hall_u"]))                        # across, over the cam (>= 1.0)
        W_.append(bx_(6.7, 8.7, 152.5, hb["y"][0] + 0.5, *WIRE["hall_u"]))                            # the riser to the board (1.0 off the lugs)
        W_.append(bx_(*hb["x"], *hb["y"], hb["u0"], DECK["u"][0] + 0.01))                              # the board's pocket
    W_.append(box(-WIRE["hall_x"][1], WIRE["hall_x"][1], *WIRE["y_merge"], 7.0, WIRE["hall_u"][1]))
    ut = RETAINER["u"][1] + 0.2                                                # just over the retainer's tab: with the retainer off, its pocket is the way out
    W_.append(box(x1 - 0.2, x1 + 4.0, base - 0.5, base + ld["y"][1] + 1.0, RETAINER["u"][0] - 1.0, U_C + ld["t"] / 2 + 1.0))   # grommet, lead down (open to -u)
    W_.append(box(x1 - 0.2, pr["x"][1] + WIRE["bend"], pr["y"][0], base - 0.4, ut, pr["u"][1]))                      # the lead over the tab to the pair's +X end
    W_.append(box(pr["x"][0] - WIRE["bend"], pr["x"][1] + WIRE["bend"], *pr["y"], ut, pr["u"][1]))                     # the pair + its two bends
    W_.append(box(pr["x"][0] - WIRE["bend"], pr["x"][0], 88.0, 93.5, pr["u"][1] - 0.1, 10.3))                         # the socket's wires up (-X end)
    W_.append(box(-4.1, pr["x"][0], 88.0, 93.5, 7.0, 10.3))                                                            # to the ribbon slot
    return W_


def retainer(filled=False):
    """the servo retainer bar (A frame): its two Ø12 ends plus the 10 wide bar, u -16.3..-12.5, with 2 countersunk M3 holes"""
    (x0, y0), (x1, y1) = RETAINER["ends"]; u0, u1 = RETAINER["u"]
    ends = cylZ(x0, y0, u0, u1, RETAINER["end_r"], 32) + cylZ(x1, y1, u0, u1, RETAINER["end_r"], 32)
    L = math.hypot(x1 - x0, y1 - y0); ang = math.degrees(math.atan2(y1 - y0, x1 - x0))
    bar = box(0, L, -RETAINER["w"] / 2, RETAINER["w"] / 2, u0, u1).rotate([0, 0, ang]).translate([x0, y0, 0])
    m = ends + bar + box(x0, WIRE["pair"]["x"][1] + WIRE["bend"] + 0.5, WIRE["pair"]["y"][0] - 0.5, 97.0, u0, u1)   # the tab under the JR pair and the lead
    if filled: return m
    for (x, y) in RETAINER["ends"]:
        m -= cylZ(x, y, u0 - 1, u1 + 1, 1.7, 24)
        m -= hull([[x + r * math.cos(a), y + r * math.sin(a), uu] for a in np.linspace(0, 2 * math.pi, 24, endpoint=False) for (r, uu) in ((3.1, u0 - 0.01), (1.6, u0 + 1.6))])   # 90 deg csk
    return m


def retainer_bosses():
    """the drawn Ø12 bosses over the retainer's ends (heat-set M3 x 5.7) and their insert holes (solid, cuts)"""
    sol = None; cut = None; u1 = RETAINER["u"][1] + 0.2
    for (x, y) in RETAINER["ends"]:
        b = cylZ(x, y, u1 - 0.01, u1 + RETAINER["insert"]["depth"] + 1.5, RETAINER["boss_r"], 32)
        c = cylZ(x, y, u1 - 0.5, u1 + RETAINER["insert"]["depth"], RETAINER["insert"]["d"] / 2, 24)
        sol = b if sol is None else sol + b; cut = c if cut is None else cut + c
    return sol, cut


def bed_land():
    """the drawn 2 mm land on the column's -X flank at BED_X: the body's flat bed face for the +x build"""
    return prism_xu(COLUMN_POLY, *COLUMN_Y) ^ box(BED_X, BED_X + 2.0, 0.0, 300.0, -100.0, 100.0)


def stops():
    """the bosses' inward stops: a block each side (face at the boss's inner end with the blade at inner 4.8) on a post to the column"""
    st = None
    xf = BOSS["x_sq"][0] - (BLADE["inner_sq"] - STOP["inner"])               # stop face X 9.8
    for s_ in (-1, 1):
        a, b = sorted((s_ * (xf - STOP["t"]), s_ * xf))
        m = box(a, b, *STOP["y"], *STOP["u"]) + box(a, b, *STOP["y"], STOP["post_u"], STOP["u"][0] + 0.01)
        st = m if st is None else st + m
    return st


def ear_holes(servo="DS3218"):
    s = SERVOS[servo]; ec = (-s["shaft"] + s["L"] - s["shaft"]) / 2
    return [(ec + a * s["holes"][0] / 2, U_C + b * s["holes"][1] / 2) for a in (-1, 1) for b in (-1, 1)]


def cradle(servo="DS3218"):
    """the servo mount (concept review r1: pads round the ear holes wrap the case): a U round the case's upper 12 (open toward -u, the
    servo slides in from the axle side), Ø12 pads under the four ears (cut by the case pocket: >= 2.6 web), the ears in slots 0.3 over
    their thickness (captured in Y); the retainer bar under the servo holds it in u (r4: no ear screws, their driver had no path on the
    arm). The U and the pads take the torque. Returns (solid, cuts); the cuts include the slide-in channel and the ear slots."""
    s = SERVOS[servo]; base = Y_SERVO_TOP - s["H"]; ye = base + s["ear_y"]; x0, x1 = -s["shaft"], s["L"] - s["shaft"]
    y0 = ye - 12.0; w, c = 2.2, 0.2
    sol = box(x0 - c - w, x0 - c + 0.01, y0, ye, U_C - s["W"] / 2 - c, U_C + s["W"] / 2 + c + w) + \
        box(x1 + c - 0.01, x1 + c + w, y0, ye, U_C - s["W"] / 2 - c, U_C + s["W"] / 2 + c + w) + \
        box(x0 - c - w, x1 + c + w, y0, ye, U_C + s["W"] / 2 + c - 0.01, U_C + s["W"] / 2 + c + w)
    for (xh, uh) in ear_holes(servo):
        sol += cylY(xh, uh, y0, ye, 6.0, 48)
    sol -= prism_xu([(-100.0, PARK_LINE + CLR * math.sqrt(2) + 0.05 + 100.0), (60.0, PARK_LINE + CLR * math.sqrt(2) + 0.05 - 60.0), (-100.0, -100.0)], 0.0, 300.0)   # the park bisector (r6)
    cuts = box(x0 - c, x1 + c, base - 1.0, Y_SERVO_TOP + 1.0, U_C - s["W"] / 2 - 40.0, U_C + s["W"] / 2 + c)        # case + slide-in from -u
    ec = (x0 + x1) / 2
    cuts += box(ec - s["ear_span"] / 2 - 0.3, ec + s["ear_span"] / 2 + 0.3, ye, ye + s["ear_t"] + 0.3, U_C - s["W"] / 2 - 40.0, U_C + s["W"] / 2 + 0.3)   # the ears' slots (captured in Y)
    return sol, cuts


# ============================================================== the body's design space (topology) and keep-outs
COLUMN_POLY = [(-28.6, 14.9), (28.1, 14.9), (31.5, 13.0), (34.5, 10.5), (38.5, 8.0), (41.5, 6.0), (45.5, 4.0), (48.5, 1.5), (48.5, -11.8), (44.0, -16.3),
               (-14.3, -16.3), (-38.1, 8.0), (-34.5, 10.5), (-31.5, 13.0)]   # X, u: the bay wedge (map - 1.0), the park bisector (X + u >= -30.6), u -16.3 (the head lane r >= 95 at the drone insert z0 111.3)
COLUMN_Y = (74.5, 188.0)
BED_X = -34.0                                            # the +x build's bed: the column's -X flank cut flat here (u 3.4..10.9 x 113 long)


def prism_xu(poly, y0, y1):
    """a prism of an (X, u) polygon along Y"""
    pts = [(x, u) for x, u in poly]
    a2 = sum(pts[i][0] * pts[(i + 1) % len(pts)][1] - pts[(i + 1) % len(pts)][0] * pts[i][1] for i in range(len(pts)))
    cs = mf.CrossSection([pts if a2 > 0 else pts[::-1]])                  # counter-clockwise
    return cs.extrude(y1 - y0).rotate([90, 0, 0]).translate([0, y1, 0])    # (x, y, z) -> (x, -z, y): X-u plane, extruded to -Y


def keepouts(servo="DS3218"):
    """what the body may not fill: every moving part over its whole travel + 1.0, the bought parts + 1.0, tool and wire paths"""
    K = []
    P0 = parts(servo, open_=False); P1 = parts(servo, open_=True)
    din = BLADE["inner_sq"] - STOP["inner"]                                    # 1.2 inward to the stops (no cartridge)
    for nm in ("carrier+", "carrier-", "roller+", "roller-"):
        sg = 1 if nm.endswith("+") else -1
        K.append(swept_grown(servo, nm, -sg * din, sg * STROKE) - stops())   # 1.0 everywhere but the stop faces (r4)
    K.append(cylY(0, U_C, CAM["y"][0] - CLR, CAM["y"][1] + CLR, CAM["lobe"] + CLR + 0.05, 64))      # the cam's swept disc (+0.05: polygons)
    K.append(cylY(0, U_C, Y_SERVO_TOP + 4.0 - CLR, Y_SERVO_TOP + 4.0 + HORN["hub_h"] + HORN["disc_t"] + CLR, HORN["disc_d"] / 2 + CLR, 64))  # horn + 1.0 (r4: its top face)
    sv_ = SERVOS[servo]                                                        # the servo: its pocket is the cradle's cut (0.2 fit, slide-in to -u,
    K.append(cylY(0.0, U_C, Y_SERVO_TOP - 0.01, Y_SERVO_TOP + sv_["boss"][1] + CLR, sv_["boss"][0] / 2 + CLR, 48))   # ear slots; r5); its spline boss
    K += wiring(servo)
    for s_ in (-1, 1):                                                         # cartridge v6 (Scout): TPU strips |x| 19.2..23.2, 2.0 under the flat bottom, z_T +-32
        a_, b_ = sorted((s_ * (19.2 - CLR), s_ * (23.2 + CLR)))
        K.append(box(a_, b_, Y_C - 33.0, Y_C + 33.0, U_BOT - 2.0 - CLR, U_BOT + 0.01))
    for s_ in (-1, 1):                                                         # the receivers' side walls come down to u 15.6 at |x| >= 23.4
        a_, b_ = sorted((s_ * (23.4 - CLR), s_ * 40.0))
        K.append(box(a_, b_, Y_C - 16.0, Y_C + 16.0, 15.6 - CLR, 30.0))
    K.append(box(-DECK["slot"][1] - CLR, DECK["slot"][1] + CLR, DECK["slot"][2] - CLR, DECK["slot"][3] + CLR, DECK["u"][1], 40.0))   # blades above the deck
    K.append(box(-40, 40, max(PLATE["y"][1], 72.75), 260.0, U_BOT, 80.0))                            # the carried cartridge + receivers (above u 15.4)
    yk = PLATE["y"][1] + 16.0                                                                         # a 3 mm L-key's short leg + 1
    K.append(cylY(WHEELS[0][0], WHEELS[0][1], PLATE["y"][1], yk, 3.5, 32) + box(WHEELS[0][0] - 3.5, 60, PLATE["y"][1], yk, WHEELS[0][1] - 3.5, WHEELS[0][1] + 3.5) +
             cylX(yk, WHEELS[0][1], WHEELS[0][0], 60, 3.5, 32))                                     # key to the near axle (L-key from +X), round top
    K.append(cylY(WHEELS[2][0], WHEELS[2][1], PLATE["y"][1], 100.0, 8.0, 32))                        # pin spanner on the eccentric flange
    K.append(box(-4.1, 4.1, PLATE["y"][0] - 1, 100.0, 7.0, 10.3))                                    # the 5-wire ribbon (6.1 x 1.3 + 1.0) to the rail's 5.0 mouth at X 0
    for yr in RODS["y"]:                                                                              # the rods go in from +X
        K.append(cylX(yr, RODS["u"], RODS["wall_out"], 70.0, ROD_D / 2 + 1.0, 24))
    K.append(cylX(SPRING_ROD["y"], SPRING_ROD["u"], SPRING_ROD["wall_out"], 70.0, ROD_D / 2 + 1.0, 24))
    sv = SERVOS[servo]; ye = Y_SERVO_TOP - sv["H"] + sv["ear_y"]
    K.append(_grow(retainer(filled=True), 0.2))                                                       # the retainer in its pockets (+0.2)
    K.append(cradle(servo)[1])
    if servo == "MG995":                                                                              # detent arm by head B at park (+1.0); the web and bar keep 1.16 (sweep)
        K.append(box(15.0, WEB["body_x"][0], 70.0, 77.0, -20.0, -14.0))
    return K


def grow_box(m, d):
    lo, hi = tm(m).bounds
    return box(lo[0] - d, hi[0] + d, lo[1] - d, hi[1] + d, lo[2] - d, hi[2] + d)


_GROWN = {}


def swept_grown(servo, nm, x0, x1):
    """part nm (squeezed datum pose) grown by CLR (exact Minkowski with a ball, 16 segments scaled to circumscribe) and swept along X from
    x0 to x1 in 0.25 steps (x 1.03 for the scallop: measured >= 1.0, `clearances`); cached per (servo, part)"""
    key = (servo, nm, x0, x1)
    if key not in _GROWN:
        g = parts(servo, open_=False)[nm].minkowski_sum(mf.Manifold.sphere(1.03 * CLR / math.cos(math.pi / 16), 16))   # 1.03: the 0.25 scallop
        n = max(1, int(math.ceil(abs(x1 - x0) / 0.25)))
        m = None
        for i in range(n + 1):
            t = g.translate([x0 + (x1 - x0) * i / n, 0, 0]); m = t if m is None else m + t
        _GROWN[key] = m
    return _GROWN[key]


def hull_of(a, b, grow=0.0, sweep_x=0.0):
    """the swept volume of a part translating along X from pose a by sweep_x (b: the end pose, included), a pose every 0.5 mm (r4: three
    poses left gaps at thin features), grown by `grow`"""
    n = max(1, int(math.ceil(abs(sweep_x) / 0.5)))
    m = b
    for i in range(n + 1):
        m = m + a.translate([sweep_x * i / n, 0, 0])
    return m if grow <= 0 else _grow(m, grow)


def _grow(m, d):
    """Minkowski with a ball of radius d, approximated by the union of 14 shifted copies (6 axes, 8 diagonals at d): every direction is
    covered to at least 0.88 d (the 26-direction gap), measured on the parts (`clearances` in layout.json)"""
    out = m
    for v in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)) + tuple((i, j, k) for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)):
        v = np.array(v, float); v = v / np.linalg.norm(v) * d
        out = out + m.translate(list(v))
    return out


def plate_ext():
    """the room below u 0 at the plate and root level (Y 68.75..74.5) the assembled arm leaves free over the four cycles (space map:
    outside the detent ring's V |X| >= 4.5 + 1.25 |u| and the rail stop |X| >= 11, + 1.0; the park bisector X + u >= -29.6 on -X;
    u >= -17.2 on +X): the carriage plate and the column's root grow into it (concept review r1, the root over the 4.0 plate)"""
    plus = [(11.0, 0.0), (11.0, -5.2), (26.0, -17.2), (39.5, -17.2), (44.0, -21.7), (44.0, 0.0)]   # -17.2: 1.0 over the detent arm (B, sweep r3); 45 deg down the web
    minus = [(-11.0, 0.0), (-29.6, 0.0), (-18.5, -11.2), (-11.0, -5.2)]
    return prism_xu(plus, PLATE["y"][0], COLUMN_Y[0] + 0.01) + prism_xu(minus, PLATE["y"][0], COLUMN_Y[0] + 0.01)


def design_space(servo="DS3218"):
    """the envelope the body's topology may fill (A frame), the drawn interfaces included: the column prism, the plate and the room
    under it (spine, a rib between the wheels, ribs beside the eccentric), the root band over the plate and the web lane; minus keep-outs"""
    col = prism_xu(COLUMN_POLY, *COLUMN_Y)
    root = prism_xu(COLUMN_POLY, PLATE["y"][1] - 0.01, COLUMN_Y[0] + 0.01) ^ box(-40.0, 44.0, 60.0, 80.0, 0.0, 20.0)   # the column's own section, no step
    web_u = WEB["uB"] if servo == "MG995" else WEB["uA"]
    plate = box(*PLATE["x"], *PLATE["y"], *PLATE["u"]) + box(*WING["x"], *PLATE["y"], *WING["u"]) + box(*WEB["body_x"], *WEB["y"], *web_u)
    plate = plate - box(WEB["body_x"][0] - 1, WEB["x"][1] + 1, WEB["y"][0] - 1, WEB["tail_y0"], web_u[0] - 1, WEB["tail_u"])
    plate = plate + prism_xu(FILLET, *PLATE["y"])
    under = box(35.0, WEB["x"][0] + 0.01, WEB["y"][0], PLATE["y"][0] + 0.01, 0.0, PLATE["u"][1]) + \
        box(13.0, 35.01, WEB["y"][0], PLATE["y"][0] + 0.01, 16.5, 19.5) + \
        box(-34.0, -13.0, WEB["y"][0], PLATE["y"][0] + 0.01, 31.0, 39.0)
    ds = col + root + plate + under + plate_ext() + tail_rib(servo)
    for k in keepouts(servo): ds = ds - k
    ds = ds - rack_bar(servo)                                               # the bonded rack bar (own part)
    ds = ds - box(-100.0, BED_X, 0.0, 300.0, -100.0, 100.0)                   # the flat bed face (concept review r2)
    for (x, u) in WHEELS:                                                   # wheels, nuts, shims and the axle bolts are own bought parts
        ds = ds - cylY(x, u, 50.0, PLATE["y"][0], WHEEL["od"] / 2 + CLR, 48)
    ds = ds - box(-12.0 - CLR, 12.0 + CLR, 50.0, PLATE["y"][0] - 0.01, -80, 80)   # the rail and its ridges under the plate
    ds = ds - box(*BUMPER["x"], PLATE["y"][0] - 1, PLATE["y"][1] + 1, BUMPER["u"][0] - 1, BUMPER["u"][1])
    return ds


# ============================================================== checks
def space_grid(heads="AB"):
    """the arm's occupied voxels over the 4 cycles in the head frame, for head A, head B or both (each head is checked on its own map)"""
    import glob
    fs = sorted(glob.glob(os.path.join(ROOT, "runs", "head", "v7", "space", "*.npz")))
    if not fs: return None
    return np.any(np.stack([dict(np.load(f))[h] for f in fs for h in heads]), axis=0)


SBOX = dict(x=(-70.0, 70.0), y=(60.0, 230.0), u=(-70.0, 80.0))


def space_check(P, G):
    """per part: the arm's occupied voxels (1 mm, union over 4 cycles) whose centres fall inside the part, the part's least X + u
    (the park bisector, min over its vertices) and its lowest Z at the commanded drone insert (C)."""
    out = {}
    o = np.array([SBOX["x"][0], SBOX["y"][0], SBOX["u"][0]])
    for nm, m in P.items():
        t = tm(m); lo, hi = t.bounds
        pts = t.voxelized(pitch=1.0).fill().points                          # the part's 1 mm voxel centres
        if nm.startswith("carrier"):                                        # blade tips in the own cartridge's recess: the map is state-blind
            pts = pts[~((pts[:, 2] > U_BOT) & (np.abs(pts[:, 0]) < LATCH["recess_x"]))]   # (a released pack's tabs); the arm sweep checks them
        if nm.startswith("body"):                                           # the rack bar meshes with pinion16 (in the map): not a clash
            pts = pts[~((pts[:, 0] > WEB["x"][0] - 1) & (pts[:, 1] < WEB["teeth_y"][1] + 1))]
        i = np.floor(pts - o).astype(int); ok = np.all((i >= 0) & (i < np.array(G.shape)), axis=1); i = i[ok]
        hit = int(G[i[:, 0], i[:, 1], i[:, 2]].sum())
        xu = float((t.vertices[:, 0] + t.vertices[:, 2]).min())
        out[nm] = dict(arm_overlap_voxels=hit, min_X_plus_u=round(xu, 2), park_ok=bool(xu >= PARK_LINE + CLR * math.sqrt(2) - 1e-6),
                       Z_min_at_drone=round(float(Z0["insert_cmd"] + lo[2]), 1),
                       X=(round(float(lo[0]), 1), round(float(hi[0]), 1)), Y=(round(float(lo[1]), 1), round(float(hi[1]), 1)),
                       u=(round(float(lo[2]), 1), round(float(hi[2]), 1)))
    return out


def clearances():
    """the least distance (C, fcl) from each moving part over its travel to the body's envelope (the design space): the carriers and
    rollers from their stops (inner 4.8) to open, the stop faces themselves excepted (designed contact); servo, horn; the cam at every turn"""
    import trimesh.collision as tc
    out = {}
    for sv in SERVOS:
        m = tc.CollisionManager(); m.add_object("ds", tm(design_space(sv) - _grow(stops(), 0.6)))
        din = BLADE["inner_sq"] - STOP["inner"]; P = parts(sv); r = {}
        for nm in ("carrier+", "carrier-", "roller+", "roller-"):
            sg = 1 if nm.endswith("+") else -1
            r[nm] = round(min(m.min_distance_single(tm(P[nm].translate([sg * d, 0, 0]))) for d in np.linspace(-din, STROKE, 15)), 2)
        r["horn"] = round(m.min_distance_single(tm(P["horn"])), 2)
        r["servo"] = "seated: ears on their pads (designed contact), 0.2 fit in its pocket"
        r["cam (every 10 deg)"] = round(min(m.min_distance_single(tm(cam_solid().translate([0, 0, -U_C]).rotate([0, a, 0]).translate([0, 0, U_C])))
                                            for a in range(0, 180, 10)), 2)
        out[sv] = r
    return out


def internal_checks():
    """moving parts against each other and the drawn body, squeezed and open (mm3 of overlap; designed touches excluded)"""
    res = {}
    for st in (False, True):
        P = parts(open_=st)
        names = [n for n in P if n not in ("servo",)]
        pairs = {}
        allowed = {("carrier+", "rods"), ("carrier-", "rods"), ("carrier+", "spring rod"), ("carrier-", "spring rod"), ("rods", "body (drawn interfaces)"),
                   ("spring rod", "body (drawn interfaces)"), ("spring", "spring rod"), ("roller+", "carrier+"), ("roller-", "carrier-"),
                   ("cam", "horn"), ("horn", "servo"), ("spring", "carrier+"), ("spring", "carrier-"), ("cam", "roller+"), ("cam", "roller-")}
        allP = dict(P); allP["servo"] = P["servo"]
        keys = list(allP)
        for i, a in enumerate(keys):
            for b in keys[i + 1:]:
                if (a, b) in allowed or (b, a) in allowed: continue
                v = (allP[a] ^ allP[b]).volume()
                if v > 0.05: pairs[f"{a} / {b}"] = round(v, 2)
        res["open" if st else "squeezed"] = pairs
    return res


def latch_checks():
    """the blades against the latch cartridge's interface (cartridge coords via the head frame), approach open, grip squeezed (C)."""
    L = LATCH; r = {}
    rib_tip_open = BLADE["inner_open"] - BLADE["rib"]["d"]
    r["rib_tip_open_vs_latched_tab"] = round(rib_tip_open - L["tab_latched"][1], 2)
    r["blade_outer_open_vs_recess"] = round(L["recess_x"] - (BLADE["inner_open"] + BLADE["t"]), 2)
    r["with_head_offset_1"] = dict(rib_tip=round(rib_tip_open - L["head_offset"] - L["tab_latched"][1], 2),
                                   outer_vs_recess=round(L["recess_x"] - (BLADE["inner_open"] + BLADE["t"] + L["head_offset"]), 2),
                                   note="a 1.0 offset puts one rib 0.8 inside the latched tab's outer face: the tab's 1.0 x 45 bottom chamfer "
                                        "cams it in 0.8 (the latch's own case: bolt still 1.5 in its pocket)")
    r["blade_tip_vs_bar"] = round((L["bar_y0"] + U_BOT) - BLADE["tip_u"], 2)
    r["rib_in_pocket"] = dict(depth=BLADE["rib"]["d"], u=BLADE["rib"]["u"], pocket_u=(U_BOT + L["pocket"]["y"] - L["pocket"]["h"] / 2, U_BOT + L["pocket"]["y"] + L["pocket"]["h"] / 2))
    hall_y = HALL_POS[0][1]
    r["hall_vs_blade_y"] = round(BLADE["y"][0] - (hall_y + HALL["pkg"][1] / 2), 2)
    r["pads_on_flat_bottom"] = dict(pad_x=DECK["pad"]["x"], frames=L["frames"], worst_offset_contact_mm=round(L["frames"][1] - (DECK["pad"]["x"][0] + L["head_offset"]), 2),
                                    note="both packs have the latch frames x 15.5..19 (Scout) / ..26.5 (Overseer) at the tab line; the 1.0 recess chamfer to 16.5")
    return r


def strength(sp):
    """hand checks (E loads, C sections): pull path, blade, rods, deck pads, carriage plate and spine; index = stress / allowable"""
    PETG = dict(allow=25.0, cyclic=TARGET["cyclic_MPa"])          # sigma allow as docs/generative.md (PETG strut 25 MPa, E)
    r = {}
    per_tab = LOADS["pull_unmate"] / 2
    opening = per_tab * math.tan(math.radians(LATCH["pocket"]["wall_deg"])) + LATCH["tab_spring_sq"]
    r["hold"] = dict(per_tab_N=round(per_tab, 1), opening_N=round(opening, 2), spring_N=sp["F_clamp"], margin=round(sp["F_clamp"] / opening, 2),
                     note=f"at the clamp (inner {CLAMP:g}, cartridge v6; {sp['F_sq']} N at the datum 6.0, cartridge v5)")
    r["release"] = {p: round(sp["F_clamp"] / v, 1) for p, v in LATCH["release_N"].items()}
    w0 = LATCH["pocket"]["w"] - 0.3
    r["rib_bearing_MPa"] = round(per_tab / (w0 * BLADE["rib"]["d"]), 2)
    lever = BLADE["rib"]["u"] - BOSS["u"][1]
    M = sp["F_open"] * lever; s_bl = 6 * M / ((BLADE["y"][1] - BLADE["y"][0]) * BLADE["t"] ** 2)
    bw = BLADE["y"][1] - BLADE["y"][0]
    r["blade_root_MPa"] = dict(squeeze=round(6 * sp["F_sq"] * lever / (bw * BLADE["t"] ** 2), 2), worst=round(s_bl, 2), lever=lever)
    span = 2 * (RODS["wall_in"] + RODS["wall_out"]) / 2; a = (BOSS["x_sq"][0] + BOSS["x_sq"][1]) / 2
    P = per_tab / 2; Mr = P * (span / 2 - a); W = math.pi * ROD_D ** 3 / 32
    r["rods"] = dict(span=span, per_boss_N=round(P, 2), sigma_MPa=round(Mr / W, 1), index=round(Mr / W / 1000.0, 3),
                     boss_MPa=round(P / (ROD_D * (BOSS["x_sq"][1] - BOSS["x_sq"][0])), 2))
    Lb = BOSS["x_sq"][1] - BOSS["x_sq"][0]
    e_yaw = max(SPRING_ROD["y"] - Y_C, Y_C - (CAM["y"][0] + CAM["y"][1]) / 2); e_pitch = RODS["u"] - U_C
    a_ = (BOSS["x_sq"][0] + BOSS["x_sq"][1]) / 2; Lg = a_ + SEAT_SQ + 2.0
    e_y = SPRING_ROD["y"] - (ROLLER["y"][0] + ROLLER["y"][1]) / 2; e_p = SPRING_ROD["u"] - U_C
    r["anti_rack"] = dict(guide_length=round(Lg, 1), note="the boss on the two rods and the seat's bore on the spring rod guide the carrier (r5)",
                          need=round(2 * 0.3 * (e_y + e_p), 1), ok=bool(Lg >= 1.3 * 2 * 0.3 * (e_y + e_p)), cam_over_spring=round(rack_factor(), 2))
    o = LATCH["head_offset"]; f0, f1 = 15.5, LATCH["frames"][1]; p0, p1 = DECK["pad"]["x"]
    ov = min(min(p1 + d, f1) - max(p0 + d, f0) for d in (-o, o))                # the worse pad at a 1.0 head offset either way
    pad_area = 2 * (DECK["pad"]["y"][1] - DECK["pad"]["y"][0]) * ov
    r["pads_MPa"] = dict(stall=round(LOADS["stall"] / pad_area, 2), area_worst_mm2=round(pad_area, 1))
    lev = Y_C - PLATE["y"][1]
    r["head_moment_Nm"] = dict(pull=round(LOADS["pull_unmate"] * lev / 1e3, 2), stall=round(LOADS["stall"] * lev / 1e3, 2), lever=lev)
    # carriage: the moment about X leaves through the wheels' axial (Y) reactions; rigid plate: near (u 4) + far (u 32), lever 28;
    # flexible plate: near + eccentric (u 18), lever 14
    for case, Mh in (("pull", r["head_moment_Nm"]["pull"]), ("stall", r["head_moment_Nm"]["stall"])):
        r[f"wheel_axial_N_{case}"] = dict(rigid=round(Mh * 1e3 / 28.0, 0), flexible=round(Mh * 1e3 / 14.0, 0))
    # spine + plate T section (X 35..46.5 x Y 59.25..72.75 + plate 4 x 40 effective), far wheel reaction from u 32 to the column at u 14
    A1, y1 = 11.5 * 13.5, (59.25 + 72.75) / 2; A2, y2 = 40 * 4.0, 70.75
    yc = (A1 * y1 + A2 * y2) / (A1 + A2)
    I = 11.5 * 13.5 ** 3 / 12 + A1 * (y1 - yc) ** 2 + 40 * 4.0 ** 3 / 12 + A2 * (y2 - yc) ** 2
    c = max(yc - 59.25, 72.75 - yc)
    for case in ("pull", "stall"):
        Rw = r[f"wheel_axial_N_{case}"]["rigid"]; Mp = Rw * (32.0 - 14.0)
        r[f"spine_MPa_{case}"] = round(Mp * c / I, 1)
    r["plate_panel_far_wheel_MPa"] = dict(pull=round(0.5 * r["wheel_axial_N_pull"]["rigid"] / 16.0 * 2, 1),
                                          note="4.0 plate panel round the far axle between ribs (clamped, central load, E): the topology run's FE decides")
    # +X carrier's spring arm (above the spring): the seat pushes at the spring axis, the arm's centroid sits off it (E)
    rs = sp["OD"] / 2; ua = (SPRING_ROD["u"] + rs + 1.0, BOSS["u"][1]); h_a = ua[1] - ua[0]; b_a = SPRING_ROD["arm_w"]
    e = (ua[0] + ua[1]) / 2 - SPRING_ROD["u"]; M_a = sp["F_open"] * e
    r["spring_arm"] = dict(section=f"{b_a:g} x {h_a:.2f}", offset=round(e, 2), sigma_MPa=round(6 * M_a / (b_a * h_a ** 2) + sp["F_open"] / (b_a * h_a), 2),
                           note="tension + bending at the arm, F open; the -X carrier's arm (below) the same")
    # servo stall on the cam (misuse: a blade jammed mid-stroke): 2 x M2.5 at r 7, PETG bearing over the cam's thickness; roller contact
    T = SERVOS["DS3218"]["kgcm"] * 98.07; t_cam = CAM["y"][1] - CAM["y"][0]
    Fs = T / (2 * 7.0); r["cam_stall"] = dict(T_Nmm=round(T), screw_N=round(Fs, 1), bearing_MPa=round(Fs / (2.5 * t_cam), 1),
                                             index_misuse=round(Fs / (2.5 * t_cam) / 10.0, 2),
                                             roller_N=round(T / (2 * 6.0), 0),
                                             roller_hertz_MPa=round(math.sqrt((T / 12.0) * 2200.0 / (math.pi * 5.0 * B623["w"])), 0),
                                             note="the roller's line contact on the PETG cam at a full DS3218 stall dents it (PETG ~50 MPa): the box "
                                                  "electronics limits the servo (stall current cut, 0.5 s), the cam is a 1 g spare")
    r["note"] = ("the carriage plate (low stack) is the weak link for the 5.2 / 6.7 N.m (lever 90.65 to the plate, C) head moment; the upper-bound FE (the whole design "
                 "space filled solid, runs/head/v7/bound_*.json) decides: plate top 74.35 (DEFAULT, asked of the arm system) and the plate / "
                 "root grown below u 0; the hand numbers above are for the 4.0 plate")
    return r


def loads_table():
    """loads per case at the head's interfaces (head A frame, N and N.mm; E), the topology run's load cases"""
    pull = LOADS["pull_unmate"]; Ov = LOADS["cart_g"]["overseer"] / 1000 * 9.81
    return {
        "insert at the drone 60 N (swap)": dict(at="deck pads (u 15.4, X +-16.0..18.2, Y 155..175)", F=[0, 0, -60.0], cls="swap"),
        "insert stall 74 N (misuse)": dict(at="deck pads", F=[0, 0, -74.0], cls="misuse"),
        "pull 57.3 N while the XT60 unmates (swap)": dict(at="rod sockets (4 ends, X +-30.5, Y 161 / 169, u 6.75)", F=[0, 0, pull], cls="swap",
                                                          note="carriers -> rods: 14.3 N per rod end"),
        "squeeze reaction (internal)": dict(at="spring between the carriers, cam on the rollers", F=[0, 0, 0], cls="swap",
                                           note="the spring is between the carriers and the cam forces cancel: nothing on the body but the servo torque 0.24 N.m at the ear pads"),
        "handling 3 g radial + 2 g along X, Overseer cartridge carried": dict(at="pads (radial out) / rib pockets (radial in) and the stops (X)",
                                                                             F=[round(2 * Ov, 1), 0, round(3 * Ov, 1)], cls="handling",
                                                                             cg="cartridge CG at Y 170.2, u 48.5 (latch: swap CG y 33.1, z_T 5.18 off)"),
        "servo stall on the cam (misuse)": dict(at="ear pads", T_Nmm=round(SERVOS["DS3218"]["kgcm"] * 98.07, 0), cls="misuse",
                                               note="the servo driven against a jammed blade: 1.96 N.m into the ear pads; the cam / rollers take 2 x 93 N at r 10.5"),
        "across 43.5 N at a seat, receiver gaps closed (swap)": dict(at="the inward stops (through a blade)", F=[43.5, 0, 0], cls="swap"),
        "centring 7.6 N + 1.05 N.m at a seat, driver on (swap)": dict(at="stops (X) + pads (couple)", F=[7.6, 0, 0], T_Nmm=1050.0, cls="swap"),
        "handling 2 g along Y, Overseer cartridge carried": dict(at="rod sockets (rib -> blades -> rods)", F=[0, round(2 * Ov, 1), 0], cls="handling"),
        "power cut carrying (braked)": dict(at="bumper notch", F=[0, 0, 3.0], cls="misuse",
                                            note="the head brake is required (System box swap: arm): 7.8 mm/s creep, 3 N on the bumper (runs/arm/v7/system.json)"),
        "supports": "the three wheel bores hold X and Y (the wheels roll along u); the pinion holds u and X on the bonded rack bar's teeth at the case's stroke (drone u -56.3, bay -46.8, park 23)",
    }


def masses():
    """mass estimate (E): drawn + bought; the body from its topology run"""
    rho = 1.27e-3
    pc = {n: round(tm(m).volume * rho, 1) for n, m in parts().items() if n in ("carrier+", "carrier-", "cam")}
    pc["rack bar"] = round(tm(rack_bar()).volume * rho, 1)
    pc["servo retainer"] = round(tm(retainer()).volume * rho, 1)
    bought = dict(servo_DS3218=60.0, servo_MG995=55.0, horn=6.0, rods_carbon=round(2 * 64 * 0.011 + 63 * 0.011, 1), spring=1.0,
                  rollers_623ZZ=2 * B623["g"], halls=1.0, screws_inserts=4.0)
    return dict(printed_parts=pc, bought=bought, body_E=75.0, note="body from the topology run (E 75 g, v5 head 56 M + carriage)")


BOUND_RESULTS = {   # upper-bound FE (head_v7_topo.bound, the design space filled solid, h 1.5, worst index in the swap class, E loads)
    "plate top 72.75, root over u 0..14.9 only": {"+y": 3.96},
    "plate top 72.75, plate / root grown to u -18": {"+y": 1.03, "+z": 0.73},
    "plate top 74.35, plate / root grown to u -18, all-direction supports (r2)": {"+y": 0.70, "+z": 0.61, "+x": 0.44},
    "r3: partial supports (bores X Y, rack teeth u X), across + Y loads, stops, bonded rack bar, bed land, sharp plate / wing corner": {"+x": 0.59},
    "r3: as above with the 12.5 x 10 plate / wing fillet": {"+x": 0.41},
    "r4: + the pinion's separating force, teeth hold u only, the tail rib, the servo window (head A / head B)": {"+x A": 0.434, "+x B": 0.484},
    "r5: + column on the head lane (u -16.3), retainer bosses, exact keep-out sweep, deck |X| 18.2, CG offset couples, stall reacted at the rods": {"+x A": 0.406, "+x B": 0.479},
    "r6: + the cradle kept (servo pocket only), wire channels, Ø12 retainer bosses": {"+x A": 0.421, "+x B": 0.48},
    "r7: + servo dims from dimensions.md M, the lead and socket slot, the deck slot = blade travel + 1, the cradle clipped at the park bisector": {"+x A": 0.407, "+x B": 0.48},
    "r8: + the JR pair along X over the retainer's tab, the hall boards past the cam, the roller collar, the horn keep-out to its disc (chosen)": {"+x A": 0.415, "+x B": 0.479},
}

ACCESS = [
    "servo: command OPEN first (the open dwell holds the carriers apart unpowered), then 2 M3 countersunk out of the retainer along +u through the exit window (driver swept), the retainer off, the servo with horn and cam out along -u (swept against the arm and the head's own carriers and rollers); service pose head A t 11.5 (phi 90, A out at z0 108.6); head B: retainer screws at t 21.5 (phi -135, both parked), the servo out at t 18..19.5 (held by hand between; checked against head A's whole envelope: recheck on the real bodies)",
    "rods: pushed in from +X through the +X walls, through both bosses, into blind sockets in the -X walls, epoxied",
    "spring rod: through the +X wall, the seat plates and the spring, into the -X wall",
    "near axle (22, 4): 3 mm L-key from +X over the plate (window Ø7 kept Y 74.35..90.35); far axle (22, 32) from above (cartridge zone); eccentric flange: pin spanner from above (r 8 kept)",
    "servo in: rest orientation, pushed in along +u; the cam's flanks spread the rollers off their stops by 1.1 each against the spring (~18 N, a hand push), then the retainer",
    "wires: the hall boards (past the cam, Y 155.5..159) each by a riser and a closed run down its side above the servo (|X| 8.8..12.3, u 9.9..12.2, sections >= 2.0 x 2.0 for 3 x 30 AWG) into the ribbon slot; the servo lead over the retainer's tab to the +X end of the mated JR pair laid along X under the case (X 9..38, 4.0 bend room each end), the socket's wires up from its -X end (unplugged from below with the retainer off); the 5-wire head ribbon (servo V G S, hall sum H, 3.3 V; 6.1 x 1.3 E) down an 8.2 x 3.3 slot at X 0, u 7..10.3 through the plate into the rail's 5.0 mouth; tied to the column above the slot (strain relief)",
]


ALTERNATIVES = {
    "spring": [
        ("one crossed spring (chosen)", "both blades 15 N squeezed / 21 N open from one 0.7 wire spring on a rod (L 44 -> 34); the spring force "
         "stays inside the carriers; needs two crossing arms"),
        ("one spring per blade in the wall", "the -X wall meets the park bisector (X + u >= -30.6): 13 mm of spring at the blade's height; "
         "15 N with a 5 mm stroke there needs a 0.7 wire at ~950 MPa (over 0.45 UTS) or a 2x force rise (servo x2)"),
    ],
    "servo": [
        ("shaft along Y under the grip (chosen)", "20 thick in u (u -12.5..7.5), long side along +X: inside the bay wedge and the park bisector "
         "with 1.1 / 3.5 mm; the cam beside the blades; MG995 x3.7 on 2.46 kg.cm"),
        ("shaft along u, cam in the X-Y plane", "the 40.7 body hangs from u ~0 to u -45: crosses the park bisector X + u = -32 by ~25 mm at "
         "X -10 (the parked heads collide) unless the heads differ in Y; rejected"),
    ],
    "cut": [
        ("one body printed on its side + a bonded rack bar (chosen)", "no joint at the highest moment (5.2 / 6.7 N.m (lever 90.65 to the plate, C) at the plate); the "
         "column's bending and the plate's lie in the layers (+x build, upper bound 0.42 / 0.48 (A / B) with the r8 loads and supports; 0.61 / 0.70 with u or Y up in r2); the rack bar "
         "(5.6 g) prints flat with its teeth in the layers and is epoxied over 93 x 13.5 (2 MPa x 1255 mm2 = 2.5 kN vs 74 N)"),
        ("carriage plate printed flat + column bolted on", "the plate and teeth print flat, but the joint sits at the root: 4 M3 inserts "
         "carry ~190..490 N per pair at the moment, the across-layer root is replaced by bolts in PETG bosses (~+8 g), one more fit and "
         "a joint that creeps under 10,000 cycles"),
    ],
}


# ============================================================== picture (sections to scale from the solids)
def section_svg(P, axis, at, view, title, colors, scale=3.0, size=(360, 330), keep=None):
    """slice every part at axis = at; view = (h index, v index) of (X, Y, u) for the drawing; returns svg"""
    W, H = size; idx = {"x": 0, "y": 1, "u": 2}[axis]
    normal = [0, 0, 0]; normal[idx] = 1; origin = [0, 0, 0]; origin[idx] = at
    hi, vi = view
    polys = []
    for nm, m in P.items():
        if keep and nm not in keep: continue
        t = tm(m)
        try:
            s = t.section(plane_origin=origin, plane_normal=normal)
        except Exception:
            s = None
        if s is None: continue
        for d in s.discrete:
            polys.append((nm, d[:, hi], d[:, vi]))
    if not polys: return ""
    allh = np.concatenate([p[1] for p in polys]); allv = np.concatenate([p[2] for p in polys])
    cx, cy = (allh.min() + allh.max()) / 2, (allv.min() + allv.max()) / 2
    sc = min(scale, 0.9 * W / max(1, allh.max() - allh.min()), 0.9 * (H - 30) / max(1, allv.max() - allv.min()))
    X = lambda h: W / 2 + (h - cx) * sc; Yp = lambda v: (H + 20) / 2 - (v - cy) * sc
    g = [f'<svg viewBox="0 0 {W} {H}" width="{W}" height="{H}" role="img" aria-label="{title}"><text x="8" y="16" class="h">{title}</text>']
    byname = {}
    for nm, h, v in polys: byname.setdefault(nm, []).append((h, v))
    for nm, loops in byname.items():
        d = " ".join("M" + " L".join(f"{X(a):.1f},{Yp(b):.1f}" for a, b in zip(h, v)) + " Z" for h, v in loops)
        g.append(f'<path d="{d}" class="p" fill-rule="evenodd" style="fill:{colors.get(nm, "var(--muted)")}"><title>{nm}</title></path>')
    # 10 mm scale bar
    g.append(f'<line x1="{W - 18 - 10 * sc:.1f}" y1="{H - 10}" x2="{W - 18}" y2="{H - 10}" class="sb"/><text x="{W - 18 - 5 * sc:.1f}" y="{H - 14}" class="s" text-anchor="middle">10 mm</text>')
    g.append("</svg>")
    return "\n".join(g)


COLORS = {"design space": "var(--ds)", "carrier+": "var(--blade)", "carrier-": "var(--blade2)", "roller+": "var(--roll)", "roller-": "var(--roll)", "cam": "var(--cam)",
          "horn": "var(--horn)", "servo": "var(--servo)", "rods": "var(--rod)", "spring rod": "var(--rod)", "spring": "var(--spring)",
          "stops": "var(--pin)", "body (drawn interfaces)": "var(--body)", "cartridge": "var(--cart)", "rack bar": "var(--spring)",
          "tail rib": "var(--cam)", "retainer": "var(--pin)", "wiring": "#d0312d"}


def cartridge_solid(pack="scout", squeeze=4.0):
    """the latch cartridge in the head frame (carried: flat bottom at u 15.4), tabs squeezed"""
    try:
        import arm_kinematics as K, arm_v7_layout as S7
        S7.use(); K.YC = Y_C
        m = K.latch_cartridge_local(squeeze, pack, 0.0)
        return m.translate([0, 0, U_BOT])
    except Exception:
        return None


def park_line(y0, y1, u0=-60.0, u1=20.0):
    """the bisector between the parked heads, X + u = -32, as a thin slab (for the pictures)"""
    return hull([[-32.0 - u + d, y, u] for u in (u0, u1) for y in (y0, y1) for d in (-0.4, 0.4)])


def alternatives_svgs():
    """the rejected alternatives drawn from the same frame, sliced like the layout (concept review r2: a picture of each)"""
    out = []
    # 1. servo shaft along u: the 40.7 body hangs below the cam; head B's (mirrored, at park) lands on it across the bisector
    sA = box(-10.0, 10.0, 100.0, 140.0, -49.0, -4.0)
    sB = box(-(-4.0 + 32.0), -(-49.0 + 32.0), 100.0, 140.0, -10.0 - 32.0, 10.0 - 32.0)    # B's (X, u) -> A frame (-(u+32), -X-32)
    out.append(section_svg({"servo": sA, "servo B (at park)": sB, "park bisector": park_line(100, 140), "body (drawn interfaces)": body_drawn() ^ box(-60, 60, 119, 121, -80, 80)},
                           "y", 120.0, (0, 2), "Alt: servo shaft along u (section Y 120): A's servo crosses B's at park", dict(COLORS, **{"servo B (at park)": "var(--cam)", "park bisector": "var(--fg)"})))
    # 2. a spring per blade in the walls: the -X spring has 13 to the bisector (the +X one 13 too, for symmetry)
    P = parts()
    spr = cylX(SPRING_ROD["y"] - 16.0, 4.0, -36.0, -23.0, 2.75) + cylX(SPRING_ROD["y"] - 16.0, 4.0, 23.0, 36.0, 2.75)
    out.append(section_svg({"carrier+": P["carrier+"], "carrier-": P["carrier-"], "spring": spr, "park bisector": park_line(150, 180),
                            "rods": P["rods"]}, "y", SPRING_ROD["y"] - 16.0, (0, 2), "Alt: a spring per blade (section Y 165): 13 mm each side",
                           dict(COLORS, **{"park bisector": "var(--fg)"})))
    # 3. carriage plate printed flat + column bolted on: the joint sits where the head moment is largest
    ds = design_space()
    cut = box(-80, 80, PLATE["y"][1] + 0.01, 300, -80, 80)
    joint = box(-28, 28, PLATE["y"][1] - 0.5, PLATE["y"][1] + 2.5, -18, 14.9)
    out.append(section_svg({"design space": ds - cut, "body (drawn interfaces)": (ds ^ cut) - joint, "servo": joint},
                           "x", 0.0, (1, 2), "Alt: plate + bolted column (side, X 0): joint at 5.3 N.m", COLORS, size=(360, 330)))
    return out


def page(R, P_sq, P_op):
    cart = cartridge_solid()
    ds = design_space()
    Ps = dict({"design space": ds}, **P_sq); Po = dict({"design space": ds}, **P_op)
    if cart is not None: Ps = dict(Ps, cartridge=cart)
    sv = []
    sv.append(section_svg(Ps, "y", Y_C, (0, 2), "Section at the tab line Y 165 (X across, u up): squeezed", COLORS))
    sv.append(section_svg(Po, "y", Y_C, (0, 2), "Section at Y 165: open", COLORS))
    sv.append(section_svg(Ps, "y", 154.5, (0, 2), "Section at the cam Y 154.5: rest (rollers clear)", COLORS))
    sv.append(section_svg(Po, "y", 154.5, (0, 2), "Section at the cam Y 154.5: open (cam turned 120 deg)", COLORS))
    sv.append(section_svg(Ps, "y", SPRING_ROD["y"], (0, 2), f"Section at the spring Y {SPRING_ROD['y']:g}: crossed arms, squeezed", COLORS))
    sv.append(section_svg(Ps, "y", 125.0, (0, 2), "Section at the servo Y 125 (ears and pads)", COLORS))
    sv.append(section_svg(Ps, "x", 0.0, (1, 2), "Side section at X 0 (Y along the axle, u up)", COLORS, size=(720, 330)))
    sv.append(section_svg(Ps, "u", RODS["u"], (0, 1), f"Plan section at the rods u {RODS['u']:g} (X across, Y up)", COLORS, size=(360, 420)))
    for h, svn in (("A", "DS3218"), ("B", "MG995")):                         # the rack tail and its rib, per head (r4), each in its own A frame
        PT = {"design space": design_space(svn), "body (drawn interfaces)": body_drawn(svn), "tail rib": tail_rib(svn), "rack bar": rack_bar(svn)}
        sv.append(section_svg(PT, "y", 66.0, (0, 2), f"Head {h}: section at Y 66 through the rack tail and its rib (X across, u up; {'B drawn in its own frame, mirrored on the arm' if h == 'B' else 'A frame'})", COLORS, size=(360, 420)))
        sv.append(section_svg(PT, "x", 36.0, (1, 2), f"Head {h}: section at X 36 along the tail rib (Y across, u up)", COLORS, size=(720, 330)))
    wr = None
    for w_ in wiring("DS3218"): wr = w_ if wr is None else wr + w_
    PR_ = {"design space": design_space(), "servo": servo_solid("DS3218"), "retainer": retainer(), "body (drawn interfaces)": body_drawn(), "wiring": wr}
    sv.append(section_svg(PR_, "u", -14.4, (0, 1), "Plan at u -14.4: the retainer bar under the servo, its two M3 into the bosses, its tab under the servo lead's socket (X across, Y up)", COLORS, size=(360, 420)))
    sv.append(section_svg(PR_, "u", 11.5, (0, 1), "Plan at u 11.5: the hall leads' two runs above the servo into the ribbon slot (wiring red; X across, Y up)", COLORS, size=(360, 420)))
    sv.append(section_svg(PR_, "x", 25.0, (1, 2), "Side section at X 25: the servo lead's socket slot along u and its channel under the case (Y across, u up)", COLORS, size=(720, 330)))
    sp = R["spring"]; cm = R["cam"]; st = R["strength"]
    nums = [
        ("Grip", f"blades 3 x 10, inner faces 6.0 squeezed / 11.0 open (stroke 5.0), rib 0.8 into the grip pocket at u 19.4; tip u 21.4 under the slider bar (u 21.9)"),
        ("Spring (one, crossed)", f"music wire {sp['d']} x OD {sp['OD']} (ID {sp['ID']} on the 3.0 rod), L0 {sp['L0']:g}, {sp['n']} coils, k {sp['k']} N/mm: "
                                  f"{sp['F_sq']} N squeezed (L {sp['L_sq']:g}), {sp['F_open']} N open (L {sp['L_open']:g}), solid {sp['solid']}, tau {sp['tau_open']:.0f} MPa ({sp['tau_ratio']} of 0.45 UTS) (E)"),
        ("Hold and release", f"hold while the XT60 unmates: {st['hold']['per_tab_N']} N per tab opens with {st['hold']['opening_N']} N vs {st['hold']['spring_N']} N: x{st['hold']['margin']}; "
                             f"release margin Scout x{st['release']['scout']}, Overseer x{st['release']['overseer']}; rib bearing {st['rib_bearing_MPa']} MPa"),
        ("Cam and servo", f"{CAM['law']}, base {CAM['base']:g} / lobe {CAM['lobe']:g} (rise {CAM['lobe'] - CAM['base']:g} over {CAM['rise_deg']:g} deg + dwells, turn {cm['turn_deg']:g} deg), 623ZZ rollers, gap at rest {cm['gap_at_rest']}, "
                          f"pressure angle <= {cm['max_pressure_deg']} deg; peak {cm['peak_kgcm']} kg·cm: DS3218 x{cm['margin']['DS3218']}, MG995 x{cm['margin']['MG995']}; spline side load {cm['spline_side_N']}"),
        ("Pull path", f"pocket -> rib -> blade -> boss 12 long -> 2 x 3.0 carbon rods (span {st['rods']['span']:g}: {st['rods']['sigma_MPa']} MPa, index {st['rods']['index']}) -> body walls; "
                      f"anti-rack: guides {st['anti_rack']['guide_length']} apart (boss + spring-rod seat) vs {st['anti_rack']['need']} needed; cam force x{st['anti_rack']['cam_over_spring']} the spring's"),
        ("Push path", f"deck pads X +-{DECK['pad']['x'][0]:g}..{DECK['pad']['x'][1]:g} on the latch frames (both packs; 1.0 off the Scout v6 TPU strips): {st['pads_MPa']['stall']} MPa at the 74 N stall (worst offset)"),
        ("Head moment", f"{st['head_moment_Nm']['pull']} N·m pull / {st['head_moment_Nm']['stall']} N·m stall at the plate (lever {st['head_moment_Nm']['lever']:g}); wheel axial "
                        f"{st['wheel_axial_N_pull']['rigid']:.0f}..{st['wheel_axial_N_pull']['flexible']:.0f} N (pull); spine + plate {st['spine_MPa_pull']} / {st['spine_MPa_stall']} MPa"),
        ("Space (C)", R["space_summary"]),
    ]
    sw = R.get("sweeps", {})
    nums.append(("Arm sweep (C)", "; ".join(f"{k}: {v['states']} states, {v['states_with_overlaps']} with overlaps, min clearance {v['min_clearance']}"
                                           for k, v in sw.items()) or "not run"))
    nums.append(("Upper-bound FE (E)", "; ".join(f"{k}: " + ", ".join(f"{b} {v}" for b, v in d.items()) for k, d in R["bound"].items()) +
                 " (swap target 0.5; peaks at the root's re-entrant corners, across layers)"))
    nums.append(("Compact (C)", R["compact"]))
    nums.append(("Access", "; ".join(ACCESS)))
    if R.get("clearances"):
        nums.append(("Clearances to the envelope (C)", "; ".join(f"{sv}: " + ", ".join(f"{k} {v}" for k, v in d.items()) for sv, d in R["clearances"].items()) +
                     " (moving parts over their travel; the carriers' stop faces are the designed contact)"))
    if R.get("access"):
        nums.append(("Access sweep (C)", "; ".join(f"{k}: open at {v['open_states']} / {v['of']} states" for k, v in R["access"].items())))
    nums.append(("Mass (E)", R["mass_line"]))
    nrows = "".join(f"<tr><th>{a}</th><td>{b}</td></tr>" for a, b in nums)
    alts = "".join(f"<tr><td>{grp}</td><td><b>{a[0][0]}</b>: {a[0][1]}</td><td>{a[1][0]}: {a[1][1]}</td></tr>" for grp, a in ALTERNATIVES.items())
    chg = "".join(f"<li>{c}</li>" for c in CHANGES)
    return f"""<title>Heads v7 layout</title>
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans:wght@400;600&family=IBM+Plex+Mono&display=swap">
<style>
:root {{ --bg:#f6f5f1; --fg:#1d232a; --muted:#5d6670; --line:#c9ccd0; --card:#fff; --blade:#2f6db3; --blade2:#5a8fd0; --roll:#8a6d1f;
  --cam:#b33a3a; --horn:#9aa3ab; --servo:#3d4a57; --rod:#222; --spring:#c47a1d; --pin:#7a4fb3; --body:#a9b4be; --cart:#cfe0c3; --ds:#ece6d6; }}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{ --bg:#14181c; --fg:#e7eaee; --muted:#9aa4ad; --line:#323a42; --card:#1b2026;
  --blade:#6aa5ec; --blade2:#8fbdf3; --roll:#d2b25c; --cam:#e06a6a; --horn:#6c757d; --servo:#56687a; --rod:#ddd; --spring:#e3a24a; --pin:#b18be6;
  --body:#55626e; --cart:#344a33; --ds:#2b2a24; color-scheme: dark }} }}
:root[data-theme="dark"] {{ --bg:#14181c; --fg:#e7eaee; --muted:#9aa4ad; --line:#323a42; --card:#1b2026; --blade:#6aa5ec; --blade2:#8fbdf3;
  --roll:#d2b25c; --cam:#e06a6a; --horn:#6c757d; --servo:#56687a; --rod:#ddd; --spring:#e3a24a; --pin:#b18be6; --body:#55626e; --cart:#344a33; --ds:#2b2a24; color-scheme: dark }}
body {{ background:var(--bg); color:var(--fg); font:15px/1.5 "IBM Plex Sans", system-ui, sans-serif; margin:0;
  padding-left:max(16px, env(safe-area-inset-left, 0px)); padding-right:max(16px, env(safe-area-inset-right, 0px)); padding-block:20px; }}
main {{ max-width:1120px; margin:0 auto; display:grid; gap:18px; }}
h1 {{ font-size:1.5rem; margin:0 }} h2 {{ font-size:1.1rem; margin:0 0 8px }} p {{ margin:0; color:var(--muted); max-width:75ch }}
.card {{ background:var(--card); border:1px solid var(--line); border-radius:10px; padding:16px; min-width:0 }}
.grid {{ display:grid; grid-template-columns:repeat(auto-fill, minmax(340px, 1fr)); gap:12px }}
.scroll {{ overflow-x:auto }}
svg .h {{ font-weight:600; font-size:12px; fill:var(--fg) }} svg .s {{ font-size:10px; fill:var(--muted) }}
svg .p {{ stroke:var(--fg); stroke-width:.4; fill-opacity:.85 }} svg .sb {{ stroke:var(--fg); stroke-width:2 }}
table {{ border-collapse:collapse; width:100%; font-size:14px }} th, td {{ text-align:left; padding:6px 8px; border-top:1px solid var(--line); vertical-align:top }}
th {{ font-weight:600; white-space:nowrap }} ul {{ margin:0; padding-left:18px }}
.legend span {{ display:inline-block; margin-right:12px; font-size:13px }} .sw {{ display:inline-block; width:11px; height:11px; border-radius:2px; vertical-align:-1px; margin-right:4px }}
</style>
<main>
<header><h1>Heads v7 layout</h1><p>Nox swap arm heads A (DS3218) and B (MG995, mirrored). Sections cut from the layout's solids, to scale. The servo lies under the grip; one crossed spring closes both blades and holds the cartridge with the servo off; the cam opens them; the pull goes through carbon rods into the body, never the servo.</p></header>
<section class="card"><h2>Sections</h2><div class="legend">
<span><i class="sw" style="background:var(--blade)"></i>carrier +X</span><span><i class="sw" style="background:var(--blade2)"></i>carrier −X</span>
<span><i class="sw" style="background:var(--roll)"></i>623ZZ rollers</span><span><i class="sw" style="background:var(--cam)"></i>cam</span>
<span><i class="sw" style="background:var(--horn)"></i>metal horn</span><span><i class="sw" style="background:var(--servo)"></i>servo</span>
<span><i class="sw" style="background:var(--rod)"></i>3.0 carbon rods</span><span><i class="sw" style="background:var(--spring)"></i>spring</span>
<span><i class="sw" style="background:var(--pin)"></i>rack bar</span><span><i class="sw" style="background:var(--body)"></i>body, drawn interfaces</span>
<span><i class="sw" style="background:var(--cart)"></i>Scout cartridge</span><span><i class="sw" style="background:var(--ds)"></i>body design space (topology)</span></div>
<div class="grid">{''.join(f'<div class="scroll">{s}</div>' for s in sv)}</div></section>
<section class="card"><h2>Numbers</h2><table>{nrows}</table></section>
<section class="card"><h2>Alternatives</h2><div class="grid">{''.join(f'<div class="scroll">{a}</div>' for a in alternatives_svgs())}</div><div class="scroll"><table><tr><th></th><th>Chosen</th><th>Alternative</th></tr>{alts}</table></div></section>
<section class="card"><h2>Changes from the system pass's concept</h2><ul>{chg}</ul></section>
<section class="card"><p>3D grip cycle (the same solids): <a href="{MOTION_URL}">head A motion view</a>. Source: <code>nox/box/head_v7_layout.py</code> (space map <code>head_v7_space.py</code>); requirements: <code>nox/dimensions.md</code> "System box swap: arm" and "latch". The body between the drawn interfaces comes from its topology run after the concept check.</p></section>
</main>"""


def latch_parts(pack="scout"):
    """the latch cartridge split for the motion page (head frame, carried): body + pack + plug, and the two sliders vs squeeze"""
    import arm_kinematics as K, swap_system as SS
    BL = SS._BL
    if BL is None: return None
    b = BL.build(pack); g = b["g"]; R = K.latch_rot(BL, g, 0.0)
    R = np.array(R); R[1, 3] = Y_C + g["Z_T"]
    T = lambda m: m.transform(R).translate([0, 0, U_BOT])
    return dict(body=T(b["cart"] + BL.pack_body(g) + BL.xt60_female(g)), slider=lambda sq, side: T(BL.slider(g, sq, side)))


def motion_page(servo="DS3218", path=None):
    """the grip cycle on the head (3D, motion.py): approach open, squeeze, carry, release, leave; head frame fixed"""
    sys.path.insert(0, os.path.join(HERE, ".."))
    from motion import Scene
    P0 = parts(servo, open_=False); cam0 = P0["cam"]
    LP = latch_parts()
    sc = Scene(f"Heads v7 layout: head {'A' if servo == 'DS3218' else 'B'} grip cycle", "concept check: blades on carbon rods, one crossed spring, "
               "cam on the servo, inward stops; drawn interfaces of the body (its topology run joins them); latch v1 Scout cartridge")
    col = dict(body="#a9b4be", carrier="#2f6db3", roller="#8a6d1f", cam="#b33a3a", horn="#9aa3ab", servo="#3d4a57", rod="#222222",
               spring="#c47a1d", pinion="#7a4fb3", cart="#7fae6a", slider="#4f7a3a")
    sc.part("body", tm(P0["body (drawn interfaces)"]), group="head body", color=col["body"], note="drawn interfaces only")
    for nm in ("carrier+", "carrier-"): sc.part(nm, tm(P0[nm]), group="grabber", color=col["carrier"])
    for nm in ("roller+", "roller-"): sc.part(nm, tm(P0[nm]), group="grabber", color=col["roller"], printed=False)
    sc.part("cam", tm(cam0), group="grabber", color=col["cam"])
    sc.part("horn", tm(P0["horn"]), group="bought", color=col["horn"], printed=False)
    sc.part("servo", tm(P0["servo"]), group="bought", color=col["servo"], printed=False)
    sc.part("rods", tm(P0["rods"] + P0["spring rod"]), group="bought", color=col["rod"], printed=False)
    sc.part("spring", tm(P0["spring"]), group="bought", color=col["spring"], printed=False)
    if LP:
        sc.part("cartridge", tm(LP["body"]), group="cartridge", color=col["cart"], printed=False)
        sc.part("slider+", tm(LP["slider"](0.0, 1)), group="cartridge", color=col["slider"], printed=False)
        sc.part("slider-", tm(LP["slider"](0.0, -1)), group="cartridge", color=col["slider"], printed=False)
    turn = CAM["dwell0"] + CAM["rise_deg"] + CAM["dwell1"]
    def rot_cam(th):
        c, s_ = math.cos(math.radians(th)), math.sin(math.radians(th))
        M = np.eye(4); M[0, 0], M[0, 2], M[2, 0], M[2, 2] = c, -s_, s_, c       # +th from X toward u about the Y axis at (0, U_C)
        T1 = np.eye(4); T1[2, 3] = -U_C; T2 = np.eye(4); T2[2, 3] = U_C
        return T2 @ M @ T1
    def tr(x=0.0, y=0.0, u=0.0):
        M = np.eye(4); M[:3, 3] = (x, y, u); return M
    def frame(t):
        # cam turn and cartridge lift vs time: 0..1 approach (open), 1..2 squeeze, 2..3 carry, 3..4 release, 4..5 leave
        if t < 1: th, lift = turn, 30 * (1 - t)
        elif t < 2: th, lift = turn * (1 - (t - 1)), 0.0
        elif t < 3: th, lift = 0.0, 0.0
        elif t < 4: th, lift = turn * (t - 3), 0.0
        else: th, lift = turn, 30 * (t - 4)
        rho = pitch(th); d = min(max(rho - ROLLER["x_sq"], 0.0), STROKE)
        if not (1 <= t <= 4) and t >= 0: d = STROKE
        inner = BLADE["inner_sq"] + d; sq = min(max(LATCH["tab_latched"][1] - inner, 0.0), 4.0) if lift == 0 else 0.0
        L = 2 * SEAT_SQ - 2 * d; S = np.eye(4); S[0, 0] = L / (2 * SEAT_SQ)
        f = {"body": np.eye(4), "carrier+": tr(d), "carrier-": tr(-d), "roller+": tr(d), "roller-": tr(-d), "cam": rot_cam(th),
             "horn": rot_cam(th), "servo": np.eye(4), "rods": np.eye(4), "spring": S}
        if LP: f.update({"cartridge": tr(u=lift), "slider+": tr(-sq, 0, lift), "slider-": tr(sq, 0, lift)})
        return f
    sc.timeline(frame, duration_s=5.0, fps=24)
    sc.view("section", eye=(0, 330, 0), target=(0, 165, 0)); sc.view("side", eye=(260, 140, 0), target=(0, 140, 0))
    out = path or os.path.join(OUT, f"motion_{'A' if servo == 'DS3218' else 'B'}.html")
    sc.write(out)
    return out


def main():
    R = {}
    sp = spring_pick(); R["spring"] = sp
    R["cam"] = cam_numbers(sp)
    R["latch"] = latch_checks()
    R["strength"] = strength(sp)
    R["loads"] = loads_table()
    P_sq = parts(open_=False); P_op = parts(open_=True)
    G = space_grid()
    if G is not None:
        sc = {}
        GA, GB = space_grid("A"), space_grid("B")
        sc["body (drawn interfaces) A"] = space_check({"body": body_drawn("DS3218")}, GA)["body"]      # the bodies differ (rib, servo): each on its own map
        sc["body (drawn interfaces) B"] = space_check({"body": body_drawn("MG995")}, GB)["body"]
        for nm, m in P_sq.items():
            if nm.startswith("body"): continue
            if nm in ("servo",):
                for sv in SERVOS:
                    sc[f"servo {sv}"] = space_check({sv: servo_solid(sv)}, G)[sv]
            else:
                sc[nm] = space_check({nm: m}, G)[nm]
        for nm in ("carrier+", "carrier-"):
            sc[nm + " open"] = space_check({nm: P_op[nm]}, G)[nm]
        R["space"] = sc
        bad = {k: (v["arm_overlap_voxels"], v["min_X_plus_u"]) for k, v in sc.items() if v["arm_overlap_voxels"] or not v["park_ok"]}
        zmin = min(v["Z_min_at_drone"] for k, v in sc.items() if not k.startswith("body"))
        R["space_summary"] = (f"every part clear of the arm's swept parts over 4 cycles and of the park bisector: {'yes' if not bad else bad}; "
                              f"lowest point at the commanded drone insert Z {zmin} (keep-out {KEEPOUT_DRONE['Z'][0]:g})")
    else:
        R["space_summary"] = "space map missing (run head_v7_space.py)"
    R["internal"] = internal_checks()
    R["clearances"] = clearances()
    R["bound"] = BOUND_RESULTS
    import glob
    fa = os.path.join(ROOT, "runs", "head", "v7", "access", "access_v7_scout_-210_30.json")
    R["access"] = json.load(open(fa))["paths"] if os.path.exists(fa) else {}
    R["sweeps"] = {}
    for f in sorted(glob.glob(os.path.join(ROOT, "runs", "head", "v7", "sweep", "sweep_v7_*_space.json"))):
        d = json.load(open(f)); R["sweeps"][d["mode"] + " (envelope)"] = {k: d[k] for k in ("states", "states_with_overlaps", "min_clearance")}
    allm = None
    for nm, m in P_sq.items():
        if nm == "body (drawn interfaces)": continue
        allm = m if allm is None else allm + m
    allm = allm + design_space()
    t = tm(allm); ext = t.bounds[1] - t.bounds[0]
    R["compact"] = (f"head A whole (carriage, rack tail, column as its design space, grabber, servo): {ext[0]:.0f} x {ext[1]:.0f} x {ext[2]:.0f} (X, Y, u) incl. the design "
                    f"space; hull {t.convex_hull.volume / 1e6:.2f} L; inside the drone keep-out's X +-47.5 and Y <= 217; nothing between stacked parts: the "
                    f"servo sits in the column under the grip, the grabber on top of it")
    mm = masses(); R["masses"] = mm
    R["mass_line"] = (f"printed grabber parts {sum(mm['printed_parts'].values()):.1f} g, body ~{mm['body_E']:.0f} g (topology, E), bought "
                      f"{sum(v for k, v in mm['bought'].items() if not k.startswith('servo')) + mm['bought']['servo_DS3218']:.0f} g with the DS3218: head ~"
                      f"{sum(mm['printed_parts'].values()) + mm['body_E'] + sum(v for k, v in mm['bought'].items() if not k.startswith('servo')) + mm['bought']['servo_DS3218']:.0f} g "
                      f"(system frame 120 g head + 60 g carriage, E)")
    R["alternatives"] = ALTERNATIVES; R["changes"] = CHANGES
    json.dump(R, open(os.path.join(OUT, "layout.json"), "w"), indent=1, default=float)
    try:
        R["motion"] = motion_page("DS3218")
    except Exception as e:
        R["motion"] = f"failed: {e}"
    open(os.path.join(OUT, "layout.html"), "w").write(page(R, P_sq, P_op))
    print(json.dumps({k: v for k, v in R.items() if k not in ("loads", "alternatives", "changes")}, indent=1, default=float)[:6000])
    print("->", os.path.join(OUT, "layout.html"))


if __name__ == "__main__":
    main()
