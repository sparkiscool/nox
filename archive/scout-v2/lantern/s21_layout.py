"""Scout v2.1 layout sweep. Drone coords: x nose+, y left+, z up, mm. Plate top z 22.
Tags: P measured, C CAD, E estimate. Sources: nox/v2/dimensions.md (C, E, G, H, I, J)."""
import itertools, json, math

# ---------------- fixed geometry (from dimensions.md) ----------------
MOTORS = [(54.1, 54.1), (-54.1, 54.1), (54.1, -54.1), (-54.1, -54.1)]  # P
R_DISK = 44.45; Z_SWEEP = (37.2, 45.2); KEEP = 12.0                     # C / E
PLATE = dict(x=(-50, 50), y=(-47, 47), z=(18.5, 22))                    # E / listing
CART = dict(x=(-41.5, 37.5), y=(-19, 19), z=(-44.5, 3.5))               # C (v3, bottom layout)
RECV_CEIL_TOP = 3.5 + 1.5 + 2.0   # TPU preload strip + 2.0 ceiling plate (E)
AXLE_Z = -144.5                   # wheel axle in drone z (seat z0 = W Z 144.5) C/E
BAY_X = (-46, 42)                 # bay modules Y 94..182 -> x = 136 - Y (C)
HEAD_LANE = dict(x=(-47, 39), y=(-26, 14))  # head body u -26..14 (C), below the receiver
ARM_SWEEP = dict(x=(42, 84), zmax=-9.5)      # bars Y 74..94 + belts/idlers Y 56..72, r<=134 (C)
OPENING = 84.0; YAW = math.radians(5)

def pts(box):  # 8 corners + face centres
    (x0, x1), (y0, y1), (z0, z1) = box['x'], box['y'], box['z']
    c = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    c += [((x0+x1)/2, (y0+y1)/2, z0), ((x0+x1)/2, (y0+y1)/2, z1)]
    return c

def inside(p, box, grow=0.0):
    return all(box[k][0]-grow <= v <= box[k][1]+grow for k, v in zip('xyz', p))

def violations(box, name):
    out = []
    for (x, y, z) in pts(box):
        # props: 12 off the swept disks (3D). In-plan rule for z inside sweep +-12
        if Z_SWEEP[0]-KEEP <= z <= Z_SWEEP[1]+KEEP:
            for mx, my in MOTORS:
                d = math.hypot(x-mx, y-my) - R_DISK
                dz = 0 if Z_SWEEP[0] <= z <= Z_SWEEP[1] else min(abs(z-Z_SWEEP[0]), abs(z-Z_SWEEP[1]))
                if d < 0 and dz < KEEP or math.hypot(max(d, 0), dz) < KEEP:
                    out.append(f'{name}: {KEEP} prop keep-out at ({x:.0f},{y:.0f},{z:.0f})'); break
        if inside((x, y, z), PLATE): out.append(f'{name}: inside plate')
        if inside((x, y, z), CART) or (inside((x, y, z), dict(CART, z=(-44.5, RECV_CEIL_TOP))) ):
            out.append(f'{name}: inside cartridge/receiver')
        # box: arm sweep under the nose
        if ARM_SWEEP['x'][0] <= x <= ARM_SWEEP['x'][1] and z <= ARM_SWEEP['zmax']:
            out.append(f'{name}: box arm/belt sweep (x 42..84 below z -9.5)')
        # box: bay sector (+-22.5 deg) for z below r 150
        r = z - AXLE_Z
        if BAY_X[0] <= x <= BAY_X[1] and r < 150 and abs(y) > r*math.tan(math.radians(22.5)) - 1:
            out.append(f'{name}: bay sector')
        # box: head lane under the receiver
        if z < -0.8 and HEAD_LANE['x'][0] <= x <= HEAD_LANE['x'][1] and HEAD_LANE['y'][0] <= y <= HEAD_LANE['y'][1] and not inside((x,y,z), CART, 0.5):
            out.append(f'{name}: swap head lane')
        # collar faces above the opening, 168 opening below it (5 deg yaw)
        if z > -0.8:
            lim = OPENING + (z + 0.8) - 1.0
            if abs(x) > lim or abs(y) > lim: out.append(f'{name}: collar face')
        else:
            if max(abs(x), abs(y))*(math.cos(YAW)+math.sin(YAW)) > OPENING: out.append(f'{name}: 168 opening')
    return sorted(set(out))

def box(cx, cy, cz, lx, ly, lz):
    return dict(x=(cx-lx/2, cx+lx/2), y=(cy-ly/2, cy+ly/2), z=(cz-lz/2, cz+lz/2))
def ctr(b): return tuple((b[k][0]+b[k][1])/2 for k in 'xyz')
def dist(a, b): return math.dist(a, b)

# ---------------- candidates ----------------
# FC 36x36x8 board (P), diamond in plan; as a box use the diamond's bounding 36 side rotated: check corners on axes
def fc_box(z0):  # represent as its 4 diamond corners + centre (plan) at z0..z0+8
    return dict(x=(1.9-25.5, 1.9+25.5), y=(-25.5, 25.5), z=(z0, z0+8)), [(1.9+25.5, 0), (1.9-25.5, 0), (1.9, 25.5), (1.9, -25.5), (1.9+12.75, 12.75), (1.9-12.75, 12.75), (1.9+12.75, -12.75), (1.9-12.75, -12.75)]
FC = {'top (plate top, 4 standoffs)': 26.0, 'under plate (v1 mount, z 9.5..18.5)': 10.5}
RADXA = {  # 65 x 30 x 9.2 (C/P)
    'L wing flat (ports out +y)': box(0, 38.5, 13.2, 65, 30, 9.2),
    'R wing flat (ports out -y)': box(0, -38.5, 13.2, 65, 30, 9.2),
    'axis flat under plate': box(0, 0, 13.2, 65, 30, 9.2),
    'L edge beside pack': box(0, 27.6, 3, 65, 9.2, 30),
    'tail bay flat': box(-70, 0, 12, 65, 30, 9.2),
}
UBEC = {  # 30 x 16 x 7 (E) + RX 15 x 11 x 3 (E) together as one 32 x 18 x 10 block (E)
    'R wing': box(0, -32, 12.8, 32, 18, 10), 'L wing': box(0, 32, 12.8, 32, 18, 10),
    'tail bay': box(-62, 0, 12, 32, 18, 10), 'axis under plate': box(0, 0, 12.8, 32, 18, 10),
}
CAM = (99.0, 0.0, 41.0)  # board back face x 99, lens on the prop plane in the nose window (this chat, C)
PINS = [(0, 21.5, -3.0), (0, -21.5, -3.0)]  # pin cups at the receiver sides, z = -44.5 + 41.5 (C)

def fc_viol(z0):
    b, diamond = fc_box(z0)
    out = []
    for (x, y) in diamond:
        for z in (z0, z0+8):
            if Z_SWEEP[0]-KEEP <= z <= Z_SWEEP[1]+KEEP:
                for mx, my in MOTORS:
                    d = math.hypot(x-mx, y-my) - R_DISK
                    dz = 0 if Z_SWEEP[0] <= z <= Z_SWEEP[1] else min(abs(z-Z_SWEEP[0]), abs(z-Z_SWEEP[1]))
                    if (d < 0 and dz < KEEP) or math.hypot(max(d, 0), dz) < KEEP:
                        out.append(f'FC: prop keep-out at ({x:.0f},{y:.0f},{z:.0f})')
            if inside((x, y, z), PLATE): out.append('FC: inside plate')
            if z < RECV_CEIL_TOP + 2.0 and inside((x, y, z), CART, 0): out.append('FC: on the cartridge')
    return sorted(set(out)), b

def overlap(a, b, gap=2.0):
    return all(a[k][0] - gap < b[k][1] and b[k][0] - gap < a[k][1] for k in 'xyz')

results = []
for (fn, fz), (rn, rb), (un, ub) in itertools.product(FC.items(), RADXA.items(), UBEC.items()):
    v, fb = fc_viol(fz)
    v += violations(rb, 'Radxa') + violations(ub, 'UBEC+RX')
    if overlap(rb, ub): v.append('Radxa / UBEC overlap')
    if fn.startswith('under') and overlap(fb, rb): v.append('FC / Radxa overlap')
    if fn.startswith('under') and overlap(fb, ub): v.append('FC / UBEC overlap')
    if fn.startswith('under') and fb['z'][0] - RECV_CEIL_TOP < 2.0: v.append('FC to receiver ceiling < 2')
    fc_c = ctr(fb); r_c = ctr(rb); u_c = ctr(ub)
    wires = (dist(fc_c, r_c) + dist(fc_c, u_c) + dist(r_c, u_c) + dist(r_c, CAM)
             + sum(dist(p, fc_c) for p in PINS) + dist(u_c, PINS[1]))
    # CG: masses E: FC 8, Radxa 15, UBEC+RX 10 (g); rest symmetric
    m = [(8, fc_c), (15, r_c), (10, u_c)]
    cgx = sum(w*c[0] for w, c in m)/420; cgy = sum(w*c[1] for w, c in m)/420
    height = max(fb['z'][1], rb['z'][1], ub['z'][1]) - min(rb['z'][0], ub['z'][0], -44.5)
    J = wires + 40*abs(cgx) + 40*abs(cgy) + 2*height + 1000*len(v)
    results.append((J, fn, rn, un, round(wires), round(cgx, 2), round(cgy, 2), round(height, 1), v))
results.sort(key=lambda r: r[0])
print('rank  J     FC | Radxa | UBEC+RX | wires cgx cgy height | violations')
for r in results[:12]:
    print(f'{r[0]:7.0f}  {r[1]} | {r[2]} | {r[3]} | {r[4]} {r[5]} {r[6]} {r[7]} | {r[8]}')
best = results[0]
json.dump(dict(fc=best[1], radxa=best[2], ubec=best[3], fc_box=fc_box(FC[best[1]])[0], radxa_box=RADXA[best[2]], ubec_box=UBEC[best[3]]),
          open('s21_best.json', 'w'), indent=1)
print('\nCamera check:', violations(dict(x=(99, 100.6), y=(-19, 19), z=(22, 60)), 'camera board'))
print('Flow check:', violations(dict(x=(63, 85), y=(-10, 6), z=(2, 10)), 'flow sensor'))
print('Flow at v3.3 spot:', violations(dict(x=(47, 69), y=(-10, 6), z=(-24, -16)), 'flow v3.3'))
