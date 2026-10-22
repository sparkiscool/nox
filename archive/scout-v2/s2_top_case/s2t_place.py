"""S2 top case: placement search (16 Oct 2026, top case chat). Spark 16 Oct 19:46: all electronics move to one top case;
FC on its own grommets, Radxa + UBEC + (DEFAULT) XR2 in the case, flow sensor stays at the bottom.
Exhaustive pose search on a grid. Hard constraints (all from dimensions.md, tags there):
  - prop keep-out: 12 (3D) from each prop sweep cylinder r 44.45 at (+-54.1, +-54.1), z 36.5..44.5 (prop plane 40.5 E, D12)
  - airflow (DEFAULT): nothing above the plate inside a prop disk in plan
  - FC zone: MicoAir743 diamond (L1 radius 25.46 + 1 grommet play) at (1.9, 0), z 22..38 (board 23..33 + plugs 5, E)
  - FC USB-C plug lane at the tail corner, slightly right (E)
  - parts 2 apart (leads, tape), UBEC to Radxa 3 (heat, DEFAULT)
  - topmost part gets a printed roof over it (3: 1 gap + 2.0 skin): the flipped drone lands on print, not a board (DEFAULT)
Objective: case height (top z - 22) + 0.02 x mass-weighted part height + 0.05 x plan reach + 0.01 x lead length.
python s2t_place.py  -> prints the best layouts, writes place.json"""
import json, itertools
import numpy as np

HUBS = np.array([(sx * 54.1, sy * 54.1) for sx in (1, -1) for sy in (1, -1)])
import os
R_DISK, Z_SW0, Z_SW1 = 44.45, 36.5, 44.5
KEEP = float(os.environ.get("S2T_KEEP", 12.0))         # prop keep-out rule (dimensions C), env only for the trade curve
PLATE = 22.0
FC_C, FC_L1, FC_Z = (1.9, 0.0), 25.46 + 1.0, (22.0, 37.0)   # Spark 17 Oct 11:35: FC stack 10 tall (P) + 5 plug room (E)
FIXED = {"fc_usb": ((-50.0, -20.0), (-13.0, 3.0), (23.0, 32.0))}       # E
GAP, GAP_HEAT = 2.0, 3.0
ROOF = 3.0
TAIL_IN = np.array([-45.0, 0.0, 22.0])                                 # battery leads up through the XT60 cut-out (E)
CAMERA = np.array([92.7, 0.0, 5.0])                                    # camera placeholder (dimensions C)

# ---- parts in local coords: list of (name, (x0,x1),(y0,y1),(z0,z1)); 'hard' boxes are solid, 'zone' boxes stay free
# Radxa Zero 3W from Radxa's STEP (dimensions H, C): X 0..65, Y 0..30, Z 0 = PCB top
RADXA = [("pcb", (0, 65), (0, 30), (-3.15, 1.6)),
         ("usb_host", (37.1, 46.0), (-1.5, 5.85), (0, 3.16)), ("usb_otg", (49.6, 58.5), (-1.5, 5.85), (0, 3.16)),
         ("hdmi", (9.2, 15.7), (-1.25, 5.5), (0, 3.03)), ("sd", (-2.0, 4.9), (7.8, 22.3), (-4.46, -1.6)),
         ("cam_plug", (36.1, 47.0), (-13.5, -1.5), (-1.0, 6.0)),       # 90 deg USB-C plug body, E
         ("sd_access", (-8.0, -2.0), (7.8, 22.3), (-4.46, -1.6))]
RADXA_HOLES = [(3.55, 3.60), (3.60, 26.45), (61.40, 3.60), (61.40, 26.50)]
UBEC = [("body", (0, float(os.environ.get("S2T_UL", 55))), (0, float(os.environ.get("S2T_UW", 20))), (0, float(os.environ.get("S2T_UT", 10))))]   # Spark 17 Oct 11:34: 20 x 55 (P), thickness E 10
XR2 = [("body", (0, 22), (0, 13), (0, 5))]                             # E
MASS = {"radxa": 10.5, "ubec": 20.0, "xr2": 1.6}                       # E


def orient(boxes, mode, rot):
    """mode: 'up' (as drawn), 'down' (flipped about local X), 'edge' (standing, local Y -> up, Z -> -y);
    rot: k x 90 deg about z. Returns boxes re-centred so the main body (first box) is centred at the origin in x, y and
    its bottom at z 0."""
    out = []
    for nm, (x0, x1), (y0, y1), (z0, z1) in boxes:
        P = np.array([[x, y, z] for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)], float)
        if mode == "down":
            P[:, 1] *= -1; P[:, 2] *= -1
        elif mode == "edge":                      # standing on the long edge, ports edge (local Y 0) down
            P = np.stack([P[:, 0], -P[:, 2], P[:, 1]], 1)
        elif mode == "edge_up":                   # standing on the long edge, ports edge up
            P = np.stack([P[:, 0], P[:, 2], -P[:, 1]], 1)
        c, s = [(1, 0), (0, 1), (-1, 0), (0, -1)][rot]
        P = np.stack([c * P[:, 0] - s * P[:, 1], s * P[:, 0] + c * P[:, 1], P[:, 2]], 1)
        out.append((nm, P.min(0), P.max(0)))
    lo, hi = out[0][1], out[0][2]
    off = np.array([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])
    return [(nm, a + off, b + off) for nm, a, b in out], off


def rect_dist(px, py, x0, x1, y0, y1):
    dx = np.maximum(0, np.maximum(x0 - px, px - x1)); dy = np.maximum(0, np.maximum(y0 - py, py - y1))
    return np.hypot(dx, dy)


def box_ok(lo, hi):
    """lo, hi: (N,3) arrays of a part's box at N candidate positions. Returns min keep-out distance (3D, to the sweep),
    min plan distance outside the disks, FC-zone clear flag."""
    kd = np.full(len(lo), np.inf); pd = np.full(len(lo), np.inf)
    dz = np.maximum(0, np.maximum(Z_SW0 - hi[:, 2], lo[:, 2] - Z_SW1))
    for hx, hy in HUBS:
        d = rect_dist(hx, hy, lo[:, 0], hi[:, 0], lo[:, 1], hi[:, 1])
        dr = np.maximum(0, d - R_DISK)
        kd = np.minimum(kd, np.hypot(dr, dz)); pd = np.minimum(pd, d - R_DISK)
    # FC diamond prism (L1 ball in plan) x z range
    ddx = np.maximum(0, np.maximum(lo[:, 0] - FC_C[0], FC_C[0] - hi[:, 0]))
    ddy = np.maximum(0, np.maximum(lo[:, 1] - FC_C[1], FC_C[1] - hi[:, 1]))
    fc_hit = (ddx + ddy < FC_L1) & (lo[:, 2] < FC_Z[1]) & (hi[:, 2] > FC_Z[0])
    for (a0, a1), (b0, b1), (c0, c1) in FIXED.values():
        fc_hit |= (lo[:, 0] < a1) & (hi[:, 0] > a0) & (lo[:, 1] < b1) & (hi[:, 1] > b0) & (lo[:, 2] < c1) & (hi[:, 2] > c0)
    return kd, pd, ~fc_hit & (lo[:, 2] >= PLATE)


def candidates(name, boxes, modes, xs, ys, zs):
    rows = []
    X, Y, Z = [a.ravel() for a in np.meshgrid(xs, ys, zs, indexing="ij")]
    T = np.stack([X, Y, Z], 1)
    for mode, rot in itertools.product(modes, range(4)):
        ob, off = orient(boxes, mode, rot)
        ok = np.ones(len(T), bool); kmin = np.full(len(T), np.inf); pmin = np.full(len(T), np.inf)
        for nm, a, b in ob:
            kd, pd, fcok = box_ok(T + a, T + b)
            ok &= fcok; kmin = np.minimum(kmin, kd); pmin = np.minimum(pmin, pd)
        ok &= (kmin >= KEEP) & (pmin >= 0)
        for i in np.flatnonzero(ok):
            rows.append(dict(part=name, mode=mode, rot=rot, t=T[i].tolist(), off=off.tolist(), keep=float(kmin[i]),
                             plan=float(pmin[i]), boxes=[(nm, (T[i] + a).tolist(), (T[i] + b).tolist()) for nm, a, b in ob]))
    return rows


def aabb(c, hard_only=False):
    bx = [(a, b) for nm, a, b in c["boxes"] if not (hard_only and nm in ("cam_plug", "sd_access"))]
    return np.min([a for a, b in bx], 0), np.max([b for a, b in bx], 0)


def sep(c1, c2, gap):
    """True if every box of c1 is >= gap from every box of c2 (zones may not overlap parts either)."""
    for _, a1, b1 in c1["boxes"]:
        for _, a2, b2 in c2["boxes"]:
            a1_, b1_, a2_, b2_ = map(np.asarray, (a1, b1, a2, b2))
            if np.all(a1_ - gap < b2_) and np.all(a2_ - gap < b1_):
                return False
    return True


def centre(c):
    a, b = aabb(c, True); return (a + b) / 2


def port(c, nm):
    for n, a, b in c["boxes"]:
        if n == nm: return (np.asarray(a) + np.asarray(b)) / 2


def score(sel):
    tops = [aabb(c, True)[1][2] for c in sel]
    top = max(tops) + ROOF
    m = np.array([MASS[c["part"]] for c in sel]); zc = np.array([centre(c)[2] for c in sel])
    cg = float((m * (zc - PLATE)).sum() / m.sum())
    reach = max(float(np.hypot(*aabb(c)[k][:2])) for c in sel for k in (0, 1))
    R = {c["part"]: c for c in sel}
    lead = (np.linalg.norm(centre(R["ubec"]) - TAIL_IN) + np.linalg.norm(centre(R["ubec"]) - centre(R["radxa"]))
            + np.linalg.norm(port(R["radxa"], "usb_host") - CAMERA) + np.linalg.norm(centre(R["xr2"]) - np.array([1.9, 0, 33])))
    J = (top - PLATE) + 0.02 * cg + 0.05 * reach + 0.01 * lead
    return J, dict(top=top, cg=cg, reach=reach, lead=float(lead))


def arr(rows):
    lo = np.array([r["boxes"][0][1] for r in rows]); hi = np.array([r["boxes"][0][2] for r in rows]); return lo, hi


def clear_of(c, lo, hi, gap):
    ok = np.ones(len(lo), bool)
    for _, a, b in c["boxes"]:
        a = np.asarray(a); b = np.asarray(b)
        ok &= ~(np.all(a - gap < hi, 1) & np.all(lo - gap < b, 1))
    return ok


if __name__ == "__main__":
    zs = np.arange(22.0, 90.0, 0.5)
    rad = candidates("radxa", RADXA, ("up", "down", "edge", "edge_up"), np.arange(-10, 10.1, 1.0), np.arange(-10, 10.1, 1.0), zs)
    print("radxa feasible", len(rad))
    for m in ("up", "down", "edge", "edge_up"):
        sub = [c for c in rad if c["mode"] == m]
        if sub:
            print(f"  {m}: lowest top {min(aabb(c, True)[1][2] for c in sub):.2f}")
    zmin = min(aabb(c, True)[1][2] for c in rad)
    radb = [c for c in rad if aabb(c, True)[1][2] <= zmin + 5]
    print(f"radxa lowest top {zmin:.2f}; kept {len(radb)} within +5")
    ube = candidates("ubec", UBEC, ("up", "edge"), np.arange(-70, 70.1, 2.0), np.arange(-40, 40.1, 2.0), np.arange(22.0, 80.0, 1.0))
    xr = candidates("xr2", XR2, ("up", "edge"), np.arange(-60, 60.1, 3.0), np.arange(-40, 40.1, 3.0), np.arange(22.0, 80.0, 1.5))
    print("ubec feasible", len(ube), "xr2 feasible", len(xr))
    ulo, uhi = arr(ube); xlo, xhi = arr(xr)
    um = np.array([MASS["ubec"]] * len(ube))
    res = []
    for r in radb:
        ra, rb = aabb(r, True); rt = rb[2]
        ok = clear_of(r, ulo, uhi, GAP_HEAT)
        if not ok.any(): continue
        # cheap pre-score for the ubec: its own terms; then full score for the 30 best with an xr2
        uc = (ulo + uhi) / 2
        pre = 0.02 * (MASS["ubec"] * (uc[:, 2] - PLATE)) / 32 + 0.05 * np.maximum(np.hypot(ulo[:, 0], ulo[:, 1]), np.hypot(uhi[:, 0], uhi[:, 1])) \
            + 0.01 * (np.linalg.norm(uc - TAIL_IN, axis=1) + np.linalg.norm(uc - (ra + rb) / 2, axis=1))
        idx = np.flatnonzero(ok)[np.argsort(pre[ok])[:30]]
        for i in idx:
            u = ube[i]
            okx = clear_of(r, xlo, xhi, GAP) & clear_of(u, xlo, xhi, GAP)
            if not okx.any(): continue
            xc = (xlo + xhi) / 2
            px = np.linalg.norm(xc - np.array([1.9, 0, 33]), axis=1) + 0.5 * xc[:, 2]
            j = np.flatnonzero(okx)[np.argmin(px[okx])]
            J, info = score([r, u, xr[j]]); res.append((J, info, r, u, xr[j]))
    res.sort(key=lambda s: s[0])
    seen = set(); top = []
    for J, info, r, u, x in res:
        key = (r["mode"], r["rot"] % 2, u["mode"], u["rot"] % 2)
        if key in seen: continue
        seen.add(key); top.append((J, info, r, u, x))
        if len(top) == 8: break
    for J, info, r, u, x in top:
        print(f"J {J:.2f} top {info['top']:.1f} cg {info['cg']:.1f} reach {info['reach']:.1f} lead {info['lead']:.0f} | "
              f"radxa {r['mode']} rot{r['rot']} t {np.round(r['t'],1).tolist()} keep {r['keep']:.1f} plan {r['plan']:.2f} | "
              f"ubec {u['mode']} rot{u['rot']} t {np.round(u['t'],1).tolist()} keep {u['keep']:.1f} | "
              f"xr2 {x['mode']} rot{x['rot']} t {np.round(x['t'],1).tolist()}")
    J, info, r, u, x = top[0]
    json.dump(dict(J=J, info=info, radxa=r, ubec=u, xr2=x), open("place.json", "w"), indent=1)
