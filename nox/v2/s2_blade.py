"""S2 latch return = two feeler-gauge blades (17 Oct 11:07, Spark: same feeler as Scout v1 latch arm V6).
Replaces the Ø3 pen spring of block v3. Blade: Precision Brand 19425, 0.012 in x 1/2 in, 1095 (0.305 x 12.7, P),
cut 35.5 (30.5 free + 5.0 clamp), deburr both ends, scuff the clamp end, epoxy.
Layout (blade_sweep.py, DEFAULT): one blade per slider, both from the lead end, cantilevered along z under the tray
floor; root slot in a post at z 71..77 (open to the flat bottom: drop the blade in from below, push it up to the slot
roof); the tip edge bears on the bottom of a tapered groove in its own tab's inner face, 4.0 in from the tab's
lead-end face. Free +x face at x 5.0, groove bottom at 4.5: 0.5 preload. Coords as s2_cartridge.py (tray coords)."""
import math, numpy as np, trimesh
import s2_cartridge as C
from s2_cartridge import B, U, D, mirror_x

T_B, W_B = 0.305, 12.7          # P
E = 205e3                       # MPa, E
Y_B1 = 12.95                    # blade top = slot roof (C); blade y 0.25..12.95
Y_B0 = Y_B1 - W_B
GROOVE = 0.5                    # groove bottom 0.5 into the tab at the tip (C)
D0 = 0.5                        # preload at rest (C, sweep)
Z_ROOT = 71.0                   # free length starts here (C, sweep)
A_TIP = 4.0                     # tip lands 4.0 in from the tab's lead-end face (C, sweep)
Z_TIP = C.Z_BLK1 - A_TIP        # 39.6
L = Z_ROOT - Z_TIP              # 30.5 free (C)
CLAMP = 5.0
X_FREE = C.TAB_IN + GROOVE + D0 # 5.0: +x face of the unloaded blade (= root)
CL = 0.1                        # running clearance round the swept blade (C)
POST = (Z_ROOT, Z_ROOT + 6.0)   # root post z 71..77 to the end wall
I = W_B * T_B**3 / 12
K = 3 * E * I / L**3

def w(s, d):                    # tip-loaded cantilever, s from the root
    return d * s**2 * (3 * L - s) / (2 * L**3)

def blade(d, s_side=1, dil=0.0, tip_exact=True, n=48, clamp=True):
    """+x blade (mirror for -x) with tip deflection d (toward -x). dil grows it on every side; with tip_exact the
    bearing (+x) face of the last 1.0 is not grown, so the groove bottom stays where the tip edge bears."""
    parts = []
    ss = np.linspace(0, L, n + 1)
    for s0, s1 in zip(ss[:-1], ss[1:]):
        pts = []
        for s in (s0, s1):
            xf = X_FREE - w(s, d)
            grow_out = 0.0 if (tip_exact and s > L - 1.0) else dil
            z = Z_ROOT - s
            for x in (xf - T_B - dil, xf + grow_out):
                for y in (Y_B0 - dil, Y_B1 + dil):
                    pts.append([x, y, z])
        parts.append(trimesh.convex.convex_hull(np.array(pts)))
    # tip end cap grows in z by dil, root runs straight into the clamp
    zt = Z_TIP - dil
    xt = X_FREE - d
    parts.append(B(xt - T_B - dil, xt + (0 if tip_exact else dil), Y_B0 - dil, Y_B1 + dil, zt, Z_TIP + 0.01))
    if clamp:
        parts.append(B(X_FREE - T_B - dil, X_FREE + dil, Y_B0 - dil, Y_B1 + dil, Z_ROOT - 0.01, Z_ROOT + CLAMP + dil))
    m = U(*parts)
    return m if s_side > 0 else mirror_x(m)

def sweep(s_side, dil, frame_shift=False, states=9, tip_exact=False):
    """union of the blade over the squeeze; frame_shift=True expresses it in the moving tab's frame (+q for +x)."""
    out = []
    for q in np.linspace(0, C.TRAVEL, states):
        b = blade(D0 + q, s_side, dil, tip_exact)
        if frame_shift:
            b = b.copy(); b.apply_translation([s_side * q, 0, 0])
        out.append(b)
    return U(*out)

def post(s_side):
    p = B(X_FREE - T_B - 1.5, X_FREE + 1.5, 0, C.Y_FLOOR1, POST[0], POST[1])
    slot = B(X_FREE - T_B - 0.05, X_FREE + 0.05, -1, Y_B1, POST[0] - 0.1, Z_ROOT + CLAMP)   # open to the bottom
    m = D(p, slot)
    return m if s_side > 0 else mirror_x(m)

def block_v4(parts=False):
    fr, s1, s2 = C.block(parts=True)
    # spring seats out: fill the Ø3.2 holes back (rebuild tabs' seat region solid)
    for s, sl in ((1, s1), (-1, s2)):
        a, b_ = sorted((s * C.TAB_IN, s * (C.TAB_IN + C.SPR_DEPTH + 0.2)))
        plug = B(a, b_, C.SPR_Y - 2, C.SPR_Y + 2, C.T / 2 - 2, C.T / 2 + 2)
        plug = plug.intersection(B(a, b_, 0, C.BAR_Y1, 0, C.T), engine="manifold")
        if s > 0: s1 = U(s1, plug)
        else: s2 = U(s2, plug)
    sh = [0, 0, -C.Z_BLK0]
    g1 = sweep(1, CL, frame_shift=True, tip_exact=True); g1.apply_translation(sh)
    g2 = sweep(-1, CL, frame_shift=True, tip_exact=True); g2.apply_translation(sh)
    s1, s2 = D(s1, g1), D(s2, g2)
    f1 = sweep(1, CL); f2 = sweep(-1, CL)
    for f in (f1, f2): f.apply_translation(sh)
    fr = D(D(fr, f1), f2)
    if parts: return fr, s1, s2
    return U(fr, s1, s2)

def tray_cut():
    return U(sweep(1, CL), sweep(-1, CL))

def tray_add():
    return U(post(1), post(-1))

if __name__ == "__main__":
    import os
    os.makedirs("out", exist_ok=True)
    print("L %.2f, k %.3f N/mm, rest %.2f N, full %.2f N, strain full %.3f %%, blade cut %.1f, mass %.2f g each" % (
        L, K, K * D0, K * (D0 + C.TRAVEL), 150 * T_B * (D0 + C.TRAVEL) / L**2, L + CLAMP, T_B * W_B * (L + CLAMP) * 7.85e-3))
    fr, s1, s2 = block_v4(parts=True)
    tr3 = D(U(C.tray(), tray_add()), tray_cut())
    # interference over the squeeze, both sides independently (4 corners + mid)
    worst = {}
    def add(k, v): worst[k] = max(worst.get(k, 0), v)
    for qa in np.linspace(0, C.TRAVEL, 5):
        for qb in (0.0, C.TRAVEL / 2, C.TRAVEL):
            A = s1.copy(); A.apply_translation([-qa, 0, C.Z_BLK0])
            Bm = s2.copy(); Bm.apply_translation([qb, 0, C.Z_BLK0])
            F = fr.copy(); F.apply_translation([0, 0, C.Z_BLK0])
            ba = blade(D0 + qa, 1); bb = blade(D0 + qb, -1)
            add("blade+ vs own tab", ba.intersection(A, engine="manifold").volume)
            add("blade+ vs other tab", ba.intersection(Bm, engine="manifold").volume)
            add("blade- vs own tab", bb.intersection(Bm, engine="manifold").volume)
            add("blade- vs other tab", bb.intersection(A, engine="manifold").volume)
            add("blades vs frame", ba.intersection(F, engine="manifold").volume + bb.intersection(F, engine="manifold").volume)
            add("blades vs tray (free part)", D(ba, B(-30, 30, -5, 20, Z_ROOT - 0.05, 90)).intersection(tr3, engine="manifold").volume
                + D(bb, B(-30, 30, -5, 20, Z_ROOT - 0.05, 90)).intersection(tr3, engine="manifold").volume)
            add("blade+ vs blade-", ba.intersection(bb, engine="manifold").volume)
            add("sliders vs frame/each other", A.intersection(F, engine="manifold").volume + Bm.intersection(F, engine="manifold").volume
                + A.intersection(Bm, engine="manifold").volume)
    for k, v in worst.items(): print("  %-30s %.4f mm3" % (k, v))
    # bearing: the tip must touch the groove bottom (gap 0) at rest and squeezed
    for q in (0, C.TRAVEL):
        A = s1.copy(); A.apply_translation([-q, 0, C.Z_BLK0])
        probe = blade(D0 + q, 1, dil=0.02, tip_exact=False, clamp=False)
        print("  tip bears at q %.1f: overlap with 0.02 grown blade %.4f mm3 (>0 = touching)" % (q, probe.intersection(A, engine="manifold").volume))
    # clamp fit
    print("  clamp vs post slot: %.4f mm3 (0 = clear)" % blade(D0, 1).intersection(post(1), engine="manifold").volume)
    # walls left
    sl_rest = s1.copy(); sl_rest.apply_translation([0, 0, C.Z_BLK0])
    print("  block v4 %.2f g (v3 6.3 g M incl. 2 x 2.01 sliders), frame %.2f, slider %.2f g" % (
        (fr.volume + s1.volume + s2.volume) * 1.27e-3, fr.volume * 1.27e-3, s1.volume * 1.27e-3))
    print("  posts %.2f g, tray v3-drawn +blades %.2f g (was %.2f)" % (tray_add().volume * 1.27e-3, tr3.volume * 1.27e-3, C.tray().volume * 1.27e-3))
    bl = U(fr, s1, s2); bl.export("out/s2_cartridge_block_v4.stl")
    tr3.export("out/s2_tray_v3_blades.stl")
    tray_cut().export("out/s2_blade_cut.stl"); tray_add().export("out/s2_blade_posts.stl")
    for nm, m in (("fr", fr), ("s1", s1), ("s2", s2)): m.export("out/_%s.stl" % nm)
    for q, nm in ((0, "rest"), (C.TRAVEL, "full")):
        U(blade(D0 + q, 1), blade(D0 + q, -1)).export("out/_blades_%s.stl" % nm)
    print("bodies: frame %d, slider %d, tray %d" % (len(fr.split(only_watertight=False)), len(s1.split(only_watertight=False)), len(tr3.split(only_watertight=False))))
