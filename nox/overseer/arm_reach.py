"""Overseer printed arms: how short can they go, and the tall-root hub.

python3 nox/overseer/arm_reach.py

Numbers come from nox/dimensions.md (O, M) and are tagged there; this script
only does the arithmetic (C) on top of them and the estimates marked E below.
"""
import math

PROP_D = 254.0            # 1045 prop (C, dimensions O Props)
STOCK_WB = (500.0, 480.0)  # generic S500 / Holybro-class 480 (dimensions O, M Q7)

# Arm beam (E, DEFAULT until the plate is measured)
E_PETG = 2780.0           # MPa, nox/topo/material.py
RHO_PETG = 1.27e-6        # kg/mm3
R_CLAMP = 70.0            # arm leaves the plates at r 70 (E, plate outline Q7)
M_TIP = 0.081 + 0.010 + 0.005  # motor + parts 81 g (spec) + prop ~10 (Q8) + screws (E), kg
WALL = 2.4                # box-section wall (E)
W_ROOT, W_TIP = 26.0, 22.0  # section width at the clamp / under the motor (E)
H_TIP = 16.0              # section height under the motor pad (E)

# Motor speed band (E): hover 2.0 kg on 4 x 1045, 1650 gf at full ~9500 rpm loaded
RPM_FULL = 9500.0
RPM_HOVER = RPM_FULL * math.sqrt(500.0 / 1650.0)


def wheelbase(gap):
    """Diagonal wheelbase for a given gap between adjacent prop tips."""
    return math.sqrt(2.0) * (PROP_D + gap)


def tip_gap(wb):
    return wb / math.sqrt(2.0) - PROP_D


def section_I(b, h, t):
    return (b * h**3 - (b - 2 * t) * (h - 2 * t) ** 3) / 12.0


def first_bending_hz(wb, h_root, n=400):
    """Vertical bending of one arm, tapered box section, motor as tip mass."""
    L = wb / 2.0 - R_CLAMP
    comp, mass = 0.0, 0.0
    for i in range(n):
        x = (i + 0.5) / n * L
        s = x / L
        h = h_root + (H_TIP - h_root) * s
        b = W_ROOT + (W_TIP - W_ROOT) * s
        comp += (L - x) ** 2 / (E_PETG * section_I(b, h, WALL)) * (L / n)
        mass += (b * h - (b - 2 * WALL) * (h - 2 * WALL)) * RHO_PETG * (L / n)
    k = 1.0 / comp * 1000.0  # N/m
    m_eff = M_TIP + 0.24 * mass
    return math.sqrt(k / m_eff) / (2 * math.pi), L, mass


def roll_authority(wb):
    """Roll torque per inertia relative to the 500 wheelbase (E body masses)."""
    def ratio(w):
        r = w / (2 * math.sqrt(2))  # motor offset from the roll axis, mm
        i_motors = 4 * M_TIP * (r / 1000) ** 2
        i_body = 0.0015 + 0.0002 + 0.0023 + 0.0008  # pack low, pack self, L2 high, hub (E)
        return r / (i_motors + i_body)
    return ratio(wb) / ratio(500.0)


def hub_under_disks(wb, half):
    """Area of a square hub (half-width `half`) under the four prop disks, mm2."""
    r = wb / (2 * math.sqrt(2))
    R = PROP_D / 2
    n, area = 200, 0.0
    d = 2 * half / n
    for i in range(n):
        for j in range(n):
            x = -half + (i + 0.5) * d
            y = -half + (j + 0.5) * d
            if any((x - sx * r) ** 2 + (y - sy * r) ** 2 <= R * R
                   for sx in (-1, 1) for sy in (-1, 1)):
                area += d * d
    return area


if __name__ == "__main__":
    print(f"motor band (E): 1P hover {RPM_HOVER/60:.0f} Hz .. full {RPM_FULL/60:.0f} Hz, "
          f"2P {2*RPM_HOVER/60:.0f} .. {2*RPM_FULL/60:.0f} Hz")
    for wb in STOCK_WB:
        print(f"stock {wb:.0f}: tip gap {tip_gap(wb):.1f}, motor r {wb/2:.0f}")
    print("\ngap  wheelbase  motor r  span   shorter/arm vs 500 / 480  roll authority")
    for gap in (0, 10, 15, 25, 40):
        wb = wheelbase(gap)
        span = wb / math.sqrt(2) + PROP_D
        print(f"{gap:3d}  {wb:7.1f}  {wb/2:7.1f}  {span:5.0f}   "
              f"{250-wb/2:5.1f} / {240-wb/2:5.1f}            x{roll_authority(wb):.2f}")
    print("\nfirst vertical bending, PETG box arm (E), clamp r 70:")
    for wb in (wheelbase(25), 480.0):
        for h_root in (19.0, 32.0, 48.0):
            f, L, m = first_bending_hz(wb, h_root)
            print(f"  wb {wb:5.1f}  free {L:5.1f}  root {h_root:4.0f} tall: "
                  f"{f:4.0f} Hz, arm {m*1000:4.0f} g")
    print("\nhub area under the disks at wb 395 (square hub):")
    for half in (50, 60, 70, 80):
        print(f"  {2*half} x {2*half}: {hub_under_disks(wheelbase(25), half):6.0f} mm2 "
              f"({hub_under_disks(wheelbase(25), half) / (4*math.pi*(PROP_D/2)**2)*100:.1f} % of disk area)")
