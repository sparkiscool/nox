"""Overseer / Scout landing into the funnel on a free-standing box (MuJoCo).

The box is one rigid body on its feet (free joint, floor friction), the drone
is a rigid body with 8 contact spheres (node wheels), motors killed at first
touch (worst case: no thrust after commit). Reports foot lift and box tilt.
"""
import numpy as np
import mujoco
from stand_model import FIXED, ring_cg, ZF, BAY_ANG, R_BAY, Y_BAY, PACK, BAR, HEAD_A, HEAD_B, R_HEAD_PARK

FUN_B, FUN_T, FUN_ZB, FUN_ZT, FUN_Y = 84.0, 160.0, 158.0, 234.0, 126.0   # E/C funnel in W
WHEEL_UP, WHEEL_R = 30.0, 6.0                                            # E seat point, wheel


def box_inertial(occ=(0,) * 7, phi=90.0):
    """Mass (kg), CG (m), full inertia about CG (kg m^2) for one box state."""
    pts = []                                  # (g, pos mm, half size mm)
    for name, g, p, hs, _ in FIXED:
        if p is None:                         # ring: sample the annular sector
            th = np.radians(np.linspace(112.5, 427.5, 64))
            for t in th:
                for r in (110, 130, 150):
                    pts.append((g / (64 * 3), (r * np.cos(t), 131, r * np.sin(t)), (5, 43, 5)))
        else:
            pts.append((g, p, hs))
    for a, o in zip(BAY_ANG, occ):
        if o:
            ar = np.radians(a)
            pts.append((PACK[o], (R_BAY * np.cos(ar), Y_BAY, R_BAY * np.sin(ar)), (19, 39, 24)))
    for ang, gm, r in ((phi, BAR, 78), (phi + 90, BAR, 78), (phi, HEAD_A, R_HEAD_PARK), (phi + 90, HEAD_B, R_HEAD_PARK)):
        ar = np.radians(ang)
        pts.append((gm, (r * np.cos(ar), 84 if gm == BAR else 125, r * np.sin(ar)), (20, 20, 40)))
    m = sum(g for g, _, _ in pts) / 1000
    P = np.array([p for _, p, _ in pts]) / 1000
    w = np.array([g for g, _, _ in pts]) / 1000
    c = (w[:, None] * P).sum(0) / m
    I = np.zeros((3, 3))
    for (g, p, hs), wi in zip(pts, w):
        r = np.array(p) / 1000 - c
        a, b, cc = (2 * np.array(hs) / 1000)
        I += wi * (r @ r * np.eye(3) - np.outer(r, r))
        I += np.diag([wi / 12 * (b * b + cc * cc), wi / 12 * (a * a + cc * cc), wi / 12 * (a * a + b * b)])
    return m, c, I


def _face_geoms(mu):
    L = np.hypot(FUN_T - FUN_B, FUN_ZT - FUN_ZB) / 2 / 1000
    mid_r, mid_z = (FUN_B + FUN_T) / 2 / 1000, (FUN_ZB + FUN_ZT) / 2 / 1000
    out = []
    for k, (sx, sy) in enumerate(((1, 0), (-1, 0), (0, 1), (0, -1))):
        if sx:
            pos = (sx * mid_r, FUN_Y / 1000, mid_z); axis = "0 1 0"; ang = -45 * sx
            size = f"{L} 0.16 0.001"
        else:
            pos = (0, FUN_Y / 1000 + sy * mid_r, mid_z); axis = "1 0 0"; ang = 45 * sy
            size = f"0.16 {L} 0.001"
        out.append(f'<geom name="face{k}" type="box" size="{size}" pos="{pos[0]} {pos[1]} {pos[2]}" '
                   f'axisangle="{axis} {ang}" friction="{mu} 0.005 0.0001" solref="0.004 1" rgba=".5 .45 .9 .4"/>')
    return "\n".join(out)


def _wheels(mu):
    """8 wheel spheres in the drone frame (origin = drone CG when seated)."""
    s = 1 / np.sqrt(2)
    xr = FUN_B + WHEEL_UP - WHEEL_R * s
    zr = FUN_ZB + WHEEL_UP + WHEEL_R * s
    pts = []
    for a in (-100, 100):
        pts += [(xr, a), (-xr, a)]
        pts += [(a, xr), (a, -xr)]
    return [(x / 1000, y / 1000, zr / 1000) for x, y in pts]


def build(feet_xy, occ=(0,) * 7, phi=90, drone_kg=2.0, drone_cgz=220.0,
          mu_face=0.05, mu_floor=0.8, foot_half=15.0):
    m, c, I = box_inertial(occ, phi)
    feet = "\n".join(
        f'<geom name="foot{i}" type="box" size="{foot_half/1000} {foot_half/1000} 0.002" '
        f'pos="{x/1000} {y/1000} {(ZF+2)/1000}" friction="{mu_floor} 0.005 0.0001" solref="0.004 1" rgba=".1 .62 .46 1"/>'
        for i, (x, y) in enumerate(feet_xy))
    wheels = "\n".join(
        f'<geom type="sphere" size="{WHEEL_R/1000}" pos="{x} {y - FUN_Y/1000} {z - drone_cgz/1000}" '
        f'friction="{mu_face} 0.005 0.0001" solref="0.004 1"/>'
        for x, y, z in [(p[0], p[1] + FUN_Y / 1000, p[2]) for p in _wheels(mu_face)])
    xml = f"""
<mujoco>
  <option timestep="0.0002" gravity="0 0 -9.81" cone="elliptic" impratio="10" noslip_iterations="10"/>
  <worldbody>
    <geom type="plane" size="2 2 .1" friction="{mu_floor} 0.005 0.0001"/>
    <body name="box" pos="0 0 {-ZF/1000}">
      <freejoint name="boxj"/>
      <inertial pos="{c[0]} {c[1]} {c[2]}" mass="{m}"
        fullinertia="{I[0,0]} {I[1,1]} {I[2,2]} {I[0,1]} {I[0,2]} {I[1,2]}"/>
      {feet}
      {_face_geoms(mu_face)}
    </body>
    <body name="drone" pos="0 {FUN_Y/1000} {(drone_cgz - ZF)/1000 + 1}">
      <freejoint name="dj"/>
      <inertial pos="0 0 0" mass="{drone_kg}" diaginertia="0.030 0.030 0.053"/>
      {wheels}
    </body>
  </worldbody>
</mujoco>"""
    return mujoco.MjModel.from_xml_string(xml), m, c


def _magnet(model, d, seat, on):
    """Funnel sequence step 3 (v2/box-funnel.md): KW11 made = seated, magnet on.
    Modelled as a spring-damper between drone and box at the seat once the
    drone first passes within 5 mm of the seat (E: 2000 N/m, 60 N s/m)."""
    b, dr = 1, 2
    R = d.xmat[b].reshape(3, 3)
    p_rel = R.T @ (d.xpos[dr] - d.xpos[b]) - seat
    if not on and np.hypot(p_rel[0], p_rel[1]) < 0.005 and abs(p_rel[2]) < 0.004:
        on = True
    d.xfrc_applied[:] = 0
    if on:
        vb = np.zeros(6); vd = np.zeros(6)
        mujoco.mj_objectVelocity(model, d, mujoco.mjtObj.mjOBJ_BODY, b, vb, 0)
        mujoco.mj_objectVelocity(model, d, mujoco.mjtObj.mjOBJ_BODY, dr, vd, 0)
        f = -2000 * (R @ p_rel) - 60 * (vd[3:] - vb[3:])
        d.xfrc_applied[dr, :3] = f
        d.xfrc_applied[b, :3] = -f
        d.xfrc_applied[b, 3:] = np.cross(d.xpos[dr] - d.xipos[b], -f)
    return on
