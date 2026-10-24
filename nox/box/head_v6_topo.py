"""Head v6 generated on the topo engine (24 Oct 2026, docs/generative.md The rule; nox/topo). Head A is generated; head B = head A
mirrored in X (post_head_v6.py). Geometry: head_v6.head_v6_parts (carriage from arm_v5_cad, gripper v4d from arm_cad).
The head is cut from the arm system at its joints: the three V-wheels (they hold X and Y, they roll along u), the rack teeth at the
pinion (they hold u: the pinion drives), and the pack in the gripper (fingers -> racks -> guide block). Loads are the swap system's
cases at those joints (LOADS: arm_system.py's head joint cases when it is on main, else the estimates below, E).
python3 head_v6_topo.py [h] [gen_cm3] [out_dir]   (from nox/box; PYTHONPATH nox:nox/box)"""
import sys, os, math
import numpy as np
import arm_layout as L, arm_cad as A
import head_v6 as HD
import arm_v5_layout as V
from arm_cad import box
from topo import Part, Settings, optimise

F_INSERT, F_PULL, F_SPRING = 74.0, 50.0, 15.0     # E: insert at the head motor's stall (0.59 N.m / r 8; design push 60), XT60 pull-out,
                                                  # gripper spring per finger; the system file replaces them when it lands
Z_DRONE, Z_BAY = max(HD.Z0S), L.Z0_BAY + 1.5
T20 = math.tan(math.radians(20))

def system_cases():
    """(name, z0 of the pinion, F along u at the pack, separating force on the teeth) per swap case. arm_system.py (arm-mech worker)
    supplies them when it lands: head_joint_cases('A') -> same tuples."""
    try:
        import arm_system as S
        return S.head_joint_cases("A")
    except Exception:
        return [("insert stall at the drone insert", Z_DRONE, -F_INSERT, F_INSERT * T20),
                ("insert stall at the bay", Z_BAY, -F_INSERT, F_INSERT * T20),
                ("pull-out at the drone", L.Z0_DRONE, F_PULL, F_PULL * T20),
                ("pull-out at the bay", Z_BAY, F_PULL, F_PULL * T20)]

def setup(h):
    P = HD.head_v6_parts("A")
    # the design envelope as given (the generated region + the web stiffener room); the carriage lies outside it and stays whole
    # (organic finish), the gripper's guide block and servo plate inside it keep their functional skins
    p = Part("head_A_v6", h, P["drawn"], P["cuts"], P["env0"], build="-y")
    y0, y1 = HD.PLATE_Y
    wheels = [A.cylY(x, u, y0, y1, 9.6 if abs(x - HD.ECC["x"]) < 1e-6 else 6.5) - A.cylY(x, u, y0 - 1, y1 + 1, 2.6) for x, u in HD.WHEELS]
    def teeth_at(z0):
        up = V.PIN_Z - z0
        return box(HD.WEB_X[0] + 1.0, HD.WEB_X[1] + 0.2, HD.WEB_Y[0], HD.RACK_TEETH_Y[1], up - 4.0, up + 4.0)
    def support_dofs(name, parts):
        fx = np.zeros((p.nn, 3), bool); cells = []
        for region, dofs in parts:
            el = p.elems(region, p.solid | p.design, what=f"support {name}"); nd = p.nodes_of(el)
            for d in dofs: fx[nd, d] = True
            cells.append(el)
        p.supports[name] = fx; p.support_cells[name] = np.unique(np.concatenate(cells))
        p.log(f"  support {name}: {len(p.support_cells[name])} cells, {int(fx.sum())} dofs")
    cases = system_cases()
    for nm, z0, Fu, Fs in cases:
        sname = f"pinion at z0 {z0:.1f}"
        if sname not in p.supports:
            support_dofs(sname, [(w, (0, 1)) for w in wheels] + [(teeth_at(z0), (2,))])
    guide = box(L.GUIDE_X[0] - 0.5, L.GUIDE_X[1] + 0.5, L.Y_C - 16, L.Y_C + 16, L.GUIDE_U[0] - 0.5, L.GUIDE_U[1] + 0.5)
    splate = box(L.P_SH - 13.5, L.P_SH + 13.5, L.Y_C - 21, L.Y_C + 40.5, L.U_PLATE[0] - 0.5, L.U_PLATE[1] + 0.5)
    for nm, z0, Fu, Fs in cases:                       # pack along u through the gripper; the rack's push-off (tan 20) on the teeth
        p.load_multi(nm, [(guide, (0, 0, Fu)), (teeth_at(z0), (-Fs, 0, 0))], support=f"pinion at z0 {z0:.1f}")
    sb = f"pinion at z0 {Z_BAY:.1f}"
    for vec, nm in (((10, 0, 0), "+x"), ((-10, 0, 0), "-x"), ((0, 10, 0), "+y"), ((0, -10, 0), "-y")):
        p.load(f"pack side {nm} 10 N", guide, vec, support=sb)
    for sgn in (1, -1):                                 # the servo opens both springs: 2 x 15 N at the 12T, reacted at the servo plate
        p.load(f"servo opens the springs {'+' if sgn > 0 else '-'}x", splate, (sgn * 2 * F_SPRING, 0, 0), support=sb)
    p.load("servo weight / whip -u", splate, (0, 0, -6), support=sb); p.load("servo whip +y", splate, (0, 6, 0), support=sb)
    p.accel_case("arm whip +x 3g", (3, 0, 0), support=sb); p.accel_case("arm whip +u 3g", (0, 0, 3), support=sb)
    return p

if __name__ == "__main__":
    h = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    gen = float(sys.argv[2]) if len(sys.argv) > 2 else 22.0
    out = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HD.HV.REPO, "runs", "head", "v6")
    if os.environ.get("NOX_SETUP_ONLY"):
        setup(h); sys.exit(0)
    res = optimise(setup, h, gen, os.path.abspath(out), "head_A_v6", settings=Settings(), git=False)
    print({k: res.get(k) for k in ("name", "solid_ok", "mass_g", "total_s")}, res["strength"]["worst"])
