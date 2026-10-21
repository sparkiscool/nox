"""Stub-wall tile v1 (NE quarter; the other three are rotations). python arm_run_stub_tile.py [h] [gen_cm3]
Print: skin down on the bed (Y 0 face), ribs up: build direction -Y. Overhang filter on (no supports)."""
import sys, os, math
import arm_layout as L, arm_casing as CS
from arm_topo import PartProblem, run_part, box, log
from arm_cad import cylY

H = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
GEN = float(sys.argv[2]) if len(sys.argv) > 2 else 25.0
P = CS.stub_tile_parts(); drawn, cuts, env = P["drawn"], P["cuts"], P["env"]
pp = PartProblem("stub_tile", H, drawn, cuts, env, build="-y", overhang=False)   # filter on: the layout died at vol 0.016 (T); ribs on a flat skin print without supports anyway
y0, y1 = L.Y_STUB_WALL; T = CS.TILE
pp.support("default", box(T - 8.5, T + 1, y0 - 5, y1 + 1, -1, T + 1) + box(-1, T + 1, y0 - 5, y1 + 1, T - 8.5, T + 1))   # glued outer edges
bx, bz = L.part_angle_pos(38.0, 45)
boss = cylY(bx, bz, y0 - 6.5, y1 + 0.5, 6.5)
seat = cylY(0, 0, y0 - 2.5, y0 + 0.5, 46.5) ^ box(0, 50, y0 - 3, y1, 0, 50)
pp.load("bolt pull 80 N", boss, (0, -80, 0)); pp.load("bolt push 80 N", boss, (0, 80, 0))   # arm moment 6 N·m on 4 bolts
pp.load("bolt shear x", boss, (25, 0, 0)); pp.load("bolt shear z", boss, (0, 0, 25))
pp.load("flange seat 50 N", seat, (0, 50, 0))                                              # arm weight through the seat
pp.load("hand on the wall", box(60, 140, y0 - 1, y1 + 1, 60, 140), (0, -40, 0))            # 40 N on the panel centre
pp.load("seam edge X", box(-1, 8.5, y0 - 5, y1 + 1, 0, T), (0, 20, 0))                     # neighbour tile pulls
pp.load("seam edge Z", box(0, T, y0 - 5, y1 + 1, -1, 8.5), (0, 20, 0))
if not os.environ.get('NOX_SETUP_ONLY'):
    res, mesh = run_part(pp, drawn, cuts, env, GEN, "out/case_stub_tile_v1.stl")