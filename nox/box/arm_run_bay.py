"""Bay v1 run (one Scout charging bay, drawn at the top position; radial = Z). python arm_run_bay.py [h] [gen_cm3]
Print: bed at the back (Z 176 face down), growing toward the axle: build direction -Z, overhang filter on."""
import sys, os
import arm_layout as L, arm_cad as A
from arm_topo import PartProblem, run_part, box, log

H = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
GEN = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
P = A.bay_parts(); drawn, cuts, env = P["drawn"], P["cuts"], P["env"]
pp = PartProblem("bay", H, drawn, cuts, env, build="-z", overhang=False)      # overhang filter wiped the part (T, as on the pocket): tree supports instead
yc = L.Y_C; r_tip, r_top = L.R, L.R_BAY_TOP
x0c, x1c = -L.Z_TABC, L.CART_L - L.Z_TABC
feet = None
for (x, r) in ((x0c - 2, r_top - 20), (x1c + 2, r_top - 20), (0.0, r_top - 2)):
    f = box(x - 6.5, x + 6.5, L.Y_FAR_WALL - 4.5, L.Y_FAR_WALL + 0.5, r - 6.5, r + 6.5); feet = f if feet is None else feet + f
pp.support("default", feet)
pad = box(x0c - 0.5, x1c - 5.5, yc - 16.5, yc + 16.5, r_top, r_top + 4)
rails = box(x0c - 0.5, x1c + 0.5, yc - L.CART_X_OUT - 3.4, yc - L.CART_X_OUT + 0.1, r_tip + 19.5, r_tip + 46.5) + \
        box(x0c - 0.5, x1c + 0.5, yc + L.CART_X_OUT - 0.1, yc + L.CART_X_OUT + 3.4, r_tip + 19.5, r_tip + 46.5)
ends = box(x0c - 3.5, x0c, yc - 16.5, yc + 16.5, r_tip + 19.5, r_top + 0.5) + box(x1c, x1c + 3.5, yc - 16.5, yc + 16.5, r_tip + 19.5, r_top + 0.5)
lips = box(-8.5, 8.5, yc - L.CART_X_OUT - 3.4, yc - L.CART_X_OUT + 2, r_tip + 19.5, r_tip + 26.5) + \
       box(-8.5, 8.5, yc + L.CART_X_OUT - 2, yc + L.CART_X_OUT + 3.4, r_tip + 19.5, r_tip + 26.5)
xpc = L.Z_PINC - L.Z_TABC
blocks = box(xpc - 12.5, xpc + 12.5, yc - L.PINBLK_X1 - 0.5, yc - L.RAIL_FACE + 0.5, L.R_PIN - 6.2, L.R_PIN + 6.2) + \
         box(xpc - 12.5, xpc + 12.5, yc + L.RAIL_FACE - 0.5, yc + L.PINBLK_X1 + 0.5, L.R_PIN - 6.2, L.R_PIN + 6.2)
pp.load("insert push 20 N", pad, (0, 0, 20))                      # head pushes the pack home onto the pad
pp.load("pack 6g into the back", pad, (0, 0, 11))
pp.load("extract through lips 20 N", lips, (0, 0, -20))           # head pulls the pack out: lips give way at 5 N, 20 N misuse
pp.load("pack 6g out (lips)", lips, (0, 0, -11))
pp.load("pack 6g +x", box(x1c, x1c + 3.5, yc - 16.5, yc + 16.5, r_tip + 19.5, r_top + 0.5), (11, 0, 0))
pp.load("pack 6g -x", box(x0c - 3.5, x0c, yc - 16.5, yc + 16.5, r_tip + 19.5, r_top + 0.5), (-11, 0, 0))
pp.load("pack 6g +y", box(x0c - 0.5, x1c + 0.5, yc + L.CART_X_OUT - 0.1, yc + L.CART_X_OUT + 3.4, r_tip + 19.5, r_tip + 46.5), (0, 11, 0))
pp.load("pack 6g -y", box(x0c - 0.5, x1c + 0.5, yc - L.CART_X_OUT - 3.4, yc - L.CART_X_OUT + 0.1, r_tip + 19.5, r_tip + 46.5), (0, -11, 0))
pp.load("pins spring +y", box(xpc - 12.5, xpc + 12.5, yc - L.PINBLK_X1 - 0.5, yc - L.RAIL_FACE + 0.5, L.R_PIN - 6.2, L.R_PIN + 6.2), (0, 10, 0))
pp.load("pins spring -y", box(xpc - 12.5, xpc + 12.5, yc + L.RAIL_FACE - 0.5, yc + L.PINBLK_X1 + 0.5, L.R_PIN - 6.2, L.R_PIN + 6.2), (0, -10, 0))
pp.load("hand on the pad 50 N", pad, (0, 0, -50))
pp.load("hand on the pins 30 N", blocks, (30, 0, 0))
pp.accel_case("box dropped 6g -z", (0, 0, -6)); pp.accel_case("box dropped 6g +x", (6, 0, 0))
if not os.environ.get('NOX_SETUP_ONLY'):
    res, mesh = run_part(pp, drawn, cuts, env, GEN, "out/box_bay_v1.stl")