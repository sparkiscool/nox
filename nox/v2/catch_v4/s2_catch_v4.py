"""Cage v4 catch check (17 Oct 22:10): the full-physics landing of s2_catch_real.py with the v4 node wheels.
v4 = wheels R 5, 3 wide, on a 3.0 pin (axle friction at r 1.5), rim 2.5 proud of a 3.0 lower ring, rings 3.0 carbon
(~8.3 g each, C). 'v3' = the file as it stood (R 4 on a 2.0 pin, 1.0 proud, 2.5 / 2.0 rings).
Run: python3 s2_catch_v4.py v3|v4 [mu]   -> one JSON line (centred % over the 204-flight sweep)"""
import sys, json, math
import s2_node_wheels as w, s2_collar_settle as s
mm = 1e-3
cfg = sys.argv[1]; mu = float(sys.argv[2]) if len(sys.argv) > 2 else 0.4
if cfg == "v4":
    w.R_W, w.R_PIN, w.PROUD = 5 * mm, 1.5 * mm, 2.5 * mm
    s.R_ROD = 1.5 * mm
import s2_catch_real as r            # imports set the slope (45 deg) with the R_ROD above
if cfg == "v4":
    w.set_slope(45)
    r.PARTS[0] = ("lower ring 3.0 carbon", 8.3, "ring", s.ZS / mm); r.PARTS[1] = ("upper ring 3.0 carbon", 8.3, "ring", s.ZS / mm + 16)
    r.M, r.CG, r.INERTIA = r.mass_props()
res = r.sweep("wheelD", mu); res.update(cfg=cfg, wheel_R_mm=w.R_W / mm, pin_r_mm=w.R_PIN / mm, proud_mm=w.PROUD / mm,
                                        mass_g=round(r.M * 1000), collar_H_mm=round((s.Z_HI - s.Z_LO) / mm, 1))
print(json.dumps(res), flush=True)
