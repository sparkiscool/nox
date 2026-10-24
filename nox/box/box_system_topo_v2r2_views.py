"""Round 2 views: iso05.stl and the 5 views of a run (TOPO_OUT), with receivers, catch and the electronics keep-outs ghosted.
Sheets are not in the model and not drawn."""
import os, sys, json
import numpy as np, trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import box_system_topo_v2_views as V
import box_system_topo_v2r2 as R2

OUT = V.OUT


def main():
    d, h, o, sh, rho = V.field()
    solid = d["solid"].reshape(sh)
    rho = np.where(solid, 1.0, rho)
    m = V.iso(rho, h, o); m.export(os.path.join(OUT, "iso05.stl"))
    EN = R2.EN
    rec = V.ghost("receivers_W.stl"); catch = V.ghost("catch_W.stl")
    el = []
    for b in R2.elec_keepouts(EN["Y_BACK"]):
        el.append(R2.tm(b))
    X, Y0, Y1, Z0, Z1 = EN["X_IN"], EN["Y_BACK"], EN["Y_FRONT"], EN["FLOOR"], EN["Z_TOP"]
    lim = [(-X, X), (Y0, Y1), (Z0, Z1)]
    G = lambda: [(m, "#2a6fdb", 1.0), (rec, "#999999", 0.25), (catch, "#e08a1e", 0.22)] + [(e, "#aa4444", 0.25) for e in el]
    names = dict(front=(0, -90), side=(0, 0), top=(90, -90), iso=(25, -55), back_iso=(25, 125))
    for n, (elv, az) in names.items():
        V.view(G(), elv, az, os.path.join(OUT, f"view_{n}.png"), f"eta 0.5, {n}", lim)
    print("views done", len(m.faces))


if __name__ == "__main__":
    main()
