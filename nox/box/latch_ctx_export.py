"""Export the battery latch's meshes the box layout uses (runs/box_system/v2/latch_ctx/): per pack the cartridge, pack,
receiver, receiver lid and XT60E1-M in cartridge coordinates (x across, y up the install axis from the flat bottom, z from
the closed end), plus meta.json (tab line, sizes, swap mass / CG, mount plane). The latch lives on its own branch until it
merges, so this reads a checkout of it:

  git worktree add --detach <dir> <latch sha>
  python3 nox/box/latch_ctx_export.py <dir> <latch sha>     (or this checkout: . <main sha>, once the latch is on main)
"""
import json, os, sys

import numpy as np
import trimesh

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
OUT = os.path.join(ROOT, "runs", "box_system", "v2", "latch_ctx")


def main(src, sha):
    src = os.path.abspath(src)
    sys.path[:0] = [os.path.join(src, "nox", "box"), os.path.join(src, "nox")]
    import battery_latch as BL
    meta = {}
    for pk in ("scout", "overseer"):
        d = BL.latch_assembly(pk, "latched"); B = BL.build(pk); g = B["g"]; r = B.get("r") or {}
        for k in ("cartridge", "pack", "receiver", "receiver_lid", "xt60e1m"):
            m = d[k].to_mesh()
            trimesh.Trimesh(np.asarray(m.vert_properties)[:, :3], np.asarray(m.tri_verts)).export(os.path.join(OUT, f"{pk}_{k}_c.stl"))
        M, c, _ = BL.swap_cg(g)
        y_m = r.get("Y_M", g.get("Y_M"))
        meta[pk] = dict(Z_T=g["Z_T"], L_C=g["L_C"], W=g["W"], H=g["H"], Y_TOP=g["Y_TOP"], Z_X=g["Z_X"], SWAP_M=float(M),
                        SWAP_CG=[float(v) for v in c], Y_M=None if y_m is None else float(y_m), RF=g["RF"])
    meta["source"] = (f"battery_latch.latch_assembly(pack, 'latched') @ {sha} ({'main' if src == ROOT else os.path.basename(src)}); "
                      "cartridge coords c (x across, y up the install axis from the flat bottom, z from the closed end)")
    json.dump(meta, open(os.path.join(OUT, "meta.json"), "w"), indent=1)
    print(json.dumps(meta, indent=1))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
