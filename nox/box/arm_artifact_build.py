"""Build the interactive swap-arm artifact: every printed part (+ bought context parts) as decimated meshes in W coords,
the swap-cycle timeline, and the clearance sweep (out/arm_sweep.json) embedded in one self-contained HTML page.
python arm_artifact_build.py  -> out/nox_swap_arm_v4.html"""
import os, json, base64, math
import numpy as np, trimesh
import fast_simplification as fsimp
import arm_layout as L, arm_cad as A
import arm_kinematics as K
from arm_cad import tm

OUT = "out"
RHO = 1.27e-3   # PETG g/mm3

def dec(m, target):
    if len(m.faces) > target:
        v, f = fsimp.simplify(np.asarray(m.vertices, np.float32), np.asarray(m.faces, np.uint32), target_count=target)
        return trimesh.Trimesh(v, f, process=False)
    return m

def encode(m):
    """positions quantised to 0.02 mm int16 (+ offset), indices uint16/uint32, base64"""
    v = np.asarray(m.vertices, np.float64); f = np.asarray(m.faces)
    off = v.min(0); q = np.round((v - off) / 0.02).astype(np.int16)
    big = len(v) > 65000
    idx = f.astype(np.uint32 if big else np.uint16).ravel()
    return dict(off=[round(float(x), 3) for x in off], n=len(v), big=big,
                p=base64.b64encode(q.tobytes()).decode(), i=base64.b64encode(idx.tobytes()).decode())

def main():
    P = K.build_parts()
    # unique meshes -> decimate once, instance per part
    targets = dict(hub=16000, headA=10000, headB=10000, stub=5000, **{f"bay{k}": 9000 for k in range(4)})
    meshes, parts = {}, []
    shared = {}
    pass                                                       # v3: the ring segments are each their own mesh
    for h in ("A", "B"):
        pass                                                   # v3: head A's gripper is mirrored, so A and B have their own finger / pinion meshes
    for nm in L.MOTORS:
        shared[f"motor_{nm}"] = "motor"; shared[f"p20_{nm}"] = "p20"
    for nm, part in P.items():            # the first instance of a shared mesh is the one decimated and embedded
        key = shared.get(nm, nm)
        if key not in meshes:
            m = dec(part.mesh, targets.get(key, 20000))
            meshes[key] = encode(m); meshes[key]["faces"] = int(len(m.faces))
            mass = None
            if part.printed:
                try: mass = round(float(abs(part.mesh.volume)) * RHO, 1)
                except Exception: mass = None
            meshes[key]["mass"] = mass
        parts.append(dict(id=nm, mesh=key, name=part.name, group=part.group, color=part.color, printed=part.printed, note=""))
    # stub wall tiles: the one generated tile, turned into the four quadrants
    if os.path.exists(f"{OUT}/case_stub_tile_v1_W.stl"):
        st = K.load_dec(f"{OUT}/case_stub_tile_v1_W.stl", 40000)
        for q in range(4):
            parts.append(dict(id=f"stubtile{q}", mesh="stubtile", name=f"stub wall tile {q}", group="fixed", color="#9aa5b1", printed=True, note="rot %d" % (90 * q)))
            if "stubtile" not in meshes:
                m = dec(st, 5000); meshes["stubtile"] = encode(m); meshes["stubtile"]["faces"] = int(len(m.faces))
                meshes["stubtile"]["mass"] = round(float(abs(st.volume)) * RHO, 1)
    # cartridges at three squeezes
    for sq in (0.0, 2.0, 4.0):
        m = tm(K.cartridge_local(sq)); meshes[f"cart{sq:.0f}"] = encode(m); meshes[f"cart{sq:.0f}"]["faces"] = int(len(m.faces)); meshes[f"cart{sq:.0f}"]["mass"] = None
    sweep = json.load(open(f"{OUT}/arm_sweep.json")) if os.path.exists(f"{OUT}/arm_sweep.json") else []
    data = dict(meshes=meshes, parts=parts, keys=K.KEYS, phases=K.PHASES, sweep=sweep, tEnd=K.T_END, tLand=K.T_LAND,
                layout=dict(R=L.R, R_PIN=L.R_PIN, CART_BOT_DZ=L.CART_BOT_DZ, Y_C=L.Y_C, P_SH=L.P_SH,
                            bays=L.BAY_ANGLES, CASE_IN=L.CASE_IN, Y_FAR=L.Y_FAR_WALL, Y_STUB_WALL=L.Y_STUB_WALL[1],
                            TOP_OPENING=L.TOP_OPENING, BAR_HALF=L.BAR_HALF))
    html = open("arm_artifact_template.html").read().replace("__DATA__", json.dumps(data, separators=(",", ":")))
    open(f"{OUT}/nox_swap_arm_v4.html", "w").write(html)
    tot = sum(m["faces"] for m in meshes.values())
    print(f"{OUT}/nox_swap_arm_v4.html {os.path.getsize(f'{OUT}/nox_swap_arm_v4.html') / 1e6:.2f} MB, {len(parts)} parts, {len(meshes)} meshes, {tot} faces, {len(sweep)} sweep states")

if __name__ == "__main__":
    main()
