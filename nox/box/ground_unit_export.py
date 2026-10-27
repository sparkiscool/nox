"""Nox Ground Unit 3D (27 Oct 2026, box master session): every real box mesh in W, decimated and quantised into one blob plus a manifest,
with the swap arm's cycle as frames. Nothing here is designed: it only assembles what the system files already hold.

Sources (read, never written):
  nox/box/box_swap_arm.assembly.json        the arm v7 (meshes runs/arm/v7/asm, local frames) posed by swap_system.pose_at over arm_kinematics.T_END
  <cut>/box_structure_v2_cut.assembly.json  the box structure v2 cut (PR 113 branch run/box_system-v2-cut, pieces in W)
  nox/box/box_structure_v2.assembly.json    the v2 layout: receivers and cartridges per bay (latch on main), sheets, PSU, arm envelopes
  runs/box_electronics/v1/result.json       the rear-sheet board placement (rects, C on E sizes)
Output: runs/ground_unit/v1/scene.json + scene.b64.txt (the blob, base64); the page nox/box/ground_unit_3d.html loads both (published beside it).

    python3 nox/box/ground_unit_export.py [--cut <dir with box_structure_v2_cut.assembly.json and pieces/>] [--out runs/ground_unit/v1]

Blob: per mesh uint16 xyz (origin + scale per mesh, <= 0.01 mm on a 400 mm part) and uint16 / uint32 triangle indices; frames float32 3x4 row-major.
"""
import argparse, json, os, struct, sys
import numpy as np, trimesh

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BOX = os.path.join(ROOT, "nox", "box")
sys.path.insert(0, os.path.join(ROOT, "nox")); sys.path.insert(0, BOX)
RHO = {"PETG": 1.27e-3, "TPU": 1.21e-3}
FPS = 4.0
BOARD_DEPTH = 16.0        # E: board + standoff above the rear sheet's inner face (Y -66 + 10 standoff)


def decimate(m, target):
    m = m.copy(); m.merge_vertices()
    if target and len(m.faces) > target:
        import fast_simplification as fs
        v, f = fs.simplify(np.asarray(m.vertices, dtype=np.float32), np.asarray(m.faces, dtype=np.int64), target_count=int(target))
        m = trimesh.Trimesh(v, f, process=False); m.merge_vertices()
    return m


class Blob:
    def __init__(self): self.chunks = []; self.n = 0
    def add(self, b):
        off = self.n; self.chunks.append(b); self.n += len(b)
        pad = (-self.n) % 4
        if pad: self.chunks.append(b"\0" * pad); self.n += pad
        return off
    def bytes(self): return b"".join(self.chunks)


def encode_mesh(blob, m):
    v = np.asarray(m.vertices, dtype=np.float64); f = np.asarray(m.faces, dtype=np.int64)
    origin = v.min(axis=0); ext = (v.max(axis=0) - origin).max()
    scale = max(ext, 1e-6) / 65000.0
    q = np.clip(np.round((v - origin) / scale), 0, 65535).astype(np.uint16)
    idx_bytes = 2 if len(v) < 65536 else 4
    idx = f.astype(np.uint16 if idx_bytes == 2 else np.uint32)
    off_v = blob.add(q.tobytes()); off_i = blob.add(idx.tobytes())
    return dict(off_v=off_v, off_i=off_i, nV=int(len(v)), nF=int(len(f)), idx_bytes=idx_bytes, origin=[round(float(x), 3) for x in origin], scale=float(scale))


def pose12(T):
    T = np.asarray(T, dtype=np.float64); return [round(float(x), 4) for x in T[:3, :].reshape(-1)]


def mesh_mass(m, material, given=None):
    if given: return given
    if material in RHO and m.is_volume:
        return round(abs(m.volume) * RHO[material], 1)
    return None


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--cut", default=os.path.join(ROOT, "runs", "box_system", "v2", "cut")); ap.add_argument("--out", default=os.path.join(ROOT, "runs", "ground_unit", "v1"))
    ap.add_argument("--faces", type=int, default=9000, help="face budget per mesh (big pieces)"); a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    blob = Blob(); parts = []; stats = dict(faces=0)

    def add(name, system, m, pose=None, frames=None, faces=None, **info):
        m = decimate(m, faces or a.faces)
        enc = encode_mesh(blob, m); stats["faces"] += enc["nF"]
        p = dict(name=name, system=system, mesh=enc, pose=pose12(pose if pose is not None else np.eye(4)), **info)
        if frames is not None: p["frames"] = frames
        parts.append(p); return p

    # ---------------- swap arm v7: meshes in local frames, the cycle as frames
    import assembly as AS
    os.chdir(BOX)
    arm = AS.load(os.path.join(BOX, "box_swap_arm.assembly.json"))
    import swap_system as S, arm_kinematics as AK
    S.kin_setup()
    T_END = float(AK.T_END); times = np.arange(0.0, T_END + 1e-9, 1.0 / FPS)
    frames_f32 = []
    for p in arm.parts:
        nm = p["name"]
        if nm.startswith("rec"): continue                         # the bays' receivers come from the layout file (latch on main)
        mesh_path = p["mesh"] if os.path.isabs(p["mesh"]) else os.path.normpath(os.path.join(BOX, p["mesh"]))
        m = trimesh.load(mesh_path, force="mesh", process=False)
        poses = [np.asarray(S.pose_at(float(t), p)) for t in times]
        moving = any(not np.allclose(P_, poses[0], atol=1e-6) for P_ in poses[1:])
        fr = None
        if moving:
            fr = dict(off=blob.add(np.asarray([np.asarray(P_, dtype=np.float32)[:3, :].reshape(-1) for P_ in poses], dtype=np.float32).tobytes()), n=len(times))
            frames_f32.append(nm)
        role = p.get("role", "structure"); mat = p.get("material", "PETG" if role == "structure" else "bought")
        add(nm, "arm", m, pose=poses[0], frames=fr, faces=6000, role=role, material=mat, mass_g=mesh_mass(m, mat, p.get("mass_g")), tag=p.get("tag", "M"),
            owner="box swap: arm", version=p.get("version", "v7"), file=os.path.relpath(mesh_path, ROOT), note=p.get("note", ""), moving=moving)
    os.chdir(ROOT)

    # ---------------- box structure v2 cut (PR 113 branch)
    cut_asm = os.path.join(a.cut, "box_structure_v2_cut.assembly.json")
    if os.path.exists(cut_asm):
        cut = json.load(open(cut_asm))
        for p in cut["parts"]:
            mp = p.get("mesh", "")
            if "cut/pieces/" not in mp: continue
            path = os.path.join(a.cut, "pieces", os.path.basename(mp))
            if not os.path.exists(path): print("missing", path); continue
            m = trimesh.load(path, force="mesh", process=False)
            mat = p.get("material", "PETG")
            add(p["name"], "structure", m, pose=np.asarray(p.get("pose", np.eye(4))) if isinstance(p.get("pose"), list) else None,
                faces=a.faces if mat == "PETG" else 2000, role="structure", material=mat, mass_g=mesh_mass(m, mat, p.get("mass_g")), tag=p.get("tag", "M"),
                owner="box structure", version="v2 cut (PR 113, unmerged, review FAIL)", file=mp, note=p.get("note", ""))
    else:
        print("no cut assembly at", cut_asm, ": structure left out")

    # ---------------- v2 layout: receivers, cartridges, sheets, PSU, envelopes
    lay = AS.load(os.path.join(BOX, "box_structure_v2.assembly.json"))
    for p in lay.parts:
        nm = p["name"]; pose = np.asarray(lay.pose(nm))
        if "box" in p:
            x0, x1, y0, y1, z0, z1 = p["box"]; m = trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0]); m.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
            system = "enclosure" if nm.startswith("sheet") else "electronics"
            add(nm, system, m, pose=pose, faces=100, role=p.get("role"), material=p.get("material", ""), mass_g=p.get("mass_g"), tag=p.get("tag", "C"), owner=p.get("owner", ""), version="v2 layout", file="box (drawn)", note="")
            continue
        mp = p.get("mesh", "")
        if "receiver" in nm or "cartridge" in nm:
            ang = nm.split()[1]
            if "cartridge" in nm and ang in ("-210", "30"): continue      # the arm's moving packs stand for the pick and return bays
            m = trimesh.load(os.path.join(ROOT, mp), force="mesh", process=False)
            pack = "Overseer" if ang in ("0", "-60", "-120", "-180") else "Scout"
            ver = ("receiver v2" if pack == "Overseer" else "receiver v3") if "receiver" in nm else ("cartridge v6" if pack == "Overseer" else "cartridge v7")
            add(nm, "latch", m, pose=pose, faces=4000, role=p.get("role"), material="PETG", mass_g=p.get("mass_g") or mesh_mass(m, "PETG"), tag=p.get("tag", "M"),
                owner="box swap: latch", version=f"{pack} {ver} (latch on main)", file=mp, note=f"bay {ang}, {pack}")
        elif "envelope" in nm or nm.startswith("arm:") or nm.startswith("drone"):
            m = trimesh.load(os.path.join(ROOT, mp), force="mesh", process=False)
            add(nm, "envelopes", m, pose=pose, faces=3000, role="envelope", material="", mass_g=None, tag="C", owner="box structure (layout v2)", version="v2 layout", file=mp, note="keep-out / design space, not a part")
        # catch (funnel) and the v1 mast: left out (no funnel; the v2 cut has its own mast)

    # ---------------- electronics boards on the rear sheet (result.json rects: x0, x1, z0, z1 in W X / Z; Y from the sheet's inner face)
    el = json.load(open(os.path.join(ROOT, "runs", "box_electronics", "v1", "result.json")))
    for key, b in el.get("placement", {}).items():
        x0, x1, z0, z1 = b["rect"]; y0 = -66.0 + 10.0; y1 = y0 + BOARD_DEPTH
        m = trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0]); m.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
        add("board " + key, "electronics", m, faces=100, role="bought", material="", mass_g=None, tag="E", owner="box electronics", version="v1 placement", file="runs/box_electronics/v1/result.json", note="rect C on E sizes; depth 16 E")
    ch = dict(X=[-162, -44], Y=[-66, -7], Z=[-200, -29])
    m = trimesh.creation.box(extents=[ch["X"][1] - ch["X"][0], ch["Y"][1] - ch["Y"][0], ch["Z"][1] - ch["Z"][0]]); m.apply_translation([np.mean(ch["X"]), np.mean(ch["Y"]), np.mean(ch["Z"])])
    add("charger (dual channel, class)", "electronics", m, faces=100, role="bought", material="", mass_g=None, tag="C", owner="box electronics", version="v1", file="runs/box_system/v2/topo_r2/summary.md", note="listing 171 x 118 x 59; DEFAULT iCharger 456DUO class")

    # ---------------- the reserved top slot (no funnel): the landing mechanism's volume, design center
    m = trimesh.creation.box(extents=[412, 403, 114]); m.apply_translation([0, (337 - 66) / 2, (120 + 234) / 2])
    add("RESERVED: top mechanism slot (design center)", "reserved", m, faces=100, role="reserved", material="", mass_g=None, tag="C", owner="design center", version="concept F (PR 125)",
        file="runs/box_catch/bars_concept/summary.md", note="X ±206, Y −66..337, Z 120..234 inside the enclosure; the catch concept F (flat deck, push bars) is not drawn yet")
    # concept F deck footprint from the parks (summary.md Deck size, C): drone x −355..301 about the catch centre = W Y about Y_C; y ±264 = W X. Height E (cartridges 7.0 above the deck).
    YC = 167.1; m = trimesh.creation.box(extents=[528, 656, 6]); m.apply_translation([0, YC + (301 - 355) / 2, 111])
    add("concept F deck footprint 656 x 528 (design center, PR 125)", "reserved", m, faces=100, role="reserved", material="", mass_g=None, tag="C", owner="design center", version="concept F (PR 125, merged)",
        file="runs/box_catch/bars_concept/summary.md", note="flat deck set by the pad and bar parks: W Y −188..468, X ±264 (C), wider than the enclosure (X ±206, Y −66..337): the parks come in or the top grows; deck height Z 111 is E")

    import base64
    with open(os.path.join(a.out, "scene.b64.txt"), "w") as f: f.write(base64.b64encode(blob.bytes()).decode())      # text/plain: the artifact host serves no .bin
    meta = dict(name="Nox Ground Unit", frame="W: X across, Y along the axle (rear −, front +), Z up; mm", fps=FPS, t_end=T_END, n_frames=int(len(times)),
                moving=frames_f32, parts=parts, W=dict(X_IN=206, Y_BACK=-66, Y_FRONT=337, FLOOR=-236, Z_TOP=234, R=113, Y_C=167.1))
    json.dump(meta, open(os.path.join(a.out, "scene.json"), "w"))
    print(f"{len(parts)} parts, {stats['faces']} faces, blob {blob.n / 1e6:.1f} MB, {len(times)} frames, moving {len(frames_f32)}")


if __name__ == "__main__":
    main()
