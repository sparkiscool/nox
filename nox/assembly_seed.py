"""Seed the system assembly files from what is on main (24 Oct 2026). Run once per system; after that the system's owner
session edits its own `<system>.assembly.json` (nox/ASSEMBLY.md) and this script is history, not a second writer.

python3 nox/assembly_seed.py overseer     -> nox/overseer/overseer.assembly.json + runs/overseer_assembly/v0/*.stl  (~8 min)
python3 nox/assembly_seed.py box_catch    -> nox/box/box_catch.assembly.json + runs/box_catch_assembly/v0/*.stl
python3 nox/assembly_seed.py scout        -> nox/scout/v3/scout.assembly.json (primitives only)

Numbers come from the layout scripts (imported, or their definitions executed) and nox/dimensions.md; a part whose pose
is not derivable is left out and listed in the file's "note". Also holds the "moves" functions the seeded files name.
"""
import io, json, math, os, re, sys, contextlib
import numpy as np, trimesh

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path[:0] = [HERE]
from assembly import new


def _rel(p):
    return os.path.relpath(p, ROOT)


def _box(m):
    lo, hi = m.bounds
    return [round(float(v), 3) for v in (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2])]


def _union(meshes):
    import manifold3d as mf
    out = None
    for m in meshes:
        for piece in (m.split(only_watertight=False) if len(m.faces) else [m]):
            x = mf.Manifold(mf.Mesh(vert_properties=np.asarray(piece.vertices, np.float32), tri_verts=np.asarray(piece.faces, np.uint32)))
            out = x if out is None else out + x
    me = out.to_mesh()
    return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=True)


def _export(m, out_dir, name):
    os.makedirs(out_dir, exist_ok=True)
    p = os.path.join(out_dir, name); m.export(p); return _rel(p)


# ====================================================================== Overseer (layout v2 = cage v0)
def _layout_v2():
    """layout_v2.py's definitions (everything above its search loop) in a namespace: the script runs its 30 min search
    at import, so its source is executed up to `results = []` (the script itself is not edited)."""
    path = os.path.join(HERE, "overseer", "layout_v2.py")
    src = open(path).read(); cut = src.index("\nresults = []")
    ns = {"__name__": "layout_v2_defs", "__file__": path}
    argv = sys.argv; sys.argv = [path]
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            exec(compile(src[:cut], path, "exec"), ns)
    finally:
        sys.argv = argv
    return ns


def overseer(out_json=None, out_dir=None):
    L = _layout_v2()
    out_json = out_json or os.path.join(HERE, "overseer", "overseer.assembly.json")
    out_dir = out_dir or os.path.join(ROOT, "runs", "overseer_assembly", "v0")
    # the chosen case of the committed run (runs/overseer_layout/v2/checks.txt): catch offset and its wheel height
    chk = open(os.path.join(ROOT, "runs", "overseer_layout", "v2", "checks.txt")).read()
    dc = float(re.search(r"chosen: catch ([\d.]+) aft", chk).group(1))
    wz = float(re.search(rf"catch\s+{dc:.1f} aft: wheels z\s+(-?[\d.]+)", chk).group(1))
    run_dm = float(re.search(r"motor square ([\d.]+) ahead;", chk).group(1))
    run_xB = float(re.search(r"L2: base centre \(([\d.]+),", chk).group(1))
    L["set_wheel"](wz)
    dm, xB, zB, g, tot, cg = L["balance"](dc, 60.0)          # the script starts every case at dm 60
    # front camera height: the script's own search (lowest h on the L2 back plane with the L2 out of the view + 2 deg)
    cam_h = None
    for h in np.arange(30, 160, 2.0):
        gg = L["build"](dm, dc, xB, zB, 0.0, h)
        d = L["pts"](gg["l2"], 4000) - gg["cam_lens"]; z = d @ gg["cam_fwd"]
        elev = np.degrees(np.arctan2(d @ gg["cam_up_v"], np.maximum(z, 1e-6))); azim = np.degrees(np.arctan2(d[:, 1], np.maximum(z, 1e-6)))
        if not ((z > 0) & (elev > -L["CAM_VFOV"] / 2 - 2.0) & (np.abs(azim) < L["CAM_HFOV"] / 2 + 2.0)).any(): cam_h = h; break
    g = L["build"](dm, dc, xB, zB, 0.0, cam_h)
    tot, cg = L["cg_of"](g)
    print(f"overseer: dc {dc}, wheels z {wz}, dm {dm:.2f} (run {run_dm}), xB {xB:.2f} (run {run_xB}), zB {zB}, cam h {cam_h}, "
          f"layout mass {tot:.1f} g, CG {np.round(cg, 2).tolist()}")

    RHO = 1.27e-3; GPM = L["ROD_GPM"]
    base = dict(tag="E", owner="overseer", version="layout v2 / cage v0", source="nox/overseer/layout_v2.py")
    parts = []
    P = lambda **k: parts.append({**k, **{kk: vv for kk, vv in base.items() if kk not in k}})
    rodlen = lambda a, b: float(np.linalg.norm(np.subtract(b, a)))
    # stock plates (Holybro S500 STEP via s500_cad)
    P(name="bottom plate", role="bought", mesh=_export(L["BOTTOM"], out_dir, "bottom_plate.stl"), mass_g=61, tag="E", note="S500 stock, STEP-derived mesh")
    P(name="top plate", role="bought", mesh=_export(L["TOP"], out_dir, "top_plate.stl"), mass_g=23, tag="E", note="S500 stock, STEP-derived mesh")
    # printed arms + the ESC each carries (layout mass model: printed fill 0.45, ESC 25 g at the arm's bbox centre)
    for i, a in enumerate(g["arms"]):
        gp = abs(a.volume) * RHO * 0.45
        c = (gp * a.centroid + 25 * a.bounding_box.centroid) / (gp + 25)
        P(name=f"arm {i + 1}", role="structure", mesh=_export(a, out_dir, f"arm_{i + 1}.stl"), material="PETG", mass_g=round(gp + 25, 2),
          cg=np.round(c, 3).tolist(), note="printed fill 0.45 + ESC 25 g (model open)")
    for i, (m, (mx, my)) in enumerate(zip(g["motors"], g["motors_xy"])):
        r = round(float(m.extents[0] / 2), 3)
        P(name=f"motor {i + 1}", role="bought", protect=True, cylinder=dict(r=r, p0=[round(mx, 3), round(my, 3), round(float(m.bounds[0][2]), 3)],
          p1=[round(mx, 3), round(my, 3), round(float(m.bounds[1][2]), 3)]), mass_g=81, note="X2216 body (E height)")
    for i, (m, (mx, my)) in enumerate(zip(g["props"], g["motors_xy"])):
        P(name=f"prop {i + 1}", role="bought", protect=True, cylinder=dict(r=L["PROP_R"], p0=[round(mx, 3), round(my, 3), round(float(m.bounds[0][2]), 3)],
          p1=[round(mx, 3), round(my, 3), round(float(m.bounds[1][2]), 3)]), mass_g=10, note="1045 swept disk at the hub mid plane (layout: 1.0 thick)")
    # cage: the script's two groups (cage_up: guard rings, spokes, V, keels + ring nodes; cage_low: wheel ring, struts + corner nodes)
    low_keys = [k for k in g["rods"] if k.startswith(("strut", "ring x", "ring y"))]
    for nm, keys, nodes, fill, fn in (("cage upper", [k for k in g["rods"] if k not in low_keys], g["ring_nodes"], 0.5, "cage_upper.stl"),
                                      ("cage lower", low_keys, g["nodes"], 0.6, "cage_lower.stl")):
        ms = [g["rod_meshes"][k] for k in keys] + list(nodes)
        gr = [(rodlen(*g["rods"][k]) * GPM, np.add(*g["rods"][k]) / 2) for k in keys] + [(abs(n.volume) * RHO * fill, n.centroid) for n in nodes]
        M = sum(x for x, _ in gr); c = sum(x * y for x, y in gr) / M
        P(name=nm, role="cage", mesh=_export(_union(ms), out_dir, fn), material="carbon + PETG", mass_g=round(M, 2), cg=np.round(c, 3).tolist(),
          note=f"{len(keys)} rods 3.0 carbon (11.0 g/m) + {len(nodes)} PETG nodes (fill {fill})")
    for i, w in enumerate(g["wheels"]):
        ax = "xyz"[int(np.argmin(w.extents))]
        P(name=f"wheel {i + 1}", role="cage", cylinder=dict(r=L["WHEEL_R"], h=L["WHEEL_W"], center=np.round(w.bounds.mean(0), 3).tolist(), axis=ax),
          material="PETG", mass_g=0.25)
    P(name="apex", role="cage", mesh=_export(g["apex"], out_dir, "apex.stl"), material="PETG", mass_g=round(abs(g["apex"].volume) * RHO * 0.45, 2),
      note="L2 seat, block, side lugs, front hub (fill 0.45)")
    gpl = abs(g["cam_plate"].volume) * RHO * 0.5; crs = [(rodlen(a, b) * GPM, np.add(a, b) / 2) for a, b in g["cam_rods"]]
    Mb = gpl + sum(x for x, _ in crs); cb = (gpl * g["cam_plate"].centroid + sum(x * y for x, y in crs)) / Mb
    P(name="cam bracket", role="structure", mesh=_export(_union([g["cam_plate"]] + [L["rod"](a, b) for a, b in g["cam_rods"]]), out_dir, "cam_bracket.stl"),
      material="carbon + PETG", mass_g=round(Mb, 2), cg=np.round(cb, 3).tolist(), note="3 thick plate (fill 0.5) on 3 rods to the apex")
    P(name="Pi deck", role="structure", mesh=_export(L["DECK"], out_dir, "pi_deck.stl"), material="PETG", mass_g=round(abs(L["DECK"].volume) * RHO * 0.5, 2))
    sled = _union([g["sled"], g["cam_pod"], g["flow_pod"]])
    P(name="belly sled + pods", role="structure", mesh=_export(sled, out_dir, "sled_pods.stl"), material="PETG",
      mass_g=round(sum(abs(m.volume) for m in (g["sled"], g["cam_pod"], g["flow_pod"])) * RHO * 0.3, 2), note="fill 0.3")
    for i, f in enumerate(g["feet"]):
        P(name=f"foot {i + 1}", role="structure", cylinder=dict(r=round(float(f.extents[0] / 2), 3), h=round(float(f.extents[2]), 3),
          center=np.round(f.bounds.mean(0), 3).tolist(), axis="z"), material="TPU", mass_g=round(abs(f.volume) * RHO * 0.5, 2))
    P(name="cartridge", role="protected", swap=True, box=_box(g["cart"]), material="PETG", mass_g=round(abs(g["cart"].volume) * RHO * 0.04, 2),
      note="envelope (E, G); receiver and latch not drawn")
    P(name="pack", role="bought", protect=True, swap=True, box=_box(g["pack"]), mass_g=650, note="4S 6000 (P size, E mass)")
    P(name="Unitree L2", role="bought", protect=True, mesh=_export(_union([g["l2"]]), out_dir, "l2.stl"), mass_g=230, note="axis 15 deg below +x")
    P(name="front camera", role="bought", protect=True, mesh=_export(_union([g["cam"]]), out_dir, "front_camera.stl"), mass_g=15)
    P(name="down camera", role="bought", protect=True, mesh=_export(_union([g["down_cam"]]), out_dir, "down_camera.stl"), mass_g=15)
    P(name="flow MTF-02P", role="bought", protect=True, box=_box(g["flow"]), mass_g=2)
    for nm, key, gm in (("Pi 5", "PI", 55), ("FC", "FC", 10), ("PDB", "PDB", 21), ("UBEC", "UBEC", 15), ("buck", "BUCK", 6), ("RX", "RX", 2)):
        P(name=nm, role="bought", protect=True, box=_box(L[key]), mass_g=gm)
    P(name="wires", role="bought", box=_box(L["FC"]), mass_g=80, note="80 g of wiring placed at the FC (layout mass model)")

    contact = [["wires", "FC"], ["pack", "cartridge"], ["Unitree L2", "apex"], ["cam bracket", "apex"], ["cage upper", "apex"],
               ["apex", "bottom plate"], ["apex", "top plate"], ["Pi 5", "Pi deck"]]
    for i in range(1, 5):
        contact += [[f"arm {i}", "top plate"], [f"arm {i}", "bottom plate"], [f"motor {i}", f"arm {i}"], ["cage upper", f"arm {i}"], ["cage lower", f"arm {i}"]]
    for i in range(1, 9):
        contact.append([f"wheel {i}", "cage lower"])
    asm = new("Overseer", "overseer", parts,
              note=("Seeded 24 Oct from nox/overseer/layout_v2.py (cage v0, the chosen case of runs/overseer_layout/v2: "
                    f"catch {dc:.0f} aft, wheels z {wz:.0f}, motors {dm:.1f} ahead, L2 base ({xB:.1f}, 0, {zB:.0f}), camera {cam_h:.0f} up the L2 plane). "
                    "Rough layout: C geometry on E sizes, masses the layout's model (printed fill factors, rods 11.0 g/m). "
                    "Drone frame: x fwd, y left, z up, z 0 bottom plate top. Contact pairs are the joints the layout script draws "
                    "(plates on the arms, motors on the pads, spokes / struts into the arms, V rods and keels into the apex, L2 seat, cam rods, "
                    "wheels in the corner nodes, Pi on its deck, the pack inside the cartridge envelope, wires lumped at the FC). "
                    "Left out (no pose in the layout): ESCs as bodies (their 25 g each is in the arm masses), wheel pins, pogo pads, receiver "
                    "and latch, scan stick mount. Openings not set: the layout defines none (L2 field and cartridge path are the owner's to add)."),
              cg_ref=[round(dm, 3), 0.0, L["PROP_Z"]], thrust_axis=[0, 0, 1], cg_tol_mm=3.0, cage_margin_mm=3.0,
              openings=[], contact_pairs=contact, guard_offsets={"guard rods to prop disks, plan (G)": L["GUARD_GAP"]})
    asm.save(out_json)
    lay_total = sum(p["mass_g"] for p in parts)
    print(f"overseer: {len(parts)} parts, assembly mass {lay_total:.1f} g vs layout cg_of {tot:.1f} g -> {out_json}")
    return asm


# ====================================================================== box catch (funnel panels v6 + hook latch v1)
def _latch():
    sys.path[:0] = [os.path.join(HERE, "box"), os.path.join(HERE, "box", "catch_v4")]
    import hook_latch as H
    best, _ = H.search()
    lay = H.servo_layout(best["r_h"], best["side"], best["body_dir"])
    return H, lay, H.build(lay)


# print-pose rotations of hook_latch.main() (housing A, housing B, plunger); the print files are these, shifted to the bed
_RA = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1.0]])
_RB = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1.0]])
_RP = np.array([[0, 0, -1, 0], [0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1.0]])


def latch_plunger(s, part):
    """moves: plunger s = 0 retracted (the file's pose) .. 1 out by hook_latch.RISE along the unit's w axis (out of the
    face). The print file's -x is the unit w (hook_latch.main: RP maps u, v, w to print z, y, -x)."""
    sys.path[:0] = [os.path.join(HERE, "box"), os.path.join(HERE, "box", "catch_v4")]
    import hook_latch as H
    T0 = np.asarray(part["pose"], float)
    w = -T0[:3, 0] / np.linalg.norm(T0[:3, 0])
    T = T0.copy(); T[:3, 3] += w * H.RISE * float(s)
    return T


def box_catch(out_json=None, out_dir=None):
    H, lay, Pp = _latch()
    import funnel_panel as F
    out_json = out_json or os.path.join(HERE, "box", "box_catch.assembly.json")
    out_dir = out_dir or os.path.join(ROOT, "runs", "box_catch_assembly", "v0")
    pa, pb = F.build()
    A = F.tm(pa.rotate([0, -F.ROT, 0])); B = F.tm(pb.rotate([0, -F.ROT, 0]))       # funnel_panel.assembly(): k = 0 copies
    mA = _export(trimesh.Trimesh(A.vertices, A.faces, process=True), out_dir, "funnel_panel_A_v6_seat.stl")
    mB = _export(trimesh.Trimesh(B.vertices, B.faces, process=True), out_dir, "funnel_panel_B_v6_seat.stl")
    parts = []
    fbase = dict(role="structure", material="PETG", tag="M", owner="box catch", version="v6", source="nox/box/catch_v4/funnel_panel.py")
    face = {0: "+x", 1: "+y", 2: "-x", 3: "-y"}
    for k in range(4):
        R = trimesh.transformations.rotation_matrix(math.radians(90 * k), [0, 0, 1])
        for nm, m in (("A", mA), ("B", mB)):
            parts.append(dict(name=f"funnel panel {nm} {face[k]}", mesh=m, pose=R.round(12).tolist(), **fbase))
    # hook latch: the committed print files posed into the seat frame (inverse of hook_latch.main's print poses)
    hA = Pp["housing"]; hB = hA.mirror([0, 1, 0])
    bA = H.to_tm(hA, _RA).bounds[0]; bB = H.to_tm(hB, _RB).bounds[0]; bP = H.to_tm(Pp["plunger"], _RP).bounds[0]
    Tr = lambda b: trimesh.transformations.translation_matrix(b)
    Mv = np.diag([1.0, -1.0, 1.0, 1.0])
    lbase = dict(material="PETG", tag="M", owner="box catch", version="v1", source="nox/box/hook_latch.py")
    for name, fs, xs in H.UNITS:
        T, mir = H.unit_frame(fs, xs)
        if not mir:
            hp, hm, hn = T @ np.linalg.inv(_RA) @ Tr(bA), "print/box_hook_latch_housing_A_v1.stl", "A"
        else:
            hp, hm, hn = T @ Mv @ np.linalg.inv(_RB) @ Tr(bB), "print/box_hook_latch_housing_B_v1.stl", "B"
        pp = T @ np.linalg.inv(_RP) @ Tr(bP)
        parts.append(dict(name=f"latch housing {name}", role="structure", mesh=hm, pose=np.round(hp, 9).tolist(), **lbase,
                          note=f"housing {hn}"))
        parts.append(dict(name=f"latch plunger {name}", role="structure", mesh="print/box_hook_latch_plunger_v1.stl", pose=np.round(pp, 9).tolist(),
                          moves="nox/assembly_seed.py:latch_plunger", **lbase,
                          note="retracted" + (" (posed mirrored, as hook_latch.py places it in this unit)" if mir else "")))
    # the seated drone's catch interface as the latch script draws it (context, E)
    D = H.drone_parts()
    cb = dict(role="structure", material="PETG", tag="E", owner="context: Scout v3 / Overseer catch interface (dimensions G)",
              version="hook_latch.drone_parts", source="nox/box/hook_latch.py")
    parts.append(dict(name="drone wheels (seated)", mesh=_export(H.to_tm(D["wheels"]), out_dir, "ctx_drone_wheels.stl"), **cb,
                      note="8 R 5 wheels on the face planes; mass is a PETG-volume stand-in"))
    parts.append(dict(name="drone cage ring (seated)", mesh=_export(H.to_tm(D["cage"]), out_dir, "ctx_drone_cage.stl"), **cb,
                      note="lower + upper ring and wheel cheeks (context); mass is a PETG-volume stand-in"))
    asm = new("Box catch", "box", parts,
              note=("Seeded 24 Oct. Frame: the seat frame of funnel_panel.py / hook_latch.py (drone coordinates of a seated drone, "
                    "contact face max(|x|, |y|) = 82.7 + z, bottom edge z 1.3). Funnel panels v6 (the printed set, built by funnel_panel.py; "
                    "the latch needs panel v7 windows and bosses, not drawn yet, so the latch housings and plungers meet the v6 skin and ribs). "
                    "Latch housings and plungers are the print/ files posed by the inverse of hook_latch.main's print poses; B-unit plungers "
                    "are posed mirrored as hook_latch.py does (one plunger file x4: CHECK the magnet pocket side). Plunger state via "
                    "'moves' (0 retracted, 1 out). Drone wheels and ring: hook_latch.drone_parts context (E; the wheels sit in their cheeks, a contact pair). Left out (no geometry or pose "
                    "on main): funnel hub + corner blocks (DESIGN, not drawn), panel v7 bosses, MG90S servos and horns (no mass in "
                    "dimensions.md), bay posts and the box frame (box stand system), KW11 + pogo post."),
              cage_margin_mm=3.0, openings=[], contact_pairs=[["drone wheels (seated)", "drone cage ring (seated)"]], guard_offsets={})
    asm.save(out_json)
    print(f"box_catch: {len(parts)} parts -> {out_json}")
    return asm


# ====================================================================== Scout v3 skeleton (dimensions.md A / C)
def scout(out_json=None):
    out_json = out_json or os.path.join(HERE, "scout", "v3", "scout.assembly.json")
    M = 54.1                    # motor centres (±54.1, ±54.1) (C Frame, P)
    PLATE_TOP, PLATE_T = 22.0, 3.5     # plate top z 22 (Coordinates), 3.5 carbon (C Frame, D8)
    PROP_Z, SWEEP = 41.2, 4.0          # prop plane (C Motor / prop layout, E) and blade sweep ±4
    base = dict(tag="E", owner="scout v3", version="skeleton", source="nox/dimensions.md A / C")
    parts = [dict(name="frame plate", role="bought", box=[-50.0, 50.0, -47.0, 47.0, PLATE_TOP - PLATE_T, PLATE_TOP], mass_g=28, **base,
                  note="centre body only: tail x -50 (C, E CHECK), nose +50 mirrored (DEFAULT), y ±47 (E), z 18.5..22; arms not drawn; "
                       "mass the middle of 24..32 (C)")]
    for i, (sx, sy) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1))):
        parts.append(dict(name=f"motor {i + 1}", role="bought", protect=True, cylinder=dict(r=12.35, p0=[sx * M, sy * M, PLATE_TOP], p1=[sx * M, sy * M, PROP_Z]),
                          mass_g=18, **base, note="EMAX ECO II 2004: plate top to prop hub face 19.2 (D12, E); Ø24.7 listing (Q6, E); 16.4..18 g, top used"))
        parts.append(dict(name=f"prop {i + 1}", role="bought", protect=True, cylinder=dict(r=44.45, p0=[sx * M, sy * M, PROP_Z - SWEEP], p1=[sx * M, sy * M, PROP_Z + SWEEP]),
                          mass_g=1.8, **base, note="Gemfan 3520 r 44.45 (A), 1.8 g (M listing), swept ±4 about the prop plane"))
    parts.append(dict(name="FC MicoAir743", role="bought", protect=True, box=[-18.0, 18.0, -18.0, 18.0, 9.5, 17.5], pose={"t": [1.9, 0, 0], "rpy_deg": [0, 0, 45]},
                      mass_g=10, **base, note="36 x 36 x 8 diamond at (1.9, 0) (A, M), lowest point drone z 9.5 (H, CHECK)"))
    asm = new("Scout v3", "scout", parts,
              note=("SKELETON (24 Oct): the Scout v3 session is building the real layout on its own branch and replaces this file; "
                    "kept minimal on purpose. Drone frame per dimensions.md Coordinates (x fwd, y left, z up, CG at x 0, plate top z 22). "
                    "Left out (no v3 pose yet, bottom layout open): 4S 1500 pack (70 x 32 x 33, 165 g, P) and cartridge, receiver and latch, "
                    "Radxa Zero 3W, flow MTF-02P, XR2, camera (pusher-era placeholder), cage and catch wheels, frame arms. "
                    "Contact pairs: each prop sits on its motor (the swept band reaches the motor top). No cage part yet, so cage first reads FAIL until the cage is drawn."),
              cg_ref=[0.0, 0.0, PROP_Z], thrust_axis=[0, 0, 1], cg_tol_mm=3.0, cage_margin_mm=3.0, openings=[],
              contact_pairs=[[f"motor {i}", f"prop {i}"] for i in range(1, 5)], guard_offsets={})
    asm.save(out_json)
    print(f"scout: {len(parts)} parts -> {out_json}")
    return asm


if __name__ == "__main__":
    which = sys.argv[1:] or ["scout", "box_catch", "overseer"]
    if "all" in which: which = ["scout", "box_catch", "overseer"]
    for w in which:
        {"overseer": overseer, "box_catch": box_catch, "scout": scout}[w]()
