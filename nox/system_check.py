"""Rule checks on the assembled model (Spark 24 Oct: system first, cage first, compact). One manifest lists every part
in its assembled pose; the tool reports:
  cage first  support functions over N Fibonacci directions: margin(d) = h_cage(d) - h_protected(d), h(d) = max v.d.
              PASS if margin >= cage_margin_mm (3.0 DEFAULT) in every direction for every protected part; inside an
              opening cone (axis + half-angle, the cartridge path, the head) that part only needs margin >= 0 (recessed).
  compact     assembled AABB volume (L), extents (mm), largest extent, convex hull volume (L), guard offsets (reported).
  fit         pairwise interference volume (mm^3, manifold3d booleans, trimesh / voxel fallback). PASS if every pair
              <= fit_tol_mm3 (0.5), a contact_pairs entry [a, b, allowed_mm3] lifts that pair's limit. Minimum clearance
              on clearance_pairs [a, b(, min_mm)] (reported).
  mass / CG   mass_g if given, else mesh volume x density (PETG 1.27, TPU 1.21, carbon 1.6 g/cm^3); bought parts must
              give mass_g. Whole mass and CG. Balance (Spark 24 Oct, CG in the middle): with cg_ref (the thrust centre)
              the CG offset normal to thrust_axis (+z) must be <= cg_tol_mm (3.0 DEFAULT), with and without the parts
              marked "swap": true (pack, cartridge), so a swap does not move the CG off the axis.
Cage first, fit and balance gate (exit 1); everything else is reported. Tags: C manifest geometry / mass, M mesh-derived,
E when any input part is tagged E, P measured.
Limits: a support function sees each role's convex hull, so it models flat walls only: a post or edge poking between
cage members, or a protected part inside a concave pocket of the cage, is not seen. Clearance is sampled (surface points).

python3 system_check.py manifest.json [--out runs/<sys>/<ver>/system_check.json] [--report] [--dirs 2000]
from system_check import run; res = run(manifest_dict_or_path); res["summary"], res["pass"]

Manifest: {"name", "parts": [{"name", "mesh": path (STL/OBJ/PLY, relative to the manifest, cwd or repo root)
  | "box": [x0,x1,y0,y1,z0,z1] | "cylinder": {"r", "p0", "p1"} or {"r", "h", "center", "axis"} | "sphere": {"r", "center"},
  "pose": 4x4 | {"t": [x,y,z], "rpy_deg": [r,p,y]} (R = Rz(y) Ry(p) Rx(r)), "role": cage|protected|structure|bought,
  "protect": true (any role), "material", "mass_g", "tag": M|C|E|P, "cg": [x,y,z] in part frame (optional), "swap": true (optional)}],
  "openings": [{"part" (omit = every protected part), "axis", "half_angle_deg", "min_mm": 0.0}],
  "contact_pairs": [[a, b, allowed_mm3]], "clearance_pairs": [[a, b, min_mm]], "guard_offsets": {name: mm},
  "cg_ref": [x,y,z], "thrust_axis": [0,0,1], "cg_tol_mm": 3.0, "cage_margin_mm": 3.0, "fit_tol_mm3": 0.5, "n_dirs": 2000}. From Python a part may carry "trimesh": <Trimesh>."""
import sys, os, json, argparse, numpy as np, trimesh

DENSITY = {"petg": 1.27, "tpu": 1.21, "carbon": 1.6, "carbon rod": 1.6, "cf": 1.6}   # g/cm^3
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TAG_RANK = {"P": 0, "C": 1, "M": 2, "E": 3}                                          # combined tag = worst input


def tag_of(tags):
    tags = [t for t in tags if t]
    return max(tags, key=lambda t: TAG_RANK.get(t, 3)) if tags else "C"


def fib_dirs(n):
    i = np.arange(n) + 0.5
    z = 1 - 2 * i / n; r = np.sqrt(1 - z * z); a = np.pi * (3 - np.sqrt(5)) * i
    return np.stack([r * np.cos(a), r * np.sin(a), z], 1)


def pose_matrix(p):
    if p is None: return np.eye(4)
    if isinstance(p, dict):
        r, pi, y = np.radians(p.get("rpy_deg", [0, 0, 0]))
        cx, sx, cy, sy, cz, sz = np.cos(r), np.sin(r), np.cos(pi), np.sin(pi), np.cos(y), np.sin(y)
        Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]]); Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
        T = np.eye(4); T[:3, :3] = Rz @ Ry @ Rx; T[:3, 3] = p.get("t", [0, 0, 0]); return T
    T = np.asarray(p, float); assert T.shape == (4, 4), "pose must be 4x4 or {t, rpy_deg}"; return T


def _resolve(path, base):
    for d in (base, os.getcwd(), REPO):
        q = os.path.join(d, path)
        if os.path.exists(q): return q
    raise FileNotFoundError(path)


def load_part(p, base="."):
    """part dict -> (trimesh in assembled pose, geometry tag)."""
    if "trimesh" in p:                                                # Python API: a trimesh already built
        m = p["trimesh"].copy(); src = p.get("geom_tag", "M")
    elif "mesh" in p:
        m = trimesh.load(_resolve(p["mesh"], base), force="mesh"); src = "M"
    elif "box" in p:
        x0, x1, y0, y1, z0, z1 = p["box"]
        m = trimesh.creation.box([x1 - x0, y1 - y0, z1 - z0]); m.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]); src = "C"
    elif "cylinder" in p:
        c = p["cylinder"]
        if "p0" in c:
            m = trimesh.creation.cylinder(radius=c["r"], segment=[c["p0"], c["p1"]], sections=64)
        else:
            ax = {"x": [1, 0, 0], "y": [0, 1, 0], "z": [0, 0, 1]}.get(c.get("axis", "z"), c.get("axis"))
            ax = np.asarray(ax, float); ax /= np.linalg.norm(ax); ctr = np.asarray(c.get("center", [0, 0, 0]), float)
            m = trimesh.creation.cylinder(radius=c["r"], segment=[ctr - ax * c["h"] / 2, ctr + ax * c["h"] / 2], sections=64)
        src = "C"
    elif "sphere" in p:
        s = p["sphere"]; m = trimesh.creation.icosphere(subdivisions=4, radius=s["r"]); m.apply_translation(s.get("center", [0, 0, 0])); src = "C"
    else:
        raise ValueError(f"part {p.get('name')}: needs mesh, box, cylinder or sphere")
    m.apply_transform(pose_matrix(p.get("pose")))
    return m, ("E" if p.get("tag") == "E" else src)


def hull_pts(m):
    try:
        from scipy.spatial import ConvexHull
        V = m.vertices; return V[ConvexHull(V).vertices]
    except Exception:
        return m.vertices


def dir_label(d):
    k = int(np.argmax(np.abs(d)))
    if abs(d[k]) > np.cos(np.radians(10)): return ("+" if d[k] > 0 else "-") + "XYZ"[k]
    return "(" + ", ".join(f"{x:+.2f}" for x in d) + ")"


# ---------- cage first
def cage_first(parts, meshes, openings, margin_mm, n_dirs):
    D = fib_dirs(n_dirs)
    D = np.vstack([D, np.eye(3), -np.eye(3)])                         # the six axes always sampled
    cage = [i for i, p in enumerate(parts) if p.get("role") == "cage"]
    prot = [i for i, p in enumerate(parts) if p.get("role") == "protected" or p.get("protect")]
    if not prot: return dict(applies=False, pass_=True)
    if not cage: return dict(applies=True, pass_=False, error="protected parts but no cage part")
    hc = (np.vstack([hull_pts(meshes[i]) for i in cage]) @ D.T).max(0)
    worst = None; per = {}
    for i in prot:
        nm = parts[i]["name"]; hp = (hull_pts(meshes[i]) @ D.T).max(0); mg = hc - hp
        req = np.full(len(D), float(margin_mm))
        for o in openings:
            if o.get("part") not in (None, nm): continue
            ax = np.asarray(o["axis"], float); ax /= np.linalg.norm(ax)
            req[D @ ax >= np.cos(np.radians(o["half_angle_deg"]))] = min(float(o.get("min_mm", 0.0)), margin_mm)
        sl = mg - req; k = int(np.argmin(sl)); free = req == margin_mm
        per[nm] = dict(min_margin_mm=round(float(mg.min()), 2), worst_dir=[round(float(x), 3) for x in D[k]],
                       worst_slack_mm=round(float(sl[k]), 2), min_margin_outside_openings_mm=round(float(mg[free].min()), 2) if free.any() else None,
                       dirs_in_openings=int((~free).sum()), pass_=bool(sl.min() >= -1e-9))
        if worst is None or sl[k] < worst[0]: worst = (float(sl[k]), nm, D[k], float(mg[k]), float(req[k]))
    s, nm, d, mg, rq = worst
    return dict(applies=True, pass_=bool(s >= -1e-9), worst_part=nm, worst_dir=[round(float(x), 3) for x in d], worst_dir_label=dir_label(d),
                margin_mm=round(mg, 2), required_mm=rq, n_dirs=len(D), cage_margin_mm=margin_mm, per_part=per)


# ---------- compact
def compact(meshes, guard):
    V = np.vstack([m.vertices for m in meshes]); lo, hi = V.min(0), V.max(0); ext = hi - lo
    try: hv = trimesh.PointCloud(V).convex_hull.volume
    except Exception: hv = float("nan")
    return dict(bbox_min=lo.round(2).tolist(), bbox_max=hi.round(2).tolist(), extents_mm=ext.round(1).tolist(),
                largest_extent_mm=round(float(ext.max()), 1), bbox_L=round(float(np.prod(ext)) / 1e6, 3),
                hull_L=round(float(hv) / 1e6, 3), guard_offsets_mm=guard or {})


# ---------- fit
def _manifold(m):
    import manifold3d as M3
    mm = M3.Manifold(M3.Mesh(vert_properties=np.asarray(m.vertices, np.float32), tri_verts=np.asarray(m.faces, np.uint32)))
    if mm.status() != M3.Error.NoError: raise ValueError(str(mm.status()))
    return mm


def _voxel_overlap(a, b, lo, hi, pitch=0.5):
    """fallback: inside-both count on a grid over the AABB overlap (needs closed meshes)."""
    n = np.maximum(np.ceil((hi - lo) / pitch).astype(int), 1)
    if np.prod(n) > 4e6: pitch = float((np.prod(hi - lo) / 4e6) ** (1 / 3)); n = np.maximum(np.ceil((hi - lo) / pitch).astype(int), 1)
    g = [lo[k] + (np.arange(n[k]) + 0.5) * (hi[k] - lo[k]) / n[k] for k in range(3)]
    P = np.stack(np.meshgrid(*g, indexing="ij"), -1).reshape(-1, 3)
    ins = a.contains(P); ins[ins] = b.contains(P[ins])
    return float(ins.sum() * np.prod((hi - lo) / n))


def interference(a, b, cache, ia, ib):
    lo = np.maximum(a.bounds[0], b.bounds[0]); hi = np.minimum(a.bounds[1], b.bounds[1])
    if np.any(hi <= lo): return 0.0, "aabb"
    try:
        for i, m in ((ia, a), (ib, b)):
            if i not in cache: cache[i] = _manifold(m)
        return float((cache[ia] ^ cache[ib]).volume()), "manifold3d"
    except Exception:
        pass
    try:
        return float(trimesh.boolean.intersection([a, b]).volume), "trimesh"
    except Exception:
        return _voxel_overlap(a, b, lo, hi), "voxel"


def clearance(a, b, n=3000):
    sub = lambda m: m.vertices[np.random.default_rng(0).choice(len(m.vertices), min(len(m.vertices), n), replace=False)]
    pa = np.vstack([sub(a), a.sample(n)]); pb = np.vstack([sub(b), b.sample(n)])
    da = trimesh.proximity.ProximityQuery(b).signed_distance(pa)        # + inside b
    db = trimesh.proximity.ProximityQuery(a).signed_distance(pb)
    if (da > 0).any() or (db > 0).any(): return 0.0
    return float(min((-da).min(), (-db).min()))


def fit(parts, meshes, contact, tol, clear_pairs):
    idx = {p["name"]: i for i, p in enumerate(parts)}
    allow = {frozenset(c[:2]): float(c[2]) if len(c) > 2 else float("inf") for c in contact}
    pairs = []; cache = {}
    for i in range(len(parts)):
        for j in range(i + 1, len(parts)):
            v, how = interference(meshes[i], meshes[j], cache, i, j)
            key = frozenset((parts[i]["name"], parts[j]["name"])); lim = allow.get(key, tol)
            if v > 1e-6 or key in allow:
                pairs.append(dict(a=parts[i]["name"], b=parts[j]["name"], overlap_mm3=round(v, 2), limit_mm3=lim,
                                  contact=key in allow, method=how, pass_=bool(v <= lim)))
    bad = [p for p in pairs if not p["pass_"]]
    nonex = [p for p in pairs if not p["contact"]]
    w = max(bad or nonex or [None], key=lambda p: -1 if p is None else p["overlap_mm3"])
    cl = []
    for c in clear_pairs:
        d = clearance(meshes[idx[c[0]]], meshes[idx[c[1]]])
        cl.append(dict(a=c[0], b=c[1], clearance_mm=round(d, 2), min_mm=c[2] if len(c) > 2 else None,
                       ok=None if len(c) < 3 else bool(d >= c[2])))
    return dict(pass_=not bad, tol_mm3=tol, worst=w, pairs=pairs, clearance=cl)


# ---------- mass / CG
def mass_cg(parts, meshes):
    rows = []; M = 0.0; S = np.zeros(3)
    for p, m in zip(parts, meshes):
        if "mass_g" in p:
            g = float(p["mass_g"]); t = p.get("tag", "C")
        else:
            if p.get("role") == "bought": raise ValueError(f"bought part {p['name']}: give mass_g")
            mat = str(p.get("material", "PETG")).lower()
            if mat not in DENSITY: raise ValueError(f"part {p['name']}: unknown material {mat} (PETG, TPU, carbon)")
            g = abs(m.volume) / 1000 * DENSITY[mat]; t = "E" if p.get("tag") == "E" else ("M" if "mesh" in p or "trimesh" in p else "C")
        if "cg" in p:
            c = (pose_matrix(p.get("pose")) @ np.r_[np.asarray(p["cg"], float), 1])[:3]
        else:
            c = m.center_mass if m.is_watertight and abs(m.volume) > 1e-9 else m.bounds.mean(0)
        rows.append(dict(name=p["name"], mass_g=round(g, 2), cg=np.round(c, 2).tolist(), tag=t, swap=bool(p.get("swap")), _g=g, _c=c))
        M += g; S += g * c
    cg = S / M if M > 0 else np.zeros(3)
    return dict(mass_g=round(M, 2), cg=np.round(cg, 2).tolist(), parts=rows, tag=tag_of([r["tag"] for r in rows]))


def balance(rows, ref, axis, tol):
    """CG offset from ref, split into the part along the thrust axis and the horizontal part normal to it; with and
    without the swap parts. PASS if every horizontal offset <= tol."""
    ax = np.asarray(axis, float); ax /= np.linalg.norm(ax); ref = np.asarray(ref, float); out = {}
    cases = [("all", rows)] + ([("no_swap", [r for r in rows if not r["swap"]])] if any(r["swap"] for r in rows) else [])
    for k, rs in cases:
        g = sum(r["_g"] for r in rs)
        if g <= 0: continue
        off = sum(r["_g"] * r["_c"] for r in rs) / g - ref; h = off - (off @ ax) * ax
        out[k] = dict(mass_g=round(g, 2), cg_offset=np.round(off, 2).tolist(), horizontal_mm=round(float(np.linalg.norm(h)), 2),
                      axial_mm=round(float(off @ ax), 2), pass_=bool(np.linalg.norm(h) <= tol + 1e-9))
    w = max(out, key=lambda k: out[k]["horizontal_mm"])
    return dict(applies=True, ref=ref.tolist(), thrust_axis=ax.round(4).tolist(), tol_mm=tol, cases=out, worst_case=w,
                worst_horizontal_mm=out[w]["horizontal_mm"], pass_=all(c["pass_"] for c in out.values()))


# ---------- run
def run(manifest, n_dirs=None):
    base = "."
    if isinstance(manifest, str):
        base = os.path.dirname(os.path.abspath(manifest)); manifest = json.load(open(manifest))
    parts = manifest["parts"]
    names = [p["name"] for p in parts]; assert len(set(names)) == len(names), "part names must be unique"
    loaded = [load_part(p, base) for p in parts]; meshes = [m for m, _ in loaded]; gtags = [t for _, t in loaded]
    anyE = any(p.get("tag") == "E" for p in parts)
    gtag = "E" if anyE else tag_of(gtags)
    n = n_dirs or manifest.get("n_dirs", 2000); cm = float(manifest.get("cage_margin_mm", 3.0))
    cf = cage_first(parts, meshes, manifest.get("openings", []), cm, n)
    cp = compact(meshes, manifest.get("guard_offsets"))
    ft = fit(parts, meshes, manifest.get("contact_pairs", []), float(manifest.get("fit_tol_mm3", 0.5)), manifest.get("clearance_pairs", []))
    mc = mass_cg(parts, meshes)
    if anyE: mc["tag"] = "E"
    ref = manifest.get("cg_ref")
    if ref is not None:
        off = np.asarray(mc["cg"]) - np.asarray(ref, float)
        mc["cg_ref"] = list(ref); mc["cg_offset"] = np.round(off, 2).tolist(); mc["cg_offset_mm"] = round(float(np.linalg.norm(off)), 2)
        bal = balance(mc["parts"], ref, manifest.get("thrust_axis", [0, 0, 1]), float(manifest.get("cg_tol_mm", 3.0)))
    else:
        bal = dict(applies=False, pass_=True)
    for r in mc["parts"]: r.pop("_g"); r.pop("_c")
    name = manifest.get("name", "system")
    if not cf["applies"]: cfs = "cage first n/a (no protected part)"
    elif "error" in cf: cfs = f"cage first FAIL ({cf['error']})"
    else: cfs = f"cage first {cf['margin_mm']:.1f} mm ({gtag}) at {cf['worst_dir_label']} ({cf['worst_part']}){'' if cf['pass_'] else ' FAIL'}"
    w = ft["worst"]
    fts = f"fit {w['overlap_mm3']:.1f} mm³ ({gtag}) ({w['a']}/{w['b']}){'' if ft['pass_'] else ' FAIL'}" if w else f"fit 0.0 mm³ ({gtag})"
    ex = cp["extents_mm"]
    if bal["applies"]:
        a = bal["cases"]["all"]; ns = bal["cases"].get("no_swap")
        cgs = (f"CG {a['horizontal_mm']:.1f} mm off thrust axis ({mc['tag']}) [{', '.join(f'{x:+.1f}' for x in a['cg_offset'])}]"
               + (f", {ns['horizontal_mm']:.1f} mm without swap parts" if ns else "") + ("" if bal["pass_"] else f" FAIL (tol {bal['tol_mm']:g})"))
    else:
        cgs = f"CG [{', '.join(f'{x:.1f}' for x in mc['cg'])}] ({mc['tag']}, no cg_ref)"
    summary = (f"System: {name}, {cfs}, {fts}, bbox {cp['bbox_L']:.3g} L ({gtag}), extents {ex[0]:.0f} x {ex[1]:.0f} x {ex[2]:.0f} mm ({gtag}), "
               f"mass {mc['mass_g']:.{0 if mc['mass_g'] >= 100 else 1}f} g ({mc['tag']}), {cgs}")
    ok = bool(cf["pass_"] and ft["pass_"] and bal["pass_"])
    return dict(name=name, pass_=ok, cage_first=cf, compact=cp, fit=ft, mass_cg=mc, balance=bal,
                geometry_tag=gtag, summary=summary, **{"pass": ok})


def report(res):
    cf, cp, ft, mc, t = res["cage_first"], res["compact"], res["fit"], res["mass_cg"], res["geometry_tag"]
    L = [f"system_check {res['name']}: {'PASS' if res['pass'] else 'FAIL'}"]
    if cf["applies"] and "error" not in cf:
        L.append(f"  cage first {'PASS' if cf['pass_'] else 'FAIL'}: worst {cf['margin_mm']} mm ({t}) at {cf['worst_dir_label']} on {cf['worst_part']} "
                 f"(needs {cf['required_mm']}), {cf['n_dirs']} dirs")
        for nm, r in cf["per_part"].items():
            L.append(f"    {nm}: min margin {r['min_margin_mm']} mm, outside openings {r['min_margin_outside_openings_mm']}, slack {r['worst_slack_mm']} "
                     f"at {dir_label(np.array(r['worst_dir']))}{'' if r['pass_'] else ' FAIL'}")
    else:
        L.append("  cage first: " + cf.get("error", "n/a (no protected part)"))
    L.append(f"  compact: bbox {cp['bbox_L']} L ({t}), hull {cp['hull_L']} L ({t}), extents {cp['extents_mm']} mm ({t}), largest {cp['largest_extent_mm']} mm"
             + (f", guards {cp['guard_offsets_mm']}" if cp["guard_offsets_mm"] else ""))
    L.append(f"  fit {'PASS' if ft['pass_'] else 'FAIL'} (tol {ft['tol_mm3']} mm³):")
    for p in ft["pairs"]:
        L.append(f"    {p['a']} / {p['b']}: {p['overlap_mm3']} mm³ ({t}) limit {p['limit_mm3']}{' contact' if p['contact'] else ''} [{p['method']}]{'' if p['pass_'] else ' FAIL'}")
    for c in ft["clearance"]:
        L.append(f"    clearance {c['a']} / {c['b']}: {c['clearance_mm']} mm ({t})" + ("" if c["ok"] is None else f" (min {c['min_mm']}{'' if c['ok'] else ', LOW'})"))
    L.append(f"  mass {mc['mass_g']} g ({mc['tag']}), CG {mc['cg']}" + (f", offset {mc['cg_offset']} = {mc['cg_offset_mm']} mm from {mc['cg_ref']}" if "cg_ref" in mc else ""))
    for r in mc["parts"]:
        L.append(f"    {r['name']}: {r['mass_g']} g ({r['tag']}) at {r['cg']}{' swap' if r['swap'] else ''}")
    b = res["balance"]
    if b["applies"]:
        L.append(f"  balance {'PASS' if b['pass_'] else 'FAIL'} (horizontal CG offset <= {b['tol_mm']} mm, thrust axis {b['thrust_axis']}):")
        for k, c in b["cases"].items():
            L.append(f"    {k}: {c['horizontal_mm']} mm horizontal, {c['axial_mm']} mm axial, {c['mass_g']} g{'' if c['pass_'] else ' FAIL'}")
    else:
        L.append("  balance: n/a (no cg_ref)")
    L.append(res["summary"])
    return "\n".join(L)


def main(argv=None):
    ap = argparse.ArgumentParser(description="cage first, compact, fit, mass/CG on an assembled manifest")
    ap.add_argument("manifest"); ap.add_argument("--out"); ap.add_argument("--report", action="store_true", help="report only, exit 0")
    ap.add_argument("--dirs", type=int, default=None)
    a = ap.parse_args(argv)
    res = run(a.manifest, n_dirs=a.dirs)
    print(report(res))
    if a.out:
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        json.dump(res, open(a.out, "w"), indent=1, default=float)
    return 0 if (res["pass"] or a.report) else 1


if __name__ == "__main__":
    sys.exit(main())
