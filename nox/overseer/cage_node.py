"""Overseer cage nodes (printed PETG, nox/topo; Spark 27 Oct: PETG, not TPU): one topology part per node design, shaped by the tubes that meet there
(Spark 27 Oct: "start the nodes runs too, make sure each node takes either 3mm or 6mm, that's the only two rods we will use,
also do topology based on the contact of the rods for each joint so it's optimized and not overly thick and wasteful").

Each node: a drawn socket per member that ends there (6 mm carbon tube: bore 6.2, 15 deep; 3 mm carbon rod: bore 3.2, 10 deep;
a 2.5 boss wall round each), the sockets set back from the node centre just far enough that neighbouring bosses clear; the
topology joins only those bosses. Cases per member, the other members holding: its axial push and pull and two side loads
(E until the frame model: 6 mm tube 250 N axial / 60 N side, 3 mm rod 80 N / 15 N). Mirror pairs share one design (the right
one is the mirror); the front centre node is part of the sensor head mount. Members: nox/overseer/cage_v5_shell.json (tubes),
runs/overseer_layout/v5/result.json links and head_system.json wing struts (rods). Drone frame, mm, N.

python3 nox/overseer/cage_node.py list | <node> [out_dir] [--h 0.8] [--smoke] | recut <node> <mesh> [out] --ref <run commit> [--built 6.1,3.1]
"""
import os, sys, json, math, argparse
import numpy as np
import manifold3d as mf

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
CG = json.load(open(os.path.join(HERE, "cage_v5_shell.json")))
RES = json.load(open(os.path.join(ROOT, "runs", "overseer_layout", "v5", "result.json")))
HS = json.load(open(os.path.join(ROOT, "runs", "overseer_layout", "v5", "head_system.json")))
N = np.array(CG["nodes"])
KINDS = {6: dict(bore=6.2, depth=15.0, axial=250.0, side=60.0), 3: dict(bore=3.2, depth=10.0, axial=80.0, side=15.0)}   # E; bores Spark 27 Oct (coupons v2: the largest fits)
WALL = 2.5
_LS = os.path.join(HERE, "link_sizes.json")
LINK6 = set(json.load(open(_LS))["tube6"]) if os.path.exists(_LS) else set()      # links the frame model made 6/4 tube
_NL = os.path.join(ROOT, "runs", "overseer_frame", "v1", "node_loads.json")
NL = json.load(open(_NL)) if os.path.exists(_NL) else {}                          # frame-model socket forces (F), else KINDS (E)
_k = [i for i in range(len(N)) if abs(N[i, 1]) < 0.5 and N[i, 0] > 300]
HEAD_NODE = N[max(_k, key=lambda i: N[i, 0])] if _k else np.array([352.88, 0.0, 47.0])   # the front centre node: part of the sensor head mount


HEAD_STRUTS = [((401.0, 83.0, 151.0), (354.0, 223.0, 128.7)), ((383.0, 83.0, 73.0), (336.5, 219.0, -29.3))]   # = head_mount.HEAD_STRUTS


FLOOR = 3.0                                      # solid boss floor behind each socket (C, DEFAULT 27 Oct: 1.0 left the arm-to-boss joint a neck under 1.8)
ARM_R, HUB_R = 3.5, 6.0                          # drawn arm Ø7 (PETG, 3.5 over the 2.0 minimum) and hub Ø12 at the node centre (DEFAULT)


def members(i):
    """(direction, kind) of every member ending at node i"""
    out = []
    for a, b in CG["rods"]:
        if i in (a, b): o = N[b if a == i else a]; out.append(((o - N[i]) / np.linalg.norm(o - N[i]), 6, f"tube to {b if a == i else a}"))
    for k, (p, q) in RES["links"].items():
        if k.startswith("head strut"): continue                        # below, from HEAD_STRUTS
        for e, o in ((p, q), (q, p)):
            if np.linalg.norm(np.array(e) - N[i]) < 0.6:
                o = np.array(o); out.append(((o - N[i]) / np.linalg.norm(o - N[i]), 6 if k in LINK6 else 3, k))
    for w, q in HEAD_STRUTS:                            # the one-piece head's struts to the side nodes (head_mount.HEAD_STRUTS)
        for sy in (1, -1):
            w_, q_ = np.array(w) * [1, sy, 1], np.array(q) * [1, sy, 1]
            if np.linalg.norm(q_ - N[i]) < 1.0:
                out.append(((w_ - N[i]) / np.linalg.norm(w_ - N[i]), 6 if f"head strut {i}" in LINK6 else 3, f"head strut {i}"))
    return out


def mirror_of(i):
    q = N[i] * [1, -1, 1]; return int(np.argmin(np.linalg.norm(N - q, axis=1)))


def socket_load(i, name):
    """the frame model's largest socket force (node_loads.json) for member `name` at node i or at its mirror (one design for the
    pair): {"axial", "side"} in N, or None. Tubes are "tube a-b" there in the cage file's order (27 Oct: the old lookup took
    min-max and missed most tubes, so the node runs took the E loads), links by name, mirrored links with or without " m"."""
    out = None
    for j in {i, mirror_of(i)}:
        L = NL.get(str(j), {})
        if name.startswith("tube to"):
            o = int(name.split()[-1]); o = o if j == i else mirror_of(o)
            keys = [f"tube {j}-{o}", f"tube {o}-{j}"]
        else:
            keys = [name, name + " m", name[:-2] if name.endswith(" m") else name]
            if j != i: keys += [k.replace("+1", "-1") if "+1" in k else k.replace("-1", "+1") for k in keys]
        for k in keys:
            if k in L:
                e = L[k]; out = {"axial": max(e["axial"], (out or {}).get("axial", 0.0)), "side": max(e["side"], (out or {}).get("side", 0.0))}; break
    return out


def designs():
    """one design per mirror pair (the y >= 0 node) and per centre-plane node; the head's node left out"""
    return [i for i in range(len(N)) if N[i, 1] > -0.5 and not np.allclose(N[i], HEAD_NODE, atol=0.5)]


def seg(a, b, r, n=32):
    a, b = np.asarray(a, float), np.asarray(b, float); d = b - a; L = float(np.linalg.norm(d))
    z = d / L; x = np.cross(z, [0, 0, 1.0]) if abs(z[2]) < 0.9 else np.cross(z, [1.0, 0, 0]); x /= np.linalg.norm(x); y = np.cross(z, x)
    T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, a
    return mf.Manifold.cylinder(L, r, r, n).transform(T[:3, :4])


def bodies(i):
    c = N[i]; M = members(i)
    # socket set-back: the bosses of every two members clear each other at the socket mouth
    r0 = 6.0
    for p in range(len(M)):
        for q in range(p + 1, len(M)):
            th = math.acos(np.clip(M[p][0] @ M[q][0], -1, 1))
            gap = (KINDS[M[p][1]]["bore"] + KINDS[M[q][1]]["bore"]) / 2 + 1.0   # the two bores clear at the socket mouths
            r0 = max(r0, gap / max(math.sin(th), 0.15)) if th < math.pi / 2 else r0
    r0 = min(r0, 30.0)
    drawn = cuts = None; sockets = []
    for d, k, nm in M:
        K = KINDS[k]; a = c + d * r0; b = a + d * K["depth"]
        boss = seg(a - d * FLOOR, b, K["bore"] / 2 + WALL)            # the boss with a solid floor FLOOR thick behind the socket floor
        boss = boss + seg(c, a, ARM_R)                                # a drawn arm from the socket floor to the hub: one body by construction
                                                                      # (v3 runs: the topology alone left sockets disconnected; node 2 after the
                                                                      # loop drop: the arm met the boss through a 1.0 floor, a neck under 1.8, 27 Oct)
        cut = seg(a, b + d * 400.0, K["bore"] / 2) + seg(b + d * 0.5, b + d * 400.0, K["bore"] / 2 + 1.0)   # bore + the member beyond
        drawn = boss if drawn is None else drawn + boss; cuts = cut if cuts is None else cuts + cut
        sockets.append(dict(boss=boss, d=d, kind=k, name=nm))
    drawn = drawn + mf.Manifold.sphere(HUB_R, 48).translate(list(map(float, c)))   # the hub the arms meet in
    R = r0 + max(KINDS[k]["depth"] for _, k, _ in M) + 1.0
    env = mf.Manifold.sphere(R, 64).translate(list(map(float, c)))
    return dict(drawn=drawn, cuts=cuts, env=env, sockets=sockets, r0=r0, R=R)


def _at_ref(ref, path):
    import subprocess
    return json.loads(subprocess.run(["git", "-C", ROOT, "show", f"{ref}:{path}"], capture_output=True, text=True, check=True).stdout)


def recut(i, mesh_in, mesh_out=None, built=(6.1, 3.1), ref=None):
    """make a saved node's sockets the current bores without a rerun (Spark 27 Oct: "dont restart runs just make the hole match"):
    the sockets placed as the run built them (its bores `built` set the set-back; with `ref`, the link ends, sizes and cage of the
    commit the run was built on, so every axis is the run's own), then each bore cut again at KINDS' bore along the same axis from
    the same floor. A larger bore only removes material. Returns the output path; run solid_check on it after. Limit: the head
    struts (HEAD_STRUTS) are this file's, not the ref's; a node with one (15 / 16 / 18 / 19) needs the strut ends of its run."""
    import trimesh
    global RES, LINK6, CG, N
    keep = {k: dict(v) for k, v in KINDS.items()}; was = (RES, LINK6, CG, N)
    if ref:
        RES = _at_ref(ref, "runs/overseer_layout/v5/result.json"); LINK6 = set(_at_ref(ref, "nox/overseer/link_sizes.json")["tube6"])
        CG = _at_ref(ref, "nox/overseer/cage_v5_shell.json"); N = np.array(CG["nodes"])
    KINDS[6]["bore"], KINDS[3]["bore"] = built
    try: b = bodies(i); c = N[i]
    finally: KINDS.update(keep); RES, LINK6, CG, N = was
    cut = None
    for sk in b["sockets"]:
        d, k = sk["d"], sk["kind"]; a = c + d * b["r0"]; bb = a + d * KINDS[k]["depth"]
        m = seg(a, bb + d * 400.0, KINDS[k]["bore"] / 2); cut = m if cut is None else cut + m
    tm = trimesh.load(mesh_in, force="mesh")
    part = mf.Manifold(mf.Mesh(vert_properties=np.asarray(tm.vertices, np.float32), tri_verts=np.asarray(tm.faces, np.uint32)))
    out = (part - cut).to_mesh()
    res = trimesh.Trimesh(np.asarray(out.vert_properties)[:, :3], np.asarray(out.tri_verts), process=True)
    res.export(mesh_out or mesh_in)
    print(f"node {i}: {len(b['sockets'])} sockets re-cut from {built} to {KINDS[6]['bore']} / {KINDS[3]['bore']}; "
          f"volume {tm.volume / 1000:.2f} -> {res.volume / 1000:.2f} cm3; {mesh_out or mesh_in}")
    return mesh_out or mesh_in


def setup_for(i):
    def setup(h):
        from topo import Part
        b = bodies(i)
        p = Part(f"node_{i}", h, b["drawn"], b["cuts"], b["env"], build="+z", material="PETG")   # PETG cage nodes (CLAUDE.md, Spark 27 Oct)
        S = b["sockets"]
        for j, s in enumerate(S):                        # member j loaded, every other member holding
            hold = None
            for t, o in enumerate(S):
                if t != j: hold = o["boss"] if hold is None else hold + o["boss"]
            p.support(f"hold {j}", hold)
            K = dict(KINDS[s["kind"]]); d = s["d"]
            fl = socket_load(i, s["name"])
            if fl: K["axial"], K["side"] = max(fl["axial"], 20.0), max(fl["side"], 5.0)   # the frame model's crash forces (F)
            x = np.cross(d, [0, 0, 1.0]) if abs(d[2]) < 0.9 else np.cross(d, [1.0, 0, 0]); x /= np.linalg.norm(x); y = np.cross(d, x)
            for nm, f in (("pull", K["axial"] * d), ("push", -K["axial"] * d), ("side a", K["side"] * x), ("side b", K["side"] * y)):
                p.load(f"{s['name']} {nm}", s["boss"], f, cls="misuse", support=f"hold {j}")
        return p
    return setup


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "recut":                   # cage_node.py recut <node> <mesh> [out] --ref <git ref of the run> [--built 6.1,3.1]
        rp = argparse.ArgumentParser(); rp.add_argument("cmd"); rp.add_argument("node", type=int); rp.add_argument("mesh"); rp.add_argument("out", nargs="?")
        rp.add_argument("--ref", required=True, help="the git commit the run was built on (its link ends, sizes and cage)")
        rp.add_argument("--built", default="6.1,3.1"); r = rp.parse_args()
        recut(r.node, r.mesh, r.out, tuple(map(float, r.built.split(","))), ref=r.ref); sys.exit(0)
    ap = argparse.ArgumentParser(); ap.add_argument("node"); ap.add_argument("out", nargs="?")
    ap.add_argument("--h", type=float, default=0.8); ap.add_argument("--gen", type=float, default=None)
    ap.add_argument("--git", action="store_true"); ap.add_argument("--smoke", action="store_true")
    ap.add_argument("--drawn", action="store_true", help="drawn first: the drawn node alone, solid gate + strength on the frame loads; saved if it passes")
    a = ap.parse_args()
    if a.node == "list":
        for i in designs():
            M = members(i); b = bodies(i)
            print(f"node {i:2d} at {np.round(N[i], 1).tolist()}: {sum(k == 6 for _, k, _ in M)} x 6 mm, {sum(k == 3 for _, k, _ in M)} x 3 mm, "
                  f"sockets {b['r0']:.1f} from the centre, envelope r {b['R']:.0f}" + (" (+ its mirror)" if N[i, 1] > 0.5 else ""))
        sys.exit(0)
    i = int(a.node); setup = setup_for(i)
    if a.smoke:
        p = setup(a.h); print(f"node {i} grid", p.shape, "ok"); sys.exit(0)
    if a.drawn:                                                         # drawn first (CLAUDE.md): no topology run when the drawn node passes
        from topo import drawn_check
        out = a.out or os.path.join(ROOT, "runs", "cage_nodes", "v4", f"node_{i}"); os.makedirs(out, exist_ok=True)
        stl = os.path.join(out, f"cage_node_{i}.stl")
        d = drawn_check(setup, a.h, gen_cm3=1.2, stl=stl); d["members"] = [nm for _, _, nm in members(i)]
        d["loads"] = {nm: socket_load(i, nm) for _, _, nm in members(i)}; d["mirror"] = mirror_of(i) if N[i, 1] > 0.5 else None
        json.dump(d, open(os.path.join(out, "drawn.json"), "w"), indent=1)
        print(f"node {i}: {d['mass_g']} g (M), solid {'PASS' if d['solid_ok'] else 'FAIL'}, worst {d['worst']} ({d['worst_case']}): {'PASS' if d['pass'] else 'FAIL'}")
        sys.exit(0 if d["pass"] else 1)
    from topo import Settings, optimise
    out = a.out or os.path.join(ROOT, "runs", f"cage_node_{i}", "v1")
    gen = a.gen if a.gen is not None else 1.2                       # cm3 generated (E; v3's node estimate 4.5 g in all)
    optimise(setup, h=a.h, gen_cm3=gen, out_dir=out, name=f"cage_node_{i}", settings=Settings(), git=a.git)
