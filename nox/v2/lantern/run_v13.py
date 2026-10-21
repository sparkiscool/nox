"""v12 node runs: each part at its gen target; one retry at x1.5 if it fails (one body, function, thin 0, index <= 1.0)."""
import sys, json, shutil, os, multiprocessing as mp
os.environ.setdefault("OMP_NUM_THREADS", "1"); os.environ.setdefault("MKL_NUM_THREADS", "1")
import node_topo3 as NT
def ok(r): return r.get("solid", False) and r["bodies"] == 1 and r["watertight"] and r["function"][0] < 0.5 and r["function"][1] < 0.5 and r["thin"] == 0 and r["worst"] <= 1.0
def one(name):
    h = 2.0 if name == "hub" else 1.25; trail = []; best = None
    for f in (1.0, 1.5, 2.25):
        try: r = NT.run(name, h, gen=NT.NODES[name]["gen"] * f, tag=f"_v13_g{f}")
        except Exception as e:
            trail.append(dict(gen=f, error=str(e)[:200])); print("FAILED", name, f, str(e)[:200], flush=True); continue
        if r.get("bodies", 1) != 1:          # in-memory decompose counts zero-volume pieces; the saved STL is what prints
            import trimesh
            nb = len(NT.man(trimesh.load(f"{name}_v13_g{f}.stl", force="mesh")).decompose()); r["bodies_mem"] = r["bodies"]; r["bodies"] = nb
        good = ok(r); trail.append(dict(gen=f, mass=r["mass_g"], worst=r["worst"], ok=good, bodies=r["bodies"]))
        if good: best = (f, r); break
    if best:
        f, r = best; shutil.copy(f"{name}_v13_g{f}.stl", f"v13/{name}.stl"); r["trail"] = trail; json.dump(r, open(f"v13/{name}.json", "w"), indent=1)
    else: json.dump(dict(node=name, failed=True, trail=trail), open(f"v13/{name}.json", "w"), indent=1)
    print("DONE", name, json.dumps(trail), flush=True)
if __name__ == "__main__":
    names = sys.argv[1:] or list(NT.NODES)
    with mp.Pool(2) as p: p.map(one, names, chunksize=1)
    print("ALL_DONE", flush=True)
