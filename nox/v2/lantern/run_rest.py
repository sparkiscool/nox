"""One run per node at gen x FACTOR (time-boxed pass; the protocol's mass stepping is the follow-up)."""
import sys, json, shutil, os, multiprocessing as mp
os.environ.setdefault("OMP_NUM_THREADS", "1"); os.environ.setdefault("MKL_NUM_THREADS", "1")
import node_topo as NT
FACTOR = float(os.environ.get("FACTOR", "0.5"))
def one(name):
    h = 2.0 if name == "hub" else 1.25
    try:
        r = NT.run(name, h, gen=NT.NODES[name]["gen"] * FACTOR, tag=f"_g{FACTOR}")
    except Exception as e:
        json.dump(dict(node=name, failed=True, trail=[dict(gen=FACTOR, error=str(e)[:300])]), open(f"{name}.json", "w")); print("FAILED", name, e); return
    ok = r["bodies"] == 1 and r["watertight"] and max(r["function"]) < 0.5 and r["thin"] == 0 and r["worst"] <= 1.0 and r["connectivity"]
    prev = json.load(open(f"{name}.json")).get("trail", []) if os.path.exists(f"{name}.json") else []
    r["trail"] = prev + [dict(gen=FACTOR, mass=r["mass_g"], worst=r["worst"], ok=ok, bodies=r["bodies"])]
    if ok or not os.path.exists(f"{name}.json"):
        shutil.copy(f"{name}_g{FACTOR}.stl", f"{name}.stl"); json.dump(r, open(f"{name}.json", "w"), indent=1)
    else:
        j = json.load(open(f"{name}.json")); j["trail"] = r["trail"]; json.dump(j, open(f"{name}.json", "w"), indent=1)
    print("DONE", name, r["mass_g"], r["worst"], ok, flush=True)
if __name__ == "__main__":
    with mp.Pool(int(os.environ.get("WORKERS", "2"))) as p: p.map(one, sys.argv[1:], chunksize=1)
