import resource, runpy, sys, time, json
t0 = time.time(); sys.argv = ["head_topo_run.py", "1.75", "20"]
try:
    runpy.run_path("../../../nox/overseer/scan_stick/head_topo_run.py", run_name="__main__")
finally:
    peak = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1e6
    json.dump(dict(peak_gb=round(peak, 2), wall_s=round(time.time() - t0)), open("peak.json", "w"))
    print(f"PEAK {peak:.2f} GB wall {time.time() - t0:.0f} s", flush=True)
