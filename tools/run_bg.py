#!/usr/bin/env python3
"""Run one heavy job to the end, committing its checkpoints, so whoever started it is woken once (docs/sessions.md Usage).

python3 tools/run_bg.py runs/<part>/<version> [--every 600] [--cap-h 6] -- python3 <script> [args]

Start it with the Bash tool's run_in_background and do not poll: the task notification when it exits is the only wake.
- the command's stdout + stderr go to <dir>/log.txt
- every --every s, if <dir> changed: git add <dir>, commit "ckpt <n>", push (files over 49 MB are left out: no LFS)
- at --cap-h hours (hard cap, docs/generative.md The rule 4) the job is stopped and the last checkpoint kept
- at exit: <dir>/run_bg.txt (exit code, wall time, peak memory GB of the job), a last commit + push, one line on stdout
Exit code = the job's (124 at the cap).
"""
import argparse, datetime, os, pathlib, resource, signal, subprocess, sys, time

ap = argparse.ArgumentParser()
ap.add_argument("dir")
ap.add_argument("--every", type=int, default=600, help="seconds between checkpoint commits")
ap.add_argument("--cap-h", type=float, default=6.0, help="hard cap in hours")
argv = sys.argv[1:]
cut = argv.index("--") if "--" in argv else len(argv)
a, cmd = ap.parse_args(argv[:cut]), argv[cut + 1:]
if not cmd:
    sys.exit("run_bg: no command after --")
d = pathlib.Path(a.dir)
d.mkdir(parents=True, exist_ok=True)
git = lambda *g: subprocess.run(["git", *g], capture_output=True, text=True)
n = 0


def checkpoint(msg):
    global n
    if not git("status", "--porcelain", "--", str(d)).stdout.strip():
        return
    big = [p for p in d.rglob("*") if p.is_file() and p.stat().st_size > 49 * 2**20]
    git("add", "--", str(d))
    for p in big:
        git("reset", "-q", "--", str(p))
    n += 1
    if git("commit", "-qm", msg.format(n=n)).returncode == 0:
        for wait in (0, 2, 4, 8, 16):
            time.sleep(wait)
            if git("push", "-q", "-u", "origin", "HEAD").returncode == 0:
                break


t0 = time.time()
with open(d / "log.txt", "ab") as log:
    job = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    code, last = None, time.time()
    while code is None:
        time.sleep(5)
        code = job.poll()
        if code is None and time.time() - t0 > a.cap_h * 3600:
            os.killpg(job.pid, signal.SIGTERM)
            time.sleep(30)
            if job.poll() is None:
                os.killpg(job.pid, signal.SIGKILL)
            job.wait()
            code = 124
        elif code is None and time.time() - last > a.every:
            checkpoint("ckpt {n}")
            last = time.time()
wall = (time.time() - t0) / 3600
peak = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss / 2**20   # KB -> GB (Linux)
line = (f"run_bg: exit {code}{' (hard cap)' if code == 124 else ''}, wall {wall:.2f} h, peak {peak:.1f} GB (M), "
        f"{datetime.datetime.now():%d %b %H:%M}: {' '.join(cmd)}")
with open(d / "run_bg.txt", "a") as f:
    f.write(line + "\n")
checkpoint("ckpt {n} (end, exit " + str(code) + ")")
print(line)
sys.exit(code)
