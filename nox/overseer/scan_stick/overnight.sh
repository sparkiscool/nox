#!/usr/bin/env bash
# Runs the scan stick topology set back to back (Spark 23 Oct: several heavy runs in one session, merge reviewed PRs).
# Each run commits + pushes its own checkpoints (topo.run --git). Status lines go to runs/scan_stick_queue.log.
cd "$(dirname "$0")/../../.." || exit 1
Q=runs/scan_stick_queue.log; S=nox/overseer/scan_stick
run() { echo "$(date -u +%H:%M) START $1" >> $Q; python3 $S/$2 $3 $4 > runs/$5/nohup.txt 2>&1; echo "$(date -u +%H:%M) END $1 rc=$? $(python3 -c "import json;r=json.load(open('runs/$5/result.json'));print('mass',round(r.get('mass_g',0),1),'worst',r.get('worst'),'solid',r.get('solid_ok'))" 2>/dev/null || echo 'no result.json')" >> $Q; }
mkdir -p runs/scan_stick_cam/v2 runs/scan_stick/v4 runs/scan_stick_base/v2
run "cam v2"  cam_topo_v2.py  0.75 2.5 scan_stick_cam/v2
run "head v4" head_topo_v4.py 1.5  6   scan_stick/v4
run "base v2" base_topo_v2.py 2.0  35  scan_stick_base/v2
echo "$(date -u +%H:%M) QUEUE DONE" >> $Q
