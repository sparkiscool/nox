#!/usr/bin/env bash
# queue 4: head v5 (24 square flange bolts) -> base v3 -> cam v4 (Ø12 posts). Status in runs/scan_stick_queue.log.
cd "$(dirname "$0")/../../.." || exit 1
Q=runs/scan_stick_queue.log; S=nox/overseer/scan_stick
run() { echo "$(date -u +%H:%M) START $1" >> $Q; python3 $S/$2 $3 $4 > runs/$5/nohup.txt 2>&1; echo "$(date -u +%H:%M) END $1 rc=$? $(python3 -c "import json;r=json.load(open('runs/$5/result.json'));print('mass',round(r.get('mass_g',0),1),'worst',r.get('worst'),'solid',r.get('solid_ok'))" 2>/dev/null || echo 'no result.json')" >> $Q; }
mkdir -p runs/scan_stick/v5 runs/scan_stick_base/v3 runs/scan_stick_cam/v4
echo "$(date -u +%H:%M) queue 4: cam v3 FAILED (Ø8 posts); flange bolts moved to a 24 square (counterbore wall rule); head v5 restarted" >> $Q
run "head v5" head_topo_v5.py 1.5  12 scan_stick/v5
run "base v3" base_topo_v3.py 2.0  35 scan_stick_base/v3
run "cam v4"  cam_topo_v4.py  0.75 5  scan_stick_cam/v4
echo "$(date -u +%H:%M) QUEUE 4 DONE" >> $Q
