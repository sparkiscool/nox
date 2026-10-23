#!/usr/bin/env bash
# queue 3 (finish=drawn): cam v3 -> head v5 -> base v3. Status lines in runs/scan_stick_queue.log.
cd "$(dirname "$0")/../../.." || exit 1
Q=runs/scan_stick_queue.log; S=nox/overseer/scan_stick
run() { echo "$(date -u +%H:%M) START $1" >> $Q; python3 $S/$2 $3 $4 > runs/$5/nohup.txt 2>&1; echo "$(date -u +%H:%M) END $1 rc=$? $(python3 -c "import json;r=json.load(open('runs/$5/result.json'));print('mass',round(r.get('mass_g',0),1),'worst',r.get('worst'),'solid',r.get('solid_ok'))" 2>/dev/null || echo 'no result.json')" >> $Q; }
mkdir -p runs/scan_stick_cam/v3 runs/scan_stick/v5 runs/scan_stick_base/v3
echo "$(date -u +%H:%M) queue 3: finish=drawn (organic opened the drawn bodies: head v4 and cam v2 failed the gate); cam v3 restarted" >> $Q
run "cam v3"  cam_topo_v3.py  0.75 5  scan_stick_cam/v3
run "head v5" head_topo_v5.py 1.5  12 scan_stick/v5
run "base v3" base_topo_v3.py 2.0  35 scan_stick_base/v3
echo "$(date -u +%H:%M) QUEUE 3 DONE" >> $Q
