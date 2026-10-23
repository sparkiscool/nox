#!/usr/bin/env bash
# second queue: waits for overnight.sh, then reruns the camera bracket as v3
cd "$(dirname "$0")/../../.." || exit 1
Q=runs/scan_stick_queue.log; S=nox/overseer/scan_stick
until grep -q "QUEUE DONE" $Q 2>/dev/null; do sleep 60; done
mkdir -p runs/scan_stick_cam/v3
echo "$(date -u +%H:%M) START cam v3" >> $Q
python3 $S/cam_topo_v3.py 0.75 4 > runs/scan_stick_cam/v3/nohup.txt 2>&1
echo "$(date -u +%H:%M) END cam v3 rc=$? $(python3 -c "import json;r=json.load(open('runs/scan_stick_cam/v3/result.json'));print('mass',round(r.get('mass_g',0),1),'worst',r.get('worst'),'solid',r.get('solid_ok'))" 2>/dev/null || echo 'no result.json')" >> $Q
echo "$(date -u +%H:%M) QUEUE2 DONE" >> $Q
