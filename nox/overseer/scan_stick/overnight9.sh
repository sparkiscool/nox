#!/usr/bin/env bash
# queue 9: waits for queue 8 (base v5), then head v8 (camera insert holes 6 deep)
cd "$(dirname "$0")/../../.." || exit 1
Q=runs/scan_stick_queue.log; S=nox/overseer/scan_stick
until grep -q "QUEUE 8 DONE" $Q 2>/dev/null; do sleep 60; done
mkdir -p runs/scan_stick/v8
echo "$(date -u +%H:%M) START head v8" >> $Q
python3 $S/head_topo_v8.py 1.5 12 > runs/scan_stick/v8/nohup.txt 2>&1
echo "$(date -u +%H:%M) END head v8 rc=$? $(python3 -c "import json;r=json.load(open('runs/scan_stick/v8/result.json'));print('mass',round(r.get('mass_g',0),1),'worst',r.get('worst'),'solid',r.get('solid_ok'))" 2>/dev/null || echo 'no result.json')" >> $Q
echo "$(date -u +%H:%M) QUEUE 9 DONE" >> $Q
