#!/usr/bin/env bash
# queue 6: waits for queue 5 (head v6), then base v4 (wider support band, no coarse stage)
cd "$(dirname "$0")/../../.." || exit 1
Q=runs/scan_stick_queue.log; S=nox/overseer/scan_stick
until grep -q "QUEUE 5 DONE" $Q 2>/dev/null; do sleep 60; done
mkdir -p runs/scan_stick_base/v4
echo "$(date -u +%H:%M) START base v4" >> $Q
python3 $S/base_topo_v4.py 2.0 35 > runs/scan_stick_base/v4/nohup.txt 2>&1
echo "$(date -u +%H:%M) END base v4 rc=$? $(python3 -c "import json;r=json.load(open('runs/scan_stick_base/v4/result.json'));print('mass',round(r.get('mass_g',0),1),'worst',r.get('worst'),'solid',r.get('solid_ok'))" 2>/dev/null || echo 'no result.json')" >> $Q
echo "$(date -u +%H:%M) QUEUE 6 DONE" >> $Q
