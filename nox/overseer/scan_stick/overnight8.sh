#!/usr/bin/env bash
# queue 8: waits for queue 7 (head v7), then base v5 (thicker drawn plates, web, 60 cm3)
cd "$(dirname "$0")/../../.." || exit 1
Q=runs/scan_stick_queue.log; S=nox/overseer/scan_stick
until grep -q "QUEUE 7 DONE" $Q 2>/dev/null; do sleep 60; done
mkdir -p runs/scan_stick_base/v5
echo "$(date -u +%H:%M) START base v5" >> $Q
python3 $S/base_topo_v5.py 2.0 60 > runs/scan_stick_base/v5/nohup.txt 2>&1
echo "$(date -u +%H:%M) END base v5 rc=$? $(python3 -c "import json;r=json.load(open('runs/scan_stick_base/v5/result.json'));print('mass',round(r.get('mass_g',0),1),'worst',r.get('worst'),'solid',r.get('solid_ok'))" 2>/dev/null || echo 'no result.json')" >> $Q
echo "$(date -u +%H:%M) QUEUE 8 DONE" >> $Q
