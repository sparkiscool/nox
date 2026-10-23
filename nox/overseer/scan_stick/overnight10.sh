#!/usr/bin/env bash
# queue 10: waits for the base v8 rebuild to finish, then runs the base v9 rebuild (cable openings)
cd "$(dirname "$0")/../../.." || exit 1
Q=runs/scan_stick_queue.log; S=nox/overseer/scan_stick
until grep -q "RESULT" runs/scan_stick_base/v8/log.txt 2>/dev/null || ! pgrep -f "^python3 nox/overseer/scan_stick/base_topo_v8" >/dev/null; do sleep 20; done
mkdir -p runs/scan_stick_base/v9
echo "$(date -u +%H:%M) START base v9 (rebuild, cable openings)" >> $Q
python3 $S/base_topo_v9.py 2.0 25 > runs/scan_stick_base/v9/nohup.txt 2>&1
echo "$(date -u +%H:%M) END base v9 rc=$? $(python3 -c "import json;r=json.load(open('runs/scan_stick_base/v9/result.json'));print('mass',round(r.get('mass_g',0),1),'worst',r.get('worst'),'solid',r.get('solid_ok'))" 2>/dev/null || echo 'no result.json')" >> $Q
echo "$(date -u +%H:%M) QUEUE 10 DONE" >> $Q
