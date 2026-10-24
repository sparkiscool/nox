#!/bin/bash
# commits and pushes runs/box_system/v2/topo_r2 every 15 min while a box_system_topo_v2r2 run is alive (logs, small files only)
W="$(cd "$(dirname "$0")/../.." && pwd)"; cd "$W"
n=${1:-1}
while pgrep -f box_system_topo_v2r2.py >/dev/null; do
  sleep 900
  git add runs/box_system/v2/topo_r2 -- ':!*.npz' >/dev/null 2>&1
  git commit -qm "ckpt $n" && git push -q -u origin HEAD && n=$((n+1))
done
