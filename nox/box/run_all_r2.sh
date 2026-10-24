#!/bin/bash
# analyse + new-lane overlap + cuts for every finished run
W="$(cd "$(dirname "$0")/../.." && pwd)"
for t in "g2000 0" "g3000 0" "g3000_ring 1" "g4000 0"; do
  set -- $t
  [ -f "$W/runs/box_system/v2/topo_r2/$1/result.json" ] || continue
  for s in box_system_topo_v2r2_analyse.py box_system_topo_v2r2_newlanes.py box_system_topo_v2r2_cuts.py box_system_topo_v2r2_views.py; do
    "$W/nox/box/run_analysis_r2.sh" "$1" "$2" "$s"
  done
  echo "== $1"; tail -1 "$W/runs/box_system/v2/topo_r2/$1/box_system_topo_v2r2_analyse.txt" | cut -c1-600
  cat "$W/runs/box_system/v2/topo_r2/$1/box_system_topo_v2r2_newlanes.txt"
done
