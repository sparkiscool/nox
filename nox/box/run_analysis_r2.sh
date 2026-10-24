#!/bin/bash
# usage: run_analysis_r2.sh <tag> <ring 0|1> [script=box_system_topo_v2r2_analyse.py]  -> runs/box_system/v2/topo_r2/<tag>/<script>.txt
W="$(cd "$(dirname "$0")/../.." && pwd)"
export PYTHONPATH="$W/nox:$W/nox/box:$W/nox/box/stand"
export TOPO_OUT="$W/runs/box_system/v2/topo_r2/$1" TOPO_RING="$2"
S="${3:-box_system_topo_v2r2_analyse.py}"
cd "$W/nox"
python3 "box/$S" > "$TOPO_OUT/${S%.py}.txt" 2>&1
