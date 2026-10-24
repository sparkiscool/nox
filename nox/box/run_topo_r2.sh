#!/bin/bash
# usage: run_topo_r2.sh <script.py> args...   (sets PYTHONPATH for nox, nox/box, nox/box/stand; cwd nox)
W="$(cd "$(dirname "$0")/../.." && pwd)"
export PYTHONPATH="$W/nox:$W/nox/box:$W/nox/box/stand"
cd "$W/nox"
exec python3 "$@"
