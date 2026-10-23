#!/usr/bin/env bash
# Exports the scan stick's OpenSCAD parts into out/: the thread test ring and the thread body every base run cuts.
# ./render.sh [acme_clear]   e.g. ./render.sh 0.40   (the printed parts themselves are nox/topo runs: *_topo_*.py)
set -e; cd "$(dirname "$0")"; mkdir -p out; CLR=${1:-0.40}
openscad -o out/scan_stick_thread_test.stl -D 'part="thread_test"' -D "acme_clear=$CLR" scan_stick.scad &
openscad -o out/thread_cut.stl -D "acme_clear=$CLR" thread_cut.scad &
wait
echo "rendered into $(pwd)/out"
