#!/usr/bin/env bash
# Renders every scan-stick STL + previews into out/.  ./render.sh [rod_d] [acme_clear]   e.g. ./render.sh 25.4 0.40
set -e; cd "$(dirname "$0")"; mkdir -p out; ROD=${1:-25.4}; CLR=${2:-0.40}
openscad -o out/scan_stick_head_up.stl    -D 'part="lidar_head"' -D 'lidar_facing="up"'   -D "acme_clear=$CLR" scan_stick.scad &
openscad -o out/scan_stick_head_down.stl  -D 'part="lidar_head"' -D 'lidar_facing="down"' -D "acme_clear=$CLR" scan_stick.scad &
openscad -o out/scan_stick_thread_test.stl -D 'part="thread_test"' -D "acme_clear=$CLR" scan_stick.scad &
openscad -o out/scan_stick_cam.stl        -D 'part="camera_bracket"' scan_stick.scad &
openscad -o out/scan_stick_cradle.stl     -D 'part="pi_bat_cradle"' -D "rod_d=$ROD" scan_stick.scad &
openscad -o out/thread_cut.stl thread_cut.scad &
xvfb-run -a openscad -o out/assembly_up.png --imgsize=1200,1800 --viewall --autocenter --camera=0,0,0,65,0,215,900 --projection=o --colorscheme=Tomorrow -D 'part="assembly"' -D "rod_d=$ROD" scan_stick.scad >/dev/null 2>&1 &
wait
echo "rendered into $(pwd)/out"
