#!/bin/bash
# cage v3 (C3) node runs, one after another (one generative run per box): corner, V tee, side node, crown
cd "$(dirname "$0")"
for spec in "corner 0.75 2400" "vtee 0.5 1800" "snode 0.6 2400" "crown 0.75 2700"; do
  set -- $spec; echo "=== $1 start $(date +%H:%M)"; python3 node_gen.py $1 $2 $3 > /home/claude/cage/nodes/$1.log 2>&1; echo "=== $1 exit $? $(date +%H:%M)"; grep RESULT /home/claude/cage/nodes/$1.log
done
