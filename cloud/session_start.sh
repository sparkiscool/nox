#!/usr/bin/env bash
# SessionStart hook (see .claude/settings.json). Runs on every session start and resume.
# Cheap when the environment snapshot already has the packages; installs them if not.
set -uo pipefail
cd "$(git rev-parse --show-toplevel 2>/dev/null || pwd)"

# noxopt must be imported before pypardiso (it points pypardiso at pip's MKL), so check from nox/
if ! (cd nox && python3 -c "import noxopt, trimesh, manifold3d, skimage, skfmm, pypardiso, pymoto, shapely, fast_simplification") >/dev/null 2>&1; then
  echo "nox: installing engine packages (first run in this box, ~2.5 min; the Nox environment's setup script avoids this)"
  bash cloud/setup.sh >/tmp/nox_setup.log 2>&1 || echo "nox: setup failed, see /tmp/nox_setup.log"
fi

echo "nox: $(git branch --show-current) @ $(git rev-parse --short HEAD); $(nproc) cores, $(awk '/MemTotal/ {printf "%.0f GB", $2/1048576}' /proc/meminfo) RAM"
exit 0
