#!/usr/bin/env bash
# Nox cloud environment setup script.
# Paste this file's contents into the "Setup script" of the Nox environment at claude.ai/code
# (Environments). It runs once as root before Claude starts; the filesystem is snapshotted and
# reused for ~7 days, so the pip install only pays once.
# Also safe to run by hand in any fresh box:  bash cloud/setup.sh
set -euo pipefail

echo "== Nox engine packages"
pip install --break-system-packages -q \
  pymoto pypardiso mkl trimesh rtree manifold3d scikit-image scikit-fmm \
  matplotlib cadquery sparse_dot_mkl psutil numpy scipy \
  shapely fast_simplification playwright \
  2>&1 | grep -v "running pip as the 'root'" || true

echo "== git-lfs"
if ! command -v git-lfs >/dev/null 2>&1; then
  (apt-get update -qq && apt-get install -y -qq git-lfs) >/dev/null 2>&1 || true
fi
git lfs install --skip-repo >/dev/null 2>&1 || true

echo "== three.js for headless motion renders (nox/motion.py; the npm registry is reachable where the CDNs are not)"
mkdir -p "$HOME/.cache/nox-motion"
[ -d "$HOME/.cache/nox-motion/node_modules/three" ] || npm install --silent --no-audit --no-fund --prefix "$HOME/.cache/nox-motion" three@0.128.0 || true

echo "== smoke import"
cd "$(dirname "$0")/.." 2>/dev/null || true
if [ -f nox/noxopt.py ]; then
  (cd nox && python3 -c "import noxopt, trimesh, manifold3d, skimage, skfmm, pypardiso, shapely, fast_simplification; print('nox engine packages OK')")
else
  python3 -c "import trimesh, manifold3d, skimage, skfmm, pypardiso; print('packages OK (repo not cloned yet)')"
fi
echo "== done"
