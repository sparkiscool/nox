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
  2>&1 | grep -v "running pip as the 'root'" || true

echo "== git-lfs"
if ! command -v git-lfs >/dev/null 2>&1; then
  (apt-get update -qq && apt-get install -y -qq git-lfs) >/dev/null 2>&1 || true
fi
git lfs install --skip-repo >/dev/null 2>&1 || true

echo "== smoke import"
cd "$(dirname "$0")/.." 2>/dev/null || true
if [ -f nox/noxopt.py ]; then
  (cd nox && python3 -c "import noxopt, trimesh, manifold3d, skimage, skfmm, pypardiso; print('nox engine packages OK')")
else
  python3 -c "import trimesh, manifold3d, skimage, skfmm, pypardiso; print('packages OK (repo not cloned yet)')"
fi
echo "== done"
