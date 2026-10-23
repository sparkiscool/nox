#!/usr/bin/env bash
# SessionStart hook (see .claude/settings.json). Runs on every session start and resume.
# Cheap when the environment snapshot already has the packages; installs them if not.
set -uo pipefail
cd "$(git rev-parse --show-toplevel 2>/dev/null || pwd)"

git lfs install --skip-repo >/dev/null 2>&1 || true
git lfs pull >/dev/null 2>&1 || true

# noxopt must be imported before pypardiso (it points pypardiso at pip's MKL), so check from nox/
if ! (cd nox && python3 -c "import noxopt, trimesh, manifold3d, skimage, skfmm, pypardiso, pymoto") >/dev/null 2>&1; then
  echo "nox: installing engine packages (first run in this box, ~1 min)"
  bash cloud/setup.sh >/tmp/nox_setup.log 2>&1 || echo "nox: setup failed, see /tmp/nox_setup.log"
fi

# amazon-parts MCP server (.mcp.json): needs the MCP SDK and httpx
if ! python3 -c "import mcp.server.mcpserver, httpx" >/dev/null 2>&1; then
  echo "nox: installing amazon-parts MCP deps"
  pip install -q "mcp>=2,<3" "httpx>=0.27" >/tmp/nox_mcp_setup.log 2>&1 || echo "nox: MCP deps failed, see /tmp/nox_mcp_setup.log"
fi

echo "nox: $(git branch --show-current) @ $(git rev-parse --short HEAD); $(nproc) cores, $(awk '/MemTotal/ {printf "%.0f GB", $2/1048576}' /proc/meminfo) RAM"
exit 0
