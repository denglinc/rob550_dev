#!/bin/bash
# Create the env550lab conda env and install the mujoco-sim bridge into it.
# This is the scripted form of the Quick Start in src/mujoco_sim/README.md.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="$REPO_ROOT/src/mujoco_sim/environment_py310.yml"
ENV_NAME=env550lab

if ! command -v conda >/dev/null 2>&1; then
  echo "ERROR: conda not found on PATH." >&2
  echo "  Run ./install_scripts/install_anaconda.sh first, then 'source ~/.bashrc'." >&2
  exit 1
fi

# conda >= 25.x refuses to touch repo.anaconda.com until channel Terms of Service are
# accepted, and environment_py310.yml needs the 'defaults' channel for package: scipy 1.15.3

# THIS ACCEPTS ANACONDA'S TERMS OF SERVICE FOR YOU -- https://www.anaconda.com/legal
if conda tos --help >/dev/null 2>&1; then
  echo "=> Accepting the Anaconda channel Terms of Service (https://www.anaconda.com/legal)"
  for ch in main r; do
    conda tos accept --override-channels --channel "https://repo.anaconda.com/pkgs/$ch" >/dev/null
  done
fi

if conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
  echo "=> Env '$ENV_NAME' exists, updating it from $ENV_FILE"
  conda env update -n "$ENV_NAME" -f "$ENV_FILE" --prune
else
  echo "=> Creating env '$ENV_NAME' from $ENV_FILE"
  conda env create -f "$ENV_FILE"
fi

# Editable install so edits under src/mujoco_sim/ take effect without reinstalling.
# --no-deps: every dependency is already pinned in the env file.
echo "=> Installing mujoco-sim into '$ENV_NAME' (editable)"
conda run -n "$ENV_NAME" --no-capture-output \
  pip install -e "$REPO_ROOT/src/mujoco_sim" --no-deps --config-settings editable_mode=compat

echo
echo "=> Done. Next:  ./install_scripts/install_camera.sh"
