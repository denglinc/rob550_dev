#!/bin/bash
# Install Anaconda (Linux x86_64).
# Follows https://www.anaconda.com/docs/getting-started/anaconda/install/linux-install
set -euo pipefail

INSTALLER=Anaconda3-2026.07-1-Linux-x86_64.sh
PREFIX="$HOME/anaconda3"

if [ -d "$PREFIX" ]; then
  echo "=> $PREFIX already exists, skipping install."
  echo "   Delete it first if you want a clean reinstall."
  exit 0
fi

cd ~
if [ ! -f "$INSTALLER" ]; then
  curl -O "https://repo.anaconda.com/archive/$INSTALLER"
fi

# The docs ask you to verify the download. Compare this against the hash listed
# next to the installer at https://repo.anaconda.com/archive/
echo
echo "=> Verify this checksum against https://repo.anaconda.com/archive/"
sha256sum "$HOME/$INSTALLER"
echo

# -b is batch mode: it installs unattended and ACCEPTS THE LICENSE on your behalf.
# The license is at https://www.anaconda.com/legal -- read it before running this.
echo "=> Installing to $PREFIX in batch mode (this accepts the Anaconda license:"
echo "   https://www.anaconda.com/legal )"
bash "$HOME/$INSTALLER" -b -p "$PREFIX"

# Batch mode skips the "initialize conda?" prompt, so do it explicitly.
"$PREFIX/bin/conda" init bash

echo
echo "=> Done. Now run:  source ~/.bashrc"
echo "   Then:           ./install_scripts/install_conda_env.sh"
