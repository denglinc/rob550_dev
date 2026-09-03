#!/bin/bash
# Install the RealSense L515 camera stack (system-wide) plus the system packages
# the control station needs at runtime.

set -euo pipefail

ENV_NAME=env550lab

sudo apt-get update

sudo apt install -y python3-pip python3-pyqt5 fonts-inter

# Install RealSense L515 Camera SDK
sudo apt-get -y install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev
sudo apt-get -y install git wget cmake build-essential
sudo apt-get -y install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at

# --- which interpreter do we build the binding for? --------------------------
# pyrealsense2 is a compiled extension: so build it against $ENV_NAME's python (3.10, pinned in environment_py310.yml),
# not /usr/bin/python3
# Hence the env must exist first.
if ! command -v conda >/dev/null 2>&1; then
  echo "ERROR: conda not found on PATH." >&2
  echo "  Run ./install_scripts/install_anaconda.sh, then 'source ~/.bashrc'." >&2
  exit 1
fi
if ! conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
  echo "ERROR: conda env '$ENV_NAME' not found." >&2
  echo "  Run ./install_scripts/install_conda_env.sh first -- this script builds" >&2
  echo "  the RealSense Python binding against that env's interpreter." >&2
  exit 1
fi
ENV_PY="$(conda run -n "$ENV_NAME" python -c 'import sys; print(sys.executable)')"
echo "=> Building the Python binding for: $ENV_PY"
conda run -n "$ENV_NAME" python -VV

# The RealSense apt repo only stores librealsense2 >= 2.55.x now, 2.54.2 is not available via apt and must be built from source.
# 2.54.2 is the last version supports L515 camera
cd ~
if [ ! -d librealsense ]; then
  git clone https://github.com/realsenseai/librealsense.git
fi
cd librealsense
git fetch --tags
git checkout v2.54.2
sudo ./scripts/setup_udev_rules.sh

# A configured build/ caches PYTHON_EXECUTABLE and everything FindPythonLibsNew
# derived from it (PYTHON_INCLUDE_DIRS, PYTHON_LIBRARY). Re-running cmake with a
# different -DPYTHON_EXECUTABLE does not reliably refresh those, so start clean.
if [ -f build/CMakeCache.txt ] && \
   ! grep -qxF "PYTHON_EXECUTABLE:FILEPATH=$ENV_PY" build/CMakeCache.txt; then
  echo "=> build/ was configured for a different Python, wiping it"
  rm -rf build
fi

mkdir -p build && cd build
# -include cstdint: GCC 13 (Ubuntu 24.04's default) stopped pulling <cstdint> in
# transitively through <string> and friends. librealsense 2.54.2 predates that and
# has ~170 files using uint64_t/uint32_t without including it, starting with
# third-party/rsutils/include/rsutils/version.h. Force-including the header into
# every C++ translation unit fixes the whole class at once instead of patching
# vendored sources one by one. Harmless on GCC 12 / Ubuntu 22.04.
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_PYTHON_BINDINGS=ON \
  -DPYTHON_EXECUTABLE="$ENV_PY" \
  -DCMAKE_CXX_FLAGS="-include cstdint"
make -j$(nproc)
sudo make install
sudo ldconfig

# The binding lands in /usr/local/OFF -- 'OFF' is a CMake variable that leaks into
# the install path, not a typo. PYTHONPATH is what makes it importable, including
# from inside a conda env.
grep -qxF 'export PYTHONPATH=/usr/local/OFF:$PYTHONPATH' ~/.bashrc || \
echo 'export PYTHONPATH=/usr/local/OFF:$PYTHONPATH' >> ~/.bashrc
export PYTHONPATH=/usr/local/OFF:${PYTHONPATH:-}   # :- because set -u and PYTHONPATH is usually unset

# Auto-activate env550lab in every new terminal. To undo: delete this line from ~/.bashrc.
grep -qxF "conda activate $ENV_NAME" ~/.bashrc || \
echo "conda activate $ENV_NAME" >> ~/.bashrc

# Prove the binding is loadable from the env that will actually import it. A camera
# does not have to be attached; this only checks the ABI and the shared libs.
echo
echo "=> Verifying 'import pyrealsense2' from '$ENV_NAME'"
conda run -n "$ENV_NAME" --no-capture-output python -c \
  "import pyrealsense2 as rs; print('   loaded', rs.__file__); print('   cameras attached:', len(rs.context().devices))"

echo
echo "=> Done. Now run:  source ~/.bashrc"
echo "   New terminals will start in '$ENV_NAME' from now on."
