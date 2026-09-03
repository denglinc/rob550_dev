# Install

Target platform: **Ubuntu 24.04**. Also works on 22.04 - no dependency on the system Python version.

Three scripts, run in this order. Each one is safe to re-run.

```bash
./install_anaconda.sh
source ~/.bashrc
```
```bash
./install_conda_env.sh
```
```bash
./install_camera.sh
source ~/.bashrc
```

That `source` is the last manual step: `install_camera.sh` appends both
`PYTHONPATH=/usr/local/OFF` and `conda activate env550lab` to `~/.bashrc`, so every terminal
you open after this starts in `env550lab` already.

**The order matters:** `install_camera.sh` builds the RealSense Python binding against the `env550lab` interpreter, 
so the env has to exist first. The script hard-errors with instructions if it does not.

| Script | What it does |
| --- | --- |
| `install_anaconda.sh` | Downloads and batch-installs Anaconda to `~/anaconda3`, then runs `conda init bash`. Skips itself if `~/anaconda3` already exists. **Batch mode accepts the [Anaconda license](https://www.anaconda.com/legal) for you**. |
| `install_conda_env.sh` | Accepts the Anaconda **channel** Terms of Service (see below), creates the `env550lab` env from `../src/mujoco_sim/environment_py310.yml`, and installs `mujoco-sim` into it in editable mode. Gives you the `mujoco-sim` and `550-lcm-spy` commands. |
| `install_camera.sh` | apt system packages, then builds **librealsense 2.54.2 from source** (the last version with L515 support) against the `env550lab` Python, and adds `PYTHONPATH=/usr/local/OFF` plus `conda activate env550lab` to `~/.bashrc`. This is the slow one - expect 5~20 minutes. Ends by verifying `import pyrealsense2` from the env. |

Everything importable lives in the `env550lab` env - **run the control station with `env550lab` active.**
After `install_camera.sh` that happens by itself in new terminals. To turn it off, delete the
`conda activate env550lab` line from `~/.bashrc`.

## Why does `install_conda_env.sh` accept a Terms of Service?

conda 25.x and newer refuse to fetch from `repo.anaconda.com` until you accept the
channel ToS. On a fresh machine the first solve dies with:

```
CondaToSNonInteractiveError: Terms of Service have not been accepted for the following channels:
    - https://repo.anaconda.com/pkgs/main
    - https://repo.anaconda.com/pkgs/r
```

`environment_py310.yml` needs the `defaults` channel for exactly one package: **`scipy
1.15.3`**, which conda-forge only carries up to 1.15.2. Everything else in the file already
resolves from conda-forge.

If your institution would rather not use Anaconda's channels at all, the fix is small: drop
`defaults` from the `channels:` list and re-pin `scipy = 1.15.2`.

## Why is the binding built against the conda Python and not `/usr/bin/python3`?

It used to be `-DPYTHON_EXECUTABLE=/usr/bin/python3`, so the build would not care whether
Anaconda was installed. That only ever worked by coincidence: the binding is named
`pyrealsense2.cpython-3XX-*.so` after the interpreter it was built against, a conda env can
only import it if its Python has the **same major.minor version**, and Ubuntu 22.04's system
Python happens to be 3.10 - the version `environment_py310.yml` pins.

On Ubuntu 24.04 `/usr/bin/python3` is 3.12, so that build produces a `cpython-312` `.so`
that `env550lab` can never load. Nothing errors during the build; `import pyrealsense2`
just silently finds nothing.

Building against the env's own interpreter removes the coupling to the distro entirely.
The cost is the ordering constraint above.

Re-pinning the env to 3.12 instead would only move the problem: it revalidates every pin in
`environment_py310.yml` and re-breaks on the next distro bump. It is also a dead end if you
ever drop the source build - PyPI's `pyrealsense2 2.54.2.5684`, the last release with L515
support, ships no cp312 wheel (cp310 is the newest).

## Why `-include cstdint` in the cmake line?

GCC 13 - Ubuntu 24.04's default - stopped pulling `<cstdint>` in transitively through
`<string>` and friends. librealsense 2.54.2 predates that change and has ~170 files using
`uint64_t` / `uint32_t` without including it, so the build dies almost immediately:

```
third-party/rsutils/include/rsutils/version.h:23:13: error: 'uint64_t' does not name a type
```

`-DCMAKE_CXX_FLAGS="-include cstdint"` force-includes the header into every C++ translation
unit, which fixes the whole class of errors at once instead of patching vendored sources one
file at a time. It is a no-op on GCC 12 / Ubuntu 22.04.

## Why does apt still install a PyQt5 nobody imports?

`install_camera.sh` runs `sudo apt install -y python3-pip python3-pyqt5 fonts-inter`.

The apt PyQt5 lands in `/usr/lib/python3/dist-packages`, which is **not** on a conda
env's `sys.path`, so the GUI never sees it - the GUI imports the `pyqt5` pinned in
`environment_py310.yml`. The apt package stays for a side effect: it depends on
`libqt5gui5`, which in turn depends on the `libxcb-*` and `libxkbcommon-x11-0` runtime
libraries. The pip PyQt5 wheel bundles Qt itself but **not** those, so without this line
a clean Ubuntu install fails at startup with:

```
qt.qpa.plugin: Could not load the Qt platform plugin "xcb"
```

`fonts-inter` backs the `"Inter"` font-family in `src/ui/style.py`. Missing it is
cosmetic only - Qt falls back to Segoe UI / Ubuntu.

## Why `opencv-python-headless` and not `opencv-python`?

`opencv-python` ships its own Qt5 libraries inside the package. When PyQt5 is also loaded
in the same process, Qt's plugin loader finds the wrong `xcb` platform plugin and crashes.
`opencv-python-headless` is identical for all image-processing purposes - it just has no
bundled Qt. `cv2.imshow` is unavailable; all display goes through PyQt5.

## Script not executable?

```bash
chmod +x install_anaconda.sh install_camera.sh install_conda_env.sh
```
