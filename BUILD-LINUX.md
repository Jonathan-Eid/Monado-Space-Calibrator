# Building and running on Linux

The full Windows app (D3D11 window, OpenXR session through the VDXR runtime,
rendered frame exports) stays on Windows — see `README.md` and
`scripts/Build.ps1`. The Linux port covers the offline pipeline: the same
model logic, the inspection API and the tracker IPC, running as a
**headless synthetic-preview app** with no window, GPU renderer or OpenXR
runtime. Everything below builds and tests without any VR or tracking
hardware.

## What works on Linux

- `vdxr-calibration --preview`: synthetic preview (simulated head, two
  controllers, a tracker fixture) with the full guarded action set.
- The inspection API on an AF_UNIX socket at
  `$XDG_RUNTIME_DIR/VDXRCalibration.<pid>.sock` (mode 0600), speaking the
  exact Windows JSON protocol: one request per connection (32 KiB limit),
  one UTF-8 JSON response, then the client acknowledges with `{"ack":true}`.
  One client at a time; `state` is answered inline, other operations are
  executed on the app thread via the request queue.
- POSIX tracker IPC (`include/vdxr_tracker_ipc.h`): named POSIX shared
  memory (`/dev/shm/VDXR.Libsurvive.v1`) with a process-shared robust mutex
  instead of the Windows named mapping + mutex; timestamps are
  `CLOCK_MONOTONIC` nanoseconds (`frequency = 1e9`) instead of QPC ticks.
  The 584-byte state ABI is unchanged. The single-provider guard is an
  exclusive `flock` on `$XDG_RUNTIME_DIR/VDXR.Libsurvive.Writer.v1`.
- Offline tests: `calibration-math`, `device-identity`, `survive-import`
  (real libsurvive database importer with the pinned patch) and `tracker-ipc`
  (new POSIX IPC roundtrip test).

## Prerequisites

- CMake ≥ 3.21 and Ninja on PATH. There is no system-wide cmake here;
  `pip3 install cmake ninja` provides known-good versions (tested with
  CMake 4.4.5 / Ninja 1.13). The pinned libsurvive and cnmatrix require
  CMake ≥ 3.12, which CMake 4.x accepts.
- g++ (C++17), libusb-1.0, and network access on first configure: the
  libsurvive `cnmatrix` sub-build clones Eigen 3.4 into
  `build/libsurvive/libs/cnmatrix/libeigen-src`.
- `/usr/bin/python3` for the inspection test scripts.
- No GPU, window system or Virtual Desktop/SteamVR is needed.

## Build and test

```bash
git submodule update --init --recursive
./scripts/build-linux.sh            # BUILD_ROOT=<dir> overrides build/
```

The script configures and builds double-precision libsurvive
(`-DUSE_SINGLE_PRECISION=OFF -DENABLE_TESTS=OFF`, target `survive-cli`),
applies the pinned importer patch, configures and builds the app, runs the
CTest suite, assembles `build/package/` and runs the end-to-end preview
inspection test. To do it manually:

```bash
cmake -S external/libsurvive -B build/libsurvive -G Ninja \
    -DCMAKE_BUILD_TYPE=Release -DUSE_SINGLE_PRECISION=OFF -DENABLE_TESTS=OFF
cmake --build build/libsurvive --target survive-cli
cmake -S . -B build/app -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DSURVIVE_SOURCE="$PWD/external/libsurvive" \
    -DSURVIVE_LIBRARY="$PWD/build/libsurvive/libsurvive.so.0" \
    -DEIGEN_INCLUDE_DIR="$PWD/build/libsurvive/libs/cnmatrix/libeigen-src"
cmake --build build/app
ctest --test-dir build/app --output-on-failure
```

## Run in preview mode

```bash
cd build/package
./vdxr-calibration --preview --seconds 45     # headless, exits after 45 s
python3 scripts/inspect-app.py --pid <pid> --request '{"op":"state"}'
python3 scripts/inspect-app.py --pid <pid> --request '{"op":"exit"}'
```

The app runs relative to its own directory (like the Windows app) and writes
`data/` next to itself. `scripts/test-app-inspection.py --package-dir
build/package` replays the Windows inspection checks for Linux: state,
capture-speed persistence, selections, invalid inputs, preview gates, Quest
2/3 model identity and mesh verification, clean export rejection and API
shutdown.

Differences from the Windows app, by design:

- `export_frame` is cleanly rejected ("no renderer"); rendering and PNG
  export belong to the Windows D3D11 app.
- `state.device_models.asset_loaded` means the `.vdm` mesh parsed and passed
  validation; textures are not decoded headlessly (`asset_error` says so).
- Starting without `--preview` fails cleanly and stays inspectable: hardware
  mode needs the Quest-side OpenXR runtime.

## What does not work on Linux yet (the runtime piece)

Hardware calibration needs, on one side, a Vive Tracker 3.0 with SteamVR
base stations (this side is libsurvive and works on Linux), and on the other
side Quest controller poses in the headset's STAGE space, which the app gets
by loading the **VDXR OpenXR runtime** (`virtualdesktop-openxr.dll`) through
`LoadLibraryW` + `xrNegotiateLoaderRuntimeInterface`. That runtime exists
only for Windows and only alongside the Virtual Desktop client; there is no
Linux Virtual Desktop, and the runtime repo (`vdxr-runtime`, a separate
Windows-only project) is out of scope for this port. No Windows DLL is
loaded and wine is not used.

Quest connectivity on Linux realistically means ALVR or WiVRN, both of
which ship their own OpenXR runtimes. The natural follow-up is: keep this
app's model logic, swap the `XrApp` session layer (`src/calibration_xr.h`)
to an OpenGL/Vulkan graphics binding against the ALVR/WiVRN runtime (or the
Monado stack), and keep the tracker IPC as the pose hand-off. The libusb and
POSIX IPC groundwork here is what that follow-up builds on. Hardware
requirements stay the same: Vive Tracker 3.0, SteamVR 2.0 base stations,
a PC GPU for the runtime's encoder.

`VALIDATION.md` still governs: paired hardware calibration has not been
validated, on any platform. Nothing in the Linux port changes that.
