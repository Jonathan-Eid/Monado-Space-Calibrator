# Building and running on Linux

The Windows app (D3D11 window, OpenXR session through the VDXR runtime,
rendered frame exports) stays on Windows — see `README.md` and
`scripts/Build.ps1`. The Linux port covers the offline pipeline plus the
Monado renderer path: the same model logic, the inspection API and the
tracker IPC, running as an app with an offscreen OpenGL renderer (EGL,
GL 3.3 core) and a Monado/OpenXR session layer. Starting without
`--preview` needs a Monado runtime (see below); `--preview` runs the
synthetic preview with real offscreen rendering and PNG frame export.

## What works on Linux

- `vdxr-calibration --preview`: synthetic preview (simulated head, two
  controllers, a tracker fixture) with the full guarded action set, rendered
  offscreen through EGL/OpenGL. `export_frame` (`mirror`, `left_eye`,
  `model_catalog`) writes real PNGs to `data/inspection/` (stb_image_write,
  public domain). Passing export releases the ticket and polls
  `state.render.export_completed` from the inspection API.
- `vdxr-calibration` (hardware path): `src/calibration_xr.h` loads the
  OpenXR loader (`libopenxr.so.1`) via dlopen and calls `xrCreateInstance`
  with `XR_KHR_opengl_enable` — the standard Monado OpenGL path on Linux
  (Wayland graphics binding + `xrGetOpenGLGraphicsRequirementsKHR` +
  `XrGraphicsBindingOpenGLWaylandKHR`). Without an installed Monado
  runtime the app fails cleanly citing the missing runtime (no session is
  claimed). See "Monado runtime & Quest connectivity" below.
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
- Offline tests: `calibration-math`, `device-identity`, `renderer-math`
  (new portable scene-math coverage for the Linux OpenGL renderer),
  `survive-import` (real libsurvive database importer with the pinned patch)
  and `tracker-ipc` (new POSIX IPC roundtrip test).

## Prerequisites

- CMake ≥ 3.21 and Ninja on PATH. There is no system-wide cmake here;
  `pip3 install cmake ninja` provides known-good versions (tested with
  CMake 4.4.5 / Ninja 1.13). The pinned libsurvive and cnmatrix require
  CMake ≥ 3.12, which CMake 4.x accepts.
- g++ (C++17), libusb-1.0, Mesa OpenGL/EGL (`libGL`, `libEGL` + headers,
  `libwayland-client`), and network access on first configure: the
  libsurvive `cnmatrix` sub-build clones Eigen 3.4 into
  `build/libsurvive/libs/cnmatrix/libeigen-src`.
- Single-header image libraries are vendored (public domain, `external/
  stb_image.h`, `external/stb_image_write.h`, `external/stb-LICENSE.txt`):
  stb_image decodes `.vdm` PNG textures, stb_image_write encodes frame
  exports.
- `/usr/bin/python3` for the inspection test scripts.
- For hardware mode only: Monado + an OpenXR runtime reachable via
  `XR_RUNTIME_JSON`, plus Quest connectivity over the local network
  (ALVR/WiVRN). No GPU, window system or Virtual Desktop/SteamVR is needed
  to build, test, or run `--preview`.

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

Clean-rebuild verification (what CI runs):

```bash
rm -rf build && ./scripts/build-linux.sh   # exits 0
ctest --test-dir build/app --output-on-failure   # 5/5 pass
```

## Run in preview mode

```bash
cd build/package
./vdxr-calibration --preview --seconds 45     # headless, exits after 45 s
python3 scripts/inspect-app.py --pid <pid> --request '{"op":"state"}'
python3 scripts/inspect-app.py --pid <pid> --request '{"op":"export_frame","view":"mirror"}'
python3 scripts/inspect-app.py --pid <pid> --request '{"op":"exit"}'
```

The app runs relative to its own directory (like the Windows app) and writes
`data/` next to itself. `scripts/test-app-inspection.py --package-dir
build/package` replays the Windows inspection checks for Linux: state,
capture-speed persistence, selections, invalid inputs, preview gates, Quest
2/3 model identity and mesh verification, real offscreen OpenGL frame
exports (mirror + model catalog), and clean API shutdown. Rendering is
offscreen (EGL pbuffer/FBO, no window): `export_frame` produces 1280x840
mirror and model-catalog PNGs and a 1440x1440 synthetic left-eye PNG.

Differences from the Windows app, by design:

- The desktop mirror is rendered offscreen at 1280x840 (no Win32 HWND, no
  ImGui UI overlay; the geometry is the same).
- `state.device_models.asset_loaded` means the `.vdm` mesh has been parsed
  **and** its PNG texture decoded and uploaded to GL (stb_image); the
  headless `asset_error` text is now "OpenGL: textured mesh uploaded" on
  success.
- Starting without `--preview` fails cleanly and stays inspectable:
  hardware mode needs a Monado OpenXR runtime (below).

## Monado runtime & Quest connectivity

Hardware calibration needs Quest controller poses in the headset's STAGE
space. On Linux those come from an OpenXR session through the **Monado**
runtime — the VDXR runtime (`virtualdesktop-openxr.dll` + `LoadLibraryW` +
`xrNegotiateLoaderRuntimeInterface`) is Windows-only and tied to the
Virtual Desktop client, which does not exist on Linux.

What a working Monado setup looks like:

- **Monado on Linux** (e.g. `pacman -S monado` on Arch). Monado is the OpenXR
  runtime providing `libopenxr.so.1` (the loader library) and runtime
  implementation. The app finds the loader with `dlopen("libopenxr.so.1")`
  and calls `xrCreateInstance` via `xrGetInstanceProcAddr` — no link-time
  `libopenxr` dependency, so missing/old loader produces a clean
  "Monado OpenXR runtime not found" startup failure (and remains inspectable
  via `state`/`exit`, with `data/inspection/error.json`). To point at a
  specific runtime, set `XR_RUNTIME_JSON` to the runtime manifest path —
  the OpenXR loader's standard mechanism — and the app picks it up without
  reconfiguring (the loader also auto-discovers
  `$XDG_RUNTIME_DIR/openxr/active_runtime.json` and
  `~/.config/openxr/active_runtime.json` when `XR_RUNTIME_JSON` is unset).
- **Quest connectivity** is via **ALVR or WiVRN** over the local network
  (both ship their own OpenXR runtimes sitting behind the Monado/session
  stack); Virtual Desktop does not exist on Linux. Connecting the Quest
  through ALVR/WiVRN first, then starting the app without `--preview`,
  forms the STAGE-space pose source.
- **Graphics binding**: `XR_KHR_opengl_enable` (OpenGL). The session is
  created with `XrGraphicsBindingOpenGLWaylandKHR` over the Wayland display,
  swapchains are `XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT` RGBA8. Monado
  supports OpenGL swapchains on Linux.

What remains unproven in this environment (see also `VALIDATION.md`):
**runtime attachment and real pose calibration have not been validated —
no Monado runtime, headset, tracker, or base stations were attached here.**
The session-creation path compiles and the runtime-detection failure is
verified end-to-end, but nothing connects to real hardware in the tests.

Hardware list (same as Windows):
Vive Tracker 3.0, SteamVR 2.0 base stations (2 or more), a Quest headset
(Quest 3/3S family with Touch controllers, also Quest 2/Pro identities),
a PC GPU (for the ALVR/WiVRN encoder and the OpenGL renderer).

`VALIDATION.md` still governs: paired hardware calibration has not been
validated, on any platform. Nothing in the Linux port changes that.
