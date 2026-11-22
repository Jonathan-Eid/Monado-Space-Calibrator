#!/usr/bin/env bash
# Linux counterpart of scripts/Build.ps1: initialize pinned dependencies, build
# double-precision libsurvive and the headless app, run the offline tests and
# end-to-end inspection check, and assemble a package.
# Requires cmake + ninja on PATH (pip3 install cmake ninja works), g++,
# libusb-1.0, and /usr/bin/python3 for the inspection test.
set -euo pipefail

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_root="${BUILD_ROOT:-$repo/build}"
jobs="$(nproc)"

command -v cmake >/dev/null || { echo "cmake not found (pip3 install cmake)" >&2; exit 1; }
command -v ninja >/dev/null || { echo "ninja not found (pip3 install ninja)" >&2; exit 1; }
python="${LINUX_TEST_PYTHON:-/usr/bin/python3}"
command -v "$python" >/dev/null || { echo "$python not found" >&2; exit 1; }

git -C "$repo" submodule update --init --recursive

# Same importer patch as the Windows build: unsigned 32-bit base-station IDs,
# binary-safe reads and parse-error propagation are shared C code. In a normal
# checkout this checks out a git submodule and applies with git; this tree was
# seeded as a plain working-tree copy (submodule metadata flattened), so fall
# back to checking the patch markers in the C source: if the patched lines are
# already present, the tree is already patched.
survive_source="$repo/external/libsurvive"
import_patch="$repo/patches/libsurvive-windows-steamvr-import.patch"
patched_markers_present() {
    grep -q '(uint32_t)strtoul(json_stack_value(obj), 0, 10)' "$survive_source/src/survive_default_devices.c" \
        && grep -q 'fopen(filename, "rb")' "$survive_source/src/survive_default_devices.c"
}
if git -C "$survive_source" apply --check "$import_patch" 2>/dev/null; then
    git -C "$survive_source" apply "$import_patch"
elif git -C "$survive_source" apply --reverse --check "$import_patch" 2>/dev/null; then
    : # already applied
elif patched_markers_present; then
    echo "libsurvive importer patch already applied in the working tree (submodule git metadata absent); continuing."
else
    echo "Pinned libsurvive does not match the tested database-import patch." >&2
    exit 1
fi

survive_build="$build_root/libsurvive"
app_build="$build_root/app"

echo "== Configuring libsurvive (double precision) =="
cmake -S "$survive_source" -B "$survive_build" -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DUSE_SINGLE_PRECISION=OFF -DENABLE_TESTS=OFF
echo "== Building survive-cli =="
cmake --build "$survive_build" --target survive-cli --parallel "$jobs"

survive_library="$(find "$survive_build" -maxdepth 1 -name 'libsurvive.so.*' | sort | head -1)"
[ -n "$survive_library" ] || { echo "libsurvive shared library not found" >&2; exit 1; }
eigen_include="$survive_build/libs/cnmatrix/libeigen-src"
[ -d "$eigen_include/Eigen" ] || { echo "Eigen from the cnmatrix sub-build not found; reconfigure libsurvive with network access" >&2; exit 1; }

echo "== Configuring app =="
cmake -S "$repo" -B "$app_build" -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DSURVIVE_SOURCE="$survive_source" \
    -DSURVIVE_LIBRARY="$survive_library" \
    -DEIGEN_INCLUDE_DIR="$eigen_include"
echo "== Building app =="
cmake --build "$app_build" --parallel "$jobs"

echo "== Offline tests =="
ctest --test-dir "$app_build" --output-on-failure

echo "== Assembling package =="
package="$build_root/package"
mkdir -p "$package/data" "$package/plugins" "$package/assets/models" "$package/scripts" "$package/licenses"
cp "$app_build/vdxr-calibration" "$app_build/calibration-replay" "$package/"
cp "$survive_build"/libsurvive.so.* "$package/"
cp "$survive_build"/plugins/*.so "$package/plugins/" 2>/dev/null || rmdir "$package/plugins"
cp -r "$repo/assets/models/." "$package/assets/models/"
cp "$repo/README.md" "$repo/BUILD-LINUX.md" "$repo/LICENSE" "$repo/NOTICE" "$package/"
cp "$repo/scripts/inspect-app.py" "$repo/scripts/test-app-inspection.py" "$repo/scripts/Import-DeviceModels.py" "$package/scripts/"
cp "$repo/licenses/SpaceCalibrator-MIT.txt" "$package/licenses/SpaceCalibrator-MIT.txt"
cp "$repo/assets/models/LICENSE-WebXR.md" "$package/licenses/WebXR-Input-Profiles.txt"
cp "$repo/external/imgui/LICENSE.txt" "$package/licenses/ImGui.txt"
cp "$repo/external/libsurvive/LICENSE" "$package/licenses/libsurvive.txt"
cp "$repo/external/libsurvive/libs/cnmatrix/LICENSE" "$package/licenses/cnmatrix.txt"
cp "$repo/external/libsurvive/libs/cnkalman/LICENSE" "$package/licenses/cnkalman.txt"
cp "$repo/external/cJSON/LICENSE" "$package/licenses/cJSON.txt"
cp "$eigen_include/COPYING.MPL2" "$package/licenses/Eigen-MPL2.txt"

echo "== End-to-end preview inspection test =="
"$python" "$repo/scripts/test-app-inspection.py" --package-dir "$package"

echo "App package: $package"
echo "Run it with: cd '$package' && ./vdxr-calibration --preview"
