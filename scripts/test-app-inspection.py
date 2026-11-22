#!/usr/bin/env python3
"""End-to-end inspection test for the Linux headless calibration app.

Counterpart of scripts/Test-AppInspection.ps1 for the Linux preview package:
starts vdxr-calibration --preview, drives the AF_UNIX inspection API and checks
native state, selections, invalid inputs, preview gates, device model identity,
real offscreen OpenGL frame export (mirror + model catalog) and clean API
shutdown. The Linux preview renders through EGL/OpenGL offscreen (no window).

Usage: test-app-inspection.py --package-dir PATH [--python /usr/bin/python3]
"""
import argparse
import importlib.util
import json
import os
import subprocess
import sys
import time

# inspect-app.py keeps the hyphenated name used by the PowerShell client; load it by path.
_client_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "inspect-app.py")
_spec = importlib.util.spec_from_file_location("inspect_app", _client_path)
inspect_app = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(inspect_app)
inspect = inspect_app.inspect
socket_path = inspect_app.socket_path


def invoke(pid: int, request: dict) -> dict:
    return json.loads(inspect(socket_path(pid), json.dumps(request), timeout=7.0))


def wait_for_preview(pid: int, attempts: int = 60) -> dict:
    state = None
    for _ in range(attempts):
        try:
            state = invoke(pid, {"op": "state"})
            if state.get("mode") == "synthetic_preview":
                return state
        except (OSError, ValueError):
            pass
        time.sleep(0.1)
    raise AssertionError(f"Preview did not initialize; last state: {state}")


def wait_for_export(pid: int, ticket: int, attempts: int = 60) -> bool:
    for _ in range(attempts):
        try:
            state = invoke(pid, {"op": "state"})
            if state.get("render", {}).get("export_completed", 0) >= ticket:
                return True
        except (OSError, ValueError):
            pass
        time.sleep(0.1)
    return False


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--package-dir", required=True)
    parser.add_argument("--seconds", default="45")
    args = parser.parse_args()
    package = os.path.abspath(args.package_dir)
    app = os.path.join(package, "vdxr-calibration")
    require(os.path.exists(app), "Packaged app is missing.")

    # Persist and restore the private settings the test toggles.
    settings = {
        "data/device-models.json": None,
        "data/calibration-settings.json": None,
    }
    for path in settings:
        full = os.path.join(package, path)
        settings[path] = open(full, "rb").read() if os.path.exists(full) else None
    try:
        def launch():
            process = subprocess.Popen([app, "--preview", "--seconds", args.seconds], cwd=package)
            state = wait_for_preview(process.pid)
            return process, state

        process, state = launch()
        try:
            require(invoke(process.pid, {"op": "set_fast_mode", "value": 0})["ok"], "Slow capture selection failed.")
            slow = invoke(process.pid, {"op": "state"})
            require(not slow["fast_mode"] and slow["capture_target_samples"] == 250, "Slow sample target did not change.")
            require(invoke(process.pid, {"op": "exit"})["ok"], "Exit was rejected.")
            process.wait(timeout=5)
            process = None

            # Capture speed persists across restart.
            process, state = launch()
            slow = invoke(process.pid, {"op": "state"})
            require(slow["mode"] == "synthetic_preview" and not slow["fast_mode"]
                    and slow["capture_target_samples"] == 250, "Capture speed did not persist across restart.")
            require(invoke(process.pid, {"op": "set_fast_mode", "value": 1})["ok"], "Fast capture selection failed.")
            fast = invoke(process.pid, {"op": "state"})
            require(fast["fast_mode"] and fast["capture_target_samples"] == 100, "Fast sample target did not change.")
            require(not invoke(process.pid, {"op": "set_fast_mode", "value": 2})["ok"], "Invalid fast mode value accepted.")
            require(invoke(process.pid, {"op": "select_reference", "value": 1})["ok"], "Selection command failed.")
            require(invoke(process.pid, {"op": "state"})["reference"] == "right", "Selection was not reflected in state.")
            require(not invoke(process.pid, {"op": "select_reference", "value": 7})["ok"], "Invalid selection was accepted.")
            require(not invoke(process.pid, {"op": "save_profile"})["ok"], "Preview allowed saving a profile.")
            require(not invoke(process.pid, {"op": "start_capture"})["ok"], "Preview allowed hardware capture.")
            require(not invoke(process.pid, {"op": "lock_map"})["ok"], "Preview allowed a Lighthouse map lock.")
            require(invoke(process.pid, {"op": "select_left_model", "value": "oculus-touch-v3"})["ok"], "Quest 2 model selection failed.")
            require(invoke(process.pid, {"op": "select_right_model", "value": "meta-quest-touch-plus"})["ok"], "Quest 3 model selection failed.")
            require(not invoke(process.pid, {"op": "select_left_model", "value": "../../invalid"})["ok"], "Invalid model was accepted.")
            models = {entry["device"]: entry for entry in invoke(process.pid, {"op": "state"})["device_models"]}
            left, right = models["left_controller"], models["right_controller"]
            require(left["model_id"] == "oculus-touch-v3" and left["asset_loaded"]
                    and left["identity_source"] == "manual override", "Quest 2 identity/mesh missing.")
            require(right["model_id"] == "meta-quest-touch-plus" and right["asset_loaded"], "Quest 3 identity/mesh missing.")
            require(invoke(process.pid, {"op": "show_device_models", "value": 1})["ok"]
                    and invoke(process.pid, {"op": "state"})["device_models_open"], "Device model view did not open.")
            require(invoke(process.pid, {"op": "show_device_models", "value": 0})["ok"]
                    and not invoke(process.pid, {"op": "state"})["device_models_open"], "Device model view did not close.")
            # Real offscreen OpenGL export: mirror renders the synthetic scene
            # to PNG; poll state.render.export_completed for the ticket.
            require(invoke(process.pid, {"op": "export_frame", "view": "mirror"})["ok"], "Mirror export was rejected.")
            require(wait_for_export(process.pid, 1), "Mirror export did not complete.")
            require(os.path.getsize(os.path.join(package, "data/inspection/mirror.png")) > 1000, "Mirror PNG is implausibly small.")
            require(invoke(process.pid, {"op": "export_frame", "view": "model_catalog"})["ok"], "Model catalog export was rejected.")
            require(wait_for_export(process.pid, 2), "Model catalog export did not complete.")
            require(os.path.getsize(os.path.join(package, "data/inspection/model-catalog.png")) > 1000, "Model catalog PNG is implausibly small.")
            require(invoke(process.pid, {"op": "export_frame", "view": "bogus"})["ok"] is False, "Unknown export view was accepted.")
            # Synthetic devices are reported, with tracker and controllers tracked.
            devices = {entry["name"]: entry for entry in invoke(process.pid, {"op": "state"})["devices"]}
            require(devices["head"]["tracked"] and devices["left_controller"]["tracked"]
                    and devices["right_controller"]["tracked"] and "PREVIEW TRACKER" in devices,
                    "Synthetic preview devices missing from state.")
            require(invoke(process.pid, {"op": "exit"})["ok"], "App did not exit through its API.")
            process.wait(timeout=5)
            process = None
        finally:
            if process is not None and process.poll() is None:
                process.kill()
                process.wait()
        print("Passed: native state, selections, Quest 2/3 mesh verification, invalid inputs, preview gates, offscreen OpenGL exports (mirror + model catalog) and API shutdown.")
        return 0
    finally:
        for path, saved in settings.items():
            full = os.path.join(package, path)
            if saved is not None:
                with open(full, "wb") as handle:
                    handle.write(saved)
            elif os.path.exists(full):
                os.remove(full)


if __name__ == "__main__":
    raise SystemExit(main())
