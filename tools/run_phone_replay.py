#!/usr/bin/env python3
"""Run original PTAM on a selected physical iPhone and export verified diagnostics.

No ARKit replay hooks or alternate pose estimator are used. A completed process
is reported separately from visual tracking coverage.
"""
import argparse
import atexit
import fcntl
import csv
import hashlib
import json
import math
import plistlib
from pathlib import Path
import subprocess
import time
import tempfile
import traceback
from storage import check_output_storage
from phone_config import bundle_identifier, engine_from_bundle, verify_expected_udid

BUNDLE = bundle_identifier("current")
_device_locks = []
_active_output = None


def acquire_device_lock(device):
    """Keep cooperating test processes from replacing each other's phone run."""
    key = hashlib.sha256(device.encode()).hexdigest()[:24]
    handle = (Path(tempfile.gettempdir()) / f"iptam-phone-{key}.lock").open("a+")
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        handle.close()
        raise RuntimeError("Another iPTAM test controls this device; run phone tests sequentially") from None
    # Keep the lock through failure cleanup registered with atexit.
    _device_locks.append(handle)


def cleanup_owned_trial(ctl, pid, summary, output, domain):
    """Try each evidence copy, and always attempt to stop our own process."""
    try:
        if summary and isinstance(summary.get("run_id"), str):
            partial_id = summary["run_id"]
            if "/" not in partial_id and ".." not in partial_id and not (output / "run").exists():
                (output / "partial-run").mkdir(exist_ok=True)
                for name in ("summary.json", "frames.jsonl"):
                    try:
                        ctl("copy", "from", "--source", f"Documents/runs/{partial_id}/{name}",
                            "--destination", str(output / "partial-run" / name), *domain, allow_failure=True)
                    except Exception as error:
                        print(f"Could not preserve {name}: {error}", flush=True)
    except Exception as error:
        print(f"Could not prepare partial evidence: {error}", flush=True)
    finally:
        try:
            ctl("process", "terminate", "--pid", str(pid), allow_failure=True)
        except Exception as error:
            print(f"Could not stop owned device process {pid}: {error}", flush=True)


def main():
    global _active_output
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dataset", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--device", required=True, help="Explicit devicectl device identifier")
    ap.add_argument("--expected-udid", help="Optional independent device identity check")
    ap.add_argument("--app", type=Path)
    ap.add_argument("--bundle", default=BUNDLE, help="Installed app bundle identifier")
    ap.add_argument("--mobile-profile", choices=["diagnostics", "serial", "coarse", "partial", "paper", "auto"])
    ap.add_argument("--source-paced", action="store_true")
    ap.add_argument("--progress-timeout", type=float, default=120)
    ap.add_argument("--timeout", type=float, default=1200)
    ap.add_argument("--skip-copy", action="store_true")
    ap.add_argument("--init-first", type=int)
    ap.add_argument("--init-second", type=int)
    args = ap.parse_args()
    if args.mobile_profile and engine_from_bundle(args.bundle) != "mobile":
        ap.error("A mobile profile requires the mobile app")
    if any(not math.isfinite(v) or v <= 0 for v in (args.timeout,args.progress_timeout)):
        ap.error("Timeouts must be positive and finite")
    acquire_device_lock(args.device)
    check_output_storage(args.dataset)
    check_output_storage(args.output)
    source = args.dataset.resolve()
    manifest_bytes = (source / "manifest.json").read_bytes()
    manifest = json.loads(manifest_bytes)
    original_manifest_digest = hashlib.sha256(manifest_bytes).hexdigest()
    overrides = {}
    for key in ("init_first", "init_second"):
        value = getattr(args, key)
        if value is not None:
            overrides[key] = value
            manifest[key] = value
    if not 0 <= manifest["init_first"] < manifest["init_second"] < len(manifest["frames"]):
        raise ValueError("Initialization pair is outside the requested input")
    if overrides:
        manifest_bytes = (json.dumps(manifest, indent=2) + "\n").encode()
    digest = hashlib.sha256(manifest_bytes).hexdigest()
    output = args.output.resolve()
    check_output_storage(output)
    output.mkdir(parents=True, exist_ok=False)
    _active_output = output
    (output / "requested-manifest.json").write_bytes(manifest_bytes)
    command_index = 0

    def ctl(*parts, allow_failure=False):
        nonlocal command_index
        command_index += 1
        evidence = output / f"device-{command_index:03d}.json"
        command = ["xcrun", "devicectl", "device", *parts[:2], "--device", args.device,
                   "--json-output", str(evidence), "--timeout", "120", *parts[2:]]
        (output / f"command-{command_index:03d}.json").write_text(json.dumps(command, indent=2) + "\n")
        result = subprocess.run(command, capture_output=True, text=True, timeout=135)
        (output / f"command-{command_index:03d}.log").write_text(result.stdout + result.stderr)
        if result.returncode:
            if allow_failure:
                return None
            raise RuntimeError(f"Device operation failed; evidence: command-{command_index:03d}.log")
        data = json.loads(evidence.read_text())
        if data.get("info", {}).get("outcome") != "success":
            raise RuntimeError(f"Device operation did not succeed: {evidence}")
        return data["result"]

    details = ctl("info", "details")
    hardware = details["hardwareProperties"]
    verify_expected_udid(hardware, args.expected_udid)
    if details["deviceProperties"]["developerModeStatus"] != "enabled":
        raise RuntimeError("Developer Mode is disabled on the selected device")
    lock = ctl("info", "lockState")
    if lock.get("passcodeRequired"):
        raise RuntimeError("The selected phone needs to be unlocked")
    app_identity = {"install_performed": False, "executable_sha256": None}
    if args.app:
        app_path = args.app.resolve()
        app_info = plistlib.loads((app_path / "Info.plist").read_bytes())
        with (app_path / app_info["CFBundleExecutable"]).open("rb") as stream:
            executable_digest = hashlib.file_digest(stream, "sha256").hexdigest()
        app_identity = {"install_performed": True, "path": str(app_path),
                        "executable_sha256": executable_digest,
                        "bundle_identifier": app_info["CFBundleIdentifier"],
                        "version": app_info.get("CFBundleShortVersionString")}
        if app_identity["bundle_identifier"] != args.bundle:
            raise RuntimeError("The requested app bundle is not the selected variant")
        ctl("install", "app", str(args.app.resolve()))
    domain = ["--domain-type", "appDataContainer", "--domain-identifier", args.bundle]
    previous = output / "previous-summary.json"
    ctl("copy", "from", "--source", "Documents/last-run.json", "--destination", str(previous),
        *domain, allow_failure=True)
    previous_id = json.loads(previous.read_text()).get("run_id") if previous.exists() else None
    device_root = f"Documents/replays/{source.name}-{digest[:12]}"
    if not args.skip_copy:
        print(f"Copying {source.name} to the selected phone…", flush=True)
        ctl("copy", "to", "--source", str(source), "--destination", device_root, *domain)
        if overrides:
            ctl("copy", "to", "--source", str(output / "requested-manifest.json"),
                "--destination", device_root + "/manifest.json", *domain)
    provenance = {"device": args.device, "hardware": hardware, "source": str(source),
                  "app": app_identity,
                  "manifest_sha256": digest, "device_dataset": device_root,
                  "original_manifest_sha256": original_manifest_digest, "initialization_override": overrides,
                  "harness_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  "fresh_process": True, "bundle": args.bundle, "engine": engine_from_bundle(args.bundle),
                  "source_paced": args.source_paced,
                  "mobile_profile": args.mobile_profile, "frame_count": len(manifest["frames"])}
    (output / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    launch_args = ["--replay", device_root.removeprefix("Documents/") + "/manifest.json"]
    if args.source_paced:
        launch_args.append("--source-paced")
    if args.mobile_profile:
        launch_args += ["--mobile-profile", args.mobile_profile]
    launch = ctl("process", "launch", "--terminate-existing", args.bundle, "--", *launch_args)
    pid = launch["process"]["processIdentifier"]
    summary = None

    def cleanup_failed_trial():
        if pid is None:
            return
        cleanup_owned_trial(ctl, pid, summary, output, domain)

    atexit.register(cleanup_failed_trial)
    deadline = time.monotonic() + args.timeout
    latest = output / "latest-summary.json"
    progress_at = time.monotonic()
    progress_frames = -1
    while time.monotonic() < deadline:
        time.sleep(3)
        copied = ctl("copy", "from", "--source", "Documents/last-run.json", "--destination", str(latest),
                     *domain, allow_failure=True)
        if time.monotonic() - progress_at > args.progress_timeout:
            raise RuntimeError("Replay made no logged progress before watchdog deadline; preserving partial evidence")
        if copied is None or not latest.exists():
            continue
        candidate = json.loads(latest.read_text())
        if candidate.get("run_id") == previous_id:
            continue
        summary = candidate
        current_frames = summary.get("frames", 0)
        if current_frames != progress_frames:
            progress_frames = current_frames
            progress_at = time.monotonic()
        print(f"{source.name}: {summary.get('frames', 0)} frames, "
              f"{summary.get('valid_visual_frames', 0)} visual, {summary.get('status')}", flush=True)
        if summary["status"] != "running":
            break
    if summary is None or summary.get("status") == "running":
        raise RuntimeError("Replay did not finish before the deadline; preserved all device evidence")
    run_id = summary["run_id"]
    if not isinstance(run_id, str) or "/" in run_id or ".." in run_id:
        raise RuntimeError("Invalid run ID from phone")
    (output / "run").mkdir()
    for name in ("summary.json", "frames.jsonl"):
        ctl("copy", "from", "--source", f"Documents/runs/{run_id}/{name}",
            "--destination", str(output / "run" / name), *domain)
    # Terminate only this test process after preserving its final results.
    ctl("process", "terminate", "--pid", str(pid), allow_failure=True)
    pid = None
    persisted = json.loads((output / "run/summary.json").read_text())
    if persisted["status"] != "completed":
        raise RuntimeError(f"Replay ended as {persisted['status']}; see run/summary.json")
    if persisted.get("source_paced", False) != args.source_paced:
        raise RuntimeError("Phone input pacing differs from requested schedule")
    if persisted.get("manifest_sha256") != digest:
        raise RuntimeError("On-device manifest hash differs from the requested input")
    expected_lens_model = manifest.get("lens_model", "pinhole")
    if persisted.get("lens_model") != expected_lens_model:
        raise RuntimeError(f"On-device lens model {persisted.get('lens_model')!r} differs from requested {expected_lens_model!r}")
    expected_pixels = json.loads((source / "provenance.json").read_text()).get("rectified_pixels_concatenated_sha256")
    if not expected_pixels or persisted.get("rectified_pixels_concatenated_sha256") != expected_pixels:
        raise RuntimeError("On-device grayscale pixel hash differs from the prepared input")
    rows = [json.loads(line) for line in (output / "run/frames.jsonl").read_text().splitlines()]
    if len(rows) != len(manifest["frames"]) or persisted["frames"] != len(rows):
        raise RuntimeError("Incomplete or inconsistent on-device frame coverage")
    for i, (frame, recorded) in enumerate(zip(manifest["frames"], rows)):
        if abs(frame["timestamp"] - recorded["timestamp"]) > 1e-9:
            raise RuntimeError(f"Timestamp mismatch at frame {i}")
        if not recorded["finite"]:
            raise RuntimeError(f"Nonfinite estimator pose at frame {i}")
        if any(not math.isfinite(x) for x in recorded["camera_from_world_3x4"]):
            raise RuntimeError(f"Nonfinite matrix at frame {i}")
        if recorded["valid_visual_pose"] and "pose_inliers" in recorded and recorded["pose_inliers"] < 10:
            raise RuntimeError(f"Accepted pose has fewer than ten geometric inliers at frame {i}")
    headers = ["timestamp", "tx", "ty", "tz", "r00", "r01", "r02", "r10", "r11", "r12", "r20", "r21", "r22",
               "quality", "found", "attempted", "points", "keyframes", "elapsed_ms", "solver_failures", "solver_retained_stops", "pose_inliers"]
    with (output / "trajectory.csv").open("w") as stream:
        writer = csv.writer(stream)
        writer.writerow(headers)
        for row in rows:
            # Invert the logged camera-from-world transform without dependencies.
            m = row["camera_from_world_3x4"]
            rotation = [[m[r * 4 + c] for c in range(3)] for r in range(3)]
            translation = [-sum(rotation[j][i] * m[j * 4 + 3] for j in range(3)) for i in range(3)]
            inverse = [rotation[c][r] for r in range(3) for c in range(3)]
            writer.writerow([row["timestamp"], *translation, *inverse, row["quality"], row["found"], row["attempted"],
                             row["map_points"], row["keyframes"], row["processing_ms"], row.get("solver_failures", 0), row.get("solver_retained_stops", 0), row.get("pose_inliers", "")])
    visual = sum(row["valid_visual_pose"] for row in rows)
    result = {"pipeline_validation": "passed", "frames": len(rows), "visual_frames": visual,
              "lens_model": persisted["lens_model"],
              "visual_coverage_all_frames": visual / len(rows), "run_id": run_id,
              "mean_processing_ms": persisted["mean_processing_ms"],
              "solver_numerical_rejections": persisted.get("solver_numerical_rejections", 0),
              "solver_retained_stops": persisted.get("solver_retained_stops", 0),
              "tracking_stability": "assess coverage and trajectory error; completion alone is insufficient"}
    (output / "validation.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        if _active_output is not None:
            (_active_output / "failure.json").write_text(json.dumps({
                "error_type": type(error).__name__, "error": str(error),
                "traceback": traceback.format_exc()}, indent=2) + "\n")
        raise
