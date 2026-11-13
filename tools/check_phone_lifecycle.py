#!/usr/bin/env python3
"""Physical iPTAM malformed-input and OS background-cancellation checks.

Uses an already installed iPTAM and already copied plane fixture. The background
check briefly opens Settings; it does not change any setting.
"""
import argparse
import atexit
import hashlib
import json
from pathlib import Path
import subprocess
import time

from run_phone_replay import BUNDLE, acquire_device_lock
from phone_config import engine_from_bundle, verify_expected_udid
from storage import check_output_storage


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--fixture", type=Path, required=True)
    ap.add_argument("--device", required=True, help="Explicit devicectl device identifier")
    ap.add_argument("--expected-udid", help="Optional independent device identity check")
    ap.add_argument("--bundle", default=BUNDLE, help="Installed app bundle identifier")
    ap.add_argument("--source-paced", action="store_true")
    ap.add_argument("--replay-path", help="Already verified Documents-relative manifest cache")
    ap.add_argument("--background-after", type=float, default=.4)
    args = ap.parse_args()
    acquire_device_lock(args.device)
    check_output_storage(args.output)
    args.output.mkdir(parents=True, exist_ok=False)
    counter = 0
    owned_pid = None
    domain = ["--domain-type", "appDataContainer", "--domain-identifier", args.bundle]

    def ctl(*parts):
        nonlocal counter
        counter += 1
        evidence = args.output / f"device-{counter:03d}.json"
        command = ["xcrun", "devicectl", "device", *parts[:2], "--device", args.device,
                   "--timeout", "30", "--json-output", str(evidence), *parts[2:]]
        result = subprocess.run(command, capture_output=True, text=True, timeout=45)
        (args.output / f"command-{counter:03d}.json").write_text(json.dumps(command, indent=2) + "\n")
        (args.output / f"command-{counter:03d}.log").write_text(result.stdout + result.stderr)
        if result.returncode:
            raise RuntimeError(f"Device command failed; see command-{counter:03d}.log")
        payload = json.loads(evidence.read_text())
        if payload["info"]["outcome"] != "success":
            raise RuntimeError(f"Device command failed: {evidence}")
        return payload["result"]

    def cleanup():
        if owned_pid is not None:
            try:
                ctl("process", "terminate", "--pid", str(owned_pid))
            except Exception as error:
                print(f"Could not stop owned lifecycle test process {owned_pid}: {error}", flush=True)
    atexit.register(cleanup)

    device = ctl("info", "details")
    verify_expected_udid(device["hardwareProperties"], args.expected_udid)

    def summary():
        target = args.output / "latest.json"
        ctl("copy", "from", "--source", "Documents/last-run.json", "--destination", str(target), *domain)
        return json.loads(target.read_text())

    results = []
    minimum = {"width": 640, "height": 480, "fx": 400, "fy": 400, "cx": 319.5, "cy": 239.5,
               "init_first": 0, "init_second": 1, "frames": [{"timestamp": 0, "path": "missing.pgm"},
                                                           {"timestamp": 1, "path": "missing.pgm"}]}
    wrong_entry = dict(minimum, frames=[None, minimum["frames"][1]])
    wrong_clock = dict(minimum, frames=[minimum["frames"][0], minimum["frames"][0]])
    wrong_lens = dict(minimum, lens_model="unsupported")
    corrupt = args.output / "corrupt.pgm"
    corrupt.write_bytes(b"P5\n640 480\n255\ntruncated")
    ctl("copy", "to", "--source", str(corrupt.resolve()), "--destination", "Documents/tests/corrupt.pgm", *domain)
    corrupt_input = dict(minimum, frames=[dict(frame, path="corrupt.pgm") for frame in minimum["frames"]])
    cases = [("array_root", []), ("null_frame", wrong_entry), ("duplicate_timestamp", wrong_clock),
             ("unsupported_lens", wrong_lens), ("missing_image", minimum), ("truncated_image", corrupt_input)]
    if engine_from_bundle(args.bundle) == "mobile":
        timing = dict(enabled=True, verified=True, coordinate_space="prepared_image", source="synthetic lifecycle fixture",
                      readout_seconds=.02, axis="rows", direction=1, timestamp_fraction=.5)
        cases += [("rolling_unknown", dict(minimum, rolling_shutter=dict(timing, verified=False))),
                  ("rolling_wrong_coordinates", dict(minimum, rolling_shutter=dict(timing, coordinate_space="raw_sensor"))),
                  ("rolling_bad_number", dict(minimum, rolling_shutter=dict(timing, readout_seconds=".02junk")))]
    for name, value in cases:
        local = args.output / f"{name}.json"
        local.write_text(json.dumps(value) + "\n")
        before = summary()["run_id"]
        ctl("copy", "to", "--source", str(local.resolve()), "--destination", f"Documents/tests/{name}.json", *domain)
        launched = ctl("process", "launch", "--terminate-existing", args.bundle, "--", "--replay", f"tests/{name}.json")
        owned_pid = launched["process"]["processIdentifier"]
        time.sleep(.4)
        current = summary()
        if current["run_id"] == before or current["status"] != "failed" or current["frames"] != 0 or not current.get("error"):
            raise RuntimeError(f"Malformed input was not rejected cleanly: {name}: {current}")
        if current.get("manifest_sha256") != hashlib.sha256(local.read_bytes()).hexdigest():
            raise RuntimeError("Rejected input identity mismatch")
        if name.startswith("rolling_") and "rolling" not in current["error"].lower():
            raise RuntimeError("Invalid rolling timing did not fail before reading images")
        # The process must remain alive and be terminable after reporting the error.
        ctl("process", "terminate", "--pid", str(launched["process"]["processIdentifier"]))
        owned_pid = None
        results.append({"case": name, "result": "passed", "summary": current})

    raw = (args.fixture / "manifest.json").read_bytes()
    fixture = json.loads(raw)
    relative = args.replay_path or f"replays/{args.fixture.name}-{hashlib.sha256(raw).hexdigest()[:12]}/manifest.json"
    before = summary()["run_id"]
    pacing = ["--source-paced"] if args.source_paced else []
    launched = ctl("process", "launch", "--terminate-existing", args.bundle, "--", "--replay", relative, *pacing)
    owned_pid = launched["process"]["processIdentifier"]
    time.sleep(args.background_after)
    ctl("process", "launch", "com.apple.Preferences")
    deadline = time.monotonic() + 10
    current = summary()
    while (current["run_id"] == before or current["status"] == "running") and time.monotonic() < deadline:
        time.sleep(.5)
        current = summary()
    if current["run_id"] == before or current["status"] != "backgrounded" or not 0 < current["frames"] < len(fixture["frames"]):
        raise RuntimeError(f"OS background cancellation failed: {current}")
    if args.source_paced and not current.get("source_paced"):
        raise RuntimeError("Background trial did not use requested source pacing")
    time.sleep(1)
    after = summary()
    if after["run_id"] != current["run_id"] or after["status"] != "backgrounded" or after["frames"] != current["frames"]:
        raise RuntimeError("Delayed replay work revived a backgrounded session")
    ctl("process", "terminate", "--pid", str(launched["process"]["processIdentifier"]))
    owned_pid = None
    results.append({"case": "real_OS_background", "result": "passed", "summary": current})
    (args.output / "results.json").write_text(json.dumps(results, indent=2) + "\n")
    print(json.dumps([{k: v for k, v in r.items() if k != "summary"} for r in results], indent=2))


if __name__ == "__main__":
    main()
