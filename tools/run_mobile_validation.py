#!/usr/bin/env python3
"""Replay frozen common inputs under explicit mobile ablations; no pair search."""
import argparse
import json
import math
import hashlib
from pathlib import Path
import platform
from types import SimpleNamespace
import cv2

from evaluate_trajectory import evaluate
from run_host_validation import run_trial, sha, trial_failed, write_json
from storage import check_output_storage

def apply_auto_denominator(result, input_frames):
    """Automatic bootstrap has no fixed second-frame eligibility cutoff."""
    result["fixed_pair_summary_unused"] = {k:result[k] for k in ("visual_valid_frames", "eligible_frames", "visual_valid_fraction")}
    result["visual_valid_frames"] = result["evaluation"]["valid_output_frames"]
    result["eligible_frames"] = input_frames
    result["visual_valid_fraction"] = result["evaluation"]["valid_output_coverage"]
    result["summary_denominator_policy"] = "all frozen input frames; second view selected automatically"

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--binary", type=Path, required=True)
    p.add_argument("--plan", type=Path, required=True)
    p.add_argument("--expected-plan-sha256", help="Reject any input-plan change from this SHA256")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--profiles", nargs="+", choices=["diagnostics", "repair", "serial", "coarse", "partial", "paper", "auto"], default=["paper"])
    p.add_argument("--cases", nargs="+")
    p.add_argument("--repeats", type=int, default=2)
    p.add_argument("--timeout", type=float, default=1200)
    p.add_argument("--max-frames", type=int, help="Explicit diagnostic prefix; never label as a full input")
    p.add_argument("--source-paced", action="store_true")
    p.add_argument("--deterministic-mapping", action="store_true", help="Disable wall-time mapping budgets and idle scheduling for algorithm repeat checks")
    args = p.parse_args()
    plan_sha = sha(args.plan)
    if args.expected_plan_sha256 and plan_sha != args.expected_plan_sha256:
        raise ValueError("Input plan hash changed")
    if args.repeats < 1 or not math.isfinite(args.timeout) or args.timeout <= 0 or (args.max_frames is not None and args.max_frames <= 0):
        p.error("repeats and timeout must be positive")
    check_output_storage(args.output)
    args.binary = args.binary.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    plan = json.loads(args.plan.read_text())
    binary_hash = sha(args.binary)
    write_json(args.output / "provenance.json", dict(binary=str(args.binary), binary_sha256=binary_hash,
               plan=str(args.plan), plan_sha256=plan_sha, harness_sha256=sha(Path(__file__)),
               profiles=args.profiles, repeats=args.repeats, max_frames=args.max_frames,
               source_paced=args.source_paced, deterministic_mapping=args.deterministic_mapping, platform=platform.platform(),
               reference_available_to_estimator=False, pair_selection="frozen common plan; no search"))
    cases = [c for c in plan["cases"] if not args.cases or c["name"] in args.cases]
    if args.cases and set(args.cases) != {c["name"] for c in cases}:
        raise ValueError("Unknown case")
    results = {}
    for case in cases:
        dataset = Path(case["dataset"])
        for filename, key in [("manifest.json", "manifest_sha256"), ("frames.csv", "frames_csv_sha256")]:
            if sha(dataset / filename) != case[key]:
                raise ValueError(f"Frozen input changed: {dataset / filename}")
        # Verify actual supplied pixels before starting any estimator. Reference
        # files are not opened during this input-only preflight.
        manifest = json.loads((dataset / "manifest.json").read_text())
        pixels = hashlib.sha256()
        for frame in manifest["frames"]:
            image = cv2.imread(str(dataset / frame["path"]), cv2.IMREAD_GRAYSCALE)
            if image is None or image.shape != (manifest["height"], manifest["width"]):
                raise ValueError("Missing or wrong-sized frozen image")
            pixels.update(image.tobytes())
        if pixels.hexdigest() != case["pixel_sha256"]:
            raise ValueError("Frozen source pixels changed")
        write_json(args.output / (case["name"] + "-input-verification.json"),
                   dict(manifest_sha256=sha(dataset / "manifest.json"), pixel_sha256=pixels.hexdigest(),
                        frames=len(manifest["frames"]), references_read=False))
        for profile in args.profiles:
            key = case["name"] + "/" + profile
            results[key] = []
            for rep in range(args.repeats):
                folder = args.output / case["name"] / profile / f"repeat_{rep+1:02d}"
                extra = ["--mobile-profile", "diagnostics" if profile == "repair" else profile]
                if profile == "repair": extra += ["--mobile-depth-repair", "1"]
                if args.source_paced: extra += ["--source-paced", "1"]
                if args.deterministic_mapping: extra += ["--mobile-idle-mapping", "0"]
                limit = min(case["frame_count"], args.max_frames or case["frame_count"])
                invocation = SimpleNamespace(binary=args.binary, timeout=args.timeout, extra_args=extra)
                r = run_trial(invocation, dataset, case["init_first"], case["init_second"], limit, folder, binary_hash)
                r.update(profile=profile, full_input=limit == case["frame_count"], input_frames=case["frame_count"],
                         automatic_initialization=profile == "auto")
                # Evaluation reads the references only after the estimator exits.
                try:
                    if sha(dataset / "reference.json") != case["reference_sha256"]:
                        raise ValueError("Frozen evaluation reference changed")
                    r["evaluation"] = evaluate(dataset / "manifest.json", folder / "trajectory.csv")
                    r["visual_frames_all_input"] = r["evaluation"]["valid_output_frames"]
                    r["visual_coverage_all_input"] = r["evaluation"]["valid_output_coverage"]
                    if profile == "auto":
                        apply_auto_denominator(r, case["frame_count"])
                    write_json(folder / "evaluation.json", r["evaluation"])
                except Exception as error:
                    r["evaluation_error"] = str(error)
                results[key].append(r)
                write_json(args.output / "results.json", results)
                if trial_failed(r):
                    write_json(args.output / "failure.json", r)
                    raise RuntimeError(f"Trial failure preserved at {folder}; inspect before continuing")
    print(f"Evidence: {args.output}", flush=True)

if __name__ == "__main__":
    main()
