#!/usr/bin/env python3
"""Run transparent image-only initialization sweeps and repeated PTAM replays.

Every trial, including initialization failures, is retained. Candidate selection
uses visual matches/coverage only; reference scores are calculated afterward.
"""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import signal
import shutil
import subprocess
import time

import numpy as np

from evaluate_trajectory import evaluate, is_rigid, read_estimates
from storage import check_output_storage


CANDIDATES = {
    "lounge_reference": [(0, 14), (0, 15), (0, 16)],
    "lounge_new": [(0, 16), (0, 17), (0, 18)],
    "euroc_cam0": [(0, 5), (0, 10), (0, 15), (0, 20), (0, 30), (0, 40), (800, 895), (800, 900)],
    "euroc_cam1": [(0, 5), (0, 10), (0, 15), (0, 20), (0, 30), (0, 40), (800, 895), (800, 900)],
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, content):
    path.write_text(json.dumps(content, indent=2, allow_nan=False) + "\n")


def summarize(path, second, limit):
    if not path.exists():
        return dict(rows=0, visual_valid_frames=0, eligible_frames=max(0, limit-second-1),
                    visual_valid_fraction=0, maximum_map_points=0)
    rows = read_estimates(path)
    valid = [r for i, r in enumerate(rows) if i > second and
             r["quality"] == "2" and r["points"] > 0 and r["found"] >= 10 and
             (r["pose_inliers"] is None or r["pose_inliers"] >= 10) and
             is_rigid(r["matrix"]) and np.isfinite(r["timestamp"])]
    elapsed = [r["elapsed_ms"] for r in rows if np.isfinite(r["elapsed_ms"])]
    eligible = max(0, limit-second-1)
    # Hash algorithm output excluding nondeterministic runtime measurements.
    with path.open() as stream:
        records = list(csv.DictReader(stream))
    normalized = [{k: v for k, v in r.items() if k != "elapsed_ms"} for r in records]
    return dict(rows=len(rows), visual_valid_frames=len(valid), eligible_frames=eligible,
                visual_valid_fraction=len(valid)/eligible if eligible else 0,
                weak_visual_frames=sum(r["quality"] == "1" and r["points"] > 0 and r["found"] >= 10
                                       and (r["pose_inliers"] is None or r["pose_inliers"] >= 10)
                                       and is_rigid(r["matrix"]) for r in rows),
                maximum_map_points=max((r["points"] for r in rows), default=0),
                mean_found_valid=float(np.mean([r["found"] for r in valid])) if valid else 0,
                pose_state_sha256=hashlib.sha256(json.dumps(normalized, sort_keys=True).encode()).hexdigest(),
                elapsed_ms_median=float(np.median(elapsed)) if elapsed else None,
                elapsed_ms_p95=float(np.percentile(elapsed, 95)) if elapsed else None,
                elapsed_ms_max=float(max(elapsed)) if elapsed else None,
                nonfinite_rows_all_states=sum(not np.isfinite(r["matrix"]).all() or
                                              not np.isfinite(r["timestamp"]) or
                                              not np.isfinite(r["elapsed_ms"]) for r in rows),
                nonrigid_rows_all_states=sum(not is_rigid(r["matrix"]) for r in rows),
                handled_solver_failures=max((r["solver_failures"] for r in rows), default=0),
                retained_solver_stops=max((r["retained_solver_stops"] for r in rows), default=0))


def run_trial(args, dataset, first, second, limit, folder, binary_hash):
    check_output_storage(folder)
    if sha(args.binary) != binary_hash:
        raise RuntimeError("Core binary changed during the suite; preserve this suite and start a fresh one")
    manifest = json.loads((dataset / "manifest.json").read_text())
    lens_model=manifest.get("lens_model","pinhole")
    if lens_model not in {"pinhole","equidistant"}:
        raise ValueError(f"Unsupported manifest camera model {lens_model}")
    folder.mkdir(parents=True)
    output = folder / "trajectory.csv"
    command = [str(args.binary), "--frames", str(dataset / "frames.csv"),
               "--fx", str(manifest["fx"]), "--fy", str(manifest["fy"]),
               "--cx", str(manifest["cx"]), "--cy", str(manifest["cy"]),
               "--lens-model",lens_model,
               "--init-first", str(first), "--init-second", str(second),
               "--max-frames", str(limit), "--output", str(output)]
    command.extend(getattr(args, "extra_args", []))
    timed = ["/usr/bin/time", "-l"] + command if platform.system() == "Darwin" else ["/usr/bin/time", "-v"] + command
    write_json(folder / "command.json", dict(argv=command, time_argv=timed,
               binary_sha256=binary_hash, manifest_sha256=sha(dataset / "manifest.json"),
               init_first=first, init_second=second, max_frames=limit,
               ground_truth_available_to_estimator=False))
    start = time.monotonic()
    timed_out = False
    numerical_abort = None
    log_offset = 0
    log_tail = b""
    with (folder / "stdout.log").open("w") as out, (folder / "stderr.log").open("w") as err:
        process = subprocess.Popen(timed, stdout=out, stderr=err, start_new_session=True)
        while True:
            try:
                code = process.wait(timeout=.2)
                break
            except subprocess.TimeoutExpired:
                with (folder / "stderr.log").open("rb") as stream:
                    stream.seek(log_offset)
                    new_log = stream.read(1024*1024)
                    log_offset = stream.tell()
                check = log_tail + new_log
                log_tail = check[-128:]
                if b"NANNERY!" in check or b"tried to give me a nan" in check:
                    numerical_abort = "Internal bundle-adjustment NaN detected; trial stopped and suite halted"
                timed_out = time.monotonic()-start > args.timeout
                if numerical_abort or timed_out:
                    try:
                        os.killpg(process.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    try:
                        code = process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        try:
                            os.killpg(process.pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                        code = process.wait()
                    break
    stderr = (folder / "stderr.log").read_text()
    stdout = (folder / "stdout.log").read_text()
    actual_models=re.findall(r"\blens_model=(pinhole|equidistant)\b",stdout)
    camera_protocol_failure=None
    if lens_model == "equidistant" and (not actual_models or any(x!=lens_model for x in actual_models)):
        camera_protocol_failure="Native camera model was not confirmed by replay; do not evaluate as equidistant"
    if numerical_abort is None and ("NANNERY!" in stderr or "tried to give me a nan" in stderr):
        numerical_abort = "Internal bundle-adjustment NaN was reported before process exit; suite halted"
    if platform.system() == "Darwin":
        match = re.search(r"(\d+)\s+maximum resident set size", stderr)
        rss = int(match.group(1)) if match else None
    else:
        match = re.search(r"Maximum resident set size \(kbytes\):\s*(\d+)", stderr)
        rss = int(match.group(1))*1024 if match else None
    result = dict(exit_code=code, timed_out=timed_out, wall_seconds=time.monotonic()-start,
                  numerical_abort=numerical_abort,
                  requested_lens_model=lens_model,actual_lens_models=actual_models,
                  camera_protocol_failure=camera_protocol_failure,
                  internal_nan_messages=stderr.count("NANNERY!") + stderr.count("tried to give me a nan"),
                  handled_solver_rejection_messages=stderr.count("Bundle solve rejected:"),
                  peak_rss_bytes=rss, init_first=first, init_second=second,
                  max_frames=limit, folder=str(folder), **summarize(output, second, limit))
    result["initialization_failed"] = (code == 2 and result["rows"] == limit and bool(re.search(r"\bmap_good=0\b", stdout)))
    write_json(folder / "result.json", result)
    print(f"{folder.name}: exit={code} visual={result['visual_valid_frames']}/{result['eligible_frames']} wall={result['wall_seconds']:.2f}s", flush=True)
    return result


def trial_failed(result):
    return ((result["exit_code"] != 0 and not result.get("initialization_failed", False)) or result["timed_out"] or
            result["numerical_abort"] or result["camera_protocol_failure"] or
            result.get("nonfinite_rows_all_states", 0) or
            result.get("nonrigid_rows_all_states", 0))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=Path("build/iptam_replay"))
    parser.add_argument("--datasets-root", type=Path, default=Path("data/prepared"))
    parser.add_argument("--native-lounge-root",type=Path,
                        help="Use exact native lounge datasets here while keeping EuRoC under --datasets-root")
    parser.add_argument("--datasets", nargs="+", choices=list(CANDIDATES)+["actual_lounge"], default=list(CANDIDATES))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--sweep-window", type=int, default=80)
    parser.add_argument("--timeout", type=float, default=900)
    parser.add_argument("--skip-sweep", action="store_true")
    parser.add_argument("--selection-from", type=Path,
                        help="Freeze initialization pairs from an earlier suite for a fair variant comparison")
    parser.add_argument("--candidates-from", type=Path,
                        help="JSON mapping dataset names to image-only [first,second] candidate pairs")
    parser.add_argument("--plan-only", action="store_true")
    args = parser.parse_args()
    args.binary = args.binary.resolve()
    check_output_storage(args.output)
    if args.output.exists():
        raise FileExistsError(f"Refusing to overwrite {args.output}")
    if args.repeats < 1 or args.sweep_window < 1 or not np.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("repeats/sweep-window/timeout must be positive and finite")
    args.output.mkdir(parents=True)
    def dataset_directory(name):
        return (args.native_lounge_root if args.native_lounge_root and name.startswith("lounge_") else args.datasets_root)/name
    binary_hash = sha(args.binary)
    snapshot = args.output/"provenance"
    snapshot.mkdir()
    shutil.copy2(args.binary, snapshot/"iptam_replay")
    for source in (Path(__file__), Path(__file__).with_name("evaluate_trajectory.py")):
        shutil.copy2(source, snapshot/source.name)
    candidate_map = json.loads(args.candidates_from.read_text()) if args.candidates_from else CANDIDATES
    if args.candidates_from:
        shutil.copy2(args.candidates_from, snapshot/"candidates.json")
    plan = dict(binary=str(args.binary), binary_sha256=binary_hash,
                script_sha256=sha(Path(__file__)), platform=platform.platform(),
                evaluator_sha256=sha(Path(__file__).with_name("evaluate_trajectory.py")),
                selection="highest visual-valid fraction in fixed post-initialization window; tie mean found; tie earliest first/second; no reference scores",
                visual_validity="quality=2,points>0,found>=10,pose_inliers>=10 when logged,finite rigid pose; weak quality1 separate",
                repeats=args.repeats, candidates=candidate_map, sweep_window=args.sweep_window,
                reference_used_for_selection=False, skip_sweep=args.skip_sweep,
                selection_from=str(args.selection_from) if args.selection_from else None,
                candidates_from=str(args.candidates_from) if args.candidates_from else None,
                native_lounge_root=str(args.native_lounge_root) if args.native_lounge_root else None,
                source_manifest_paths={name:str(dataset_directory(name)/"manifest.json") for name in args.datasets},
                source_manifest_hashes={name: sha(dataset_directory(name)/"manifest.json") for name in args.datasets})
    write_json(args.output / "plan.json", plan)
    if args.plan_only:
        return
    results = {}
    for name in args.datasets:
        dataset = dataset_directory(name)
        manifest = json.loads((dataset / "manifest.json").read_text())
        total = len(manifest["frames"])
        if args.candidates_from:
            candidates=[tuple(p) for p in candidate_map[name]]
            if not candidates:
                raise ValueError(f"No image-supported initialization candidates for {name}")
        elif args.selection_from:
            prior = json.loads((args.selection_from/"results.json").read_text())[name]
            candidates = [tuple(prior["selected_initialization"])]
        else:
            candidates = [(manifest["init_first"], manifest["init_second"])] if args.skip_sweep or name not in CANDIDATES else CANDIDATES[name]
        sweep = []
        for first, second in candidates:
            folder = args.output / name / f"sweep_{first}_{second}"
            sweep.append(run_trial(args, dataset, first, second,
                                   min(total, second+1+args.sweep_window), folder, binary_hash))
            if trial_failed(sweep[-1]):
                write_json(args.output/"failure.json", sweep[-1])
                raise RuntimeError(f"Trial failure preserved in {folder}; recovery must be validated before another trial")
        selected = max(sweep, key=lambda r: (r["visual_valid_fraction"], r.get("mean_found_valid",0),
                                             -r["init_first"], -r["init_second"]))
        write_json(args.output/name/"selection.json", dict(
            init_first=selected["init_first"], init_second=selected["init_second"],
            selected_without_ground_truth=True, all_candidates=sweep))
        repeated = []
        for repeat in range(args.repeats):
            folder = args.output/name/f"repeat_{repeat+1:02d}"
            result = run_trial(args, dataset, selected["init_first"], selected["init_second"], total, folder, binary_hash)
            if trial_failed(result):
                write_json(args.output/"failure.json", result)
                raise RuntimeError(f"Trial failure preserved in {folder}; recovery must be validated before another trial")
            if (folder/"trajectory.csv").exists():
                try:
                    metrics = evaluate(dataset/"manifest.json", folder/"trajectory.csv")
                    write_json(folder/"metrics.json", metrics)
                    result["evaluation"] = metrics
                except Exception as error:
                    result["evaluation_error"] = str(error)
            repeated.append(result)
            # Incremental report preserves all finished evidence during a long suite.
            results[name] = dict(sweep=sweep, selected_initialization=[selected["init_first"], selected["init_second"]],
                                  manifest_path=str(dataset/"manifest.json"),
                                  repetitions=repeated,
                                  identical_pose_states=len({r.get("pose_state_sha256") for r in repeated}) == 1)
            write_json(args.output/"results.json", results)
    print(f"All evidence: {args.output}", flush=True)


if __name__ == "__main__":
    main()
