#!/usr/bin/env python3
"""Evaluate PTAM's monocular optical-camera poses with explicit Sim(3) gauge.

The reference is evaluation-only. The estimated trajectory uses portable
iptam_replay CSV columns timestamp,tx,ty,tz,r00,...,r22,quality. All poses are
T_world_from_CV_camera. No reference pose initializes or feeds the estimator.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from storage import check_output_storage
from scipy.spatial.transform import Rotation


def umeyama(source, target):
    """Least-squares proper rotation, translation and positive scale."""
    source, target = np.asarray(source), np.asarray(target)
    a, b = source.mean(axis=0), target.mean(axis=0)
    x, y = source - a, target - b
    variance = np.sum(x * x) / len(x)
    if variance < 1e-12 or np.linalg.matrix_rank(x, tol=1e-7) < 2:
        raise ValueError("Trajectory does not constrain Sim3 alignment (insufficient translation)")
    u, singular, vt = np.linalg.svd(y.T @ x / len(x))
    signs = np.ones(3)
    if np.linalg.det(u @ vt) < 0:
        signs[-1] = -1
    r = u @ np.diag(signs) @ vt
    scale = float(np.sum(singular * signs) / variance)
    if scale <= 0:
        raise ValueError("Nonpositive fitted scale")
    return scale, r, b - scale * r @ a


def is_rigid(matrix):
    m = np.asarray(matrix)
    return bool(m.shape == (4, 4) and np.isfinite(m).all() and
                np.allclose(m[3], [0, 0, 0, 1], atol=1e-6) and
                np.allclose(m[:3, :3].T @ m[:3, :3], np.eye(3), atol=2e-3) and
                abs(np.linalg.det(m[:3, :3]) - 1) < 2e-3)


def read_estimates(path):
    rows = []
    with path.open() as stream:
        contents = list(csv.DictReader(stream))
    for row in contents:
        matrix = np.eye(4)
        matrix[:3, 3] = [float(row[k]) for k in ("tx", "ty", "tz")]
        matrix[:3, :3] = [[float(row[f"r{i}{j}"]) for j in range(3)] for i in range(3)]
        rows.append(dict(timestamp=float(row["timestamp"]), matrix=matrix,
                         quality=row["quality"], map_id=row.get("map_id", "0"),
                         points=int(row.get("points", 0)),
                         found=int(row.get("found", 0)),
                         pose_inliers=int(row["pose_inliers"]) if row.get("pose_inliers") else None,
                         solver_failures=int(row.get("solver_failures", 0)),
                         retained_solver_stops=int(row.get("solver_retained_stops", row.get("retained_solves", 0))),
                         elapsed_ms=float(row.get("elapsed_ms", 0))))
    return rows


def evaluate(manifest_path, estimates_path):
    manifest = json.loads(manifest_path.read_text())
    reference = json.loads((manifest_path.parent / "reference.json").read_text())
    poses = reference["poses"]
    estimates = read_estimates(estimates_path)
    frame_times = np.asarray([r["timestamp"] for r in manifest["frames"]])
    truth_times = np.asarray([r["timestamp"] for r in poses])
    if not len(frame_times) or not np.isfinite(frame_times).all() or not np.isfinite(truth_times).all():
        raise ValueError("Dataset/reference times must be finite, with at least one input exposure")
    if np.any(np.diff(frame_times) <= 0) or np.any(np.diff(truth_times) <= 0):
        raise ValueError("Dataset/reference timestamps must be strictly increasing")
    if any(not is_rigid(r["matrix"]) for r in poses):
        raise ValueError("Invalid reference transform")
    matched, quality, invalid = [], {}, 0
    nonfinite_rows = sum(not np.isfinite(r["matrix"]).all() or not np.isfinite(r["timestamp"])
                         or not np.isfinite(r["elapsed_ms"]) for r in estimates)
    nonrigid_rows = sum(not is_rigid(r["matrix"]) for r in estimates)
    seen = set()
    for row in estimates:
        q = row["quality"].lower()
        quality[q] = quality.get(q, 0) + 1
        # Harness: good=2,poor=1,bad=0,initializing=-1. Initialization without
        # an actual nonempty map cannot count as a valid trajectory output.
        if (q not in {"good", "2"} or row["points"] <= 0 or row["found"] < 10 or
                (row["pose_inliers"] is not None and row["pose_inliers"] < 10)):
            continue
        if not np.isfinite(row["timestamp"]) or not is_rigid(row["matrix"]):
            invalid += 1
            continue
        i = int(np.argmin(np.abs(frame_times - row["timestamp"])))
        if abs(frame_times[i] - row["timestamp"]) > 1e-6:
            raise ValueError("Estimate is not at an original exposure timestamp")
        if i in seen:
            raise ValueError("Duplicate valid output exposure")
        seen.add(i)
        if not len(truth_times):
            continue
        j = int(np.argmin(np.abs(truth_times - row["timestamp"])))
        if abs(truth_times[j] - row["timestamp"]) <= 1e-6:
            matched.append((i, row, np.asarray(poses[j]["matrix"])))
    matched.sort(key=lambda r: r[0])
    report = dict(dataset=manifest.get("name"), input_frames=len(frame_times),
                  output_rows=len(estimates), valid_output_frames=len(seen),
                  valid_output_coverage=len(seen) / len(frame_times), quality_counts=quality,
                  weak_visual_frames=sum(r["quality"].lower() in {"poor", "dodgy", "1"} and
                                         r["points"] > 0 and r["found"] >= 10 and is_rigid(r["matrix"])
                                         and (r["pose_inliers"] is None or r["pose_inliers"] >= 10)
                                         for r in estimates),
                  reference_role=reference["role"], reference_frames=len(poses),
                  matched_reference_frames=len(matched), invalid_valid_quality_poses=invalid,
                  nonfinite_rows_all_states=nonfinite_rows, nonrigid_rows_all_states=nonrigid_rows,
                  handled_solver_failures=max((r["solver_failures"] for r in estimates), default=0),
                  retained_solver_stops=max((r["retained_solver_stops"] for r in estimates), default=0),
                  pose_inlier_diagnostics_available=bool(estimates) and all(r["pose_inliers"] is not None for r in estimates),
                  visual_validity_rule="quality=2(good),points>0,found>=10,pose_inliers>=10 when logged,finite rigid pose; weak quality1 separate",
                  alignment="best-fit Sim3 over matched valid initialized optical poses; scale is fitted",
                  metric_units="metres after Sim3 fitting; source monocular scale is arbitrary",
                  elapsed_ms_median=float(np.median([r["elapsed_ms"] for r in estimates])) if estimates else None)
    if matched:
        report["first_valid_frame"], report["last_valid_frame"] = min(seen), max(seen)
        report["missing_after_initialization"] = len(frame_times) - min(seen) - len(seen)
        gaps, current = [], []
        for i in range(min(seen), len(frame_times)):
            if i not in seen:
                current.append(i)
            elif current:
                gaps.append(current); current = []
        if current:
            gaps.append(current)
        report["tracking_gaps"] = [dict(first_frame=g[0], last_frame=g[-1], frames=len(g)) for g in gaps]
        report["largest_tracking_gap_frames"] = max((len(g) for g in gaps), default=0)
    map_ids = sorted({r[1]["map_id"] for r in matched})
    report["map_ids"] = map_ids
    if len(map_ids) > 1:
        report["failure"] = "Multiple map gauges: evaluate each map separately; one Sim3 would be misleading"
        return report
    if len(matched) < 3:
        report["failure"] = "Fewer than three reference-associated valid poses"
        return report
    estimate = np.asarray([r[1]["matrix"] for r in matched])
    truth = np.asarray([r[2] for r in matched])
    try:
        scale, rotation, translation = umeyama(estimate[:, :3, 3], truth[:, :3, 3])
    except ValueError as error:
        report["failure"] = str(error)
        return report
    aligned_position = scale * estimate[:, :3, 3] @ rotation.T + translation
    errors = np.linalg.norm(aligned_position - truth[:, :3, 3], axis=1)
    aligned_rotation = rotation[None] @ estimate[:, :3, :3]
    angles = np.rad2deg(Rotation.from_matrix(truth[:, :3, :3].transpose(0, 2, 1) @ aligned_rotation).magnitude())
    report.update(failure=None, sim3=dict(scale=scale, rotation=rotation.tolist(), translation=translation.tolist()),
                  position_rms_m=float(np.sqrt(np.mean(errors ** 2))),
                  position_median_m=float(np.median(errors)), position_max_m=float(max(errors)),
                  endpoint_m=float(errors[-1]), orientation_median_deg=float(np.median(angles)),
                  orientation_max_deg=float(max(angles)),
                  first_scored_timestamp=matched[0][1]["timestamp"],
                  last_scored_timestamp=matched[-1][1]["timestamp"])
    # Consecutive-image rotation disagreement is independent of the fitted
    # global Sim3 orientation. Do not bridge any missing visual output.
    adjacent = np.flatnonzero(np.diff([r[0] for r in matched]) == 1)
    relative_angles = []
    for i in adjacent:
        de = estimate[i, :3, :3].T @ estimate[i+1, :3, :3]
        dg = truth[i, :3, :3].T @ truth[i+1, :3, :3]
        relative_angles.append(float(np.rad2deg(Rotation.from_matrix(dg.T @ de).magnitude())))
    report["adjacent_good_pose_pairs"] = len(relative_angles)
    report["adjacent_rotation_median_deg"] = float(np.median(relative_angles)) if relative_angles else None
    report["adjacent_rotation_rms_deg"] = float(np.sqrt(np.mean(np.square(relative_angles)))) if relative_angles else None
    reference_positions = np.asarray([r["matrix"] for r in poses])[:, :3, 3]
    reference_distance = float(np.linalg.norm(np.diff(reference_positions, axis=0), axis=1).sum())
    covered_distance = float(sum(np.linalg.norm(truth[i+1, :3, 3]-truth[i, :3, 3]) for i in adjacent))
    report["reference_path_length_m"] = reference_distance
    report["good_adjacent_reference_path_length_m"] = covered_distance
    report["reference_motion_coverage"] = covered_distance / reference_distance if reference_distance > 1e-8 else None
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    check_output_storage(args.output)
    if args.output.exists():
        raise FileExistsError(f"Refusing to overwrite {args.output}")
    report = evaluate(args.manifest, args.trajectory)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
