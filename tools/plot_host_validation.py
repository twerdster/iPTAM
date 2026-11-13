#!/usr/bin/env python3
"""Plot actual PTAM trajectories, explicit tracking gaps and reference errors."""
import argparse
import json
from pathlib import Path
import shutil

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from evaluate_trajectory import is_rigid, read_estimates
from storage import check_output_storage


COLORS = ["#0072B2", "#D55E00", "#009E73", "#CC79A7"]


def segments(indices):
    indices = np.asarray(indices, dtype=int)
    return np.split(indices, np.where(np.diff(indices) > 1)[0] + 1) if len(indices) else []


def draw_dataset(name, info, root, trajectory_axis, coverage_axis, error_axis=None):
    manifest_path=Path(info.get("manifest_path",root/name/"manifest.json"))
    dataset = manifest_path.parent
    manifest = json.loads(manifest_path.read_text())
    truth = json.loads((dataset/"reference.json").read_text())["poses"]
    truth_times = np.asarray([p["timestamp"] for p in truth])
    positions = np.asarray([np.asarray(p["matrix"])[:3, 3] for p in truth])
    all_times = np.asarray([p["timestamp"] for p in manifest["frames"]])
    if len(positions):
        trajectory_axis.plot(positions[:, 0], positions[:, 1], color="#555555", lw=1.5, label="reference")
    lines = []
    total_good = 0
    for repeat, result in enumerate(info["repetitions"]):
        color = COLORS[repeat % len(COLORS)]
        rows = read_estimates(Path(result["folder"])/"trajectory.csv")
        row_times = np.asarray([r["timestamp"] for r in rows])
        good = np.asarray([r["quality"] == "2" and r["found"] >= 10 and r["points"] > 0
                           and is_rigid(r["matrix"]) and np.isfinite(r["timestamp"]) for r in rows])
        weak = np.asarray([r["quality"] == "1" and r["found"] >= 10 and r["points"] > 0
                           and is_rigid(r["matrix"]) and np.isfinite(r["timestamp"]) for r in rows])
        total_good += int(good.sum())
        initialized = np.asarray([r["quality"] != "-1" for r in rows])
        state = np.where(good, "#169B62", np.where(weak, "#E69F00", np.where(initialized,"#C33333","#BBBBBB")))
        coverage_axis.scatter(row_times, np.full(len(rows), repeat+1), c=state,
                              marker="|", s=100, linewidths=1.6, rasterized=True)
        metrics = result.get("evaluation", {})
        fit = metrics.get("sim3")
        count = metrics.get("valid_output_frames", 0)
        rms = metrics.get("position_rms_m")
        lines.append(f"run{repeat+1}: {count}/{len(all_times)} good; Sim3 RMS {rms:.3f}m" if rms is not None else f"run{repeat+1}: {count}/{len(all_times)} good; no accuracy fit")
        if not fit:
            continue
        r, t = np.asarray(fit["rotation"]), np.asarray(fit["translation"])
        estimated = np.asarray([p["matrix"][:3, 3] for p in rows])
        aligned = fit["scale"] * estimated @ r.T + t
        indices = np.flatnonzero(good)
        for n, segment in enumerate(segments(indices)):
            trajectory_axis.plot(aligned[segment, 0], aligned[segment, 1],
                                 color=color, lw=1.2, marker="." if len(segment)==1 else None,
                                 label=f"run{repeat+1} good" if n == 0 else None)
        if error_axis is not None:
            error = np.full(len(rows), np.nan)
            for i in indices:
                j = np.argmin(abs(truth_times-row_times[i]))
                if abs(truth_times[j]-row_times[i]) <= 1e-6:
                    error[i] = np.linalg.norm(aligned[i]-positions[j])
            for n, segment in enumerate(segments(np.flatnonzero(np.isfinite(error)))):
                error_axis.plot(row_times[segment], error[segment], color=color, lw=1.2,
                                marker="." if len(segment)==1 else None,
                                label=f"run{repeat+1}" if n == 0 else None)
    trajectory_axis.set(title=f"{name}: XY trajectory (Sim3 fitted scale)", xlabel="world X [m]", ylabel="world Y [m]")
    trajectory_axis.axis("equal")
    trajectory_axis.grid(alpha=.2)
    trajectory_axis.legend(fontsize=8, loc="best")
    if total_good == 0:
        trajectory_axis.set_title(f"{name}: reference XY; no initialized trajectory")
        trajectory_axis.text(.5,.08,"No initialized GOOD trajectory",ha="center",transform=trajectory_axis.transAxes,
                             bbox=dict(facecolor="white",alpha=.9,edgecolor="#D55E00"))
    coverage_axis.set(xlabel="source relative time [s]", ylabel="repetition",
                      yticks=list(range(1,len(info["repetitions"])+1)), xlim=(all_times[0],all_times[-1]),
                      title="Good = green; weak = orange; lost = red; initialization = gray")
    coverage_axis.set_ylim(.5,len(info["repetitions"])+.5)
    coverage_axis.grid(axis="x", alpha=.2)
    if error_axis is not None:
        error_axis.set(xlabel="source relative time [s]", ylabel="position error [m]",
                       title="Error only at good original exposures; gaps remain blank",
                       xlim=(all_times[0],all_times[-1]))
        error_axis.grid(alpha=.2)
        if error_axis.get_legend_handles_labels()[0]:
            error_axis.legend(fontsize=8)
    return lines


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", type=Path, required=True)
    parser.add_argument("--datasets-root", type=Path, default=Path("data/prepared"))
    parser.add_argument("--overview-copy", type=Path)
    args = parser.parse_args()
    check_output_storage(args.suite)
    if args.overview_copy:
        check_output_storage(args.overview_copy)
    results = json.loads((args.suite/"results.json").read_text())
    plt.rcParams.update({"font.size":10,"axes.spines.top":False,"axes.spines.right":False})
    for name, info in results.items():
        fig = plt.figure(figsize=(12,10))
        grid = fig.add_gridspec(3,1, height_ratios=[2.2,.8,1.3])
        ax, coverage, error = [fig.add_subplot(grid[i]) for i in range(3)]
        lines = draw_dataset(name,info,args.datasets_root,ax,coverage,error)
        fig.suptitle("Original PTAM replay: actual outputs, including failures\n"+"; ".join(lines), fontsize=11)
        fig.subplots_adjust(left=.08,right=.98,bottom=.06,top=.88,hspace=.45)
        fig.savefig(args.suite/name/"trajectory_coverage.png",dpi=140)
        plt.close(fig)
    fig, axes = plt.subplots(len(results),2,figsize=(14,max(4.8,3.7*len(results))),squeeze=False)
    for i,(name,info) in enumerate(results.items()):
        draw_dataset(name,info,args.datasets_root,axes[i,0],axes[i,1])
    fig.suptitle("Original PTAM: full-sequence repeated replay; fitted monocular scale; tracking gaps are explicit", fontsize=13)
    fig.subplots_adjust(left=.07,right=.985,bottom=.15 if len(results)==1 else .05,
                        top=.81 if len(results)==1 else .94,hspace=.55,wspace=.25)
    target=args.suite/"overview.png"
    fig.savefig(target,dpi=130)
    plt.close(fig)
    if args.overview_copy:
        args.overview_copy.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(target,args.overview_copy)
        args.overview_copy.chmod(args.overview_copy.stat().st_mode & ~0o111)
    print(target)


if __name__ == "__main__":
    main()
