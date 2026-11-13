#!/usr/bin/env python3
"""Check optical-frame references against independently tracked image features.

This is an evaluation/data audit, never an estimator or initialization input.
Correct reference extrinsics should predict epipolar lines in the actual images.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from storage import check_output_storage


def median_epipolar_error(first, second, before, after, k):
    relative = np.linalg.inv(after) @ before
    x, y, z = relative[:3, 3]
    essential = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]]) @ relative[:3, :3]
    inverse = np.linalg.inv(k)
    fundamental = inverse.T @ essential @ inverse
    lines = first @ fundamental.T
    return float(np.median(abs(np.sum(second * lines, axis=1)) / np.linalg.norm(lines[:, :2], axis=1)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--datasets-root", type=Path, default=Path("data/prepared"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    check_output_storage(args.output)
    if args.output.exists():
        raise FileExistsError(args.output)
    results = {}
    for name, a, b in [("lounge_reference", 15, 16), ("lounge_new", 17, 18),
                       ("euroc_cam0", 1000, 1002), ("euroc_cam1", 1000, 1002)]:
        root = args.datasets_root/name
        m = json.loads((root/"manifest.json").read_text())
        reference = json.loads((root/"reference.json").read_text())["poses"]
        times = np.asarray([p["timestamp"] for p in reference])
        poses = [np.asarray(reference[np.argmin(abs(times-m["frames"][j]["timestamp"]))]["matrix"]) for j in (a,b)]
        images = [cv2.imread(str(root/m["frames"][j]["path"]),0) for j in (a,b)]
        first = cv2.goodFeaturesToTrack(images[0],600,.02,8)
        second, ok, _ = cv2.calcOpticalFlowPyrLK(images[0],images[1],first,None)
        backward, ok_back, _ = cv2.calcOpticalFlowPyrLK(images[1],images[0],second,None)
        keep = ok[:,0].astype(bool) & ok_back[:,0].astype(bool) & (np.linalg.norm(backward-first,axis=2)[:,0] < .5)
        first = np.c_[first[keep,0],np.ones(keep.sum())]
        second = np.c_[second[keep,0],np.ones(keep.sum())]
        k = np.array([[m["fx"],0,m["cx"]],[0,m["fy"],m["cy"]],[0,0,1]])
        correct = median_epipolar_error(first,second,*poses,k)
        flip = np.diag([1,-1,-1,1])
        wrong = median_epipolar_error(first,second,poses[0]@flip,poses[1]@flip,k)
        results[name] = dict(frames=[a,b], matches=int(keep.sum()),
                             median_epipolar_error_px=correct,
                             deliberately_wrong_render_axes_error_px=wrong,
                             passed=bool(keep.sum() >= 100 and correct < 1 and wrong > 2))
    report = dict(evaluation_only=True, feature_method="GFTT600 + LK, forward/back <0.5px",
                  numpy=np.__version__, opencv=cv2.__version__, datasets=results,
                  passed=all(r["passed"] for r in results.values()))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps(report,indent=2))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
