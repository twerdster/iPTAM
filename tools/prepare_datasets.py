#!/usr/bin/env python3
"""Rectify local lounge/EuRoC inputs for the original monocular PTAM camera.

Requires NumPy, OpenCV and SciPy. Source datasets are read only. Ground truth
is exported separately for evaluation and never changes the frame selection.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from storage import check_output_storage


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def read_json(path):
    return json.loads(path.read_text())


def read_csv(path):
    with path.open() as stream:
        return [r for r in csv.reader(stream) if r and not r[0].startswith("#")]


def pinhole_maps(width, height, focal, source_k, distortion, lens):
    """Map target pinhole pixel centers to original source pixel centers."""
    cx, cy = (width - 1) / 2, (height - 1) / 2
    yy, xx = np.indices((height, width), dtype=np.float64)
    x, y = (xx - cx) / focal, (yy - cy) / focal
    if lens == "ideal_equidistant":
        radius = np.hypot(x, y)
        factor = np.divide(np.arctan(radius), radius,
                           out=np.ones_like(radius), where=radius > 0)
        x, y = x * factor, y * factor
    elif lens == "radtan":
        k1, k2, p1, p2 = distortion
        r2 = x * x + y * y
        radial = 1 + k1 * r2 + k2 * r2 * r2
        x, y = (x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x),
                y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y)
    else:
        raise ValueError(f"Unsupported source lens {lens}")
    k = np.asarray(source_k)
    return (k[0, 0] * x + k[0, 2]).astype(np.float32), \
        (k[1, 1] * y + k[1, 2]).astype(np.float32)


def interpolate_poses(times, matrices, exposures):
    times, matrices = np.asarray(times), np.asarray(matrices)
    if len(times) < 2 or np.any(np.diff(times) <= 0):
        raise ValueError("Reference times must be strictly increasing")
    rotation = Slerp(times, Rotation.from_matrix(matrices[:, :3, :3]))
    output = []
    for t in exposures:
        if t < times[0] or t > times[-1]:
            continue
        m = np.eye(4)
        m[:3, :3] = rotation([t]).as_matrix()[0]
        m[:3, 3] = [np.interp(t, times, matrices[:, i, 3]) for i in range(3)]
        output.append({"timestamp": t, "matrix": m.tolist()})
    return output


def source_dataset(name, viobench, replay):
    if name.startswith("lounge_"):
        sub = {"lounge_reference": "lounge_reference",
               "lounge_new": "new_simulation_test"}[name]
        root = viobench / "datasets" / sub
        rig, manifest = read_json(root / "rig.json"), read_json(root / "manifest.json")
        origin = manifest["timestamp_origin_s"]
        rows = read_csv(root / "mav0/cam0/data.csv")
        frames = [(int(r[0]) * 1e-9 - origin,
                   root / "mav0/cam0/data" / r[1], int(r[0])) for r in rows]
        truth_raw = read_json(root / "ground_truth.json")["truth"]
        render_to_cv = np.diag([1, -1, -1, 1])
        gt = [{"timestamp": r["relative_timestamp"],
               "matrix": (np.array(r["camera_transform_rows"]) @ render_to_cv).tolist()}
              for r in truth_raw]
        # Preserve exact sidecar timestamps where present (CSV ns rounding <=0.5ns).
        if (root / "timestamps.json").exists():
            sidecar = read_json(root / "timestamps.json")
            if isinstance(sidecar, list) and len(sidecar) == len(frames):
                print(f"{name}: exact timestamp sidecar retained in provenance")
        return dict(root=root, frames=frames, width=rig["width"], height=rig["height"],
                    k=rig["intrinsics_rows"], distortion=[0] * 4,
                    lens=rig["lens_model"], truth=gt, reference_role="simulation truth",
                    context=[root / "rig.json", root / "manifest.json", root / "ground_truth.json"],
                    # Image-only LK correspondence inspection: retain several
                    # hundred tracks with about 20 pixels of parallax.
                    init_second=15 if name == "lounge_reference" else 17,
                    origin=origin, truth_source=str(root / "ground_truth.json"))
    camera = name.rsplit("_", 1)[-1]
    root = replay / "experiments" / f"euroc_original_{camera}"
    calibration = read_json(root / "calibration_used.json")["cameras"]["ultra"]
    ingest = read_json(root / "ingest_report.json")
    frames = [(r["relative_timestamp"], root / "data" / r["path"], r["timestamp_ns"])
              for r in read_json(root / "data/cameras.json")]
    gt_file = replay / "experiments/euroc_replay/data/groundtruth/state_groundtruth_estimate0/data.csv"
    rows = read_csv(gt_file)
    times = [(int(r[0]) - ingest["origin_ns"]) * 1e-9 for r in rows]
    numbers = np.asarray([[float(v) for v in r[1:8]] for r in rows])
    body = np.repeat(np.eye(4)[None], len(rows), axis=0)
    body[:, :3, :3] = Rotation.from_quat(numbers[:, [4, 5, 6, 3]]).as_matrix()
    body[:, :3, 3] = numbers[:, :3]
    optical = body @ np.asarray(calibration["T_IMU_from_CVCamera_rows"])
    truth = interpolate_poses(times, optical, [r[0] for r in frames])
    return dict(root=root, frames=frames, width=calibration["width"],
                height=calibration["height"], k=calibration["intrinsics_rows"],
                distortion=calibration["distortion_coefficients"], lens="radtan",
                truth=truth, reference_role="EuRoC Leica/IMU batch reference",
                context=[root / "calibration_used.json", root / "ingest_report.json", gt_file],
                init_second=30, origin_ns=ingest["origin_ns"], truth_source=str(gt_file))


def prepare(name, args):
    check_output_storage(args.output)
    source = source_dataset(name, args.viobench, args.replay)
    output = args.output / name
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite {output}")
    native = args.lens_model == "native"
    if native:
        if source["lens"] != "ideal_equidistant":
            raise ValueError("Native output is supported only for ideal equidistant lounge sources; rectify EuRoC radtan")
        if args.focal is not None or (args.width,args.height) != (source["width"],source["height"]):
            raise ValueError("Native output preserves the exact source dimensions and K")
        k=np.asarray(source["k"])
        intrinsics=dict(width=source["width"],height=source["height"],
                        fx=float(k[0,0]),fy=float(k[1,1]),cx=float(k[0,2]),cy=float(k[1,2]))
        yy,xx=np.indices((source["height"],source["width"]))
        theta=np.hypot((xx-k[0,2])/k[0,0],(yy-k[1,2])/k[1,1])
        valid=theta < np.pi/2-1e-6
        lens_model="equidistant"
        operation=("Original grayscale pixels copied; invalid back-hemisphere corners zeroed" if args.mask_native_corners else
                   "Original grayscale pixels copied losslessly; separate valid-domain mask, no source pixels altered")
    else:
        focal = args.focal if args.focal is not None else (460 if name.startswith("euroc_") else 400)
        mapx, mapy = pinhole_maps(args.width, args.height, focal, source["k"],
                                  source["distortion"], source["lens"])
        # A full bilinear support margin prevents invented black-border features.
        valid = ((mapx >= 0) & (mapy >= 0) &
                 (mapx < source["width"] - 1) & (mapy < source["height"] - 1))
        if not np.all(valid):
            raise ValueError(f"{name}: target optics have {np.size(valid)-valid.sum()} unsupported pixels; increase focal")
        intrinsics = dict(width=args.width, height=args.height, fx=focal, fy=focal,
                          cx=(args.width - 1) / 2, cy=(args.height - 1) / 2)
        lens_model="pinhole"
        operation="cv2.remap INTER_LINEAR grayscale uint8"
    check_output_storage(args.output)
    (output / "images").mkdir(parents=True)
    count = args.max_frames or len(source["frames"])
    selected = source["frames"][:count]
    records = []
    pixel_hash = hashlib.sha256()
    source_pixel_hash = hashlib.sha256()
    for i, (timestamp, path, timestamp_ns) in enumerate(selected):
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None or image.shape != (source["height"], source["width"]):
            raise ValueError(f"Invalid source image {path}")
        source_pixel_hash.update(image.tobytes())
        if native:
            rectified=image.copy()
            if args.mask_native_corners:
                rectified[~valid]=0
        else:
            rectified = cv2.remap(image, mapx, mapy, cv2.INTER_LINEAR)
        relative = f"images/{i:06d}.pgm"
        target = output / relative
        if not cv2.imwrite(str(target), rectified):
            raise OSError(f"Writing {target} failed")
        pixel_hash.update(rectified.tobytes())
        records.append(dict(timestamp=timestamp, path=relative,
                            original_timestamp_ns=timestamp_ns,
                            original_path=str(path)))
    if native:
        cv2.imwrite(str(output/"valid_mask.pgm"),valid.astype(np.uint8)*255)
    manifest = dict(**intrinsics, name=name, lens_model=lens_model, distortion=0,
                    init_first=0, init_second=min(source["init_second"], len(records)-1),
                    frames=[{k: r[k] for k in ("path", "timestamp")} for r in records])
    if native:
        manifest.update(valid_mask="valid_mask.pgm",invalid_corner_pixels=int((~valid).sum()),
                        pixels_preserved_exactly=not args.mask_native_corners,
                        domain="forward hemisphere theta<pi/2-1e-6; model rejects invalid corners; no source pixels resampled")
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    with (output / "frames.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["timestamp", "path"])
        writer.writerows((format(r["timestamp"], ".17g"), r["path"]) for r in records)
    exposure_times = np.asarray([r["timestamp"] for r in records])
    truth = [r for r in source["truth"]
             if exposure_times[0] - 1e-6 <= r["timestamp"] <= exposure_times[-1] + 1e-6]
    (output / "reference.json").write_text(json.dumps(dict(
        frame="T_world_from_CV_camera; +X right,+Y down,+Z forward; metres",
        role=source["reference_role"], source=source["truth_source"],
        evaluation_only=True, poses=truth), indent=2) + "\n")
    provenance = dict(source_root=str(source["root"]), source_lens=source["lens"],
                      source_intrinsics=source["k"], source_distortion=source["distortion"],
                      target_intrinsics=intrinsics, image_operation=operation,
                      output_lens_model=lens_model,
                      all_output_pixels_from_source=True,
                      all_pixels_have_bilinear_source_support=not native,
                      clipped_outside_model_domain_pixels=int((~valid).sum()) if native and args.mask_native_corners else 0,
                      masked_outside_model_domain_pixels=int((~valid).sum()) if native else 0,
                      frame_count=len(records), source_frame_count=len(source["frames"]),
                      frames=records, source_context_sha256={str(p): digest(p) for p in source["context"]},
                      rectified_pixels_concatenated_sha256=pixel_hash.hexdigest(),
                      source_pixels_concatenated_sha256=source_pixel_hash.hexdigest(),
                      camera_to_camera_rotation="identity; optical center unchanged",
                      imu_used=False, original_clock_origin_ns=source.get("origin_ns"),
                      original_clock_origin_s=source.get("origin"),
                      script_sha256=digest(Path(__file__)))
    (output / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    print(f"{name}: {len(records)} frames -> {output}; {len(truth)} reference poses", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--datasets", nargs="+", choices=["lounge_reference", "lounge_new", "euroc_cam0", "euroc_cam1"],
                        default=["lounge_reference", "lounge_new", "euroc_cam0", "euroc_cam1"])
    parser.add_argument("--viobench", type=Path, help="VIOBench source root, required for lounge datasets")
    parser.add_argument("--replay", type=Path, help="ARReplayLab source root, required for EuRoC datasets")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--focal", type=float, default=None,
                        help="target focal in pixels (default lounge400, EuRoC460, for supported pixels)")
    parser.add_argument("--lens-model",choices=["pinhole","native"],default="pinhole",
                        help="native preserves ideal equidistant lounge pixels/K; pinhole rectifies/crops")
    parser.add_argument("--mask-native-corners",action="store_true",
                        help="optionally zero the16 native back-hemisphere corner pixels; default is exact pixel preservation with separate domain mask")
    parser.add_argument("--max-frames", type=int, default=0)
    args = parser.parse_args()
    if args.width < 64 or args.height < 64 or (args.focal is not None and args.focal <= 0) or args.max_frames < 0:
        parser.error("Invalid dimensions, focal length or frame limit")
    if any(name.startswith("lounge_") for name in args.datasets) and args.viobench is None:
        parser.error("--viobench is required for lounge datasets")
    if any(name.startswith("euroc_") for name in args.datasets) and args.replay is None:
        parser.error("--replay is required for EuRoC datasets")
    check_output_storage(args.output)
    for name in args.datasets:
        prepare(name, args)


if __name__ == "__main__":
    main()
