#!/usr/bin/env python3
"""Create a calibrated textured-plane / occlusion / return integration fixture.

The plane is one metre away, with fixed pinhole intrinsics. Images are integer
translations of one deterministic texture, not poses substituted into PTAM.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import random


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    (root / "images").mkdir()
    width, height, focal = 640, 480, 400
    texture_width, texture_height = 832, 544
    texture = random.Random(42).randbytes(texture_width * texture_height)
    schedule = [(i, round(4 * math.sin(i / 12)), "translation") for i in range(60)]
    schedule += [(59, round(4 * math.sin(59 / 12)), "occluded") for _ in range(8)]
    schedule += [(59, round(4 * math.sin(59 / 12)), "return_to_map") for _ in range(8)]
    schedule += [(59 + i, round(8 * math.sin((59 + i) / 15)), "translation") for i in range(60)]
    frames, truth = [], []
    pixel_hash = hashlib.sha256()
    for i, (x, y, phase) in enumerate(schedule):
        timestamp = i / 30
        image = bytearray(width * height)
        if phase != "occluded":
            for row in range(height):
                start = (32 + y + row) * texture_width + 32 + x
                image[row * width:(row + 1) * width] = texture[start:start + width]
        relative = f"images/{i:06d}.pgm"
        (root / relative).write_bytes(f"P5\n{width} {height}\n255\n".encode() + image)
        pixel_hash.update(image)
        frames.append({"path": relative, "timestamp": timestamp, "phase": phase})
        truth.append({"timestamp": timestamp, "matrix": [[1, 0, 0, x / focal], [0, 1, 0, y / focal],
                                                         [0, 0, 1, 0], [0, 0, 0, 1]]})
    def save(name, data):
        (root / name).write_text(json.dumps(data, indent=2) + "\n")
    save("manifest.json", {"name": "plane_occlusion_return", "width": width, "height": height,
                           "fx": focal, "fy": focal, "cx": 319.5, "cy": 239.5,
                           "init_first": 0, "init_second": 20, "frames": frames,
                           "expected_occlusion_frames": [60, 67], "expected_return_frames": [68, 75]})
    save("reference.json", {"role": "analytic rendered plane truth", "evaluation_only": True,
                            "frame": "T_world_from_CV_camera; metres; plane at z=1", "poses": truth})
    save("provenance.json", {"seed": 42, "generator": str(Path(__file__).resolve()),
                             "generator_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                             "rectified_pixels_concatenated_sha256": pixel_hash.hexdigest(),
                             "lens_model": "exact pinhole; integer texture sampling; no noise/blur",
                             "frame_count": len(frames), "imu_used": False})
    with (root / "frames.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["timestamp", "path"])
        writer.writerows((format(r["timestamp"], ".17g"), r["path"]) for r in frames)
    print(f"Wrote {len(frames)} frames to {root}")


if __name__ == "__main__":
    main()
