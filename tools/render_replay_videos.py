#!/usr/bin/env python3
"""Render saved physical-iPhone replay logs; never run or alter an estimator.

Example: .venv/bin/python tools/render_replay_videos.py --output \
  testing/videos/example
Requires numpy, OpenCV, ffmpeg and ffprobe. Output is an offline visualization,
not an iPhone screen capture. Reference poses are used only for presentation of
the already-saved evaluation fit. Sparse source images are held by timestamp.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess

import cv2
import numpy as np

from storage import check_output_storage


CASES = [
    ("lounge_new", "New lounge | simulation", "SIMULATION TRUTH"),
    ("lounge_reference", "Historical lounge | simulation", "SIMULATION TRUTH"),
    ("euroc_cam0", "EuRoC MH01 | left camera", "EUROC LEICA / IMU REFERENCE"),
    ("euroc_cam1", "EuRoC MH01 | right camera", "EUROC LEICA / IMU REFERENCE"),
    ("actual_lounge", "Actual lounge | sparse ARKit export", "ARKIT REFERENCE (not independent truth)"),
]
COLORS = {"GOOD": (95, 215, 100), "WEAK": (60, 190, 245),
          "LOST": (90, 85, 235), "UNINITIALIZED": (155, 145, 135)}
FG = (235, 237, 242)
MUTED = (160, 170, 185)
W, H = 1280, 720


def digest(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def text(canvas, value, xy, scale=.55, color=FG, thickness=1):
    cv2.putText(canvas, str(value), xy, cv2.FONT_HERSHEY_SIMPLEX,
                scale, color, thickness, cv2.LINE_AA)


def state(row):
    if not row["initialized"]:
        return "UNINITIALIZED"
    supported = (row["finite"] and row["map_points"] > 0 and
                 row["found"] >= 10 and row["pose_inliers"] >= 10)
    if supported and row["quality"] == 2 and row["valid_visual_pose"]:
        return "GOOD"
    if supported and row["quality"] == 1:
        return "WEAK"
    return "LOST"


class Replay:
    def __init__(self, root, case):
        self.name, self.title, self.reference_label = case
        self.run = root / "phone" / f"final_geometric_support_{self.name}_01"
        self.provenance = json.loads((self.run / "provenance.json").read_text())
        self.source = Path(self.provenance["source"])
        self.manifest_path = self.run / "requested-manifest.json"
        self.manifest = json.loads(self.manifest_path.read_text())
        self.rows_path = self.run / "run/frames.jsonl"
        self.rows = [json.loads(line) for line in self.rows_path.read_text().splitlines()]
        self.summary = json.loads((self.run / "run/summary.json").read_text())
        assert digest(self.manifest_path) == self.provenance["manifest_sha256"] == self.summary["manifest_sha256"]
        self.evaluation = json.loads((self.run / "evaluation.json").read_text())
        self.reference_path = self.source / "reference.json"
        reference = json.loads(self.reference_path.read_text())
        self.times = np.array([f["timestamp"] for f in self.manifest["frames"]])
        assert len(self.rows) == len(self.times)
        assert np.all(np.diff(self.times) > 0)
        assert np.allclose(self.times, [r["timestamp"] for r in self.rows], atol=1e-6, rtol=0)
        self.states = [state(r) for r in self.rows]
        assert self.states.count("GOOD") == self.evaluation["valid_output_frames"]
        self.ref_times = np.array([r["timestamp"] for r in reference["poses"]])
        self.ref_pos = np.array([np.array(r["matrix"])[:3, 3] for r in reference["poses"]])
        # The saved fit maps estimated world -> reference world. Never refit here.
        sim = self.evaluation.get("sim3")
        self.estimated = None
        if sim:
            p = np.array([r["world_from_camera_t_qxyzw"][:3] for r in self.rows])
            self.estimated = sim["scale"] * (p @ np.array(sim["rotation"]).T) + sim["translation"]
        good = np.array([s == "GOOD" for s in self.states])
        all_points = self.ref_pos if self.estimated is None else np.vstack([self.ref_pos, self.estimated[good]])
        self.axes = np.sort(np.argsort(np.ptp(self.ref_pos, axis=0))[-2:])
        projected = all_points[:, self.axes]
        self.center = (projected.max(axis=0) + projected.min(axis=0)) / 2
        span = max(float(np.ptp(projected, axis=0).max()), .1)
        self.plot_scale = 310 / span
        self.ref_pixels = self.to_pixels(self.ref_pos)
        self.estimate_pixels = self.to_pixels(self.estimated) if self.estimated is not None else None
        self.span = float(self.times[-1] - self.times[0])
        self.source_duration = self.span + float(np.median(np.diff(self.times)))
        self.last_image_index = -1
        self.last_image = None
        pixel_hash, pgm_hash = hashlib.sha256(), hashlib.sha256()
        for entry in self.manifest["frames"]:
            path = self.source / entry["path"]
            image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if image is None or image.shape != (self.manifest["height"], self.manifest["width"]):
                raise ValueError(f"Unreadable or wrong-sized source: {path}")
            pixel_hash.update(image.tobytes())
            pgm_hash.update(path.read_bytes())
        assert pixel_hash.hexdigest() == self.summary["rectified_pixels_concatenated_sha256"]
        self.receipt = {
            "case": self.name, "source_dataset": str(self.source), "phone_run": str(self.run),
            "source_app_sha256": self.provenance["app"]["executable_sha256"],
            "inputs": {str(p): digest(p) for p in [self.manifest_path, self.rows_path,
                self.run / "run/summary.json", self.run / "evaluation.json", self.reference_path,
                self.run / "provenance.json", self.source / "manifest.json"]},
            "decoded_source_pixels_sha256": pixel_hash.hexdigest(),
            "source_pgm_files_concatenated_sha256": pgm_hash.hexdigest(),
            "source_frames": len(self.rows), "source_span_seconds": self.span,
            "reference_role": reference["role"], "saved_sim3": sim,
            "state_counts": {s: self.states.count(s) for s in COLORS},
            "plot_axes": ["XYZ"[i] for i in self.axes],
            "trajectory_policy": "Saved Sim3; only adjacent source exposures both GOOD are connected; no predicted or weak pose segments",
            "source_sampling": "Last exposure at or before output source time; sparse gaps held, no image interpolation",
        }

    def to_pixels(self, positions):
        p = (positions[:, self.axes] - self.center) * self.plot_scale
        return np.rint(p * [1, -1] + [983, 403]).astype(np.int32)

    def frame(self, output_time, duration, overview=False):
        source_time = self.times[0] + min(output_time / duration, 1) * self.source_duration
        index = int(np.clip(np.searchsorted(self.times, source_time, side="right") - 1, 0, len(self.rows) - 1))
        row, status = self.rows[index], self.states[index]
        canvas = np.full((H, W, 3), (20, 24, 31), np.uint8)
        text(canvas, self.title, (28, 38), .85, FG, 2)
        speed = self.source_duration / duration
        text(canvas, f"Visualization of saved iPhone replay; not screen capture | source time {speed:.2f}x", (28, 68), .52, MUTED)
        if self.last_image_index != index:
            image = cv2.imread(str(self.source / self.manifest["frames"][index]["path"]), cv2.IMREAD_GRAYSCALE)
            self.last_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            self.last_image_index = index
        canvas[102:582, 28:668] = self.last_image
        cv2.rectangle(canvas, (28, 102), (667, 581), (75, 80, 90), 1)
        text(canvas, "Prepared grayscale input | no logged feature coordinates", (28, 607), .48, MUTED)
        cv2.rectangle(canvas, (704, 100), (1250, 153), COLORS[status], -1)
        text(canvas, status, (720, 137), 1, (12, 19, 24), 2)
        text(canvas, f"Frame {index + 1}/{len(self.rows)} | source {row['timestamp'] - self.times[0]:.2f}s", (706, 180), .59)
        text(canvas, f"Matches {row['found']} | pose inliers {row['pose_inliers']} | {row['processing_ms']:.1f} ms", (706, 208), .57)
        text(canvas, f"Map {row['map_points']} points / {row['keyframes']} keyframes", (706, 235), .54, MUTED)
        cv2.rectangle(canvas, (706, 253), (1250, 568), (30, 35, 44), -1)
        # Full offline reference in gray; estimator fragments only accumulate to current exposure.
        if len(self.ref_pixels) > 1:
            cv2.polylines(canvas, [self.ref_pixels], False, (113, 118, 130), 1, cv2.LINE_AA)
        if self.estimate_pixels is not None:
            for j in range(1, index + 1):
                if self.states[j - 1] == self.states[j] == "GOOD":
                    cv2.line(canvas, tuple(self.estimate_pixels[j - 1]), tuple(self.estimate_pixels[j]), COLORS["GOOD"], 2, cv2.LINE_AA)
            if status == "GOOD":
                cv2.circle(canvas, tuple(self.estimate_pixels[index]), 5, COLORS["GOOD"], -1, cv2.LINE_AA)
        else:
            text(canvas, "Initialization failed", (795, 393), .72, COLORS["LOST"], 2)
            text(canvas, "No estimated trajectory", (790, 425), .61, FG)
        text(canvas, f"{''.join('XYZ'[i] for i in self.axes)} projection | full reference gray", (720, 277), .47, MUTED)
        text(canvas, "Green: GOOD fragments; gaps never joined", (720, 550), .45, COLORS["GOOD"])
        text(canvas, self.reference_label, (706, 593), .44, MUTED)
        count = self.evaluation["valid_output_frames"]
        rms = self.evaluation.get("position_rms_m")
        fit = f"offline Sim3 RMS {rms:.3f}m on GOOD subset" if rms is not None else "No Sim3 fit / no supported poses"
        text(canvas, f"GOOD {count}/{len(self.rows)} | {fit}", (28, 638), .53)
        text(canvas, "Full source-time coverage", (28, 663), .45, MUTED)
        x0, x1, y0, y1 = 235, 1248, 650, 668
        for j, s in enumerate(self.states):
            a = int(x0 + (self.times[j] - self.times[0]) / self.source_duration * (x1 - x0))
            end = self.times[j + 1] if j + 1 < len(self.times) else self.times[0] + self.source_duration
            b = int(x0 + (end - self.times[0]) / self.source_duration * (x1 - x0))
            cv2.rectangle(canvas, (a, y0), (max(a, b), y1), COLORS[s], -1)
        cursor = int(x0 + min((source_time - self.times[0]) / self.source_duration, 1) * (x1 - x0))
        cv2.line(canvas, (cursor, y0 - 4), (cursor, y1 + 4), FG, 2)
        for j, s in enumerate(COLORS):
            text(canvas, s, (28 + j * 182, 701), .43, COLORS[s])
        text(canvas, "Source-time visualization; not measured playback speed", (767, 701), .40, MUTED)
        return canvas, index


def encode(path, frames, fps):
    command = ["ffmpeg", "-hide_banner", "-loglevel", "warning", "-y", "-f", "rawvideo",
               "-pix_fmt", "bgr24", "-s", f"{W}x{H}", "-r", str(fps), "-i", "pipe:0",
               "-an", "-c:v", "libx264", "-threads", "2", "-preset", "veryfast", "-crf", "21",
               "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(path)]
    with open(path.with_suffix(".ffmpeg.log"), "w") as log:
        child = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=log)
        try:
            for frame in frames:
                child.stdin.write(frame.tobytes())
            child.stdin.close()
            if child.wait() != 0:
                raise RuntimeError(f"ffmpeg failed: {path}")
        except BaseException:
            child.kill()
            child.wait()
            raise
    probe = json.loads(subprocess.check_output(["ffprobe", "-v", "error", "-show_streams", "-show_format", "-of", "json", str(path)]))
    stream = probe["streams"][0]
    assert stream["codec_name"] == "h264" and stream["width"] == W and stream["height"] == H
    return {"path": str(path), "sha256": digest(path), "bytes": path.stat().st_size,
            "codec": stream["codec_name"], "pixel_format": stream["pix_fmt"], "fps": stream["avg_frame_rate"],
            "duration_seconds": float(probe["format"]["duration"]), "frames": int(stream["nb_frames"]),
            "ffmpeg_command": command}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--testing-root", type=Path, default=Path("testing"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fps", type=int, default=20)
    args = parser.parse_args()
    check_output_storage(args.output)
    args.output.mkdir(parents=True, exist_ok=False)
    write_probe = args.output / ".write-check"
    write_probe.write_bytes(b"ok")
    write_probe.unlink()
    cv2.setNumThreads(1)
    receipt = {"description": "Offline visualization of saved physical iPhone replay, not screen capture",
               "script_sha256": digest(__file__), "fps": args.fps, "opencv_threads": 1, "ffmpeg_threads": 2,
               "source_app_sha256": "e7fccf008a7647f4f8a6079ddbbe2e26787ad60c53165b5923638ee0e334166d",
               "argv": ["--testing-root", str(args.testing_root), "--output", str(args.output), "--fps", str(args.fps)],
               "cases": [], "outputs": [], "overview_sections": []}
    shutil.copy2(__file__, args.output / Path(__file__).name)
    replays, previews = [], []
    for case in CASES:
        replay = Replay(args.testing_root, case)
        assert replay.receipt["source_app_sha256"] == receipt["source_app_sha256"]
        replays.append(replay)
        duration = min(30., math.ceil(replay.source_duration * args.fps) / args.fps)
        n = round(duration * args.fps)
        print(f"Rendering {replay.name}: {len(replay.rows)} source frames -> {duration:.2f}s", flush=True)
        # Include the final exposure explicitly despite source-time decimation.
        output = encode(args.output / f"{replay.name}.mp4", (replay.frame(duration if i == n - 1 else i / args.fps, duration)[0] for i in range(n)), args.fps)
        output["source_time_speed"] = replay.source_duration / duration
        assert abs(output["duration_seconds"] - duration) < 1 / args.fps
        assert output["duration_seconds"] <= 30.01
        receipt["outputs"].append(output)
        receipt["cases"].append(replay.receipt)
        chosen = [i for i, s in enumerate(replay.states) if s == "GOOD"]
        sample_index = chosen[len(chosen) // 2] if chosen else len(replay.rows) // 2
        sample_time = (replay.times[sample_index] - replay.times[0]) / replay.source_duration * duration + .00001
        sample, _ = replay.frame(sample_time, duration)
        cv2.imwrite(str(args.output / f"{replay.name}_sample.jpg"), sample)
        previews.append(cv2.resize(sample, (640, 360)))
        (args.output / "provenance.json").write_text(json.dumps(receipt, indent=2) + "\n")
    def overview():
        for replay in replays:
            for i in range(10 * args.fps):
                yield replay.frame(10 if i == 10 * args.fps - 1 else i / args.fps, 10, overview=True)[0]
    print("Rendering sequential overview: 6 x 10s", flush=True)
    receipt["outputs"].append(encode(args.output / "overview_60s.mp4", overview(), args.fps))
    for j, replay in enumerate(replays):
        receipt["overview_sections"].append({"case": replay.name, "output_start_seconds": j * 10,
            "output_duration_seconds": 10, "source_time_speed": replay.source_duration / 10})
    sheet = np.vstack([np.hstack(previews[i:i + 2]) for i in range(0, 6, 2)])
    cv2.imwrite(str(args.output / "contact_sheet.jpg"), sheet)
    receipt["versions"] = {"opencv": cv2.__version__, "numpy": np.__version__,
        "ffmpeg": subprocess.check_output(["ffmpeg", "-version"], text=True).splitlines()[0]}
    (args.output / "provenance.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(f"Complete: {args.output}", flush=True)


if __name__ == "__main__":
    main()
