"""Contract checks for rectification geometry and monocular evaluation."""
import csv
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from evaluate_trajectory import evaluate, umeyama
from prepare_datasets import pinhole_maps


class DatasetToolsTest(unittest.TestCase):
    def test_pinhole_pixel_centres_are_identity(self):
        k = [[400, 0, 319.5], [0, 400, 239.5], [0, 0, 1]]
        x, y = pinhole_maps(640, 480, 400, k, [0] * 4, "radtan")
        yy, xx = np.indices((480, 640))
        np.testing.assert_allclose(x, xx, atol=1e-5)
        np.testing.assert_allclose(y, yy, atol=1e-5)

    def test_equidistant_inverse_ray(self):
        k = [[253, 0, 319.5], [0, 254, 239.5], [0, 0, 1]]
        x, y = pinhole_maps(640, 480, 400, k, [0] * 4, "ideal_equidistant")
        for i, j in ((0, 0), (100, 200), (239, 319), (479, 639)):
            a, b = (x[i, j] - 319.5) / 253, (y[i, j] - 239.5) / 254
            radius = np.hypot(a, b)
            factor = np.tan(radius) / radius
            np.testing.assert_allclose([a * factor, b * factor],
                                       [(j - 319.5) / 400, (i - 239.5) / 400], atol=3e-7)

    def test_similarity_and_coverage(self):
        points = np.array([[0, 0, 0], [1, 0, 0], [1, 1, .2], [0, 1, .4], [-1, .2, .1]])
        r = Rotation.from_euler("xyz", [13, -21, 37], degrees=True).as_matrix()
        scale, translation = 2.75, np.array([5, -2, .4])
        truth_positions = scale * points @ r.T + translation
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "manifest.json").write_text(json.dumps(dict(
                name="gauge_contract", frames=[dict(timestamp=float(i), path="unused") for i in range(5)])))
            poses = []
            for i, p in enumerate(truth_positions):
                m = np.eye(4); m[:3, :3] = r; m[:3, 3] = p
                poses.append(dict(timestamp=float(i), matrix=m.tolist()))
            (root / "reference.json").write_text(json.dumps(dict(role="simulation truth", poses=poses)))
            keys = ["timestamp", "tx", "ty", "tz"] + [f"r{i}{j}" for i in range(3) for j in range(3)] + ["quality", "points", "found"]
            with (root / "trajectory.csv").open("w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=keys); writer.writeheader()
                for i, p in enumerate(points):
                    row = dict(timestamp=i, tx=p[0], ty=p[1], tz=p[2], quality=0 if i == 2 else 2, points=100, found=80)
                    row.update({f"r{a}{b}": float(a == b) for a in range(3) for b in range(3)})
                    writer.writerow(row)
            report = evaluate(root / "manifest.json", root / "trajectory.csv")
            self.assertIsNone(report["failure"])
            self.assertAlmostEqual(report["sim3"]["scale"], scale)
            self.assertLess(report["position_rms_m"], 1e-12)
            self.assertLess(report["orientation_max_deg"], 1e-10)
            self.assertEqual(report["valid_output_coverage"], .8)
            self.assertEqual(report["missing_after_initialization"], 1)
            # Numerically broken BAD output must be counted, while weak or
            # low-match GOOD states must not inflate strict visual coverage.
            with (root / "trajectory.csv").open() as stream:
                rows = list(csv.DictReader(stream))
            rows[2]["tx"] = "nan"
            rows[3]["quality"] = "1"
            rows[4]["found"] = "9"
            with (root / "trajectory.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=keys)
                writer.writeheader(); writer.writerows(rows)
            report = evaluate(root / "manifest.json", root / "trajectory.csv")
            self.assertEqual(report["nonfinite_rows_all_states"], 1)
            self.assertEqual(report["valid_output_frames"], 2)
            self.assertEqual(report["weak_visual_frames"], 1)
            self.assertIsNotNone(report["failure"])
            # Legacy files above remain readable. New diagnostics must reject
            # nominal GOOD/weak poses with insufficient geometric support.
            keys.append("pose_inliers")
            for row in rows:
                row["pose_inliers"] = "10"
            rows[0]["pose_inliers"] = "9"
            rows[3]["pose_inliers"] = "0"
            with (root / "trajectory.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=keys)
                writer.writeheader(); writer.writerows(rows)
            report = evaluate(root / "manifest.json", root / "trajectory.csv")
            self.assertEqual(report["valid_output_frames"], 1)
            self.assertEqual(report["weak_visual_frames"], 0)
            self.assertTrue(report["pose_inlier_diagnostics_available"])

    def test_stationary_path_does_not_produce_accuracy(self):
        with self.assertRaises(ValueError):
            umeyama(np.zeros((5, 3)), np.ones((5, 3)))


if __name__ == "__main__":
    unittest.main()
