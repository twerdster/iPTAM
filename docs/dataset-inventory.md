# Dataset and calibration contracts

The benchmark uses five full prepared inputs. Its
[input catalog](results/frozen-input-plan.json) records frame counts, calibration,
frozen initialization pairs and concatenated pixel hashes. It is a sanitized
catalog, not an executable plan or a download of the underlying recordings.
Raw private recordings are not redistributed.

## Inputs

| Input | Images | Timing | Reference |
| --- | ---: | --- | --- |
| Historical simulated lounge | 230 | About 10 Hz / 22.9 s | Simulation truth |
| New simulated lounge | 100 | 10 Hz / 9.9 s between exposures | Simulation truth |
| EuRoC MH01 left | 3,682 | 20 Hz / 184.05 s | Leica/IMU batch reference |
| EuRoC MH01 right | 3,682 | Same exposure clock | Same body reference with camera extrinsic |
| Sparse real lounge | 142 | 0.75–4.34 s image gaps / 131.9 s | ARKit reference with uncertainty |

Both simulated lounges use photo-fitted room geometry; neither is an actual
RGB recording of a scan. EuRoC cameras are tested independently by a monocular
estimator, but share one physical environment. The sparse real export is not a
continuous video. No reference pose, depth, mesh or IMU enters tracking.

## Prepared recording format

A recording directory has `manifest.json`, binary grayscale PGM images and
`frames.csv` (`timestamp,path`, seconds, relative image filenames). All image
sizes must match the calibration. Optional `reference.json` stores evaluation
poses separately. See [the example manifest](BUILDING.md#prepare-your-data).
The preparation adapters write new directories and preserve image-time gaps.

All common-comparison images are 640×480 pinhole with principal point 319.5,239.5.
Focal lengths are 400 px for both simulations and 460 px for EuRoC/sparse lounge.
These are prepared-image calibrations, not camera parameters to reuse on arbitrary
recordings.

The native simulated source uses ideal equidistant projection `r=theta`, with
`[fx,fy,cx,cy]=[253.30344967,253.34336776,319.5,239.5]`. The core supports this
explicitly as `equidistant`. Exact native tests preserve all pixels and write a
separate domain mask; their results are separate from the common pinhole table.

EuRoC's original radial/tangential distortion is rectified consistently with
its supplied intrinsics. Cropping, resizing or rotation must update calibration.
The pinhole output focal was chosen for valid source sampling without
feature-bearing black borders. This is a cropped monocular comparison, not a
stereo or visual-inertial leaderboard evaluation.

## Preparation helpers

`tools/prepare_datasets.py` adapts the benchmark's simulation and previously
ingested EuRoC layouts. Supply their roots explicitly with `--viobench` and
`--replay`; it does not download those projects or infer arbitrary folder layouts.
For a new recording, preparing the documented PGM/CSV/manifest format directly
is often simpler. Supply measured intrinsics and actual exposure times.

The sparse-lounge adapter accepts an explicit source path:

```sh
python tools/prepare_actual_lounge.py --source data/raw/LiteReality-export \
  --output data/prepared/actual_lounge --init-first 47 --init-second 48
```

It targets a specific export format; this command is an example for compatible
recordings, not a bundled dataset. Per-exposure calibration and the RGB/metadata
alignment must agree. Provenance records transformations and pixel hashes.
Raw timestamps remain intact apart from an explicit origin shift. Never replace
a sparse source clock with a nominal video frame rate or invent missing images.

## Evaluation and reproducibility

Transforms are world-from-optical-camera, with +X right,+Y down,+Z forward.
Render-camera poses receive `diag(1,-1,-1,1)` exactly once. EuRoC body poses are
interpolated to exposure times and composed with the calibrated camera
extrinsic. No extrapolation is used outside the reference interval.

GOOD uses quality 2, initialized map points, at least ten raw matches, ten
final-pose robust contributors when available, and finite rigid geometry.
Original upstream support diagnostics remain unknown. Weak/lost states and
uninitialized images stay in whole-input denominators.

Monocular metric scale is unobservable. Evaluation fits one proper-rotation
Sim3 to accepted, reference-associated poses, reports its scale and error, and
also reports tracking gaps and reference-motion coverage. Adjacent accepted
motion does not join lost intervals. A stationary or degenerate alignment cannot
provide a meaningful accuracy score. Reference poses must never select a window,
initial hypothesis or preferred repeat.

For new datasets, freeze a plan before replay and record its hash plus manifest,
image and reference hashes. `run_mobile_validation.py` supports an optional
`--expected-plan-sha256`; it always verifies each frozen case's manifests and
pixels before estimation. The comparison helpers require explicit plan paths
when using a different input catalog. Keep automatic and fixed-pair policies
separate, and report all failed trials.
