# Portable original PTAM core

> Detailed validation record. See [current results](RESULTS.md) for the main comparison.
> `evidence/` labels identify local validation artifacts; raw logs, app binaries
> and private inputs are not redistributed with this repository.

The modern host and iOS builds compile the original tracker, map maker, homography initializer, patch finder, bundle adjuster, LibCVD FAST/pyramid code and TooN math. The stable integration runs all of these on one serial worker. Legacy OpenGL drawing is disabled with `IPTAM_HEADLESS=1`; iOS camera preview/UI is separate.

## Build and regression tests

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j 6
ctest --test-dir build --output-on-failure

cmake -S . -B build-sanitize -DCMAKE_BUILD_TYPE=RelWithDebInfo -DIPTAM_SANITIZE=ON
cmake --build build-sanitize -j 6
ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 ctest --test-dir build-sanitize --output-on-failure
```

Apple arm64 AddressSanitizer does not support its leak detector. `detect_leaks=0` disables that detector only; AddressSanitizer and UndefinedBehaviorSanitizer remain enabled. A separate macOS `MallocStackLogging=1 leaks --atExit -- ./build-experiment/iptam_core_tests` check on the default-transaction ownership code with mandatory mapper cleanup, before the minimum-support-only predicate/test addition, reported zero leaks and zero leaked bytes over the maintenance regressions and three initialized/reset/destruct lifecycles, with peak physical footprint reported as 37.1M. The receipt `core-mapper-cleanup-leaks-2026-11-12.txt` records the local log identity and hash; this is a host regression observation, not an on-device or indefinite-session memory bound. Earlier control receipts remain in `core-transaction-leaks-2026-11-12.txt` and `core-final-leaks-2026-11-12.txt`.

The current five-test release suite passed in 4.95 seconds; the corresponding AddressSanitizer/UndefinedBehaviorSanitizer suite passed in 29.80 seconds. Core, patch finder, solver and camera targets are instrumented; the Foundation manifest parser target is a separate host test. Exact logs and binary hash are in `core-validation-2026-11-12.txt`.

`core_regressions` first verifies that a pending recent bundle stage still retires a bad active point and erases its keyframe measurement, then proves that a queued refind cannot reinsert a retired point after a real textured patch matched successfully. It also exercises calibrated camera round trips, invalid calibration and frame dimensions, duplicate capture timestamps, empty homography input, zero-residual robust estimation, blank-frame initialization failure, actual textured-plane initialization and 39/39 subsequent tracked frames, eight blank frames of occlusion, return/relocalization with visual matches, and three repeated complete object lifecycles with identical poses/match counts. High-fraction support on one, three or nine matches must be bad and advance consecutive loss; ten matches with adequate ratio resume good tracking. A real rejected WLS update makes its frame bad, then a later valid textured frame and reset clear that failure. `bundle_solver_regressions` exercises finite-coordinate residual overflow, singular camera systems with bounded damping retries, invalid measurement variances/points, failed-trial rollback, a recorded iPhone Schur matrix, and a multi-depth/outlier fixture that retains accepted optimization after a late failed trial. The finite `1e200` image-residual fixture reproduces an infinite outer loop in the previous frozen archive. `replay_manifest_regressions` validates the iOS Foundation manifest parser on macOS. `patchfinder_regressions` covers cached-view recovery, singular/nonfinite warps, and the actual tracker search boundary rejecting a point invalidated by pose refinement or a nonfinite linear update before integer pixel conversion. `equidistant_camera_regressions` checks original FOV/pinhole behavior, exact equidistant projection/inversion/derivatives, copies/resizing, and invalid projection domains.

CMake target `iptam_core` exports its include paths, C++14 requirements, headless definitions and Accelerate linkage. `IPTAM_BUILD_TOOLS` and `IPTAM_BUILD_TESTS` can be disabled by an iOS embedding project. On non-Apple hosts CMake looks for LAPACK and pthreads; this path has not been validated in this session.

## Capture and calibration API

Create one `ATANCamera`, `Map`, `MapMaker(map, camera)` and `Tracker(imageSize, camera, map, mapper)` on the serial worker, with lifetimes in that order. `MapMaker` defaults to synchronous maintenance, called automatically at the end of `Tracker::TrackFrame`. The optional historical threaded mode is outside the stable/tested integration. Each serial iteration selects a mapping stage with if/else and always performs bad-point retirement afterward, matching the historical loop's cleanup requirement. A bounded nonconverged bundle stage cannot starve that maintenance, and queued refinds reject retired points.

For an already rectified pinhole image of width W and height H, set the camera's five parameters to:

```
fx/W, fy/H, (cx+0.5)/W, (cy+0.5)/H, 0
```

The half-pixel term is required by the original camera model. The default `ATANCamera::ProjectionModel::FOV` retains the original FOV model; its fifth parameter zero gives pinhole projection. A calibrated original FOV lens can supply its normalized distortion parameter.

For an ideal equidistant source, call `camera.SetProjectionModel(ATANCamera::ProjectionModel::Equidistant)` before setting the normalized parameters above, keeping the fifth parameter zero. This uses the original pixels and matching source intrinsics, with `r = theta` in normalized image coordinates. Rays at or beyond the forward hemisphere (`theta >= pi/2 - 1e-6`) are explicitly invalid. The lounge's native 640x480 calibration has exactly 16 invalid corner pixels. Camera copies retain the selected model: select it before constructing the mapper and tracker, which copy the camera. Resizing also retains the model but must match any corresponding intrinsics change.

Radtan and other lens models require appropriate calibrated rectification unless they exactly match one of these supported models. Do not label arbitrary distorted input equidistant. Image orientation, resolution, pixel buffer stride and calibration must describe the same pixels. The live iPhone camera uses its pinhole calibration; native equidistant replay is an explicit dataset choice.

Call `SetFrameTimestamp(captureSeconds)` immediately before `TrackFrame(grayImage, 0, false)`. Timestamps must be finite and increasing within a session; negative relative timestamps are accepted. The motion prior scales with the current/previous frame interval, with ratios bounded to 0.25–4; a gap over 0.5 seconds clears that prior. With timestamps, new-keyframe spacing is at least 0.3 seconds and three frames, in addition to the original geometric keyframe criterion. Without timestamps the historical frame-count spacing applies.

`StartTracking()` requests the first initialization frame, then the second frame on a later call. Initialization still needs textured scene overlap and sideways translation; fast movement, pure rotation, blur and weak parallax are not reliably handled. A failed second-frame initialization retains the first frame and remaining trails for another manual attempt. `InitialTrailCount()` and `InitialMedianDisplacement()` support live guidance. `StopTracking()` resets the session/map.

`TrackingQuality()` returns -1 while uninitialized, 0 for bad tracking, 1 for poor tracking, and 2 for good tracking. During map tracking, quality assessment requires at least ten found matches before applying the original match-fraction heuristics. This aligns the internal lost-state and GOOD-keyframe eligibility decisions with the existing Engine/evaluator validity policy. Successful pose reporting must also require actual visual matches, finite rigid poses and an initialized map; the final evaluation/phone runner requires quality 2 and at least ten found measurements. Map creation or a finite motion-prior pose alone is insufficient. `GetCurrentPose()` returns camera-from-world in the original PTAM coordinate system; invert it for world-from-camera. Monocular scale and global gauge are arbitrary.

## Headless replay

Input `frames.csv` has the header `timestamp,path`, timestamps in seconds, and unquoted paths relative to that CSV directory or absolute paths. Frames are binary 8-bit PGM (`P5`, maximum value 255). All frames must have the calibrated dimensions.

```sh
./build/iptam_replay \
  --frames data/prepared/dataset/frames.csv \
  --fx 400 --fy 400 --cx 319.5 --cy 239.5 \
  --init-first 0 --init-second 16 \
  --output evidence/fresh-run/trajectory.csv
```

Optional `--lens-model pinhole|equidistant` selects the camera model; the default is pinhole. For native lounge replay use `--lens-model equidistant` with the original focal lengths (approximately 253.30344967 and 253.34336776 pixels), not the rectified 400-pixel focal lengths in the example. Standard output prints the camera's actual selected `lens_model` for audit. Optional `--max-frames N` limits replay to the first N manifest rows. Initialization indices are absolute row indices. Exit 0 means a map was created; exit 2 means initialization never succeeded; exit 1 indicates invalid input or a runtime error. Evaluate coverage and trajectory quality separately before judging a run successful.

Output CSV exports `timestamp,tx,ty,tz,r00,...,r22,quality,found,attempted,points,keyframes,elapsed_ms,visible,warp_rejected,template_rejected,initial_trails,solver_failures,solver_retained_stops`. Translation and rotation are **world-from-CV-camera**. The diagnostic counts help distinguish projection/warp rejection from patch mismatch. Each row is flushed so termination preserves progress. `solver_failures` counts hard rejected bundle solves; `solver_retained_stops` counts bounded late trial failures that retain usable previously accepted optimization. Both are cumulative and distinct from tracking loss and nonfinite pose counts. The stdout `good_frames` count requires quality 2, at least ten found measurements, and map points. Failed/lost rows remain in the file and must remain in coverage denominators.

## Repeatability and fixes

Tracker, map maker and homography RANSAC each use an explicit `std::mt19937` seed of 42, reset with their session. Bundle camera/point/measurement inputs and nearest-keyframe ties use stable insertion order. Pointer-ordered numerical accumulation previously made identical inputs produce different map gauges, tracking decisions and runtimes in fresh processes. Repeated host comparison must ignore `elapsed_ms` while comparing all pose/state/count fields. Cross-device LAPACK/libc++ floating-point differences may still require numerical tolerances; measure device results directly.

The first complete right-camera EuRoC evaluation exposed a nonfinite-update loop in the historical bundle adjuster; failure logs and matrix samples remain under `evidence/host/final_20261112_frozen/euroc_cam1/repeat_01`. Finite current/trial objectives, normal systems, inverses and updates are now checked; LDL pivots must be finite and positive, damping retries and the outer solver loop have independent bounds. A failed trial does not apply its trial copy. If late singular trials exhaust the trial budget after useful finite cost-decreasing steps, the solver retains the committed accepted geometry and prior committed outlier list, reports a bounded stop, and does not claim convergence. Invalid inputs and unsupported/no-accepted-step systems still fail. An existing valid map is retained after a hard solve failure, and that same system waits for a new keyframe before another solve; an invalid bootstrap is rejected.

The default `Bundle` minimum damping parameter is zero, retaining the original adaptive LM policy together with the new transactional safeguards. A real optimization regression exercises this actual default constructor and verifies retention after a bounded late singular trial. An explicit `Bundle(camera, 1e-6)` profile remains available for controlled experiments: it prevents the observed near-scale-gauge Schur cancellation on the fixture, but full-sequence EuRoC coverage regressed, so it is not the shipping default. Fewer retained-stop events alone do not justify accepting that coverage loss. No positive-definiteness or finite-cost acceptance check is bypassed. `MapMaker::NumericalFailureCount()`, `RetainedSolveCount()` and `LastNumericalFailure()` expose the distinct events and reason. See `solver-validation-2026-11-12.txt`. These checks do not establish accurate or uninterrupted tracking; complete sequence evaluation must still report coverage, accuracy and rejection counts.

Repairs include modern TooN template lookup, explicit calibration, initialized/reset poses and flags, correct projection derivatives and image bounds, zero-residual robust scales, finite geometry/warp/solve checks, singular image-alignment rejection, initialization failure state, recovery, physical ownership cleanup, the accidentally drawing-dependent previous trail frame, stale template rejection after a valid view returns, rejected warp handling, failed subpixel refind rejection, and pixel footprint refresh after accepted bundle adjustments. Mapping has a 5,000-point live-map cap and 300 successful insertions per pyramid level per insertion pass. These caps do not bound keyframe or trash storage: CLI sessions have no hard keyframe limit. The iOS live shell saves/stops at 96 keyframes or 10,000 trash points and on a memory warning. The original PTAM estimator and coordinate conventions remain in use; the rejected experimental weak-tracking keyframe gate is not integrated.

Dataset inventory, repeated full-sequence results and physical iPhone evidence are recorded separately. Passing synthetic regressions or a successful build does not establish stable tracking across a complete room scan or EuRoC run.
