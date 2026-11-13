# Official upstream PTAM comparison

This harness builds the official Oxford PTAM-GPL r114-derived source separately
from Aaron's iPTAM and its restoration. It does not import any iPTAM tracking,
initialization, solver, recovery, RNG, timestamp or quality fixes.

The official checkout stays pristine. `pins.json` fixes all four repository
revisions, with dependencies selected from before the January 2010 r114 release.
Upstream PTAM source is GPLv3 (`UPSTREAM-LICENSE`); TooN, LibCVD and GVars retain
their original copyright/license files and notices in the fetched trees.

## Reproduce on arm64 macOS

Use a fresh directory on a filesystem with native links and permissions:

```bash
python3 upstream/bootstrap.py --work-root ../ptam-upstream --build
```

The bootstrap fetches pinned official sources, copies this small harness and
generates `generated/`, `generated-deps/`, `patches/compatibility.patch`,
`patches/dependency-compatibility.patch`, `source-inventory.json` and
`dependency-inventory.json`. Every generated file is hashed. Existing official
checkouts with local changes or unexpected origins are rejected. Do not run a
bootstrap over a work tree while a host/iOS build uses it.

The compatibility changes are limited to:

- Disabled window/OpenGL linkage and calls, using `TrackFrame(gray, false)`.
  Original previous-frame assignment and all estimator branches remain intact.
- POSIX `usleep` declaration and modern C++ template declaration/lookup fixes
  in TooN. No matrix, optimizer or residual formulas are changed.
- LibCVD's existing scalar/noarch implementations, since x86 SSE is unavailable
  on arm64; configure headers select pthreads, TooN and LAPACK/Accelerate.
- GVars' official headless backend. Its unused shell command reports unsupported
  on iOS (where `system()` cannot compile); macOS behavior remains original.

## Host driver and scheduling

```bash
../ptam-upstream/build/original_ptam_replay \
  --frames /absolute/prepared/pinhole/dataset/frames.csv \
  --output /absolute/fresh/trajectory.csv \
  --fx 400 --fy 400 --cx 319.5 --cy 239.5 \
  --init-first 0 --init-second 18
```

Use the prepared dataset's actual calibration. CSV header is `timestamp,path`,
with paths relative to that CSV. Images must be binary 8-bit PGM of the requested
size. The default driver schedules every exposure at source time without
dropping frames; backlog is logged. `--as-fast` is a separate throughput profile
that may starve the original asynchronous mapper. `--max-frames N` limits a smoke
run. Place an external timeout/log/NaN watchdog around all trials: original
unbounded numerical failures are preserved, not fixed or silently restarted.

Original `rand()`/`random_shuffle`, asynchronous MapMaker, 5ms mapper sleep,
per-frame motion model and defaults remain unchanged. Timestamps only control
replay scheduling and logging; they do not enter the estimator. Thread scheduling
makes repeatability different from the restored serial implementation. Upstream
supports its original FOV model (w=0 is pinhole); it has no ideal equidistant mode.
Use common prepared pinhole pixels for a comparison, with no truth pose input.

## iOS adapter

Embed this work root as a CMake subdirectory, disable
`ORIGINAL_PTAM_BUILD_DRIVER` and `ORIGINAL_PTAM_BUILD_TOOLS`, and link only
`original_ptam_core`. Do not link `iptam_core` into that same executable: original
global Tracker/MapMaker/TooN symbols would collide. Separate app/executable
profiles share the recording controller, not estimator symbols.

`adapter/OriginalPTAM.h` exposes only standard C++ types and an opaque Engine:

```cpp
original_ptam::Camera camera;
// Set width, height, fx, fy, cx, cy and original FOV parameter w.
original_ptam::Engine engine(camera);
auto snapshot = engine.process(gray, stride, source_timestamp, initialize_press);
```

Set `initialize_press` on the selected first and second exposures, matching the
original spacebar behavior. Each replay uses a fresh process. `process()` returns
the original quality (BAD=0, DODGY=1, GOOD=2; -1 before a map), initialized/finite
flags, current pose and diagnostic counts. `found` is the current keyframe's
unique successful measurement count. `attempted` is zero when that measurement
set is empty, since original counters can be uninitialized before tracking;
zero therefore can mean unavailable. Map sizes are approximate asynchronous
snapshots, following the original status pattern.

`pose_support_diagnostics_available` and `solver_diagnostics_available` are false;
`pose_inliers=-1` is unavailable and must be omitted from JSON/CSV metrics.
An attempted after-frame geometric observer was removed because it introduced
additional reads of asynchronously modified map points. Upstream GOOD plus ten
matches is only raw-supported tracking, not the restored final-pose geometric
support criterion. Preserve original quality separately; never apply a new gate
inside the upstream tracker/keyframe policy.

`camera_from_world` is row-major 3x4; `world_from_camera` is translation followed
by quaternion xyzw. Reference/Sim3 evaluation happens offline. Frame processing
time includes tracker/export work but excludes background mapper CPU; it is not
directly comparable to serial tracker-plus-mapper latency.

## Initial smoke evidence

The 2026-11-13 Mac source-paced common-pinhole new-lounge smoke completed all100
frames in10.17s:32 raw-supported GOOD frames, no nonfinite outputs,207 initial
points, four final keyframes. It subsequently lost tracking. This is a build/run
check, not evidence of general stability. Full physical-phone comparisons and
the exact tested binary/source identities are recorded separately by the main
project under `docs/testing/` and on the external evidence volume.

The fresh bootstrap above was then executed in
`../ptam-upstream`. All51 generated PTAM files,
522 generated dependency files, both adapter files and the compatibility header
matched the tested port byte for byte. The resulting host executable also
matched SHA256
`7b99592a2187c6edba1d145e0adaa93130bf0976cbcf8c7f7021357228701266`.
Its 100-frame source-paced run completed in10.37s with33 raw-supported GOOD,
no nonfinite/nonrigid output,2.22cm fitted position RMS on that subset,
43.1% reference-motion coverage and a46-frame lost tail. The32/33 difference
between identical executables illustrates retained upstream asynchronous
nondeterminism. Reproduction receipt, bootstrap log, CSV and evaluation are in
`evidence/upstream_comparison/portable_core_reproduction_20261113`.

Evidence labels above describe archived validation artifacts. Raw machine logs
and binaries are not bundled; public aggregates and build hashes are in
[the results report](../docs/RESULTS.md).
