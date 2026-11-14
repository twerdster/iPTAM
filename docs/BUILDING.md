# Build, replay and test

The modern targets use CMake and C++14. The recorded benchmark was run on an
arm64 Mac and a physical iPhone 12; the non-Apple LAPACK build path is provided
but has not been validated to the same extent. The original iOS 4 project is
historical; use `ios/` for current development.

## Host build

Install CMake and a C++ compiler. macOS links Accelerate; other platforms need
LAPACK and pthreads.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j 4
ctest --test-dir build --output-on-failure
```

For the experimental mobile algorithms, use a separate build:

```sh
cmake -S . -B build-mobile -DCMAKE_BUILD_TYPE=Release -DIPTAM_MOBILE_PAPER=ON
cmake --build build-mobile -j 4
ctest --test-dir build-mobile --output-on-failure
```

The default build preserves the restored tracker. The mobile build defaults to
`auto`: image-only automatic second-view selection. `--mobile-profile paper`
selects the fixed-pair ablation; `diagnostics`, `serial`, `coarse` and `partial`
are development profiles. See [implementation notes](testing/mobile-implementation.md).

## iPhone app

Use macOS, Xcode with an iOS SDK, Developer Mode and your own signing account.
Minimum deployment is iOS 16. Choose a unique bundle prefix for your account.

```sh
IPTAM_DEVELOPMENT_TEAM=YOUR_TEAM_ID IPTAM_BUNDLE_PREFIX=org.example.iptam \
  bash scripts/build_ios.sh mobile
```

The app is `build-ios-mobile/Release-iphoneos/PTAMMobile.app`. Use `current` for
the restored app, or follow [the upstream guide](../upstream/README.md) to build
`upstream` separately. Each variant has its own bundle identifier.
`IPTAM_DEVELOPMENT_TEAM` and `IPTAM_BUNDLE_PREFIX` are configuration values,
not credentials stored in this repository.
For a build without device signing, use `IPTAM_UNSIGNED=1` instead of a team ID.

Open the app to see its recording library. Prepared recordings belong in the
app's Documents `replays/` directory. Selecting a recording starts playback;
Back stops and saves the run. Logs are in Documents `runs/`.
The mobile comparison app uses recordings only and does not request camera
permission. The restored app retains an explicit `--live` development mode;
recorded validation does not establish live-capture correctness.

## Prepare your data

Use calibrated binary grayscale PGM images and their actual exposure timestamps.
Rectify unsupported lens distortion first. Cropping, resizing and rotation must
be reflected in the camera intrinsics. See [dataset contracts](dataset-inventory.md).

Each prepared recording contains:

- `images/`: PGM frames, all with the calibrated size.
- `frames.csv`: `timestamp,path` in seconds, with relative image paths.
- `manifest.json`: dimensions, pixel intrinsics, timestamps and initial indices.
- Optional `reference.json`: evaluation poses kept separate from estimator input.

A minimal manifest looks like this; supply calibration for your own images:

```json
{
  "width": 640, "height": 480,
  "fx": 400, "fy": 400, "cx": 319.5, "cy": 239.5,
  "lens_model": "pinhole", "init_first": 0, "init_second": 1,
  "frames": [
    {"timestamp": 0.0, "path": "images/000000.pgm"},
    {"timestamp": 0.1, "path": "images/000001.pgm"}
  ]
}
```

Two images illustrate the format; sustained tracking needs a longer sequence.
Pure rotation, low texture, blur and large image gaps can prevent initialization.

## Replay and evaluate

```sh
build-mobile/iptam_replay --frames data/prepared/sample/frames.csv \
  --fx 400 --fy 400 --cx 319.5 --cy 239.5 --lens-model pinhole \
  --init-first 0 --init-second 18 --mobile-profile auto \
  --source-paced 1 --output results/sample/trajectory.csv
```

Create the fresh output parent first. Use the input's actual calibration and a
valid start/pair. Automatic mode uses the first index and selects the second from
images; fixed mode uses both. Host options are value pairs, including
`--source-paced 1`. The iOS harness uses the standalone `--source-paced` flag.

For a physical phone, list devices with `xcrun devicectl list devices`, select
one explicitly and use the matching app/bundle. Do not overlap trials:

```sh
python tools/run_phone_replay.py --dataset data/prepared/sample \
  --app build-ios-mobile/Release-iphoneos/PTAMMobile.app \
  --bundle org.example.iptam.PTAMMobile --mobile-profile auto \
  --device DEVICE_ID --source-paced --output results/phone-trial
```

An optional `--expected-udid` adds an identity check. The harness preserves
source clocks, verifies consumed pixel hashes, saves every output frame and
retains failure evidence. It stops only its own test process.

Python tools need the validation dependencies:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-validation.txt
.venv/bin/python -m unittest discover -s tools -p 'test_*.py' -v
.venv/bin/python tools/evaluate_trajectory.py \
  --manifest data/prepared/sample/manifest.json \
  --trajectory results/sample/trajectory.csv --output results/sample/metrics.json
```

A completed replay is not proof of tracking. Inspect accepted coverage, gaps,
finite geometry and accuracy together. Translation uses a fitted monocular
scale; reference poses must never initialize or guide the estimator.

## Sanitizers and storage

```sh
cmake -S . -B build-sanitize -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DIPTAM_MOBILE_PAPER=ON -DIPTAM_SANITIZE=ON
cmake --build build-sanitize -j 4
ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 \
  ctest --test-dir build-sanitize --output-on-failure
```

Apple arm64 needs a separate `leaks` check because ASan leak detection is
unavailable. Optimized sanitizer builds avoid the slower unoptimized fixture's
test timeout; this is not a performance comparison.

Keep source, dependencies and build trees on a filesystem with native links and
permissions. Large data and results can use external storage. Optional
`IPTAM_STORAGE_MOUNT` and `IPTAM_STORAGE_UUID` configure mount verification;
never write through an assumed, disconnected mount directory.

Raw images, signed apps and machine-specific logs are not distributed. Public
[aggregate results](RESULTS.md) identify archived measurements by hashes.
To recreate the README GIF from your saved evaluated replay, use
`tools/render_readme_tracking.py`; it renders existing measurements and does
not run a tracker or invent feature locations.

## Publication checks

The public snapshot separately passes 12 host Release suites, 32 Python tests
(including portable configuration contracts), and an unsigned mobile iOS build.
See [the check receipt](results/publication-checks.json). These package checks
do not replace the recorded tracking benchmark or establish live capture.
