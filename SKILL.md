# Development workflow

Read `SUMMARY.md` first, then `docs/BUILDING.md` and the relevant report.
`README.md` stays friendly and concise; keep detailed evidence in `docs/`.
Use generic relative examples and configurable device/storage/signing values.
Do not commit recordings, device identifiers, local paths or signed apps.

## Build and checks

```sh
cmake -S . -B build-mobile -DCMAKE_BUILD_TYPE=Release -DIPTAM_MOBILE_PAPER=ON
cmake --build build-mobile -j 4
ctest --test-dir build-mobile --output-on-failure
python -m unittest discover -s tools -p 'test_*.py' -v
```

Use a virtual environment with `requirements-validation.txt`. For the default
restored core omit `IPTAM_MOBILE_PAPER`; keep its checks separate. Use
RelWithDebInfo for mobile ASan/UBSan tests, and Apple's `leaks` separately on
arm64 macOS. Full commands and iOS signing configuration: `docs/BUILDING.md`.
Source/build trees need native filesystem semantics; validate configured mounted
storage before large output writes.

## Estimator and replay contracts

All supported tracker/map state belongs to one serial worker. Call
`SetFrameTimestamp` before tracking; preserve source clocks and pixel intrinsics.
Images, resolution and lens model must agree. GOOD needs a nonempty initialized
map, at least ten raw matches, ten final-pose robust contributors, quality 2 and
finite rigid geometry. Keep weak/lost states separate. Original upstream support
is unavailable, not zero, and its asynchronous mapper remains unchanged.

References are evaluation-only. Do not select an initialization window, variant
or repeat by its reference error. `auto` initialization and explicit fixed-pair
`paper` are different policies. Complete-system comparisons require explicit
`--automatic-system-comparison`/`--system-comparison`; automatic runs cannot enter
fixed-pair tracking isolation. Report whole-input coverage, gaps, error, memory,
lag and thermal state together. Upstream tracker latency excludes background
mapper CPU; modern serial/idle costs have different scopes.

Use fresh output directories and sequential physical trials with an explicit
`--device`; optional `--expected-udid` verifies identity. Never blindly relaunch
failures. `STOP_AFTER_TRIAL` requests a graceful stop between suite trials.
Timed mapping needs actual repetitions. `--deterministic-mapping` disables its
wall-time scheduling for exact host algorithm checks. A four-ms cooperative
budget is not a hard deadline. Unknown readout timing keeps rolling shutter off.

## Public results and media

`docs/RESULTS.md` and `docs/results/` are sanitized benchmark summaries. Benchmark
and subsequent error-receipt builds have different hashes; do not pool outputs.
Detailed historical reports keep their own profile labels. Raw local evidence
is not redistributed; retain hashes without computer-specific paths.

`tools/render_readme_tracking.py` renders saved evaluated phone logs into the
README GIF. Preserve all source exposures and weak/lost gaps, verify source
pixel hashes and label offline fits. Do not invent feature overlays or imply
screen capture/phone FPS. Check every GIF frame decodes, inspect samples and
require file size below 2,000,000 bytes. Its hash-only public receipt belongs
beside the GIF. No source paths or device IDs enter media metadata.
After public-facing changes, check relative documentation links, scan text and
image metadata for identifying paths, and validate the portable configuration
tests. Publication-only checks go in `docs/results/publication-checks.json`;
keep their counts separate from the archived benchmark tests.


Before every commit, review `SUMMARY.md` and this file; explicitly check and
update each for behavior, workflow, command, output or status changes. Preserve
all PTAM, LibCVD and TooN notices. The original Xcode project is historical.
