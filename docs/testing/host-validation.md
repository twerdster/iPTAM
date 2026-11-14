# Current replay validation

> Detailed validation record. See [current results](../RESULTS.md) for the main comparison.
> `evidence/` labels identify local validation artifacts; raw logs, app binaries
> and private inputs are not redistributed with this repository.

The current comparison is summarized in [the results report](../RESULTS.md). The tables below
retain the earlier frozen minimum-support profile for comparison.

Exact equidistant optics restore continuous post-initialization tracking of the
new synthetic lounge on the physical iPhone12: 81/81 exposures, 6.15mm fitted-scale
position RMS, repeated exactly three times. The actual lounge scan does not
initialize, and the historical lounge and real EuRoC sequences still lose long
sections. The requested general tracking stability has not been achieved.

Scope clarified 13 November 2026: continue using existing recordings and
datasets only. Live capture, camera authorization and a new handheld session
are out of scope and do not block replay development or evaluation.

The frozen profile below restores mandatory bad-point cleanup in the serial
mapper, prevents retired points from being reinserted, and requires at least 10
actual matches for internal GOOD classification. This matches the existing app
and evaluator rule and prevents a few matches from seeding a new keyframe. It
keeps bounded, transactional solver safeguards with the original adaptive damping default
(minimum 0). The fixed 1e-6 floor is a [rejected control](damping-floor-control.md).
No weak-keyframe gate, relaxed bootstrap or ground-truth pose input is enabled.
The host and phone binaries below identify the exact tested implementations.

## Physical iPhone results

Target: physical iPhone 12 (iPhone13,2), iOS 18.0.1. App executable SHA256:
`dc0e3d2b551468e4e2013163c9e0873295a5c9d334a805be6d2c11b02a6c8d74`.
App/source/build provenance is under `testing/builds/dc0e3d2b551468e4` in the local evidence archive.
These are actual recorded-input runs on the physical phone, not live handheld
accuracy measurements. Detailed device checks belong in [phone-validation.md](phone-validation.md).

Each evidence directory under `evidence/phone/`
contains input/time/model/pixel checks, app provenance, complete trajectory rows
and `evaluation.json`.

| Evidence ID | Model; init | GOOD / input | Weak | Fitted position RMS | Reference motion covered | Largest gap | Hard / retained stops | Mean / max processing |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `validated_support_lounge_new_01` (also `_02`, `_03`) | equidistant; 0/18 | 81/100 | 0 | 0.615 cm | 95.62% | 0 | 0 / 3 | 7.55–8.38 / 40.71 ms |
| `validated_support_lounge_reference_01` | equidistant; 0/16 | 89/230 | 8 | 0.533 cm | 33.40% | 124 frames | 0 / 4 | 8.66 / 95.71 ms |
| `validated_support_euroc_cam0_01` | pinhole; 1200/1205 | 914/3682 | 276 | 5.770 cm | 28.89% | 535 frames | 0 / 14 | 15.09 / 779.16 ms |
| `validated_support_euroc_cam1_01` (also `_02`, `_03`) | pinhole; 0/5 | 262/3682 | 91 | 10.009 cm | 3.95% | 2574 frames | 0 / 1 | 3.03–3.07 / 65.68 ms |
| `validated_support_actual_lounge_01` | pinhole approximation; 47/48 | 0/142 | 0 | no initialized trajectory | 0% | never initialized | 0 / 0 | 1.63 / 3.56 ms |

All three new-lounge phone runs have identical serialized pose/state fields after
excluding processing time, SHA256
`e168dfe8ae18585eaa2aa70d24886aaf46e9814beafd1acf7e1bec5d9b1d62d9`.
They cover 95.62% of the full reference path; omitted motion precedes initialization.
The three cam1 phone runs are also identical, normalized-state SHA256
`38584fca6b44640e72568ef97c6c38428a532daff4873e1344019b84387eaab8`.
All completed rows have finite rigid poses, but predicted or uninitialized poses
do not count as tracking. The textured-plane control loses exactly its eight
blank exposures 60–67 and recovers at 68, with no numerical rejection.

Cam0 starts deliberately near 60 s at an image-supported pair. All 3,682 original
exposures remain in the denominator. Of cam1's 262 GOOD outputs,260 fall within
the reference interval and determine its fitted RMS. Cam1 has poor motion coverage;
its low average runtime mainly reflects lost tracking. Processing averages include
all states, not only successful frames. Thermal state/maximum and low-power mode
are recorded in current phone summaries; device timing is not a controlled thermal
benchmark. No general real-time guarantee follows from these means.

An earlier `restored_cleanup_actual_lounge_01` trial collided with a lifecycle
test and was explicitly invalidated, excluded and rerun sequentially. The final
`validated_support_*` trials use exclusive device locking. Both phone tools now share
a per-device lock with a real two-process regression. The valid run verifies the
input pipeline and pixel hashes but never initializes PTAM. See
[actual-lounge-data.md](actual-lounge-data.md) for its source, sampling and audit.

## Host repetitions

Frozen executable:
`a319cbc477f734d08cf4fbf1690b2ff6bb2673adaba7881dd9161dc9f7717654`.
Full suites are `evidence/host/final_support_20261112`
and `final_support_actual_lounge_20261112`. All five inputs completed three
full repetitions: 23,508 total original exposure rows. Every dataset's normalized
pose/state fields are exactly equal across its three runs. No saved transform
was nonfinite or nonrigid and the legacy NaN-loop watchdog never fired.

| Input; init | GOOD / input, each of 3 | Weak | Fitted position RMS | Reference motion covered | Largest gap | Hard / retained | Peak RSS | Worst frame |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Historical native lounge; 0/16 | 89/230 | 10 | 0.565 cm | 33.40% | 124 | 0 / 4 | 57.8 MiB | 71.12 ms |
| New native lounge; 0/18 | 81/100 | 0 | 0.517 cm | 95.62% | 0 | 0 / 4 | 45.0 MiB | 30.20 ms |
| EuRoC cam0; 1200/1205 | 597/3682 | 167 | 4.526 cm | 21.81% | 892 | 0 / 11 | 215.3 MiB | 382.84 ms |
| EuRoC cam1; 0/5 | 237/3682 | 79 | 8.762 cm | 3.42% | 2593 | 1 / 1 | 65.5 MiB | 30.32 ms |
| Actual lounge; 47/48 | 0/142 | 0 | no initialized trajectory | 0% | never initialized | 0 / 0 | 12.4 MiB | 2.82 ms |

The minimum-support guard prevents the severe cam1 drift seen in the
[cleanup-only control](cleanup-controls.md): the valid-subset RMS changes from
2.96 m to 8.76 cm, while coverage remains poor. This is a changed trajectory,
not merely removal of low-match rows from scoring; the evaluator already required
10 matches. Phone and host floating-point/map decisions differ, so their results
are reported separately and neither replaces the other. Exact repetition establishes
reproducibility, not accurate or continuous tracking.

![All host trajectories and gaps](host-final.png)

## Inputs and scoring

The two synthetic lounge motions use photo-fitted Blender geometry. Authoritative
lossless native images are in `testing/native_datasets_exact/`, original K
`[253.30344967,253.34336776,319.5,239.5]`, 640×480, `lens_model=equidistant`.
Every source grayscale pixel is preserved, including 16 back-hemisphere corners;
a separate domain mask marks those corners and the camera rejects them.
Retained pinhole controls have different angular coverage.

EuRoC cam0 and cam1 are two monocular views of the same real MH_01_easy flight,
rectified consistently to K 460. The separate real ARKit lounge export supplies 142 saved images over 131.9 s, with source
intervals 0.75–4.34 s, rather than dense RGB video. Per-exposure intrinsics are
normalized, and ARKit output is an evaluation reference only. No pose, IMU,
point cloud, depth or mesh initializes or feeds the PTAM estimator.

Strict GOOD means quality2, at least 10 actual image matches, nonempty map and a
finite rigid transform. Weak quality1 is counted separately. One proper Sim3 fit
resolves monocular scale using reference-associated strict poses. RMS concerns
that valid subset; it does not penalize absent output and must be read with
coverage. Endpoint is the last scored exposure, possibly before the source end.
Motion coverage sums reference path length only between adjacent strict-GOOD
original exposures, never across lost segments. ARKit disagreement is not
independent metrology, and partial EuRoC scores are not VIO leaderboard results.

The original four conversions pass a separate actual-image epipolar audit near
0.17px. The real lounge has 0.634 px median across 51 usable adjacent pairs versus
35.325 px with deliberately wrong ARKit axes. Lens selection is explicit and the
harness checks the runtime model. See [dataset-inventory.md](../dataset-inventory.md)
for source paths, clocks, transforms and preparation commands.

## Retained failures and comparison controls

The historical native lounge lost interval has ample image features, so low
texture alone does not explain its failure. An isolated weak-keyframe experiment
improved that source but caused severe EuRoC regressions and is not integrated.
Actual-lounge image-only candidate selection preserves all inspected pairs;
47/48 meets the fixed geometry/followup rules but fails PTAM initialization in
all three complete host runs and on the phone. No missing images are synthesized
from ARKit poses.

The original serial port accidentally skipped required bad-point cleanup whenever
it selected a nonidle mapper stage. Restoring original-loop cleanup is a concrete
correctness fix. The prior default0 starvation control processed cam0 fully once
in 654.9 s with 984 GOOD; remaining slow repetitions were explicitly stopped as
superseded, and its suite is **incomplete**, not 3/3. The corrected runs take 26 s
for cam0 with 597 GOOD, so runtime and coverage both changed. Raw evidence remains
under `testing/host/final_transaction_default_20261112/superseded.json`.

The fixed 1e-6 floor control has its complete 12 host trials and [separate report](damping-floor-control.md).
Its phone cam0 trial reached the declared 900 s deadline with partial output and
must not be counted as full completion. Earlier transaction-only phone artifacts
are retained in [transaction-controls.md](transaction-controls.md).

An older binary entered an unbounded NaN retry loop on cam1. Its 489 MB raw log,
matrices and manual-stop record remain in `testing/host/final_20261112_frozen`.
Current iteration/finite guards reject invalid trials and retain accepted finite
improvements. Rejections and retained stops are reported separately from saved
pose validity. The watchdog stops legacy loops and preserves evidence before
another trial. The first full baseline and all its 12 runs remain in
[archive-baseline.md](archive-baseline.md).

## Reproduction

Use the local Python3.11 environment pinned in `requirements-validation.txt`.
Release and ASan/UBSan core suites pass 5/5 for this profile. Eleven Python tests
pass, including real child-process failure/watchdog/model checks, the per-device
lock, optical serialization and trajectory scoring. Original source recordings
remain unchanged; validation helpers check writable output storage before writes.

```bash
.venv/bin/python -m unittest discover -s tools -p 'test_*.py' -v
.venv/bin/python tools/run_host_validation.py --binary build-experiment/iptam_replay \
  --datasets-root data/prepared \
  --native-lounge-root data/prepared/native \
  --candidates-from evidence/host/final_pairs_20261112.json \
  --repeats 3 --output evidence/host/FRESH
.venv/bin/python tools/run_host_validation.py --binary build-experiment/iptam_replay \
  --datasets-root data/prepared \
  --datasets actual_lounge --candidates-from \
  evidence/host/actual_lounge_exhaustive_20261112/candidates.json \
  --repeats 3 --output evidence/host/FRESH_ACTUAL
```

Each fresh suite freezes its executable/hash, evaluator, plan, initialization
candidates, complete CSV/logs, resource measurements and all repetitions. Selection
uses images and intrinsics only. A complete uninitialized run is retained as a
failed tracking outcome; input errors, timeouts and numerical failures halt the
suite rather than triggering blind repetition.
