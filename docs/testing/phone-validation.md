# Physical iPhone validation — minimum-support core and recording-first app

> Detailed validation record. See [current results](../RESULTS.md) for the main comparison.
> `evidence/` labels identify local validation artifacts; raw logs, app binaries
> and private inputs are not redistributed with this repository.

The current comparison is summarized in [the results report](../RESULTS.md). The frozen profile
and first recording-only launcher checks below remain comparison evidence.

The frozen original-PTAM build completed all ten recorded-input phone runs and
all seven failure/lifecycle checks. The new lounge simulation tracks every
post-initialization exposure repeatably, with 6.145 mm fitted-scale RMS.
Historical lounge and real EuRoC still have long tracking gaps, and the actual
142-keyframe lounge export never initializes. General stable scanning is not
established. As clarified on 13 November 2026, this work evaluates existing
recordings and datasets. Live capture and camera-permission interaction are
out of scope and are not prerequisites for recorded-input validation.

The original 2010 report achieved planar initialization while subsequent map
matching failed and recovery was disabled. This restoration retains PTAM's
initializer, matching, mapper, relocalizer and bundle adjuster. ARKit/IMU poses
do not feed the estimator. See [report analysis](../report-analysis.md).

## Frozen identity and execution evidence

Frozen minimum-support profile iPhone executable SHA256:
`dc0e3d2b551468e4e2013163c9e0873295a5c9d334a805be6d2c11b02a6c8d74`.
Build archive:
`evidence/builds/dc0e3d2b551468e4`, including
app/source ZIPs, build log, `identity.json` and per-file hashes in `source.json`.
Source-manifest SHA256:
`7f124a5636b22c901e0a98e37a4b986a3ac0ba79ac082c0c3f381a8ec79a72f3`.
Matching final host executable SHA256:
`a319cbc477f734d08cf4fbf1690b2ff6bb2673adaba7881dd9161dc9f7717654`.

This profile uses serial mapping, mandatory after-stage cleanup, retired-point
refind rejection, internal/external minimum-ten visual support, bounded finite
transaction-safe bundle adjustment with default damping minimum 0, exact
runtime equidistant optics and live lifecycle/thermal diagnostics. The tested
1e-6 floor, weak-keyframe gate and LK bootstrap are not defaults.

Target: physical iPhone 12 (iPhone13,2), iOS 18.0.1. Runs are serialized
under a per-device lock. Each replay installs/hashes this app, starts a fresh process,
checks selected lens and exact manifest/pixel hashes, and retrieves every row.
All ten final replays have `status=completed`, full input/hash counts and
`storage_error=false`; every saved pose is finite and rigid. Safe completion
and accepted tracking are reported separately.

Evidence root: `evidence/phone`.
`validated_support_aggregate.json` contains all evaluations, full summaries,
app hashes, compared columns and state hashes. Individual run directories keep
requested manifests, source references, provenance, device commands,
`run/frames.jsonl`, `run/summary.json`, `trajectory.csv`, `validation.json` and
`evaluation.json`.

## Recording-first app update — 13 November 2026

The installed app now opens a library of valid prepared recordings rather than
camera startup. Selecting a recording uses the existing replay controller;
returning to the list cancels and saves that run. Only the explicit developer
argument `--live` enters camera startup. A missing `--replay` path fails safely
without falling through to live capture. Launch routing is covered by the
Foundation regression target; all five Release and five ASan/UBSan suites pass.

Installed executable SHA256:
`386ea9d1e408c15e85d3aba023c4dfd119fe954a323bbe4e9048f6d207dbfa7b`.
App, exact source snapshot and build log are preserved in
`testing/builds/386ea9d1e408c15e` under the local evidence root. The PTAM core is
unchanged from the frozen profile above.

Physical evidence `phone/recording_first_launch_01` shows the default library
loaded nine valid cached manifests without requesting camera permission. In
`phone/recording_first_safety_01`, malformed `--replay` produced a failed
zero-frame recorded run; normal launch then opened the library without creating
any new capture/replay run or camera-permission request.

Full fresh-process tests `recording_first_lounge_new_01` (100 frames, 81 GOOD)
and `recording_first_euroc_cam1_01` (3682 frames, 262 GOOD) verified input hashes,
source timestamps and finite poses. Every frame field other than processing
time is exactly equal to the corresponding frozen-profile run, recorded in
`phone/recording_first_comparison.json`. These checks validate the new app
launch path; they do not improve the tracking coverage in the table below.

## Final recorded-input results

All evidence IDs below have prefix `validated_support_`. Native lounge uses
ideal equidistant pixels; EuRoC, real lounge and the plane use calibrated
pinhole inputs. Fixed pairs come from existing image-only selection; no
reference pose selects or initializes the estimator.

| Input / evidence suffix | Init pair | Strict GOOD / all input | Weak | Sim3 position RMS | Reference motion covered | Largest GOOD gap |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| New lounge, `lounge_new_01/02/03` | 0/18 | 81/100 each | 0 | 6.145 mm | 95.62% | 0 after initialization |
| Historical lounge, `lounge_reference_01` | 0/16 | 89/230 | 8 | 5.330 mm | 33.40% | 124 frames |
| EuRoC MH_01_easy left, `euroc_cam0_01` | 1200/1205 | 914/3682 | 276 | 57.70 mm | 28.89% | 535 frames |
| EuRoC MH_01_easy right, `euroc_cam1_01/02/03` | 0/5 | 262/3682 each | 91 | 100.09 mm | 3.95% | 2574 frames |
| Actual lounge, `actual_lounge_01` | 47/48 | **0/142** | 0 | Not scored: no map | 0% accepted motion | All 142 uninitialized |
| Plane / blank occlusion / return, `plane_01` | 0/20 | 107/136 | 0 | Numerical zero (~1.87e-9 m) | 82.89% | 8 frames |

New lounge has 81/81 GOOD post-initialization exposures. The plane fixture loses
tracking on exactly the eight blank frames 60–67 and recovers at 68. Its tiny
numerical residual checks the controlled fixture, not physical-world accuracy.
The actual lounge processes every image safely but never creates a visual map;
that is a tracking failure, not successful reconstruction.

The real lounge source is the recovered LiteReality/ARKit archive with 142
unique RGB/depth/confidence keyframes and matching per-frame camera metadata.
RGB exposures are 0.75–4.34 seconds apart, rather than continuous video. ARKit
provides an uncertain reference, not independent truth; it never feeds PTAM.
The two simulated lounge paths traverse photo-fitted geometry, and the two
EuRoC inputs are monocular views of the same flight. These are not five
independent physical environments. See [dataset inventory](../dataset-inventory.md)
and [actual lounge data](actual-lounge-data.md).

Strict GOOD requires quality 2, at least ten actual matches, a nonempty map and
a finite rigid transform. Weak quality 1 stays separate. Sim3 fits arbitrary
monocular scale for evaluation only. RMS scores actual valid reference-associated
exposures and must be read with missing motion/gaps; it does not penalize all
missing poses. Motion coverage counts reference path length only between
adjacent GOOD original exposures and never bridges loss. Cam0's delayed start
keeps all 3682 exposures in the denominator. No RMS is invented for the actual
lounge, whose evaluator reports too few valid poses.

## Exact repeatability, processing and thermal state

New lounge and EuRoC right each completed three fresh-process repetitions.
All compared pose/state fields are exactly identical within each group; only
processing time differs. This establishes same-binary/same-device replay
repeatability, not continuous tracking or cross-device identity.

CSV comparison uses `timestamp`, `tx/ty/tz`, all nine rotation entries
`r00,r01,r02,r10,r11,r12,r20,r21,r22`, `quality`, `found`, `attempted`, `points`,
`keyframes`, `solver_failures` and `solver_retained_stops`. It parses
`list(csv.DictReader(...))`, removes only `elapsed_ms`, serializes with
`json.dumps(..., sort_keys=True)` and SHA256-hashes the result:

- New lounge, each run: `e168dfe8ae18585eaa2aa70d24886aaf46e9814beafd1acf7e1bec5d9b1d62d9`.
- EuRoC right, each run: `38584fca6b44640e72568ef97c6c38428a532daff4873e1344019b84387eaab8`.

An additional comparison of the entire original JSON frame objects removes
only `processing_ms`, then uses sorted-key compact JSON. It also agrees
exactly across all three repetitions: new lounge hash
`792dcfe58a970a1681d5a70e0ce001274255f9fe165e48e5784884300231d1f9`,
EuRoC-right hash
`3d8ea749c2635bc147cfa284661cb07fc3c250f8d878f0228a2087cf1af8ba13`.
That comparison additionally covers initialization/validity flags, trails,
displacement, retired-point count, both pose representations, solver message
and user status message. `validated_support_full_json_repeatability.json`
preserves the exact compared fields, original JSONL hashes and normalized
state hashes for all six runs.

| Input | Mean phone processing ms | Maximum ms | Hard numerical rejects / retained stops | Thermal end / maximum |
| --- | ---: | ---: | ---: | --- |
| New lounge, runs 1/2/3 | 8.38 / 7.76 / 7.55 | 36.13 / 40.71 / 36.16 | 0 / 3 each | fair / fair each |
| Historical lounge | 8.66 | 95.71 | 0 / 4 | fair / fair |
| EuRoC left | 15.09 | **779.16** | 0 / 14 | serious / serious |
| EuRoC right, runs 1/2/3 | 3.03 / 3.05 / 3.07 | 60.30 / 62.70 / 65.68 | 0 / 1 each | fair / fair each |
| Actual lounge | 1.63 | 3.56 | 0 / 0 | serious / serious |
| Plane / occlusion / return | 17.60 | 89.07 | 0 / 0 | fair / fair |

Low-power mode is false in every final summary. Thermal codes are 0 nominal,
1 fair, 2 serious, 3 critical; these are logged observations during the serial
suite. The actual-lounge short run follows the sustained EuRoC-left workload,
so its serious state is not evidence of its own processing cost. Retained
stops preserve previously accepted finite cost-decreasing iterates after a
bounded solve; they are not labeled convergence. No hard rejection occurred
in these final replays.

Replay uses source timestamps for prediction and runs as fast as possible.
Processing timings exclude a complete live camera/UI pipeline. The 779 ms
worst frame and serious thermal state preclude a blanket real-time guarantee,
even though mean cam0 processing is much lower than earlier controls.

## Actual-device safety checks

All seven cases passed in `validated_support_lifecycle_01/results.json`, using
the already installed final build after the serialized replay suite.

| Case | Verified outcome |
| --- | --- |
| Array JSON root | New failed run, zero frames, explicit schema error |
| Null frame entry | New failed run, zero frames, explicit schema error |
| Duplicate timestamp | New failed run, zero frames, explicit clock error |
| Unsupported lens | New failed run, zero frames, explicit model error |
| Missing image | Failed run, zero processed/hash frames, image error |
| Truncated image | Failed run, zero processed/hash frames, image error |
| Real OS backgrounding | Saved `backgrounded` at 40/136 frames, matching partial hash count |

Every rejected manifest's hash is checked, and the process remains responsive
and terminable. Backgrounding briefly opens Settings without changing any
setting; it yields and saves partial evidence instead of claiming completion.
Commands and individual summaries are preserved. Disk-full behavior was not
induced on the physical phone. Live sessions stop and save at 96 keyframes,
10000 retired points or an OS memory warning; indefinite capture is not proven.

## Recording-only scope and retained controls

The user clarified on 13 November 2026 that testing should use recordings and
datasets only. No new live capture, camera permission or handheld session is
required to continue this workflow. Recorded-input tracking failures remain
work to resolve against the existing images.

The earlier live-mode launch is retained as historical evidence only in
`phone/live_validation_01`: zero frames and no startup error. It is not a capture or tracking test, and is not part of the
recording workflow's acceptance criteria.

Earlier phone profiles and the rejected floor timeout are retained in
[phone-controls.md](phone-controls.md). The introduced cleanup starvation was
corrected rather than retained for favorable subset scores. The minimum-ten
policy matches existing external validity internally and prevents unsupported
keyframe seeds. See [cleanup](cleanup-scheduler-experiment.md),
[minimum support](minimum-support-experiment.md),
[optics](equidistant-optics-experiment.md) and
[rejected weak mapping](weak-keyframe-experiment.md). None resolves the remaining
long-path loss or real sparse-lounge initialization failure.

Use `tools/run_phone_replay.py --app build-ios/Release-iphoneos/iPTAM.app` with
the explicit device and a fresh fresh output directory to reproduce a
run. `tools/check_phone_lifecycle.py` exercises safety cases;
`tools/evaluate_trajectory.py` scores exported trajectories. Full commands,
input identities and artifacts remain in each evidence directory. Relevant
context used: `SUMMARY.md`, `SKILL.md`, report analysis and dataset inventory.
