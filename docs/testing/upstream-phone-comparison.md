# Original PTAM versus the current iPTAM port on iPhone

> Detailed validation record. See [current results](../RESULTS.md) for the main comparison.
> `evidence/` labels identify local validation artifacts; raw logs, app binaries
> and private inputs are not redistributed with this repository.

Physical comparison completed 13 November 2026: 15 completed benchmark trials on a physical iPhone.

## Question and baseline

Run the original PTAM without the historical iPTAM changes on the same connected phone,
then compare it with the current restored iPTAM implementation. The baseline is
Oxford's official PTAM-GPL r114 lineage, pinned to
`d9dca71ad57b0ef5b409e1670443060d49df63cb`. The exact pre-fork download is
unknown. This comparison measures the complete implementations; it cannot
attribute a difference to one particular historical change or restoration fix.

Read the [compatibility audit](upstream-compatibility-audit.md) for authoritative
source links, all four dependency pins, independent source/hash review and the
narrow arm64/iOS compatibility patch. Original initialization, matching, solver,
recovery, motion model, RNG, thresholds, asynchronous mapper and its 5 ms poll
remain intact. No modern numerical guards or geometric-quality gates enter the
upstream estimator. Only the current engine uses the modern serial mapper and
source-timestamp motion model.

## Physical builds and common inputs

Both executables use the same UIKit recording/replay controller, compiled into
separate apps so their original global C++ symbols cannot collide. Both run on
a physical iPhone 12 (iPhone13,2), iOS 18.0.1. No live camera capture or permission is needed.

- Current app: `BUNDLE_ID_CURRENT`; executable SHA256
  `72171a7c3e9765838bd58881e82af3f2bed8f0cacab4ee4dd16e87983cdb1cc2`.
- Original app: `BUNDLE_ID_UPSTREAM`; executable SHA256
  `64e342dfb7e4b06dd01f40f1ce84a2b67ac7815f9e971fd2c5f0aa13b48434c6`.
- Exact apps, full executable hashes, source ZIPs/inventories and build logs are
  under `evidence/builds/<prefix>/`.
- All 149 compiled historical-core files in the current build match the previous
  final e7fcc profile. Its replay shell gains explicit source pacing and
  diagnostic metadata; no tracking algorithm change was made for this comparison.
- Fresh bootstrap independently reproduced all 51 upstream PTAM files, 522
  generated dependency files and the exact upstream host executable.

The [frozen input plan](upstream-input-plan.md) specifies five full inputs,
7,836 frames, identical pinhole pixels/K/clocks and the same predeclared image
index pairs for each engine. Native equidistant lounge results are separate
controls and are not substituted here. The two synthetic lounge references are
simulation truth; EuRoC uses its dataset reference; actual lounge uses
uncertain ARKit references. No references, IMU, depth or meshes enter either
estimator, initialization or pair selection.

Both replays target `wall_start + exposure_timestamp - first_timestamp`. They
retain every late frame in order and log delivery/lag. Different actual lag is
an outcome, not hidden frame dropping. In particular, as-fast feeding could
starve upstream's background mapper. There is no extra optimization/drain before
exporting a frame's pose. External progress/total watchdogs preserve failures
and stop only their own process; failed trials are not silently retried.

## Scoring and limits

Common coverage requires initialized nonempty map, native GOOD, at least ten
matches and finite rigid output. Native decisions differ: the current tracker
already enforces final-pose robust support internally. Its additional strict
score requires at least ten measured final-pose robust contributors. Upstream
has no safe equivalent diagnostic; its strict score and solver-event counts are
**unavailable**, not zero. Upstream found is the current keyframe's unique
measurement count; asynchronous map sizes are approximate status snapshots.

One offline Sim3 fits monocular scale/orientation. RMS is on associated accepted
poses only. Report total-input denominator, reference-motion coverage and lost
gaps/tails with RMS; a small fitted subset error is not whole-run success.
Original processing time excludes background mapper CPU; current processing
time includes serial mapping. These means are not equivalent total-compute
measurements. Wall time, lag and thermal conditions accompany them.

## Physical results

The original estimator runs on the phone, but neither implementation succeeds across this input set. Results are mixed: upstream has better shared-time fitted accuracy on new lounge and both EuRoC cameras, while the current port tracks more of new lounge/EuRoC-left and repeats exactly on its two repeated cases. Both fail actual-lounge initialization. Current EuRoC-left performance is far below the input rate. These findings do not establish general stable scanning.

Primary results below use the predeclared **rep01**, never a best repeat. Common GOOD is the rule above; current strict counts equal its common counts. RMS is fitted separately on each engine's accepted subset, so its motion coverage must be read with it.

| Recording | Original common GOOD / input | Current common/strict GOOD / input | Original / current RMS | Original / current reference motion | Largest lost gap, original / current |
| --- | ---: | ---: | ---: | ---: | ---: |
| Historical synthetic lounge | 57/230 | 44/230 | 0.31 m / 0.2341 m | 16.93% / 11.38% | 153 / 169 |
| New synthetic lounge | 33/100 | 71/100 | 0.02225 m / 0.249 m | 43.09% / 77.11% | 46 / 6 |
| EuRoC left | 366/3682 | 932/3682 | 0.01922 m / 0.02901 m | 12.74% / 28.66% | 2110 / 545 |
| EuRoC right | 557/3682 | 234/3682 | 0.05774 m / 0.05907 m | 7.71% / 3.19% | 2591 / 2594 |
| Actual lounge export | 0/142 | 0/142 | no fit / no fit | not tracked / not tracked | uninitialized / uninitialized |


## Accuracy at shared accepted times

To reduce the coverage confound, the following diagnostic independently fits each engine on exactly the same intersection of accepted exposure indices. Selection uses logged quality/support only, never reference error. It does not score the unobserved tails or replace complete coverage results.

| Recording | Shared accepted / full input | Shared reference motion | Original RMS | Current RMS |
| --- | ---: | ---: | ---: | ---: |
| Historical synthetic lounge | 44/230 | 11.38% | 25.836 cm | 23.407 cm |
| New synthetic lounge | 33/100 | 43.09% | 2.225 cm | 3.501 cm |
| EuRoC left | 355/3682 | 12.09% | 1.871 cm | 2.369 cm |
| EuRoC right | 211/3682 | 2.75% | 2.297 cm | 5.503 cm |
| Actual lounge export | 0/142 | 0.00% | no fit | no fit |

EuRoC-right has 211 shared accepted timestamps and 210 reference-associated poses. For new lounge, the primary 25 cm versus 2.2 cm error gap shrinks to 3.5 cm versus 2.2 cm on the same 33 frames. This distinction prevents interpreting different scored intervals as a direct accuracy ratio.

## Repeatability and runtime

Current new-lounge 71/100 and EuRoC-right 234/3682 repeat exactly in all exported frame fields after excluding only `processing_ms`, `delivery_wall_seconds` and `schedule_lag_ms`. Original repeated new-lounge counts are 33/32, historical 57/53 and EuRoC-right 557/898; poses differ from the initialization exposure onward in each pair. This is observed same-binary variability with the original asynchronous implementation, not a claim that one scheduling factor alone causes every difference.

| Recording | Original / current replay wall | Original / current maximum schedule lag | Original / current max thermal code |
| --- | ---: | ---: | ---: |
| Historical synthetic lounge | 22.909 / 22.919 s | 0.010 / 0.018 s | 0 / 0 |
| New synthetic lounge | 9.913 / 9.919 s | 0.010 / 0.018 s | 0 / 0 |
| EuRoC left | 184.067 / 386.326 s | 0.021 / 223.761 s | 2 / 2 |
| EuRoC right | 184.056 / 184.062 s | 0.025 / 0.008 s | 0 / 0 |
| Actual lounge export | 131.938 / 131.936 s | 0.265 / 0.269 s | 1 / 1 |

Thermal codes: 0 nominal, 1 fair, 2 serious, 3 critical; low-power mode was false. EuRoC-left source span is 184.05 s. Current takes 386.326 s, mean tracker plus serial mapping 78.35 ms/frame, maximum 888.22 ms, and maximum lag 223.76 s. Original completes at 184.067 s, but loses tracking much earlier (366 versus 932 accepted frames and 12.74% versus 28.66% motion). Original tracker-only mean 10.39 ms excludes its background mapper CPU and is not an equal total-compute measure.

## Verification and videos

- Fifteen retained benchmark trials complete all 23,466 planned rows. Recorded
  executable, manifest, calibration, pair, exposure clock and complete consumed
  pixel hashes were independently verified.
- Five Release suites and the presentation helper regressions pass, including new timeout-cleanup, partial-invalid-pose, engine-identity and shared-time scoring regressions. The supported current core's 149 compiled historical files are unchanged; existing ASan/UBSan control results are not presented as newly rerun upstream tests.
- Both exact phone builds pass six malformed/missing/truncated input cases plus real OS background cancellation: 14 checks total. Source-paced initialized sessions stop at 41 (upstream) / 40 (current) frames; pending delayed work does not revive either run.
- Default-library checks report `camera_permission_requested=false`; only
  existing recordings are replayed.
- Final paired videos are `videos_20261113_final/lounge_new_upstream_vs_current.mp4` and `euroc_cam1_upstream_vs_current.mp4` under the evidence root. They visualize fixed rep01 physical-phone logs offline, including primary and shared-time errors, coverage, unjoined gaps and timing scope. They are not phone screen captures or measured phone-FPS demonstrations. Exact renderer/input/output hashes, ffprobe validation and visually inspected samples accompany them. The earlier `videos_20261113_rep01` is retained as a presentation draft.

Full machine report: `evidence/upstream_comparison/final_report_20261113/report.json`; human table in its `report.md`. Trial evidence is in `phone_20261113/`; independent identity/pixel verification and repeat-state hashes are there too. New validation logs are in `validation_20261113/`, lifecycle receipts in `lifecycle_20261113/`, library receipts in `libraries_20261113/`. Raw outputs and build snapshots are retained in the local validation archive.

## Read-only diagnosis

The separate [divergence analysis](upstream-divergence-analysis.md) compares
gauge-invariant first motion, map-point totals and keyframe timelines without
changing either implementation. Lounge initialization/first motion closely
agree. On both modern EuRoC-right repeats, active plus retired points stays
exactly 450 while the map grows from two to 24 keyframes; upstream creates many
additional points. Candidate-rejection diagnostics are a better next step than
tuning point caps that never bind. These are observations and hypotheses, not a
claim that one identified change explains all accuracy or coverage differences.

## Reproduction and evidence

```bash
python3 upstream/bootstrap.py --work-root upstream-work --build
IPTAM_UPSTREAM_PORT_DIR=upstream-work bash scripts/build_ios.sh upstream
bash scripts/build_ios.sh current

.venv/bin/python tools/run_phone_replay.py \
  --dataset data/prepared/dataset \
  --app build-ios-upstream/Release-iphoneos/PTAMUpstream.app \
  --bundle BUNDLE_ID_UPSTREAM --device DEVICE_ID --source-paced \
  --init-first FIRST --init-second SECOND \
  --output evidence/phone/FRESH

.venv/bin/python tools/compare_upstream_phone.py \
  --root evidence/upstream_comparison/phone_20261113 \
  --output evidence/upstream_comparison/FRESH_REPORT
```

Use the frozen plan's correct input/pair. Hold the shared per-device lock and
run phone trials sequentially. To test the current engine choose its app and
`--bundle BUNDLE_ID_CURRENT`. Set `DEVICE_ID` and bundle placeholders for your
connected device and signed targets. Every trial uses a fresh process and result path.
A verified complete first trial permits its exact manifest cache for repeats;
never skip copying an unverified different input. Preserve original logs.

Evidence root:
`evidence/upstream_comparison/`.
The phone suite records command JSON, device output, effective manifest,
per-frame JSON/CSV, complete pixel hash, binary identity and summary. The saved
orchestration script and outcome list define trial order. Repeated-state checks
exclude only processing/delivery/lag times. The read-only evaluator writes
private normalized scoring inputs and reports unavailable/missing/partial
trials explicitly.

Context used: project SUMMARY.md, then SKILL.md, report analysis, dataset
inventory, common-input plan and the actual official PTAM/dependency sources.
No GitHub push or remote publication is part of this work.
