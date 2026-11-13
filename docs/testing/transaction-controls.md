# Pre-freeze transaction-only phone controls

> Detailed validation record. See [current results](../RESULTS.md) for the main comparison.
> `evidence/` labels identify local validation artifacts; raw logs, app binaries
> and private inputs are not redistributed with this repository.

The native equidistant camera model tracks every post-initialization exposure of the new lounge simulation on the physical iPhone12, with6.01mm fitted-scale position RMS. Historical-lounge and real EuRoC tests still contain long tracking gaps. Successful execution, deterministic output and finite poses do not establish continuous stable tracking.

These are actual physical-phone recorded-input results, not simulated iPhone execution, live handheld camera accuracy, or ARKit-derived estimates. These tables preserve earlier transaction-only artifacts before the final tracker safety fixes. They are controls, not the selected final build. The damping-floor experiment was subsequently rejected; see damping-floor-control.md.

## Physical iPhone results

Target: physical iPhone 12 (iPhone13,2), iOS 18.0.1. Each evidence directory under `evidence/phone/` contains on-device manifest/pixel hashes, complete source-time rows, app/source provenance and `evaluation.json`.

| Source / evidence ID | Camera model | Init frames | Strict GOOD / input | RMS on valid reference poses | Reference motion covered | Largest lost gap | Hard rejects / retained stops | Mean / max phone processing |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| New lounge: `native_lounge_new_01` | equidistant | 0/18 | 81/100 | 0.601 cm | 95.62% | 0 frames | 0 / 8 | 8.00 / 37.42 ms |
| Historical lounge: `native_lounge_reference_01` | equidistant | 0/16 | 84/230 | 0.538 cm | 31.31% | 129 frames | 0 / 78 | 28.76 / 57.06 ms |
| EuRoC MH_01_easy cam0: `transaction_euroc_cam0_01` | rectified pinhole | 1200/1205 | 868/3682 | 7.052 cm | 25.35% | 678 frames | 0 / 414 | 122.54 / 696.55 ms |
| EuRoC MH_01_easy cam1: `native_profile_euroc_cam1_01` | rectified pinhole | 0/5 | 534/3682 | 7.804 cm | 13.70% | 1988 frames | 65 / 8 | 5.71 / 450.47 ms |

New lounge has81/81 strict-good post-initialization exposures, no subsequent gap, zero weak output, zero hard numerical rejection and eight safely retained solver stops. It covers95.62% of the full reference path; the omitted motion precedes initialization. Its position endpoint is4.87mm and median absolute orientation disagreement0.254°. All four completed phone recordings have finite rigid saved poses, but the coverage failures in the table remain failures.

The EuRoC cam0 start was deliberately delayed to an image-supported initialization pair near60s. All3,682 source exposures remain in the denominator. Cam1 uses the original early pair; only533 of its534 valid outputs fall in the available reference interval. None of these partial-coverage results should be presented as a full-sequence VIO or stereo leaderboard score.

## Inputs and scoring

Lounge inputs are separate synthetic motions through a photo-fitted Blender lounge, not a measured lounge RGB/depth survey. `lounge_new` comes from VIOBench/new_simulation_test; `lounge_reference` is the historical circular trajectory. EuRoC cam0 and cam1 are two monocular views of the same real MH_01_easy flight. Original recordings, model assets and previously prepared controls are unchanged.

The authoritative lossless native lounge inputs are `evidence/native_datasets_exact/{lounge_reference,lounge_new}`. All original grayscale pixel hashes match the generated PGM hashes. K is `[253.30344967,253.34336776,319.5,239.5]` at640×480 and `lens_model=equidistant`. A separate mask marks the16 outside-domain corners; image intensities are preserved. The model rejects the back hemisphere. Old pinhole controls remain in `testing/datasets`; they have different angular coverage and are not interchangeable native-image tests.

Strict GOOD means quality2, at least ten visual matches, nonempty map and a finite rigid transform. Weak quality1 is counted separately. A proper Sim3 fit resolves the arbitrary monocular scale, and ground truth/reference enters evaluation only. Position RMS is on actual valid reference-associated exposures; it does not penalize missing output, so it must be read with coverage. Endpoint is the last scored output, which may precede the dataset end. Motion coverage sums reference path length only between adjacent strict-good original exposures and never bridges a lost segment.

All four prepared camera/reference conversions passed an independent actual-image epipolar audit, about0.17pixel median error. Native lens selection is explicit and the harness verifies the binary/app actually selected the requested model. See [dataset inventory](../dataset-inventory.md) for exact paths, clocks, optical transforms and commands.

## Failure evidence and current limitations

The historical native lounge gap has ample image features: lost frames have median477 Shi–Tomasi corners versus427 in good frames, and297 persistent consecutive LK tracks versus279. Texture availability alone does not explain the failure. Its early weak-pose segment73–83 remains accurate at4.96mm RMS under an alignment fitted only to GOOD frames. An isolated weak-keyframe insertion experiment was therefore investigated, but its EuRoC regression means that change is not integrated. Raw inputs were never altered to make a test pass.

A retained old binary entered an unbounded NaN map-update retry loop on cam1. The489MB raw log, first matrices and manual-stop record remain in `testing/host/final_20261112_frozen/euroc_cam1/repeat_01`. The numerical guard now bounds iteration, rejects invalid trial updates and preserves accepted finite improvements. Internal rejections/retained stops are reported separately from nonfinite output poses. The host watchdog halts legacy numerical loops rather than blindly launching another repetition.

Active map points are capped at5,000, but keyframe/trash retention is not proven bounded for indefinite capture. Earlier later-pair EuRoC host replay reached314MiB peak RSS and792ms worst-frame processing. The phone cam0 mean above exceeds a30Hz frame budget; completion therefore does not establish real-time continuous operation. No unsupported global memory or performance guarantee is made.

## Reproduction and retained controls

Use the task-local Python3.11 environment and `requirements-validation.txt`. Six Python geometry/scoring/watchdog/model-protocol tests pass. Native release and sanitizer outcomes belong to the final frozen core report, not the older baseline.

```bash
.venv/bin/python -m unittest discover -s tools -p 'test_*.py' -v
.venv/bin/python tools/prepare_datasets.py --datasets lounge_reference lounge_new \
  --lens-model native --output evidence/FRESH_NATIVE
.venv/bin/python tools/run_host_validation.py --binary build-experiment/iptam_replay \
  --datasets-root data/prepared/native \
  --datasets lounge_reference lounge_new --selection-from /path/to/frozen-pairs \
  --output evidence/host/FRESH_NATIVE_RUN
```

Every new host suite preserves its executable, evaluator, command plan, all initialization candidates, all full repetitions, camera-model confirmation, timing and peak RSS. Image-only selection reads no reference poses; short tails cannot win because at least80 follow-up frames are required. Corrected final host repetitions will be recorded here after the source freeze.

Earlier full baseline tables, all12 completed replays and the first failure investigation are preserved in [archive-baseline.md](archive-baseline.md). Its [baseline plot](host-baseline.png) shows actual tracking gaps. Intermediate guarded fixed-pair and visual-reselection suites remain under `testing/host/guarded_*_20261112`; they are transparent controls, not current final accuracy claims.
