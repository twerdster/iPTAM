# Final-pose visual support follow-up — 13 November 2026

> Detailed validation record. See [current results](../RESULTS.md) for the main comparison.
> `evidence/` labels identify local validation artifacts; raw logs, app binaries
> and private inputs are not redistributed with this repository.

The tested correction rejects tracking poses with fewer than ten final robust inliers while preserving raw patch-match ratios and mapper inputs. It corrects unsupported GOOD/POOR states. It does **not** restore continuous tracking of the difficult recordings.

## Finding and diagnostics

The frozen `a319cbc4…` tracker increments `found` when a patch matches. Subsequent geometric rejection and zero robust weight do not remove that raw patch count. A finite WLS result can also be the zero update supplied by the stabilizing prior when every observation has zero weight. Patch-match presence therefore does not establish final-pose support.

An isolated diagnostic build records final-pose reprojection residuals, robust support and the visual normal-matrix spectrum before mapper maintenance can change point coordinates. It does not feed evaluation/reference poses into the tracker. Its complete historical/new/EuRoC-left/EuRoC-right trajectories match the frozen binary in every non-timing CSV field, establishing that this instrumentation is noninvasive.

On cam1, 63 GOOD/POOR frames have fewer than ten final Tukey inliers, including four GOOD frames with only 3/5/5/6 inliers despite 10–12 matched patches. One POOR frame has zero geometric inliers. The historical loss begins differently: frames 73–83 still contain 78–199 inliers, roughly 0.8–1.0 pyramid-pixel median residual and finite visual Hessians, but the original match-ratio criterion prevents a needed keyframe. This supports further investigation of the estimator; it does not justify restoring the previously rejected weak-keyframe gate.

## Minimal correction

The correction reprojects found points at the final pose after the last update, rather than using cached projections at the solve's earlier linearization pose. It requires finite front-facing geometry, a finite in-image projection, positive finite measurement noise and positive finite robust weight. The same final-solve variance override 16 is reused; for the current Tukey estimator this is a four-pyramid-pixel cutoff. The ten-support minimum is the existing app/evaluator policy, applied to the pose estimator's actual support.

`PoseInliers()` is separate from `MeasurementsFound()` (raw patch matches). The count clears at frame entry, map-tracking entry and reset, so a failed recovery that performs no map search reports zero current support. No raw match flag/counter, match-ratio heuristic, mapper measurement, camera model, solver threshold or mapping schedule changes. No Hessian/observability threshold is introduced.

## Bounded full-input comparisons

Each candidate uses the same fixed image-only initialization pairs and a 120-second child deadline. All five gate-only runs complete safely. The three initialized non-cam1 inputs are exactly unchanged in every legacy non-timing state/pose field; the real sparse lounge remains uninitialized.

Values below are GOOD frames / fitted position RMS in cm. RMS scores only valid reference-associated exposures and must be read with coverage.

| Input | Frozen a319 | Final-pose gate only | Remove final outliers and recount ratio | Refresh warp after coarse pose |
| --- | --- | --- | --- | --- |
| Historical native lounge | 89 / 0.565 | 89 / 0.565 | 79 / 0.655 | 82 / 0.430 |
| New native lounge | 81 / 0.517 | 81 / 0.517 | 81 / 0.563 | 81 / 0.565 |
| EuRoC cam0 | 597 / 4.526 | 597 / 4.526 | 426 / 23.491 | 357 / 2.608 |
| EuRoC cam1 | 237 / 8.762 | 230 / 4.964 | 218 / 3.874 | 237 / 8.762 |
| Actual 142-frame lounge | never initialized | never initialized | never initialized | never initialized |

Gate-only cam1 motion coverage falls from 3.424% to 3.132%; the longest gap increases from 2593 to 2594 frames. On the same 218 mutually GOOD exposures, RMS changes from 5.006 to 4.664 cm and median orientation disagreement from 3.616 to 2.502 degrees, while endpoint error worsens from 33.0 to 37.2 cm. Thus the lower overall fitted RMS is not evidence of an across-the-board accuracy or coverage improvement.

A further rejected control removes zero-weight exported measurements while preserving the old ratio. Historical/new give 85/81 GOOD, but cam0 reaches its 120-second deadline with 2857 preserved rows. The affected case is not restarted; cam1 and actual were not attempted in that rejected control. It is an incomplete failed experiment. Removing mapper inputs and refreshing warps are not part of the promoted fix.

## Canonical implementation and regressions

The canonical implementation changes only `Tracker.cc`, `Tracker.h` and the existing core regression test; app/CLI/evaluator logging is handled separately. It shares the final variance constant with the actual pose solve and respects the selected robust estimator. The prototype used the default Tukey profile.

The fixture solves twelve calibrated multi-depth correspondences: nine zero-residual inliers plus three rejected outliers must give raw12/inliers9/BAD; ten plus two must resume GOOD; all twelve outliers give a finite zero-prior update but zero support/BAD. A final-pose shift after linearization invalidates observations whose cached projections still fit. Level-one noise scaling, unchanged raw counters/found flags/mapper measurements, lost-state transition and reset/no-search clearing are covered.

Canonical host binary SHA256 after integration:
`a85110d2098a5ecd1dcc18e3f8d2d1a5c25be018998cbc9547f8aa3bc1090b05`.
Five release tests passed in 5.20 s; five ASan/UBSan tests passed in 29.67 s. The Foundation parser is a separate host test; Apple leak detection is not supplied by ASan. Formal repeated host and physical-phone validation of this canonical binary is recorded separately. No physical-device outcome is inferred from these host experiments.

## Evidence and reproduction

All raw commands, complete/partial CSVs, diagnostics, outputs, evaluations and candidate results are under:
`evidence/experiments/loss_diagnostics_20261113`.

`reproduction/manifest.json` records exact source ZIP/executable hashes. The folder contains the diagnostic control, each isolated candidate source and binary, semantic patches, orchestration/evaluation scripts and current canonical portable source/tests. Its README gives native-filesystem CMake commands. Each trial's `command.json` preserves the exact argument vector and calibration; reference data enters offline evaluation only. `geometric-gate/common_exposure_comparison/comparison.json` preserves the identical-exposure comparison above.

The frozen a319 profile and all earlier controls remain unchanged. General scan stability remains unachieved: actual sparse lounge initialization still fails, historical tracking has a long gap, and EuRoC motion coverage is incomplete. These experiments correct support semantics without concealing those failures.
