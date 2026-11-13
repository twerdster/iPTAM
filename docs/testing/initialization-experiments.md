# Isolated initialization experiments, 2026-11-12

> Detailed validation record. See [current results](../RESULTS.md) for the main comparison.
> `evidence/` labels identify local validation artifacts; raw logs, app binaries
> and private inputs are not redistributed with this repository.

The dependency-free optical-flow prototype improves initial correspondences, and some partial-trajectory accuracy, but does **not** establish sustained stable tracking. It is not integrated into the shipped estimator. The experiments used a frozen earlier core; subsequent canonical numerical guards must be assessed separately.

Context: project `SUMMARY.md`, `SKILL.md`, the original report review, and `docs/dataset-inventory.md`. No source recordings or canonical core files were edited by these experiments. The lounge sequences are rendered from photo-fitted geometry, not a measured lounge scan.

## Method and reproducibility

All preserved artifacts are under:

`evidence/experiments/lk_bootstrap_v1`

Output storage was checked before writing. `plan.json`, `same_pair_control_plan.json`, `window_candidates/plan.json`, `selected_windows.json`, `artifact_hashes.json`, and `frozen_core_sources.json` record identities and selection rules. `artifacts/` contains executable binaries, source, frozen core source/header/archive snapshots, image-only correspondence CSVs and a sampled stack trace. Keep this directory when handing off the work.

The prototype seeds spatially separated Shi-Tomasi features by scanning the image, rather than requiring a FAST corner. It tracks translation using four existing image-pyramid levels, bilinear sampling, inverse-composition image gradients and mean-offset elimination. It rejects low-gradient patches, excessive updates, final photometric RMS over 25 intensity levels, and forward/back disagreement over 1 pixel. The original 9x9 patch was compared with 15x15 and 21x21 patches. It has a 25-iteration cap; reaching the cap alone is not a rejection, so the residual and forward/back gates remain important limitations.

The estimator-side prototype has no OpenCV dependency. OpenCV was used only for separate correspondence/reference audits. Ground-truth poses were never supplied to the detector, flow, initializer, tracker or mapper, and were not used to select windows.

All bootstrap experiments use original PTAM homography decomposition, triangulation, bundle adjustment, map matching and relocalization from the frozen archive. The harness rounds flow correspondences to the initializer's integer-pair API; the original initializer then performs its own 8x8 subpixel refinement. After initialization it calls the original tracker with the source timestamps. Uninitialized rows remain in the full-sequence denominator.

Native controls exercise the original MiniPatch path, using the **same frozen archive, same input frames, same camera intrinsics and same initialization pairs**. These are separate from the broader project suite's selected pairs. No comparison below silently changes the pair to favor one frontend.

| Identity | SHA256 |
| --- | --- |
| Frozen mapper/tracker archive, `artifacts/libiptam_core.a` | `5e2c0a12911baf1dc0b91e1f90471673eb716b4ddce3529ec9a79f533bd237b8` |
| Frozen core source/header/archive manifest aggregate | `14d0618ac03e633d4efd778e45eb1d3f066b73bdbb0c2c1ade21461b9fabff25` |
| Flow source, `pyramidal_flow_probe.cc` | `11d3c44908c71d1f8596f283a989ae03e4320d412bece78adbc4adb34b0c7914` |
| Flow executable, `pyramidal_flow_probe` | `67a9424577b7be0ff764b8c6a01b462b9428a5503ef3cc8f020dfefcc7ac168e` |
| Flow generator's earlier image-helper archive, `libiptam_core-probe.a` | `11da0e18ce88cb228a99d75fd8ef903d603bf41742206e824c223685a835c9b2` |
| Original bootstrap harness source, `lk_bootstrap_replay_v1.cc` | `43e4abd045491965811b16dbc965db8def4fe2877850838e4104202864908084` |
| Bootstrap executable, `lk_bootstrap_replay` | `87791502c4ed00583db51864755cb46a91f31976404252b79ac07def306aeac5` |
| Harness source with native control mode, `lk_bootstrap_replay.cc` | `ea282d001ed2f3a41d9be452bbd2b532627a799cbf90b30d78a65e17ccb1c622` |
| Native-control executable, `lk_bootstrap_replay_with_control` | `9f6dd51012bd15ce88630e1aec91e482b05db3cce2ebee58a1a01140f59b31dd` |
| Instrumented copied mapper source, `MapMaker.instrumented.cc` | `ebfca73d678daadf3075fd28df9b6609d62106d501f355d4115120ba441cd850` |
| Instrumented audit executable, `lk_bootstrap_instrumented` | `f63f68351ad26c7deff628304bb237848e1065d633fc3a1a33cf99b04b784c93` |
| Copied residual-acceptance mapper source, `MapMaker.residualaccept.cc` | `571acca935c656802a51c21f06e89ddbc84d8a5bf81cc97c0317d5e5fe0b0b2c` |
| Residual-acceptance executable, `lk_bootstrap_residualaccept` | `5dbdc8c4cbf1c7ebdbc642d65443786d20ff79d57c7a30e0535c2f13d9cd9b74` |

The flow generator uses only KeyFrame/image/Shi-Tomasi helpers from its earlier archive; mapper experiments and native controls both use the later, identical frozen mapper/tracker archive. The manifest aggregate is SHA256 over sorted `relative_path + NUL + file_digest + newline` entries; individual files are listed in `frozen_core_sources.json`.

Example invocations from `artifacts/`:

```bash
./pyramidal_flow_probe DATASET_DIR 15 4 evidence/tracks.csv dense
./lk_bootstrap_replay DATASET_DIR evidence/tracks.csv 15 400 evidence/trajectory.csv
./lk_bootstrap_replay_with_control DATASET_DIR native 15 400 evidence/native.csv
```

The flow arguments are dataset directory, final bootstrap frame, patch half-width, output CSV, and dense detector mode. The replay arguments are dataset directory, correspondence CSV (or `native` in the control binary), second bootstrap frame, pixel focal length and output CSV, followed by an optional frame limit. Actual invocations, exit states and walltimes are in the saved run JSONs. Reference evaluation uses the existing `tools/evaluate_trajectory.py`, after estimation has finished.

## Correspondence and short-window results

Fixed pairs are lounge reference 0/15, lounge new 0/17, and both EuRoC cameras 0/30. Focal lengths are 400 pixels for lounge and 460 pixels for EuRoC, all at 640x480 with center (319.5,239.5).

The dense detector seeds 577, 710, 1000 and 1000 features respectively. On lounge reference the FAST/Shi-Tomasi path exposed only 147 original candidate locations, illustrating that increasing `MaxInitialTrails` alone does not create more candidates.

| Sequence | Surviving LK9 tracks | LK15 tracks | LK21 tracks |
| --- | ---: | ---: | ---: |
| Lounge reference | 309 | 368 | 401 |
| Lounge new | 289 | 389 | 436 |
| EuRoC cam0 | 101 | 65 | 38 |
| EuRoC cam1 | 106 | 82 | 41 |

LK9 total host flow-generation times for the entire bootstrap span were approximately 125, 142, 277 and 287 milliseconds respectively. These are host measurements, exclude subsequent mapping, and are not phone performance claims.

For LK9, independent lounge reference epipolar medians were 0.354 and 0.693 pixels. Image-only homography RANSAC at 3 pixels retained 240/309 and 188/289 tracks. EuRoC fundamental-matrix RANSAC at 1 pixel retained 84/101 and 85/106. No reference epipolar error is reported for EuRoC frame0: its supplied reference begins at 1.105 seconds, later than that exposure. Extrapolating would be invalid.

The following table uses exactly the first **80 frames after initialization**. A strict valid frame requires quality2, at least ten visual matches, a nonempty map and a finite rigid pose. Weak quality1 never counts as success.

| Sequence | Native MiniPatch | LK9 | LK15 | LK21 |
| --- | ---: | ---: | ---: | ---: |
| Lounge reference | 28/80 | 29/80 | 27/80 | 24/80 |
| Lounge new | 30/80 | 0/80, initialization failed | 26/80 | 30/80 |
| EuRoC cam0 | 0/80, initialization failed | 56/80 | 52/80 | 0/80, initialization failed |
| EuRoC cam1 | 0/80, initialization failed | 57/80 | 63/80 | 0/80, initialization failed |

The exploratory window rule selected the highest strict coverage, breaking ties by mean match count, without reference errors. This selects LK9, LK21, LK9 and LK15 respectively. Per-sequence window selection is exploratory tuning; it is not evidence for a universal live-camera default.

## Full sequences and failures

All four LK9 and native-control full runs completed. Counts include every input frame.

| Sequence | Native strict valid | LK9 strict valid | LK9 largest lost span | LK9 reference motion coverage |
| --- | ---: | ---: | ---: | ---: |
| Lounge reference | 42/230 (18.3%) | 54/230 (23.5%) | 160 frames | 15.4% |
| Lounge new | 30/100 (30.0%) | 0/100 | Entire sequence | 0% |
| EuRoC cam0 | 0/3682 | 208/3682 (5.65%) | 2592 frames | 3.87% |
| EuRoC cam1 | 0/3682 | 236/3682 (6.41%) | 2598 frames | 4.29% |

The selected LK21 full lounge-new run yielded 30/100 strict valid frames, a 43-frame largest lost span, and 39.4% reference motion coverage. It did not improve full valid-frame coverage over the same-pair native control. For context, its fitted Sim3 position RMS was 0.0190m and adjacent valid-frame rotation median error 0.222 degrees, versus 0.0635m and 0.333 degrees for native. These scores concern the tracked subset; missing motion remains failure.

The selected LK15 EuRoC-cam1 run grew to 5000 points and 37 keyframes, with GOOD tracking logged through frame300, then recovery near frame400. It did not finish within the predeclared 90-second external timeout. Sampling at about 87 seconds found nearly all CPU in `Tracker::TrackFrame -> MapMaker::BundleAdjustAll -> Bundle::Compute/Do_LM_Step/Cholesky`; sampled peak process footprint was 90.5MiB. The timeout killed the process. The interrupted trajectory CSV is zero bytes, so there are **no invented full-trajectory metrics** for this trial. Its completed 111-frame candidate prefix, stdout/stderr, timeout record and sampled stack are preserved. This result cannot distinguish expensive continued optimization from a numerical loop solely from the sample, but it fails the bounded full-completion requirement in either case.

Low RMS on short recovered portions does not establish consistency. For example, LK9 EuRoC cam1 has a fitted position RMS of 0.0225m on its 236 valid exposures while covering only 4.29% of reference motion and missing 2598 consecutive frames. Coverage and gaps take precedence over promoting that RMS value.

## Why the lounge-new LK9 initializer rejected a map

Instrumentation was confined to a copied mapper. Of 289 supplied correspondences, the original unwarped 8x8 subpixel refinement rejected 55 and triangulation/cheirality rejected 56, leaving 178 points. After the first bundle call, 12 points were flagged bad; the unflagged residual RMS was 0.729px, median 0.504px, p90 1.131px. After five calls, 18 were flagged bad and unflagged RMS was 0.678px.

Map expansion produced 480 points. After the final bounded twenty bundle calls, 290 were flagged bad, 477 had positive depth, and 190 unflagged points had support in both views within 2px (184 within 1px). Their 380 measurements had RMS 0.318px, median 0.142px and p90 0.445px. The optimizer's `Converged()` flag was still false, causing initialization rejection.

A separate copied variant pruned flagged points and allowed bounded termination when at least thirty points were supported in two views within 2px, at least 90% had positive depth, and RMS was at most 1.5px. It accepted all 190 surviving points. Yet the full lounge-new run then supplied only 19/100 strict valid frames, ended with 54 lost frames, and had 1.69-degree median adjacent rotation error. Residual-based acceptance can prevent rejecting a well-fitted two-view model, but does not prove that model or subsequent tracking is reliable. This diagnostic variant is also not integrated.

## Resulting engineering conclusions

The original integer FAST-only trail matcher and sparse feature selection are real bootstrap limitations. A pyramidal subpixel frontend is a viable future direction, but these tests do not justify replacing the canonical initializer now. Larger feature counts and successful initialization are insufficient: map ambiguity, loss under new views, map growth, optimizer termination and runtime remain decisive.

The code review also identified concrete independent defects and supplied regression tests to the canonical core owner: rejected warp followed by a cached valid view retaining a stale bad-template flag; a public template wrapper inverting rejected/singular warps; nonfinite geometry passing visibility comparisons; and failed subpixel refinement being inserted as a map measurement. Those fixes and numerical termination guards belong to the canonical validation report, with their own binary identities.

Preserve the proven canonical safety fixes and report the remaining loss/coverage limitations. A future frontend change should freeze one general profile, retain all failed candidates, verify recovery and map growth over full unseen sequence remainders, enforce solver/runtime bounds, and repeat the same test on the phone before integration.
