# Mobile PTAM implementation and validation

> Detailed validation record. See [current results](../RESULTS.md) for the main comparison.
> `evidence/` labels identify local validation artifacts; raw logs, app binaries
> and private inputs are not redistributed with this repository.

The paper-inspired mobile variant is implemented. Its corrected host executable
completed 31 retained prepared-data trials, but substantial tracking failures remain.
Physical phone validation is complete and reported separately below and in the
[phone comparison](mobile-phone-comparison.md). General stable scanning has not
been established.

Work began on 2026-11-13 on branch `mobile-paper-variant`, based on modern
commit `2c11b6a`. The design follows ideas
from Klein and Murray's [ISMAR 2009 paper](https://www.robots.ox.ac.uk/~dwm/Publications/Papers/klein_murray_ismar2009/klein_murray_ismar2009.pdf).
It retains the guarded modern estimator and is not an exact reconstruction of
the historical commercial phone source.

## Frozen builds and controls

| Build | Executable SHA256 |
| --- | --- |
| Corrected host | `694807b9c7081caa61fa885050e99235ca4416f40d9d4f20c60316a522be3638` |
| Corrected phone | `0c740562b775e644bf7cb2bbd73c3568bcedbfc904776d3f22b9b214c32d96f7` |

Exact identities, source snapshots and executables are archived under
`evidence/mobile_paper_20261113/builds/`, in
`host_deferred-694807b9c7081caa/` and `phone_deferred-0c740562b775e644/`.
Source and build snapshots are retained alongside the local validation archive.

Earlier scheduling controls use different binaries and do not validate the
corrected scheduling path. Canonical current/upstream comparison builds remain
separate.

## Implemented scope

- Mapping rejection/stage diagnostics and a guarded scene-depth interval repair.
  The isolated EuRoC-right diagnostic reproduces the legacy reversed interval;
  final partial-point profiles use inverse-depth growth and do not exercise that
  legacy epipolar repair.
- Serial foreground/idle mapping with cooperative 4 ms quanta, bounded numerical
  solves and explicit overrun counters. This is a soft budget, not a hard
  real-time guarantee.
- Observation and active-keyframe culling with connectivity/gauge checks, safe
  retained-image ownership, archived-pose anchors and finite session limits.
- Five pyramid levels, multiscale Shi–Tomasi features, exhaustive coarse patch
  matching, ESM refinement and a rotation/blur fallback that cannot seed KFs.
- Provisional inverse-depth association, followed by measured two-view
  triangulation and ordinary mapper checks. Provisional points do not count as
  supported poses.
- Incremental calibrated SL3 initialization, conditioned decomposition and
  temporally competing pose hypotheses. The chosen first/current pair undergoes
  normal triangulation/BA; intermediate views are not retained as a third BA KF.
- First-order visual rolling-shutter correction with verified prepared-image
  timing only. All five retained datasets keep it disabled; synthetic tests
  exercise the enabled path.

Modern numerical, recovery and final-pose visual-support checks remain active.
The app/CLI defaults to `auto`: one predeclared first image, then an image-only
second-view decision. Explicit `--mobile-profile paper` retains the fixed-pair
ablation. The [code audit](mobile-code-audit.md) records implementation settings,
the forced minimum-utility culling adaptation, graph-cap exceptions and remaining
test limits. This task uses existing recordings only, with no live capture or
camera-permission prerequisite.

## Corrected deferred promotion

The earlier phone control exposed a real scheduling defect: KF insertion could
consume its quantum before the only partial-promotion loop ran, silently losing
that opportunity. The corrected mapper snapshots at most 25 eligible stable
candidate IDs before seeding, queues a separate promotion stage ahead of BA, and
retains its cursor when the deadline expires. Retirement/reset cancel ownership
safely; obsolete IDs skip, and delayed older frames do not re-assimilate newer
filter observations. No geometric or quality threshold was relaxed.

The actual-image regression inserts a translated textured-plane KF under an
already expired deadline, verifies the task survives, and promotes it during a
fresh idle quantum. The corrected host logs retain the accounting invariant
`queued - completed - cancelled = pending`. Two pending tasks remain newly
queued at the final historical-lounge exposure. No later
quantum exists for those two tasks; they are not reported as completed.

## Verified host results

All five retained inputs preserve their full prepared chronology, pixels,
calibration and exposure clocks. Reference poses, IMU and depth do not enter
estimation or initialization. Actual-lounge ARKit poses are uncertain offline
references.

The corrected host completed 30 retained fast repeated trials (47,016 rows), plus one
source-paced new-lounge control (100 rows): **31 trials, 47,116 rows, zero
nonfinite and zero nonrigid outputs**. All five automatic pairs with wall-time
budgets disabled repeat exactly; six of ten timed pairs do. Wall-clock mapping
remains load-dependent.

Automatic new lounge has 82/100 GOOD in fast runs. The single source-paced host
control reaches 83/100, 98.02% reference-motion coverage and 1.237 cm fitted RMS;
it is not a physical-device or repeated source-paced result. The actual sparse
lounge never initializes. EuRoC-right's 6.24 mm fit has only three isolated GOOD
exposures and zero adjacent GOOD motion. These are substantial tracking limitations, despite safe
complete processing. Lower subset RMS alone is not a whole-path improvement.

Peak host RSS is 41.05 MiB; worst measured foreground frame is 17.71 ms. Timers
exclude image loading/export and extra idle-slot mapping. All common inputs are
640×480, four times the pixel count of the paper's 320×240 setting; these results
do not reproduce its historical phone frame rate. Full accuracy, gaps, repeats,
resources and scope are in [host validation](mobile-host-validation.md) and
[mapper results](mobile-mapping-results.md), backed by
`host_report_deferred_final/summary.json` and
`mapping_analysis/deferred_20261113/report.json` under the evidence root.

## Tests and physical status

Final corrected-source receipts under
`evidence/mobile_paper_20261113/` record:

- `deferred-release-tests.log`: 12/12 passed in 4.60 seconds.
- `deferred-sanitizer-tests.log`: 12/12 passed in 37.52 seconds, optimized
  RelWithDebInfo (`-O2`) with ASan/UBSan. Apple arm64 leak detection is separate.
- `deferred-mapping-leaks.log`: the final mapping/reset fixture reports zero
  leaked bytes across 733 scanned allocation nodes / 1,118 KB, with reported
  peak physical footprint 6,593K. This is focused ownership evidence, not proof
  of leak-free long recorded runs.

The corrected phone batch completes **nine retained source-paced trials / 15,400
exposures**, covering all five inputs, with new lounge and EuRoC-right each run
three times. New lounge accepts the same 82/100 exposures in each run, but its
poses differ (2.394/4.163/3.876 cm fitted RMS). EuRoC-right repeats exactly while
accepting only one exposure; actual lounge still never initializes. Every
automatic replay completes with finite rigid outputs and nominal sampled
thermal state, but these results do not establish general stable tracking.
[Phone comparison](mobile-phone-comparison.md) records the exact physical logs,
coverage/error, pacing, resources and video provenance. Additional fixed-pair
ablations and lifecycle checks remain separate from this completed replay batch.
Earlier partial/control phone results are not substituted for the corrected
executable.

Two fixed-pair physical controls add 3,782 frames (new lounge 0 GOOD, right 63).
The installed receipt-only handoff revision is `f4681a6e…`: only `ios/App.mm`
changes error metadata among 186 compiled inputs; all estimator sources remain
identical. Its ten malformed/lifecycle checks and two full successful-path
compatibility replays pass separately. The recording library works without requesting camera permission. The retained
five-input benchmark metrics/videos remain attributed to `0c740562…`; the
handoff revision is not represented as a fresh five-input benchmark.

Comparisons with different initialization policies require explicit system mode
and visible policy labels. Automatic or unknown initialization cannot qualify
for fixed-pair isolation. Videos must use saved physical logs, show lost
intervals and source-time speed, and must not imply measured phone FPS.
