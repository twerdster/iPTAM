# Serial cleanup starvation experiment

> Detailed validation record. See [current results](../RESULTS.md) for the main comparison.
> `evidence/` labels identify local validation artifacts; raw logs, app binaries
> and private inputs are not redistributed with this repository.

The serial scheduler's early returns can starve bad-point cleanup. A frozen
default-0 control held flagged bad points active for 197 consecutive historical
lounge frames and 890 EuRoC-left frames. Restoring cleanup after every selected
mapping stage removes that starvation and greatly reduces repeated full-BA
work. It does **not** establish stable tracking: EuRoC-right gains many GOOD
labels on inaccurate poses. Canonical code now restores mandatory cleanup
rather than retaining the introduced starvation defect for a favorable scored
subset. Corrected-profile phone results are forthcoming; the accuracy caveat
below remains part of the evidence.

## Isolated change and identities

The original historical mapper loop reached `HandleBadPoints()` after bundle
work. The serial adaptation instead returned immediately after queue insertion,
recent BA or full BA, reaching cleanup only when all those stages were idle.
A bounded, nonconverged solve could therefore be rescheduled every frame while
its bad points remained in the active map.

This experiment changes only a copied `ProcessOneIteration`: select one
queue/recent/full/maintenance stage using `if/else`, then unconditionally call
`HandleBadPoints()` afterward. Initialization, GOOD/weak/BAD thresholds, map
insertion policy, camera calibration and all BA mathematics remain unchanged.
Both control and variant have identical added observation counters for cleanup
calls, retired points and selected stage. No canonical source was edited by
the experiment. The policy was fixed before any results or reference scoring.

Frozen profile: minimum damping 0, exact runtime camera, transactional finite
solver bounds, invalid-projection search rejection and tracker failure recovery.
Its archive SHA256 is
`97f806f76f3ba83532437b5a0f777932626af202a7cbbf90e6c2c0f21e2b805a`.
The native-lens lounge pairs are reference 0/16 and new 0/18; rectified pinhole
EuRoC uses left 1200/1205 and right 0/5. These are fixed existing image-selected
pairs. References enter evaluation only. Each run starts a fresh seeded map.

Evidence root:
`evidence/experiments/cleanup_scheduler_v1`.
It contains phase plans, all commands/status/CSV/logs/metrics, stage audits,
`starvation_diagnostics.json` and complete source/header/archive/binary copies
under `artifacts/`, with per-file hashes. Important SHA256s:

| Artifact | SHA256 |
| --- | --- |
| Copied instrumented control `MapMaker.control.cc` | `42ebc99c195bc67a1dd04718a6cefb32b1a3c1578a4c5db40180c4019d49a83e` |
| Copied restoration `MapMaker.cleanup.cc` | `c9027166f725b0597f12b74d4d75e815881db202f804f93d0f226cfb5f404dc3` |
| Control executable | `252e690771d4926255745ee5de1346a6a1e36482a656d2e3955c021a80659900` |
| Cleanup executable | `4ee38bbbcd499dee3ead713b49064d2c2d4470fb4fc5d86cdcac89dc05314495` |

At most two trials ran concurrently. Each process had a preregistered external
120-second bound, and each row was flushed. Host timing is not phone timing.
Copied mapper objects and the harness used `-O2` without `NDEBUG`, while the
frozen archive was Release `-O3 -DNDEBUG`. Both sides of this isolated comparison
use the same configuration, but it differs from the canonical production
build. Source identity alone therefore cannot transfer its trajectory to the
production executable; matched production flags and actual-phone checks are
required. In particular, differences from the combined phone profile do not
isolate the retired-refind guard's effect.

## Actual scheduling evidence

| Sequence | Cleanup calls, control → restoration | Maximum flagged bad active points, control → restoration | Longest control starvation with flagged bad points |
| --- | ---: | ---: | --- |
| Historical lounge | 15 → 214 | 180 → 0 | 197 frames, 21–217 |
| New lounge | 26 → 82 | 43 → 0 | 18 frames, 59–76 |
| EuRoC left | 78 (partial) → 2477 | 371 → 0 | 890 frames, 1324–2213, then watchdog stop |
| EuRoC right | 3575 → 3677 | 0 → 0 | No flagged-bad starvation in control |

Historical full-BA stages fall from 137 to 24; left full-BA stages fall from
797 in its interrupted control to 120 in the completed restoration. This is
direct evidence of resource starvation, rather than a claim based solely on
tracking accuracy or feature counts. `HandleBadPoints` also applies the
original accumulated-outlier heuristic; calling it more regularly can alter
future data association even when no already-flagged point was waiting.

## Tracking, cost and failures

Strict GOOD means quality 2, at least ten matches, initialized nonempty map and
finite rigid pose. RMS uses fitted monocular Sim3 on actual valid exposures;
read it together with coverage and completion. Motion coverage counts only
consecutive GOOD original exposures, without bridging gaps.

| Sequence | GOOD control → restoration | Position RMS control → restoration | Motion coverage control → restoration | Host mean / p95 ms control → restoration |
| --- | --- | --- | --- | --- |
| Historical lounge | 89/230 → 89/230 | 5.84 → 5.65 mm | 33.40 → 33.40% | 26.77/44.27 → 7.39/39.47 |
| New lounge | 81/100 → 81/100 | 6.33 → 5.17 mm | 95.62 → 95.62% | 5.72/18.80 → 5.99/20.19 |
| EuRoC left | 452 in **2214/3682 processed** → 597/3682 | 33.81 mm (partial) → 45.26 mm | 15.15% (partial) → 21.81% | 53.16/178.40 (partial) → 7.33/12.77 |
| EuRoC right | 262/3682 → 473/3682 | 103.62 mm → **2957.82 mm** | 3.90 → 10.74% | 2.28/2.95 → 10.98/34.46 |

Left control timed out at 120.02 seconds with 1468 exposures unprocessed.
Its raw status says `timed_out=true`, `execution_complete=false`; geometric
evaluation alone cannot override that failure. Restoration completed the full
left sequence in 31.67 seconds. Both right runs and all lounge runs completed.
No nonfinite/nonrigid output was observed in any completed or saved prefix.

EuRoC-right restoration has 151.22° median absolute orientation error and
0.0968° median adjacent-frame rotation error, versus 6.01° and 0.0359° in its
control. A common-exposure audit keeps only the 199 strict GOOD exposures
shared by both runs, fitting each trajectory independently on the same
timestamps: control RMS 111.49 mm / median orientation 6.94°, restoration
122.41 mm / 7.94°. Thus the much larger full-restoration error mainly concerns
the additional inaccurately tracked GOOD exposures; it is not explained away
by merely changing the scored time interval. First map/state divergence is
frame 95; first pose divergence is 96.

Maximum point/keyframe counts are historical 1447/25 → 1262/24, new lounge
1267/15 → 1216/15, left 5000/39 (partial) → 5000/57, and right 383/37 →
5000/53. The active-point cap remains; large-map/trash retention and map
quality still need explicit limits and honest reporting.

The starvation defect is real and separate from the original estimator's
accuracy limitations. These isolated results must not be relabeled final-phone
results or presented as a general stability improvement.

One related read-only review finding remains distinct: `ReFindNewlyMade`
skips `bBad` points, while `ReFindFromFailureQueue` / `ReFind_Common` do not.
Retained trash pointers stay allocated, but a queued failed observation can
potentially insert a retired point into a keyframe measurement map. Any guard
for that condition is a separate change and was not included here.

## Canonical correctness restoration

The canonical scheduler now performs the tested stage selection and mandatory
cleanup afterward. It also rejects `bBad` points at entry to `ReFind_Common`,
preventing failure-queue refind from adding retired points back to keyframe
measurement maps. That additional refind guard was not part of the isolated
numbers above. Damping minimum remains 0; no quality, map-insertion or solver
threshold was relaxed.

The rationale is correctness and bounded progress: retaining an introduced
cleanup starvation bug to obtain a smaller favorable valid-pose subset would
hide resource failures and leave bad map entries active. Restoring cleanup
does not justify accepting inaccurate GOOD poses as stable tracking. Actual
corrected-profile phone results, including the EuRoC drift/loss limitation,
must be measured and reported separately.

All five release and five ASan/UBSan regression targets pass on the integrated
profile. New tests exercise an actual pending Recent-BA short-circuit followed
by cleanup, and a textured successful refind followed by retirement and
failure-queue rejection. Immutable corrected host executable SHA256 is
`8e6d843cb8a15df592662169e143bde0c4d80989252cc29c2403f5a0598c1092`;
matching core archive SHA256 is
`bc95856d806080dde73ada983f053515698db7e9e78e545794430c4938c07448`.
This host identity is separate from both isolated executables and the upcoming
corrected iPhone build. See [phone validation](phone-validation.md) for its
eventual app/source identity and complete recorded-input results.

## Read-only diagnosis of the cleanup-only cam1 trajectory

Keep the cleanup-only prototype separate from the combined corrected profile.
Holding its Sim3 fixed from the 199 shared GOOD exposures (198 associated with
available reference poses), the 274 additional GOOD exposures have 6.36 m RMS
and 50.57° median orientation error. This uses no additional alignment to the
bad segments. Both recovery discontinuities and subsequent tracking of an
incorrect local map are visible:

| Prototype event | Fixed-common-gauge evidence |
| --- | --- |
| Frame 313 after three BAD frames | Recovery-path entry jumps 30.3° / 0.50 m and becomes raw GOOD with 42/137 matches; subsequent ten-frame segment remains about 0.59 m / 30.5° wrong |
| Frame 1548 → 1565 | Recovery exits weak with 2/7 matches and a 112° step. At 1564 raw quality 2 has only 1/3 matches yet adds a keyframe; 1565 then has 155/162 matches while 8.94 m / 86.1° wrong |
| Frame 3473 → 3474 | Raw quality 2 has 7/21 matches and adds a keyframe; the next frame has 108/126 matches while 4.04 m / 49.1° wrong |

This shows an internal support inconsistency: the app/evaluator exclude fewer
than ten matches, while raw tracker quality can still call those frames GOOD
and permit map insertion. It is an observation, not a new gate or a claimed
cause of every failure. The diagnostic scripts, fixed-gauge per-frame CSV and
event records are preserved as `cam1_recovery_diagnosis_v2.json`,
`cam1_fixed_common_gauge_per_frame_v2.csv` and `artifacts/diagnostics/`.

The combined corrected physical-phone cam1 run, executable
`d270bfc7e5e4c569e434af1582858f3b8581120971574bc4facbed60a00ce26c`,
has separately completed with 262 strict GOOD frames, 31 handled numerical
rejections and one retained stop. Its full accuracy assessment belongs in the
phone report. These different source/binary/platform observations do not alone
isolate causality of the retired-refind guard, and the cleanup-only prototype's
473 GOOD labels must not be assigned to the combined build.

An actual replay of the immutable combined macOS executable `8e6d843c…` later
exactly reproduced the 473-GOOD trajectory. The guard therefore does not
eliminate that trajectory on the host; the phone's different result cannot
establish guard causality. A separate all-input experiment aligning internal
quality with the existing ten-match output requirement is documented in
[minimum-support-experiment.md](minimum-support-experiment.md).
