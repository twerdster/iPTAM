# Internal minimum-support consistency

> Detailed validation record. See [current results](../RESULTS.md) for the main comparison.
> `evidence/` labels identify local validation artifacts; raw logs, app binaries
> and private inputs are not redistributed with this repository.

The app and evaluator already require at least ten found matches for a valid
visual pose. The tracker could nevertheless label one match out of three as
GOOD and permit keyframe insertion. A one-line isolated change makes
`AssessTrackingQuality` report BAD when `nTotalFound < 10`, replacing its
previous `nTotalFound == 0` test. No other threshold, recovery gate, camera,
mapping schedule or solver rule changes.

This tests an existing support contract, rather than selecting a new cutoff
from reference errors. The policy was fixed before running; reference poses
enter independent evaluation only. Both sides use the combined mandatory
cleanup/retired-refind guard, minimum damping 0 and all camera/tracker/solver
safety fixes, with fixed existing pairs: native historical 0/16, native new
0/18, EuRoC left 1200/1205 and right 0/5.

## Reproducibility and baseline confirmation

Evidence is under
`evidence/experiments/minimum_support_production_v2`.
`plan.json`, per-run commands/status/logs/CSV/metrics, `comparison.json` and
`artifacts/sha256.json` preserve the complete experiment. Copied Tracker and
harness objects use production flags `-O3 -DNDEBUG -std=gnu++14 -arch arm64`,
with the same headless definitions and immutable combined core archive:
`bc95856d806080dde73ada983f053515698db7e9e78e545794430c4938c07448`.
At most two trials ran concurrently; each had a 120-second external bound.

| Artifact | SHA256 |
| --- | --- |
| Unchanged copied Tracker source | `377f19a7dfa16cc9cc4ce1b0e432f648f1c50ef05924a45dde84a720085fabac` |
| Minimum-ten Tracker source | `f2b3d91dc1af5445cfdd1f5c2c6709341bdef3e56aedb67247e6b1f7ba705c4c` |
| Production-flag copied control executable | `f1ee474416bd85e127e2d066e875df192a6f77a52eca1a7ef6a38ebaeffe9065` |
| Production-flag minimum-ten executable | `9c142faad9721b85d62f8b8ddb732baeb7d6baec511af4992c7f2c75ea3cb69f` |

The actual frozen canonical macOS executable
`8e6d843cb8a15df592662169e143bde0c4d80989252cc29c2403f5a0598c1092`
was independently replayed on full cam1. Its serialized pose/state fields
exactly match the copied control: 473 GOOD poses and 2.958 m fitted RMS.
`frozen_baseline_confirmation.json` records the comparison. This establishes
that the baseline association is correct; the phone's 262 GOOD result cannot
be attributed to a missing retired-refind guard in the host control.

An earlier `minimum_support_v1` suite compiled the copied objects at `-O2`
without `NDEBUG` against the Release archive. That mixed configuration is
retained separately. Repeating all eight trials with matched production flags
produces exactly the same full serialized non-timing state fields;
`compile_profile_comparison.json` records the comparison. Production results,
not assumptions about the earlier compile configuration, support the table.

## All four complete inputs

Every control and variant completed within the bound, with all input rows and
no nonfinite/nonrigid states. Strict GOOD still uses the same output contract.
RMS is fitted monocular Sim3 on the valid subset; missing motion remains a
failure and is reported separately.

| Input | Strict GOOD, control → minimum ten | Position RMS, control → minimum ten | Reference motion coverage | Largest GOOD gap, control → minimum ten |
| --- | --- | --- | --- | --- |
| Native historical lounge | 89/230 → 89/230 | 5.654 → 5.654 mm | 33.40% unchanged | 124 → 124 |
| Native new lounge | 81/100 → 81/100 | 5.169 → 5.169 mm | 95.62% unchanged | 0 → 0 |
| EuRoC left | 597/3682 → 597/3682 | 45.265 → 45.265 mm | 21.81% unchanged | 892 → 892 |
| EuRoC right | 473/3682 → 237/3682 | 2957.82 → 87.62 mm | 10.74% → 3.42% | 563 → 2593 |

The first three trajectories/map states are exactly unchanged. Their controls
never entered a post-init raw GOOD state with fewer than ten matches.

Cam1 control contains 233 post-init raw GOOD frames below ten matches and
five keyframe insertions below that minimum: frames 199, 374, 1564, 2202 and
3473. The variant has none. It avoids the grossly inaccurate additional
segments, but has fewer valid frames and a long lost interval; it does not
solve real-sequence coverage. Cam1 median absolute orientation error is
151.22° → 4.56°, adjacent rotation median 0.0968° → 0.0346°, maximum
keyframes 53 → 25 and handled numerical rejections 21 → 1. Mean host
processing is 9.86 → 2.31 ms, with no phone performance inference.

A concrete control example is frame 1564: raw quality 2 with only **1/3**
matches adds keyframe 31. The next frame has **155/162** matches against the
new local map while remaining 8.94 m / 86.1° wrong under the fixed alignment
fitted only to shared GOOD reference exposures. This is the same seeded-map
failure diagnosed in the cleanup experiment; the support variant prevents
the sub-ten insertion without modifying any temporal recovery rule.

The result supports making internal quality/map insertion consistent with the
already enforced output support requirement. It does not support adding other
temporal recovery gates or claiming stable full-sequence tracking. The one-line
policy is now integrated canonically. Boundary/recovery regression cases
exercise 1/3, 3/3 and 9/9 as BAD with advancing loss state, then 10/30 as GOOD
with loss state cleared; the complete checks and actual-phone verification
are being finalized for the new frozen identity. The pre-change phone build
`d270bfc7e5e4c569e434af1582858f3b8581120971574bc4facbed60a00ce26c`
has a different measured cam1 trajectory (262 GOOD); host numbers must not be
substituted for its result or for the next build's result.
