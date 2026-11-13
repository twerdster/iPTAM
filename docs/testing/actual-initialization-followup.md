# Actual lounge initialization follow-up

> Detailed validation record. See [current results](../RESULTS.md) for the main comparison.
> `evidence/` labels identify local validation artifacts; raw logs, app binaries
> and private inputs are not redistributed with this repository.

The actual lounge failure at fixed exposures 47/48 is first a correspondence
search failure: the original initializer retains only 3 of 148 trails and resets
before homography estimation or bundle adjustment. An isolated, image-only
pyramidal LK fallback reaches the original mapper and initializes, but tracks
only exposures 49–52. It does not solve the recording or justify promotion to
the default. No candidate in this report was integrated into canonical code.

These tests use the exact minimum-support host baseline
`a319cbc477f734d08cf4fbf1690b2ff6bb2673adaba7881dd9161dc9f7717654`,
with default damping minimum 0, bounded transactional solves and mandatory mapper
cleanup. They precede the later final-pose-support change. All 142 actual
exposures remain in the denominator. ARKit poses are evaluation-only device
references, not independent ground truth; no pose, depth, mesh or reference file
feeds a candidate. The pair was fixed before this follow-up, not selected by
trajectory accuracy. See [actual-lounge-data.md](actual-lounge-data.md) for
calibration and [initialization-experiments.md](initialization-experiments.md)
for the earlier, unsuccessful general LK experiment.

## Failure stages at 47/48

The prepared images are 640×480 with common pinhole K
`[460,460,319.5,239.5]`; the interval is 0.750302 s. Native `MiniPatch` uses a
9×9 patch, ±10-pixel corner search, the existing SSD threshold and a backward
check whose squared return error must be ≤2. An actual-API probe gives:

| Search radius | Seeds | Forward SSD passes | Backward SSD passes | Married matches |
| --- | --- | --- | --- | --- |
| 10, original | 148 | 6 | 6 | 3 |
| 20, diagnostic | 148 | 30 | 30 | 10 |
| 40, diagnostic | 148 | 116 | 116 | 57 |
| 80, diagnostic | 148 | 138 | 138 | 69 |

`TrailTracking_Advance()` itself returns 3 on the original path. The following
`nGoodTrails < 10` branch resets the tracker, so homography, triangulation and BA
are never called. The physical-phone baseline shows the same seed/reset sequence
and zero numerical failure counters. Independent image-only analysis found about
45.5 pixels median motion and substantial geometric support, so missing texture
is not an adequate explanation.

The wider-search and LK correspondences were then passed through production-order
initialization, retaining the original homography, triangulation, subpixel patch
refinement, map expansion and solver acceptance rules:

| Correspondences | Input / H inliers | Subpixel / depth rejects | Initial / expanded points | Final full BA | Result |
| --- | --- | --- | --- | --- | --- |
| Native ±20 | 10 / 5 | 9 / 0 | 1 / — | not reached | rejected |
| Native ±40 | 57 / 24 | 27 / 3 | 27 / 205 | 20-call bound; not converged | rejected |
| Native ±80 | 69 / 36 | 29 / 3 | 37 / 199 | 20-call bound; not converged | rejected |
| Strict dense LK | 42 / 36 | 19 / 0 | 23 / 234 | converged in 3 calls | initialized |

The wider runs have zero hard numerical failures and retain 8/6 bounded solver
stops respectively, but the existing final convergence requirement rejects them.
This experiment does not relax it. Mandatory cleanup leaves 113 active points
in the accepted LK map; expanded counts include points subsequently retired.

The dependency-free LK prototype uses four pyramid levels, a 9×9 mean-subtracted
patch, at most 25 iterations per level, mandatory convergence, finite/border and
Hessian checks, RMS ≤25 and forward/backward error ≤1 pixel. Spatially separated
Shi–Tomasi seeds give 316 candidates, 85 forward passes, 53 backward passes and
42 married pairs. Pixel coordinates are rounded for the original integer-pair
mapper API, whose own subpixel refinement remains enabled. Candidate fallback
is attempted only when the original trails fall below the existing minimum 10.

## Full-input controls

Each run has a 120-second watchdog. All completed. State equality below compares
every serialized CSV field except elapsed processing time, not just the count
of GOOD rows. The wider and LK fallback branches are never reached on the other
four fixed-pair inputs, whose complete states match their native controls.

| Input; fixed pair | Original GOOD / input | ±80 fallback | LK fallback | LK fitted RMS / reference motion covered |
| --- | --- | --- | --- | --- |
| Historical native lounge; 0/16 | 89/230 | exact original states | exact original states | 5.65 mm / 33.40% |
| New native lounge; 0/18 | 81/100 | exact original states | exact original states | 5.17 mm / 95.62% |
| EuRoC cam0; 1200/1205 | 597/3682 | exact original states | exact original states | 45.26 mm / 21.81% |
| EuRoC cam1; 0/5 | 237/3682 | exact original states | exact original states | 87.62 mm / 3.42% |
| Actual lounge; 47/48 | 0/142 | 0/142; BA rejects | 4/142 | 46.96 mm / 1.48% |

The actual LK run has 2 weak outputs, an 89-exposure final gap (53–141), zero
hard solver failures and no nonfinite/nonrigid saved transform. The four GOOD
poses have adjacent rotation RMS 1.82°. Its fitted-scale RMS describes only
those four poses and cannot establish stable tracking of the room.

## Immediate tracking loss and gap/coarse control

The accepted LK map gains a third keyframe at 52. At 53 it still has 205
potentially visible points, no rejected warp/template, and 49/205 actual matches;
quality becomes weak. At 54 there are 182 attempted points but only 18 matches,
including 0/34 at the two highest pyramid levels, and quality becomes BAD.
Recovery attempts start at 57 and never restore strict tracking. This is a
matching/pose-association collapse, not an empty-map or hard solver rejection.

A logging-only copy confirms a concrete timing interaction: every actual image
interval exceeds 0.5 s, so the timestamp safety branch clears velocity and its
depth-scaled magnitude. That magnitude also controls the original coarse-stage
trigger. Normal tracking therefore skips coarse search despite large image
motion; recovery explicitly enables it later.

One isolated control retained the same LK initialization and enabled the
existing coarse stage after the same >0.5 s gap, without changing search ranges,
quality thresholds or map/solver rules. It improves frame 53 to 75/178 matches
and GOOD, but frame 54 still collapses to 9/157. It produces only 5/142 GOOD
outputs, 28.99 mm subset RMS and 2.24% reference motion coverage, followed by an
88-frame gap. This demonstrates a limited contribution of coarse scheduling;
it does not explain or repair the entire loss and was not promoted.

An independent image-only audit of the fixed loss boundary finds 31 essential
inliers at 52→53 and 25 descriptor fundamental-matrix inliers at 53→54. Estimated
image rotation rises from 8.56° to about 19.84°, while sharpness drops. All three
images retain corner coverage across all 12 spatial tiles. Thus significant
view change and blur coincide with failure, but overlap still exists; the
evidence does not support declaring the recording featureless or unobservable.

## Reproducibility and invalidated probe

The complete archive is
`evidence/experiments/actual_initialization_followup_20261113/`.
It includes exact baseline library/headers/sources under `base-core/`, isolated
source overrides, executables, all failure logs/trajectories, `aggregate.json`,
per-file `artifact_hashes.json` and a bounded `reproduce.py`. Compilation uses
Apple Silicon `clang++ -O3 -DNDEBUG -std=gnu++14`, headless core flags and
Accelerate, outside the canonical tree. Source/binary associations include:

| Profile | Executable SHA256 |
| --- | --- |
| Corrected injected driver | `a9519ed3082927f21b58d4037cb4b4bb735816ee4f36a149b8b9869bf5533ff7` |
| ±80 fallback, normal TrackFrame | `23017b1a410b42b8cb5d246b9ced07ccfbe3faf5e91a6b44cdbdd951f9375171` |
| Strict LK fallback, normal TrackFrame | `a389181122d3a12e800100bcfb0cafe63aa241270e66dd25bba4c60ac5cd2b1d` |
| LK plus gap→coarse diagnostic | `03bad818cd20b7b4389953c1db7bd113d04ed0e3a0ca3627fbf8a9695cf6839f` |

The baseline static library SHA256 is
`84a8d638b76305dafc5dee40fc07c3c96d53a16d0a5165195962772ead26a2f0`.
The LK tracker source hash is
`b60973b2e2c3fcdb645233cac1906d912eaf50d842806fad3983a5ef0249c949`;
its helper hash is
`fdfbe58a16823cc674b8d3ed4aac7263a342e6b63f930f2bbee1b0515339c2d7`.

The first injected probe is **invalidated**: its driver called
`mCurrentKF.MakeKeyFrame_Rest()` before the mapper. That method appends candidates,
so the extra call changed candidate multiplicity and mapping behavior. The
driver also copied depth statistics unlike the original initialization path.
This made the first wider-search probes appear to initialize. Correcting both
differences makes those wider probes reject, as the native fallback does.
The corrected LK fallback still initializes and still produces only four GOOD
exposures. Both driver sources/binaries and invalid outputs are retained and
explicitly listed in `artifacts/INVALIDATED_INITIAL_PROBES.md`; they are not
evidence for unchanged production behavior. The normal-TrackFrame full-input
controls never execute that injected branch and are unaffected.

Independent loss-boundary audit:
`testing/host/actual_lounge_52_54_image_support_20261113.json` in the local evidence archive.
The next initializer/frontend work would need to improve sustained image
association and demonstrate all-input accuracy/coverage. More bootstrap matches
or a lower subset error alone are insufficient.
