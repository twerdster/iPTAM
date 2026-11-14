# Recorded-data results

The mobile variant runs on a physical iPhone and improves both simulated lounge
sequences. **Reliable tracking across real recordings remains unresolved.**
EuRoC-right accepts one frame, and the sparse real lounge never initializes.
A completed replay or a finite predicted pose does not mean successful tracking.

This report summarizes the frozen 13 November 2026 benchmark. The
[trial CSV](results/phone-trials.csv), [aggregate JSON](results/summary.json),
[build identities](results/builds.json) and [input catalog](results/frozen-input-plan.json)
provide compact, machine-readable details. Private recordings and raw logs are
not included.

## What was compared?

- **Original:** official Oxford PTAM-GPL r114 with build/input compatibility work.
  Its original asynchronous tracker/mapper algorithms are retained.
- **Restored:** the modernized iPTAM control, including stricter visual support
  and numerical checks, with serial mapping.
- **Mobile:** a modern-core implementation inspired by Klein and Murray's
  [2009 camera-phone paper](https://www.robots.ox.ac.uk/~dwm/Publications/Papers/klein_murray_ismar2009/klein_murray_ismar2009.pdf).
  This is not the unavailable historical iPhone source.

Every engine receives the same prepared 640×480 grayscale images, intrinsics
and exposure timestamps. Original/restored use frozen initialization pairs.
Mobile `auto` uses the same first image and selects its second view from image
content. The main table therefore compares **complete systems with different
initialization policies**, rather than isolating their trackers.

Only images, calibration and time enter estimation. Reference poses, ARKit,
IMU, depth and meshes never initialize or correct the estimator. Evaluation
reads references after replay. Camera permission and live capture were not used.

## Tracking coverage

Primary results always use the predeclared first repeat, rather than the best
repeat. Accepted means native GOOD, an initialized nonempty map, at least ten
raw matches and a finite rigid pose. Restored/mobile additionally require ten
final-pose robust contributors internally; equivalent original diagnostics are
unavailable, so original strict support is **unknown**, not zero.

| Input | Frames | Original accepted | Restored accepted | Mobile accepted | Mobile RMS | Mobile motion covered | Largest mobile gap¹ |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Historical simulated lounge | 230 | 57 | 44 | **142** | 11.08 cm | 58.27% | 62 frames |
| New simulated lounge | 100 | 33 | 71 | **82** | 2.39 cm | 95.16% | 1 frame |
| EuRoC MH01 left | 3,682 | 366 | 932 | 329 | 12.46 cm | 10.93% | 1,845 frames |
| EuRoC MH01 right | 3,682 | 557 | 234 | 1 | No fit | 0% adjacent motion | 3,584 frames |
| Sparse real lounge | 142 | 0 | 0 | 0 | No fit | Uninitialized | Whole input |

¹ Largest consecutive nonaccepted interval after first acceptance, including a
lost tail. Startup frames are separate. Motion coverage counts reference distance
between adjacent accepted exposures, relative to the whole reference path.

RMS is position error after one offline Sim3 fit, which accounts for monocular
scale. Each primary fit uses that engine's own accepted subset. **A small error
on a short subset does not establish whole-path accuracy.**

Both lounge image sequences are photo-fitted simulations, not actual scans.
The real lounge is 142 sparse RGB keyframes over 132 seconds, with 0.75–4.34-second
gaps. Its ARKit reference has uncertainty.
EuRoC left/right are separate monocular views of the same MH01 recording, not
independent environments.

## Repeats and equal-time accuracy

Three timed new-lounge runs accept exactly the same 82/100 exposures. Their RMS
is **2.39, 4.16 and 3.88 cm**. Wall-time mapping changes maps and poses, so matching
accepted masks do not make those runs bitwise deterministic. Three EuRoC-right
runs each accept one frame and exactly repeat the logged nontiming fields; they
repeat a tracking failure. All five unbudgeted automatic host pairs repeat exactly.

The following diagnostic fits each engine at exactly the intersection of its
logged accepted exposures. The intersection is chosen by visual status, never
reference error. It supplements the coverage table above.

| Input | Shared / full frames | Shared motion | Original RMS | Restored RMS | Mobile RMS |
| --- | ---: | ---: | ---: | ---: | ---: |
| Historical simulated lounge | 44/230 | 11.38% | 25.84 cm | 23.41 cm | 1.12 cm |
| New simulated lounge | 33/100 | 43.09% | 2.22 cm | 3.50 cm | 0.92 cm |
| EuRoC MH01 left | 326/3,682 | 10.89% | 1.76 cm | 1.78 cm | 7.79 cm |

Mobile is worse on the shared EuRoC-left interval. No three-way fit is available
for right or sparse lounge.

Two additional mobile `paper` runs hold the exact fixed initialization pairs:
new lounge accepts **0/100**, and EuRoC-right **63/3,682** with 6.46 cm RMS over
1.10% motion. The other three fixed-pair mobile phone cases were not run. All five
fixed-pair host cases were tested separately. These poor ablations remain separate
from automatic-system results.

## Runtime and bounded mapping

All phone runs follow the source exposure clock without dropping late frames.
Mobile mean foreground time is 7.58 ms on the new lounge and 2.91 ms on EuRoC-left.
Cheap lost/uninitialized frames lower whole-input averages; speed must be read
with coverage. Original tracker timing excludes concurrent mapper CPU, while
mobile/restored foreground timing includes serial mapping. Additional mobile
idle mapping is logged separately, so these timings are not equal total-cost
measurements.

Across nine mobile automatic trials, sampled resident memory peaks at **67.44 MiB**;
thermal state stays nominal. The active map peaks at **550 full/provisional points**
and **35 active keyframes**, with 14 retained keyframes. Historical lounge exercises
14 keyframe culls. All 130 deferred promotion tasks complete with none pending.
The 4 ms mapping budget is cooperative, not a hard deadline: a cull reaches
28.12 ms, and 600 idle-work overruns are recorded.

Rolling-shutter correction stays **off on every benchmark input** because verified
prepared-image readout timing is missing. Its enabled mathematics is tested with
synthetic fixtures; this does not establish real-recording correction accuracy.

## Validation and build identity

Validation includes **31 host replays / 47,116 frames** and **13 complete phone
replays / 22,964 frames**. Phone totals comprise nine automatic benchmarks, two
fixed-pair ablations and two separate handoff compatibility runs. The frozen
benchmark phone build starts `0c740562`; the handoff build starts `f4681a6e`.
Only malformed-input error receipts change between them; estimator sources and
the successful replay path are identical. Their measurements are not pooled.

Checks pass: **12 Release, 12 optimized ASan/UBSan, 24 Python, 5 mobile-disabled
Release and 10 physical malformed-input/background checks**. A focused mapping/
reset leak check reports zero bytes. Handoff compatibility runs preserve the
82-frame lounge mask and one-frame right-camera mask. Raw logs, complete source
snapshots and binaries are retained separately. Shared executable and focused
test-receipt hashes are recorded in [builds.json](results/builds.json).

The example animation visualizes saved phone results offline. It is not a phone
screen capture, a measured-FPS demonstration or evidence that the real-data
failures above have been solved.
