# Mobile PTAM: physical iPhone comparison

> Detailed validation record. See [current results](../RESULTS.md) for the main comparison.
> `evidence/` labels identify local validation artifacts; raw logs, app binaries
> and private inputs are not redistributed with this repository.

The paper-inspired mobile system is implemented, installed and tested on a physical
iPhone 12. **All nine retained automatic-profile trials processed their complete
inputs: 15,400 finite, rigid pose rows, with no numerical failure.** The simulated
lounges improve over the frozen controls, but **general stable tracking is not
achieved**: EuRoC-right accepts one frame and actual sparse lounge never
initializes. Completion and finite prediction are not successful tracking.

## Identity, scope and original intent

Branch `mobile-paper-variant` starts at modern commit `2c11b6a`. Benchmark phone executable SHA256 is
`0c740562b775e644bf7cb2bbd73c3568bcedbfc904776d3f22b9b214c32d96f7`;
host is `694807b9c7081caa61fa885050e99235ca4416f40d9d4f20c60316a522be3638`.
Exact source ZIP/hashes, signed app and build log are archived under
`evidence/mobile_paper_20261113/builds/phone_deferred-0c740562b775e644/`.
Benchmark sources remain frozen in that archive. Device class: iPhone 12 (iPhone13,2), iOS 18.0.1. The mobile app is a separate target.
Canonical current/upstream source and benchmark apps remain unchanged.

The installed handoff executable is
`f4681a6e2d9583e3a49470943903fcf006bccf7370be7d0d6fcb2fe55e174dbe`,
archived in `builds/phone_receipt-f4681a6e2d9583e3/` under the same evidence root.
Its sole compiled-file change is `ios/App.mm`: malformed mobile configuration
now records the manifest identity and an explicit no-camera-request flag before
reporting failure. All estimator sources and the successful replay flow are
unchanged. Benchmark metrics/videos below stay attributed to `0c740562…`;
handoff-build checks are reported separately.

Context: `SUMMARY.md`, `SKILL.md`, [original-report analysis](../report-analysis.md),
[dataset inventory](../dataset-inventory.md), and Klein/Murray's
[ISMAR2009 camera-phone paper](https://www.robots.ox.ac.uk/~dwm/Publications/Papers/klein_murray_ismar2009/klein_murray_ismar2009.pdf).
The 2010 report describes initialization followed by failed map-point
matching and disabled recovery, leaving motion prediction as the demonstration.
The restoration criterion is therefore measured visual correction, continuing
map growth, honest loss/recovery and repeatability. No ARKit pose, depth or IMU
feeds the estimator. This implements the later paper's ideas on the restored
core; it is not the unavailable historical commercial iPhone executable.

The mobile feature set includes five-level multiscale Shi–Tomasi detection,
coarse exhaustive normalized patch search/ESM, blur fallback, partial inverse
depth with measured triangulation on promotion, serial idle mapping, bounded
observation/keyframe graphs and image-only automatic planar initialization.
Rolling-shutter math is tested synthetically; every actual input lacks verified
prepared-image readout timing, so correction stays disabled. See the
[implementation](mobile-implementation.md), [audit](mobile-code-audit.md) and
[mapper results](mobile-mapping-results.md) for adaptations and ownership limits.
Only existing recordings were used; no camera permission or capture is required.

## Frozen inputs and comparison policy

The [public input catalog](../results/frozen-input-plan.json) describes the
five retained full prepared inputs. All are 640×480 pinhole, with identical pixels,
calibration and exposure clocks across engines. Both lounge trajectories are
photo-fitted simulations. The actual lounge export has only 142 sparse RGB
keyframes over 132 seconds, with 0.75–4.34-second gaps; no continuous lounge RGB
recording is available in the supplied export. Its ARKit poses are uncertain
references. EuRoC cameras are two
monocular views of the same MH01 recording, not independent environments.

Upstream/current use frozen fixed initialization pairs. Mobile `auto` keeps the
same predeclared first image and selects the second using image evidence. This
is a **complete-system comparison with different initialization policies**, not
an isolated tracker comparison. Mobile automatic trials cannot enter fixed-pair
isolation reports. References are evaluated only after replay, never used to
choose a start, hypothesis or preferred repeat. Primary repeat01 is predeclared.

Common GOOD requires native quality GOOD, initialized nonempty map, at least ten
raw matches and a finite rigid pose. Mobile/current also enforce at least ten
final-pose robust contributors internally; upstream does not export equivalent
support, so its strict quality remains unknown. One offline Sim3 fits each
accepted subset. Report coverage/gaps with error; low subset error cannot
establish performance over an untracked path.

## Automatic-profile results

Numbers use rep01 throughout. Upstream/current are the unchanged saved physical
controls, not reruns selected for a favorable outcome. Mobile common and strict
GOOD counts agree.

| Input | Frames | Original GOOD | Restored GOOD | Mobile GOOD | Mobile RMS m | Mobile motion coverage | Largest mobile gap frames |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Historical simulated lounge | 230 | 57 | 44 | 142 | 0.11077 | 58.27% | 62 |
| New simulated lounge | 100 | 33 | 71 | 82 | 0.02394 | 95.16% | 1 |
| EuRoC MH01 left | 3682 | 366 | 932 | 329 | 0.12462 | 10.93% | 1845 |
| EuRoC MH01 right | 3682 | 557 | 234 | 1 | no fit | 0 adjacent GOOD motion | 3584 |
| Actual sparse lounge | 142 | 0 | 0 | 0 | no fit | uninitialized | whole input |

Mobile right-camera support is one exposure at 97 and no usable fit; its fast
replay is a tracking failure. Actual lounge remains uninitialized on all three
systems.

Three new-lounge runs each accept exactly the same 82/100 exposures, with a weak
frame at 61. Their fitted RMS values are 2.394, 4.163 and 3.876 cm. Timed mapping makes
their maps and poses differ; do not call them bitwise repeatable. Three
EuRoC-right runs each accept one frame and match all 21 non-timing CSV fields
exactly. Exact repetition of a failed track does not establish robustness.
All five unbudgeted host repeat pairs are exact; they are a distinct scheduling
control, documented in [host validation](mobile-host-validation.md).

The supplemental three-engine shared-time fits use exactly the intersection of
logged GOOD exposures, without selecting by reference error. New lounge shares
33/100 exposures and 43.09% reference motion: original/restored/mobile RMS is
2.225/3.501/0.921 cm. Historical shares 44/230: 25.836/23.407/1.121 cm. EuRoC-left
shares 326/3682 and 10.89% motion: 1.757/1.780/7.789 cm. These supplement whole-input
coverage; mobile is worse on this real-data interval. There is no three-way
shared fit for right or actual lounge.

## Separate fixed-pair ablation

Two additional full phone trials on the benchmark app use
`--mobile-profile paper`, the exact frozen pairs and the same source schedule. They process all
3,782 exposures with finite rigid output. New lounge accepts 0/100 because its
local initialization trails reset before the fixed second image; the image-only
flow audit and eight-root-pixel search limit are documented in the host report.
EuRoC-right accepts 63/3682, with 6.458 cm RMS over 1.10% reference motion. Its
55-exposure three-way shared-time fit covers 0.86% motion and gives
original/restored/mobile RMS 0.482/0.727/6.872 cm. These are tracking failures too;
they do not turn the automatic system's different second-view choice into a
fair fixed-pair isolation comparison.

Raw trials are in `phone_deferred_paper/`; the explicitly fixed-policy report is
`final_fixed_pair_report/`. Its other three mobile cases are intentionally absent,
not inferred results. All five fixed-pair host cases were tested twice separately.

## Pacing, resources and mapping

All phone trials use source-paced delivery, keep every exposure in order and
log late delivery. Times include foreground tracker/serial mapper plus export;
extra idle mapping is logged separately. Upstream reports tracker latency while
its mapper runs concurrently, so its latency is not equivalent total CPU work.
Whole-input means also include cheap lost/uninitialized frames.

| Mobile input / rep01 | Mean foreground ms | Maximum ms | Wall seconds | Maximum lag ms | Sampled peak RSS MiB |
| --- | ---: | ---: | ---: | ---: | ---: |
| Historical lounge | 8.165 | 42.207 | 22.912 | 11.754 | 67.44 |
| New lounge | 7.583 | 57.802 | 9.915 | 11.938 | 52.52 |
| EuRoC left | 2.908 | 36.942 | 184.058 | 9.190 | 62.23 |
| EuRoC right | 0.865 | 25.042 | 184.056 | 22.852 | 44.34 |
| Actual sparse lounge | 13.754 | 40.944 | 131.955 | 245.836 | 42.14 |

Every final automatic run remains thermal 0/nominal with low-power mode off.
Across repeats the maximum foreground frame is 60.501 ms, sampled RSS 67.44 MiB and
physical footprint 15.11 MiB. These are sampled app memory measurements, not a
proof of peak allocation within an operation. EuRoC-left stays on its 184.05 s
source clock, while the restored control takes 386.33 s with 223.76 s maximum lag;
mobile also accepts fewer poses and has worse shared-time error.

All 15,400 retained frame diagnostics independently satisfy task ownership/accounting and
configured active limits. All 130 retained deferred promotion tasks complete, none remain
pending; historical lounge actually exercises 14 keyframe culls and retained
image owners. Peaks are 550 combined full/provisional points and 35 active plus 14
retained keyframes. The 4 ms mapping budget is cooperative: indivisible operations
overrun it, including a measured 28.12 ms cull. It is not a hard real-time guarantee.

The pre-fix phone control `c0a67492…` is retained separately. Its keyframe
insertion consumed the mapping quantum before its only partial promotion loop,
so new lounge had 23 GOOD and zero promotions in each repeat. The final mapper
retains bounded candidate-ID tasks for a fresh quantum; new lounge now promotes
101/100/96 points. Geometry/support gates were preserved. Older 82c2b707… and
c0a67492… apps are not pooled into this final result.

## Final app checks

The handoff app `f4681a6e…` passes all ten physical checks in
`phone_lifecycle_receipt_fixed/results.json`: malformed root/frame/clock/lens,
missing and truncated images, three invalid rolling-shutter timing declarations,
and actual OS background cancellation. Each rejected input saves its exact
manifest hash with zero processed frames. Backgrounding an initialized,
source-paced lounge replay saves 41 frames / 24 accepted poses and delayed work
does not revive it. The earlier failed hash assertion remains in
`phone_lifecycle_final/`, with the correctly rejected input preserved.

Two additional full source-paced compatibility replays on this app process all
3,782 images. New lounge again accepts 82/100 with the identical accepted mask;
its wall-budgeted poses differ on 67 rows. EuRoC-right again accepts 1/3682 and
matches all 21 non-timing CSV fields exactly. Manifest bytes, complete consumed
pixel hashes and app identity are verified in
`phone_receipt_compatibility/compatibility-receipt.json`. These are separate
handoff checks; their outputs are not substituted into benchmark videos/errors.

Default launch opens the recording library with
`camera_permission_requested=false`.
The read-only receipt is `phone_library_final/result.json`. No camera access was
requested. The handoff build does not rerun all five retained complete inputs; its unchanged
estimator is covered by the retained five-input benchmark and these two complete
successful-path controls. This distinction is explicit rather than claiming
every measurement belongs to the same executable.

Final focused suites pass: 12 Release, 12 optimized ASan/UBSan, 5 macro-disabled Release tests. Apple leaks finds zero leaked bytes in the focused
mapping/reset fixture. These and the 13 retained complete phone replays (22,964 images
across separately identified builds/policies) do not remove the real-data
tracking failures above.

## Reproduction and evidence

The local evidence root is labeled
`evidence/mobile_paper_20261113`:

- `phone_deferred_auto/`: all nine retained trial outputs, commands, app identities,
  complete pixel hashes, frame logs and pipeline-validation receipts.
- `final_system_report/report.json` and `report.md`: the 24 retained saved baseline/mobile
  trials, primary/shared-time evaluations and missing diagnostics.
- `mapping_analysis/phone_deferred_20261113/`: independent per-frame mapper audit.
- `phone_deferred_auto_independent_audit/audit.json`: independent complete-input,
  app, pose and accepted-mask verification of all nine retained automatic benchmark runs.
- `videos_final/`: predeclared rep01 three-way new-lounge and EuRoC-right clips,
  exact renderer/input hashes, ffprobe metadata and sampled previews. These are
  offline visualizations of phone logs, not phone screen capture or measured FPS.

```bash
.venv/bin/python tools/compare_mobile_phone.py \
  --baseline-root evidence/upstream_comparison/phone_20261113 \
  --mobile-root evidence/mobile_paper_20261113/phone_deferred_auto \
  --output evidence/FRESH_REPORT --automatic-system-comparison
.venv/bin/python tools/render_upstream_comparison.py \
  --report evidence/FRESH_REPORT/report.json --output evidence/FRESH_VIDEOS \
  --engines upstream current mobile --system-comparison --fps 20
```

Raw validation outputs are retained locally. Public aggregates are linked in
[the results report](../RESULTS.md). No training job was part of this validation.
