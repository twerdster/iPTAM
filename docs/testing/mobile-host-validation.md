# Mobile variant: final host validation

> Detailed validation record. See [current results](../RESULTS.md) for the main comparison.
> `evidence/` labels identify local validation artifacts; raw logs, app binaries
> and private inputs are not redistributed with this repository.

All **31 corrected-profile replays processed their complete prepared inputs: 47,116 rows, zero nonfinite poses and zero nonrigid poses**. This comprises 30 repeated fast replays and one source-paced new-lounge control. Long tracking gaps remain on historical lounge, both EuRoC views; the sparse actual lounge never initializes. Safe complete processing does not establish general stable scanning.

Frozen host SHA256: `694807b9c7081caa61fa885050e99235ca4416f40d9d4f20c60316a522be3638`. This report uses only `host_deferred_timed`, `host_deferred_deterministic` and `host_deferred_source_paced`. The machine summary (local evidence `evidence/mobile_paper_20261113/host_report_deferred_final/summary.json`) contains independently recomputed evaluations, all trial/resource measurements, gaps and artifact hashes.

## Inputs and policies

Every manifest and actual concatenated grayscale pixel hash was verified before replay. All five inputs preserve the full prepared chronology at 640×480: two lounge simulations with simulation truth, two monocular cameras from the same EuRoC MH01 recording with dataset ground truth, 142 sparse actual-lounge keyframes over about 132 seconds. The real lounge ARKit poses are uncertain offline references. Images, calibration and exposure clocks feed the estimator; reference poses, IMU and depth do not.

Fixed first/second indices are historical 0/16, new 0/18, EuRoC-left 1200/1205, right 0/5, actual 47/48. Automatic mode retains that first image and selects its second view from image evidence. It is a separate initialization policy. Coverage always uses the full prepared input, including images before initialization.

The repeated arm64 macOS 15.1.1 suites run as fast as possible. “Timed” means a cooperative 4 ms serial-mapping budget, not source-paced delivery. The deterministic control passes `--mobile-idle-mapping 0`, removing wall-time deadlines while retaining serial mapping. The additional source-paced control permits mapping during available idle slots. Rolling-shutter correction is disabled for all five inputs because verified prepared-image readout timing is unavailable.

## Coverage and accuracy

GOOD requires quality 2, a nonempty map, at least 10 raw matches, at least 10 final-pose robust contributors and a finite rigid pose. Weak quality 1 is separate. One Sim3 fits reference-associated GOOD poses per trial; RMS describes that observed subset. Motion coverage counts adjacent GOOD reference movement without joining gaps. Different accepted subsets prevent interpreting lower RMS alone as an accuracy improvement.

Tables show repeat01/repeat02; a single value means rounded displayed values agree. Gap is the largest interval without GOOD after its first occurrence, including weak states and lost tails. “—” indicates no GOOD output.

### Timed mapping, fixed initialization (`paper`)

| Prepared input | GOOD / full input | Weak | RMS m | Motion % | Max gap frames |
| --- | ---: | ---: | ---: | ---: | ---: |
| Historical simulated lounge | 0/230 | 0 | — | — | — |
| New simulated lounge | 0/100 | 0 | — | — | — |
| EuRoC MH01 left | 221/3682; 252/3682 | 2/9 | 0.03471/0.04328 | 7.783/8.71 | 2254/2224 |
| EuRoC MH01 right | 63/3682 | 0 | 0.05588 | 1.097 | 3486 |
| Actual lounge (sparse) | 0/142 | 0 | — | — | — |

### Timed mapping, automatic second view (`auto`)

| Prepared input | GOOD / full input | Weak | RMS m | Motion % | Max gap frames |
| --- | ---: | ---: | ---: | ---: | ---: |
| Historical simulated lounge | 61/230 | 3 | 0.02345 | 18.51 | 152 |
| New simulated lounge | 82/100 | 1 | 0.01596/0.01595 | 95.89 | 1 |
| EuRoC MH01 left | 238/3682 | 1 | 0.1769 | 8.105 | 2235 |
| EuRoC MH01 right | 3/3682 | 0 | 0.00624 | 0 | 3584 |
| Actual lounge (sparse) | 0/142 | 0 | — | — | — |

### Deterministic mapping control, automatic second view

| Prepared input | GOOD / full input | Weak | RMS m | Motion % | Max gap frames |
| --- | ---: | ---: | ---: | ---: | ---: |
| Historical simulated lounge | 61/230 | 2 | 0.02352 | 18.51 | 152 |
| New simulated lounge | 82/100 | 1 | 0.01595 | 95.89 | 1 |
| EuRoC MH01 left | 246/3682 | 2 | 0.2268 | 8.393 | 2229 |
| EuRoC MH01 right | 3/3682 | 0 | 0.00624 | 0 | 3584 |
| Actual lounge (sparse) | 0/142 | 0 | — | — | — |

### Source-paced new-lounge control

The single full source-paced run produces **83/100 GOOD**, no weak output or post-initialization gap, **98.02% reference-motion coverage** and **1.237 cm Sim3 RMS**. Relative to the frozen index 18 comparison anchor, it has 81/81 GOOD versus 80/81 in the fast runs; automatic initialization actually precedes that anchor. Whole-input counts remain 83/100 versus 82/100. This is a scheduling control on the same corrected binary, not a repeated or physical-device result.

The right-camera automatic result has only three isolated GOOD exposures, zero adjacent GOOD motion coverage and a 3,584-frame gap; its 6.24 mm fitted RMS does not demonstrate tracking. Historical automatic output has a 152-frame gap. Actual lounge remains uninitialized in both policies.

All five deterministic automatic pairs match exactly in all 25 serialized CSV fields other than `elapsed_ms`. Six of 10 timed pairs match exactly. Timed historical-auto, new-auto and EuRoC-left-auto keep the same GOOD counts but differ in serialized states; left-camera fixed-pair counts vary from 221 to 252. Wall-time scheduling remains load-dependent. These same-binary host checks do not assert cross-device identity.

## Runtime and memory

Means use the predeclared repeat01; p95, worst frame and peak RSS take the larger value across both repetitions. “All/GOOD” distinguishes every frame from supported tracking frames. Large lost/uninitialized intervals can make overall averages misleading. `elapsed_ms` covers timestamp handling and the tracker call, including serial mapping, and excludes image loading/export and extra idle-slot mapping. Source-paced wall time includes those stages and intentional waits. Host measurements are not phone FPS.

| Mode | Prepared input | Mean all/GOOD ms | p95 ms | Worst ms | Peak RSS MiB |
| --- | --- | ---: | ---: | ---: | ---: |
| Timed fixed | Historical simulated lounge | 0.0598/— | 0.5495 | 1.283 | 9.81 |
| Timed fixed | New simulated lounge | 0.1059/— | 0.4664 | 1.207 | 6.59 |
| Timed fixed | EuRoC MH01 left | 0.2881/1.028 | 0.5306 | 4.802 | 41.05 |
| Timed fixed | EuRoC MH01 right | 0.1403/0.4527 | 0.1753 | 4.941 | 16.23 |
| Timed fixed | Actual lounge (sparse) | 0.02741/— | 0.02123 | 1.152 | 7.23 |
| Timed auto | Historical simulated lounge | 0.876/1.473 | 2.6 | 11.25 | 23.62 |
| Timed auto | New simulated lounge | 1.462/1.076 | 4.39 | 17.07 | 24.45 |
| Timed auto | EuRoC MH01 left | 0.2964/1.054 | 0.5941 | 11.91 | 33.27 |
| Timed auto | EuRoC MH01 right | 0.1375/0.1294 | 0.1109 | 9.244 | 15.00 |
| Timed auto | Actual lounge (sparse) | 1.915/— | 5.039 | 5.773 | 10.41 |
| Deterministic auto | Historical simulated lounge | 0.8765/1.505 | 2.641 | 11.31 | 25.50 |
| Deterministic auto | New simulated lounge | 1.453/1.084 | 4.278 | 16.59 | 20.34 |
| Deterministic auto | EuRoC MH01 left | 0.3056/1.098 | 0.5528 | 11.59 | 28.59 |
| Deterministic auto | EuRoC MH01 right | 0.136/0.1336 | 0.1056 | 9.145 | 14.66 |
| Deterministic auto | Actual lounge (sparse) | 1.917/— | 5.03 | 5.778 | 12.67 |
| Source-paced auto | New simulated lounge | 1.636/1.157 | 3.16 | 17.71 | 26.23 |

Peak RSS is **41.05 MiB** and the worst measured frame is **17.71 ms**. Source-paced wall time is 9.981 s with 10.47 ms maximum schedule lag. Small active graphs and extensive tracking failures limit conclusions about sustained scanning resources. Accepted-iterate retained stops indicate bounded termination, not convergence.

## Checks and retained diagnosis

Final logs report **12/12 Release tests (4.60 s), 12/12 optimized ASan/UBSan tests (37.52 s)**. Focused tests retain geometric-support checks, actual-image plane tracking, reset/loss/recovery, blur keyframe blocking, source ownership and malformed timing tests. The corrected mapper also tests insertion under an already expired deadline, preservation of the pending opportunity, fresh-quantum promotion and cancellation on retirement/reset. The final-source mapping/reset fixture also passed Apple's `leaks`: **0 leaks / 0 leaked bytes**, with 733 scanned allocation nodes totaling 1,118 KB and reported peak physical footprint 6,593K. Its full log (local evidence `evidence/mobile_paper_20261113/deferred-mapping-leaks.log`) has SHA256 `11922e3b4e1ce8931eef7b1354fd771bd658cf1954f38650c1ec8729f4796a0b`. This focused ownership check does not establish leak-free long replays; replay RSS above is measured separately.

- Canonical diagnostic control (local evidence `evidence/mobile_paper_20261113/diagnostics_ablation_01/control-equivalence.json`): an earlier EuRoC-right 200-frame diagnostic prefix exactly matches frozen canonical `a85110d2…` across 25 non-timing fields. This is a bounded compatibility check.
- Allocator-order proof (local evidence `evidence/mobile_paper_20261113/stage1/seed_prior_order_regression_20261113/provenance.json`): reversed allocation exposes the old partial-depth mean’s pointer-order dependence; stable landmark-order accumulation fixes it without changing thresholds.
- The `c0a67492…` phone control (paired with host `332a9d…`) exposed keyframe insertion consuming the mapping deadline before promotion. The corrected profile queues a bounded candidate-ID snapshot for a fresh quantum and preserves progress across expiry. This is why fast-host and source-paced/device validation remain distinct.
- Image-only flow audit (local evidence `evidence/mobile_paper/frontend_20261113/fixed-pair-flow-audit.json`): historical 14→15 median motion is 9.69 full-resolution pixels and 15→16 is 14.75; new 16→17 is 9.94. The manual radius of 4 pixels at L1 (8 at full resolution), without homography prediction/warping, loses trails before the scheduled second exposure. Its mapper is never called; this does not show that the fixed stereo pair is geometrically unusable.

Full receipts and test-log hashes are in the machine summary. Physical iPhone results belong to their separate device report. The task uses existing recordings only; no live capture or camera permission is required.
