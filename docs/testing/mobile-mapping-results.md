# Mobile mapper: corrected scheduling results

> Detailed validation record. See [current results](../RESULTS.md) for the main comparison.
> `evidence/` labels identify local validation artifacts; raw logs, app binaries
> and private inputs are not redistributed with this repository.

Primary host executable:
`694807b9c7081caa61fa885050e99235ca4416f40d9d4f20c60316a522be3638`.
This read-only audit covers **31 retained complete runs / 47,116 logged frames** on the
five retained immutable common pinhole inputs: two fixed-pair and two automatic sweeps
with a 4 ms foreground mapping budget; two automatic sweeps without wall-time
budgets; and one actual source-paced automatic new-lounge run. No estimator was
rerun or changed for this analysis. The corresponding corrected phone executable
starts `0c740562`; device outcomes are reported separately.

Exact per-run measurements, source log/config hashes and reproduction script:
`evidence/mobile_paper_20261113/mapping_analysis/deferred_20261113/`
(`report.json`, `analyze.py`, `analysis.log`). Inputs are
`host_deferred_timed/`, `host_deferred_deterministic/` and
`host_deferred_source_paced/`. Earlier scheduling controls are separate from the corrected profile.
Do not pool different binaries, initialization policies or source schedules.

## Corrected promotion scheduling

The older physical-phone control (`c0a674...`) completed six replays / 7,894 frames,
including three new-lounge repeats. Each new-lounge repeat had 23/100 GOOD poses,
56 partial seeds, 188 updates and **zero promotions**. Its KF insertion took 6–8 ms,
already consuming the 4 ms quantum before the only promotion loop checked its
deadline. The mapper silently dropped the promotion opportunity. This was a
scheduling defect; geometric thresholds did not explain the missing work.

The corrected mapper snapshots at most 25 eligible stable candidate IDs per newly
owned active KF before seeding. A separate `PartialPromotion` stage (index 7)
resumes work in a fresh quantum before BA. Expired, unprocessed IDs retain the
task cursor; obsolete IDs skip safely. Pending tasks keep idle scheduling alive
with converged BA. KF retirement/reset cancel relevant tasks before owner reuse,
and a deferred older frame cannot re-assimilate a newer filter observation.
No geometry, quality threshold or deadline was relaxed.
See [mapper scheduling](../../xPTAMclean/iPTAM/MapMaker.cc) and
[task lifecycle](../../xPTAMclean/iPTAM/MobileMapping.cc).

Across **every logged frame**, the following invariants hold:

- `queued - completed - cancelled == pending`;
- pending task count never exceeds active KF count; maximum observed pending = 1;
- counters do not decrease/reset in the saved logs.

Two pending tasks are each newly queued at final frame 229 of the as-fast historical
repeats. No later quantum exists in those recordings; the work is explicitly
retained and logged, not reported as completed. All other runs end with 0 pending.
Cancellation is not exercised by these runs because no active KF reaches pruning;
actual image/ownership fixtures separately cover target retirement/reset.

The regression inserts a real textured translated-plane KF under an already
expired deadline, verifies its promotion remains queued, marks BA converged and
requires a fresh idle quantum to promote valid two-view depth. Release 12/12,
optimized ASan/UBSan and independent lifecycle review pass. Raw source snapshots,
pre-fix phone evidence and test receipts remain in
`mapping_analysis/deferred_promotion_fix_20261113/`.

## Default configuration and observed resource use

The app/CLI explicitly selects full automatic `auto`. The `paper` profile is the
fixed-pair ablation of the same mobile mapper; the raw `Settings` constructor
keeps manual initialization for controlled construction. Both use the following
[configured limits](../../xPTAMclean/iPTAM/MobileConfig.h):

- **600 combined active full landmarks + provisional candidates**, at most 25
  candidate seeds/visits per batch and 120 full points selected for pose tracking.
- **35 active KFs**, preserving the fixed gauge, newest two views and connected
  remaining graph. Retained image owners follow active pose anchors. Failure to
  find a safe prune produces a resource stop.
- **Eight observations per landmark**, except required fixed-gauge bridges or
  three-measurement free-view support. These exceptions are explicitly counted;
  unsupported free views are fixed before BA.
- Stop new map work at 128 retained image KFs or 2,000 retired points. Cleanup can
  cross the latter threshold by at most one active batch. Budgeted insertion
  holds at most two queued KFs; promotion task storage is bounded by active KFs
  times 25 stable IDs. Reset owns/releases all source images and pointers.

All 31 retained logs stay below these active targets: **maximum 488 combined points,
299 full points and 30 active KFs**. No resource stop, observation-bridge/support
exception or low-support-view freeze is logged. No KF culling occurs in this
corrected full-input set. Therefore these logs do not exercise every cap;
pruning/retained-anchor behavior is supported by geometry/reset tests and the
separately labeled older control. Actual observation maxima, trash size and
retained-image bytes are not exported, so logs cannot independently verify them.

## Map growth and accepted coverage

Repeat 01 is shown consistently, not selected by outcome. `P first→peak→end`
counts full landmarks after initialization. Promotions are insertion events,
including points later retired. GOOD uses the existing finite rigid pose,
raw-match and final-pose-support policy. It is not an accuracy claim.

### Full automatic system,4 ms foreground budget, as-fast input

| Input | GOOD / frames | P first→peak→end | Partial seeds / promotions | Peak P+partial | Observation culls | Active KFs peak |
|---|---:|---:|---:|---:|---:|---:|
| Historical lounge | 61/230 | 117→182→178 | 139 / 87 | 210 | 619 | 23 |
| New lounge | 82/100 | 103→178→177 | 176 / 106 | 211 | 59 | 20 |
| EuRoC left | 238/3682 | 117→289→282 | 575 / 239 | 452 | 131 | 24 |
| EuRoC right | 3/3682 | 13→13→13 | 25 / 0 | 38 | 0 | 2 |
| Actual sparse lounge | 0/142 | 0→0→0 | 0 / 0 | 0 | 0 | 0 |

Both lounges and EuRoC-left perform delayed point insertion. EuRoC-right remains
an early tracking failure:13 full points,2KFs,0 partial updates/promotions and only
3 accepted poses. A sufficiently supported GOOD pose is required to observe
provisional depth, so loss prevents further mapping despite spare capacity.
The sparse actual lounge never initializes. The point/KF caps do not explain these failures. Growing a map
cannot establish trajectory accuracy or general stable scanning.

### Automatic, wall-time mapping budget disabled

| Input | GOOD / frames | End points | Partial promotions | Observation culls | Exact pose/state repeats |
|---|---:|---:|---:|---:|---|
| Historical lounge | 61/230 | 177 | 88 | 518 | yes |
| New lounge | 82/100 | 177 | 106 | 59 | yes |
| EuRoC left | 246/3682 | 295 | 256 | 130 | yes |
| EuRoC right | 3/3682 | 13 | 0 | 0 | yes |
| Actual sparse lounge | 0/142 | 0 | 0 | 0 | yes |

All five unbudgeted repeat pairs match exact normalized pose/state hashes. Timed
historical/new lounge and EuRoC-left have equal accepted-frame counts within each
automatic pair but different states; actual elapsed budget boundaries are not
promised bitwise repeatable. The prior allocator-order seed-average bug was
separate and is fixed in this binary; its old-fail/new-pass regression is in
`stage1/seed_prior_order_regression_20261113/`.

### Fixed-pair ablation,4 ms foreground budget

| Input | GOOD repeats 01 / 02 | First map frame | End points rep01 | Partial promotions rep01 |
|---|---:|---:|---:|---:|
| Historical lounge | 0 / 0 | none | 0 | 0 |
| New lounge | 0 / 0 | none | 0 | 0 |
| EuRoC left | 221 / 252 | 1205 | 208 | 247 |
| EuRoC right | 63 / 63 | 5 | 46 | 42 |
| Actual sparse lounge | 0 / 0 | none | 0 | 0 |

Fixed lounge trails reset before the requested second frame and never reach
stereo geometry. At the reset, image-only adjacent motion exceeds many features'
eight-root-pixel local search windows. Automatic initialization uses a distinct
global-homography predictive frontend. These are different initialization
policies; this is not an isolated pair-geometry/solver comparison. Independent image-flow evidence is summarized in the
[host validation report](mobile-host-validation.md).

The isolated stage1 EuRoC-right control still demonstrates a real legacy
reversed depth interval: lower 6.00821 versus capped upper 4; all 1,843 attempts
reject before matching. Its guarded scene-upper repair accepts 612 epipolar points
while 13 still fail unchanged cheirality. Exact source/probe evidence is in
`stage1/depth_window_probe_20261113/`. Final mobile profiles use inverse-depth
growth instead and log **zero legacy epipolar attempts/repairs**; their promotions
cannot be attributed to exercising that isolated repair.

## Actual source-idle work and soft-budget limits

The one new-lounge **source-paced** host run delivers all 100 images in source
order and achieves 83 GOOD, 178 ending full points and 105 partial promotions.
All 17 queued promotion tasks complete, with 0 pending/cancelled. It records:

- **61 idle stages, 62.266 ms extra idle wall work, 10 soft idle overruns**;
- 8 solver budget yields; 34 full-BA calls, maximum 4.074 ms;
- 17 deferred-promotion calls, maximum 0.188 ms;
- 1.636 ms mean / 17.706 ms maximum foreground frame processing;
- 9.92 s process wall time, 0.23 s user + 0.03 s system CPU (time-tool rounding).

This is direct evidence that idle work runs in the source clock's available
intervals on this host. It is one trial, not a repeatability or iPhone-throughput
claim. No frames are dropped. Source waiting, processing and additional idle CPU
are distinct quantities.

The full timed/deterministic sweeps are **as-fast**, not source paced, and record
zero idle stages. Individual bounded mapping operations can exceed the soft deadline. Bootstrap
is outside the budget;
whole-input mean latency also includes cheap lost/uninitialized frames.

Bundle checks at cooperative solver boundaries; one bounded attempt is permitted
when setup consumes a fresh window, ensuring progress. Individual setup/solve/
cull operations can exceed the soft deadline. The source-paced diagnostics can
combine several idle calls between logged exposures, so summed per-frame stage
time is **not** a single-quantum overrun. Its 10 idle-overrun counter is directly
logged; exact per-call exceedance counts cannot be recovered for those combined
intervals. Stage wall timers exclude direct bootstrap BA and uninstrumented
cleanup/resource bookkeeping, and are not total mapper CPU.

Peak RSS across the 31 retained host runs is 41.047 MiB. No saved row is nonfinite/nonrigid; no
resource stop or observed counter reset occurs. Intra-frame initialization work
cleared by Reset cannot be recovered from cumulative mapper counters alone.
Desktop processing, complete finite output and larger maps do not establish
phone thermal performance, accurate full-path tracking or the stability goal.
All common inputs are 640×480, four times 320×240 pixel count; this does not
reproduce the historical paper's frame-rate setting.

## PHONE: completed physical automatic-profile audit

The independent read-only audit is now complete for **9 retained physical iPhone 12
trials / 15,400 frames**, with `suite-complete.json` verified. App SHA256:
`0c740562b775e644bf7cb2bbd73c3568bcedbfc904776d3f22b9b214c32d96f7`.
All nine use the explicit automatic mobile profile, common pinhole recordings,
source pacing and 4 ms soft mapping budgets. The paired host binary is 694807b9.
Three new-lounge and three EuRoC-right trials are included; other inputs have one.
Fixed-pair phone ablations are separate and are not pooled here.

Reproduction and pinned summary/JSONL/manifest/validation hashes:
`mapping_analysis/phone_deferred_20261113/` (`analyze.py --require-complete`,
`report.json`, `provenance.json`, `analysis.log`). The reducer reads saved files
only and independently checks every frame, not just the app's final counters.

All 15,400 retained frames satisfy deferred accounting, pending≤active KFs, combined-point
and active/retained-KF limits, `retained=culls`,
`active KFs=2+insertions−culls` after initialization, increasing source times,
finite rigid poses and the strict exported visual flag. App summaries match the
last JSONL mapper fields exactly. No observed counter decreases or resource stop
occurs. Every final task is accounted for: **130 queued, 130 completed, 0 cancelled,
0 pending**, with maximum one pending task at any logged frame. Cancellation is
covered by the ownership/reset fixture, not exercised by these final replays.

| Input (frames/run) | Runs | GOOD/run | Ending full points/run | Promotions/run | Promotion tasks complete/queued, all runs | Active / retained KF peaks |
|---|---:|---:|---:|---:|---:|---:|
| Historical lounge (230) | 1 | 142 | 299 | 270 | 47/47 | 35 / 14 |
| New lounge (100) | 3 | 82 | 179 / 183 / 180 | 101 / 100 / 96 | 51/51 | 20 / 0 |
| EuRoC left (3682) | 1 | 329 | 410 | 372 | 32/32 | 34 / 0 |
| EuRoC right (3682) | 3 | 1 | 12 | 0 | 0/0 | 2 / 0 |
| Actual sparse lounge (142) | 1 | 0 | 0 | 0 | 0/0 | 0 / 0 |

New-lounge repeat counts are 82 GOOD each; their ending maps and promotions differ.
That restores promotion after the old control's 0 promotions/23 GOOD failure, but
**does not demonstrate exact timed repeatability**. EuRoC-right still has only
one accepted pose per complete trial and no updates/promotions: it never queues
a new-KF promotion task. This is remaining tracking failure, not deferred-task
starvation. Actual sparse lounge still does not initialize. Reference-fit
accuracy is reported separately. Larger maps or finished processing are not accuracy claims.

Observed phone resource maxima are **550 combined points, 410 full points,
35 active KFs, 14 retained image KFs and 113 retired points**. Historical lounge
actually exercises 14 KF culls and 477 observation culls; all nine retained runs together
record 772 observation culls. Retained counts match pruning counters at every
frame, and fixed-gauge/bridge support exceptions remain 0. EuRoC-left records
3 low-support view freezes before BA. All rolling-shutter flags, observations,
corrections and mapper RS counts are 0; unknown readout timing stays disabled.
Actual per-landmark observation maxima are not exported, so the eight-observation
policy remains source/test backed rather than independently measurable here.

### Phone wall-time work and overruns

All runs record nominal thermal state 0 and low-power mode off. Mapper idle scheduling records its extra wall-time work and cooperative
overruns separately from foreground processing. The wall time includes small empty-scheduler overhead and overlaps
its measured stage sections; it must not be added to their totals or treated as
measured CPU. For example, EuRoC-right records 0 idle stages but a few milliseconds
of empty-call overhead. Foreground processing excludes that idle time.

| Input | Foreground mean ms/run | Maximum foreground frame ms | Recorded idle wall ms, all runs | Idle overruns, all runs |
|---|---:|---:|---:|---:|
| Historical lounge | 8.165 | 42.207 | 1722.913 | 323 |
| New lounge | 7.583–8.241 | 60.501 | 726.501 | 139 |
| EuRoC left | 2.908 | 36.942 | 731.050 | 138 |
| EuRoC right | 0.828–0.865 | 25.042 | 5.914 | 0 |
| Actual sparse lounge | 13.754 | 40.944 | 0.000 | 0 |

The soft 4 ms budget is not a hard execution-time bound. KF insertion reaches
10.678 ms, full BA 7.674 ms, and historical KF/observation culling **28.120 ms**;
the separate deferred-promotion stage remains at or below 0.999 ms. The slowest
foreground frame is 60.501 ms (new lounge); peak OS resident size is 67.438 MiB
(historical). Scheduling lag reaches 245.836 ms on the sparse actual recording.
These tail costs remain visible despite low whole-input means; lost and
uninitialized frames are often cheaper. Available per-frame deltas combine
foreground and idle calls, so individual overrun counts cannot be inferred from
summed interval time. Idle-overrun counts are logged directly.

This validates the deferred work/accounting and observes actual bounded map
pruning on the target phone. It does not resolve major tracking gaps, verify
reference accuracy, guarantee real-time deadlines or establish general scanning
stability. No camera capture, permission request or estimator change was needed
for this audit.

## PHONE appendix: fixed-pair ablation (separate)

The separately completed `phone_deferred_paper` suite contains **two additional
physical trials / 3,782 frames**, using the same `0c740562...` app and source
pacing, with explicit `paper` fixed-pair initialization. Its completion marker
and all logged resource/accounting checks pass. These runs are not part of the
nine automatic trials above and are not pooled with their totals.

| Input | Frozen first/second | GOOD / frames | Ending full points | Partial promotions | Completed / queued tasks | Peak active / retained KFs |
|---|---:|---:|---:|---:|---:|---:|
| New lounge | 0 / 18 | 0 / 100 | 0 | 0 | 0 / 0 | 0 / 0 |
| EuRoC right | 0 / 5 | 63 / 3682 | 46 | 39 | 4 / 4 | 6 / 0 |

The new-lounge local trails fail before reaching stereo initialization. EuRoC
right creates a map and performs deferred promotions, ending with zero pending
or cancelled tasks. It records 16 idle stages, 12.176 ms total idle scheduling
wall time and one soft overrun. Both cases keep RS disabled; neither hits a
resource bound or performs KF/observation culling. Peak combined full/provisional
count is 122 and peak retired-point count is 21. Final summaries match the last
frame counters exactly.

The right-camera accepted count differs from the automatic profile's one frame,
but 63 accepted exposures still cover only a small part of the recording; this
mapping audit makes no comparative accuracy claim. These two inputs expose
initialization-policy limits rather than establishing a universal better policy.
Reproducible reduction, original log/config/validation hashes and all-frame
checks are in `mapping_analysis/phone_deferred_paper_20261113/`.
