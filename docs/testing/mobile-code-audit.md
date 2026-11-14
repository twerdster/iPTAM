# Mobile paper variant: code audit

> Detailed validation record. See [current results](../RESULTS.md) for the main comparison.
> `evidence/` labels identify local validation artifacts; raw logs, app binaries
> and private inputs are not redistributed with this repository.

This is an experimental implementation of ideas from Klein and Murray's
[ISMAR 2009 paper](https://www.robots.ox.ac.uk/~dwm/Publications/Papers/klein_murray_ismar2009/klein_murray_ismar2009.pdf),
built on the guarded modern core. The audit found and corrected integration
defects before source freeze. It does not establish stable tracking on recorded
sequences or reproduce the original iPhone 3G executable.

Context read: project SUMMARY.md, SKILL.md, the paper's initialization and
rolling-shutter sections, and the actual implementation/tests. Compiled source
was read-only during the final integration audit; each owning agent applied its
fixes.

## Findings corrected

- **Incomplete rolling-shutter observations:** the mapper initially cleared all
  correction flags and re-corrected only landmarks having adjacent image
  observations. It now estimates one camera velocity from the available subset,
  then corrects every usable raw measurement with its fresh projection Jacobian.
  Root/epipolar measurements safely acquire their original raw coordinates.
  The regression includes a neighborless measurement and repeated correction;
  raw pixels remain unchanged and corrections do not accumulate.
- **Retired provisional sources:** partial landmarks initially survived their
  source keyframe's retirement even though promotion requires two active
  observations. They could consume capacity and update work without being able
  to promote. Culling now drops those candidates and normalizes the cursor.
  The regression seeds multiple source views and checks that only active-source
  candidates survive.
- **Coarse initializer prediction:** a rendered-image test exposed a large,
  finite SBI rotation from a nearly uninformative coarsest image. The initializer
  now validates SBI and retained-homography predictions by actual warped-patch
  geometric support before committing state. The original image-displacement
  assertion passes without relaxing its tolerance. These are two bounded
  predictions, not a RANSAC or initialization-window search.
- **Timing parsing:** the host loader initially accepted trailing garbage through
  `std::stod`. Parsing now consumes each complete finite numeric value and
  rejects unknown rolling-shutter options. The phone loader requires explicit
  verified timing, provenance and prepared-image coordinates.

## Preserved contracts and deliberate adaptations

Mapping remains serial. Cooperative deadlines yield between bounded operations;
setup or one complete solver step can overrun the nominal budget. A yield is
neither convergence nor numerical failure. The inspected budget exits expose
committed poses/points and the committed outlier list, rather than tentative
trial geometry. Wall-clock scheduling can change how much mapping work occurs
between exposures; exact state repeatability must be measured for this variant.

Culling retains the fixed view, checks graph connectivity, preserves required
observation bridges and minimum free-view support, and retains retired image
ownership. Archived poses follow a direct active anchor; they are not separately
optimized. These checks protect ownership and graph structure, not complete
geometric observability. Observation bridges can exceed the nominal eight-view
cap. Unlike the paper's additional fixed utility threshold, this implementation
retires the minimum-utility eligible view whenever the active-keyframe cap is
exceeded. That is an explicit bounded-map adaptation and may remove a useful
view. If a safe connected cull is unavailable, it stops rather than disconnecting
the graph. Retained-view and retired-point memory caps remain session limits.

The mobile app and replay CLI default to the `auto` profile; the explicit
`paper` profile preserves fixed-pair controls. Automatic initialization
tracks an incremental calibrated SL3 homography, waits for translational parallax
and a conditioned eight-parameter decomposition, and tracks competing SE3 maps
on the same feature identities. A visible but poorly conditioned alternative is
not discarded merely because its competitor is better conditioned. Selection
uses subsequent image objectives, never a reference pose. The condition ratio
`1e-6` and median parallax `0.005` radians are implementation settings, not
published paper constants. The selected first/current pair passes the ordinary
mapper's triangulation and bundle checks. Intermediate views resolve ambiguity;
they are not retained as a third initial BA keyframe.

Partial inverse-depth estimates guide association only. They do not count as
regular pose support. Promotion re-triangulates a measured pair, requires
positive depth and reprojection support, and discards the filter's accumulated
information. Global scaling transforms both inverse depth and its variance;
reset clears provisional source pointers and retained-anchor records.

Rolling-shutter correction is first-order visual least squares, with no IMU or
guessed 3G timing. Readout duration means first-pixel-center to last-pixel-center;
axis, direction and timestamp fraction must describe the supplied prepared
pixels. The available five recordings provide no verified usable readout timing,
so their correction remains disabled. Synthetic fixtures exercise the enabled
path; this is not physical sensor-calibration validation.

## Verification and limits

Independent focused Release runs passed initializer/rolling-shutter tests
(2/2), and ASan/UBSan passed those same tests (2/2). They cover pure rotation,
collinear geometry, noisy/outlier planar tracks, two-map ambiguity followed by
image-based disambiguation, rendered patch motion, invalid clocks and pixels,
known shutter warps for both axes/directions, and rank-deficient or unknown
timing rejection. After the integration fixes, a separate Release run passed
the mapping and budget suites (2/2), including retained ownership/reset,
connected culling, real textured-plane partial promotion and raw correction
ownership.

The deterministic budget test covers an expired deadline before the first trial;
there is no dedicated deterministic deadline-expiry-after-acceptance fixture.
Existing accepted-iterate/singular-stop tests exercise transactional solver
bookkeeping, but do not substitute for that exact scheduling case. No new
clock/test hook was introduced during freeze. Full recorded-sequence accuracy,
coverage, performance, thermal conditions and repeated runs must be reported
separately; passing these focused contracts is not a general stability result.

## Final source and presentation addendum

The final partial-depth prior sums eligible measured landmarks in stable map
insertion order. Iterating the pointer-keyed measurement map had allowed allocator
layout to change floating-point accumulation and later association. This fix
changes accumulation order, not thresholds or quality policy. A regression
reverses allocator-address order for the same inserted scene: the corrected code
passes in Release and optimized ASan/UBSan, while the saved previous library
fails the intended assertion. Sources, binary/log hashes and the expected-old
failure are preserved in
`evidence/mobile_paper_20261113/stage1/seed_prior_order_regression_20261113/provenance.json`.
This removes that specific dependence; it does not prove general repeatability
under wall-clock mapping budgets.

Retained pre-deferred-promotion complete-suite control checks are under
`evidence/mobile_paper_20261113/`:

- `release-tests-final.log`: 12/12 passed in 6.42 seconds.
- `sanitizer-tests-final.log`: 12/12 passed in 37.39 seconds. The
  `build-mobile-sanitize` configuration is RelWithDebInfo (`-O2 -g -DNDEBUG`),
  with address/undefined sanitizers enabled and Apple arm64 leak detection
  disabled. These are optimized sanitizer results, not an unoptimized timing
  comparison.

Automatic second-view selection and fixed-pair controls answer different
questions. Same pixels, calibration, exposure clocks and a predeclared start
permit a complete-system comparison; they do not isolate subsequent tracking
when initialization policy differs. The report and video paths must explicitly
select system-comparison mode and label that distinction. Shared accepted-time
fits supplement full-input coverage and do not remove this policy difference.

The presentation audit corrected a second eligibility flag that had remained
true when source artifacts changed during analysis. Both comparison modes now
fail closed in that case. Mobile fixed-pair eligibility requires executed config
explicitly report `false`/`0`; missing or malformed config is marked unknown and
can enter only an explicitly labeled system comparison. Conflicting executed
config, profile and summary declarations qualify for neither comparison mode.
Video captions use only the selected trials' actual policy, so selecting two
fixed controls never produces an automatic-mobile label. Regressions also prove
automatic mobile trials cannot enter the default shared-time isolation path.
Presentation helper regressions verify these comparison-policy guards.

A later read-only review covered deferred partial promotion after a keyframe
insertion consumes its mapping quantum. The mapper now snapshots bounded stable
candidate IDs before seeding and processes them in a fresh stage ahead of BA.
An unprocessed candidate keeps its task cursor on deadline expiry; retired
targets and reset cancel ownership, and erased source candidates are safely
skipped. Pending tasks also keep idle scheduling alive. The actual-image fixture
covers expired insertion followed by promotion in a fresh idle quantum. No
additional lifetime/cursor blocker was found. Earlier phone trials remain
controls rather than evidence for the revised binary.

## Corrected source freeze and remaining scope

Final host SHA256:
`694807b9c7081caa61fa885050e99235ca4416f40d9d4f20c60316a522be3638`.
Corrected phone SHA256:
`0c740562b775e644bf7cb2bbd73c3568bcedbfc904776d3f22b9b214c32d96f7`.
Exact source/build identities are in the evidence root's
`builds/host_deferred-694807b9c7081caa/` and
`builds/phone_deferred-0c740562b775e644/`. Previous host `332a9d…` and phone
`c0a674…` are superseded controls.

The corrected source passes `deferred-release-tests.log` (12/12, 4.60 seconds)
and `deferred-sanitizer-tests.log` (12/12, 37.52 seconds, RelWithDebInfo `-O2`
ASan/UBSan). Their SHA256 values are
`c3a589e0ae7b6b1649020aa7fc66a3ab7ed2bb5da657bdecf21980e63bff3540`
and `8fa21d7338c4443dd22d4c2e7da500fbd9b9ff58f14e05a40340275ca3c2ee5a`.
Final mapping/reset
ownership also passed Apple's `leaks`: 0 leaked bytes, 733 scanned nodes totaling
1,118 KB, peak reported footprint 6,593K. Full log `deferred-mapping-leaks.log`
has SHA256 `11922e3b4e1ce8931eef7b1354fd771bd658cf1954f38650c1ec8729f4796a0b`.
This is a focused fixture, not a long-replay leak guarantee.

The mobile compile guard was checked separately with `IPTAM_MOBILE_PAPER=OFF`:
all five Release tests passed in 5.29 seconds (`mobile-off-tests.log`). Executable
`3be4c5c047b664fa0bc397c9949a3ad014d45f1dc5f22c64597b5e1adfca120c`
matched all 25 non-timing CSV fields over the same 200-frame EuRoC-right prefix
against frozen modern executable
`a85110d2098a5ecd1dcc18e3f8d2d1a5c25be018998cbc9547f8aa3bc1090b05`.
The exact command and comparison are in `mobile_off_compatibility/receipt.json`
under the evidence root. This is a bounded compatibility guard, not equivalence
over every recording or platform.

[Host validation](mobile-host-validation.md) covers the retained 31 corrected runs /
47,116 rows, including a source-paced host control. The five deterministic
automatic repeat pairs match exactly; six of ten timed pairs do. The retained
physical automatic batch covers nine trials / 15,400 exposures.
[Phone comparison](mobile-phone-comparison.md) records mixed accuracy/coverage,
nonidentical timed new-lounge poses, exact failed right-camera repeats and
resource measurements. Finite processing does not establish general stability.
No live capture or permission is required.

## Failure-receipt handoff build

The full automatic phone benchmark above belongs to executable `0c740562…`.
The subsequent handoff app is
`f4681a6e2d9583e3a49470943903fcf006bccf7370be7d0d6fcb2fe55e174dbe`,
archived under `builds/phone_receipt-f4681a6e2d9583e3/`. An invalid
rolling-shutter setup had correctly rejected the input before processing, but
threw before the run's manifest identity was saved. The app now starts that
failure receipt, records the manifest path/hash, and includes the explicit
no-camera-request flag in mobile run summaries.

Independent comparison of both archived source inventories confirms that only
`ios/App.mm` differs among 186 compiled inputs: one default summary flag and
four exception-path lines. No estimator, configuration, source pacing or
successful-replay statement changed. Current inputs match the new archive, and
the archived executable hash matches `handoff-build.json`. The nine retained
automatic trials and fixed-profile controls remain associated with their
original benchmark app; no metrics are pooled across these builds.

The handoff app passes all ten fresh physical rejection/lifecycle checks in
`phone_lifecycle_receipt_fixed/results.json`. Two full source-paced compatibility
replays process 3,782 images: new lounge has the same 82-frame accepted mask with
timed pose differences, and right has the same single accepted frame with all 21
non-timing fields exact. Receipt is
`phone_receipt_compatibility/compatibility-receipt.json`; these are not pooled
into the benchmark. Default library opens without requesting camera access. The retained five-input
benchmark was not rerun in full on this receipt-only app revision.
