# Read-only analysis of upstream/current divergence

> Detailed validation record. See [current results](../RESULTS.md) for the main comparison.
> `evidence/` labels identify local validation artifacts; raw logs, app binaries
> and private inputs are not redistributed with this repository.

13 November 2026. This analyzes completed common-pinhole phone runs; it changes
no estimator, calibration, initialization pair, recording or phone state. The
full primary comparison is still in progress. Context: `SUMMARY.md`, `SKILL.md`,
the [frozen input plan](upstream-input-plan.md) and actual source/logs.

Source logs are under
`evidence/upstream_comparison/phone_20261113/{current,upstream}/{lounge_new,lounge_reference,euroc_cam1}/repNN/run/frames.jsonl`.
The machine-readable diagnostics and input hashes (local evidence `evidence/upstream_comparison/divergence_analysis_20261113/diagnostics.json`)
have SHA256 `aa08a1ca274976615dede15e8e1ba8155630d46d4a49531c79e903de10061fa2`.
The independent evaluator's matching results are in
`upstream_comparison/interim_matched_20261113/report.json`.

## What the saved observations establish

The lounge frontend and first tracked motion agree closely. New lounge has
133 initial trails and 17 remaining at the second press in both engines;
historical has 147 and 19. To remove the arbitrary world-plane gauge, compare
`C_i_from_C_init = C_i_from_W * inverse(C_init_from_W)`:

| Recording / first tracked frame | Rotation difference | Translation difference, original monocular units |
| --- | ---: | ---: |
| New lounge /19 | 0.000198° | 0.0000371 |
| Historical lounge /17 | 0.000366° | 0.0000267 |

These are engine-to-engine comparisons without reference poses. They argue
against a gross initial pose/calibration disagreement on these two runs. They
do not prove the bootstrap scene has sufficient parallax or depth observability.

The initial point-count difference mostly reflects retirement timing. New
upstream reports 207 points at initialization, then 152 at frame19; current reports
152 active plus 55 retired, also 207 total. Both find 139/151 at frame19.
Historical upstream reports 78 initially, then 38; current reports 39 active plus 39
retired, also 78 total. Upstream asynchronous status counts are approximate;
current serial active/retired accounting is exact for the saved frame.

The active-keyframe timelines then differ substantially. These are the first
**logged active counts**, not exact asynchronous request/commit times:

| Recording | Current initialization; later keyframe activations | Upstream initialization; later activations |
| --- | --- | --- |
| New lounge |18;30, 39, 46, 51, 59, 66, 78, 84 —10 final KFs |18;25, 46 —4 final KFs |
| Historical |16;22, 25, 28 —5 final KFs |16;22, 43, 213 —5 final KFs |

New lounge relative-motion rotation differences grow from 0.00337° at frame20 to
0.112° at 25 and 1.067° at 40. Historical grows from 0.00635° at 20 to 6.731° at 25,
after its first added keyframe and around two additional current insertions.
This places the observed divergence during tracking/map maintenance rather
than at the first tracked exposure. It does not isolate which maintenance change
causes it. Comparing predicted poses after an engine loses tracking would not
establish accuracy, so later lost-motion discrepancies are not used as evidence.

The full new-lounge RMS comparison uses different accepted intervals. On the
same 33 accepted/reference-associated exposures,19–53, separately fitted Sim3
RMS is **3.50cm current versus 2.22cm upstream**. The full-run24.90cm versus 2.22cm
headline therefore exaggerates the common-interval difference, although current
still has a smaller regression there. Historical's same 44 accepted exposures
give 23.41cm current versus 25.84cm upstream: both remain poor. EuRoC-right's210
reference-associated common exposures give 5.50cm current versus 2.30cm upstream.
The exposure intersections were selected only from logged validity, then scored;
no reference selected a pair, window, pose or estimator change.

## EuRoC-right has a concrete map-growth stall

Both engines start with 1000 trails and 466 at the second press. At frame6 current
has 383 active points and 108 matches; upstream has 1122 active and 455 matches.
Unlike the lounges, a substantial support difference is already present before
later keyframe growth. Upstream initially reports 1402 points before retirement.
The saved data does not explain which bootstrap stage accounts for the gap.

More decisively, **current active plus retired points stays exactly 450 on every
initialized frame in both repetitions**, while keyframes increase2→24. Active
points383→353 and retired67→97 account entirely for retirement: there are no
accepted new map points after initialization. Upstream grows to 3561 active
points at its maximum and 2339 at the first run's end. Its second run has different
mapping/coverage, consistent with retained asynchronous nondeterminism.

Current's5000-point global cap is never approached. The per-level300-addition
cap cannot directly prevent a first accepted point when a call starts with zero
additions; it might still affect the bootstrap state. Merely relaxing quality
cannot explain or fix zero point creation. The current
[insertion path](../../xPTAMclean/iPTAM/MapMaker.cc), lines417–443, still calls
all four epipolar levels for each incoming keyframe. Its
[epipolar search](../../xPTAMclean/iPTAM/MapMaker.cc), lines448–570, can reject on
depth/search geometry, patch/subpixel failure or finite/positive-depth checks.
The logs contain no per-reason counts, candidate depths or point coordinates.
Removing a correct cheirality check simply to resemble upstream would be unsafe:
upstream does not perform that final two-camera check on expanded points.

## Supported hypotheses and next controlled work

There are two material source differences beyond numerical/quality corrections:

- The [current mapper](../../xPTAMclean/iPTAM/MapMaker.cc), lines66–82, chooses
  one heavy stage per `ProcessPending`; the
  [tracker](../../xPTAMclean/iPTAM/Tracker.cc), line329, calls it once per image.
  Source-paced waiting does no additional mapping. Original `MapMaker::run`,
  lines70–116, runs independently with a5ms sleep and can execute multiple
  applicable stages between exposures. Insufficient settling/refinding between
  current keyframes is plausible, but stage/accepted-step traces are absent.
- Current [keyframe policy](../../xPTAMclean/iPTAM/Tracker.cc), lines304–308,
  uses0.3s plus a three-frame minimum; upstream uses more than 20 frames.
  Current's inherited distance multiplier is2, upstream's default is1. These
  opposing cadence/distance changes explain why neither engine is uniformly
  earlier. Original code/defaults are preserved in
  `upstream-work/Src/{Tracker,MapMaker}.cc`.

The current retained-solve stops occur at new-lounge frames 18, 32, 79 and
historical16, 27, 30; all retain finite cost-decreasing accepted iterates, and these
runs have no hard solver rejection. Their timing overlaps map changes but does
not prove that retention caused drift. Necessary finite/bounded guards should
remain enabled in any investigation.

Recommended order, as **future isolated experiments**, without changing the
frozen production source:

1. Instrument accepted/rejected epipolar candidates by reason and level, target
   baseline, depth interval, patch/subpixel outcome and two-view depths. Record
   bundle stage, support, objective decrease and pending/converged flags. First
   locate the right-camera zero-growth condition without changing decisions.
2. Ablate only mapping service scheduling: allow bounded serial pending stages
   during existing source-time idle, with explicit work/deadline limits and the
   same delivered images. Preserve numeric guards, quality, RNG and thresholds.
3. Separately test original keyframe spacing and distance multiplier, one factor
   at a time. Keep fixed inputs/pairs and evaluate all five complete recordings,
   repeated states, common-exposure error, total motion coverage and lost tails.
   Do not promote a variant from a single favorable lounge result.

The previous native-equidistant scores remain a separate optical condition.
Their wider prepared view retains359→146 new-lounge trails and 312→190 historical
trails, versus 133→17 and 147→19 here. That supports a real feature-continuity
difference between preparations; it is not evidence that either engine received
incorrect common-pinhole intrinsics, and it does not justify changing the frozen
comparison calibration or pair.
