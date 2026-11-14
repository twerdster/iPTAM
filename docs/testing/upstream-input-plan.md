# Common inputs for upstream PTAM comparison

> Detailed validation record. See [current results](../RESULTS.md) for the main comparison.
> `evidence/` labels identify local validation artifacts; raw logs, app binaries
> and private inputs are not redistributed with this repository.

Prepared 13 November 2026. This plan defines identical recorded input for
upstream PTAM and the current iPTAM port on the same phone. It establishes input
readiness, not comparison results. No phone operation or estimator source change
was performed for this audit.

The primary comparison uses the existing **pinhole** preparations for all five
recordings. Both implementations receive the same images, pixel intrinsics,
source exposure timestamps, complete chronological sequence and initialization
commands. The native equidistant lounge results remain separate controls;
their existing scores cannot substitute for fresh runs on these pinhole images.

The [public input catalog](../results/frozen-input-plan.json) records the retained
inputs and their calibration and provenance. Keep prepared pixels and calibration
immutable when reproducing a comparison.

## Fixed input set

Paths below are relative to
`evidence/`. All inputs are 640×480 P5 grayscale
PGMs with principal point `(319.5,239.5)` and `lens_model=pinhole`.

| Recording | Dataset directory | Frames | Source span | fx = fy | Fixed first/second |
| --- | --- | ---: | ---: | ---: | --- |
| Historical synthetic lounge | `datasets/lounge_reference` | 230 | 22.900463 s | 400 | 0 / 16 |
| New synthetic lounge | `datasets/lounge_new` | 100 | 9.900000 s | 400 | 0 / 18 |
| EuRoC MH_01_easy left | `datasets/euroc_cam0` | 3682 | 184.050000 s | 460 | 1200 / 1205 |
| EuRoC MH_01_easy right | `datasets/euroc_cam1` | 3682 | 184.050000 s | 460 | 0 / 5 |
| Actual lounge export | `actual_datasets/actual_lounge` | 142 | 131.902875 s | 460 | 47 / 48 |

Total: **7,836 images**. No dataset
regeneration was required for the benchmark. Each phone test must use a
fresh result directory and record the actual executable/source identity and
effective requested manifest hash.

These pairs reuse the final suite's previously frozen image-only index choices;
no reference pose, accuracy score or new implementation outcome chooses a pair.
The lounge indices are retained even though pinhole rectification changes the
visible features relative to native images. This is a predeclared common-input
test, not a claim that either initializer will succeed. Do not replace a failed
pair silently or trim the sequence to successful tracking.

Immutable preparation suggestions differ from the fixed commands: historical
0/15, new 0/17, both EuRoC 0/30 and actual 1/2. Pass the table's explicit
`--init-first` / `--init-second` overrides through the replay adapter; do not edit
the dataset manifests. Preserve all 3,682 cam0 exposures, including its delayed
initialization prefix, in the denominator and output.

## Calibration contract

The inspected upstream camera is the PTAM-GPL source at
`upstream-work`, commit
`d9dca71ad57b0ef5b409e1670443060d49df63cb`. Its `Src/ATANCamera.cc` sets pixel
focal lengths to image size times normalized parameters, and the pixel center
to image size times normalized offset minus 0.5. The fifth parameter `w=0`
disables radial distortion exactly in `rtrans_factor` and `invrtrans`.
The plan includes hashes of both upstream camera source files.

Use the existing camera configuration interface, without an algorithm change:

```text
Camera.Parameters = [fx/W, fy/H, (cx+0.5)/W, (cy+0.5)/H, 0]

lounge:       [0.625,    0.8333333333333334, 0.5, 0.5, 0]
EuRoC/actual: [0.71875,  0.9583333333333334, 0.5, 0.5, 0]
```

The current port uses `lens_model=pinhole` and the identical pixel K from the
table. The lounge preparations inverse-resample their ideal equidistant images
to pinhole K400. EuRoC inputs are rectified from their recorded radial/tangential
calibrations to K460. The actual lounge uses an explicit pinhole approximation
with K460; its export has no measured distortion table. Every target pixel has source sampling support.
The recorded image schedule, including sparse actual-lounge gaps, is retained.

An ideal equidistant forward projection has an algebraic FOV reparameterization
(`w=2*atan(0.5)`, FOV focal equal to `w` times equidistant focal). That alone does
not establish compatibility with the entire original camera implementation:
its small-radius approximations and the native lounge's 16 pixels outside the
front hemisphere differ. The common pinhole comparison avoids claiming an
untested exact-equidistant upstream model.

## Hash and storage audit

All 7,836 files were read and decoded as uint8 640×480 grayscale. Their
concatenated pixel hashes match the original preparation provenance. CSV frame
counts, paths and timestamps match each manifest exactly; all exposure clocks
are finite and strictly increasing. All available reference transforms are
finite and rigid. Reference associations remain separate from estimation.

| Recording | Concatenated grayscale pixel SHA256 |
| --- | --- |
| Historical pinhole lounge | `12ab22217853c79ba05963fefd6d6883058b1d39d2602cca43a58b6f8313034c` |
| New pinhole lounge | `e2f3d034af4fe161e9f61d986d74dbd23a3d8d55eb8c47e7210cd5b1aa493999` |
| EuRoC left | `b645c399203efbf250a594e7970128e6432ef9512cfcf23110d9c086591f8fc7` |
| EuRoC right | `4509df34db4506b0df3a582ae92ff351a1234cebee490291380143839ab61469` |
| Actual lounge | `9e3da201bfbd1f475fa6292c315cb4418ca85367c8502e53301353ec8ed46fd4` |

Input storage was checked writable before validation. No source recordings,
prepared images or existing native controls were modified.

Exact native lounge controls remain at
`native_datasets_exact/{lounge_reference,lounge_new}` with their original
equidistant K `[253.30344967,253.34336776,319.5,239.5]`. Their pixels were also
rehash-verified and their separate manifests/hashes are retained in the JSON
plan. Previous `a851…` host and `e7fcc…` phone results on these native images
remain valid controls of that different optical condition.

## Input delivery schedule

Upstream's original asynchronous mapper needs a declared wall-clock input
schedule as well as correct exposure timestamps. An as-fast loop could starve
its mapper and change the comparison. The primary paired runs should deliver
both implementations against the same source clock: target exposure time is
`run_start + source_timestamp - first_source_timestamp`. Preserve every image;
if processing misses a deadline, continue in order and record the lag rather
than dropping or duplicating images. Sleep only for positive remaining time.
This also preserves the sparse lounge's real gaps for both implementations.

Record the chosen schedule, deadline misses, input count, estimator processing
time and total wall time separately. If an as-fast replay is also useful, label
it as a distinct scheduling control and repeat it on both implementations.
Mapper shutdown/drain time must be distinguished from image processing; do not
silently add extra optimization before exporting a frame's pose. This plan does
not change the original upstream mapper or force the modern serial mapper into
the original asynchronous execution model.

## Reporting boundaries

This remains a recording-only comparison. Neither implementation receives
ARKit poses/features, IMU, depth, mesh or reference initialization. Synthetic
lounge references are simulation truth; EuRoC uses its Leica/IMU batch reference.
The actual lounge uses an uncertain ARKit device reference.

Report initialization, native quality/support counts, complete input coverage,
lost gaps and tails, finite/rigid state checks, runtime/resources and associated
trajectory error together. Use the same independent timestamp association and
Sim3 scoring policy, without filling missing poses or scoring predicted motion
as tracking. Preserve original upstream decisions: do not insert the current
port's tracking-quality or numerical behavior into upstream merely to make
its outputs match. Any different available support diagnostics must be stated.

Context used: iPTAM `SUMMARY.md`, then `SKILL.md` and dataset inventory; upstream
`README.md` and the actual camera implementation. No source/core edits, phone
operations or additional accuracy-driven pair selection were performed.
