# Native equidistant lounge experiment

> Detailed validation record. See [current results](../RESULTS.md) for the main comparison.
> `evidence/` labels identify local validation artifacts; raw logs, app binaries
> and private inputs are not redistributed with this repository.

The lounge source calibration is ideal equidistant (`r_image = theta`), with
`fx=253.30344967`, `fy=253.34336776`, `cx=319.5`, `cy=239.5` at 640×480.
The previous adapter rectified it to a 400-pixel-focal pinhole image, reducing
horizontal field of view from about 145° to 77°. This experiment tests whether
keeping the original field of view helps the original PTAM estimator.

## Controlled comparison

Both variants use the same transaction-safe, bounded original tracker, mapper,
bundle adjuster and native MiniPatch initializer. The experiment changes only
the projection, inverse projection and analytic camera Jacobian, and supplies
the corresponding calibrated pixels. It adds no ARKit/VIO estimator, LK
initializer or truth-based selection. Initialization pairs remain the existing
image-selected pairs: reference 0/16 and new lounge 0/18, identical in both
optical variants. Each run starts a fresh map with the same seed.

Native PNG grayscale intensities are copied losslessly to PGM, without
resampling or changing the 16 invalid corner-pixel intensities. Model validity
excludes rays whose image radius is at least `pi/2 - 1e-6` radians. Source image
hashes, concatenated pixel hashes, calibration, timestamps and independent
reference poses are retained in each data directory's provenance files.

All evidence is under
`evidence/experiments/equidistant_v1`.
`plan.json` records the fixed pairs, runtime bounds and exact initial source and
binary hashes. `artifacts/` preserves the complete prototype sources, binaries
and matching guarded archive/headers; `artifacts/sha256.json` hashes every file.
The guarded archive SHA256 is
`c186aff325cba8d4efbcecf118b37b9482450a03efda472a330b82a2546ce422`.
Pinhole replay SHA256 is
`4533d014d58bdd6b90cba70ebf6f98abccd57f23858c9ac98c55ce7bd79b5511`;
equidistant replay SHA256 is
`8a89faf2128da6790ccc310e35c7c27f222993b0f0983d7031e1f6b6dc25cfe0`.
This snapshot has accepted-iterate preservation and numerical bounds, without a
minimum damping floor. Later canonical solver profiles need separate results.

## Full-sequence results

Strict GOOD requires quality 2, at least 10 visual matches, a nonempty map and a
finite rigid pose. Initialization rows and weak quality 1 do not count. Scores
use a best-fit monocular Sim3 over valid poses; position errors are after fitted
scale and do not establish metric-scale estimation. Reference motion coverage
counts only consecutive GOOD exposures and excludes every lost gap.

| Sequence/model | GOOD/all frames | GOOD/post-init | Position RMS | Adjacent rotation median | Reference motion coverage |
| --- | ---: | ---: | ---: | ---: | ---: |
| Reference, cropped pinhole | 43/230 | 43/213 | 160.1 mm | 1.430° | 11.1% |
| Reference, native equidistant | 86/230 | 86/213 | 5.1 mm | 0.051° | 32.2% |
| New lounge, cropped pinhole | 38/100 | 38/81 | 49.8 mm | 0.230° | 54.2% |
| New lounge, native equidistant | 81/100 | 81/81 | 6.3 mm | 0.049° | 95.6% |

All four runs completed, with no nonfinite/nonrigid rows or hard solver
failures. Bounded solves retaining an earlier accepted iterate occurred and are
reported rather than labeled convergence. CSVs, logs, run status and independent
metrics are preserved separately for every run.

Three complete native runs per sequence have exactly identical serialized pose
and tracker/map state fields, excluding elapsed time. Reference state hash is
`7245005edab29515e0fe0b783e6b3e5e1a64a025c855adcdd8f5221e9f69f27b`;
new lounge state hash is
`0d1753c0f41e92963e5a27c19c0932798069c53a194016bb749640871f288913`.
`repeat_plan.json` and `repeat_comparison.json` record this comparison.

## Remaining reference-sequence gap

The longer reference sequence still has no strict GOOD tracking on frames
73–199 inclusive. Frames 73–83 are weak; frames 84–199 are BAD. Recovery begins
at frame 200. This is a real limitation, despite accurate poses on tracked
segments; it prevents claiming stable full-path tracking.

The 5000-point cap is not the cause: the full run peaks at 1243 map points and
24 keyframes. There are 1135 points and 20 keyframes at first loss. The last
keyframe before loss is frame 71; the next is 201. At frame 72 there are
245/803 matches, 905 visible points and 98 warp rejections. At frame 73 the
match fraction falls below the existing 0.3 GOOD threshold (228/771), so the
GOOD-only keyframe gate stops expansion. By frame 84 there are 90/342 matches,
542 visible points and 198 warp rejections. No hard solver failure occurs.
This establishes a chain of weakening view/patch support followed by stalled
map expansion; it does not by itself distinguish texture, viewpoint change,
patch deformation and the keyframe policy as the underlying cause. Exact
selected rows and keyframe insertion indices are in
`historical_gap_diagnostics.json`.

An independent image audit subsequently found plentiful corners and persistent
tracks during the gap. The weak poses immediately after frame 72 were still
accurate. A preregistered experiment permitting supported weak frames to add
keyframes improved lounge coverage but seriously worsened EuRoC accuracy and
runtime, so it was rejected. See [the experiment](weak-keyframe-experiment.md).
The remaining limit is not solved by simply loosening tracking/map gates.

## Canonical implementation and regression checks

The resulting camera extension is runtime-selected:
`ATANCamera::ProjectionModel::{FOV,Equidistant}` with `SetProjectionModel` and
`GetProjectionModel`. Constructor default remains historical FOV; its `w=0`
case remains pinhole. Select the model before copying cameras into the tracker
and mapper. Ideal equidistant requires a zero fifth parameter. Original FOV
equations and numerical behavior remain in their existing branches.

For a z=1 camera-plane radius `r`, projection scales by `atan(r)/r`; inverse
projection scales the normalized image vector of radius `theta` by
`tan(theta)/theta`. Small-radius series avoid zero divisions. Projection and
inverse reject nonfinite or excluded front-domain coordinates and return NaN,
with invalidity propagated to derivative queries. They never fold rear rays
through tangent. Cached derivatives work after both projection and inverse.

Linear render helpers use a conservative symmetric square of radius
`tan(min(maximum image-corner angle, pi/2 - 1e-6))`. This finite enclosure
avoids evaluating invalid corners, covers all accepted rays and changes no
measurements or image intensities. Near-horizon render bounds can be large;
they do not relax the measurement validity rule.

`tests/equidistant_camera_tests.cc` checks all 307200 native pixel coordinates
(exactly 16 invalid), original FOV/pinhole equations, horizon and nonfinite
inputs, cached analytic derivatives, model copying/resizing/switching, and all
public derived bounds/scales/frustum entries. An optimized standalone build
passes with maximum pixel round-trip error `1.659e-13` and maximum analytic
versus central finite-difference Jacobian error `4.384e-8`. Canonical host and
actual-phone replay results are recorded by the main validation workflow;
the isolated numbers above must retain their own binary identity.
