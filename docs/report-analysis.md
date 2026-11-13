# Original intent and restoration criteria

Reviewed 2026-11-12 from both rendered pages of `iPTAM report.pdf`, `README.md`, and the original source at commit `94afb6359226a8ee140ddd0701dc23e96529785f`. No project `SUMMARY.md` or `SKILL.md` existed at initial review. The PDF contains two text pages and no experimental figures or numerical benchmark tables.

## What Aaron was trying to build

The 2010 course project aimed to port Klein and Murray's PTAM to the iPhone 4, run it on live camera frames at frame rate, and obtain a markerless six degree of freedom camera pose for augmented reality. The intended work was a port of the existing algorithm, followed by speed and robustness improvements inspired by the 2009 camera-phone PTAM paper. It was not an attempt to reimplement the whole algorithm or to use a platform tracking API.

The port removes GVars configuration/menu dependencies and unused LibCVD code, includes reduced LibCVD and TooN implementations directly in the iOS build, connects Apple's camera capture to the tracker, and replaces desktop drawing with OpenGL ES. Camera parameters were measured from an iPhone recording using the desktop PTAM calibrator, then hardcoded.

PTAM uses two views of a roughly planar, textured scene for initialization, then estimates camera pose from measured map-point patches. Its mapper adds keyframes and triangulated points and optimizes poses and points using bundle adjustment. The small blurry image rotation estimate and decaying velocity model provide predictions; they do not themselves establish accurate six degree of freedom tracking.

## What the report actually achieved

The report says planar initialization eventually produced map points and started the tracking and mapping threads. It also explicitly says no map-point matches were found afterward, causing immediate recovery mode. Aaron disabled recovery to keep the motion-model prediction running. The reported demonstration could follow substantial rotations and small translations, but failed under scale changes and drifted under quick motion. The report describes this as using only the predictive part, without true closed-loop tracking or ongoing map building.

Consequently, compiling and displaying the historical app is not proof of restoring the intended project. A restoration must demonstrate image-supported pose correction after initialization, continuing map growth, honest lost/recovering states, and repeatable replay results. ARKit poses can provide comparison data for a lounge recording, but substituting them for PTAM's estimator would not fulfill this restoration.

## Source findings in the original snapshot

These are verified defects or risks in the historical code, not a claim that one isolated defect explains every reported 2010 failure. Modernization changes may move or remove the cited line numbers.

The subsequent official-upstream audit found several of these defects in Oxford
PTAM itself, including premature pixel-vector construction, coarse-point
replacement, stale derivative conditions and zero robust scales. Their presence
in Aaron's snapshot does not establish that his changes introduced them. The
[upstream comparison](testing/upstream-phone-comparison.md) preserves them in its
baseline and measures whole implementations; isolating a specific historical
change would require a separate controlled ablation.

| Area | Evidence in the original snapshot | Consequence / restoration requirement |
| --- | --- | --- |
| Recovery bypass | `Tracker.cc:236`, `if(1)//mnLostFrames < 3)` | The intended lost/relocalization path is unreachable. Restore it and verify recovery using actual map matches. |
| Initialization result | `Tracker.cc:447-448`, `InitFromStereo` return ignored | Failed initialization is marked complete, preventing a clean retry. Advance only on success and expose useful retry guidance. |
| Camera conversion | `MyVideoBuffer.mm:134-146` casts to byte before shifting | Red/green terms become zero; the input effectively uses only BGRA's blue channel. It also ignores row stride and assumes 640x480. Use the capture luminance plane or correct stride-aware conversion. |
| Calibration | `ATANCamera.cc:15-17`, fixed iPhone 4 parameters and dimensions | A modern phone, lens, crop or replay dataset needs its own calibration. Keep all image transformations consistent with the intrinsics. |
| Point initialization | `MapMaker.cc:257`, `RefreshPixelVectors` precedes `v3WorldPos` assignment at line 271 | Reads uninitialized 3D position while creating patch warps. Calculate the vectors after valid triangulation. A later global-transform refresh does not make the earlier read valid. |
| Coarse point selection | `Tracker.cc:689`, assignment to `vNextToSearch` | Level 2 points replace rather than extend already selected level 3 points. Append them. |
| Projection state | `TrackerData.h:19-21,62,77-81` | Uninitialized flags, inclusive width/height bounds, and derivative refresh conditioned on an old `bFound` value can make behavior depend on stale state. Initialize state and validate projections before using derivatives. |
| Robust loss | `MEstimator.h:77-86,111-120,155-164` | A zero median gives zero scale and division by zero; the small-sample correction is singular at three measurements and uses unsigned size arithmetic. Clamp robust scales and define behavior for insufficient support. |
| Flat image handling | `SmallBlurryImage.cc:184-199`; `PatchFinder.cc:271-273` | Singular image-gradient Hessians need explicit rejection. Blank/low-texture input must yield a lost/uninitialized state, not NaN poses. |
| Image copying | `KeyFrame.cc:100-109` | `Level::operator=` omits `vCandidates`; image and candidate metadata can become inconsistent. Copy all state and handle self-assignment. |
| Ownership and concurrency | `Map.h:8-13`; `MapMaker.cc:39-52`; `MapPoint.h`; `Tracker.h` | Shared vectors, poses, and reset flags are not protected; resets leak keyframes and per-point private data. Serialize access or synchronize it, then define ownership before introducing destructors because keyframes shallow-copy `pSBI`. |
| Pose output | `System.cc:28-31` | Rotation angle conversion is twice the expected degrees and axis normalization uses that degree value; identity divides by zero. Export a documented transform or normalized quaternion. |
| Timing | `Tracker.cc:1050-1054` | Velocity prediction assumes consecutive frames and ignores timestamps. Replay must preserve capture timing, and dropped-frame behavior must be measured. |

An isolated C++ probe invoking the original `MEstimator.h` routines confirmed that zero residuals produce non-finite Tukey weights for 1, 3 and 10 measurements (sigma was zero, NaN and zero respectively). This is a reproduced numerical defect, separate from the report's historical matching failure. The probe bypassed unrelated TooN/global includes; it was not a full-estimator test.

## Faithful modernization and validation

Keep the PTAM tracker, mapper, patch matching, bundle adjustment and recovery. Separate their C++ computation from the iOS capture/display shell so the same estimator can run deterministic recorded sequences on the host and live frames on the phone. Log timestamps, initialization state, attempted/found map measurements, tracking quality, map/keyframe counts, finite pose checks, frame latency and resets.

Start with synthetic textured planar imagery with known camera motion to test initialization and subsequent reprojection correction. Include static images, low texture, temporary occlusion, failed initialization and repeated reset/recovery. Run memory and undefined-behavior checks on this path. Then replay real data and report failures as well as successes.

For a lounge ARKit recording, verify that there are actual RGB frames or video plus per-frame calibration/timestamps; a point cloud or mesh alone cannot test PTAM's image tracker. Treat ARKit poses as a reference with its own uncertainty. For EuRoC, feed one calibrated camera to this monocular algorithm; do not describe it as a visual-inertial estimator because this port does not consume IMU data. EuRoC radial-tangential distortion is not interchangeable with PTAM's single-parameter ATAN model: rectify consistently and supply the resulting pinhole intrinsics, or implement the actual camera model.

Monocular PTAM scale is not observable in meters. `MapMakerWiggleScale = 0.1` assigns a chosen initial baseline rather than measuring one. Compare translation to metric reference trajectories after a documented similarity alignment, and report rotation error, tracked coverage, recovery behavior and runtime alongside translation error. A visually plausible grid or a finite pose alone is insufficient evidence of accuracy or stability.

Successful restoration needs repeatable initialization, nonzero image-supported map matches over sustained motion, sensible map growth, bounded memory across reset cycles, no numerical errors, recoverable loss, and acceptable phone performance over a representative live session. It cannot promise reliable tracking in textureless rooms, pure-rotation initialization, fast blurred motion or arbitrary large trajectories simply from porting the old algorithm.
