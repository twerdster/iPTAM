# Retained physical-phone controls

> Detailed validation record. See [current results](../RESULTS.md) for the main comparison.
> `evidence/` labels identify local validation artifacts; raw logs, app binaries
> and private inputs are not redistributed with this repository.

These earlier executables are controls, not the final minimum-ten build.
Current results and source identity are in [phone-validation.md](phone-validation.md).
All named run directories are under
`evidence/phone`; build archives, when listed,
are under sibling `builds/`. Directory names containing `final_` do not imply
that an old run used the current executable.

## Rejected damping-floor profile

Executable SHA256:
`3d9bf8b9807b02fb63af7316590d0e1fd931333165a3648f53068e46b071f15f`.
Archive: `builds/3d9bf8b9807b02fb`, including app/source ZIPs, log and file hashes.
Source-manifest SHA256:
`43281c54459d30dcb13498ac99c8f68b900b5af6e616ff6fd19901938cd25ac3`.
This profile used a `1e-6` damping floor with bounded transaction-safe solving,
exact equidistant optics and tracker safety fixes, before mandatory serial
cleanup and minimum-ten internal support.

| Evidence | Fixed pair / model | Strict GOOD | Sim3 RMS | Reference motion coverage | Largest GOOD gap |
| --- | --- | ---: | ---: | ---: | ---: |
| `final_native_lounge_new_01/02/03` | 0/18, equidistant | 81/100 each | 5.553 mm | 95.62% | 0 |
| `final_native_lounge_reference_01` | 0/16, equidistant | 88/230 | 6.224 mm | 32.98% | 125 |
| `final_euroc_cam1_01` | 0/5, rectified pinhole | 223/3682 | 152.71 mm | 3.02% | 2593 |
| `final_euroc_cam0_01` | 1200/1205, rectified pinhole | Incomplete | No completed score | No completed score | No completed score |

Cam0 exceeded the 900-second watchdog. Its last retrieved running summary had
3180/3682 processed frames, 1301 GOOD, 269.42 ms mean and 1547.89 ms maximum
processing. The harness preserved `partial-run/` and terminated its own child;
that partial running summary is not a completed result. Three new-lounge runs
had identical per-frame state excluding processing time, but that repeatability
does not justify the floor's real-sequence regression. The floor remains an
explicit experimental option, with default minimum 0.

## Cleanup and retired-refind correction before minimum ten

Executable SHA256:
`d270bfc7e5e4c569e434af1582858f3b8581120971574bc4facbed60a00ce26c`.
It restores mandatory after-stage cleanup and rejects retired-point refind,
retaining default damping minimum 0. It precedes the final internal
minimum-ten support rule. Every listed replay installed and hashed the app.

| Evidence | Strict GOOD | Sim3 RMS | Motion coverage | Largest GOOD gap | Mean / max phone ms |
| --- | ---: | ---: | ---: | ---: | ---: |
| `restored_cleanup_lounge_new_01/02/03` | 81/100 each | 6.145 mm | 95.62% | 0 | 7.72–7.80 / 35.90 worst |
| `restored_cleanup_lounge_reference_01` | 89/230 | 5.330 mm | 33.40% | 124 | 8.55 / 90.21 |
| `restored_cleanup_euroc_cam0_01` | 914/3682 | 57.70 mm | 28.89% | 535 | 14.16 / 690.93 |
| `restored_cleanup_euroc_cam1_01` | 262/3682 | 103.63 mm | 3.90% | 2573 | 2.88 / 56.75 |
| `restored_cleanup_actual_lounge_02` | 0/142 | No map | 0% | No initialized output | 1.57 / 3.65 |

The real-lounge `_01` attempt failed validation before processing; `_02`
used the corrected serialized calibration and completed safely without
initializing. The 136-frame plane/occlusion/return fixture completed with
107 GOOD. These are retained controls, not the final run IDs.

## Earlier transaction and initialization history

Native transaction profile SHA256:
`cf98b6b4663b145ddd5bf7db7019b4a8bdf77fb107c83d7e44867ac80784aeae`.
Its archive `builds/cf98b6b4663b145d` has source-manifest SHA256
`99d84460b62505d4a2479613b30e01196df3cbc2769ad4fc9c819d79ee90e162`.
It includes native optics/live limits but predates later cleanup/support fixes.

| Evidence | Strict GOOD | Sim3 RMS | Motion coverage | Largest GOOD gap |
| --- | ---: | ---: | ---: | ---: |
| `native_lounge_new_01` | 81/100 | 6.01 mm | 95.62% | 0 |
| `native_lounge_reference_01` | 84/230 | 5.38 mm | 31.31% | 129 |
| `native_profile_euroc_cam1_01` | 534/3682 | 78.04 mm | 13.70% | 1988 |

The latter two reused the installed app; individual provenance records
`install_performed=false` and `executable_sha256=null`. Do not describe those
as independently rehashed installations. Earlier pinhole cam0
`transaction_euroc_cam0_01` records executable
`f2b7e6adbfdbe39142dfc981ae0eb4b703c1489d90c8bd016265937197bca708`:
868/3682 GOOD, 70.52 mm RMS, 25.35% motion coverage, 678-frame gap and
122.54 ms mean / 696.55 ms maximum processing.

Earlier pinhole `final_lounge_new_*` used
`cd54a4a74e6414017b4c5b89594b4607cc1397ba9aff008b78fc09aae0eef238`.
The guarded rejection control used
`199a89ad82706c1c3abf7be6a53b5825e80c2ce7565494408cd609302bfd1b76`;
its Schur diagnostic trial used
`913a8b68bb58e73255a75f5c464b0fb8efe74ee79e8490322e7a63f55736845e`.
The pinhole-only floor trial `floor_lounge_new_01` used
`aa22997e9e71502c7bf360832954270c829e08215151d2ed6dc1245a9ff8724f`.
Inputs, camera models, initialization pairs and binaries must remain associated;
these controls are not interchangeable with native full-field lounge replays.

See [cleanup scheduling](cleanup-scheduler-experiment.md),
[minimum support](minimum-support-experiment.md),
[optics](equidistant-optics-experiment.md),
[rejected weak mapping](weak-keyframe-experiment.md) and
[archived host baseline](archive-baseline.md) for exact methods and failure
evidence. No favorable subset score overrides a long lost span or timeout.
