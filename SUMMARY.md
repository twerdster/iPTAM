# Project summary

## Current state

This repository contains the modern restoration of the 2010 iPhone PTAM port,
a separately built official Oxford baseline, and an experimental implementation
inspired by the 2009 camera-phone PTAM paper. Read `README.md` for the overview,
`docs/BUILDING.md` for setup, and `docs/RESULTS.md` for measured outcomes.

The restored C++ core keeps PTAM's tracker, patch matcher, homography initializer,
mapper, relocalizer and bundle adjuster. The supported integration owns them on
one serial worker. ARKit poses, IMU, depth and reference trajectories are not
estimator inputs. A finite motion prediction is not accepted visual tracking.

Mobile algorithms are guarded by `IPTAM_MOBILE_PAPER` (default OFF). The mobile
app/CLI use `auto`; explicit `paper` keeps the fixed-pair ablation. Features
include multiscale Shi–Tomasi detection, coarse patch tracking, partial inverse
depth, bounded serial/idle mapping, observation/keyframe culling and automatic
planar initialization. Rolling-shutter correction requires verified prepared
image timing and stays disabled on the five benchmark inputs.

## Results and limits

Nine full automatic phone trials processed 15,400 frames. Accepted counts:
historical simulated lounge 142/230, new lounge 82/100 in three repeats,
EuRoC-left 329/3682, right 1/3682 in three repeats, actual sparse lounge 0/142.
New-lounge fitted RMS is 2.4–4.2 cm over 95.16% reference motion;
timed poses differ. These results do **not** establish general stable scanning.
The two EuRoC views share one environment. ARKit recording references have
uncertainty; synthetic lounge geometry is photo-fitted.

Two separate fixed-pair phone trials add 3,782 frames; two compatibility runs
on the error-receipt revision add 3,782. Host validation has 31 full trials and
47,116 finite rigid rows. Twelve Release, twelve optimized ASan/UBSan, 24 Python,
five macro-disabled tests and ten physical malformed/background checks passed.
Build hashes, policies and aggregate tables are in `docs/results/`.

A legacy reversed depth window, allocator-dependent partial prior and deferred
promotion starvation were corrected without relaxing geometry/support gates.
Mapping's 4 ms cooperative budget can overrun; a 28.12 ms cull was measured.
All five unbudgeted automatic host repeat pairs are exact; timed scheduling can
change maps.

## Repository map and publication

- `ios/`: modern recording-first iOS shell; builds use configurable signing and bundle IDs.
- `xPTAMclean/iPTAM/`: restored core and guarded mobile algorithms.
- `tests/`, `tools/`: regression tests, preparation, replay, evaluation and visualization.
- `upstream/`: pinned compatibility-only Oxford PTAM baseline.
- `docs/testing/`: detailed and historical reports; current results are in `docs/RESULTS.md`.
- `docs/assets/lounge-tracking.gif`: full ten-second saved-phone visualization, below 2 MB.
- `LICENSES.md`: original component notices and attribution.

This public snapshot uses generic configuration. Raw recordings, signed apps,
private machine inventories and local evidence paths are not included. Detailed
reports use descriptive relative evidence labels; they do not imply raw logs are
bundled. Historical source attribution and licensing notices remain intact.

Publication checks separately pass 12 host Release suites, 32 Python tests
(including eight portable configuration contracts), and an unsigned iOS build.
The GIF is 1,852,923 bytes and all 100 frames decode. See
`docs/results/publication-checks.json`. The public setup makes device, signing,
bundle and storage values configurable; no new tracking benchmark is implied.
Two historical example video paths and dispatch queue labels are generic.


## Modern-update navigation

The root README links through a local badge to
`updated/README.md#updated-and-ported-for-modern-hardware`. That page provides a
short guide to the restored core, guarded mobile algorithms, build instructions
and the current five-input results. Keep it synchronized with the main docs.
The original attribution and licensing notices remain intact.

Before committing, review and update this file and `SKILL.md` when behavior,
commands, results or handoff context changes. Use `SKILL.md` for the workflow.
