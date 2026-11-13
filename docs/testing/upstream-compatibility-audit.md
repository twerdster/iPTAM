# Upstream PTAM compatibility audit

> Detailed validation record. See [current results](../RESULTS.md) for the main comparison.
> `evidence/` labels identify local validation artifacts; raw logs, app binaries
> and private inputs are not redistributed with this repository.

Audited 13 November 2026. The comparison port preserves the selected official
PTAM estimator and its defaults. Its changes provide build, input, display and
logging compatibility; none of the restored iPTAM tracking or numerical fixes
are imported. This is a source-preservation finding, not a claim that the
original implementation is safe, accurate or stable on all recordings.

## Source identity and historical scope

The [official Oxford project page](https://robots.ox.ac.uk/~gk/PTAM/) links the
GPL repository. Its [README](https://github.com/Oxford-PTAM/PTAM-GPL/blob/d9dca71ad57b0ef5b409e1670443060d49df63cb/README.md)
identifies the source as a re-licensed fork of v1.0-r114. Oxford's
[original release README](https://robots.ox.ac.uk/~gk/PTAM/README.txt) dates that
package to 29 January 2010. The selected `Src` tree is identical to Georg
Klein's import commit `3c6a6a6be5be529a130db1c09d89d13f95d2dd0a`; the later
selected commit changes only README text relative to that import.

This establishes a contemporary r114 lineage. The exact downloaded
pre-fork revision and original dependency snapshots are unknown, so this is
not claimed to reproduce the original desktop executable byte for byte.

The [pinned recipe](../../upstream/pins.json) selects:

| Component | Authoritative repository | Exact commit | Commit date |
| --- | --- | --- | --- |
| PTAM | [Oxford-PTAM/PTAM-GPL](https://github.com/Oxford-PTAM/PTAM-GPL) | `d9dca71ad57b0ef5b409e1670443060d49df63cb` | 5 November 2013; r114 import lineage |
| TooN | [edrosten/TooN](https://github.com/edrosten/TooN) | `70a52c159cefec1bcf5ca9ed93a61e55663f6f43` | 21 January 2010 |
| LibCVD | [edrosten/libcvd](https://github.com/edrosten/libcvd) | `0c7e481cc81606e32b079a6adb046b380f8349af` | 22 January 2010 |
| GVars | [edrosten/gvars](https://github.com/edrosten/gvars) | `5f9c50a35ab0c169c0b8d0f79abf2ae118665e8d` | 4 November 2009 |

The original checkout remains pristine. `UPSTREAM-LICENSE` and
`UPSTREAM-README.md` are byte-identical copies from that checkout. The GPLv3
license and dependency copyright/license files remain in the source archive.
The [bootstrap instructions](../../upstream/README.md) reject modified existing
checkouts and unexpected repository origins.

## Compatibility boundary

Only `Tracker.cc` differs among the 51 copied PTAM source files:

- Replace the OpenGL header with the headless rendering shim.
- Remove the LibCVD OpenGL helper include. The adapter always passes
  `bDraw=false`, an existing tracker option.
- Include `unistd.h` to declare the existing POSIX `usleep` call.

No estimator statement, formula or branch changes. TooN changes are limited to
declaration order, dependent-name lookup, template parameter names and a modern
equivalent compile-time size assertion. LibCVD uses its original scalar/noarch
implementations on arm64, with generated configuration headers selecting
pthreads, TooN and LAPACK/Accelerate. GVars uses its original headless backend;
its unused shell command reports unsupported on iOS, where `system()` cannot
compile. That guard leaves macOS behavior unchanged.

The adapter supplies calibrated grayscale pixels and initialization presses,
then exports the original pose and quality. Camera parameters use the original
GVars interface and pixel-center convention. It supports the original FOV
camera, with `w=0` for the common pinhole recordings. It does not add the new
ideal equidistant model. Inputs, pairs and scheduling are defined separately in
the [common-input plan](upstream-input-plan.md).

Preserved behavior includes the asynchronous MapMaker and its 5 ms polling
sleep, original scheduling hierarchy, `rand()` and `random_shuffle`, per-frame
motion prediction, initializer, matcher, solver, recovery and keyframe policy.
No `srand()` call is added. Source timestamps control replay scheduling and
logging, not the original estimator's motion model. Default source-paced replay
keeps every exposure and logs backlog; `--as-fast` is a separate throughput
profile that can starve the background mapper.

The original defaults include 1,000 initialization trails, coarse minimum 20,
keyframe distance multiplier 1.0, Tukey estimators, quality fractions 0.3/0.13,
bundle iteration setting 20 and update convergence threshold `1e-6`.
The historical iPTAM hardcoded values differ in some places, including 100 trails,
coarse minimum 10 and keyframe multiplier 2. Those values are not substituted
into the upstream baseline. Neither restored support gates nor bounded solvers,
deterministic per-class RNG, serial mapping, timestamp prediction or map-point
caps are copied into it.

## Independent verification and build identity

The audit recomputed all 51 PTAM source hashes and all 522 generated dependency
file hashes against the inventories, with no mismatch. Only the enumerated
Tracker include changes differ from upstream. The canonical recipe's adapter,
CMake, rendering shim and driver match the tested port. Its generator differs
only in portable default paths and creating the patch directory.

The independently checked iOS app executable is:

```text
64e342dfb7e4b06dd01f40f1ce84a2b67ac7815f9e971fd2c5f0aa13b48434c6
```

Archive root:
`evidence/builds/64e342dfb7e4b06d/`.
The executable hash matches `identity.json`. All 585 entries in
`source/upstream-source.zip` match `source/upstream-source.json`; there are no
missing or extra ZIP entries. The archive also preserves the app-shell source
and build log. Key SHA256 receipts are:

| Artifact | SHA256 |
| --- | --- |
| `source-inventory.json` | `8c09c85b7eda32775d70958e76b66ab11a4cb92ad0e4e0f370fd22ca8996ad78` |
| `dependency-inventory.json` | `671b09d472815e7d05b01b7b7e1053e26fb6afce9e9eb6325b71101461a00c33` |
| `patches/compatibility.patch` | `7d44add295e286023b927299a81e7070d44f9f0a808e421e7e49a7798b84d950` |
| `patches/dependency-compatibility.patch` | `085f4481717c60c5fb1c51ef12e9bf7278e87a123b7aa125d4a7b2a77828657b` |
| `source/upstream-source.zip` | `05925ac34a68da4c9bb84a08e739fc125149ac707c1426d3d2a4ec16f3437350` |

The core owner's separate fresh bootstrap reproduced all 51 generated PTAM,
522 generated dependency, two adapter and one shim files. It also reproduced
the exact host executable hash
`7b99592a2187c6edba1d145e0adaa93130bf0976cbcf8c7f7021357228701266`.
Its receipt is
`testing/upstream_comparison/portable_core_reproduction_20261113/reproduction.json`
in the local evidence archive. A 100-frame source-paced smoke completed with no nonfinite output;
two runs of the same binary produced 32 and 33 raw-supported GOOD frames.
That variation is consistent with preserved asynchronous scheduling. These
smokes establish build/run reproducibility, not deterministic tracking or
general stability. Physical comparison outcomes belong in the results report.

## Diagnostic and comparison limits

The observer does not change original quality, found flags, measurements, map
state, solver state or estimator pose. It does not reproject live map points
after a frame: that would add unsynchronized reads while the original mapper
can update or reset them. Consequently:

- `pose_support_diagnostics_available=false` and
  `solver_diagnostics_available=false`. `pose_inliers=-1` means unavailable and
  is omitted from JSON/CSV metrics. Missing solver counters must not be reported
  as a measured zero.
- `found` counts unique measurements in the current keyframe, rather than the
  original raw match-counter sum. `attempted=0` when that measurement set is
  empty can mean unavailable, not that no search was attempted.
- Map and keyframe sizes are approximate asynchronous observations, following
  the original status pattern. They are not an atomic map snapshot. The original
  shared-map/threading hazards remain; this audit does not certify thread safety.
- Original GOOD is retained. External GOOD plus at least ten measurements and
  a finite rigid pose is raw-supported output, not the restored final-pose
  geometric-support criterion. The distinction must remain visible in results.
- Per-frame processing time includes tracker and export work but excludes
  background mapper CPU. It cannot be directly compared with serial
  tracker-plus-mapper latency. Report wall time and scheduling backlog too.

Original numerical failures and unbounded loops remain possible. Tests use
external timeout/NaN/process watchdogs and preserve partial failure evidence;
they do not silently repair, restart or relax the baseline.

Several defects addressed during restoration also exist in this upstream
source, including pixel-vector construction before triangulated position,
stale projection/derivative conditions, coarse-point replacement, zero-scale
robust weights and ignored initialization results. Their presence in the original
port does not establish who introduced them. The exact source comparison,
not the historical failure alone, determines that attribution.
