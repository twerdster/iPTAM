# Rejected weak-frame mapping experiment

> Detailed validation record. See [current results](../RESULTS.md) for the main comparison.
> `evidence/` labels identify local validation artifacts; raw logs, app binaries
> and private inputs are not redistributed with this repository.

This isolated change is **not integrated**. Allowing well-supported weak
frames to add keyframes improved lounge coverage, but caused a major EuRoC
accuracy and runtime regression. Tracking GOOD/weak/BAD labels were unchanged.

The hypothesis followed a specific observation in the native equidistant
reference lounge run: GOOD ended at frame 72, weak poses remained accurate,
and the GOOD-only mapping gate prevented further keyframes until recovery at
200. Independent image analysis found plentiful texture throughout the gap,
and independent evaluation fitted only to strict GOOD poses measured weak
frames 73–83 at 4.96 mm position RMS and 0.231° median rotation error. This
supports testing the gate; it does not establish that relaxed mapping is safe.
Those audits are preserved at
`evidence/host/native_lounge_quality_20261112`.

## Preregistered change

Only a copied `Tracker.cc` changes. A DODGY frame can pass the mapping quality
gate when it has at least 100 found matches, found/attempted at least 0.20,
found/visible at least 0.15, finite rigid pose, positive finite scene depth and
no tracker numerical failure. The original mapmaker need/distance, timestamp,
queue and insertion checks remain. BAD frames remain excluded. No threshold
was selected or revised using reference trajectories.

The unchanged frozen transaction-safe guarded archive is identical to the
optics experiment, SHA256
`c186aff325cba8d4efbcecf118b37b9482450a03efda472a330b82a2546ce422`.
The raw lounge runs use the same exact equidistant override and unchanged
pixels as that experiment; EuRoC uses the original pinhole model. Fixed
image-selected pairs are reference 0/16, new 0/18, EuRoC left 1200/1205 and
EuRoC right 0/5. Each run starts fresh with the same seed. An external
120-second bound was declared before running.

All evidence is under
`evidence/experiments/weak_kf_v1`:
`plan.json`, `comparison.json`, per-run commands/status, flushed CSVs, logs,
independent metrics, and `artifacts/` with source/object/executables plus
SHA256s. Copied Tracker source SHA256 is
`7a8cfb96aa03dedd5c0decc53fd8b144affbafeb7d1d2de61e0f62a0a97c9793`;
weak equidistant binary SHA256 is
`63ce14698c6b898ecc6e1d5b0b6d43325ea0989653b4b0d9bb5313d326835eb0`;
weak pinhole binary SHA256 is
`1e71eae517f3a464eb7ecc07fef6c20a7ad06eaa92eb3ffbee334e19a327e166`.
The immutable core snapshot is preserved with the optics experiment. This
experiment predates the final canonical damping profile and later search
guard fixes; those identities must not be conflated.

## Results

Strict GOOD requires quality 2, at least 10 matches, a map and finite rigid
pose. Position RMS uses best-fit monocular Sim3, on scored valid poses only;
it does not establish metric scale. Missing rows and gaps remain failures.

| Sequence | GOOD-only control | Weak mapping variant | Position RMS control → variant | Motion coverage control → variant |
| --- | ---: | ---: | ---: | ---: |
| Native reference lounge | 86/230 | 135/230 | 5.06 → 7.00 mm | 32.2 → 56.0% |
| Native new lounge | 81/100 | 81/100 | 6.33 → 6.33 mm | 95.6 → 95.6% |
| EuRoC left | 612/3682 | 671 GOOD in **2847/3682 processed**, timed out | 44.0 → **834.8 mm** | 21.8 → 23.4% |
| EuRoC right | 262/3682 | 262/3682 | 103.6 → 103.6 mm | 3.90 → 3.90% |

The EuRoC-left variant hit the external bound after 120.02 seconds; its
remaining 835 frames were not processed. The flushed 2847-row prefix is
preserved, with `timed_out=true` and `complete=false`. The control restricted
to those same 2847 frames still has the same 612 GOOD poses and 44.0 mm RMS,
so its accuracy advantage is not an artifact of different exposure intervals.
An evaluator's geometric `failure=null` alone does not indicate run completion;
the run-status file is authoritative for this timeout.

EuRoC-left adjacent rotation median error worsened from 0.0417° to 0.0576°.
No nonfinite/nonrigid rows were observed, illustrating why finite state and
larger match coverage alone are insufficient acceptance criteria. The right
variant/control produce exactly the same evaluated trajectory metrics.

Historical reference coverage improves, but still has a 73-frame largest
GOOD gap (135–207), plus short weak gaps at 73–74, 77, 113 and 133. New lounge
remains exactly unchanged in pose/state. No further threshold relaxation was
attempted.

## Mapping cost and decision

| Sequence | Max points / keyframes, control → variant | Host mean / p95 processing ms, control → variant |
| --- | --- | --- |
| Reference lounge | 1243/24 → 1749/41 | 6.46/38.82 → 13.07/99.48 |
| New lounge | 1267/15 → 1267/15 | 5.61/17.94 → 5.67/17.92 |
| EuRoC left | 5000/56 → 5000/60 (partial) | 7.12/19.22 → 40.97/177.42 (partial) |
| EuRoC right | 383/37 → 383/37 | 2.27/2.91 → 2.21/2.87 |

These are host timings with at most two concurrent replay jobs, not phone
performance claims. EuRoC-left weak-run RSS was observed at about 176 MiB
mid-run; this is a lower bound, not a measured peak. The extra map work and
accuracy regression outweigh the lounge gain. Keep canonical GOOD-only
keyframe insertion and retain this failed experiment as evidence. A future
mapping policy needs stronger geometric support/pose validation and bounded
cost, rather than treating many photometric matches as sufficient.
