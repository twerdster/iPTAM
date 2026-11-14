# iPTAM updates

## Updated and ported for modern hardware

The restoration retains PTAM's C++ tracker, patch matcher, initializer, mapper
and bundle adjuster, with portable builds and a modern iOS recording-replay app.
It fixes a reversed depth interval and mapping/promotion defects; a separately
selected mobile variant adds new feature tracking, inverse-depth landmarks and
bounded mapping inspired by the camera-phone PTAM paper. These are changes to
the estimator and its integration, beyond a platform wrapper; real recordings
still show substantial tracking failures.

### Code and setup

- [Restored core and mobile variant](../xPTAMclean/iPTAM/) and [iOS app](../ios/).
- [Build and replay guide](../docs/BUILDING.md): host configuration, signing,
  prepared recordings and the distinct restored/mobile build options.
- [Mobile implementation](../docs/testing/mobile-implementation.md) and
  [restoration analysis](../docs/report-analysis.md): retained algorithms,
  repairs and new components.

### Validation and limits

[Results and limitations](../docs/RESULTS.md) report whole-recording support,
tracking gaps, accuracy and repeatability. The [tests](../tests/) and
[publication checks](../docs/results/publication-checks.json) cover different
parts of the build/replay workflow. Reference trajectories, ARKit poses, depth
and IMU do not feed the estimator. The [upstream baseline](../upstream/README.md)
is kept separately; the mobile variant is a new implementation on the restored
core, rather than the unavailable historical commercial phone source.

[Project reference and attribution](../README.md#attribution-and-licensing)
