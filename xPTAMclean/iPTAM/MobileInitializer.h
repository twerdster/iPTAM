#pragma once
// Experimental implementation of Klein/Murray ISMAR 2009 section 5.1.
// A planar first view is required; arbitrary scenes and pure rotation cannot
// establish a metric stereo baseline. No reference poses enter this interface.
#include "ATANCamera.h"
#include "KeyFrame.h"
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace IPTAMMobile {
class MobileInitializer {
public:
  struct Options {
    int maxFeatures = 180;
    int minimumInliers = 20;
    double maximumReprojectionError = 3.0; // Root pixels.
    double minimumConditionRatio = 1e-6; // Decomposition JtJ min/max eigenvalue.
    double minimumMedianParallaxRadians = 0.005;
    double maximumPatchRms = 25.0;
    int patchSearchRadius = 4; // Pixels at L1, matching the paper's search scale.
  };
  struct HypothesisDiagnostic {
    int id = -1;
    int positiveDepthPoints = 0;
    int poseInliers = 0;
    double conditionRatio = 0.0;
    double medianParallaxRadians = 0.0;
    double robustCost = 0.0;
    bool valid = false;
    TooN::SE3<> secondFromFirst;
  };
  struct Result {
    bool ready = false;
    std::string stage = "not_started", reason;
    int tracked = 0, inliers = 0, chosenHypothesis = -1;
    double conditionRatio = 0.0, medianParallaxRadians = 0.0;
    double homographyCost = 0.0;
    TooN::Matrix<3> homography = TooN::Identity;
    TooN::SE3<> secondFromFirst;
    std::vector<std::pair<CVD::ImageRef,CVD::ImageRef>> matches;
    std::vector<HypothesisDiagnostic> hypotheses;
  };

  MobileInitializer();
  explicit MobileInitializer(const Options& options);
  ~MobileInitializer();
  MobileInitializer(const MobileInitializer&) = delete;
  MobileInitializer& operator=(const MobileInitializer&) = delete;
  void Reset();
  bool Begin(const KeyFrame& first, const ATANCamera& camera, double timestamp);
  Result Process(const KeyFrame& current, const ATANCamera& camera, double timestamp);
  // The selected map is bootstrapped from first/current, using the ordinary
  // mapper's triangulation and BA checks. Intermediate frames disambiguate the
  // hypotheses; they are not retained as an additional third BA keyframe.
  const KeyFrame& FirstKeyFrame() const;
  // Geometric frontend boundary, also used by deterministic synthetic tests.
  // Coordinates are measured image pixels, paired to the original first view.
  // It runs the identical incremental SL3 fit and multi-hypothesis state machine
  // used by Process, without requiring a particular image matcher.
  Result ProcessCorrespondences(
      const std::vector<std::pair<TooN::Vector<2>,TooN::Vector<2>>>& matches,
      const ATANCamera& camera, double timestamp);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
