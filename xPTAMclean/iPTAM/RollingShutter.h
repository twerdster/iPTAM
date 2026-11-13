#pragma once
// First-order visual rolling-shutter correction from Klein/Murray ISMAR 2009,
// section 6. No IMU, reference pose or guessed sensor timing is used.
#include "TooN.h"
#include <cstddef>
#include <string>
#include <vector>

namespace IPTAMMobile {
struct RollingShutterCalibration {
  bool known = false; // Set only for verified timing in these pixel coordinates.
  int width = 0, height = 0;
  int axis = 1;       // 0: columns; 1: rows of the supplied image.
  int direction = 1;  // +1: increasing coordinate is later; -1: earlier.
  double readoutSeconds = 0.0; // First to last pixel-center exposure time.
  double timestampFraction = 0.5; // Timestamp location in first-to-last readout.
  bool Validate(std::string& reason) const;
  bool TimeOffset(const TooN::Vector<2>& pixel, double& seconds,
                  std::string& reason) const;
};

struct RollingShutterObservation {
  TooN::Vector<2> pixel = TooN::Zeros;
  TooN::Matrix<2,6> poseJacobian = TooN::Zeros; // Root pixels / left SE3 increment.
  double timestamp = 0.0;
  bool hasPrevious = false, hasNext = false;
  TooN::Vector<2> previousPixel = TooN::Zeros, nextPixel = TooN::Zeros;
  double previousTimestamp = 0.0, nextTimestamp = 0.0;
};

struct RollingShutterEstimate {
  bool valid = false;
  TooN::Vector<6> cameraVelocity = TooN::Zeros; // Left SE3 coordinates / second.
  std::size_t observations = 0;
  double conditionRatio = 0.0;
  double residualRmsPixelsPerSecond = 0.0;
  std::string reason;
};

class RollingShutter {
public:
  // Adjacent *video-frame* observations, not arbitrarily separated keyframes.
  // At least six observations and a full-rank 6-DOF information matrix are
  // required. Malformed supplied timing fails closed; missing neighbors may be
  // skipped. The caller must not fabricate timestamps for this API.
  static RollingShutterEstimate EstimateVelocity(
      const std::vector<RollingShutterObservation>& observations,
      const RollingShutterCalibration& calibration);
  // Always correct the original measured pixel. Reapplying this to a previously
  // corrected measurement would incorrectly accumulate the correction.
  static bool Correct(const TooN::Vector<2>& pixel,
      const TooN::Matrix<2,6>& poseJacobian,
      const RollingShutterEstimate& estimate,
      const RollingShutterCalibration& calibration,
      TooN::Vector<2>& corrected, std::string& reason);
};
}
