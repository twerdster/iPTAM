#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace original_ptam {
struct Camera {
  int width = 640, height = 480;
  double fx = 460, fy = 460, cx = 319.5, cy = 239.5, w = 0;
};
struct Snapshot {
  double timestamp = 0;
  int quality = -1; // Original BAD=0, DODGY=1, GOOD=2; -1 means no initialized map.
  bool initialized = false, finite = false;
  int found = 0, attempted = 0, trails = 0, pose_inliers = -1;
  bool solver_diagnostics_available = false;
  bool pose_support_diagnostics_available = false;
  std::size_t map_points = 0, keyframes = 0;
  std::array<double, 12> camera_from_world{}; // row-major 3x4
  std::array<double, 7> world_from_camera{}; // translation then quaternion xyzw
  std::string message;
};
class Engine {
 public:
  explicit Engine(const Camera&);
  ~Engine();
  Engine(const Engine&) = delete;
  Engine& operator=(const Engine&) = delete;
  // timestamp is logged, never supplied to upstream's original motion model.
  // Press on the first and selected second exposure, exactly like spacebar.
  Snapshot process(const std::uint8_t* gray, std::size_t stride,
                   double timestamp, bool initialize_press = false);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace original_ptam
