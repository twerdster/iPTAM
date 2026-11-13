#include "OriginalPTAM.h"
#include "Tracker.h"
#include "MapPoint.h"
#include <gvars3/instances.h>
#include <TooN/so3.h>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace original_ptam {
namespace {
class ObserverTracker : public Tracker {
 public:
  using Tracker::Tracker;
  void press() { mbUserPressedSpacebar = true; }
  int quality() const { return mTrackingQuality; }
  int found() const { return static_cast<int>(mCurrentKF.mMeasurements.size()); }
  int attempted() const {
    if(mCurrentKF.mMeasurements.empty()) return 0;
    int count=0; for(int i=0;i<LEVELS;++i) count+=manMeasAttempted[i]; return count;
  }
  int trails() const { return static_cast<int>(mlTrails.size()); }
};
ATANCamera configure(const Camera& c) {
  if(c.width<=0 || c.height<=0 || !(c.fx>0) || !(c.fy>0)) throw std::invalid_argument("Invalid input calibration");
  std::ostringstream value;
  value<<std::setprecision(17)<<"["<<c.fx/c.width<<" "<<c.fy/c.height<<" "
       <<(c.cx+.5)/c.width<<" "<<(c.cy+.5)/c.height<<" "<<c.w<<"]";
  GVars3::GV2.SetVar("Camera.Parameters",value.str());
  ATANCamera camera("Camera");
  camera.SetImageSize(CVD::ImageRef(c.width,c.height));
  return camera;
}
std::array<double,4> quaternion(const TooN::Matrix<3>& r) {
  // Read-only matrix -> xyzw export; no pose is fed back to the original code.
  double q[4];
  const double trace=r[0][0]+r[1][1]+r[2][2];
  if(trace>0) { double s=2*std::sqrt(trace+1);q[3]=s/4;q[0]=(r[2][1]-r[1][2])/s;q[1]=(r[0][2]-r[2][0])/s;q[2]=(r[1][0]-r[0][1])/s; }
  else { int i=0;if(r[1][1]>r[i][i])i=1;if(r[2][2]>r[i][i])i=2;int j=(i+1)%3,k=(i+2)%3;double s=2*std::sqrt(1+r[i][i]-r[j][j]-r[k][k]);q[i]=s/4;q[j]=(r[j][i]+r[i][j])/s;q[k]=(r[k][i]+r[i][k])/s;q[3]=(r[k][j]-r[j][k])/s; }
  return {{q[0],q[1],q[2],q[3]}};
}
}
struct Engine::Impl {
  Camera input;
  Map map;
  ATANCamera camera;
  MapMaker mapper;
  ObserverTracker tracker;
  CVD::Image<CVD::byte> gray;
  explicit Impl(const Camera& c) : input(c),camera(configure(c)),mapper(map,camera),
    tracker(CVD::ImageRef(c.width,c.height),camera,map,mapper),gray(CVD::ImageRef(c.width,c.height)) {}
};
Engine::Engine(const Camera& c):impl_(new Impl(c)) {}
Engine::~Engine() = default;
Snapshot Engine::process(const std::uint8_t* gray,std::size_t stride,double timestamp,bool press) {
  if(!gray || stride<static_cast<std::size_t>(impl_->input.width)) throw std::invalid_argument("Invalid luminance buffer/stride");
  for(int y=0;y<impl_->input.height;++y) std::memcpy(impl_->gray[y],gray+y*stride,impl_->input.width);
  if(press) impl_->tracker.press();
  impl_->tracker.TrackFrame(impl_->gray,false);
  Snapshot s;s.timestamp=timestamp;s.initialized=impl_->map.IsGood();s.quality=s.initialized?impl_->tracker.quality():-1;
  s.found=impl_->tracker.found();s.attempted=impl_->tracker.attempted();s.trails=impl_->tracker.trails();
  s.map_points=impl_->map.vpPoints.size();s.keyframes=impl_->map.vpKeyFrames.size();s.message=impl_->tracker.GetMessageForUser();
  const auto cf=impl_->tracker.GetCurrentPose(); const auto wc=cf.inverse();
  s.finite=true;
  for(int r=0;r<3;++r) {for(int c=0;c<3;++c)s.camera_from_world[4*r+c]=cf.get_rotation().get_matrix()[r][c];s.camera_from_world[4*r+3]=cf.get_translation()[r];s.world_from_camera[r]=wc.get_translation()[r];}
  auto q=quaternion(wc.get_rotation().get_matrix());for(int i=0;i<4;++i)s.world_from_camera[3+i]=q[i];
  for(double value:s.camera_from_world)s.finite=s.finite&&std::isfinite(value);
  for(double value:s.world_from_camera)s.finite=s.finite&&std::isfinite(value);
  return s;
}
} // namespace original_ptam
