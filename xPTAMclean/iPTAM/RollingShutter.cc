#include "RollingShutter.h"
#include "SymEigen.h"
#include <cmath>
#include <limits>

namespace IPTAMMobile {
namespace {
template<int N> bool Finite(const TooN::Vector<N>& value) {
  for(int i=0;i<N;++i) if(!std::isfinite(value[i])) return false;
  return true;
}
template<int R,int C> bool Finite(const TooN::Matrix<R,C>& value) {
  for(int r=0;r<R;++r) for(int c=0;c<C;++c)
    if(!std::isfinite(value[r][c])) return false;
  return true;
}
}

bool RollingShutterCalibration::Validate(std::string& reason) const {
  if(!known) { reason="rolling shutter timing is unknown"; return false; }
  if(width<2 || height<2 || (axis!=0 && axis!=1) ||
     (direction!=1 && direction!=-1) || !std::isfinite(readoutSeconds) ||
     readoutSeconds<=0.0 || !std::isfinite(timestampFraction) ||
     timestampFraction<0.0 || timestampFraction>1.0) {
    reason="invalid rolling shutter calibration"; return false;
  }
  reason.clear(); return true;
}

bool RollingShutterCalibration::TimeOffset(const TooN::Vector<2>& pixel,
    double& seconds,std::string& reason) const {
  seconds=0.0;
  if(!Validate(reason)) return false;
  if(!Finite(pixel) || pixel[0]<0.0 || pixel[1]<0.0 ||
     pixel[0]>width-1.0 || pixel[1]>height-1.0) {
    reason="rolling shutter pixel is outside calibrated sensor coordinates";
    return false;
  }
  // readoutSeconds is explicitly first-center to last-center, so endpoints
  // correspond exactly to fractions 0 and 1. Pixel resampling/rotation requires
  // a separately transformed calibration, never inferred from image dimensions.
  double fraction=pixel[axis]/double((axis==0?width:height)-1);
  if(direction<0) fraction=1.0-fraction;
  seconds=(fraction-timestampFraction)*readoutSeconds;
  if(!std::isfinite(seconds)) { reason="nonfinite readout offset"; return false; }
  reason.clear(); return true;
}

RollingShutterEstimate RollingShutter::EstimateVelocity(
    const std::vector<RollingShutterObservation>& observations,
    const RollingShutterCalibration& calibration) {
  RollingShutterEstimate result;
  if(!calibration.Validate(result.reason)) return result;
  TooN::Matrix<6> information=TooN::Zeros;
  TooN::Vector<6> rhs=TooN::Zeros;
  std::vector<TooN::Vector<2>> velocities;
  std::vector<TooN::Matrix<2,6>> jacobians;
  double commonTimestamp=std::numeric_limits<double>::quiet_NaN();
  for(const auto& observation:observations) {
    if(!observation.hasPrevious && !observation.hasNext) continue;
    if(!std::isfinite(observation.timestamp) || !Finite(observation.pixel) ||
       !Finite(observation.poseJacobian)) {
      result.reason="nonfinite rolling shutter observation"; return result;
    }
    if(std::isfinite(commonTimestamp) &&
       std::abs(commonTimestamp-observation.timestamp)>1e-9) {
      result.reason="observations are not from the same exposure"; return result;
    }
    commonTimestamp=observation.timestamp;
    double unused=0.0;
    if(!calibration.TimeOffset(observation.pixel,unused,result.reason)) return result;
    if((observation.hasPrevious &&
        (!Finite(observation.previousPixel) ||
         !std::isfinite(observation.previousTimestamp) ||
         observation.previousTimestamp>=observation.timestamp)) ||
       (observation.hasNext &&
        (!Finite(observation.nextPixel) ||
         !std::isfinite(observation.nextTimestamp) ||
         observation.nextTimestamp<=observation.timestamp))) {
      result.reason="invalid adjacent exposure timing"; return result;
    }
    TooN::Vector<2> velocity;
    if(observation.hasPrevious && observation.hasNext)
      velocity=(observation.nextPixel-observation.previousPixel)/
               (observation.nextTimestamp-observation.previousTimestamp);
    else if(observation.hasPrevious)
      velocity=(observation.pixel-observation.previousPixel)/
               (observation.timestamp-observation.previousTimestamp);
    else
      velocity=(observation.nextPixel-observation.pixel)/
               (observation.nextTimestamp-observation.timestamp);
    if(!Finite(velocity)) { result.reason="nonfinite visual velocity"; return result; }
    const auto& J=observation.poseJacobian;
    information+=J.T()*J; rhs+=J.T()*velocity;
    velocities.push_back(velocity); jacobians.push_back(J);
  }
  result.observations=velocities.size();
  if(result.observations<6) {
    result.reason="insufficient adjacent-frame observations"; return result;
  }
  if(!Finite(information) || !Finite(rhs)) {
    result.reason="nonfinite visual-velocity system"; return result;
  }
  TooN::SymEigen<6> eigen(information);
  const auto values=eigen.get_evalues();
  if(!Finite(values) || values[5]<=0.0 || values[0]<=0.0) {
    result.reason="rank deficient visual-velocity system"; return result;
  }
  result.conditionRatio=values[0]/values[5];
  if(result.conditionRatio<1e-8) {
    result.reason="ill-conditioned visual-velocity system"; return result;
  }
  result.cameraVelocity=eigen.backsub(rhs);
  if(!Finite(result.cameraVelocity)) {
    result.reason="nonfinite estimated camera velocity"; return result;
  }
  double squared=0.0;
  for(std::size_t i=0;i<velocities.size();++i) {
    const TooN::Vector<2> residual=jacobians[i]*result.cameraVelocity-velocities[i];
    squared+=residual*residual;
  }
  result.residualRmsPixelsPerSecond=std::sqrt(squared/(2*velocities.size()));
  if(!std::isfinite(result.residualRmsPixelsPerSecond)) {
    result.reason="nonfinite visual-velocity residual"; return result;
  }
  result.valid=true; result.reason="visual velocity estimated"; return result;
}

bool RollingShutter::Correct(const TooN::Vector<2>& pixel,
    const TooN::Matrix<2,6>& jacobian,const RollingShutterEstimate& estimate,
    const RollingShutterCalibration& calibration,
    TooN::Vector<2>& corrected,std::string& reason) {
  corrected=pixel;
  if(!estimate.valid || !Finite(estimate.cameraVelocity) || !Finite(jacobian)) {
    reason="valid visual velocity and projection Jacobian are required"; return false;
  }
  double dt=0.0;
  if(!calibration.TimeOffset(pixel,dt,reason)) return false;
  const TooN::Vector<2> value=pixel-jacobian*estimate.cameraVelocity*dt;
  if(!Finite(value)) { reason="nonfinite corrected measurement"; return false; }
  // A corrected global-shutter coordinate may lie outside the recorded image;
  // it remains a valid observation for geometric BA, not a new pixel lookup.
  corrected=value; reason.clear(); return true;
}
}
