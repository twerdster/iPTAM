#include "RollingShutter.h"
#include "se3.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using namespace TooN;
using namespace IPTAMMobile;
void Check(bool condition,const char* message) {if(!condition) throw std::runtime_error(message);}
Matrix<2,6> ProjectionJacobian(const Vector<3>& p) {
  Matrix<2,3> division=Zeros;
  division[0][0]=division[1][1]=400/p[2];
  division[0][2]=-400*p[0]/(p[2]*p[2]);
  division[1][2]=-400*p[1]/(p[2]*p[2]);
  Matrix<3,6> motion=Zeros;
  for(int d=0;d<3;++d) motion[d][d]=1;
  for(int d=0;d<3;++d) {Vector<3> e=Zeros;e[d]=1;motion.T()[d+3]=e^p;}
  return division*motion;
}
void KnownWarp(int axis,int direction) {
  RollingShutterCalibration calibration;
  calibration.known=true;calibration.width=640;calibration.height=480;
  calibration.axis=axis;calibration.direction=direction;
  calibration.readoutSeconds=.018;calibration.timestampFraction=.4;
  const Vector<6> velocity=makeVector(.12,-.03,.07,.02,-.08,.03);
  std::vector<RollingShutterObservation> observations;
  std::vector<Vector<2>> ideal;
  for(int y=0;y<6;++y) for(int x=0;x<8;++x) {
    const double z=2.0+.13*((x+2*y)%7);
    const Vector<3> point=makeVector((x-3.5)*.18,(y-2.5)*.16,z);
    const Vector<2> clean=makeVector(319.5+400*point[0]/z,239.5+400*point[1]/z);
    const Matrix<2,6> J=ProjectionJacobian(point);
    const Vector<2> flow=J*velocity;
    const double extent=double((axis==0?calibration.width:calibration.height)-1);
    const double signedSlope=direction*calibration.readoutSeconds/extent;
    const double start=direction>0?0.:1.;
    // Solve measured = ideal + flow * dt(measured) analytically, rather than
    // generating the test with the correction implementation under test.
    const double dt=calibration.readoutSeconds*(start+direction*clean[axis]/extent-
        calibration.timestampFraction)/(1.0-flow[axis]*signedSlope);
    RollingShutterObservation observation;
    observation.pixel=clean+flow*dt;observation.poseJacobian=J;
    observation.timestamp=-.0015;
    observation.hasPrevious=observation.hasNext=true;
    observation.previousTimestamp=observation.timestamp-.031;
    observation.nextTimestamp=observation.timestamp+.029;
    observation.previousPixel=observation.pixel-flow*.031;
    observation.nextPixel=observation.pixel+flow*.029;
    observations.push_back(observation);ideal.push_back(clean);
  }
  const auto estimate=RollingShutter::EstimateVelocity(observations,calibration);
  Check(estimate.valid,"known visual velocity was rejected");
  Check(norm(estimate.cameraVelocity-velocity)<1e-10,"6DOF visual velocity is inaccurate");
  Check(estimate.observations==48 && estimate.residualRmsPixelsPerSecond<1e-9,
        "known visual velocity residual/count wrong");
  for(std::size_t i=0;i<observations.size();++i) {
    Vector<2> corrected;std::string reason;
    Check(RollingShutter::Correct(observations[i].pixel,observations[i].poseJacobian,
          estimate,calibration,corrected,reason),"valid RS correction rejected");
    Check(norm(corrected-ideal[i])<1e-9,"RS correction failed known warped measurement");
  }
  // One-sided adjacent observations remain a valid constant-velocity estimate.
  for(auto& o:observations) o.hasNext=false;
  Check(RollingShutter::EstimateVelocity(observations,calibration).valid,
        "valid preceding-frame velocities rejected");
  observations[0].previousTimestamp=observations[0].timestamp;
  Check(!RollingShutter::EstimateVelocity(observations,calibration).valid,
        "duplicate exposure timing accepted");
}
void InvalidAndUnobservable() {
  RollingShutterCalibration calibration;std::string reason;double dt=9;
  Check(!calibration.Validate(reason),"unknown timing accepted");
  Check(!calibration.TimeOffset(makeVector(20.,20.),dt,reason) && dt==0,
        "unknown timing returned correction offset");
  calibration.known=true;calibration.width=640;calibration.height=480;
  calibration.readoutSeconds=.02;
  Check(calibration.Validate(reason),"valid timing rejected");
  calibration.direction=0;Check(!calibration.Validate(reason),"invalid direction accepted");
  calibration.direction=1;calibration.timestampFraction=std::numeric_limits<double>::quiet_NaN();
  Check(!calibration.Validate(reason),"NaN timestamp convention accepted");
  calibration.timestampFraction=.5;
  std::vector<RollingShutterObservation> observations(20);
  for(auto& o:observations) {
    o.pixel=makeVector(320.,240.);o.timestamp=1.;o.hasPrevious=true;
    o.previousPixel=o.pixel;o.previousTimestamp=.9;
    o.poseJacobian=Zeros;o.poseJacobian[0][0]=o.poseJacobian[1][1]=1;
  }
  Check(!RollingShutter::EstimateVelocity(observations,calibration).valid,
        "rank-deficient visual velocity accepted");
  RollingShutterEstimate invalid;
  Vector<2> corrected;
  Check(!RollingShutter::Correct(observations[0].pixel,observations[0].poseJacobian,
          invalid,calibration,corrected,reason),"unknown velocity corrected a measurement");
  Check(norm(corrected-observations[0].pixel)==0,"failed correction mutated raw measurement");
}
}
int main() {
  try {for(int axis:{0,1}) for(int direction:{-1,1}) KnownWarp(axis,direction);InvalidAndUnobservable();}
  catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
  std::cout<<"mobile rolling shutter regressions passed\n";return 0;
}
