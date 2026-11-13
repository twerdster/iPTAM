#include "MobileInitializer.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>

namespace {
using namespace TooN;
using namespace IPTAMMobile;
using Pair=std::pair<Vector<2>,Vector<2>>;
void Check(bool condition,const char* message) {if(!condition) throw std::runtime_error(message);}
ATANCamera Camera() {
  ATANCamera camera("mobile-initializer-test");camera.SetImageSize(CVD::ImageRef(320,240));
  camera.SetParameters(makeVector(250./320,250./240,.5,.5,0.));return camera;
}
KeyFrame FirstFrame() {
  CVD::Image<CVD::byte> image(CVD::ImageRef(320,240));
  std::mt19937 generator(17);
  for(int y=0;y<240;++y) for(int x=0;x<320;++x)
    image[CVD::ImageRef(x,y)]=CVD::byte(40+generator()%176);
  KeyFrame frame;frame.MakeKeyFrame_Lite(image);return frame;
}
std::vector<Pair> PlanarMatches(ATANCamera camera,const SE3<>& pose,double noise=0,bool collinear=false) {
  std::vector<Pair> result;
  Vector<3> normal=makeVector(.12,.08,1.);normal/=norm(normal);
  for(int y=0;y<8;++y) for(int x=0;x<10;++x) {
    const Vector<2> first=makeVector(55.+x*22.,collinear?120.:45.+y*21.);
    const Vector<3> ray=unproject(camera.UnProject(first));
    const Vector<3> point=ray*3./(normal*ray);
    const Vector<3> moved=pose*point;
    Vector<2> second=camera.Project(makeVector(moved[0]/moved[2],moved[1]/moved[2]));
    second+=makeVector(noise*std::sin(x*7.+y*3.),noise*std::cos(x*2.-y*5.));
    result.push_back({first,second});
  }
  return result;
}
void PureRotationAndDegenerateDistribution() {
  auto camera=Camera();auto first=FirstFrame();
  MobileInitializer initializer;
  Check(initializer.Begin(first,camera,-.0015),"first view rejected");
  for(int i=1;i<=20;++i) {
    SE3<> pose;pose.get_rotation()=SO3<>::exp(makeVector(.0007*i,.001*i,-.0003*i));
    const auto result=initializer.ProcessCorrespondences(PlanarMatches(camera,pose),camera,.03*i);
    Check(!result.ready,"pure rotation created a stereo map");
    Check(result.hypotheses.empty(),"pure rotation entered metric pose hypothesis stage");
  }
  initializer.Reset();Check(initializer.Begin(first,camera,0.),"reset first view rejected");
  for(int i=1;i<=15;++i) {
    SE3<> pose;pose.get_translation()=makeVector(.012*i,0.,0.);
    const auto result=initializer.ProcessCorrespondences(PlanarMatches(camera,pose,0,true),camera,.03*i);
    Check(!result.ready,"collinear correspondences constrained an 8DOF decomposition");
  }
}
void PlanarAmbiguityAndNoise() {
  auto camera=Camera();auto first=FirstFrame();
  MobileInitializer initializer;
  Check(initializer.Begin(first,camera,0.),"first view rejected");
  bool hypothesesObserved=false,competingObserved=false,ready=false;
  for(int i=1;i<=80;++i) {
    SE3<> pose;pose.get_rotation()=SO3<>::exp(makeVector(.0004*i,.001*i,-.0002*i));
    pose.get_translation()=makeVector(.0015*i,.0008*i,.009*i);
    auto matches=PlanarMatches(camera,pose,.04);
    // A few out-of-plane/bad correspondences must be robustly suppressed.
    for(std::size_t j=0;j<4;++j) matches[j].second+=makeVector(20.,-15.);
    auto result=initializer.ProcessCorrespondences(matches,camera,.03*i);
    if(!result.hypotheses.empty()) hypothesesObserved=true;
    if(result.hypotheses.size()==2 && result.stage=="competing_poses") {
      competingObserved=true;
      Check(!result.ready,"ambiguous maps selected without subsequent evidence");
    }
    if(result.ready) {
      Check(result.chosenHypothesis>=0 && result.matches.size()>=20,
            "selected hypothesis lost correspondences");
      Check(result.conditionRatio>=1e-6 && result.medianParallaxRadians>=.005,
            "unconditioned or rotation-only initialization accepted");
      Check(std::isfinite(norm(result.secondFromFirst.ln())),"selected pose is nonfinite");
      Check(norm((result.secondFromFirst.get_rotation()*pose.get_rotation().inverse()).ln())<.015,
            "early ready hypothesis has incorrect scene rotation");
      const auto a=result.secondFromFirst.get_translation(),b=pose.get_translation();
      Check((a*b)/(norm(a)*norm(b))>.99,"early ready hypothesis has incorrect translation direction");
      ready=true;break;
    }
  }
  // A truly planar scene may keep the two-fold ambiguity. It is a valid outcome
  // to defer initialization rather than choosing an arbitrary pose by index.
  Check(hypothesesObserved,"translated planar scene never produced a conditioned hypothesis");
  Check(competingObserved,"two physical planar hypotheses were prematurely discarded");
  // Change the translation direction while maintaining image overlap; this is
  // third-view image evidence, not a pose hint to the initializer.
  for(int j=1;j<=60 && !ready;++j) {
    SE3<> pose;pose.get_rotation()=SO3<>::exp(makeVector(.032+.0004*j,.08+.001*j,-.016));
    pose.get_translation()=makeVector(.12+.012*j,.064+.003*j,.72-.001*j);
    const auto result=initializer.ProcessCorrespondences(PlanarMatches(camera,pose,.04),camera,2.4+.03*j);
    if(result.ready) {
      const double angle=norm((result.secondFromFirst.get_rotation()*pose.get_rotation().inverse()).ln());
      const auto a=result.secondFromFirst.get_translation(),b=pose.get_translation();
      Check(angle<.015,"ambiguity resolved to incorrect scene rotation");
      Check((a*b)/(norm(a)*norm(b))>.99,"ambiguity resolved to incorrect translation direction");
      ready=true;
    }
  }
  Check(ready,"subsequent images did not resolve conditioned planar hypotheses");
  const auto rejected=initializer.ProcessCorrespondences({},camera,-1.);
  Check(!rejected.ready && rejected.stage=="rejected","backwards initializer clock accepted");
}
void WarpedImageFrontend() {
  auto camera=Camera();auto first=FirstFrame();MobileInitializer initializer;
  Check(initializer.Begin(first,camera,0.),"image frontend first view rejected");
  int maximumSupport=0;
  for(int i=1;i<=8;++i) {
    CVD::Image<CVD::byte> image(CVD::ImageRef(320,240));
    for(int y=0;y<240;++y) for(int x=0;x<320;++x) {
      const int sourceX=x-i;
      image[CVD::ImageRef(x,y)]=sourceX>=0?first.aLevels[0].im[CVD::ImageRef(sourceX,y)]:0;
    }
    KeyFrame current;current.MakeKeyFrame_Lite(image);
    const auto result=initializer.Process(current,camera,.03*i);
    maximumSupport=std::max(maximumSupport,result.inliers);
    if(result.inliers>=20) {
      const Vector<3> ray=unproject(camera.UnProject(makeVector(160.,120.)));
      const Vector<3> moved=result.homography*ray;
      const auto pixel=camera.Project(makeVector(moved[0]/moved[2],moved[1]/moved[2]));
      if(norm(pixel-makeVector(160.+i,120.))>=.4)
        std::cerr<<"rendered shift="<<i<<" fitted="<<pixel<<" inliers="<<result.inliers<<" tracked="<<result.tracked<<'\n';
      Check(norm(pixel-makeVector(160.+i,120.))<.4,"warped patch SL3 update does not match rendered translation");
    }
  }
  Check(maximumSupport>=40,"warped patch image frontend failed textured planar motion");
}
void InvalidInputs() {
  auto camera=Camera();auto first=FirstFrame();MobileInitializer initializer;
  Check(!initializer.Begin(first,camera,std::numeric_limits<double>::quiet_NaN()),
        "NaN first timestamp accepted");
  Check(initializer.Begin(first,camera,0.),"first view rejected");
  SE3<> pose;pose.get_translation()=makeVector(.05,0.,0.);
  auto matches=PlanarMatches(camera,pose);
  for(auto& pair:matches) pair.first[0]=std::numeric_limits<double>::quiet_NaN();
  const auto result=initializer.ProcessCorrespondences(matches,camera,.03);
  Check(!result.ready && result.tracked==0,"nonfinite correspondences created support");
  initializer.Reset();
  Check(!initializer.ProcessCorrespondences({},camera,1.).ready,"reset preserved ready state");
}
}
int main() {
  try {PureRotationAndDegenerateDistribution();PlanarAmbiguityAndNoise();WarpedImageFrontend();InvalidInputs();}
  catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
  std::cout<<"mobile initializer regressions passed\n";return 0;
}
