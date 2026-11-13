#include "ATANCamera.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using TooN::makeVector;
const double kFx = 253.30344967;
const double kFy = 253.34336776;
const double kCx = 319.5;
const double kCy = 239.5;
const double kFront = std::acos(-1.0)/2.0 - 1e-6;

void Check(bool condition, const char* message)
{
  if(!condition) throw std::runtime_error(message);
}

bool Finite(const TooN::Vector<2>& value)
{
  return std::isfinite(value[0]) && std::isfinite(value[1]);
}

void Near(const TooN::Vector<2>& actual, const TooN::Vector<2>& expected,
          double tolerance, const char* message)
{
  Check(Finite(actual) && TooN::norm(actual-expected) <= tolerance, message);
}

void Configure(ATANCamera& camera)
{
  camera.SetImageSize(CVD::ImageRef(640,480));
  camera.SetParameters(makeVector(kFx/640.0,kFy/480.0,0.5,0.5,0.0));
  camera.SetProjectionModel(ATANCamera::ProjectionModel::Equidistant);
}

void DefaultModelRegression()
{
  ATANCamera camera("historical-default");
  Check(camera.GetProjectionModel() == ATANCamera::ProjectionModel::FOV,
        "default projection model changed");
  // Independent historical FOV equations, including the w=0 pinhole case.
  for(double w : {0.0,0.7})
    {
      camera.SetParameters(makeVector(0.6,0.8,0.47,0.52,w));
      for(double y : {-0.5,0.0,0.4}) for(double x : {-0.6,0.0,0.7})
        {
          const auto ray = makeVector(x,y);
          const double r = std::hypot(x,y);
          const double scale = (w == 0.0 || r < 0.001) ? 1.0
                             : std::atan(r*2.0*std::tan(w/2.0))/(w*r);
          const auto expected = makeVector(640.0*0.47-0.5+640.0*0.6*scale*x,
                                            480.0*0.52-0.5+480.0*0.8*scale*y);
          Near(camera.Project(ray),expected,1e-12,"historical FOV projection changed");
          const auto jac = camera.GetProjectionDerivs();
          if(w == 0.0)
            {
              Check(jac[0][0] == 384.0 && jac[1][1] == 384.0 &&
                    jac[0][1] == 0.0 && jac[1][0] == 0.0,
                    "historical pinhole Jacobian changed");
            }
          Near(camera.UnProject(expected),ray,1e-12,"historical FOV inverse changed");
        }
    }
  // Nonzero FOV w must not be silently reinterpreted as an ideal lens.
  bool rejected=false;
  try { camera.SetProjectionModel(ATANCamera::ProjectionModel::Equidistant); }
  catch(const std::invalid_argument&) { rejected=true; }
  Check(rejected && camera.GetProjectionModel() == ATANCamera::ProjectionModel::FOV,
        "model change silently ignored a nonzero distortion parameter");
}

void RoundTripAndDomain()
{
  ATANCamera camera("native-lounge");
  Configure(camera);
  int invalidCount=0;
  double maxError=0.0;
  for(int y=0; y<480; ++y) for(int x=0; x<640; ++x)
    {
      const auto pixel=makeVector(double(x),double(y));
      const auto ray=camera.UnProject(pixel);
      const double theta=std::hypot((x-kCx)/kFx,(y-kCy)/kFy);
      Check(camera.Invalid() == (theta >= kFront),"inverse front-domain mismatch");
      if(camera.Invalid())
        {
          ++invalidCount;
          Check(!std::isfinite(ray[0]) && !std::isfinite(ray[1]),
                "invalid image ray folded into finite front plane");
          continue;
        }
      const auto recovered=camera.Project(ray);
      Check(!camera.Invalid(),"inverse returned ray rejected by forward projection");
      maxError=std::max(maxError,TooN::norm(recovered-pixel));
    }
  Check(invalidCount == 16,"native lens invalid corner count changed");
  Check(maxError < 1e-9,"equidistant full-image pixel round trip inaccurate");

  for(double theta : {kFront-1e-7,kFront+1e-7,2.0})
    {
      const auto ray=camera.UnProject(makeVector(kCx+kFx*theta,kCy));
      Check(camera.Invalid() == (theta >= kFront),"horizon inverse acceptance mismatch");
      if(theta < kFront) Check(Finite(ray) && ray[0] > 0.0,"valid near-horizon ray rejected");
      else Check(!Finite(ray),"rear-hemisphere ray folded through tangent");
    }
  const double nan=std::numeric_limits<double>::quiet_NaN();
  const double inf=std::numeric_limits<double>::infinity();
  for(const auto& ray : {makeVector(nan,0.0),makeVector(0.0,inf),
                         makeVector(std::tan(kFront+1e-7),0.0)})
    {
      Check(!Finite(camera.Project(ray)) && camera.Invalid(),"invalid plane ray accepted");
      const auto jac=camera.GetProjectionDerivs();
      for(int row=0; row<2; ++row) for(int col=0; col<2; ++col)
        Check(!std::isfinite(jac[row][col]),"invalid projection retained stale Jacobian");
    }
  Check(!Finite(camera.UnProject(makeVector(nan,kCy))) && camera.Invalid(),
        "nonfinite image coordinate accepted");
  Near(camera.Project(makeVector(0.0,0.0)),makeVector(kCx,kCy),0.0,
       "valid projection did not recover after invalid input");
  Check(!camera.Invalid(),"valid projection retained invalid flag");
  std::cout << "native_pixels=307200 invalid_corners=" << invalidCount
            << " max_roundtrip_px=" << maxError << '\n';
}

void DerivativesAndCache()
{
  ATANCamera camera("analytical-Jacobian");
  Configure(camera);
  double maxError=0.0;
  for(double y : {-4.0,-0.001,-0.00005,0.0,0.00005,0.001,3.0})
    for(double x : {-4.0,-0.001,-0.00005,0.0,0.00005,0.001,3.0})
      {
        const auto ray=makeVector(x,y);
        const auto pixel=camera.Project(ray);
        const auto jac=camera.GetProjectionDerivs();
        camera.UnProject(pixel);
        const auto inverseCachedJac=camera.GetProjectionDerivs();
        for(int row=0; row<2; ++row) for(int col=0; col<2; ++col)
          Check(std::abs(jac[row][col]-inverseCachedJac[row][col]) < 1e-10,
                "UnProject did not establish the projection derivative cache");
        for(int axis=0; axis<2; ++axis)
          {
            auto plus=ray,minus=ray;
            plus[axis]+=1e-6;
            minus[axis]-=1e-6;
            const auto derivative=(camera.Project(plus)-camera.Project(minus))/(2e-6);
            for(int row=0; row<2; ++row)
              maxError=std::max(maxError,std::abs(derivative[row]-jac[row][axis]));
          }
      }
  Check(maxError < 1e-5,"analytic equidistant Jacobian differs from finite differences");
  std::cout << "max_jacobian_error=" << maxError << '\n';
}

void DerivedBoundsCopiesAndScale()
{
  ATANCamera camera("finite-derived-state");
  Configure(camera);
  Check(std::isfinite(camera.LargestRadiusInImage()) && camera.LargestRadiusInImage()>0.0,
        "invalid corners poisoned largest radius");
  Check(std::isfinite(camera.OnePixelDist()) && camera.OnePixelDist()>0.0,
        "invalid corners poisoned one-pixel scale");
  Check(std::isfinite(camera.PixelAspectRatio()),"nonfinite pixel aspect ratio");
  const auto tl=camera.ImplaneTL(),br=camera.ImplaneBR();
  Check(Finite(tl) && Finite(br) && tl[0]<br[0] && tl[1]<br[1],
        "invalid corners poisoned render bounds");
  Near(camera.UFBLinearProject(tl),makeVector(0.0,0.0),1e-15,"linear bbox TL mismatch");
  Near(camera.UFBLinearProject(br),makeVector(1.0,1.0),1e-15,"linear bbox BR mismatch");
  Near(camera.UFBLinearUnProject(makeVector(0.0,0.0)),tl,1e-9,"linear inverse TL mismatch");
  const auto frustum=camera.MakeUFBLinearFrustumMatrix(0.01,100.0);
  for(int row=0; row<4; ++row) for(int col=0; col<4; ++col)
    Check(std::isfinite(frustum[row][col]),"invalid corners poisoned render frustum");

  const auto ray=makeVector(0.8,-0.4);
  const auto pixel=camera.Project(ray);
  const auto ufb=camera.UFBProject(ray);
  Near(ufb,makeVector((pixel[0]+0.5)/640.0,(pixel[1]+0.5)/480.0),1e-15,
       "equidistant UFB pixel convention mismatch");
  Near(camera.UFBUnProject(ufb),ray,1e-14,"equidistant UFB inverse mismatch");
  ATANCamera copy=camera;
  Check(copy.GetProjectionModel() == ATANCamera::ProjectionModel::Equidistant,
        "camera copy lost projection model");
  Near(copy.Project(ray),pixel,0.0,"copied camera lost calibration");
  copy.SetImageSize(CVD::ImageRef(320,240));
  Near(copy.Project(ray),makeVector((pixel[0]+0.5)/2.0-0.5,(pixel[1]+0.5)/2.0-0.5),
       1e-13,"model/normalized calibration lost on resize");
  Check(copy.GetProjectionModel() == ATANCamera::ProjectionModel::Equidistant,
        "resize lost projection model");

  camera.SetProjectionModel(ATANCamera::ProjectionModel::FOV);
  Near(camera.Project(ray),makeVector(kCx+kFx*ray[0],kCy+kFy*ray[1]),1e-13,
       "switching back to pinhole changed calibration");
  camera.SetProjectionModel(ATANCamera::ProjectionModel::Equidistant);
  Near(camera.Project(ray),pixel,0.0,"switching back to equidistant changed calibration");
}
}

int main()
{
  try
    {
      DefaultModelRegression();
      RoundTripAndDomain();
      DerivativesAndCache();
      DerivedBoundsCopiesAndScale();
      std::cout << "equidistant camera regressions passed\n";
      return 0;
    }
  catch(const std::exception& error)
    {
      std::cerr << "equidistant camera regression: " << error.what() << '\n';
      return 1;
    }
}
