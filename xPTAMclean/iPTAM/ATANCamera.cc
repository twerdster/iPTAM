// Copyright 2008 Isis Innovation Limited
#include "ATANCamera.h"
#include "helpers.h"
#include "vector_image_ref.h"
#include <iostream>
#include <stdexcept>
#include <limits>

using namespace std;
using namespace CVD;

namespace {
// A finite z=1 plane cannot represent the horizon. Keep a one-microradian
// front-facing margin and reject its boundary instead of folding tan(theta).
const double kEquidistantFrontLimit = std::acos(-1.0) / 2.0 - 1e-6;
const double kNaN = std::numeric_limits<double>::quiet_NaN();
}

ATANCamera::ATANCamera(string sName) : mProjectionModel(ProjectionModel::FOV)
{
  msName = sName;
//	mgvvCameraParams=CameraParameters;
//TooN::makeVector(0.92295, 1.28292, 0.497764, 0.490052, 0); -> best vector or calib s far
	mgvvCameraParams= TooN::makeVector(0.92295, 1.28292, 0.497764, 0.490052, 0);
  mvImageSize[0] = 640; 
  mvImageSize[1] = 480;
  RefreshParams();
}

void ATANCamera::SetProjectionModel(ProjectionModel model)
{
  if(model != ProjectionModel::FOV && model != ProjectionModel::Equidistant)
    throw std::invalid_argument("Unknown camera projection model");
  if(model == ProjectionModel::Equidistant && mgvvCameraParams[4] != 0.0)
    throw std::invalid_argument("Ideal equidistant camera requires zero FOV distortion");
  mProjectionModel = model;
  RefreshParams();
}

void ATANCamera::SetParameters(const Vector<5>& p) {
  for(int i=0;i<5;++i) if(!std::isfinite(p[i])) throw std::invalid_argument("Camera parameters must be finite");
  if(p[0] <= 0 || p[1] <= 0 || std::abs(p[4]) >= 3.0)
    throw std::invalid_argument("Invalid focal length or FOV distortion");
  if(mProjectionModel == ProjectionModel::Equidistant && p[4] != 0.0)
    throw std::invalid_argument("Ideal equidistant camera requires zero FOV distortion");
  mgvvCameraParams = p;
  RefreshParams();
}

void ATANCamera::SetImageSize(Vector<2> vImageSize)
{
  if(!std::isfinite(vImageSize[0]) || !std::isfinite(vImageSize[1]) || vImageSize[0] <= 0 || vImageSize[1] <= 0)
    throw std::invalid_argument("Camera image must be positive in each dimension");
  mvImageSize = vImageSize;
  RefreshParams();
};

void ATANCamera::RefreshParams() 
{
  if(mProjectionModel == ProjectionModel::Equidistant)
    {
      for(int axis=0; axis<2; ++axis)
        {
          mvFocal[axis] = mvImageSize[axis] * mgvvCameraParams[axis];
          mvCenter[axis] = mvImageSize[axis] * mgvvCameraParams[axis+2] - 0.5;
          mvInvFocal[axis] = 1.0 / mvFocal[axis];
        }
      mdW = mdWinv = md2Tan = mdOneOver2Tan = 0.0;
      mdDistortionEnabled = 1.0;
      double cornerTheta = 0.0;
      for(int y=0; y<2; ++y)
        for(int x=0; x<2; ++x)
          {
            const double a = ((x ? mvImageSize[0]-0.5 : -0.5) - mvCenter[0]) * mvInvFocal[0];
            const double b = ((y ? mvImageSize[1]-0.5 : -0.5) - mvCenter[1]) * mvInvFocal[1];
            cornerTheta = std::max(cornerTheta, std::hypot(a,b));
          }
      mdLargestRadius = std::tan(std::min(cornerTheta, kEquidistantFrontLimit));
      mdMaxR = std::tan(kEquidistantFrontLimit);
      mdOnePixelDist = std::hypot(mvInvFocal[0], mvInvFocal[1]) / std::sqrt(2.0);

      // Conservative finite linear-render bounds enclose every valid front
      // ray. Image corners beyond the horizon do not get unprojected. This
      // clipping affects the render frustum only; UnProject rejects those
      // measurements and leaves their original image intensities untouched.
      mvImplaneTL = makeVector(-mdLargestRadius, -mdLargestRadius);
      mvImplaneBR = makeVector(mdLargestRadius, mdLargestRadius);
      mvUFBLinearInvFocal = mvImplaneBR - mvImplaneTL;
      for(int axis=0; axis<2; ++axis)
        {
          mvUFBLinearFocal[axis] = 1.0 / mvUFBLinearInvFocal[axis];
          mvUFBLinearCenter[axis] = 0.5;
        }
      Project(makeVector(0.0,0.0));
      return;
    }

  // This updates internal member variables according to the current camera parameters,
  // and the currently selected target image size.
  //
  
  // First: Focal length and image center in pixel coordinates
  mvFocal[0] = mvImageSize[0] * mgvvCameraParams[0];
  mvFocal[1] = mvImageSize[1] * (mgvvCameraParams)[1];
  mvCenter[0] = mvImageSize[0] * (mgvvCameraParams)[2] - 0.5;
  mvCenter[1] = mvImageSize[1] * (mgvvCameraParams)[3] - 0.5;
  
  // One over focal length
  mvInvFocal[0] = 1.0 / mvFocal[0];
  mvInvFocal[1] = 1.0 / mvFocal[1];

  // Some radial distortion parameters..
  mdW =  (mgvvCameraParams)[4];
  if(mdW != 0.0)
    {
      md2Tan = 2.0 * tan(mdW / 2.0);
      mdOneOver2Tan = 1.0 / md2Tan;
      mdWinv = 1.0 / mdW;
      mdDistortionEnabled = 1.0;
    }
  else
    {
      mdWinv = 0.0;
      md2Tan = 0.0;
      mdDistortionEnabled = 0.0;
    }
  
  // work out biggest radius in image
  Vector<2> v2;
  v2[0]= max((mgvvCameraParams)[2], 1.0 - (mgvvCameraParams)[2]) / (mgvvCameraParams)[0];
  v2[1]= max((mgvvCameraParams)[3], 1.0 - (mgvvCameraParams)[3]) / (mgvvCameraParams)[1];
  mdLargestRadius = invrtrans(sqrt(v2*v2));
  
  // At what stage does the model become invalid?
  mdMaxR = 1.5 * mdLargestRadius; // (pretty arbitrary)

  // work out world radius of one pixel
  // (This only really makes sense for square-ish pixels)
  {
    Vector<2> v2Center = UnProject(mvImageSize / 2);
    Vector<2> v2RootTwoAway = UnProject(mvImageSize / 2 + vec(ImageRef(1,1)));
    Vector<2> v2Diff = v2Center - v2RootTwoAway;
    mdOnePixelDist = sqrt(v2Diff * v2Diff) / sqrt(2.0);
  }
  
  // Work out the linear projection values for the UFB
  {
    // First: Find out how big the linear bounding rectangle must be
    vector<Vector<2> > vv2Verts;
    vv2Verts.push_back(UnProject(makeVector( -0.5, -0.5)));
    vv2Verts.push_back(UnProject(makeVector( mvImageSize[0]-0.5, -0.5)));
    vv2Verts.push_back(UnProject(makeVector( mvImageSize[0]-0.5, mvImageSize[1]-0.5)));
    vv2Verts.push_back(UnProject(makeVector( -0.5, mvImageSize[1]-0.5)));
    Vector<2> v2Min = vv2Verts[0];
    Vector<2> v2Max = vv2Verts[0];
    for(int i=0; i<4; i++)
      for(int j=0; j<2; j++)
	{
	  if(vv2Verts[i][j] < v2Min[j]) v2Min[j] = vv2Verts[i][j];
	  if(vv2Verts[i][j] > v2Max[j]) v2Max[j] = vv2Verts[i][j];
	}
    mvImplaneTL = v2Min;
    mvImplaneBR = v2Max;
    
    // Store projection parameters to fill this bounding box
    Vector<2> v2Range = v2Max - v2Min;
    mvUFBLinearInvFocal = v2Range;
    mvUFBLinearFocal[0] = 1.0 / mvUFBLinearInvFocal[0];
    mvUFBLinearFocal[1] = 1.0 / mvUFBLinearInvFocal[1];
    mvUFBLinearCenter[0] = -1.0 * v2Min[0] * mvUFBLinearFocal[0];
    mvUFBLinearCenter[1] = -1.0 * v2Min[1] * mvUFBLinearFocal[1];
  }
  
}

// Project from the camera z=1 plane to image pixels,
// while storing intermediate calculation results in member variables
Vector<2> ATANCamera::Project(const Vector<2>& vCam){
  if(mProjectionModel == ProjectionModel::Equidistant)
    {
      mvLastCam = vCam;
      mdLastR = std::hypot(vCam[0],vCam[1]);
      mbInvalid = !std::isfinite(mdLastR) || mdLastR >= mdMaxR;
      if(mbInvalid)
        {
          mdLastFactor = mdLastDistR = kNaN;
          mvLastDistCam = mvLastIm = makeVector(kNaN,kNaN);
          return mvLastIm;
        }
      const double r2 = mdLastR * mdLastR;
      mdLastFactor = r2 < 1e-8 ? 1.0-r2/3.0+r2*r2/5.0 : std::atan(mdLastR)/mdLastR;
      mdLastDistR = std::atan(mdLastR);
      mvLastDistCam = mdLastFactor * vCam;
      mvLastIm = makeVector(mvCenter[0]+mvFocal[0]*mvLastDistCam[0],
                            mvCenter[1]+mvFocal[1]*mvLastDistCam[1]);
      return mvLastIm;
    }

  mvLastCam = vCam;
  mdLastR = sqrt(vCam * vCam);
  mbInvalid = (mdLastR > mdMaxR);
  mdLastFactor = rtrans_factor(mdLastR);
  mdLastDistR = mdLastFactor * mdLastR;
  mvLastDistCam = mdLastFactor * mvLastCam;
  
  mvLastIm[0] = mvCenter[0] + mvFocal[0] * mvLastDistCam[0];
  mvLastIm[1] = mvCenter[1] + mvFocal[1] * mvLastDistCam[1];
  
  return mvLastIm;
}

// Un-project from image pixel coords to the camera z=1 plane
// while storing intermediate calculation results in member variables
Vector<2> ATANCamera::UnProject(const Vector<2>& v2Im)
{
  if(mProjectionModel == ProjectionModel::Equidistant)
    {
      mvLastIm = v2Im;
      mvLastDistCam = makeVector((v2Im[0]-mvCenter[0])*mvInvFocal[0],
                                 (v2Im[1]-mvCenter[1])*mvInvFocal[1]);
      mdLastDistR = std::hypot(mvLastDistCam[0],mvLastDistCam[1]);
      mbInvalid = !std::isfinite(mdLastDistR) || mdLastDistR >= kEquidistantFrontLimit;
      if(mbInvalid)
        {
          mdLastR = mdLastFactor = kNaN;
          mvLastCam = makeVector(kNaN,kNaN);
          return mvLastCam;
        }
      const double theta2 = mdLastDistR * mdLastDistR;
      const double scale = theta2 < 1e-8 ? 1.0+theta2/3.0+2.0*theta2*theta2/15.0
                                         : std::tan(mdLastDistR)/mdLastDistR;
      mvLastCam = scale * mvLastDistCam;
      mdLastR = std::tan(mdLastDistR);
      mdLastFactor = 1.0 / scale;
      return mvLastCam;
    }

  mvLastIm = v2Im;
  mvLastDistCam[0] = (mvLastIm[0] - mvCenter[0]) * mvInvFocal[0];
  mvLastDistCam[1] = (mvLastIm[1] - mvCenter[1]) * mvInvFocal[1];
  mdLastDistR = sqrt(mvLastDistCam * mvLastDistCam);
  mdLastR = invrtrans(mdLastDistR);
  double dFactor;
  if(mdLastDistR > 0.01)
    dFactor =  mdLastR / mdLastDistR;
  else
    dFactor = 1.0;
  mdLastFactor = 1.0 / dFactor;
  mvLastCam = dFactor * mvLastDistCam;
  return mvLastCam;
}

// Utility function for easy drawing with OpenGL
// C.f. comment in top of ATANCamera.h
Matrix<4> ATANCamera::MakeUFBLinearFrustumMatrix(double near, double far)
{
  Matrix<4> m4 = Zeros;
  

  double left = mvImplaneTL[0] * near;
  double right = mvImplaneBR[0] * near;
  double top = mvImplaneTL[1] * near;
  double bottom = mvImplaneBR[1] * near;
  
  // The openGhelL frustum manpage is A PACK OF LIES!!
  // Two of the elements are NOT what the manpage says they should be.
  // Anyway, below code makes a frustum projection matrix
  // Which projects a RHS-coord frame with +z in front of the camera
  // Which is what I usually want, instead of glFrustum's LHS, -z idea.
  m4[0][0] = (2 * near) / (right - left);
  m4[1][1] = (2 * near) / (top - bottom);
  
  m4[0][2] = (right + left) / (left - right);
  m4[1][2] = (top + bottom) / (bottom - top);
  m4[2][2] = (far + near) / (far - near);
  m4[3][2] = 1;
  
  m4[2][3] = 2*near*far / (near - far);

  return m4;
};

Matrix<2,2> ATANCamera::GetProjectionDerivs()
{
  if(mProjectionModel == ProjectionModel::Equidistant)
    {
      Matrix<2> jac;
      if(mbInvalid)
        {
          jac[0] = makeVector(kNaN,kNaN);
          jac[1] = makeVector(kNaN,kNaN);
          return jac;
        }
      const double x=mvLastCam[0], y=mvLastCam[1];
      const double r2=x*x+y*y, r=std::sqrt(r2);
      const double scale = r2 < 1e-8 ? 1.0-r2/3.0+r2*r2/5.0 : std::atan(r)/r;
      const double radial = r2 < 1e-8 ? -2.0/3.0+4.0*r2/5.0-6.0*r2*r2/7.0
                                      : (r/(1.0+r2)-std::atan(r))/(r2*r);
      jac[0] = makeVector(mvFocal[0]*(scale+radial*x*x), mvFocal[0]*radial*x*y);
      jac[1] = makeVector(mvFocal[1]*radial*x*y, mvFocal[1]*(scale+radial*y*y));
      return jac;
    }

  // get the derivative of image frame wrt camera z=1 frame at the last computed projection
  // in the form (d im1/d cam1, d im1/d cam2)
  //             (d im2/d cam1, d im2/d cam2)
  
  double dFracBydx;
  double dFracBydy;
  
  double &k = md2Tan;
  double &x = mvLastCam[0];
  double &y = mvLastCam[1];
  double r = mdLastR * mdDistortionEnabled;
  
  if(r < 0.01)
    {
      dFracBydx = 0.0;
      dFracBydy = 0.0;
    }
  else
    {
      dFracBydx = 
	mdWinv * (k * x) / (r*r*(1 + k*k*r*r)) - x * mdLastFactor / (r*r); 
      dFracBydy = 
	mdWinv * (k * y) / (r*r*(1 + k*k*r*r)) - y * mdLastFactor / (r*r); 
    }
  
  Matrix<2> m2Derivs;
  
  m2Derivs[0][0] = mvFocal[0] * (dFracBydx * x + mdLastFactor);  
  m2Derivs[1][0] = mvFocal[1] * (dFracBydx * y);  
  m2Derivs[0][1] = mvFocal[0] * (dFracBydy * x);  
  m2Derivs[1][1] = mvFocal[1] * (dFracBydy * y + mdLastFactor);  
  return m2Derivs;
}

Matrix<2,NUMTRACKERCAMPARAMETERS> ATANCamera::GetCameraParameterDerivs()
{
  // Differentials wrt to the camera parameters
  // Use these to calibrate the camera
  // No need for this to be quick, so do them numerically
  
  Matrix<2, NUMTRACKERCAMPARAMETERS> m2NNumDerivs;
  Vector<NUMTRACKERCAMPARAMETERS> vNNormal = mgvvCameraParams;
  Vector<2> v2Cam = mvLastCam;
  Vector<2> v2Out = Project(v2Cam);
  for(int i=0; i<NUMTRACKERCAMPARAMETERS; i++)
    {
      if(i == NUMTRACKERCAMPARAMETERS-1 && mdW == 0.0)
	continue;
      Vector<NUMTRACKERCAMPARAMETERS> vNUpdate;
      vNUpdate = Zeros;
      vNUpdate[i] += 0.001;
      UpdateParams(vNUpdate); 
      Vector<2> v2Out_B = Project(v2Cam);
      m2NNumDerivs.T()[i] = (v2Out_B - v2Out) / 0.001;
      mgvvCameraParams = vNNormal;
      RefreshParams();
    }
  if(mdW == 0.0)
    m2NNumDerivs.T()[NUMTRACKERCAMPARAMETERS-1] = Zeros;
  return m2NNumDerivs;
}

void ATANCamera::UpdateParams(Vector<5> vUpdate)
{
  // Update the camera parameters; use this as part of camera calibration.
  (mgvvCameraParams) = (mgvvCameraParams) + vUpdate;
  RefreshParams();
}

void ATANCamera::DisableRadialDistortion()
{
  // Set the radial distortion parameter to zero
  // This disables radial distortion and also disables its differentials
  mgvvCameraParams[NUMTRACKERCAMPARAMETERS-1] = 0.0;
  RefreshParams();
}

Vector<2> ATANCamera::UFBProject(const Vector<2>& vCam)
{
  if(mProjectionModel == ProjectionModel::Equidistant)
    {
      const Vector<2> pixel = Project(vCam);
      return makeVector((pixel[0]+0.5)/mvImageSize[0], (pixel[1]+0.5)/mvImageSize[1]);
    }

  // Project from camera z=1 plane to UFB, storing intermediate calc results.
  mvLastCam = vCam;
  mdLastR = sqrt(vCam * vCam);
  mbInvalid = (mdLastR > mdMaxR);
  mdLastFactor = rtrans_factor(mdLastR);
  mdLastDistR = mdLastFactor * mdLastR;
  mvLastDistCam = mdLastFactor * mvLastCam;
  
  mvLastIm[0] = (mgvvCameraParams)[2]  + (mgvvCameraParams)[0] * mvLastDistCam[0];
  mvLastIm[1] = (mgvvCameraParams)[3]  + (mgvvCameraParams)[1] * mvLastDistCam[1];
  return mvLastIm;
}

Vector<2> ATANCamera::UFBUnProject(const Vector<2>& v2Im)
{
  if(mProjectionModel == ProjectionModel::Equidistant)
    return UnProject(makeVector(v2Im[0]*mvImageSize[0]-0.5, v2Im[1]*mvImageSize[1]-0.5));

  mvLastIm = v2Im;
  mvLastDistCam[0] = (mvLastIm[0] - (mgvvCameraParams)[2]) / (mgvvCameraParams)[0];
  mvLastDistCam[1] = (mvLastIm[1] - (mgvvCameraParams)[3]) / (mgvvCameraParams)[1];
  mdLastDistR = sqrt(mvLastDistCam * mvLastDistCam);
  mdLastR = invrtrans(mdLastDistR);
  double dFactor;
  if(mdLastDistR > 0.01)
    dFactor =  mdLastR / mdLastDistR;
  else
    dFactor = 1.0;
  mdLastFactor = 1.0 / dFactor;
  mvLastCam = dFactor * mvLastDistCam;
  return mvLastCam;
}

const Vector<NUMTRACKERCAMPARAMETERS> ATANCamera::mvDefaultParams = makeVector(0.5, 0.75, 0.5, 0.5, 0.1);

