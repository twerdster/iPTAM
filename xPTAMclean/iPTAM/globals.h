#ifndef GLOBAL_VARS
#define GLOBAL_VARS
#include "image.h"
#include "TooN.h"

#define NUMTRACKERCAMPARAMETERS 5
const TooN::Vector<NUMTRACKERCAMPARAMETERS> CameraParameters = TooN::makeVector(0.942415 ,1.27385 ,0.520778 ,0.597289 ,0.709802);

const double CameraCalibratorBlurSigma = 10;
const double CameraCalibratorMeanGate = 10;
const int CameraCalibratorMinCornersForGrabbedImage = 20;
const double MapMakerMaxKFDistWiggleMult =10;
const int MapMakerPlaneAlignerRansacs = 100;
const double Reloc2MaxScore = 9e6;
const int TrackerDrawFASTCorners =0;
const int MaxInitialTrails = 100;
const int BundleMaxIterations = 20;
const double BundleUpdateSquaredConvergenceLimit = 1e-06;
const int BundleCout = 0;
//const CVD::ImageRef ARDriverFrameBufferSize = CVD::ImageRef(320,240);
const double MapMakerWiggleScale = 0.1;
const std::string BundleMEstimator = "Tukey";
const double BundleMinTukeySigma = 0.4;

//const double CameraCalibratorCornerPatchPixelSize = 20;
//const double CameraCalibratorExpandByStepMaxDistFrac = 03;

//double DrawMap = 0;
//double DrawAR = 0;

const double TrackerRotationEstimatorBlur = 0.75;
const int TrackerUseRotationEstimator = 1;
const int TrackerMiniPatchMaxSSD = 100000;
const int TrackerCoarseMin = 20;
const int TrackerCoarseMax = 60;
const int TrackerCoarseRange = 30;
const int TrackerCoarseSubPixIts = 8;

const int TrackerDisableCoarse = 0;
const double TrackerCoarseMinVelocity = 0.006;
const int TrackerMaxPatchesPerFrame = 100;
const std::string TrackerMEstimator = "Tukey";
const double TrackerTrackingQualityGood = 0.3;
const double TrackerTrackingQualityLost = 0.13;
const double MapMakerCandidateMinShiTomasiScore = 70;
const int nMaxSSDPerPixel  = 500;

#endif






