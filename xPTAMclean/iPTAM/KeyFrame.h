// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

//
// This header declares the data structures to do with keyframes:
// structs KeyFrame, Level, Measurement, Candidate.
// 
// A KeyFrame contains an image pyramid stored as array of Level;
// A KeyFrame also has associated map-point mesurements stored as a vector of Measurment;
// Each individual Level contains an image, corner points, and special corner points
// which are promoted to Candidate status (the mapmaker tries to make new map points from those.)
//
// KeyFrames are stored in the Map class and manipulated by the MapMaker.
// However, the tracker also stores its current frame as a half-populated
// KeyFrame struct.


#ifndef __KEYFRAME_H
#define __KEYFRAME_H
#include "TooN.h"
using namespace TooN;
#include "se3.h"
#include "image.h"
#include "byte.h"
#include <vector>
#include <set>
#include <map>
#include "globals.h"

class MapPoint;
class SmallBlurryImage;

#ifdef IPTAM_MOBILE_PAPER
#define LEVELS 5
#else
#define LEVELS 4
#endif

// Candidate: a feature in an image which could be made into a map point
struct Candidate
{
  CVD::ImageRef irLevelPos;
  Vector<2> v2RootPos;
  double dSTScore;
};

// Measurement: A 2D image measurement of a map point. Each keyframe stores a bunch of these.
struct Measurement
{
  int nLevel;   // Which image level?
  bool bSubPix; // Has this measurement been refined to sub-pixel level?
  Vector<2> v2RootPos;  // Position of the measurement, REFERED TO PYRAMID LEVEL ZERO
#ifdef IPTAM_MOBILE_PAPER
  Vector<2> v2RawRootPos = Zeros, v2BundleRootPos = Zeros;
  Vector<2> v2PreviousRootPos = Zeros, v2NextRootPos = Zeros;
  Matrix<2,6> m26PoseJacobian = Zeros;
  double timestamp = 0, previousTimestamp = 0, nextTimestamp = 0;
  bool hasRawPosition = false, hasPrevious = false, hasNext = false, rollingShutterCorrected = false;
  Vector<2> BundlePosition() const { return rollingShutterCorrected ? v2BundleRootPos : v2RootPos; }
#endif
  enum {SRC_TRACKER, SRC_REFIND, SRC_ROOT, SRC_TRAIL, SRC_EPIPOLAR} Source; // Where has this measurement come frome?
};

// Each keyframe is made of LEVELS pyramid levels, stored in struct Level.
// This contains image data and corner points.
struct Level
{
  inline Level()
  {
    bImplaneCornersCached = false;
  };
  
  CVD::Image<CVD::byte> im;                // The pyramid level pixels
  std::vector<CVD::ImageRef> vCorners;     // All FAST corners on this level
  std::vector<int> vCornerRowLUT;          // Row-index into the FAST corners, speeds up access
  std::vector<CVD::ImageRef> vMaxCorners;  // The maximal FAST corners
  Level& operator=(const Level &rhs);
  
  std::vector<Candidate> vCandidates;   // Potential locations of new map points
  
  bool bImplaneCornersCached;           // Also keep image-plane (z=1) positions of FAST corners to speed up epipolar search
  std::vector<Vector<2> > vImplaneCorners; // Corner points un-projected into z=1-plane coordinates
};

// The actual KeyFrame struct. The map contains of a bunch of these. However, the tracker uses this
// struct as well: every incoming frame is turned into a keyframe before tracking; most of these 
// are then simply discarded, but sometimes they're then just added to the map.
struct KeyFrame
{
  inline KeyFrame()
  {
    pSBI = NULL;
    bFixed = false; dSceneDepthMean = 1.0; dSceneDepthSigma = 1.0;
  }
  KeyFrame(const KeyFrame& rhs);
  KeyFrame& operator=(const KeyFrame& rhs);
  ~KeyFrame();
  SE3<> se3CfromW;    // The coordinate frame of this key-frame as a Camera-From-World transformation
  bool bFixed;      // Is the coordinate frame of this keyframe fixed? (only true for first KF!)
  Level aLevels[LEVELS];  // Images, corners, etc lives in this array of pyramid levels
  std::map<MapPoint*, Measurement> mMeasurements;           // All the measurements associated with the keyframe
  
  void MakeKeyFrame_Lite(CVD::BasicImage<CVD::byte> &im
#ifdef IPTAM_MOBILE_PAPER
                         , bool detectFeatures=true
#endif
                         );   // This takes an image and calculates pyramid levels etc to fill the
                                                            // keyframe data structures with everything that's needed by the tracker..
  void MakeKeyFrame_Rest();                                 // ... while this calculates the rest of the data which the mapmaker needs.
  
#ifdef IPTAM_MOBILE_PAPER
  double captureTimestamp = 0;
  bool hasCaptureTimestamp = false;
  bool mobileBlurFallback = false;
  bool mobileCornersReady = false;
#endif
  double dSceneDepthMean;      // Hacky hueristics to improve epipolar search.
  double dSceneDepthSigma;
  
  SmallBlurryImage *pSBI; // The relocaliser uses this
};

typedef std::map<MapPoint*, Measurement>::iterator meas_it;  // For convenience, and to work around an emacs paren-matching bug


#endif

