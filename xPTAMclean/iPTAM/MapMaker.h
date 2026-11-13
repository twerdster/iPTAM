// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

//
// This header declares the MapMaker class
// MapMaker makes and maintains the Map struct
// Starting with stereo initialisation from a bunch of matches
// over keyframe insertion, continual bundle adjustment and 
// data-association refinement.
// MapMaker runs in its own thread, although some functions
// (notably stereo init) are called by the tracker and run in the 
// tracker's thread.

#ifndef __MAPMAKER_H
#define __MAPMAKER_H
#include "image.h"
#include "byte.h"
#include "thread.h"

#include "Map.h"
#include "KeyFrame.h"
#include "ATANCamera.h"
#include <queue>
#include <random>
#ifdef IPTAM_MOBILE_PAPER
#include <array>
#include <deque>
#include <cstdint>
#include <limits>
struct MappingDiagnostics {
  enum Rejection { BackwardDepth, BehindDepth, DegenerateRay, OutsideLine,
    BadTemplate, NoMatch, Subpixel, Cheirality, RejectionCount };
  enum Stage { InsertKeyframe, RecentBundle, Refind, FullBundle, Retry, Cull, Partial, PartialPromotion, StageCount };
  std::uint64_t epipolarAttempts=0, epipolarAccepted=0, invalidDepthIntervals=0;
  std::uint64_t repairedDepthIntervals=0, budgetStops=0;
  std::array<std::uint64_t,RejectionCount> rejected{};
  std::array<std::uint64_t,StageCount> stageCalls{};
  std::array<double,StageCount> stageMs{}, maxStageMs{};
  std::uint64_t culledMeasurements=0, culledKeyframes=0, partialSeeded=0, partialPromoted=0;
  std::uint64_t partialUpdates=0, partialRejected=0, idleStages=0, idleOverruns=0;
  std::uint64_t observationBridgeRetentions=0, observationSupportRetentions=0, fixedLowSupportViews=0, bundleRSCorrections=0;
  std::uint64_t deferredPartialQueued=0, deferredPartialCompleted=0, deferredPartialCancelled=0;
  double idleMs=0, lastDepthMean=0, lastDepthSigma=0, lastDepthStart=0, lastDepthEnd=0;
};
#endif


// Each MapPoint has an associated MapMakerData class
// Where the mapmaker can store extra information
 
struct MapMakerData
{
  std::set<KeyFrame*> sMeasurementKFs;   // Which keyframes has this map point got measurements in?
  std::set<KeyFrame*> sNeverRetryKFs;    // Which keyframes have measurements failed enough so I should never retry?
  inline int GoodMeasCount()            
  {  return sMeasurementKFs.size(); }
};

// MapMaker dervives from CVD::Thread, so everything in void run() is its own thread.
class MapMaker : protected CVD::Thread
{
public:
  MapMaker(Map &m, const ATANCamera &cam, bool threaded = false);
  unsigned int NumericalFailureCount() const { return mnNumericalFailures; }
  unsigned int RetainedSolveCount() const { return mnRetainedSolveStops; }
  const std::string& LastNumericalFailure() const { return msLastNumericalFailure; }
  void ProcessPending(); // Call once per frame when using serial mode.
#ifdef IPTAM_MOBILE_PAPER
  const MappingDiagnostics& Diagnostics() const { return mDiagnostics; }
  static double SteadyTimeSeconds();
  unsigned int ProcessIdleWork(double deadlineSeconds,double budgetMs);
  bool ResourceLimitReached() const { return mbMobileResourceLimit; }
  const std::string& ResourceLimitReason() const { return msMobileResourceReason; }
  void CullMobileMap();
  void ObservePartialPoints(KeyFrame& current);
  void NotifyMobileMeasurementsChanged(KeyFrame&) { mbBundleConverged_Recent=mbBundleConverged_Full=false; }
  std::size_t PartialPointCount() const;
  std::size_t PendingPartialPromotionCount() const { return mPendingPartialPromotions.size(); }
#endif
  ~MapMaker();
  
  // Make a map from scratch. Called by the tracker.
  bool InitFromStereo(KeyFrame &kFirst, KeyFrame &kSecond, 
		      std::vector<std::pair<CVD::ImageRef, CVD::ImageRef> > &vMatches,
		      SE3<> &se3CameraPos
#ifdef IPTAM_MOBILE_PAPER
                      , const SE3<>* selectedSecondFromFirst=nullptr
#endif
                      );

  bool InitFromStereo_OLD(KeyFrame &kFirst, KeyFrame &kSecond,  // EXPERIMENTAL HACK
		      std::vector<std::pair<CVD::ImageRef, CVD::ImageRef> > &vMatches,
		      SE3<> &se3CameraPos);
  
  
  void AddKeyFrame(KeyFrame &k);   // Add a key-frame to the map. Called by the tracker.
  void RequestReset();   // Request that the we reset. Called by the tracker.
  bool ResetDone();      // Returns true if the has been done.
  int  QueueSize() { return mvpKeyFrameQueue.size() ;} // How many KFs in the queue waiting to be added?
  bool NeedNewKeyFrame(KeyFrame &kCurrent);            // Is it a good camera pose to add another KeyFrame?
  bool IsDistanceToNearestKeyFrameExcessive(KeyFrame &kCurrent);  // Is the camera far away from the nearest KeyFrame (i.e. maybe lost?)
  
protected:
  
  std::mt19937 mRng{42}; // Fixed seed: reproducible replay and reset.
  unsigned int mnNumericalFailures=0;
  unsigned int mnRetainedSolveStops=0;
  std::string msLastNumericalFailure;
  CVD::Image<Vector<2>> mimUnprojectCache;
  bool mbThreaded;
#ifdef IPTAM_MOBILE_PAPER
  MappingDiagnostics mDiagnostics;
  bool mbMobileResourceLimit=false;
  std::string msMobileResourceReason;
  struct PartialLandmark {
    std::uint64_t id=0;
    KeyFrame* source=nullptr;
    Vector<2> sourceRoot=Zeros;
    Vector<3> sourceRay=Zeros;
    double inverseDepth=0,variance=std::numeric_limits<double>::infinity();
    unsigned int observations=0,misses=0,age=0;
    int sourceLevel=0;
    double lastUpdateTime=std::numeric_limits<double>::quiet_NaN();
  };
  std::vector<PartialLandmark> mPartialLandmarks;
  std::size_t mPartialObservationCursor=0;
  struct PendingPartialPromotion {
    KeyFrame* target=nullptr; // Owned by active map; cancelled before KF retirement/reset.
    std::vector<std::uint64_t> candidates;
    std::size_t next=0;
  };
  std::deque<PendingPartialPromotion> mPendingPartialPromotions;
  std::uint64_t mNextPartialId=1;
  void QueuePartialPromotion(KeyFrame& target);
  void ProcessPendingPartialPromotion();
  std::map<KeyFrame*,std::pair<KeyFrame*,SE3<>>> mRetainedPoseAnchors;
  void SeedPartialPoints(KeyFrame& source);
  std::size_t ObserveAndPromotePartialPoints(KeyFrame& current,bool promote,std::uint64_t onlyId=0);
  void RefreshRetainedPoses();
  void RefreshMobileRollingShutter(KeyFrame& frame);
  std::size_t ActivePointLimit() const;
  void CheckMobileResources();
#endif
  void ProcessOneIteration();
  Map &mMap;               // The map
  ATANCamera mCamera;      // Same as the tracker's camera: N.B. not a reference variable!
  virtual void run();      // The MapMaker thread code lives here

  // Functions for starting the map from scratch:
  SE3<> CalcPlaneAligner();
  void ApplyGlobalTransformationToMap(SE3<> se3NewFromOld);
  void ApplyGlobalScaleToMap(double dScale);
  
  // Map expansion functions:
  void AddKeyFrameFromTopOfQueue();  
  void ThinCandidates(KeyFrame &k, int nLevel);
  void AddSomeMapPoints(int nLevel);
  bool AddPointEpipolar(KeyFrame &kSrc, KeyFrame &kTarget, int nLevel, int nCandidate);
  // Returns point in ref frame B
  Vector<3> ReprojectPoint(SE3<> se3AfromB, const Vector<2> &v2A, const Vector<2> &v2B);
  
  // Bundle adjustment functions:
  void BundleAdjust(std::set<KeyFrame*>, std::set<KeyFrame*>, std::set<MapPoint*>, bool);
  void BundleAdjustAll();
  void BundleAdjustRecent();

  // Data association functions:
  int ReFindInSingleKeyFrame(KeyFrame &k);
  void ReFindFromFailureQueue();
  void ReFindNewlyMade();
  void ReFindAll();
  bool ReFind_Common(KeyFrame &k, MapPoint &p);
  void SubPixelRefineMatches(KeyFrame &k, int nLevel);
  
  // General Maintenance/Utility:
  void Reset();
  void HandleBadPoints();
  double DistToNearestKeyFrame(KeyFrame &kCurrent);
  double KeyFrameLinearDist(KeyFrame &k1, KeyFrame &k2);
  KeyFrame* ClosestKeyFrame(KeyFrame &k);
  std::vector<KeyFrame*> NClosestKeyFrames(KeyFrame &k, unsigned int N);
  bool RefreshSceneDepth(KeyFrame *pKF);
  

  // GUI Interface:
  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
  struct Command {std::string sCommand; std::string sParams; };
  std::vector<Command> mvQueuedCommands;
  

  // Member variables:
  std::vector<KeyFrame*> mvpKeyFrameQueue;  // Queue of keyframes from the tracker waiting to be processed
  std::vector<std::pair<KeyFrame*, MapPoint*> > mvFailureQueue; // Queue of failed observations to re-find
  std::queue<MapPoint*> mqNewQueue;   // Queue of newly-made map points to re-find in other KeyFrames
  
  double mdWiggleScale;  // Metric distance between the first two KeyFrames (copied from GVar)
                         // This sets the scale of the map
  double mgvdWiggleScale;   // GVar for above
  double mdWiggleScaleDepthNormalized;  // The above normalized against scene depth, 
                                        // this controls keyframe separation
  
  bool mbBundleConverged_Full;    // Has global bundle adjustment converged?
  bool mbBundleConverged_Recent;  // Has local bundle adjustment converged?
  
  // Thread interaction signalling stuff
  bool mbResetRequested;   // A reset has been requested
  bool mbResetDone;        // The reset was done.
  bool mbBundleAbortRequested;      // We should stop bundle adjustment
  bool mbBundleRunning;             // Bundle adjustment is running
  bool mbBundleRunningIsRecent;     //    ... and it's a local bundle adjustment.

  
};

#endif


















